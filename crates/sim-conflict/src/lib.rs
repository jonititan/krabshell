//! Probabilistic conflict detection and resolution advisories for aircraft,
//! on A5 cells.
//!
//! Given the states of the aircraft in an area, [`assess`]:
//!
//! 1. **Predicts** each one over the next few steps ([`predict`]): a cloud
//!    of particles flown under its energy model ([`performance`]: BADA's Total
//!    Energy Model for fixed-wing aircraft, an excess-power envelope for
//!    multirotors), counted into A5 cells and altitude layers, so each step
//!    is a list of cells it could be in, each with a probability.
//! 2. **Detects** conflicts ([`conflict`]): at each step, the probabilities
//!    every aircraft puts into a cell are summed, and a cell where two or more
//!    aircraft reach the threshold needs resolving.
//! 3. **Resolves** them ([`resolve`]): tries climbing, descending, levelling
//!    off, turning and changing speed for the aircraft involved, predicts
//!    again, and advises what removes the most risk, aircraft by aircraft.
//!
//! Altitude matters as much as position, so a "cell" here is an A5 cell and
//! an altitude layer ([`Layers`]): by default the airspace's own, 1 000 ft
//! deep below 29 000 ft and 2 000 ft deep from there up, placed so every
//! flight level sits inside its layer rather than on a boundary, and aircraft
//! at properly separated levels sit in different layers. Each aircraft carries a protected zone
//! (`buffer_rings` of neighbouring cells and `vertical_buffer_m`) so two
//! aircraft either side of a cell edge are not missed.
//!
//! Everything is deterministic: the same states and configuration give the
//! same advisories, bit for bit, on every machine.
//!
//! This is a planning aid on a grid, not a certified collision avoidance
//! system: its probabilities are only as good as the noise model and the
//! performance figures it is given.

pub mod atmosphere;
pub mod conflict;
pub mod performance;
pub mod predict;
pub mod resolve;

use std::collections::BTreeMap;

use serde::{Deserialize, Serialize};

pub use conflict::{
    detect, encounters, heat, hotspots, two_or_more, Conflict, Encounter, HeatCell, Hotspot,
};
pub use performance::{FixedWing, Multirotor, Performance};
pub use predict::{
    AircraftState, Calm, FieldWind, Manoeuvre, Prediction, Predictor, Targets, WindSource,
};
pub use resolve::{assess, Advisory, Assessment};

/// The performance models aircraft refer to by name.
pub type Fleet = BTreeMap<String, Performance>;

#[derive(Debug, Clone, PartialEq, thiserror::Error)]
pub enum Error {
    #[error("aircraft {0} has a state that is not finite or out of range")]
    BadState(u64),
    #[error("aircraft {0} appears twice")]
    DuplicateId(u64),
    #[error("no performance model called {0:?} in the fleet")]
    UnknownPerformance(String),
    #[error("configuration: {0}")]
    BadConfig(String),
}

/// One foot, m.
pub const FT: f64 = 0.3048;

/// How the airspace is split into altitude layers: bands, each from its
/// `base_m` up to the next band's, cut into layers `depth_m` deep. Layers are
/// numbered from 0 at the first band's base, upward through every band;
/// below it they continue the first band's depth downward.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Layers {
    pub bands: Vec<Band>,
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Band {
    pub base_m: f64,
    pub depth_m: f64,
}

impl Layers {
    /// 1 000 ft layers below 29 000 ft and 2 000 ft layers from there up,
    /// with every flight level inside a layer rather than on a boundary:
    ///
    /// - below FL290 each layer is its flight level ±500 ft, so layer *n* is
    ///   FL*n*0 (FL100 is layer 10, 9 500 to 10 500 ft);
    /// - from FL290 each runs from 500 ft below FL290, FL310, FL330... to
    ///   1 500 ft above (FL330's is 32 500 to 34 500 ft). They cannot be
    ///   centred, as FL290's would then overlap FL280's, and the two are
    ///   properly separated.
    ///
    /// The bands meet at 28 500 ft. With the boundaries on the levels
    /// themselves, two aircraft at adjacent levels would straddle the same
    /// boundary and always look in conflict; this way two at properly
    /// separated levels (1 000 ft apart below FL290, 2 000 ft from it) are in
    /// different layers even with some height error.
    pub fn standard() -> Layers {
        Layers {
            bands: vec![
                Band {
                    base_m: -500.0 * FT,
                    depth_m: 1_000.0 * FT,
                },
                Band {
                    base_m: 28_500.0 * FT,
                    depth_m: 2_000.0 * FT,
                },
            ],
        }
    }

    /// Layers of one depth everywhere.
    pub fn uniform(depth_m: f64) -> Layers {
        Layers {
            bands: vec![Band {
                base_m: 0.0,
                depth_m,
            }],
        }
    }

    /// Where each band starts counting: the layer index at its base.
    fn starts(&self) -> impl Iterator<Item = (Band, f64, i32)> + '_ {
        let mut first = 0i32;
        self.bands.iter().enumerate().map(move |(i, b)| {
            let top = self.bands.get(i + 1).map_or(f64::INFINITY, |n| n.base_m);
            let at = first;
            if top.is_finite() {
                first += libm::ceil((top - b.base_m) / b.depth_m) as i32;
            }
            (*b, top, at)
        })
    }

    /// The layer an altitude above mean sea level is in.
    pub fn index(&self, alt_m: f64) -> i32 {
        let mut last = (self.bands[0], 0);
        for (band, top, first) in self.starts() {
            last = (band, first);
            if alt_m < top {
                break;
            }
        }
        let (band, first) = last;
        first + libm::floor((alt_m - band.base_m) / band.depth_m) as i32
    }

    /// A layer's bottom and top, m above mean sea level.
    pub fn bounds(&self, layer: i32) -> (f64, f64) {
        let mut last = (self.bands[0], 0);
        for (band, top, first) in self.starts() {
            last = (band, first);
            let count = if top.is_finite() {
                libm::ceil((top - band.base_m) / band.depth_m) as i32
            } else {
                i32::MAX
            };
            if layer < first.saturating_add(count) {
                break;
            }
        }
        let (band, first) = last;
        let bottom = band.base_m + f64::from(layer - first) * band.depth_m;
        (bottom, bottom + band.depth_m)
    }

    fn check(&self) -> Result<(), Error> {
        let ok = !self.bands.is_empty()
            && self
                .bands
                .iter()
                .all(|b| b.depth_m > 0.0 && b.base_m.is_finite())
            && self.bands.windows(2).all(|w| w[0].base_m < w[1].base_m);
        if ok {
            Ok(())
        } else {
            Err(Error::BadConfig(
                "layers: bands with ascending bases, each with a positive depth".into(),
            ))
        }
    }
}

/// How uncertain a reported state is, and how much an aircraft wanders.
/// Standard deviations.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Noise {
    pub pos_m: f64,
    pub alt_m: f64,
    pub heading_deg: f64,
    /// Of airspeed.
    pub speed_frac: f64,
    pub vs_ms: f64,
    /// Turn-rate random walk, degrees per second per √second.
    pub turn_walk_dps: f64,
    /// Airspeed random walk in level flight, share per √second.
    pub speed_walk_frac: f64,
    /// Error in the wind, each way, m/s.
    pub wind_ms: f64,
}

/// How to predict, what counts as a conflict, and what to advise.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Config {
    /// A5 resolution of the cells.
    pub resolution: i32,
    /// The altitude layers.
    pub layers: Layers,
    /// Rings of neighbouring cells in an aircraft's protected zone.
    pub buffer_rings: usize,
    /// Height above and below an aircraft in its protected zone, m.
    pub vertical_buffer_m: f64,
    /// Seconds between predicted steps, and how many.
    pub step_s: f64,
    pub steps: usize,
    /// Integration step, s.
    pub substep_s: f64,
    pub particles: usize,
    /// Summed probability in a cell that is a conflict. At least 1.
    pub threshold: f64,
    pub seed: u64,
    pub noise: Noise,
    /// Heading change a turn advisory asks for, degrees.
    pub turn_deg: f64,
    /// Speed change a speed advisory asks for, as a share.
    pub speed_change: f64,
    /// Lowest altitude a descend advisory is given at, m.
    pub min_alt_m: f64,
    /// Conflicts this far off count half as much as one now, s.
    pub urgency_s: f64,
    /// Most advisories one assessment gives.
    pub max_advisories: usize,
}

impl Config {
    /// Small drones at low level: ~180 m cells (resolution 15), a minute
    /// ahead in 5 s steps, and 30 m layers: drones keep to the bottom few
    /// hundred feet, so the standard 1 000 ft layers would put them all in
    /// one. Set `layers` to [`Layers::standard`] to use the airspace's.
    pub fn drones() -> Config {
        Config {
            resolution: 15,
            layers: Layers::uniform(30.0),
            buffer_rings: 1,
            vertical_buffer_m: 15.0,
            step_s: 5.0,
            steps: 12,
            substep_s: 1.0,
            particles: 200,
            threshold: 1.0,
            seed: 0x5eed_c0ff_1c70,
            noise: Noise {
                pos_m: 5.0,
                alt_m: 2.0,
                heading_deg: 3.0,
                speed_frac: 0.05,
                vs_ms: 0.3,
                turn_walk_dps: 0.05,
                speed_walk_frac: 0.01,
                wind_ms: 0.5,
            },
            turn_deg: 45.0,
            speed_change: 0.3,
            min_alt_m: 20.0,
            urgency_s: 30.0,
            max_advisories: 16,
        }
    }

    /// Airliners: ~3 km cells (resolution 11), the standard layers (1 000 ft
    /// below 29 000 ft, 2 000 ft above), five minutes ahead in 15 s steps.
    pub fn airliners() -> Config {
        Config {
            resolution: 11,
            layers: Layers::standard(),
            buffer_rings: 1,
            // Less than half the 1 000 ft separation: aircraft at adjacent
            // levels stay in their own layers.
            vertical_buffer_m: 250.0 * FT,
            step_s: 15.0,
            steps: 20,
            substep_s: 2.5,
            particles: 200,
            threshold: 1.0,
            seed: 0x5eed_c0ff_1c70,
            noise: Noise {
                pos_m: 100.0,
                alt_m: 15.0,
                heading_deg: 1.0,
                speed_frac: 0.02,
                vs_ms: 1.0,
                turn_walk_dps: 0.01,
                speed_walk_frac: 0.002,
                wind_ms: 2.0,
            },
            turn_deg: 30.0,
            speed_change: 0.1,
            min_alt_m: 300.0,
            urgency_s: 120.0,
            max_advisories: 16,
        }
    }

    pub(crate) fn check(&self) -> Result<(), Error> {
        let bad = |m: &str| Err(Error::BadConfig(m.into()));
        if !(0..=sim_core::MAX_RESOLUTION).contains(&self.resolution) {
            return bad("resolution out of range");
        }
        if self.threshold.is_nan() || self.threshold < 1.0 {
            return bad("threshold must be at least 1, or one aircraft would conflict with itself");
        }
        self.layers.check()?;
        if !(self.step_s > 0.0 && self.substep_s > 0.0) {
            return bad("step_s and substep_s must be positive");
        }
        if self.steps == 0 || self.particles == 0 {
            return bad("steps and particles must be at least 1");
        }
        if self.vertical_buffer_m.is_nan() || self.vertical_buffer_m < 0.0 {
            return bad("vertical_buffer_m must be 0 or more");
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn standard_layers_are_1000_ft_to_fl290_and_2000_ft_above_off_the_levels() {
        let l = Layers::standard();
        let fl = |f: f64| f * 100.0 * FT;
        // Below FL290: layer n is FLn0, ±500 ft.
        assert_eq!(l.index(0.0), 0);
        assert_eq!(l.index(fl(100.0)), 10);
        assert_eq!(l.index(fl(104.9)), 10);
        assert_eq!(l.index(fl(105.1)), 11);
        assert_eq!(l.index(fl(280.0)), 28);
        // From FL290: 2 000 ft layers on FL290, FL310, FL330...
        assert_eq!(l.index(fl(285.0)), 29);
        assert_eq!(l.index(fl(290.0)), 29);
        assert_eq!(l.index(fl(300.0)), 29);
        assert_eq!(l.index(fl(305.1)), 30);
        assert_eq!(l.index(fl(310.0)), 30);
        assert_eq!(l.index(fl(330.0)), 31);
        assert_eq!(l.index(fl(410.0)), 35);
        let ft = |(a, b): (f64, f64)| ((a / FT).round(), (b / FT).round());
        assert_eq!(ft(l.bounds(10)), (9_500.0, 10_500.0));
        assert_eq!(ft(l.bounds(28)), (27_500.0, 28_500.0));
        assert_eq!(ft(l.bounds(29)), (28_500.0, 30_500.0));
        assert_eq!(ft(l.bounds(31)), (32_500.0, 34_500.0));
        for alt in [-100.0, 0.0, 5_000.0, 8_686.0, 8_687.0, 12_500.0] {
            let (lo, hi) = l.bounds(l.index(alt));
            assert!(lo <= alt && alt < hi, "{alt}");
        }
    }

    #[test]
    fn a_threshold_below_one_or_bad_layers_are_refused() {
        let mut c = Config::airliners();
        c.threshold = 0.9;
        assert!(c.check().is_err());
        let mut c = Config::airliners();
        c.layers.bands[1].base_m = -1_000.0;
        assert!(c.check().is_err());
        assert!(Config::drones().check().is_ok());
    }
}
