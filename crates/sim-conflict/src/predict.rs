//! Where an aircraft could be over the next few steps, as probabilities over
//! A5 cells and altitude layers.
//!
//! The reported state is not exact and the aircraft will not fly exactly
//! straight, so the prediction flies a cloud of particles: each starts from
//! the state with its own small errors in position, heading, speed and
//! vertical rate, and wanders a little as it goes (a random walk in turn rate
//! and speed). Each particle flies the energy model ([`Performance`]): a
//! climbing aircraft keeps climbing at the rate the model gives for its
//! altitude, scaled to the rate it was seen climbing, on its speed schedule;
//! a level one holds its speed; a turning one keeps turning.
//!
//! At each output step every particle is put in its A5 cell and altitude
//! layer, plus a protected zone round it (`buffer_rings` of neighbouring
//! cells, `vertical_buffer_m` up and down) so two aircraft either side of a
//! cell edge still meet. A cell's probability is the share of particles whose
//! zone covers it. An aircraft therefore puts at most 1 into any cell.
//!
//! Particles are drawn from a generator seeded by the configuration's seed
//! and the aircraft's id alone, so the same aircraft gets the same cloud
//! under every manoeuvre tried for it: the comparison between manoeuvres is
//! not swamped by sampling noise, and a prediction repeats exactly.

use std::collections::{BTreeMap, HashMap};

use serde::{Deserialize, Serialize};
use sim_core::rng::Rng;
use sim_core::GeoCell;

use crate::atmosphere::{isa, tas_to_cas};
use crate::performance::{Performance, Thrust};
use crate::{Config, Error};

const EARTH_RADIUS_M: f64 = 6_371_000.0;

/// An aircraft as reported.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct AircraftState {
    pub id: u64,
    pub lat: f64,
    pub lon: f64,
    /// Above mean sea level, m.
    pub alt_m: f64,
    /// True airspeed, m/s. Ground speed will do in still air.
    pub tas_ms: f64,
    /// Where the air velocity points, degrees clockwise from north.
    pub heading_deg: f64,
    /// Vertical speed, m/s, positive up.
    pub vs_ms: f64,
    /// Rate of turn, degrees per second, positive to the right.
    #[serde(default)]
    pub turn_rate_dps: f64,
    /// The performance's reference mass if absent.
    #[serde(default)]
    pub mass_kg: Option<f64>,
    /// Which entry of the [`crate::Fleet`] it flies like.
    pub performance: String,
}

impl AircraftState {
    pub(crate) fn check(&self) -> Result<(), Error> {
        let finite = [
            self.lat,
            self.lon,
            self.alt_m,
            self.tas_ms,
            self.heading_deg,
            self.vs_ms,
            self.turn_rate_dps,
        ]
        .iter()
        .all(|v| v.is_finite());
        if !finite || !(-90.0..=90.0).contains(&self.lat) || !(-180.0..=180.0).contains(&self.lon) {
            return Err(Error::BadState(self.id));
        }
        if self.tas_ms < 0.0 || self.mass_kg.is_some_and(|m| m.is_nan() || m <= 0.0) {
            return Err(Error::BadState(self.id));
        }
        Ok(())
    }
}

/// What an aircraft is asked to do.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[serde(tag = "action", rename_all = "snake_case")]
pub enum Manoeuvre {
    /// Carry on as it is.
    Maintain,
    Climb,
    Descend,
    /// Stop climbing or descending.
    LevelOff,
    TurnLeft,
    TurnRight,
    SpeedUp,
    SlowDown,
}

impl Manoeuvre {
    pub const CANDIDATES: [Manoeuvre; 7] = [
        Manoeuvre::Climb,
        Manoeuvre::Descend,
        Manoeuvre::LevelOff,
        Manoeuvre::TurnLeft,
        Manoeuvre::TurnRight,
        Manoeuvre::SpeedUp,
        Manoeuvre::SlowDown,
    ];

    /// How much less welcome than a vertical manoeuvre, which is what
    /// collision avoidance prefers: it changes least of the flight.
    pub fn cost(self) -> f64 {
        match self {
            Manoeuvre::Maintain => 0.0,
            Manoeuvre::LevelOff => 0.03,
            Manoeuvre::Climb | Manoeuvre::Descend => 0.05,
            Manoeuvre::TurnLeft | Manoeuvre::TurnRight => 0.10,
            Manoeuvre::SpeedUp | Manoeuvre::SlowDown => 0.15,
        }
    }
}

/// A manoeuvre made concrete for one aircraft: the targets it flies to.
#[derive(Debug, Clone, Copy, PartialEq, Default, Serialize, Deserialize)]
pub struct Targets {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub vs_ms: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub heading_deg: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub tas_ms: Option<f64>,
}

/// The targets a manoeuvre sets for an aircraft, or `None` if it cannot fly
/// it (a climb at its ceiling, a descent too low, a level-off when level).
pub fn targets(
    state: &AircraftState,
    perf: &Performance,
    manoeuvre: Manoeuvre,
    cfg: &Config,
) -> Option<Targets> {
    let mass = state.mass_kg.unwrap_or_else(|| perf.default_mass_kg());
    let none = Targets::default();
    match manoeuvre {
        Manoeuvre::Maintain => Some(none),
        Manoeuvre::Climb => {
            let max = perf.max_climb_ms(state.alt_m, state.tas_ms, mass);
            (max > 0.3 && state.vs_ms < perf.advisory_vs_ms()).then(|| Targets {
                vs_ms: Some(perf.advisory_vs_ms().min(max)),
                ..none
            })
        }
        Manoeuvre::Descend => {
            let rate =
                perf.advisory_vs_ms()
                    .min(perf.max_descent_ms(state.alt_m, state.tas_ms, mass));
            (state.alt_m > cfg.min_alt_m && state.vs_ms > -rate).then(|| Targets {
                vs_ms: Some(-rate),
                ..none
            })
        }
        Manoeuvre::LevelOff => (state.vs_ms.abs() > 0.5).then_some(Targets {
            vs_ms: Some(0.0),
            ..none
        }),
        Manoeuvre::TurnLeft | Manoeuvre::TurnRight => {
            let sign = if manoeuvre == Manoeuvre::TurnLeft {
                -1.0
            } else {
                1.0
            };
            Some(Targets {
                heading_deg: Some((state.heading_deg + sign * cfg.turn_deg).rem_euclid(360.0)),
                ..none
            })
        }
        Manoeuvre::SpeedUp | Manoeuvre::SlowDown => {
            let (low, high) = perf.speed_envelope(state.alt_m);
            let factor = if manoeuvre == Manoeuvre::SpeedUp {
                1.0 + cfg.speed_change
            } else {
                1.0 - cfg.speed_change
            };
            let tas = (state.tas_ms * factor).clamp(low, high);
            ((tas - state.tas_ms).abs() > 0.5).then_some(Targets {
                tas_ms: Some(tas),
                ..none
            })
        }
    }
}

/// A cell and an altitude layer, numbered by [`crate::Layers`].
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
pub struct Key {
    pub cell: GeoCell,
    pub layer: i32,
}

/// Where an aircraft could be.
#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct Prediction {
    pub id: u64,
    pub manoeuvre: Manoeuvre,
    pub targets: Targets,
    /// Seconds from now of each step.
    pub times_s: Vec<f64>,
    /// For each step, the probability its protected zone covers each key.
    pub cells: Vec<BTreeMap<Key, f64>>,
    /// The particles' mean position at each step: `[lat, lon, alt_m]`.
    pub mean: Vec<[f64; 3]>,
}

/// The wind a prediction flies in.
pub trait WindSource {
    /// Toward the east and north, m/s, at a place and height, `t_s` seconds
    /// from now.
    fn wind(&self, lat: f64, lon: f64, alt_m: f64, t_s: f64) -> (f64, f64);
}

/// No wind.
pub struct Calm;

impl WindSource for Calm {
    fn wind(&self, _: f64, _: f64, _: f64, _: f64) -> (f64, f64) {
        (0.0, 0.0)
    }
}

/// A krabshell wind field, looked up as the drones look it up: through the
/// cell at `cell_resolution` (the world's) a point is in. Heights are taken
/// as above the ground.
pub struct FieldWind<'a> {
    pub field: &'a sim_core::WindField,
    /// UTC Unix seconds now.
    pub utc_now: f64,
    pub cell_resolution: i32,
}

impl WindSource for FieldWind<'_> {
    fn wind(&self, lat: f64, lon: f64, alt_m: f64, t_s: f64) -> (f64, f64) {
        GeoCell::from_lonlat(lon, lat, self.cell_resolution)
            .ok()
            .and_then(|cell| self.field.wind_at(cell, alt_m, self.utc_now + t_s))
            .map_or((0.0, 0.0), |w| (w.east_ms, w.north_ms))
    }
}

/// Predicts, keeping the cell neighbourhoods it has worked out.
pub struct Predictor<'a> {
    cfg: &'a Config,
    wind: &'a dyn WindSource,
    zones: HashMap<GeoCell, Vec<GeoCell>>,
}

#[derive(Debug, Clone, Copy)]
struct Particle {
    lat: f64,
    lon: f64,
    alt: f64,
    tas: f64,
    hdg: f64,
    vs: f64,
    turn: f64,
    /// Its starting airspeed over the reported one: how far off the
    /// schedule it flies.
    ratio: f64,
}

#[derive(Debug, Clone, Copy, PartialEq)]
enum Phase {
    Climb,
    Descent,
    Level,
}

fn normal(rng: &mut Rng) -> f64 {
    let u1 = rng.unit().max(1e-12);
    let u2 = rng.unit();
    libm::sqrt(-2.0 * libm::log(u1)) * libm::cos(std::f64::consts::TAU * u2)
}

fn displace(lat: f64, lon: f64, east_m: f64, north_m: f64) -> (f64, f64) {
    let dlat = (north_m / EARTH_RADIUS_M).to_degrees();
    let dlon = (east_m / (EARTH_RADIUS_M * libm::cos(lat.to_radians()).max(1e-6))).to_degrees();
    (lat + dlat, lon + dlon)
}

/// Move `value` toward `target` by at most `step`.
fn approach(value: f64, target: f64, step: f64) -> f64 {
    value + (target - value).clamp(-step, step)
}

/// Signed shortest turn from `from` to `to`, degrees.
fn turn_to(from: f64, to: f64) -> f64 {
    (to - from + 540.0).rem_euclid(360.0) - 180.0
}

impl<'a> Predictor<'a> {
    pub fn new(cfg: &'a Config, wind: &'a dyn WindSource) -> Predictor<'a> {
        Predictor {
            cfg,
            wind,
            zones: HashMap::new(),
        }
    }

    /// The cells a protected zone centred in `cell` covers.
    fn zone(&mut self, cell: GeoCell) -> &[GeoCell] {
        let rings = self.cfg.buffer_rings;
        self.zones.entry(cell).or_insert_with(|| {
            if rings == 0 {
                vec![cell]
            } else {
                cell.grid_disk(rings).unwrap_or_else(|_| vec![cell])
            }
        })
    }

    /// Where `state` could be over the configured steps, flying `manoeuvre`.
    pub fn predict(
        &mut self,
        state: &AircraftState,
        perf: &Performance,
        manoeuvre: Manoeuvre,
    ) -> Prediction {
        let cfg = self.cfg;
        let targets = targets(state, perf, manoeuvre, cfg).unwrap_or_default();
        let mass = state.mass_kg.unwrap_or_else(|| perf.default_mass_kg());
        let n = cfg.particles.max(1);
        let noise = &cfg.noise;
        let mut rng = Rng::new(cfg.seed ^ state.id.wrapping_mul(0x9e37_79b9_7f4a_7c15));

        // What it is doing, and how hard compared with what the energy model
        // says it could.
        let phase = if state.vs_ms > 0.5 {
            Phase::Climb
        } else if state.vs_ms < -0.5 {
            Phase::Descent
        } else {
            Phase::Level
        };
        let calibrate = |thrust: Thrust| {
            let model = perf.energy_rate_ms(state.alt_m, state.tas_ms, mass, thrust);
            if model.abs() < 0.1 {
                1.0
            } else {
                (state.vs_ms / model).clamp(0.2, 1.5)
            }
        };
        let scale = match phase {
            Phase::Climb => calibrate(Thrust::Climb),
            Phase::Descent => calibrate(Thrust::Idle),
            Phase::Level => 1.0,
        };
        // The speed schedule it is on: this CAS, or this Mach.
        let (cas0, mach0) = match perf {
            Performance::FixedWing(_) => (
                tas_to_cas(state.tas_ms, state.alt_m),
                state.tas_ms / isa(state.alt_m).sound_ms,
            ),
            Performance::Multirotor(_) => (state.tas_ms, 0.0),
        };

        let mut particles: Vec<Particle> = (0..n)
            .map(|_| {
                let (lat, lon) = displace(
                    state.lat,
                    state.lon,
                    normal(&mut rng) * noise.pos_m,
                    normal(&mut rng) * noise.pos_m,
                );
                let ratio = (1.0 + normal(&mut rng) * noise.speed_frac).max(0.0);
                Particle {
                    lat,
                    lon,
                    alt: state.alt_m + normal(&mut rng) * noise.alt_m,
                    tas: state.tas_ms * ratio,
                    hdg: state.heading_deg + normal(&mut rng) * noise.heading_deg,
                    vs: state.vs_ms + normal(&mut rng) * noise.vs_ms,
                    turn: state.turn_rate_dps,
                    ratio,
                }
            })
            .collect();

        let substeps = (cfg.step_s / cfg.substep_s).ceil().max(1.0) as usize;
        let h = cfg.step_s / substeps as f64;
        let delay = perf.response_delay_s();
        let turn_rate = perf.turn_rate_dps(state.tas_ms);
        let v_accel = perf.vertical_accel_ms2();
        let s_accel = perf.speed_accel_ms2();
        let ceiling = perf.ceiling_m();

        let mut out = Prediction {
            id: state.id,
            manoeuvre,
            targets,
            times_s: Vec::with_capacity(cfg.steps),
            cells: Vec::with_capacity(cfg.steps),
            mean: Vec::with_capacity(cfg.steps),
        };
        let mut t = 0.0;
        for _ in 0..cfg.steps {
            for _ in 0..substeps {
                let following = t >= delay;
                for p in particles.iter_mut() {
                    // Draw every number whatever happens, so the stream is
                    // the same under every manoeuvre.
                    let turn_walk = normal(&mut rng) * noise.turn_walk_dps * libm::sqrt(h);
                    let speed_walk = normal(&mut rng) * noise.speed_walk_frac * libm::sqrt(h);
                    let wind_err = (normal(&mut rng), normal(&mut rng));

                    // Vertical.
                    let target_vs = match targets.vs_ms.filter(|_| following) {
                        Some(vs) => vs.clamp(
                            -perf.max_descent_ms(p.alt, p.tas, mass),
                            perf.max_climb_ms(p.alt, p.tas, mass).max(0.0),
                        ),
                        None => match phase {
                            Phase::Climb => scale * perf.max_climb_ms(p.alt, p.tas, mass),
                            Phase::Descent => {
                                scale * perf.energy_rate_ms(p.alt, p.tas, mass, Thrust::Idle)
                            }
                            Phase::Level => 0.0,
                        },
                    };
                    p.vs = approach(p.vs, target_vs, v_accel * h);

                    // Speed: a commanded one, else the schedule in a climb or
                    // descent, else what it has, wandering a little.
                    let (low, high) = perf.speed_envelope(p.alt);
                    let target_tas = match targets.tas_ms.filter(|_| following) {
                        Some(tas) => tas,
                        None if phase != Phase::Level || targets.vs_ms.is_some() => {
                            perf.scheduled_tas(cas0, mach0, p.alt) * p.ratio
                        }
                        None => p.tas * (1.0 + speed_walk),
                    };
                    p.tas = approach(p.tas, target_tas.clamp(low, high), s_accel * h);

                    // Heading.
                    match targets.heading_deg.filter(|_| following) {
                        Some(hdg) => {
                            let left = turn_to(p.hdg, hdg);
                            p.hdg += left.clamp(-turn_rate * h, turn_rate * h);
                        }
                        None => {
                            p.turn = (p.turn + turn_walk).clamp(-turn_rate, turn_rate);
                            p.hdg += p.turn * h;
                        }
                    }

                    // Position, through the wind.
                    let (we, wn) = self.wind.wind(p.lat, p.lon, p.alt, t);
                    let rad = p.hdg.to_radians();
                    let east = p.tas * libm::sin(rad) + we + wind_err.0 * noise.wind_ms;
                    let north = p.tas * libm::cos(rad) + wn + wind_err.1 * noise.wind_ms;
                    (p.lat, p.lon) = displace(p.lat, p.lon, east * h, north * h);
                    p.alt += p.vs * h;
                    if p.alt >= ceiling {
                        p.alt = ceiling;
                        p.vs = p.vs.min(0.0);
                    }
                    if p.alt <= 0.0 {
                        p.alt = 0.0;
                        p.vs = p.vs.max(0.0);
                    }
                }
                t += h;
            }
            self.record(&particles, t, &mut out);
        }
        out
    }

    fn record(&mut self, particles: &[Particle], t: f64, out: &mut Prediction) {
        let cfg = self.cfg;
        let share = 1.0 / particles.len() as f64;
        let mut cells: BTreeMap<Key, f64> = BTreeMap::new();
        let mut mean = [0.0; 3];
        let mut covered: Vec<Key> = Vec::new();
        for p in particles {
            mean[0] += p.lat * share;
            mean[1] += p.lon * share;
            mean[2] += p.alt * share;
            let Ok(cell) = GeoCell::from_lonlat(p.lon, p.lat, cfg.resolution) else {
                continue;
            };
            let low = cfg.layers.index(p.alt - cfg.vertical_buffer_m);
            let high = cfg.layers.index(p.alt + cfg.vertical_buffer_m);
            covered.clear();
            for &c in self.zone(cell) {
                for layer in low..=high {
                    covered.push(Key { cell: c, layer });
                }
            }
            covered.sort_unstable();
            covered.dedup();
            for k in &covered {
                *cells.entry(*k).or_insert(0.0) += share;
            }
        }
        out.times_s.push(t);
        out.cells.push(cells);
        out.mean.push(mean);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn quad(id: u64, lat: f64, lon: f64, hdg: f64) -> AircraftState {
        AircraftState {
            id,
            lat,
            lon,
            alt_m: 60.0,
            tas_ms: 10.0,
            heading_deg: hdg,
            vs_ms: 0.0,
            turn_rate_dps: 0.0,
            mass_kg: None,
            performance: "quad".into(),
        }
    }

    #[test]
    fn a_prediction_is_probabilities_that_follow_the_track() {
        let cfg = Config::drones();
        let perf = Performance::generic_quadcopter();
        let mut p = Predictor::new(&cfg, &Calm);
        let pred = p.predict(&quad(1, 54.9, 23.9, 90.0), &perf, Manoeuvre::Maintain);
        assert_eq!(pred.cells.len(), cfg.steps);
        for step in &pred.cells {
            assert!(step.values().all(|&v| v > 0.0 && v <= 1.0 + 1e-9));
        }
        // Flying east at 10 m/s: 600 m east after a minute.
        let last = pred.mean.last().unwrap();
        let east = (last[1] - 23.9).to_radians() * EARTH_RADIUS_M * 54.9f64.to_radians().cos();
        assert!(
            (east - 10.0 * cfg.step_s * cfg.steps as f64).abs() < 40.0,
            "{east}"
        );
        // Uncertainty grows: later steps spread over more cells.
        assert!(pred.cells.last().unwrap().len() >= pred.cells[0].len());
    }

    #[test]
    fn the_same_aircraft_gets_the_same_cloud_every_time() {
        let cfg = Config::drones();
        let perf = Performance::generic_quadcopter();
        let mut p = Predictor::new(&cfg, &Calm);
        let a = p.predict(&quad(7, 54.9, 23.9, 10.0), &perf, Manoeuvre::Maintain);
        let b = p.predict(&quad(7, 54.9, 23.9, 10.0), &perf, Manoeuvre::Maintain);
        assert_eq!(a, b);
    }

    #[test]
    fn a_climb_advisory_lifts_the_cloud_after_the_response_delay() {
        let cfg = Config::drones();
        let perf = Performance::generic_quadcopter();
        let mut p = Predictor::new(&cfg, &Calm);
        let s = quad(3, 54.9, 23.9, 0.0);
        let level = p.predict(&s, &perf, Manoeuvre::Maintain);
        let climb = p.predict(&s, &perf, Manoeuvre::Climb);
        let rise = climb.mean.last().unwrap()[2] - level.mean.last().unwrap()[2];
        // 3 m/s for most of a minute, less the delay and the push-over.
        assert!((140.0..180.0).contains(&rise), "{rise}");
    }

    #[test]
    fn wind_carries_the_cloud() {
        struct West;
        impl WindSource for West {
            fn wind(&self, _: f64, _: f64, _: f64, _: f64) -> (f64, f64) {
                (5.0, 0.0)
            }
        }
        let cfg = Config::drones();
        let perf = Performance::generic_quadcopter();
        let s = quad(4, 54.9, 23.9, 0.0);
        let calm = Predictor::new(&cfg, &Calm).predict(&s, &perf, Manoeuvre::Maintain);
        let windy = Predictor::new(&cfg, &West).predict(&s, &perf, Manoeuvre::Maintain);
        assert!(windy.mean.last().unwrap()[1] > calm.mean.last().unwrap()[1]);
    }
}
