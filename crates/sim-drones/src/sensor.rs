//! A downward-looking camera with a range-dependent chance of noticing things.
//!
//! The footprint on the ground is a circle whose radius grows with altitude
//! (`alt · tan(hfov / 2)`) up to the sensor's maximum range. Inside it, the
//! chance of a detection on each scan falls linearly from `p_max` at the centre
//! to `p_max · (1 − edge_falloff)` at the rim. Other drones are spotted by
//! slant range instead, since they are not on the ground.
//!
//! The draw for each (observer, target, tick) comes from its own seeded stream,
//! so a `gold` run sees exactly the same detections every time, whatever order
//! the scan happens in.

use std::collections::BTreeMap;

use sim_core::AgentId;

use crate::flight::{self, Point};
use sim_core::rng::gold_draw;

/// Keeps sensor draws independent of every other stream seeded from the run.
const SENSOR_SALT: u64 = 0x5e45_0e5d_d7a9_c3b1;

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Sensor {
    /// `tan(hfov / 2)`.
    pub half_fov_tan: f64,
    pub max_range_m: f64,
    pub p_max: f64,
    pub edge_falloff: f64,
    /// Scan every this many ticks.
    pub every_ticks: u64,
    /// Report the same target to the same observer at most this often.
    pub report_every_ticks: u64,
}

/// Something a scan could pick up.
pub struct Candidate {
    pub id: AgentId,
    pub at: Point,
    /// Airborne targets are searched for by slant range, not the ground footprint.
    pub airborne: bool,
}

/// A detection worth reporting.
pub struct Sighting {
    pub target: AgentId,
    pub range_m: f64,
}

impl Sensor {
    pub fn scans_on(&self, tick: u64) -> bool {
        tick % self.every_ticks.max(1) == 0
    }

    /// Ground radius the camera covers from `alt_m`.
    pub fn footprint_m(&self, alt_m: f64) -> f64 {
        (alt_m.max(0.0) * self.half_fov_tan).min(self.max_range_m)
    }

    /// Scan once from `observer` at `from`, over ground at `ground_mm`. Detections not already reported
    /// recently are pushed to `out`; `last` remembers when each pair was last
    /// reported.
    #[allow(clippy::too_many_arguments)]
    pub fn scan(
        &self,
        seed: u64,
        tick: u64,
        observer: AgentId,
        from: &Point,
        ground_mm: i32,
        candidates: impl Iterator<Item = Candidate>,
        last: &mut BTreeMap<(AgentId, AgentId), u64>,
        out: &mut Vec<Sighting>,
    ) {
        let from_len = out.len();
        self.scan_seen(seed, tick, observer, from, ground_mm, candidates, last, out);
        for sighting in &out[from_len..] {
            last.insert((observer, sighting.target), tick);
        }
    }

    /// [`Sensor::scan`] without remembering: the caller records each
    /// sighting pushed to `out` as reported at `tick`. Candidates are
    /// distinct, so no pair is looked up after it would have been recorded.
    #[allow(clippy::too_many_arguments)]
    pub fn scan_seen(
        &self,
        seed: u64,
        tick: u64,
        observer: AgentId,
        from: &Point,
        ground_mm: i32,
        candidates: impl Iterator<Item = Candidate>,
        last: &BTreeMap<(AgentId, AgentId), u64>,
        out: &mut Vec<Sighting>,
    ) {
        // The footprint grows with height above the ground below, not above sea level.
        let alt_m = f64::from(from.alt_mm - ground_mm) * 1e-3;
        let footprint = self.footprint_m(alt_m);
        if footprint < 1.0 {
            return;
        }
        let stream = seed ^ SENSOR_SALT ^ observer.wrapping_mul(0xd6e8_feb8_6659_fd93);
        for candidate in candidates {
            if candidate.id == observer {
                continue;
            }
            let ground = flight::ground_distance_m(from, &candidate.at);
            let (range, reach) = if candidate.airborne {
                let dz = f64::from(candidate.at.alt_mm - from.alt_mm) * 1e-3;
                (libm::sqrt(ground * ground + dz * dz), self.max_range_m)
            } else {
                (ground, footprint)
            };
            if range > reach {
                continue;
            }
            let p = self.p_max * (1.0 - self.edge_falloff * (range / reach));
            let (roll, _) = gold_draw(stream, candidate.id, tick);
            if roll >= p {
                continue;
            }
            let key = (observer, candidate.id);
            let due = last
                .get(&key)
                .is_none_or(|at| tick.saturating_sub(*at) >= self.report_every_ticks);
            if due {
                out.push(Sighting {
                    target: candidate.id,
                    range_m: range,
                });
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::flight::Pose;

    const SENSOR: Sensor = Sensor {
        half_fov_tan: 0.7,
        max_range_m: 150.0,
        p_max: 0.9,
        edge_falloff: 0.6,
        every_ticks: 1,
        report_every_ticks: 100,
    };

    fn ground_target(id: AgentId, from: &Pose, east_m: f64) -> (AgentId, Point) {
        let (lat, lon) = flight::displace(from.lat_e7, from.lon_e7, east_m, 0.0);
        (
            id,
            Point {
                lat_e7: lat,
                lon_e7: lon,
                alt_mm: 0,
            },
        )
    }

    fn scan(
        sensor: &Sensor,
        tick: u64,
        from: &Pose,
        targets: &[(AgentId, Point)],
        last: &mut BTreeMap<(AgentId, AgentId), u64>,
    ) -> Vec<AgentId> {
        let mut out = Vec::new();
        sensor.scan(
            42,
            tick,
            1,
            &from.point(),
            0,
            targets.iter().map(|(id, at)| Candidate {
                id: *id,
                at: *at,
                airborne: false,
            }),
            last,
            &mut out,
        );
        out.into_iter().map(|s| s.target).collect()
    }

    #[test]
    fn the_footprint_grows_with_altitude_up_to_the_maximum_range() {
        assert_eq!(SENSOR.footprint_m(0.0), 0.0);
        assert!((SENSOR.footprint_m(100.0) - 70.0).abs() < 1e-9);
        assert_eq!(SENSOR.footprint_m(1_000.0), 150.0);
    }

    #[test]
    fn nothing_outside_the_footprint_is_ever_seen() {
        let drone = Pose::at(0.0, 0.0, 50.0); // footprint 35 m
        let targets = [
            ground_target(2, &drone, 36.0),
            ground_target(3, &drone, 200.0),
        ];
        let mut last = BTreeMap::new();
        for tick in 0..500 {
            assert!(scan(&SENSOR, tick, &drone, &targets, &mut last).is_empty());
        }
    }

    #[test]
    fn a_target_under_the_drone_is_seen_then_not_re_reported_until_the_interval() {
        let drone = Pose::at(0.0, 0.0, 50.0);
        let targets = [ground_target(2, &drone, 1.0)];
        let mut last = BTreeMap::new();
        let mut reported_at = Vec::new();
        for tick in 0..250 {
            if !scan(&SENSOR, tick, &drone, &targets, &mut last).is_empty() {
                reported_at.push(tick);
            }
        }
        assert!(
            reported_at.len() >= 2 && reported_at.len() <= 3,
            "{reported_at:?}"
        );
        for pair in reported_at.windows(2) {
            assert!(pair[1] - pair[0] >= 100);
        }
    }

    #[test]
    fn the_same_seed_and_tick_give_the_same_detections() {
        let drone = Pose::at(0.0, 0.0, 100.0);
        let targets: Vec<_> = (0..40)
            .map(|i| ground_target(10 + i, &drone, f64::from(i as u32) * 1.7))
            .collect();
        let run = || {
            let mut last = BTreeMap::new();
            (0..50)
                .map(|t| scan(&SENSOR, t, &drone, &targets, &mut last))
                .collect::<Vec<_>>()
        };
        assert_eq!(run(), run());
    }
}
