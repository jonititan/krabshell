//! Where drones spend their time in the air, accumulated into A5 cells.
//!
//! [`Heatmap::observe`] takes the drones from successive published views and
//! credits the simulated time between two observations to the cells along the
//! straight line the drone flew between them, sampled every quarter of a
//! cell edge (at most [`STEP_M`] metres), so no cell along the way is skipped. The result is drone-seconds of flight per cell, which
//! reads the same whether the views came every tick or every few seconds of
//! simulated time (the gap only blurs the corners at waypoints).
//!
//! A drone counts as flying unless it is disarmed or sitting on its home pad.
//! Nothing here touches the simulation: it only reads what any edge reads.

use std::collections::BTreeMap;

use sim_core::GeoCell;

use super::flight::{self, Point};
use super::mission::Mode;
use super::DroneStatus;

/// Widest spacing of the points a segment is credited at, metres.
pub const STEP_M: f64 = 25.0;

/// ColorBrewer YlOrRd, 7 classes: a sequential choropleth palette that reads
/// on light and dark basemaps and in greyscale.
const PALETTE: [&str; 7] = [
    "#ffffb2", "#fed976", "#feb24c", "#fd8d3c", "#fc4e2a", "#e31a1c", "#b10026",
];

/// A jump longer than this between two observations is a reset or a restored
/// checkpoint, not a flight: it is credited where the drone ended up.
const TELEPORT_M: f64 = 5_000.0;

#[derive(Debug, Clone, Copy)]
struct Seen {
    tick: u64,
    at: Point,
    flying: bool,
}

#[derive(Debug, Clone)]
pub struct Heatmap {
    resolution: i32,
    tick_seconds: f64,
    /// Spacing of the points a segment is credited at, metres.
    step_m: f64,
    first_tick: Option<u64>,
    last_tick: u64,
    last: BTreeMap<u8, Seen>,
    /// Drone-seconds of flight, by cell.
    seconds: BTreeMap<GeoCell, f64>,
    /// Simulated seconds each drone has flown, by callsign.
    per_drone: BTreeMap<String, f64>,
}

/// Flying, as opposed to disarmed or parked on the home pad.
pub fn flying(drone: &DroneStatus) -> bool {
    if matches!(drone.mode, Mode::Disarmed) {
        return false;
    }
    let on_pad = flight::ground_distance_m(&drone.pose.point(), &drone.home) <= 15.0
        && (drone.pose.alt_mm - drone.home.alt_mm).abs() < 500;
    !on_pad
}

/// Settings for a flight heatmap from a pack's params: `heatmap_resolution`
/// (an A5 resolution; absent means the pack does not ask for one) and the
/// pack's `tick_seconds`.
pub fn settings(config: &sim_core::config::Config) -> Option<(i32, f64)> {
    let resolution = config.model.get("heatmap_resolution")?.as_i64()?;
    let tick_seconds = config
        .tick_seconds()
        .unwrap_or(sim_core::config::DEFAULT_TICK_SECONDS);
    (0..=30)
        .contains(&resolution)
        .then_some((resolution as i32, tick_seconds))
}

impl Heatmap {
    pub fn new(resolution: i32, tick_seconds: f64) -> Self {
        Heatmap {
            resolution,
            tick_seconds,
            step_m: (sim_core::cell::nominal_cell_edge_meters(resolution) / 4.0).clamp(1.0, STEP_M),
            first_tick: None,
            last_tick: 0,
            last: BTreeMap::new(),
            seconds: BTreeMap::new(),
            per_drone: BTreeMap::new(),
        }
    }

    pub fn resolution(&self) -> i32 {
        self.resolution
    }

    /// The tick of the last observation.
    pub fn last_tick(&self) -> u64 {
        self.last_tick
    }

    /// Simulated seconds between the first and last observation.
    pub fn span_seconds(&self) -> f64 {
        self.first_tick.map_or(0.0, |first| {
            (self.last_tick - first) as f64 * self.tick_seconds
        })
    }

    pub fn drone_seconds(&self) -> f64 {
        self.seconds.values().sum()
    }

    /// Fold in the drones as they stood at `tick`. Ticks must not go
    /// backwards; start a new heatmap after a reset.
    pub fn observe(&mut self, tick: u64, drones: &BTreeMap<u8, DroneStatus>) {
        if self.first_tick.is_none() {
            self.first_tick = Some(tick);
        }
        if tick < self.last_tick {
            return;
        }
        self.last_tick = tick;
        for (sysid, drone) in drones {
            let now = Seen {
                tick,
                at: drone.pose.point(),
                flying: flying(drone),
            };
            if let Some(before) = self.last.insert(*sysid, now) {
                if tick > before.tick && (now.flying || before.flying) {
                    let dt = (tick - before.tick) as f64 * self.tick_seconds;
                    self.credit(&before.at, &now.at, dt);
                    *self.per_drone.entry(drone.callsign.clone()).or_default() += dt;
                }
            }
        }
    }

    fn credit(&mut self, from: &Point, to: &Point, seconds: f64) {
        let distance = flight::ground_distance_m(from, to);
        let (from, steps) = if distance > TELEPORT_M {
            (*to, 1)
        } else {
            (*from, (distance / self.step_m).ceil().max(1.0) as usize)
        };
        let share = seconds / steps as f64;
        for i in 0..steps {
            let f = (i as f64 + 0.5) / steps as f64;
            let lat = from.lat() + (to.lat() - from.lat()) * f;
            let lon = from.lon() + (to.lon() - from.lon()) * f;
            if let Ok(cell) = GeoCell::from_lonlat(lon, lat, self.resolution) {
                *self.seconds.entry(cell).or_default() += share;
            }
        }
    }

    /// A GeoJSON FeatureCollection: one polygon per cell flown over, with
    /// `drone_seconds` and its `share` of the total, plus a summary in the
    /// collection's `properties`.
    ///
    /// It is a ready-styled choropleth: cells fall into [`PALETTE`]'s classes
    /// on a log scale of drone-seconds from the 5th-percentile cell to the
    /// busiest (a few corridors carry most of the time, so a linear scale
    /// would leave nearly every cell the palest colour), and carry simplestyle `fill`/`stroke` properties that
    /// geojson.io and other simplestyle viewers draw as they are. The class
    /// breaks are in the collection's `legend`. QGIS can style on `class`.
    pub fn to_geojson(&self, scenario_id: &str) -> serde_json::Value {
        let total = self.drone_seconds();
        let peak = self.seconds.values().copied().fold(0.0, f64::max);
        let classes = PALETTE.len();
        // A log scale from the 5th-percentile cell to the busiest. Starting at
        // zero instead would spend the bottom classes on the slivers where a
        // segment just clips a cell.
        let mut values: Vec<f64> = self.seconds.values().copied().collect();
        values.sort_by(f64::total_cmp);
        let floor = values
            .get(values.len() / 20)
            .copied()
            .unwrap_or(0.0)
            .max(peak * 1e-6)
            .max(1e-9);
        let span = (peak / floor).ln();
        let class_of = |seconds: f64| -> usize {
            if span <= 0.0 || seconds <= floor {
                return 0;
            }
            (((seconds / floor).ln() / span * classes as f64) as usize).min(classes - 1)
        };
        // Class i holds drone-seconds from edge(i) up to edge(i + 1); the
        // first class also takes everything below the floor.
        let edge = |i: usize| {
            if i == 0 {
                0.0
            } else {
                floor * (span * i as f64 / classes as f64).exp()
            }
        };
        let legend: Vec<serde_json::Value> = (0..classes)
            .map(|i| {
                serde_json::json!({
                    "class": i + 1,
                    "fill": PALETTE[i],
                    "from_drone_seconds": round3(edge(i)),
                    "to_drone_seconds": round3(if i + 1 == classes { peak } else { edge(i + 1) }),
                })
            })
            .collect();
        // Hottest last, so their outlines sit on top where cells meet.
        let mut ordered: Vec<(&GeoCell, &f64)> = self.seconds.iter().collect();
        ordered.sort_by(|a, b| a.1.total_cmp(b.1));
        let features: Vec<serde_json::Value> = ordered
            .into_iter()
            .filter_map(|(cell, seconds)| {
                let mut ring: Vec<[f64; 2]> = cell
                    .boundary()
                    .ok()?
                    .into_iter()
                    .map(|(lon, lat)| [round7(lon), round7(lat)])
                    .collect();
                if ring.first() != ring.last() {
                    ring.push(ring[0]);
                }
                Some(serde_json::json!({
                    "type": "Feature",
                    "geometry": { "type": "Polygon", "coordinates": [ring] },
                    "properties": {
                        "cell": cell.to_hex(),
                        "drone_seconds": round3(*seconds),
                        "share": if total > 0.0 { round6(seconds / total) } else { 0.0 },
                        "of_peak": if peak > 0.0 { round6(seconds / peak) } else { 0.0 },
                        "class": class_of(*seconds) + 1,
                        "fill": PALETTE[class_of(*seconds)],
                        "fill-opacity": 0.75,
                        "stroke": PALETTE[class_of(*seconds)],
                        "stroke-width": 0.5,
                        "stroke-opacity": 1.0,
                    },
                }))
            })
            .collect();
        let per_drone: serde_json::Map<String, serde_json::Value> = self
            .per_drone
            .iter()
            .map(|(callsign, s)| (callsign.clone(), round3(*s).into()))
            .collect();
        serde_json::json!({
            "type": "FeatureCollection",
            "properties": {
                "scenario": scenario_id,
                "what": "drone-seconds of flight per A5 cell",
                "resolution": self.resolution,
                "cell_edge_m": round3(sim_core::cell::nominal_cell_edge_meters(self.resolution)),
                "first_tick": self.first_tick,
                "last_tick": self.last_tick,
                "simulated_seconds": round3(self.span_seconds()),
                "drone_seconds": round3(total),
                "peak_drone_seconds": round3(peak),
                "cells": self.seconds.len(),
                "flight_seconds_by_drone": per_drone,
                "scale": "log",
                "legend": legend,
            },
            "features": features,
        })
    }
}

fn round3(v: f64) -> f64 {
    (v * 1e3).round() / 1e3
}

fn round6(v: f64) -> f64 {
    (v * 1e6).round() / 1e6
}

fn round7(v: f64) -> f64 {
    (v * 1e7).round() / 1e7
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::models::drones::flight::Pose;

    fn status(sysid: u8, pose: Pose, home: Point, mode: Mode) -> DroneStatus {
        DroneStatus {
            id: u64::from(sysid),
            cell: GeoCell::from_lonlat(pose.lon(), pose.lat(), 15).unwrap(),
            sysid,
            callsign: format!("D{sysid}"),
            pose,
            mode,
            home,
            mission: Vec::new(),
            mission_seq: 0,
            mission_rev: 0,
            battery_pct: 100.0,
        }
    }

    #[test]
    fn a_straight_leg_spreads_its_time_along_the_cells_it_crosses() {
        let home = Pose::at(23.90, 54.92, 50.0);
        let mut map = Heatmap::new(15, 0.1);
        let start = Pose::at(23.90, 54.92, 110.0);
        let (lat, lon) = flight::displace(start.lat_e7, start.lon_e7, 2_000.0, 0.0);
        let end = Pose {
            lat_e7: lat,
            lon_e7: lon,
            ..start
        };
        map.observe(
            0,
            &BTreeMap::from([(1, status(1, start, home.point(), Mode::Auto))]),
        );
        map.observe(
            1_000,
            &BTreeMap::from([(1, status(1, end, home.point(), Mode::Auto))]),
        );

        // 1000 ticks of 0.1 s, all credited, over a 2 km line of ~180 m cells.
        assert!((map.drone_seconds() - 100.0).abs() < 1e-9);
        assert!(map.seconds.len() >= 8, "{} cells", map.seconds.len());
        let json = map.to_geojson("t");
        assert_eq!(json["properties"]["flight_seconds_by_drone"]["D1"], 100.0);
        // Every feature is styled with a palette colour matching its class,
        // and the busiest cell is in the top class.
        let features = json["features"].as_array().unwrap();
        for f in features {
            let class = f["properties"]["class"].as_u64().unwrap() as usize;
            assert_eq!(f["properties"]["fill"], PALETTE[class - 1]);
        }
        let hottest = features.last().unwrap();
        assert_eq!(hottest["properties"]["of_peak"], 1.0);
        assert_eq!(hottest["properties"]["class"], PALETTE.len());
        assert_eq!(
            json["properties"]["legend"].as_array().unwrap().len(),
            PALETTE.len()
        );
        assert_eq!(
            json["features"].as_array().unwrap().len(),
            map.seconds.len()
        );
    }

    #[test]
    fn time_parked_on_the_pad_or_disarmed_is_not_flight() {
        let pad = Pose::at(23.90, 54.92, 50.0);
        let mut map = Heatmap::new(15, 0.1);
        for tick in [0, 100, 200] {
            map.observe(
                tick,
                &BTreeMap::from([
                    (1, status(1, pad, pad.point(), Mode::Auto)),
                    (
                        2,
                        status(2, Pose::at(23.95, 54.90, 30.0), pad.point(), Mode::Disarmed),
                    ),
                ]),
            );
        }
        assert_eq!(map.drone_seconds(), 0.0);
        assert!((map.span_seconds() - 20.0).abs() < 1e-9);
    }
}
