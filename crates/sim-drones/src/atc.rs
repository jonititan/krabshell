//! Conflict advisories, flown.
//!
//! Every `atc_every_s` the world assesses the aircraft in the air with
//! `sim-conflict` ([`World::run_atc`](crate::World)), the same assessment the
//! dashboard's Conflicts screen shows, and each aircraft advised flies its
//! manoeuvre for `advisory_hold_s` ([`Avoid`]) before its autopilot or route
//! takes over again. Drones give way first; a manned aircraft is advised only
//! when no drone can resolve the conflict; an intruder never is, as it would
//! not listen. Drones taking off from or landing on their pads are left out:
//! the stations' slots already keep those apart.
//!
//! It all happens inside a step, from the committed world, so a gold run
//! flies the same advisories every time, at any speed.

use std::collections::BTreeMap;

use serde::{Deserialize, Serialize};
use sim_conflict::{
    AircraftState, Config, Fleet, GiveWay, Manoeuvre, Multirotor, Performance, Protection, Targets,
};
use sim_core::config::Param;

use crate::flight::{self, Limits, Point, Pose};
use crate::settings::param_f64;

/// The cell resolutions conflicts may be assessed at.
pub const RESOLUTIONS: std::ops::RangeInclusive<i32> = 8..=18;

/// The performance models: drones and intruders as quadcopters at the pack's
/// speeds, manned aircraft as the generic narrow-body jet.
pub fn fleet(params: &BTreeMap<String, Param>) -> Fleet {
    let param = |key: &str, default: f64| {
        let v = param_f64(params, key, default);
        if v > 0.0 {
            v
        } else {
            default
        }
    };
    let quad = |name: &str, cruise: f64| {
        let Performance::Multirotor(base) = Performance::generic_quadcopter() else {
            unreachable!()
        };
        Performance::Multirotor(Multirotor {
            name: name.into(),
            cruise_ms: cruise,
            max_speed_ms: (cruise * 1.5).max(base.max_speed_ms),
            max_climb_ms: param("climb_ms", base.max_climb_ms),
            advisory_vs_ms: param("climb_ms", base.advisory_vs_ms),
            ..base
        })
    };
    Fleet::from([
        (
            "drone".to_owned(),
            quad("krabshell drone", param("cruise_ms", 10.0)),
        ),
        (
            "intruder".to_owned(),
            quad("intruder", param("intruder_speed_ms", 12.0)),
        ),
        ("aircraft".to_owned(), Performance::generic_narrowbody()),
    ])
}

/// The pack's `conflict_resolution`, if it sets one in range.
pub fn pack_resolution(params: &BTreeMap<String, Param>) -> Option<i32> {
    params
        .get("conflict_resolution")
        .and_then(Param::as_i64)
        .map(|r| r as i32)
        .filter(|r| RESOLUTIONS.contains(r))
}

/// The drones preset, on cells of `resolution`.
pub fn config(resolution: i32) -> Config {
    Config {
        resolution,
        ..Config::drones()
    }
}

/// The separation drones and intruders must keep from a manned aircraft:
/// `manned_lateral_m` across (300 m) and `manned_vertical_m` above and below
/// (150 m).
pub fn manned_protection(params: &BTreeMap<String, Param>) -> Protection {
    Protection {
        lateral_m: param_f64(params, "manned_lateral_m", 300.0).max(0.0),
        vertical_m: param_f64(params, "manned_vertical_m", 150.0).max(0.0),
    }
}

/// Who gives way, and what protects it, by kind.
pub fn give_way(kind: &str) -> GiveWay {
    match kind {
        "aircraft" => GiveWay::Last,
        "intruder" => GiveWay::Never,
        _ => GiveWay::First,
    }
}

/// An aircraft's state for assessment, from its pose.
pub fn state(id: u64, kind: &str, pose: &Pose, protection: Protection) -> AircraftState {
    AircraftState {
        id,
        lat: pose.lat(),
        lon: pose.lon(),
        alt_m: pose.alt_m(),
        tas_ms: pose.airspeed_ms(),
        heading_deg: f64::from(pose.yaw()) * 0.01,
        vs_ms: f64::from(pose.vs_cms) * 0.01,
        turn_rate_dps: 0.0,
        mass_kg: None,
        performance: kind.to_owned(),
        protection: (kind == "aircraft").then_some(protection),
        give_way: give_way(kind),
    }
}

/// A conflict advisory being flown. It overrides the autopilot (or a manned
/// aircraft's route) until it runs out; then that carries on from wherever
/// the aircraft is.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct Avoid {
    /// What it was asked to do: `climb`, `descend`, `level_off`,
    /// `turn_left`, `turn_right`, `speed_up` or `slow_down`.
    pub action: String,
    /// Vertical speed to fly, cm/s. Absent, it holds its altitude.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub vs_cms: Option<i32>,
    /// Heading to fly, centidegrees. Absent, it keeps going the way it was,
    /// or holds its position if it was not going anywhere.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub heading_cdeg: Option<i32>,
    /// Airspeed to fly, cm/s. Absent, its usual speed.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub speed_cms: Option<i32>,
    /// Milliseconds it has left.
    pub left_ms: u32,
    /// Who it is keeping clear of, by id.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub against: Vec<u64>,
}

impl Avoid {
    pub fn new(manoeuvre: Manoeuvre, targets: &Targets, hold_ms: u32, against: Vec<u64>) -> Self {
        let action = serde_json::to_value(manoeuvre)
            .ok()
            .and_then(|v| v["action"].as_str().map(str::to_owned))
            .unwrap_or_default();
        Avoid {
            action,
            vs_cms: targets.vs_ms.map(|v| libm::round(v * 100.0) as i32),
            heading_cdeg: targets
                .heading_deg
                .map(|h| libm::round(h.rem_euclid(360.0) * 100.0) as i32),
            speed_cms: targets.tas_ms.map(|v| libm::round(v * 100.0) as i32),
            left_ms: hold_ms,
            against,
        }
    }
}

/// Fly one tick of an advisory, never below `floor_mm`. `limits` carries
/// the usual speed and the wind; the advisory sets what it asks for. True
/// once it has run out.
pub fn fly(
    pose: &mut Pose,
    avoid: &mut Avoid,
    limits: Limits,
    dt: f64,
    dt_ms: u32,
    floor_mm: i32,
) -> bool {
    let heading = avoid
        .heading_cdeg
        .or((pose.spd_cms > 30).then_some(pose.hdg_cdeg));
    let (lat, lon) = match heading {
        Some(h) => {
            let rad = (f64::from(h) * 0.01).to_radians();
            flight::displace(
                pose.lat_e7,
                pose.lon_e7,
                libm::sin(rad) * 1_000.0,
                libm::cos(rad) * 1_000.0,
            )
        }
        None => (pose.lat_e7, pose.lon_e7),
    };
    let vs = f64::from(avoid.vs_cms.unwrap_or(0)) * 0.01;
    let alt_mm = (pose.alt_mm + flight::to_mm(vs * dt)).max(floor_mm);
    let limits = Limits {
        speed_ms: avoid
            .speed_cms
            .map_or(limits.speed_ms, |s| f64::from(s) * 0.01),
        climb_ms: vs.abs().max(0.01),
        ..limits
    };
    flight::step_toward(
        pose,
        &Point {
            lat_e7: lat,
            lon_e7: lon,
            alt_mm,
        },
        limits,
        dt,
    );
    avoid.left_ms = avoid.left_ms.saturating_sub(dt_ms);
    avoid.left_ms == 0
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_descent_advisory_descends_at_its_rate_for_its_time_and_keeps_going_the_same_way() {
        let mut pose = Pose::at(24.0, 55.0, 500.0);
        pose.hdg_cdeg = 9_000;
        pose.spd_cms = 1_000;
        let mut avoid = Avoid::new(
            Manoeuvre::Descend,
            &Targets {
                vs_ms: Some(-2.5),
                ..Targets::default()
            },
            10_000,
            vec![7],
        );
        assert_eq!(avoid.action, "descend");
        let start = pose;
        let mut ticks = 0;
        while !fly(&mut pose, &mut avoid, Limits::still(10.0, 3.0), 0.1, 100, 0) {
            ticks += 1;
        }
        assert_eq!(ticks + 1, 100, "10 s at 0.1 s a tick");
        assert!((pose.alt_m() - 475.0).abs() < 0.01, "{}", pose.alt_m());
        // Still eastbound, 100 m on.
        let (east, north) = flight::offset_m(start.lat_e7, start.lon_e7, pose.lat_e7, pose.lon_e7);
        assert!(
            (east - 100.0).abs() < 1.0 && north.abs() < 0.5,
            "{east} {north}"
        );
    }

    #[test]
    fn a_hovering_drone_descends_in_place_and_never_below_the_floor() {
        let mut pose = Pose::at(24.0, 55.0, 30.0);
        let mut avoid = Avoid::new(
            Manoeuvre::Descend,
            &Targets {
                vs_ms: Some(-2.5),
                ..Targets::default()
            },
            20_000,
            vec![],
        );
        let start = pose.point();
        while !fly(
            &mut pose,
            &mut avoid,
            Limits::still(10.0, 3.0),
            0.1,
            100,
            20_000,
        ) {}
        assert_eq!(pose.alt_mm, 20_000);
        assert_eq!((pose.lat_e7, pose.lon_e7), (start.lat_e7, start.lon_e7));
    }
}
