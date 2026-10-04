//! Positions and straight-line motion over a small patch of the Earth.
//!
//! A pose is stored in the integer units MAVLink's `_INT` messages use —
//! degrees × 10⁷, millimetres, centidegrees, centimetres per second — so the
//! published state converts to telemetry without rounding, and so a `gold` run
//! carries no float state from one tick to the next. Each step converts to
//! `f64`, moves, and rounds back.
//!
//! All trigonometry goes through `libm` rather than the platform maths library,
//! so the same inputs give the same bits on every machine.

use serde::{Deserialize, Serialize};

/// Mean Earth radius, metres. The local-tangent approximation below is good to
/// well under a metre over the few kilometres a scenario spans.
const EARTH_RADIUS_M: f64 = 6_371_000.0;

/// Where something is and how it is moving. Altitude is above mean sea level;
/// see [`super::terrain`].
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize)]
pub struct Pose {
    pub lat_e7: i32,
    pub lon_e7: i32,
    pub alt_mm: i32,
    /// Heading, centidegrees clockwise from true north, 0..36000.
    pub hdg_cdeg: i32,
    /// Ground speed, cm/s.
    pub spd_cms: i32,
    /// Vertical speed, cm/s, positive up.
    #[serde(default)]
    pub vs_cms: i32,
}

impl Pose {
    pub fn at(lon: f64, lat: f64, alt_m: f64) -> Self {
        Pose {
            lat_e7: to_e7(lat),
            lon_e7: to_e7(lon),
            alt_mm: to_mm(alt_m),
            ..Pose::default()
        }
    }

    pub fn lat(&self) -> f64 {
        f64::from(self.lat_e7) * 1e-7
    }

    pub fn lon(&self) -> f64 {
        f64::from(self.lon_e7) * 1e-7
    }

    pub fn alt_m(&self) -> f64 {
        f64::from(self.alt_mm) * 1e-3
    }

    pub fn point(&self) -> Point {
        Point {
            lat_e7: self.lat_e7,
            lon_e7: self.lon_e7,
            alt_mm: self.alt_mm,
        }
    }

    fn stop(&mut self) {
        self.spd_cms = 0;
        self.vs_cms = 0;
    }
}

/// A place to go.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize)]
pub struct Point {
    pub lat_e7: i32,
    pub lon_e7: i32,
    pub alt_mm: i32,
}

impl Point {
    pub fn at(lon: f64, lat: f64, alt_m: f64) -> Self {
        Pose::at(lon, lat, alt_m).point()
    }

    pub fn lat(&self) -> f64 {
        f64::from(self.lat_e7) * 1e-7
    }

    pub fn lon(&self) -> f64 {
        f64::from(self.lon_e7) * 1e-7
    }

    pub fn with_alt_mm(self, alt_mm: i32) -> Self {
        Point { alt_mm, ..self }
    }
}

pub fn to_e7(degrees: f64) -> i32 {
    libm::round(degrees * 1e7) as i32
}

pub fn to_mm(metres: f64) -> i32 {
    libm::round(metres * 1e3) as i32
}

/// East and north offset in metres from `from` to `to`.
pub fn offset_m(from_lat_e7: i32, from_lon_e7: i32, to_lat_e7: i32, to_lon_e7: i32) -> (f64, f64) {
    let lat0 = f64::from(from_lat_e7) * 1e-7;
    let dlat = f64::from(to_lat_e7.wrapping_sub(from_lat_e7)) * 1e-7;
    let dlon = f64::from(to_lon_e7.wrapping_sub(from_lon_e7)) * 1e-7;
    let rad = core::f64::consts::PI / 180.0;
    let east = dlon * rad * EARTH_RADIUS_M * libm::cos(lat0 * rad);
    let north = dlat * rad * EARTH_RADIUS_M;
    (east, north)
}

/// Horizontal distance in metres.
pub fn ground_distance_m(a: &Point, b: &Point) -> f64 {
    let (e, n) = offset_m(a.lat_e7, a.lon_e7, b.lat_e7, b.lon_e7);
    libm::sqrt(e * e + n * n)
}

/// Move `metres_east`/`metres_north` from a position, returning the new
/// latitude and longitude in degrees × 10⁷.
pub fn displace(lat_e7: i32, lon_e7: i32, east_m: f64, north_m: f64) -> (i32, i32) {
    let rad = core::f64::consts::PI / 180.0;
    let lat0 = f64::from(lat_e7) * 1e-7;
    let dlat = north_m / EARTH_RADIUS_M / rad;
    let cos_lat = libm::cos(lat0 * rad).max(1e-6);
    let dlon = east_m / (EARTH_RADIUS_M * cos_lat) / rad;
    (
        lat_e7.saturating_add(to_e7(dlat)),
        lon_e7.saturating_add(to_e7(dlon)),
    )
}

/// How fast something may move.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Limits {
    pub speed_ms: f64,
    pub climb_ms: f64,
}

/// What a step left to do.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Remaining {
    pub ground_m: f64,
    pub vertical_m: f64,
}

impl Remaining {
    pub fn within(&self, ground_m: f64, vertical_m: f64) -> bool {
        self.ground_m <= ground_m && self.vertical_m <= vertical_m
    }
}

/// Fly straight toward `target` for `dt` seconds.
///
/// Horizontal and vertical motion are independent, as a multirotor's are:
/// it turns to face the target at once and never overshoots.
pub fn step_toward(pose: &mut Pose, target: &Point, limits: Limits, dt: f64) -> Remaining {
    let (east, north) = offset_m(pose.lat_e7, pose.lon_e7, target.lat_e7, target.lon_e7);
    let ground = libm::sqrt(east * east + north * north);
    let reach = limits.speed_ms * dt;

    if ground > 0.005 {
        let travel = ground.min(reach);
        let (lat, lon) = displace(
            pose.lat_e7,
            pose.lon_e7,
            east / ground * travel,
            north / ground * travel,
        );
        pose.lat_e7 = lat;
        pose.lon_e7 = lon;
        pose.hdg_cdeg = heading_cdeg(east, north);
        pose.spd_cms = libm::round(travel / dt * 100.0) as i32;
    } else {
        pose.lat_e7 = target.lat_e7;
        pose.lon_e7 = target.lon_e7;
        pose.spd_cms = 0;
    }

    let dz = f64::from(target.alt_mm - pose.alt_mm) * 1e-3;
    let climb = limits.climb_ms * dt;
    let moved = dz.clamp(-climb, climb);
    pose.alt_mm += to_mm(moved);
    pose.vs_cms = libm::round(moved / dt * 100.0) as i32;

    let after = ground_distance_m(&pose.point(), target);
    Remaining {
        ground_m: after,
        vertical_m: f64::from((target.alt_mm - pose.alt_mm).abs()) * 1e-3,
    }
}

/// Stay put, reporting zero motion.
pub fn hold(pose: &mut Pose) {
    pose.stop();
}

/// Compass heading of an east/north vector, in centidegrees.
pub fn heading_cdeg(east: f64, north: f64) -> i32 {
    let degrees = libm::atan2(east, north).to_degrees();
    let cdeg = libm::round(degrees * 100.0) as i32;
    cdeg.rem_euclid(36_000)
}

#[cfg(test)]
mod tests {
    use super::*;

    const LIMITS: Limits = Limits {
        speed_ms: 10.0,
        climb_ms: 3.0,
    };

    #[test]
    fn a_hundred_metres_north_takes_ten_seconds_at_ten_metres_a_second() {
        let mut pose = Pose::at(-0.1276, 51.5072, 20.0);
        let (lat, lon) = displace(pose.lat_e7, pose.lon_e7, 0.0, 100.0);
        let target = Point {
            lat_e7: lat,
            lon_e7: lon,
            alt_mm: 20_000,
        };
        let mut ticks = 0;
        loop {
            let left = step_toward(&mut pose, &target, LIMITS, 0.1);
            ticks += 1;
            if left.within(0.05, 0.05) {
                break;
            }
            assert!(ticks < 1000);
        }
        assert!((99..=101).contains(&ticks), "{ticks}");
        assert_eq!(pose.hdg_cdeg, 0);
    }

    #[test]
    fn heading_is_compass_clockwise_from_north() {
        assert_eq!(heading_cdeg(1.0, 0.0), 9_000);
        assert_eq!(heading_cdeg(0.0, -1.0), 18_000);
        assert_eq!(heading_cdeg(-1.0, 0.0), 27_000);
    }

    #[test]
    fn climbing_is_rate_limited_and_does_not_overshoot() {
        let mut pose = Pose::at(0.0, 0.0, 0.0);
        let target = pose.point().with_alt_mm(1_000);
        let left = step_toward(&mut pose, &target, LIMITS, 0.1);
        assert_eq!(pose.alt_mm, 300);
        assert!(!left.within(0.1, 0.1));
        for _ in 0..5 {
            step_toward(&mut pose, &target, LIMITS, 0.1);
        }
        assert_eq!(pose.alt_mm, 1_000);
    }

    #[test]
    fn distance_matches_displacement() {
        let (lat, lon) = displace(515_072_000, -1_276_000, 300.0, 400.0);
        let d = ground_distance_m(
            &Point {
                lat_e7: 515_072_000,
                lon_e7: -1_276_000,
                alt_mm: 0,
            },
            &Point {
                lat_e7: lat,
                lon_e7: lon,
                alt_mm: 0,
            },
        );
        assert!((d - 500.0).abs() < 0.05, "{d}");
    }
}
