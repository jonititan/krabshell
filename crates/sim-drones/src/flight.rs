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
/// see [`crate::terrain`].
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
    /// Horizontal airspeed, cm/s, when it flew in wind. Absent in still
    /// air, where airspeed is ground speed.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub air_cms: Option<i32>,
    /// Where the nose points, centidegrees, when it flew in wind: crabbed
    /// off the track to hold it, or into the wind to hover. Absent in still
    /// air, where it points along the track.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub yaw_cdeg: Option<i32>,
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

    /// Horizontal airspeed, m/s.
    pub fn airspeed_ms(&self) -> f64 {
        f64::from(self.air_cms.unwrap_or(self.spd_cms)) * 0.01
    }

    /// Where the nose points, centidegrees.
    pub fn yaw(&self) -> i32 {
        self.yaw_cdeg.unwrap_or(self.hdg_cdeg)
    }

    fn stop(&mut self) {
        self.spd_cms = 0;
        self.vs_cms = 0;
        self.air_cms = None;
        self.yaw_cdeg = None;
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

/// How fast something may move, and the air it moves through.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Limits {
    /// Horizontal airspeed it flies at; in still air, its ground speed.
    pub speed_ms: f64,
    pub climb_ms: f64,
    /// The wind, toward the east and north, m/s. Zero for anything on the
    /// ground.
    pub wind_east_ms: f64,
    pub wind_north_ms: f64,
}

impl Limits {
    /// Limits in still air.
    pub const fn still(speed_ms: f64, climb_ms: f64) -> Limits {
        Limits {
            speed_ms,
            climb_ms,
            wind_east_ms: 0.0,
            wind_north_ms: 0.0,
        }
    }

    fn calm(&self) -> bool {
        self.wind_east_ms == 0.0 && self.wind_north_ms == 0.0
    }
}

/// The fastest ground speed along a unit direction `(de, dn)` an aircraft
/// flying at `airspeed` can make in wind `(we, wn)`, holding its track: it
/// crabs into the crosswind and the rest of its airspeed goes along. `None`
/// if the crosswind alone is more than it can fly, or the headwind leaves
/// it no headway.
pub fn ground_speed_along(de: f64, dn: f64, we: f64, wn: f64, airspeed: f64) -> Option<f64> {
    let along = we * de + wn * dn;
    let cross_sq = (we * we + wn * wn - along * along).max(0.0);
    let spare = airspeed * airspeed - cross_sq;
    if spare < 0.0 {
        return None;
    }
    let speed = along + libm::sqrt(spare);
    (speed > 1e-6).then_some(speed)
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
/// it turns at once and never overshoots. In still air it faces the target
/// and flies at `limits.speed_ms`. In wind it holds the straight track by
/// crabbing into the crosswind, so its ground speed is what the wind leaves
/// it; a wind stronger than it can fly blows it back, nose into the wind.
pub fn step_toward(pose: &mut Pose, target: &Point, limits: Limits, dt: f64) -> Remaining {
    let (east, north) = offset_m(pose.lat_e7, pose.lon_e7, target.lat_e7, target.lon_e7);
    let ground = libm::sqrt(east * east + north * north);

    if ground <= 0.005 {
        pose.lat_e7 = target.lat_e7;
        pose.lon_e7 = target.lon_e7;
        hover_level(pose, limits, dt);
    } else if limits.calm() {
        let travel = ground.min(limits.speed_ms * dt);
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
        pose.air_cms = None;
        pose.yaw_cdeg = None;
    } else {
        let (de, dn) = (east / ground, north / ground);
        let (we, wn) = (limits.wind_east_ms, limits.wind_north_ms);
        match ground_speed_along(de, dn, we, wn, limits.speed_ms) {
            Some(fastest) => {
                // Slow down for the last stretch rather than overshoot.
                let speed = fastest.min(ground / dt);
                let (lat, lon) =
                    displace(pose.lat_e7, pose.lon_e7, de * speed * dt, dn * speed * dt);
                pose.lat_e7 = lat;
                pose.lon_e7 = lon;
                pose.hdg_cdeg = heading_cdeg(east, north);
                pose.spd_cms = libm::round(speed * 100.0) as i32;
                // Air velocity is ground velocity less the wind.
                set_air(pose, de * speed - we, dn * speed - wn);
            }
            None => blown(pose, limits, dt),
        }
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

/// Stay put, reporting zero motion. For anything on the ground.
pub fn hold(pose: &mut Pose) {
    pose.stop();
}

/// Hover where it is, in the air: in wind it flies into it to stay put, and
/// in a wind stronger than it can fly it drifts downwind.
pub fn hover(pose: &mut Pose, limits: Limits, dt: f64) {
    hover_level(pose, limits, dt);
    pose.vs_cms = 0;
}

fn hover_level(pose: &mut Pose, limits: Limits, dt: f64) {
    if limits.calm() {
        pose.stop();
        return;
    }
    let (we, wn) = (limits.wind_east_ms, limits.wind_north_ms);
    if we * we + wn * wn <= limits.speed_ms * limits.speed_ms {
        pose.spd_cms = 0;
        set_air(pose, -we, -wn);
    } else {
        blown(pose, limits, dt);
    }
}

/// More wind than it can fly: full airspeed straight into it, and the rest
/// carries it downwind.
fn blown(pose: &mut Pose, limits: Limits, dt: f64) {
    let (we, wn) = (limits.wind_east_ms, limits.wind_north_ms);
    let wind = libm::sqrt(we * we + wn * wn);
    let (ae, an) = (-we / wind * limits.speed_ms, -wn / wind * limits.speed_ms);
    let (ge, gn) = (we + ae, wn + an);
    let (lat, lon) = displace(pose.lat_e7, pose.lon_e7, ge * dt, gn * dt);
    pose.lat_e7 = lat;
    pose.lon_e7 = lon;
    pose.hdg_cdeg = heading_cdeg(ge, gn);
    pose.spd_cms = libm::round(libm::sqrt(ge * ge + gn * gn) * 100.0) as i32;
    set_air(pose, ae, an);
}

fn set_air(pose: &mut Pose, east: f64, north: f64) {
    let air = libm::sqrt(east * east + north * north);
    pose.air_cms = Some(libm::round(air * 100.0) as i32);
    pose.yaw_cdeg = Some(if air > 1e-6 {
        heading_cdeg(east, north)
    } else {
        pose.hdg_cdeg
    });
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

    const LIMITS: Limits = Limits::still(10.0, 3.0);

    fn windy(east: f64, north: f64) -> Limits {
        Limits {
            wind_east_ms: east,
            wind_north_ms: north,
            ..LIMITS
        }
    }

    fn north_of(pose: &Pose, metres: f64) -> Point {
        let (lat, lon) = displace(pose.lat_e7, pose.lon_e7, 0.0, metres);
        Point {
            lat_e7: lat,
            lon_e7: lon,
            alt_mm: pose.alt_mm,
        }
    }

    fn ticks_to(pose: &mut Pose, target: &Point, limits: Limits) -> u32 {
        let mut ticks = 0;
        while !step_toward(pose, target, limits, 0.1).within(0.05, 0.05) {
            ticks += 1;
            assert!(ticks < 100_000, "never arrived");
        }
        ticks + 1
    }

    #[test]
    fn a_headwind_slows_it_and_a_tailwind_speeds_it_up() {
        let start = Pose::at(23.9, 54.9, 50.0);
        let mut p = start;
        let target = north_of(&p, 100.0);
        // 10 m/s airspeed: 5 m/s over the ground into 5 m/s, 15 with it.
        assert!((199..=201).contains(&ticks_to(&mut p, &target, windy(0.0, -5.0))));
        let mut p = start;
        assert!((66..=68).contains(&ticks_to(&mut p, &target, windy(0.0, 5.0))));
        // Into the wind it flies at full airspeed.
        let mut p = start;
        step_toward(&mut p, &target, windy(0.0, -5.0), 0.1);
        assert_eq!(p.air_cms, Some(1_000));
        assert_eq!(p.spd_cms, 500);
    }

    #[test]
    fn in_a_crosswind_it_crabs_and_holds_its_track() {
        let mut p = Pose::at(23.9, 54.9, 50.0);
        let target = north_of(&p, 100.0);
        // 6 m/s from the west: it points 36.87° left of north and makes 8 m/s.
        step_toward(&mut p, &target, windy(6.0, 0.0), 0.1);
        assert_eq!(p.spd_cms, 800);
        assert_eq!(p.hdg_cdeg, 0);
        assert_eq!(p.yaw(), 36_000 - 3_687);
        assert_eq!(p.air_cms, Some(1_000));
        let ticks = ticks_to(&mut p, &target, windy(6.0, 0.0));
        assert!((123..=126).contains(&ticks), "{ticks}");
        assert_eq!(p.lon_e7, Pose::at(23.9, 54.9, 50.0).lon_e7);
    }

    #[test]
    fn a_wind_stronger_than_it_flies_blows_it_back() {
        let mut p = Pose::at(23.9, 54.9, 50.0);
        let start = p.point();
        let target = north_of(&p, 100.0);
        for _ in 0..10 {
            step_toward(&mut p, &target, windy(0.0, -15.0), 0.1);
        }
        // 5 m/s backwards for a second.
        let back = ground_distance_m(&start, &p.point());
        assert!((back - 5.0).abs() < 0.05, "{back}");
        assert!(p.lat_e7 < start.lat_e7);
        assert_eq!(p.yaw(), 0);
    }

    #[test]
    fn hovering_in_wind_costs_airspeed_but_not_position() {
        let mut p = Pose::at(23.9, 54.9, 50.0);
        let start = p;
        hover(&mut p, windy(3.0, 4.0), 0.1);
        assert_eq!((p.lat_e7, p.lon_e7), (start.lat_e7, start.lon_e7));
        assert_eq!(p.spd_cms, 0);
        assert_eq!(p.air_cms, Some(500));
        hover(&mut p, LIMITS, 0.1);
        assert_eq!(p.air_cms, None);
        assert_eq!(p.airspeed_ms(), 0.0);
    }

    #[test]
    fn still_air_moves_exactly_as_before_wind_existed() {
        let mut a = Pose::at(23.9, 54.9, 50.0);
        let target = north_of(&a, 37.0).with_alt_mm(60_000);
        let mut b = a;
        for _ in 0..50 {
            step_toward(&mut a, &target, LIMITS, 0.1);
            step_toward(&mut b, &target, windy(0.0, 0.0), 0.1);
        }
        assert_eq!(a, b);
        assert_eq!(a.air_cms, None);
    }

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
