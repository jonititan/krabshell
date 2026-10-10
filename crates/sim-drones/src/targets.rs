//! Things for drones to find: people and vehicles that wander, intruder drones
//! that fly a fixed circuit, and points of interest that stay put. Also the
//! manned aircraft that share the airspace, flying published routes.

use serde::{Deserialize, Serialize};

use crate::flight::{self, Limits, Point, Pose};

/// What sort of vehicle a vehicle is. Each is one of APP-6's civilian
/// vehicles, so a map can draw it as one.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum VehicleType {
    Car,
    Van,
    Pickup,
    Bus,
    Lorry,
}

impl VehicleType {
    pub fn name(self) -> &'static str {
        match self {
            VehicleType::Car => "car",
            VehicleType::Van => "van",
            VehicleType::Pickup => "pickup",
            VehicleType::Bus => "bus",
            VehicleType::Lorry => "lorry",
        }
    }

    pub fn parse(name: &str) -> Option<VehicleType> {
        Some(match name {
            "car" => VehicleType::Car,
            "van" => VehicleType::Van,
            "pickup" => VehicleType::Pickup,
            "bus" => VehicleType::Bus,
            "lorry" => VehicleType::Lorry,
            _ => return None,
        })
    }

    /// One drawn from city traffic by a uniform number: mostly cars, then
    /// vans, pickups, lorries and buses.
    pub fn draw(u: f64) -> VehicleType {
        match u {
            u if u < 0.70 => VehicleType::Car,
            u if u < 0.84 => VehicleType::Van,
            u if u < 0.92 => VehicleType::Pickup,
            u if u < 0.97 => VehicleType::Lorry,
            _ => VehicleType::Bus,
        }
    }
}

/// A ground target that walks or drives between random destinations near
/// where it started.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Wander {
    pub anchor: Point,
    pub radius_m: u32,
    pub speed_cms: i32,
    pub dest: Point,
    /// Milliseconds to wait at the destination before choosing another.
    #[serde(default)]
    pub pause_ms: u32,
}

impl Wander {
    /// Move one tick. `draw` supplies two uniform numbers for choosing a new
    /// destination; `accepts` says whether a point is inside the world.
    pub fn step(
        &mut self,
        pose: &mut Pose,
        dt: f64,
        dt_ms: u32,
        draw: (f64, f64),
        accepts: impl Fn(&Point) -> bool,
    ) {
        if self.pause_ms > 0 {
            self.pause_ms = self.pause_ms.saturating_sub(dt_ms);
            flight::hold(pose);
            return;
        }
        let limits = Limits::still(f64::from(self.speed_cms) * 0.01, 0.0);
        let left = flight::step_toward(pose, &self.dest, limits, dt);
        if left.ground_m > 0.5 {
            return;
        }
        // Arrived: linger a moment, then pick somewhere else within the radius.
        self.pause_ms = (draw.0 * 20_000.0) as u32;
        let angle = draw.0 * core::f64::consts::TAU;
        let reach = libm::sqrt(draw.1) * f64::from(self.radius_m);
        let (lat, lon) = flight::displace(
            self.anchor.lat_e7,
            self.anchor.lon_e7,
            libm::sin(angle) * reach,
            libm::cos(angle) * reach,
        );
        let next = Point {
            lat_e7: lat,
            lon_e7: lon,
            alt_mm: 0,
        };
        // A destination outside the world is skipped; the next arrival tries again.
        self.dest = if accepts(&next) { next } else { self.anchor };
    }
}

/// An uncontrolled drone flying a closed circuit.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Route {
    pub waypoints: Vec<Point>,
    pub leg: u16,
    pub speed_cms: i32,
}

impl Route {
    pub fn step(&mut self, pose: &mut Pose, dt: f64) {
        let Some(target) = self.waypoints.get(usize::from(self.leg)).copied() else {
            flight::hold(pose);
            return;
        };
        // Intruders fly scripted routes at a set ground speed, wind or not.
        let limits = Limits::still(f64::from(self.speed_cms) * 0.01, 3.0);
        let left = flight::step_toward(pose, &target, limits, dt);
        if left.within(1.0, 1.0) {
            self.leg = ((usize::from(self.leg) + 1) % self.waypoints.len()) as u16;
        }
    }
}

/// A point on a manned aircraft's route: where, at what altitude above mean
/// sea level, and the airspeed to be flying by the time it gets there.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub struct Fix {
    #[serde(flatten)]
    pub at: Point,
    pub speed_cms: i32,
}

/// A manned aircraft flying a route from its first fix to its last: an
/// arrival down the approach, say, or a departure climbing out.
///
/// Altitude changes evenly along each leg, so a constant gradient (a 3°
/// glide path) is a straight leg. Airspeed changes evenly along each leg
/// too. It flies that airspeed through the wind and holds the track, as
/// [`flight::step_toward`] does. It turns at once at each fix: a few seconds
/// early or late at airliner speeds, which is well inside a conflict
/// horizon, but not a flown turn.
///
/// Before its first start and between flights it is parked out of sight at
/// its first fix: not drawn, not seen and not assessed for conflicts.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Flight {
    pub fixes: Vec<Fix>,
    /// The fix it is flying to. Meaningless while it is parked.
    pub leg: u16,
    /// In the air on its route.
    pub active: bool,
    /// The tick it next starts at; `u64::MAX` once it has none.
    pub next_start: u64,
    /// Ticks from one start to the next; 0 flies the route once.
    #[serde(default)]
    pub repeat_ticks: u64,
}

impl Flight {
    /// One tick. `wind` is toward the east and north, m/s, where it is.
    pub fn step(&mut self, pose: &mut Pose, tick: u64, dt: f64, wind: (f64, f64)) {
        if !self.active {
            if tick < self.next_start || self.fixes.len() < 2 {
                flight::hold(pose);
                return;
            }
            self.active = true;
            self.leg = 1;
            // Starts keep to the timetable, however long a flight took.
            self.next_start = match self.repeat_ticks {
                0 => u64::MAX,
                every => self.next_start.saturating_add(every),
            };
            let first = self.fixes[0];
            *pose = Pose {
                lat_e7: first.at.lat_e7,
                lon_e7: first.at.lon_e7,
                alt_mm: first.at.alt_mm,
                spd_cms: first.speed_cms,
                ..Pose::default()
            };
            return;
        }

        let from = self.fixes[usize::from(self.leg) - 1];
        let to = self.fixes[usize::from(self.leg)];
        let leg_m = flight::ground_distance_m(&from.at, &to.at);
        let along = |left_m: f64| {
            if leg_m > 0.0 {
                (1.0 - left_m / leg_m).clamp(0.0, 1.0)
            } else {
                1.0
            }
        };
        let lerp = |a: i32, b: i32, t: f64| f64::from(a) + (f64::from(b) - f64::from(a)) * t;

        let done = along(flight::ground_distance_m(&pose.point(), &to.at));
        let limits = Limits {
            speed_ms: lerp(from.speed_cms, to.speed_cms, done).max(1.0) * 0.01,
            climb_ms: 0.0,
            wind_east_ms: wind.0,
            wind_north_ms: wind.1,
        };
        // Horizontal only; the altitude follows the leg's gradient below.
        let left = flight::step_toward(pose, &to.at.with_alt_mm(pose.alt_mm), limits, dt);
        let alt_mm = libm::round(lerp(from.at.alt_mm, to.at.alt_mm, along(left.ground_m))) as i32;
        pose.vs_cms = libm::round(f64::from(alt_mm - pose.alt_mm) * 0.1 / dt) as i32;
        pose.alt_mm = alt_mm;

        if left.ground_m <= 1.0 {
            self.leg += 1;
            if usize::from(self.leg) >= self.fixes.len() {
                self.active = false;
                self.leg = 0;
                flight::hold(pose);
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_route_loops_back_to_its_first_waypoint() {
        let start = Pose::at(0.0, 0.0, 50.0);
        let (lat, lon) = flight::displace(start.lat_e7, start.lon_e7, 20.0, 0.0);
        let mut route = Route {
            waypoints: vec![
                Point {
                    lat_e7: lat,
                    lon_e7: lon,
                    alt_mm: 50_000,
                },
                start.point(),
            ],
            leg: 0,
            speed_cms: 1_000,
        };
        // 20 m each way at 10 m/s: both legs are done inside 45 ticks, and the
        // second lap has not finished its first leg yet.
        let mut pose = start;
        for _ in 0..45 {
            route.step(&mut pose, 0.1);
        }
        assert_eq!(route.leg, 0, "should have completed both legs");
    }

    /// An approach: 10 km east of the threshold at 600 m, down a straight
    /// leg to 50 m, slowing from 80 to 70 m/s.
    fn approach(start_tick: u64, repeat_ticks: u64) -> (Flight, Point) {
        let threshold = Pose::at(24.0, 55.0, 50.0).point();
        let (lat, lon) = flight::displace(threshold.lat_e7, threshold.lon_e7, 10_000.0, 0.0);
        let start = Point {
            lat_e7: lat,
            lon_e7: lon,
            alt_mm: 600_000,
        };
        let flight = Flight {
            fixes: vec![
                Fix {
                    at: start,
                    speed_cms: 8_000,
                },
                Fix {
                    at: threshold,
                    speed_cms: 7_000,
                },
            ],
            leg: 0,
            active: false,
            next_start: start_tick,
            repeat_ticks,
        };
        (flight, threshold)
    }

    #[test]
    fn an_aircraft_waits_for_its_start_then_flies_a_straight_glide_path() {
        let (mut flight, threshold) = approach(50, 0);
        let mut pose = Pose::default();
        for tick in 0..50 {
            flight.step(&mut pose, tick, 0.1, (0.0, 0.0));
            assert!(!flight.active, "started early at tick {tick}");
        }
        flight.step(&mut pose, 50, 0.1, (0.0, 0.0));
        assert!(flight.active);
        assert_eq!(pose.point(), flight.fixes[0].at);

        // Altitude stays on the line from 600 m to 50 m over the 10 km.
        let mut tick = 51;
        while flight.active {
            flight.step(&mut pose, tick, 0.1, (0.0, 0.0));
            if flight.active {
                let left = flight::ground_distance_m(&pose.point(), &threshold);
                let on_path = 50.0 + 550.0 * left / 10_000.0;
                assert!(
                    (pose.alt_m() - on_path).abs() < 0.05,
                    "{:.2} m against {on_path:.2} m with {left:.0} m to go",
                    pose.alt_m()
                );
                assert!(pose.vs_cms < 0);
            }
            tick += 1;
            assert!(tick < 5_000, "never arrived");
        }
        // 10 km at 70-80 m/s is about 133 s.
        let flown_s = (tick - 51) as f64 * 0.1;
        assert!((128.0..140.0).contains(&flown_s), "{flown_s} s");
        assert_eq!(pose.alt_mm, threshold.alt_mm);
        assert_eq!(flight.next_start, u64::MAX, "flies once");
    }

    #[test]
    fn a_repeating_aircraft_keeps_its_timetable() {
        let (mut flight, _) = approach(0, 3_000);
        let mut pose = Pose::default();
        let mut starts = Vec::new();
        for tick in 0..9_500 {
            let was = flight.active;
            flight.step(&mut pose, tick, 0.1, (0.0, 0.0));
            if flight.active && !was {
                starts.push(tick);
            }
        }
        assert_eq!(starts, vec![0, 3_000, 6_000, 9_000]);
    }

    #[test]
    fn an_aircraft_flies_its_airspeed_through_the_wind_and_says_so() {
        let (mut flight, _) = approach(0, 0);
        let mut pose = Pose::default();
        // Flying west into a 20 m/s westerly (blowing toward the east).
        for tick in 0..200 {
            flight.step(&mut pose, tick, 0.1, (20.0, 0.0));
        }
        let air = f64::from(pose.air_cms.expect("airspeed in wind")) * 0.01;
        let ground = f64::from(pose.spd_cms) * 0.01;
        assert!(
            (air - ground - 20.0).abs() < 0.5,
            "air {air}, ground {ground}"
        );
        assert!((70.0..=80.0).contains(&air), "{air}");
    }

    #[test]
    fn a_wanderer_stays_within_its_radius_and_inside_the_world() {
        let anchor = Pose::at(0.0, 0.0, 0.0);
        let mut wander = Wander {
            anchor: anchor.point(),
            radius_m: 50,
            speed_cms: 500,
            dest: anchor.point(),
            pause_ms: 0,
        };
        let mut pose = anchor;
        let mut rng = sim_core::rng::Rng::new(7);
        for _ in 0..5_000 {
            wander.step(&mut pose, 0.1, 100, (rng.unit(), rng.unit()), |_| true);
            let d = flight::ground_distance_m(&pose.point(), &anchor.point());
            assert!(d <= 50.5, "strayed {d} m");
        }
        // With nothing accepted it just returns to the anchor.
        wander.dest = anchor.point();
        wander.pause_ms = 0;
        for _ in 0..200 {
            wander.step(&mut pose, 0.1, 100, (0.5, 0.5), |_| false);
        }
        assert_eq!(wander.dest, anchor.point());
    }
}
