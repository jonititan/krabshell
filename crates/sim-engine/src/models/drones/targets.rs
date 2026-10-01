//! Things for drones to find: people and vehicles that wander, intruder drones
//! that fly a fixed circuit, and points of interest that stay put.

use serde::{Deserialize, Serialize};

use super::flight::{self, Limits, Point, Pose};

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
        let limits = Limits {
            speed_ms: f64::from(self.speed_cms) * 0.01,
            climb_ms: 0.0,
        };
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
        let limits = Limits {
            speed_ms: f64::from(self.speed_cms) * 0.01,
            climb_ms: 3.0,
        };
        let left = flight::step_toward(pose, &target, limits, dt);
        if left.within(1.0, 1.0) {
            self.leg = ((usize::from(self.leg) + 1) % self.waypoints.len()) as u16;
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
