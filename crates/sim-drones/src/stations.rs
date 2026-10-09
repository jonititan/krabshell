//! Launch and recovery stations, and planning trips too long for one charge.
//!
//! A [`Station`] is a pad with a charger, on the ground, that any drone may
//! land on, charge at and take off from again. A pack lists those it starts
//! with in `world/stations.json`, each `{"name", "lat", "lon"}`; while it
//! runs, intents add and remove them (see [`crate::codes`]). A drone's own
//! home pad is always a station for it.
//!
//! A drone the simulation flies plans each trip ([`Planner`]) with what one
//! charge will carry it: flight at cruise speed, the climbs and descents of
//! each hop, and any work at the far end (a winch cycle, a survey), all with
//! a margin for detours ([`SAFETY`]). A trip that is too far goes by way of
//! stations: the shortest chain of hops from home to a station from which
//! the far end and back to a station is in reach, and on home from there.
//! At each stop the drone lands, charges until it has enough for the next
//! hop and the reserve, and takes off. With no stations, or no charger, a
//! drone flies straight there as it always has.

use serde::{Deserialize, Serialize};

use crate::dispatch::DispatchSettings;
use crate::flight::{self, Point};
use crate::mission::{MissionItem, Tuning, NAV_LAND, NAV_TAKEOFF, NAV_WAYPOINT};

const FRAME_TERRAIN: u8 = 11;

/// How near a station a drone coming in to land on it takes one of its
/// slots, or holds off until one is free, metres.
pub const APPROACH_M: f64 = 50.0;

/// Is the drone flying a mission whose current item lands at `at`?
pub fn landing_on(drone: &crate::mission::Drone, at: &Point) -> bool {
    drone.mode == crate::mission::Mode::Auto
        && drone
            .mission
            .get(usize::from(drone.mission_seq))
            .is_some_and(|i| {
                i.command == NAV_LAND
                    && flight::ground_distance_m(
                        &Point {
                            lat_e7: i.x,
                            lon_e7: i.y,
                            alt_mm: 0,
                        },
                        at,
                    ) <= crate::mission::PAD_RADIUS_M
            })
}

/// How much more than the straight-line estimate a hop is allowed to need.
pub const SAFETY: f64 = 1.25;

/// A full charge, parts per million.
const FULL: f64 = 1_000_000.0;

/// A pad with a charger that any drone may use.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Station {
    pub id: u32,
    #[serde(default, skip_serializing_if = "String::is_empty")]
    pub name: String,
    pub lat_e7: i32,
    pub lon_e7: i32,
    /// Launch and recovery slots: how many drones can be down on it (or
    /// coming down onto it) at once.
    pub slots: u32,
    /// Who added it: `scenario`, `dashboard` or `intent`.
    pub source: String,
    pub added_tick: u64,
}

impl Station {
    pub fn point(&self) -> Point {
        Point {
            lat_e7: self.lat_e7,
            lon_e7: self.lon_e7,
            alt_mm: 0,
        }
    }
}

/// Every station in a world. Checkpointed through the model's fields.
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct Stations {
    pub list: Vec<Station>,
    pub next_id: u32,
    /// Bumped whenever a station is added or removed.
    #[serde(default)]
    pub rev: u64,
}

impl Stations {
    pub fn is_empty(&self) -> bool {
        self.list.is_empty()
    }

    pub fn take_id(&mut self) -> u32 {
        self.next_id = self.next_id.max(1);
        let id = self.next_id;
        self.next_id += 1;
        id
    }

    pub fn add(&mut self, station: Station) {
        self.list.push(station);
        self.list.sort_by_key(|s| s.id);
        self.rev += 1;
    }

    pub fn remove(&mut self, id: u32) -> Option<Station> {
        let at = self.list.iter().position(|s| s.id == id)?;
        self.rev += 1;
        Some(self.list.remove(at))
    }

    /// The station whose pad `point` is on, if any.
    pub fn at(&self, point: &Point) -> Option<&Station> {
        self.list
            .iter()
            .find(|s| flight::ground_distance_m(point, &s.point()) <= crate::mission::PAD_RADIUS_M)
    }
}

/// How far a charge goes, from the pack's flight and battery settings.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Range {
    /// Charge used per metre flown level at cruise, ppm.
    per_m: f64,
    /// Charge used per metre climbed or descended, ppm.
    per_m_up: f64,
    /// Charge used per second hovering, ppm.
    per_s: f64,
    reserve: f64,
    /// Stations charge drones at all.
    charging: bool,
}

impl Range {
    pub fn new(tuning: &Tuning, cruise_cms: i32) -> Range {
        let per_tick = f64::from(tuning.drain_ppm_tick);
        let per_s = per_tick / tuning.dt;
        Range {
            per_m: per_s / (f64::from(cruise_cms.max(1)) / 100.0),
            per_m_up: per_s / tuning.climb_ms,
            per_s,
            reserve: f64::from(tuning.reserve_ppm),
            charging: tuning.charge_ppm_tick > 0,
        }
    }

    /// The battery never runs down: anywhere is in reach.
    pub fn endless(&self) -> bool {
        self.per_s <= 0.0
    }

    /// Charge one hop needs: `metres` level, up `climb_m` at the start and
    /// down again at the end, and `work_s` hovering, with the margin.
    pub fn need(&self, metres: f64, climb_m: f64, work_s: f64) -> f64 {
        SAFETY * (metres * self.per_m + 2.0 * climb_m * self.per_m_up + work_s * self.per_s)
    }

    /// The furthest a hop can go level, metres, starting with `charge`,
    /// climbing `climb_m` and doing `work_s` at the far end, the reserve
    /// kept and the margin allowed. Infinite if the battery never runs down.
    pub fn reach_m(&self, charge: f64, climb_m: f64, work_s: f64) -> f64 {
        if self.endless() {
            return f64::INFINITY;
        }
        let left =
            (charge - self.reserve) / SAFETY - 2.0 * climb_m * self.per_m_up - work_s * self.per_s;
        (left / self.per_m).max(0.0)
    }

    /// Can a hop needing `need` be flown starting with `charge`?
    fn fits(&self, need: f64, charge: f64) -> bool {
        need + self.reserve <= charge
    }

    /// The charge to leave a stop with for a hop needing `need`, percent.
    pub fn leave_pct(&self, need: f64) -> f32 {
        let pct = ((need + self.reserve) / 10_000.0).ceil();
        pct.clamp(1.0, 100.0) as f32
    }
}

/// One trip: out to a target and home, with the stations to stop at.
#[derive(Debug, Clone, PartialEq)]
pub struct Trip {
    /// Stops on the way out, in order, each with the charge (percent) to
    /// leave it with.
    pub out: Vec<(Point, f32)>,
    /// Stops on the way back, likewise.
    pub back: Vec<(Point, f32)>,
}

impl Trip {
    pub fn direct() -> Trip {
        Trip {
            out: Vec::new(),
            back: Vec::new(),
        }
    }

    pub fn is_direct(&self) -> bool {
        self.out.is_empty() && self.back.is_empty()
    }
}

/// Plans trips for one drone, from where it is, with what it has.
pub struct Planner<'a> {
    pub range: Range,
    /// Stations it may stop at.
    pub stations: Vec<Point>,
    /// How high it climbs on each hop, metres.
    pub climb_m: f64,
    pub home: &'a Point,
    /// What it has now, ppm.
    pub charge: f64,
}

impl Planner<'_> {
    /// The way from home out to `target`, `work_s` there, and back home, or
    /// `None` if it cannot be done. Straight there and back when one charge
    /// does it, or when there are no stations to stop at (the drone tries
    /// anyway, as it always has), so `direct_anyway` says whether a trip
    /// out of reach is still flown straight.
    pub fn trip(&self, target: &Point, work_s: f64, direct_anyway: bool) -> Option<Trip> {
        let r = &self.range;
        let there_and_back = 2.0 * flight::ground_distance_m(self.home, target);
        if r.endless() || r.fits(r.need(there_and_back, self.climb_m, work_s), self.charge) {
            return Some(Trip::direct());
        }
        if self.stations.is_empty() || !r.charging {
            return direct_anyway.then(Trip::direct);
        }
        let Chains { nodes, out, back } = self.chains();
        let n = nodes.len();
        let d = |a: usize, b: usize| flight::ground_distance_m(&nodes[a], &nodes[b]);
        let cap = |a: usize| self.cap(a);
        let mut best: Option<(f64, usize, usize)> = None;
        for c1 in 0..n {
            let Some(to_c1) = out.0[c1] else { continue };
            for c2 in 0..n {
                let Some(from_c2) = back.0[c2] else { continue };
                let reach = flight::ground_distance_m(&nodes[c1], target)
                    + flight::ground_distance_m(target, &nodes[c2]);
                if !r.fits(r.need(reach, self.climb_m, work_s), cap(c1)) {
                    continue;
                }
                let total = to_c1 + reach + from_c2;
                if best.is_none_or(|(b, ..)| total < b) {
                    best = Some((total, c1, c2));
                }
            }
        }
        let Some((_, c1, c2)) = best else {
            return direct_anyway.then(Trip::direct);
        };
        // The stops, each with the charge for the hop that leaves it.
        let mut out_path = path(&out.1, c1);
        out_path.retain(|i| *i != 0);
        let mut back_path = path(&back.1, c2);
        back_path.reverse();
        back_path.retain(|i| *i != 0);
        let leave = |metres: f64, work: f64| r.leave_pct(r.need(metres, self.climb_m, work));
        let mut trip = Trip::direct();
        for (k, &s) in out_path.iter().enumerate() {
            let pct = match out_path.get(k + 1) {
                Some(&next) => leave(d(s, next), 0.0),
                None => leave(
                    flight::ground_distance_m(&nodes[s], target)
                        + flight::ground_distance_m(target, &nodes[c2]),
                    work_s,
                ),
            };
            trip.out.push((nodes[s], pct));
        }
        for (k, &s) in back_path.iter().enumerate() {
            let next = back_path.get(k + 1).copied().unwrap_or(0);
            trip.back.push((nodes[s], leave(d(s, next), 0.0)));
        }
        Some(trip)
    }

    /// What a node of [`Planner::chains`] starts a hop with: what the drone
    /// has at home (node 0), a full charge at a station.
    fn cap(&self, node: usize) -> f64 {
        if node == 0 {
            self.charge
        } else {
            FULL
        }
    }

    /// Home (node 0) and the stations, with the shortest chains of hops
    /// out from home to each, and (on the reverse graph) back home from
    /// each. On the way back home is only ever the end.
    fn chains(&self) -> Chains {
        let r = &self.range;
        let mut nodes = vec![*self.home];
        nodes.extend(self.stations.iter().copied());
        let n = nodes.len();
        let d = |a: usize, b: usize| flight::ground_distance_m(&nodes[a], &nodes[b]);
        let hop = |a: usize, b: usize| r.fits(r.need(d(a, b), self.climb_m, 0.0), self.cap(a));
        let out = shortest(n, 0, |a, b| hop(a, b).then(|| d(a, b)));
        let back = shortest(n, 0, |a, b| (b != 0 && hop(b, a)).then(|| d(b, a)));
        Chains { nodes, out, back }
    }

    /// Everywhere a trip from home can do `work_s` of work and come back,
    /// by way of stations as [`Planner::trip`] would: the union of the
    /// [`Area`]s, one for each station (or home) it can get out to and each
    /// it can get home from, within one charge between them. `None` if
    /// anywhere can be reached.
    pub fn extent(&self, work_s: f64) -> Option<Vec<Area>> {
        let r = &self.range;
        if r.endless() {
            return None;
        }
        let stations = if r.charging { self.stations.len() } else { 0 };
        let planner = Planner {
            stations: self.stations[..stations].to_vec(),
            home: self.home,
            ..*self
        };
        let Chains { nodes, out, back } = planner.chains();
        let mut areas = Vec::new();
        for c1 in (0..nodes.len()).filter(|c| out.0[*c].is_some()) {
            let reach = r.reach_m(planner.cap(c1), self.climb_m, work_s);
            for c2 in (0..nodes.len()).filter(|c| back.0[*c].is_some()) {
                if flight::ground_distance_m(&nodes[c1], &nodes[c2]) <= reach {
                    areas.push(Area {
                        a: nodes[c1],
                        b: nodes[c2],
                        sum_m: reach,
                    });
                }
            }
        }
        Some(areas)
    }

    /// The way home from where it stands, by way of stations if one charge
    /// will not do (leaving each with enough for the hop after). `None` if
    /// home is out of reach even so.
    pub fn home_from(&self, here: &Point) -> Option<Vec<(Point, f32)>> {
        let r = &self.range;
        let direct = flight::ground_distance_m(here, self.home);
        if r.endless() || r.fits(r.need(direct, self.climb_m, 0.0), self.charge) {
            return Some(Vec::new());
        }
        if !r.charging {
            return None;
        }
        // Node 0 is home, 1 is here, the rest the stations.
        let mut nodes = vec![*self.home, *here];
        nodes.extend(self.stations.iter().copied());
        let n = nodes.len();
        let d = |a: usize, b: usize| flight::ground_distance_m(&nodes[a], &nodes[b]);
        let cap = |a: usize| if a == 1 { self.charge } else { FULL };
        let (dist, prev) = shortest(n, 1, |a, b| {
            (a != 0 && r.fits(r.need(d(a, b), self.climb_m, 0.0), cap(a))).then(|| d(a, b))
        });
        dist[0]?;
        let mut stops = path(&prev, 0);
        stops.retain(|i| *i > 1);
        Some(
            stops
                .iter()
                .enumerate()
                .map(|(k, &s)| {
                    let next = stops.get(k + 1).copied().unwrap_or(0);
                    (nodes[s], r.leave_pct(r.need(d(s, next), self.climb_m, 0.0)))
                })
                .collect(),
        )
    }

    /// The nearest station it can reach from `here` on what it has.
    pub fn nearest_in_reach(&self, here: &Point) -> Option<Point> {
        let r = &self.range;
        self.stations
            .iter()
            .map(|s| (flight::ground_distance_m(here, s), *s))
            .filter(|(d, _)| r.need(*d, 0.0, 0.0) <= self.charge)
            .min_by(|a, b| a.0.total_cmp(&b.0))
            .map(|(_, s)| s)
    }

    /// Can it get home from `here` on what it has, down to empty?
    pub fn home_in_reach(&self, here: &Point) -> bool {
        let d = flight::ground_distance_m(here, self.home);
        self.range.endless() || self.range.need(d, 0.0, 0.0) <= self.charge
    }
}

/// See [`Planner::chains`].
struct Chains {
    nodes: Vec<Point>,
    out: (Vec<Option<f64>>, Vec<usize>),
    back: (Vec<Option<f64>>, Vec<usize>),
}

/// Somewhere a trip can reach: every point whose distances to `a` and to
/// `b` add up to no more than `sum_m`, an ellipse with them as its foci
/// (a circle when they are the same). It sets out from `a` and comes back
/// to `b`.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Area {
    pub a: Point,
    pub b: Point,
    pub sum_m: f64,
}

impl Area {
    pub fn contains(&self, p: &Point) -> bool {
        flight::ground_distance_m(&self.a, p) + flight::ground_distance_m(p, &self.b) <= self.sum_m
    }

    /// The furthest any point of the ellipse is from `from`, metres:
    /// from its centre, out along its long axis or across it.
    pub fn farthest_m(&self, from: &Point) -> f64 {
        let (bx, by) = flight::offset_m(self.a.lat_e7, self.a.lon_e7, self.b.lat_e7, self.b.lon_e7);
        let (fx, fy) = flight::offset_m(self.a.lat_e7, self.a.lon_e7, from.lat_e7, from.lon_e7);
        let (cx, cy) = (bx / 2.0, by / 2.0);
        let half = self.sum_m / 2.0;
        let focal = libm::sqrt(bx * bx + by * by) / 2.0;
        let minor = libm::sqrt((half * half - focal * focal).max(0.0));
        let (ux, uy) = if focal > 1e-9 {
            (bx / (2.0 * focal), by / (2.0 * focal))
        } else {
            (1.0, 0.0)
        };
        (0..72)
            .map(|i| {
                let t = i as f64 / 72.0 * core::f64::consts::TAU;
                let (u, v) = (half * libm::cos(t), minor * libm::sin(t));
                let (x, y) = (cx + u * ux - v * uy, cy + u * uy + v * ux);
                libm::sqrt((x - fx) * (x - fx) + (y - fy) * (y - fy))
            })
            .fold(0.0, f64::max)
    }
}

/// Dijkstra over `n` nodes from `start`, with `edge(a, b)` the length of
/// the hop from `a` to `b` if it can be flown. Distances and predecessors.
fn shortest(
    n: usize,
    start: usize,
    edge: impl Fn(usize, usize) -> Option<f64>,
) -> (Vec<Option<f64>>, Vec<usize>) {
    let mut best: Vec<Option<f64>> = vec![None; n];
    let mut prev = vec![usize::MAX; n];
    let mut done = vec![false; n];
    best[start] = Some(0.0);
    while let Some(u) = (0..n)
        .filter(|&i| !done[i] && best[i].is_some())
        .min_by(|&i, &j| best[i].unwrap().total_cmp(&best[j].unwrap()))
    {
        done[u] = true;
        let at = best[u].expect("filtered");
        for v in 0..n {
            if done[v] || v == u {
                continue;
            }
            if let Some(len) = edge(u, v) {
                if best[v].is_none_or(|b| at + len < b) {
                    best[v] = Some(at + len);
                    prev[v] = u;
                }
            }
        }
    }
    (best, prev)
}

/// The nodes from the start to `to`, start included.
fn path(prev: &[usize], to: usize) -> Vec<usize> {
    let mut out = vec![to];
    let mut at = to;
    while prev[at] != usize::MAX {
        at = prev[at];
        out.push(at);
    }
    out.reverse();
    out
}

fn item(command: u16, frame: u8, at: &Point, z: f32) -> MissionItem {
    MissionItem {
        seq: 0,
        frame,
        command,
        autocontinue: 1,
        param1: 0.0,
        param2: 0.0,
        param3: 0.0,
        param4: f32::NAN,
        x: at.lat_e7,
        y: at.lon_e7,
        z,
    }
}

/// Mission items to stop at a station on the way: fly there at the cruise
/// height, land, charge until it holds `leave_pct`, and take off again.
pub fn stop(
    at: &Point,
    leave_pct: f32,
    cruise_agl_m: f32,
    ds: &DispatchSettings,
) -> [MissionItem; 3] {
    let there = item(NAV_WAYPOINT, FRAME_TERRAIN, at, cruise_agl_m);
    let land = item(NAV_LAND, FRAME_TERRAIN, at, 0.0);
    let mut takeoff = item(NAV_TAKEOFF, FRAME_TERRAIN, &Point::default(), ds.takeoff_m);
    takeoff.param2 = leave_pct;
    [there, land, takeoff]
}

/// Number a mission's items in order.
pub fn renumber(items: &mut [MissionItem]) {
    for (i, item) in items.iter_mut().enumerate() {
        item.seq = i as u16;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn tuning() -> Tuning {
        // 10 m/s, 0.1 %/s: 1000 s of flight, 10 km; 20 % reserve.
        Tuning {
            dt: 0.1,
            dt_ms: 100,
            climb_ms: 3.0,
            takeoff_alt_mm: 20_000,
            rtl_alt_mm: 30_000,
            drain_ppm_tick: 100,
            reserve_ppm: 200_000,
            charge_ppm_tick: 50,
            launch_ppm: 800_000,
            winch: crate::mission::Winch {
                drop_ms: 2.0,
                slow_ms: 0.5,
                slow_m: 3.0,
                release_s: 5.0,
                reel_ms: 2.5,
                lock_s: 3.0,
            },
        }
    }

    fn east(m: f64) -> Point {
        let (lat, lon) = flight::displace(549_000_000, 239_000_000, m, 0.0);
        Point {
            lat_e7: lat,
            lon_e7: lon,
            alt_mm: 0,
        }
    }

    fn planner<'a>(home: &'a Point, stations: Vec<Point>, charge: f64) -> Planner<'a> {
        Planner {
            range: Range::new(&tuning(), 1_000),
            stations,
            climb_m: 0.0,
            home,
            charge,
        }
    }

    #[test]
    fn a_trip_in_reach_is_direct_and_one_too_far_stops_at_stations_both_ways() {
        let home = east(0.0);
        let p = planner(&home, vec![east(3_000.0), east(-4_000.0)], FULL);
        // 1.25 × 2 × 2 km = 5 km of the 8 km above the reserve.
        assert_eq!(p.trip(&east(2_000.0), 0.0, false), Some(Trip::direct()));
        // 6 km out: 15 km there and back. Via the station at 3 km: 3.75 km
        // to it, then 3 km on and 3 km back with the margin, 7.5 km.
        let trip = p.trip(&east(6_000.0), 0.0, false).unwrap();
        assert_eq!(trip.out.len(), 1);
        assert_eq!(trip.out[0].0, east(3_000.0));
        assert_eq!(trip.back.len(), 1);
        assert!(trip.out[0].1 > 90.0 && trip.out[0].1 <= 100.0);
        // Out of reach even so: refused, or flown straight if asked.
        assert_eq!(p.trip(&east(20_000.0), 0.0, false), None);
        assert_eq!(p.trip(&east(20_000.0), 0.0, true), Some(Trip::direct()));
        // No stations: straight, as before.
        let bare = planner(&home, Vec::new(), FULL);
        assert_eq!(bare.trip(&east(6_000.0), 0.0, true), Some(Trip::direct()));
    }

    #[test]
    fn a_chain_of_stations_carries_a_drone_further_than_any_one() {
        let home = east(0.0);
        let p = planner(&home, vec![east(5_000.0), east(10_000.0)], FULL);
        let trip = p.trip(&east(13_000.0), 0.0, false).unwrap();
        let stops: Vec<Point> = trip.out.iter().map(|s| s.0).collect();
        assert_eq!(stops, vec![east(5_000.0), east(10_000.0)]);
        let back: Vec<Point> = trip.back.iter().map(|s| s.0).collect();
        assert_eq!(back, vec![east(10_000.0), east(5_000.0)]);
        // And home from the far one, charging there first: one stop.
        let way = p.home_from(&east(10_000.0)).unwrap();
        assert_eq!(
            way.iter().map(|s| s.0).collect::<Vec<_>>(),
            vec![east(5_000.0)]
        );
        // Too little to reach any station from out in the open.
        let flat = planner(&home, p.stations.clone(), 250_000.0);
        assert_eq!(flat.home_from(&east(12_000.0)), None);
    }
}
