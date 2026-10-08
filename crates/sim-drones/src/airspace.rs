//! Geofences: keep-out zones that open and close while a run goes on, and
//! routing around them.
//!
//! A [`Fence`] is a circle or a polygon, closed to drones from its floor to
//! its ceiling ([`Band`]), in metres above the ground: from the surface to
//! unlimited unless it says otherwise. A scenario pack lists the fences it
//! starts with (`world/geofences.json`);
//! while it runs, intents open and close them (see [`crate::codes`]): from
//! the dashboard, the API, or a ground station's fence plan.
//!
//! Drones route around every fence by a margin ([`Airspace::margin_m`]): each
//! fence is grown by the margin into a convex outline, and a leg that would
//! cross an outline is replaced by the shortest path around the outlines'
//! corners (a visibility graph). A drone keeps its detour until the fences
//! change or it heads somewhere else, so a new fence reroutes every drone on
//! the next tick. A drone that finds itself inside a fence (one opened around
//! it) flies straight out the nearest side; one sent to a point inside a fence
//! holds at the edge.
//!
//! A fence whose floor is high enough can be flown beneath. A leg the fence
//! blocks at the height it is flown at is also tried low enough to pass
//! under it (by [`Airspace::vertical_margin_m`], and no lower than
//! [`Airspace::min_agl_m`]), and flown that way if it is shorter, counting
//! the climb back: the drone drops to the low height where the leg starts,
//! follows the terrain under the fence, and climbs again where it ends. An
//! address under such a fence can still be delivered to; one under a fence
//! that reaches too low cannot ([`Airspace::covers`]).
//!
//! Geometry is worked in metres east and north of a fixed reference point, so
//! the same position always maps to the same coordinates, and a `gold` run
//! replays bit for bit. Polygons are flown around as their convex hull;
//! whether a drone is *inside* one is judged by the polygon itself.

use serde::{Deserialize, Serialize};

use crate::flight::{self, Point};

/// Corners of the outline a circle is flown around.
const CIRCLE_SIDES: usize = 16;

/// How far past an outline a drone that has to get out of one aims, metres.
const CLEAR_M: f64 = 0.5;

/// The shape of a fence, in degrees × 10⁷ and metres.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(tag = "shape", rename_all = "snake_case")]
pub enum Shape {
    Circle {
        lat_e7: i32,
        lon_e7: i32,
        radius_m: f64,
    },
    /// Corners in order, as `[lat_e7, lon_e7]`, not closed.
    Polygon { points: Vec<[i32; 2]> },
}

/// The heights a fence is closed between, metres above the ground.
#[derive(Debug, Clone, Copy, Default, PartialEq, Serialize, Deserialize)]
pub struct Band {
    /// From the surface if `None`.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub floor_m: Option<f64>,
    /// Unlimited if `None`.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub ceiling_m: Option<f64>,
}

impl Band {
    pub fn floor(&self) -> f64 {
        self.floor_m.unwrap_or(0.0)
    }

    pub fn ceiling(&self) -> f64 {
        self.ceiling_m.unwrap_or(f64::INFINITY)
    }

    /// Surface to unlimited.
    pub fn is_full(&self) -> bool {
        self.floor() <= 0.0 && self.ceiling_m.is_none()
    }
}

/// A keep-out zone.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Fence {
    pub id: u32,
    #[serde(flatten)]
    pub shape: Shape,
    #[serde(flatten)]
    pub band: Band,
    /// What it is, for people: "Kaunas Old Town". May be empty.
    #[serde(default, skip_serializing_if = "String::is_empty")]
    pub name: String,
    /// Who opened it: `scenario`, `dashboard`, `gcs` (a ground station's
    /// fence plan) or `intent`.
    pub source: String,
    pub opened_tick: u64,
    /// When it closes by itself, if ever.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub closes_tick: Option<u64>,
}

impl Fence {
    /// The fence as GeoJSON geometry, a circle as a polygon of 64 sides.
    pub fn geometry(&self) -> serde_json::Value {
        let mut ring: Vec<[f64; 2]> = match &self.shape {
            Shape::Circle {
                lat_e7,
                lon_e7,
                radius_m,
            } => (0..64)
                .map(|i| {
                    let a = i as f64 / 64.0 * core::f64::consts::TAU;
                    let (lat, lon) = flight::displace(
                        *lat_e7,
                        *lon_e7,
                        radius_m * libm::sin(a),
                        radius_m * libm::cos(a),
                    );
                    [deg(lon), deg(lat)]
                })
                .collect(),
            Shape::Polygon { points } => points
                .iter()
                .map(|[lat, lon]| [deg(*lon), deg(*lat)])
                .collect(),
        };
        if let Some(first) = ring.first().copied() {
            ring.push(first);
        }
        serde_json::json!({ "type": "Polygon", "coordinates": [ring] })
    }

    /// The centre of a circle, or the mean of a polygon's corners.
    pub fn centre(&self) -> Point {
        match &self.shape {
            Shape::Circle { lat_e7, lon_e7, .. } => Point {
                lat_e7: *lat_e7,
                lon_e7: *lon_e7,
                alt_mm: 0,
            },
            Shape::Polygon { points } => {
                let n = points.len().max(1) as i64;
                let lat = points.iter().map(|p| i64::from(p[0])).sum::<i64>() / n;
                let lon = points.iter().map(|p| i64::from(p[1])).sum::<i64>() / n;
                Point {
                    lat_e7: lat as i32,
                    lon_e7: lon as i32,
                    alt_mm: 0,
                }
            }
        }
    }
}

fn deg(e7: i32) -> f64 {
    f64::from(e7) * 1e-7
}

type Xy = (f64, f64);

fn sub(a: Xy, b: Xy) -> Xy {
    (a.0 - b.0, a.1 - b.1)
}

fn dot(a: Xy, b: Xy) -> f64 {
    a.0 * b.0 + a.1 * b.1
}

fn cross(a: Xy, b: Xy) -> f64 {
    a.0 * b.1 - a.1 * b.0
}

fn dist(a: Xy, b: Xy) -> f64 {
    let d = sub(a, b);
    libm::sqrt(dot(d, d))
}

/// A convex outline, corners counter-clockwise.
#[derive(Debug, Clone)]
struct Hull(Vec<Xy>);

impl Hull {
    /// The convex hull of some points (Andrew's monotone chain).
    fn of(points: &[Xy]) -> Hull {
        let mut p = points.to_vec();
        p.sort_by(|a, b| a.0.total_cmp(&b.0).then(a.1.total_cmp(&b.1)));
        p.dedup();
        if p.len() < 3 {
            return Hull(p);
        }
        let mut lower: Vec<Xy> = Vec::new();
        for &q in &p {
            while lower.len() >= 2
                && cross(
                    sub(lower[lower.len() - 1], lower[lower.len() - 2]),
                    sub(q, lower[lower.len() - 1]),
                ) <= 0.0
            {
                lower.pop();
            }
            lower.push(q);
        }
        let mut upper: Vec<Xy> = Vec::new();
        for &q in p.iter().rev() {
            while upper.len() >= 2
                && cross(
                    sub(upper[upper.len() - 1], upper[upper.len() - 2]),
                    sub(q, upper[upper.len() - 1]),
                ) <= 0.0
            {
                upper.pop();
            }
            upper.push(q);
        }
        lower.pop();
        upper.pop();
        lower.extend(upper);
        Hull(lower)
    }

    fn area(&self) -> f64 {
        let n = self.0.len();
        (0..n)
            .map(|i| cross(self.0[i], self.0[(i + 1) % n]))
            .sum::<f64>()
            / 2.0
    }

    /// Edges as (start, outward unit normal).
    fn edges(&self) -> impl Iterator<Item = (Xy, Xy, Xy)> + '_ {
        let n = self.0.len();
        (0..n).map(move |i| {
            let a = self.0[i];
            let b = self.0[(i + 1) % n];
            let d = sub(b, a);
            let len = libm::sqrt(dot(d, d)).max(1e-12);
            (a, b, (d.1 / len, -d.0 / len))
        })
    }

    /// Grown outward by `by` metres, corners mitred.
    fn grown(&self, by: f64) -> Hull {
        let normals: Vec<Xy> = self.edges().map(|(_, _, n)| n).collect();
        let n = self.0.len();
        Hull(
            (0..n)
                .map(|i| {
                    let n1 = normals[(i + n - 1) % n];
                    let n2 = normals[i];
                    let k = by / (1.0 + dot(n1, n2)).max(0.1);
                    let v = self.0[i];
                    (v.0 + (n1.0 + n2.0) * k, v.1 + (n1.1 + n2.1) * k)
                })
                .collect(),
        )
    }

    fn contains(&self, p: Xy) -> bool {
        self.0.len() >= 3 && self.edges().all(|(a, _, n)| dot(sub(p, a), n) < 0.0)
    }

    /// Does the segment pass through the inside, rather than along an edge
    /// or past a corner? Cyrus–Beck clipping.
    fn crossed_by(&self, p: Xy, q: Xy) -> bool {
        if self.0.len() < 3 {
            return false;
        }
        let d = sub(q, p);
        let (mut enter, mut leave) = (0.0f64, 1.0f64);
        for (a, _, n) in self.edges() {
            let denom = dot(n, d);
            let num = dot(n, sub(p, a));
            if denom.abs() < 1e-12 {
                if num >= 0.0 {
                    return false;
                }
                continue;
            }
            let t = -num / denom;
            if denom < 0.0 {
                enter = enter.max(t);
            } else {
                leave = leave.min(t);
            }
            if enter >= leave {
                return false;
            }
        }
        (leave - enter) * libm::sqrt(dot(d, d)) > 0.01
    }

    /// The nearest point on the outline, pushed `out` metres outward.
    fn nearest_outside(&self, p: Xy, out: f64) -> Xy {
        let mut best = (f64::INFINITY, p);
        for (a, b, n) in self.edges() {
            let ab = sub(b, a);
            let t = (dot(sub(p, a), ab) / dot(ab, ab).max(1e-12)).clamp(0.0, 1.0);
            let on = (a.0 + ab.0 * t, a.1 + ab.1 * t);
            let d = dist(p, on);
            if d < best.0 {
                best = (d, (on.0 + n.0 * out, on.1 + n.1 * out));
            }
        }
        best.1
    }
}

/// A fence as the planner sees it.
#[derive(Debug, Clone)]
struct Obstacle {
    id: u32,
    /// The fence itself, for telling whether a point is inside.
    real: Real,
    /// Grown by the margin: drones fly along this and around its corners.
    outline: Hull,
    /// A metre inside the outline: a leg that crosses it is blocked, so legs
    /// along the outline are not.
    keep_out: Hull,
    /// Bounding box of the outline, to skip far-away fences quickly.
    min: Xy,
    max: Xy,
    /// Its band, metres above the ground.
    floor: f64,
    ceiling: f64,
}

#[derive(Debug, Clone)]
enum Real {
    Circle(Xy, f64),
    Polygon(Vec<Xy>),
}

impl Real {
    fn contains(&self, p: Xy) -> bool {
        match self {
            Real::Circle(c, r) => dist(*c, p) < *r,
            Real::Polygon(v) => {
                // Even–odd ray casting, so a concave polygon is its own shape.
                let mut inside = false;
                let n = v.len();
                for i in 0..n {
                    let (a, b) = (v[i], v[(i + 1) % n]);
                    if (a.1 > p.1) != (b.1 > p.1)
                        && p.0 < a.0 + (p.1 - a.1) / (b.1 - a.1) * (b.0 - a.0)
                    {
                        inside = !inside;
                    }
                }
                inside
            }
        }
    }
}

impl Obstacle {
    /// Is a drone `agl` metres above the ground inside the band?
    fn holds(&self, agl: f64) -> bool {
        agl >= self.floor && agl < self.ceiling
    }

    fn near(&self, p: Xy, q: Xy) -> bool {
        p.0.min(q.0) <= self.max.0
            && p.0.max(q.0) >= self.min.0
            && p.1.min(q.1) <= self.max.1
            && p.1.max(q.1) >= self.min.1
    }
}

/// Every fence open now, and the planner's view of them.
#[derive(Debug, Clone, Default)]
pub struct Airspace {
    /// Projection origin, `[lat_e7, lon_e7]`.
    origin: [i32; 2],
    pub margin_m: f64,
    /// How far above or below a fence's band drones keep, metres.
    pub vertical_margin_m: f64,
    /// The lowest a drone flies beneath a fence, metres above the ground.
    pub min_agl_m: f64,
    fences: Vec<Fence>,
    /// Bumped on every change, so a drone knows when to replan.
    rev: u64,
    next_id: u32,
    obstacles: Vec<Obstacle>,
}

/// The part of an [`Airspace`] a checkpoint keeps.
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct SavedAirspace {
    pub fences: Vec<Fence>,
    pub rev: u64,
    pub next_id: u32,
}

impl Airspace {
    pub fn new(origin: &Point, margin_m: f64) -> Airspace {
        Airspace {
            origin: [origin.lat_e7, origin.lon_e7],
            margin_m: margin_m.max(2.0),
            vertical_margin_m: 10.0,
            min_agl_m: 20.0,
            next_id: 1,
            ..Airspace::default()
        }
    }

    pub fn is_empty(&self) -> bool {
        self.fences.is_empty()
    }

    pub fn fences(&self) -> &[Fence] {
        &self.fences
    }

    pub fn rev(&self) -> u64 {
        self.rev
    }

    pub fn saved(&self) -> SavedAirspace {
        SavedAirspace {
            fences: self.fences.clone(),
            rev: self.rev,
            next_id: self.next_id,
        }
    }

    pub fn restore(&mut self, saved: SavedAirspace) {
        self.fences = saved.fences;
        self.rev = saved.rev;
        self.next_id = saved.next_id.max(1);
        self.rebuild();
    }

    /// Close every fence and forget every id, for a fresh run.
    pub fn clear(&mut self) {
        self.fences.clear();
        self.rev = 0;
        self.next_id = 1;
        self.obstacles.clear();
    }

    /// The id the next fence gets. Ids are never reused.
    pub fn take_id(&mut self) -> u32 {
        let id = self.next_id;
        self.next_id += 1;
        id
    }

    pub fn open(&mut self, fence: Fence) {
        self.fences.push(fence);
        self.fences.sort_by_key(|f| f.id);
        self.changed();
    }

    pub fn close(&mut self, id: u32) -> Option<Fence> {
        let at = self.fences.iter().position(|f| f.id == id)?;
        let fence = self.fences.remove(at);
        self.changed();
        Some(fence)
    }

    /// Change when an open fence closes by itself (`None`: never). Its shape
    /// is unchanged, so nothing needs replanning. The fence as it now is.
    pub fn set_closes(&mut self, id: u32, closes_tick: Option<u64>) -> Option<Fence> {
        let fence = self.fences.iter_mut().find(|f| f.id == id)?;
        fence.closes_tick = closes_tick;
        Some(fence.clone())
    }

    /// Rename an open fence. Its shape is unchanged, so nothing needs
    /// replanning. The fence as it now is.
    pub fn rename(&mut self, id: u32, name: &str) -> Option<Fence> {
        let fence = self.fences.iter_mut().find(|f| f.id == id)?;
        fence.name = name.to_owned();
        Some(fence.clone())
    }

    /// Close every fence due to close by `tick`.
    pub fn expire(&mut self, tick: u64) -> Vec<Fence> {
        let (done, open): (Vec<Fence>, Vec<Fence>) = std::mem::take(&mut self.fences)
            .into_iter()
            .partition(|f| f.closes_tick.is_some_and(|t| t <= tick));
        self.fences = open;
        if !done.is_empty() {
            self.changed();
        }
        done
    }

    fn changed(&mut self) {
        self.rev += 1;
        self.rebuild();
    }

    fn xy(&self, p: &Point) -> Xy {
        flight::offset_m(self.origin[0], self.origin[1], p.lat_e7, p.lon_e7)
    }

    fn xy_e7(&self, lat_e7: i32, lon_e7: i32) -> Xy {
        flight::offset_m(self.origin[0], self.origin[1], lat_e7, lon_e7)
    }

    fn at(&self, p: Xy) -> [i32; 2] {
        let (lat, lon) = flight::displace(self.origin[0], self.origin[1], p.0, p.1);
        [lat, lon]
    }

    fn obstacle(&self, fence: &Fence) -> Option<Obstacle> {
        let (real, hull) = match &fence.shape {
            Shape::Circle {
                lat_e7,
                lon_e7,
                radius_m,
            } => {
                let c = self.xy_e7(*lat_e7, *lon_e7);
                // A polygon round the circle, edges touching it.
                let r = radius_m / libm::cos(core::f64::consts::PI / CIRCLE_SIDES as f64);
                let corners: Vec<Xy> = (0..CIRCLE_SIDES)
                    .map(|i| {
                        let a = i as f64 / CIRCLE_SIDES as f64 * core::f64::consts::TAU;
                        (c.0 + r * libm::cos(a), c.1 + r * libm::sin(a))
                    })
                    .collect();
                (Real::Circle(c, *radius_m), Hull(corners))
            }
            Shape::Polygon { points } => {
                let v: Vec<Xy> = points.iter().map(|p| self.xy_e7(p[0], p[1])).collect();
                let hull = Hull::of(&v);
                (Real::Polygon(v), hull)
            }
        };
        if hull.0.len() < 3 || hull.area() <= 1.0 {
            return None;
        }
        let outline = hull.grown(self.margin_m);
        let keep_out = hull.grown(self.margin_m - 1.0);
        let min = outline
            .0
            .iter()
            .fold((f64::INFINITY, f64::INFINITY), |m, p| {
                (m.0.min(p.0), m.1.min(p.1))
            });
        let max = outline
            .0
            .iter()
            .fold((f64::NEG_INFINITY, f64::NEG_INFINITY), |m, p| {
                (m.0.max(p.0), m.1.max(p.1))
            });
        Some(Obstacle {
            id: fence.id,
            real,
            outline,
            keep_out,
            min,
            max,
            floor: fence.band.floor(),
            ceiling: fence.band.ceiling(),
        })
    }

    fn rebuild(&mut self) {
        self.obstacles = self
            .fences
            .iter()
            .filter_map(|f| self.obstacle(f))
            .collect();
    }

    /// Whether a shape makes a usable fence: three or more corners that
    /// enclose some area, or a positive radius.
    pub fn usable(&self, shape: &Shape) -> bool {
        self.obstacle(&Fence {
            id: 0,
            shape: shape.clone(),
            band: Band::default(),
            name: String::new(),
            source: String::new(),
            opened_tick: 0,
            closes_tick: None,
        })
        .is_some()
    }

    /// Would a fence of this shape keep drones off `point`, margin included?
    pub fn would_cover(&self, shape: &Shape, point: &Point) -> bool {
        let fence = Fence {
            id: 0,
            shape: shape.clone(),
            band: Band::default(),
            name: String::new(),
            source: String::new(),
            opened_tick: 0,
            closes_tick: None,
        };
        self.obstacle(&fence)
            .is_some_and(|o| o.outline.contains(self.xy(point)))
    }

    /// Ids of the fences `point` is inside, `agl_m` above the ground,
    /// ascending.
    pub fn inside(&self, point: &Point, agl_m: f64) -> Vec<u32> {
        if self.obstacles.is_empty() {
            return Vec::new();
        }
        let p = self.xy(point);
        self.obstacles
            .iter()
            .filter(|o| o.holds(agl_m) && o.near(p, p) && o.real.contains(p))
            .map(|o| o.id)
            .collect()
    }

    /// Is `point`, `agl_m` above the ground, inside a fence?
    pub fn blocks(&self, point: &Point, agl_m: f64) -> bool {
        !self.inside(point, agl_m).is_empty()
    }

    /// Fences over `point` that reach too low to fly beneath, so nothing can
    /// be delivered there (or a station used), ascending.
    pub fn covering(&self, point: &Point) -> Vec<u32> {
        if self.obstacles.is_empty() {
            return Vec::new();
        }
        let p = self.xy(point);
        self.obstacles
            .iter()
            .filter(|o| !self.passable_below(o) && o.near(p, p) && o.real.contains(p))
            .map(|o| o.id)
            .collect()
    }

    /// Is the ground at `point` under a fence that cannot be flown beneath?
    pub fn covers(&self, point: &Point) -> bool {
        !self.covering(point).is_empty()
    }

    /// Can a drone pass under this fence at or above the lowest height?
    fn passable_below(&self, o: &Obstacle) -> bool {
        o.floor - self.vertical_margin_m >= self.min_agl_m
    }

    /// Is a fence in the way of a drone flying `agl` above the ground,
    /// vertical margin included?
    fn relevant(&self, o: &Obstacle, agl: f64) -> bool {
        agl > o.floor - self.vertical_margin_m && agl < o.ceiling + self.vertical_margin_m
    }

    /// Where a drone inside a fence should head to get out: just past the
    /// nearest side of its outline. `None` if it is not inside one.
    pub fn way_out(&self, point: &Point, agl_m: f64) -> Option<Point> {
        let p = self.xy(point);
        let o = self
            .obstacles
            .iter()
            .find(|o| o.holds(agl_m) && o.near(p, p) && o.real.contains(p))?;
        let [lat_e7, lon_e7] = self.at(o.outline.nearest_outside(p, CLEAR_M));
        Some(Point {
            lat_e7,
            lon_e7,
            alt_mm: point.alt_mm,
        })
    }

    /// Step `p` out of every keep-out zone in the way at `agl` it is in, to
    /// the nearest side.
    fn escape(&self, mut p: Xy, agl: f64) -> Xy {
        for _ in 0..self.obstacles.len() {
            match self
                .obstacles
                .iter()
                .find(|o| self.relevant(o, agl) && o.near(p, p) && o.keep_out.contains(p))
            {
                Some(o) => p = o.outline.nearest_outside(p, CLEAR_M),
                None => break,
            }
        }
        p
    }

    /// Can a drone fly straight from `from` to `to`, `agl_m` above the
    /// ground, without coming inside the margin of a fence?
    pub fn is_clear(&self, from: &Point, to: &Point, agl_m: f64) -> bool {
        self.obstacles.is_empty() || self.clear_between(self.xy(from), self.xy(to), agl_m)
    }

    fn clear_between(&self, p: Xy, q: Xy, agl: f64) -> bool {
        self.obstacles
            .iter()
            .all(|o| !self.relevant(o, agl) || !o.near(p, q) || !o.keep_out.crossed_by(p, q))
    }

    /// The way from `from` to `to` around every fence in the way at `agl_m`
    /// above the ground, as `[lat_e7, lon_e7]` points to fly through in
    /// turn. The last is `to` itself, or, if `to` is inside a fence, the
    /// nearest point outside it that can be reached.
    pub fn route(&self, from: &Point, to: &Point, agl_m: f64) -> Vec<[i32; 2]> {
        self.route_at(from, to, agl_m).0
    }

    /// As [`Airspace::route`], and whether the way is a good one: false if
    /// the drone has to get out of a fence first, cannot reach `to`, or is
    /// walled in and flies straight.
    fn route_at(&self, from: &Point, to: &Point, agl: f64) -> (Vec<[i32; 2]>, bool) {
        let goal = [to.lat_e7, to.lon_e7];
        if self.obstacles.is_empty() {
            return (vec![goal], true);
        }
        let p0 = self.xy(from);
        let g0 = self.xy(to);
        let start = self.escape(p0, agl);
        let end = self.escape(g0, agl);
        let reachable = !self
            .obstacles
            .iter()
            .any(|o| self.relevant(o, agl) && o.real.contains(g0));

        let mut out = Vec::new();
        let escaped = dist(start, p0) > 0.01;
        if escaped {
            out.push(start);
        }
        if !self.clear_between(start, end, agl) {
            match self.shortest(start, end, agl) {
                Some(via) => out.extend(via),
                // Walled in: fly straight and let the breach be logged.
                None => return (vec![goal], false),
            }
        }
        let mut path: Vec<[i32; 2]> = out.into_iter().map(|p| self.at(p)).collect();
        if reachable {
            if dist(end, g0) > 0.01 {
                path.push(self.at(end));
            }
            path.push(goal);
        } else {
            path.push(self.at(end));
        }
        path.dedup();
        (path, reachable && !escaped)
    }

    /// The way from `from` to `to` for a leg flown `agl_m` above the
    /// ground: round the fences at that height, or beneath one whose floor
    /// is high enough, if that is shorter (the drop and climb counted) or
    /// the only good way.
    pub fn passage(&self, from: &Point, to: &Point, agl_m: f64) -> Passage {
        let (around, good) = self.route_at(from, to, agl_m);
        let at_height = Passage {
            path: around,
            agl_m,
            beneath: false,
        };
        let Some(low) = self.beneath(from, to, agl_m) else {
            return at_height;
        };
        let (under, under_good) = self.route_at(from, to, low);
        if !under_good {
            return at_height;
        }
        let climb = 2.0 * (agl_m - low);
        if good && self.length(from, &under) + climb >= self.length(from, &at_height.path) {
            return at_height;
        }
        Passage {
            path: under,
            agl_m: low,
            beneath: true,
        }
    }

    /// How low to fly to pass beneath the fences in the way of a leg at
    /// `agl`: under the lowest floor among them, by the vertical margin.
    /// `None` if nothing is in the way, or something in the way cannot be
    /// flown beneath.
    fn beneath(&self, from: &Point, to: &Point, agl: f64) -> Option<f64> {
        let (p, q) = (self.xy(from), self.xy(to));
        let mut low: Option<f64> = None;
        for o in &self.obstacles {
            if !self.relevant(o, agl) || !o.near(p, q) {
                continue;
            }
            let in_way =
                o.keep_out.crossed_by(p, q) || o.keep_out.contains(p) || o.keep_out.contains(q);
            if !in_way {
                continue;
            }
            if !self.passable_below(o) {
                return None;
            }
            let under = o.floor - self.vertical_margin_m;
            low = Some(low.map_or(under, |l| l.min(under)));
        }
        low.filter(|l| *l < agl)
    }

    /// Metres along a path from `from`.
    fn length(&self, from: &Point, path: &[[i32; 2]]) -> f64 {
        let mut at = self.xy(from);
        let mut total = 0.0;
        for p in path {
            let next = self.xy_e7(p[0], p[1]);
            total += dist(at, next);
            at = next;
        }
        total
    }

    /// Corners to pass between `a` and `b`, neither included: Dijkstra over
    /// the outlines' corners, with each leg checked as it is relaxed.
    fn shortest(&self, a: Xy, b: Xy, agl: f64) -> Option<Vec<Xy>> {
        let mut nodes = vec![a, b];
        for o in self.obstacles.iter().filter(|o| self.relevant(o, agl)) {
            for &c in &o.outline.0 {
                if !self.obstacles.iter().any(|other| {
                    self.relevant(other, agl) && other.near(c, c) && other.keep_out.contains(c)
                }) {
                    nodes.push(c);
                }
            }
        }
        let n = nodes.len();
        let mut best = vec![f64::INFINITY; n];
        let mut prev = vec![usize::MAX; n];
        let mut done = vec![false; n];
        best[0] = 0.0;
        loop {
            let u = (0..n)
                .filter(|&i| !done[i] && best[i].is_finite())
                .min_by(|&i, &j| best[i].total_cmp(&best[j]))?;
            if u == 1 {
                break;
            }
            done[u] = true;
            for v in 0..n {
                if done[v] {
                    continue;
                }
                let d = best[u] + dist(nodes[u], nodes[v]);
                if d < best[v] && self.clear_between(nodes[u], nodes[v], agl) {
                    best[v] = d;
                    prev[v] = u;
                }
            }
        }
        let mut via = Vec::new();
        let mut at = prev[1];
        while at != 0 {
            via.push(nodes[at]);
            at = prev[at];
        }
        via.reverse();
        Some(via)
    }
}

/// How to fly one leg: through `path`, at `agl_m` above the ground.
#[derive(Debug, Clone, PartialEq)]
pub struct Passage {
    /// `[lat_e7, lon_e7]` points in turn, ending at the leg's end (or the
    /// edge of the fence it is in).
    pub path: Vec<[i32; 2]>,
    pub agl_m: f64,
    /// Beneath a fence, lower than the leg was to be flown.
    pub beneath: bool,
}

#[cfg(test)]
mod tests {
    use super::*;

    fn origin() -> Point {
        Point::at(23.90, 54.90, 0.0)
    }

    fn east_north(e: f64, n: f64) -> Point {
        let o = origin();
        let (lat, lon) = flight::displace(o.lat_e7, o.lon_e7, e, n);
        Point {
            lat_e7: lat,
            lon_e7: lon,
            alt_mm: 50_000,
        }
    }

    fn circle(id: u32, e: f64, n: f64, r: f64) -> Fence {
        let c = east_north(e, n);
        Fence {
            id,
            shape: Shape::Circle {
                lat_e7: c.lat_e7,
                lon_e7: c.lon_e7,
                radius_m: r,
            },
            band: Band::default(),
            name: String::new(),
            source: "test".into(),
            opened_tick: 0,
            closes_tick: None,
        }
    }

    /// Fly a route in 1 m steps and return the closest it came to the
    /// fence's centre, and its length.
    fn fly(air: &Airspace, from: &Point, route: &[[i32; 2]], centre: Xy) -> (f64, f64) {
        let mut at = air.xy(from);
        let mut closest = f64::INFINITY;
        let mut length = 0.0;
        for p in route {
            let next = air.xy_e7(p[0], p[1]);
            let d = dist(at, next);
            let steps = d.ceil().max(1.0) as usize;
            for i in 0..=steps {
                let f = i as f64 / steps as f64;
                let q = (at.0 + (next.0 - at.0) * f, at.1 + (next.1 - at.1) * f);
                closest = closest.min(dist(q, centre));
            }
            length += d;
            at = next;
        }
        (closest, length)
    }

    #[test]
    fn a_clear_leg_is_flown_straight() {
        let mut air = Airspace::new(&origin(), 30.0);
        air.open(circle(1, 0.0, 1_000.0, 100.0));
        let to = east_north(1_000.0, 0.0);
        assert_eq!(
            air.route(&east_north(0.0, 0.0), &to, 50.0),
            vec![[to.lat_e7, to.lon_e7]]
        );
    }

    #[test]
    fn a_leg_through_a_circle_goes_round_it_by_the_margin_and_not_far_out_of_the_way() {
        let mut air = Airspace::new(&origin(), 30.0);
        air.open(circle(1, 500.0, 0.0, 100.0));
        let from = east_north(0.0, 0.0);
        let to = east_north(1_000.0, 0.0);
        let route = air.route(&from, &to, 50.0);
        assert!(route.len() > 2, "{route:?}");
        assert_eq!(*route.last().unwrap(), [to.lat_e7, to.lon_e7]);
        let (closest, length) = fly(&air, &from, &route, air.xy(&east_north(500.0, 0.0)));
        assert!(closest >= 129.0, "came within {closest} m of the centre");
        // Round a 130 m circle: about 1000 − 260 + π·130 ≈ 1150 m.
        assert!(length < 1_200.0, "{length} m");
        for p in &route {
            assert!(!air.blocks(
                &Point {
                    lat_e7: p[0],
                    lon_e7: p[1],
                    alt_mm: 0
                },
                50.0
            ));
        }
    }

    #[test]
    fn overlapping_fences_are_rounded_together_and_a_walled_in_goal_holds_at_the_edge() {
        let mut air = Airspace::new(&origin(), 20.0);
        air.open(circle(1, 500.0, -80.0, 120.0));
        air.open(circle(2, 500.0, 80.0, 120.0));
        let from = east_north(0.0, 0.0);
        let to = east_north(1_000.0, 0.0);
        let route = air.route(&from, &to, 50.0);
        let mut at = air.xy(&from);
        for p in &route {
            let next = air.xy_e7(p[0], p[1]);
            assert!(air.clear_between(at, next, 50.0), "leg into a fence");
            at = next;
        }
        // The centre of fence 1 cannot be reached: the route ends outside it.
        let inside = east_north(500.0, -80.0);
        let route = air.route(&from, &inside, 50.0);
        let last = *route.last().unwrap();
        let end = Point {
            lat_e7: last[0],
            lon_e7: last[1],
            alt_mm: 0,
        };
        assert!(!air.blocks(&end, 50.0));
        assert_ne!(last, [inside.lat_e7, inside.lon_e7]);
    }

    #[test]
    fn inside_a_polygon_is_judged_by_the_polygon_and_there_is_a_way_out() {
        let mut air = Airspace::new(&origin(), 30.0);
        // An L: the notch at the top right is outside.
        let corners = [
            (0.0, 0.0),
            (400.0, 0.0),
            (400.0, 200.0),
            (200.0, 200.0),
            (200.0, 400.0),
            (0.0, 400.0),
        ];
        let points = corners
            .iter()
            .map(|(e, n)| {
                let p = east_north(*e, *n);
                [p.lat_e7, p.lon_e7]
            })
            .collect();
        air.open(Fence {
            id: 7,
            shape: Shape::Polygon { points },
            band: Band::default(),
            name: String::new(),
            source: "test".into(),
            opened_tick: 0,
            closes_tick: None,
        });
        assert_eq!(air.inside(&east_north(100.0, 100.0), 50.0), vec![7]);
        assert!(
            air.inside(&east_north(300.0, 300.0), 50.0).is_empty(),
            "the notch"
        );
        let out = air.way_out(&east_north(100.0, 20.0), 50.0).unwrap();
        assert!(!air.blocks(&out, 50.0));
        assert!(flight::ground_distance_m(&out, &east_north(100.0, -30.0)) < 2.0);
        assert!(air.way_out(&east_north(-100.0, 0.0), 50.0).is_none());
    }

    #[test]
    fn fences_expire_and_every_change_bumps_the_revision() {
        let mut air = Airspace::new(&origin(), 30.0);
        let mut fence = circle(air.take_id(), 0.0, 0.0, 50.0);
        fence.closes_tick = Some(100);
        air.open(fence);
        let id = air.take_id();
        air.open(circle(id, 500.0, 0.0, 50.0));
        assert_eq!(air.rev(), 2);
        assert!(air.expire(99).is_empty());
        assert_eq!(air.expire(100).len(), 1);
        assert_eq!(air.rev(), 3);
        assert!(air.close(2).is_some() && air.close(2).is_none());
        assert!(air.is_empty());
        assert_eq!(air.take_id(), 3, "ids are never reused");
    }

    fn raised(id: u32, e: f64, n: f64, r: f64, floor: f64) -> Fence {
        let mut f = circle(id, e, n, r);
        f.band = Band {
            floor_m: Some(floor),
            ceiling_m: None,
        };
        f
    }

    #[test]
    fn a_fence_holds_only_its_band_and_one_high_enough_leaves_the_ground_reachable() {
        let mut air = Airspace::new(&origin(), 30.0);
        air.open(raised(1, 0.0, 0.0, 200.0, 100.0));
        air.open(raised(2, 1_000.0, 0.0, 200.0, 25.0));
        let centre = east_north(0.0, 0.0);
        assert!(air.inside(&centre, 60.0).is_empty(), "below the floor");
        assert_eq!(air.inside(&centre, 150.0), vec![1]);
        // Fence 1 can be flown beneath (90 m clears the 20 m lowest); fence
        // 2's floor less its margin is below that, so its ground is covered.
        assert!(!air.covers(&centre));
        assert_eq!(air.covering(&east_north(1_000.0, 0.0)), vec![2]);
    }

    #[test]
    fn a_leg_below_the_floor_flies_straight_and_one_above_it_goes_beneath_when_shorter() {
        let mut air = Airspace::new(&origin(), 30.0);
        air.open(raised(1, 500.0, 0.0, 300.0, 100.0));
        let from = east_north(0.0, 0.0);
        let to = east_north(1_000.0, 0.0);
        let goal = [to.lat_e7, to.lon_e7];
        assert_eq!(air.route(&from, &to, 60.0), vec![goal]);

        // At 120 m it is in the way: beneath, at the floor less the margin.
        let p = air.passage(&from, &to, 120.0);
        assert!(p.beneath);
        assert!((p.agl_m - 90.0).abs() < 1e-9);
        assert_eq!(p.path, vec![goal]);

        // A full-height fence cannot be flown beneath: round it, as before.
        let mut full = Airspace::new(&origin(), 30.0);
        full.open(circle(1, 500.0, 0.0, 300.0));
        let p = full.passage(&from, &to, 120.0);
        assert!(!p.beneath && p.path.len() > 1);
        assert_eq!(p.path, full.route(&from, &to, 120.0));
    }

    #[test]
    fn going_round_a_small_raised_fence_beats_dropping_far_beneath_it() {
        let mut air = Airspace::new(&origin(), 5.0);
        // A small fence just off the line: a short way round it.
        air.open(raised(1, 500.0, 8.0, 10.0, 25.0 + 10.0));
        let from = east_north(0.0, 0.0);
        let to = east_north(1_000.0, 0.0);
        let p = air.passage(&from, &to, 120.0);
        // Beneath at 25 m would cost a 190 m drop and climb; round is a few.
        assert!(!p.beneath, "{p:?}");
        assert!(p.path.len() > 1);
    }
}
