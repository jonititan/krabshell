//! Drones flying MAVLink missions over a world of things to find.
//!
//! Five kinds of entity share the grid:
//!
//! - `drone`: flown by ground stations through krabshell's MAVLink gateway.
//!   Missions and commands arrive as intents (see [`codes`]) carrying the same
//!   fields as `MISSION_ITEM_INT` and `COMMAND_LONG`.
//! - `intruder`: an uncontrolled drone flying a fixed circuit.
//! - `person` and `vehicle`: ground targets wandering near where they started.
//! - `poi`: a named point of interest that never moves.
//!
//! Each drone's camera scans on a fixed cadence; what it sees becomes a
//! [`Report`], which the CoT emitter turns into TAK markers. Reaching a mission
//! item is reported too, for the gateway's `MISSION_ITEM_REACHED`.
//!
//! Every entity's full state lives in its published `WorldObject.state`, so a
//! checkpoint restores missions, flight modes and wander plans exactly. Poses
//! are integers and every random choice comes from a (seed, entity, tick)
//! stream, so `gold` runs replay bit for bit.
//!
//! Ticks are simulated time: `tick_seconds` of it each. Nothing here paces the
//! run to the wall clock — krabshell's gateways keep real-time protocols happy
//! while the simulation runs as fast as it can.

pub mod deliveries;
pub mod dispatch;
pub mod flight;
pub mod heatmap;
pub mod mission;
pub mod runlog;
pub mod sensor;
pub mod targets;
pub mod terrain;

use std::collections::{BTreeMap, BTreeSet};

use serde::{Deserialize, Serialize};
use sim_core::config::Param;
use sim_core::{
    AgentId, CellBag, GeoCell, Intent, IntentVerb, Profile, RejectReason, Report, ReportKind,
    ScenarioBlob, SeedAgent, WorldObject,
};

use crate::grid::SparseCellGrid;
use crate::model::{Model, Truncation};
use crate::models::walkers::gold_draw;
use dispatch::{AddressBook, Courier, Dispatch, DispatchSettings};
use flight::{Point, Pose};
use mission::{Drone, Event, MissionItem, Mode, Tuning, Winch};
use sensor::{Candidate, Sensor, Sighting};
use targets::{Route, Wander};
use terrain::Terrain;

/// `IntentVerb::Custom` codes this model handles. The MAVLink gateway builds
/// these; anything else can too (REST, WebSocket).
pub mod codes {
    /// Payload `{"items": [MissionItem, ...]}`. Replaces the mission.
    pub const MISSION_UPLOAD: u16 = 100;
    /// No payload. Empties the mission.
    pub const MISSION_CLEAR: u16 = 101;
    /// Payload `{"seq": n}`.
    pub const MISSION_SET_CURRENT: u16 = 102;
    /// Payload `{"command": n, "params": [p1..p7]}` (null for NaN), as `COMMAND_LONG`.
    pub const COMMAND: u16 = 103;
}

/// Salts that keep each use of the seed independent.
const SPAWN_SALT: u64 = 0x0b5e_55ed_5a17_0001;
const WANDER_SALT: u64 = 0x0b5e_55ed_5a17_0002;
const ORDER_SALT: u64 = 0x0b5e_55ed_5a17_0003;

/// Occupancy is re-resolved when an entity crosses into a new square of this
/// many degrees × 10⁷ (≈4.5 m of latitude). Cells are tens of metres across or
/// more, so occupancy lags a boundary crossing by a few metres at most, and
/// nobody pays for an A5 lookup every tick. Keying on a fixed grid rather than
/// "distance since the last lookup" keeps it a pure function of the pose, so a
/// restored checkpoint refreshes on exactly the same ticks.
const CELL_REFRESH_E7: i32 = 400;

fn refresh_key(pose: &Pose) -> (i32, i32) {
    (
        pose.lat_e7.div_euclid(CELL_REFRESH_E7),
        pose.lon_e7.div_euclid(CELL_REFRESH_E7),
    )
}

const HIGHEST_SYSID: u8 = 250;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Kind {
    Drone,
    Intruder,
    Person,
    Vehicle,
    Poi,
}

impl Kind {
    fn name(self) -> &'static str {
        match self {
            Kind::Drone => "drone",
            Kind::Intruder => "intruder",
            Kind::Person => "person",
            Kind::Vehicle => "vehicle",
            Kind::Poi => "poi",
        }
    }

    fn parse(name: &str) -> Option<Kind> {
        Some(match name {
            "drone" => Kind::Drone,
            "intruder" => Kind::Intruder,
            "person" => Kind::Person,
            "vehicle" => Kind::Vehicle,
            "poi" => Kind::Poi,
            _ => return None,
        })
    }
}

/// The published shape of an entity, and what a checkpoint restores from.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
struct Saved {
    #[serde(flatten)]
    pose: Pose,
    #[serde(default, skip_serializing_if = "String::is_empty")]
    callsign: String,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    drone: Option<Drone>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    route: Option<Route>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    wander: Option<Wander>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    courier: Option<Courier>,
}

#[derive(Debug, Clone)]
struct Entity {
    kind: Kind,
    pose: Pose,
    callsign: String,
    drone: Option<Drone>,
    route: Option<Route>,
    wander: Option<Wander>,
    courier: Option<Courier>,
    /// The refresh square the pose is in; see [`refresh_key`]. Derived.
    cell_key: (i32, i32),
}

impl Entity {
    fn new(kind: Kind, pose: Pose) -> Self {
        Entity {
            kind,
            pose,
            callsign: String::new(),
            drone: None,
            route: None,
            wander: None,
            courier: None,
            cell_key: refresh_key(&pose),
        }
    }

    fn saved(&self) -> Saved {
        Saved {
            pose: self.pose,
            callsign: self.callsign.clone(),
            drone: self.drone.clone(),
            route: self.route.clone(),
            wander: self.wander.clone(),
            courier: self.courier.clone(),
        }
    }

    fn from_saved(kind: Kind, saved: Saved) -> Self {
        Entity {
            kind,
            pose: saved.pose,
            callsign: saved.callsign,
            drone: saved.drone,
            route: saved.route,
            wander: saved.wander,
            courier: saved.courier,
            cell_key: refresh_key(&saved.pose),
        }
    }
}

/// Settings read from `params.toml`.
#[derive(Debug, Clone, Copy)]
struct Settings {
    tuning: Tuning,
    sensor: Sensor,
    max_mission_items: usize,
    /// Orders per second from each address.
    order_rate_per_address_s: f64,
    dispatch: DispatchSettings,
    cruise_cms: i32,
    person_speed_cms: i32,
    vehicle_speed_cms: i32,
    intruder_speed_cms: i32,
    wander_radius_m: u32,
}

pub struct Drones {
    grid: SparseCellGrid,
    entities: BTreeMap<AgentId, Entity>,
    /// Ground elevation, from the pack's `world/fields.json`.
    terrain: Terrain,
    /// Addresses that order deliveries, from the pack's `world/fields.json`.
    book: AddressBook,
    dispatch: Dispatch,
    settings: Settings,
    seed: u64,
    gold: bool,
    next_id: AgentId,
    /// When each (observer, target) pair was last reported.
    last_seen: BTreeMap<(AgentId, AgentId), u64>,
    reports: Vec<Report>,
    events: Vec<Event>,
    sightings: Vec<Sighting>,
}

/// Registered as `drones` in the model registry.
pub fn build(blob: &ScenarioBlob) -> anyhow::Result<Box<dyn Model>> {
    let mut model = Drones {
        grid: SparseCellGrid::new(blob.config.grid.resolution, blob.config.grid.root),
        entities: BTreeMap::new(),
        terrain: Terrain::FLAT,
        book: AddressBook::default(),
        dispatch: Dispatch::default(),
        settings: settings(&blob.config.model),
        seed: 0,
        gold: false,
        next_id: 1,
        last_seen: BTreeMap::new(),
        reports: Vec::new(),
        events: Vec::new(),
        sightings: Vec::new(),
    };
    model.reset(blob)?;
    Ok(Box::new(model))
}

fn param_f64(params: &BTreeMap<String, Param>, key: &str, default: f64) -> f64 {
    params
        .get(key)
        .and_then(Param::as_f64)
        .filter(|v| v.is_finite())
        .unwrap_or(default)
}

fn cms(metres_per_second: f64) -> i32 {
    libm::round(metres_per_second * 100.0) as i32
}

fn settings(params: &BTreeMap<String, Param>) -> Settings {
    let dt = param_f64(params, "tick_seconds", 0.1).clamp(0.001, 10.0);
    let ticks = |seconds: f64| (libm::round(seconds / dt) as u64).max(1);
    let hfov = param_f64(params, "hfov_deg", 70.0).clamp(1.0, 170.0);
    Settings {
        tuning: Tuning {
            dt,
            dt_ms: libm::round(dt * 1000.0) as u32,
            climb_ms: param_f64(params, "climb_ms", 3.0).max(0.1),
            takeoff_alt_mm: flight::to_mm(param_f64(params, "takeoff_alt_m", 20.0)),
            rtl_alt_mm: flight::to_mm(param_f64(params, "rtl_alt_m", 30.0)),
            // A 25-minute endurance by default. 0 flies forever.
            drain_ppm_tick: libm::round(
                param_f64(params, "battery_drain_pct_s", 100.0 / 1500.0).max(0.0) * 10_000.0 * dt,
            ) as u32,
            reserve_ppm: (param_f64(params, "battery_reserve_pct", 15.0).clamp(0.0, 100.0)
                * 10_000.0) as u32,
            // No charger by default.
            charge_ppm_tick: libm::round(
                param_f64(params, "charge_pct_s", 0.0).max(0.0) * 10_000.0 * dt,
            ) as u32,
            launch_ppm: (param_f64(params, "launch_battery_pct", 0.0).clamp(0.0, 100.0) * 10_000.0)
                as u32,
            winch: Winch {
                drop_ms: param_f64(params, "winch_drop_ms", 2.0).max(0.01),
                slow_ms: param_f64(params, "winch_slow_ms", 0.5).max(0.01),
                slow_m: param_f64(params, "winch_slow_m", 3.0).max(0.0),
                release_s: param_f64(params, "winch_release_s", 5.0).max(0.0),
                reel_ms: param_f64(params, "winch_reel_ms", 2.5).max(0.01),
                lock_s: param_f64(params, "winch_lock_s", 3.0).max(0.0),
            },
        },
        sensor: Sensor {
            half_fov_tan: libm::tan(hfov.to_radians() / 2.0),
            max_range_m: param_f64(params, "max_range_m", 250.0).max(1.0),
            p_max: param_f64(params, "p_detect", 0.85).clamp(0.0, 1.0),
            edge_falloff: param_f64(params, "edge_falloff", 0.6).clamp(0.0, 1.0),
            every_ticks: ticks(param_f64(params, "sensor_period_s", 1.0)),
            report_every_ticks: ticks(param_f64(params, "report_interval_s", 10.0)),
        },
        max_mission_items: param_f64(params, "max_mission_items", 128.0).clamp(1.0, 4096.0)
            as usize,
        order_rate_per_address_s: param_f64(params, "order_rate_per_address_day", 0.0).max(0.0)
            / 86_400.0,
        dispatch: DispatchSettings {
            // Set at reset, once the address book's size is known.
            orders_per_tick: 0.0,
            takeoff_m: param_f64(params, "dispatch_takeoff_m", 30.0) as f32,
            approach_m: param_f64(params, "dispatch_approach_m", 15.0) as f32,
            load_ticks: ticks(param_f64(params, "dispatch_load_s", 120.0)),
        },
        cruise_cms: cms(param_f64(params, "cruise_ms", 10.0)),
        person_speed_cms: cms(param_f64(params, "person_speed_ms", 1.4)),
        vehicle_speed_cms: cms(param_f64(params, "vehicle_speed_ms", 8.0)),
        intruder_speed_cms: cms(param_f64(params, "intruder_speed_ms", 12.0)),
        wander_radius_m: param_f64(params, "wander_radius_m", 600.0).max(1.0) as u32,
    }
}

fn rejected(message: impl Into<String>) -> RejectReason {
    RejectReason::Rejected {
        message: message.into(),
    }
}

fn drone_in<'a>(
    entities: &'a mut BTreeMap<AgentId, Entity>,
    intent: &Intent,
) -> Result<&'a mut Entity, RejectReason> {
    let agent = intent
        .agent
        .ok_or_else(|| rejected("drone commands need an agent id"))?;
    match entities.get_mut(&agent) {
        Some(entity) if entity.drone.is_some() => Ok(entity),
        Some(entity) => Err(rejected(format!(
            "agent {agent} is a {}, not a drone",
            entity.kind.name()
        ))),
        None => Err(rejected(format!("no agent {agent}"))),
    }
}

/// Two uniform numbers from one (seed, entity, tick) stream.
fn draw_pair(seed: u64, id: AgentId, tick: u64) -> (f64, f64) {
    let (a, b) = gold_draw(seed, id, tick);
    (a, (b >> 11) as f64 / (1u64 << 53) as f64)
}

impl Drones {
    fn cell_of(&self, lat_e7: i32, lon_e7: i32) -> Option<GeoCell> {
        let cell = GeoCell::from_lonlat(
            f64::from(lon_e7) * 1e-7,
            f64::from(lat_e7) * 1e-7,
            self.grid.resolution(),
        )
        .ok()?;
        self.grid.accepts(cell).then_some(cell)
    }

    fn accepts(&self, point: &Point) -> bool {
        self.cell_of(point.lat_e7, point.lon_e7).is_some()
    }

    fn insert(&mut self, id: AgentId, entity: Entity) -> anyhow::Result<()> {
        let cell = self
            .cell_of(entity.pose.lat_e7, entity.pose.lon_e7)
            .ok_or_else(|| {
                anyhow::anyhow!(
                    "{} {id} at {:.6}, {:.6} is outside the world",
                    entity.kind.name(),
                    entity.pose.lat(),
                    entity.pose.lon()
                )
            })?;
        self.grid.set_location(id, cell);
        self.entities.insert(id, entity);
        self.next_id = self.next_id.max(id + 1);
        Ok(())
    }

    fn origin(&self, blob: &ScenarioBlob) -> anyhow::Result<Point> {
        let params = &blob.config.model;
        match (
            params.get("origin_lon").and_then(Param::as_f64),
            params.get("origin_lat").and_then(Param::as_f64),
        ) {
            (Some(lon), Some(lat)) => Ok(Point::at(lon, lat, 0.0)),
            _ => {
                let (lon, lat) = match blob.config.grid.root {
                    Some(root) => root.centroid()?,
                    None => (0.0, 0.0),
                };
                Ok(Point::at(lon, lat, 0.0))
            }
        }
    }

    fn seed_position(seed: &SeedAgent, resolution: i32) -> anyhow::Result<(f64, f64)> {
        match (seed.lon, seed.lat) {
            (Some(lon), Some(lat)) => Ok((lon, lat)),
            _ => Ok(seed.resolve_cell(resolution)?.centroid()?),
        }
    }

    fn seed_entity(
        &mut self,
        seed: &SeedAgent,
        used_sysids: &mut BTreeSet<u8>,
    ) -> anyhow::Result<()> {
        let kind = Kind::parse(&seed.kind).ok_or_else(|| {
            anyhow::anyhow!(
                "seed agent kind '{}' is not one of drone, intruder, person, vehicle, poi",
                seed.kind
            )
        })?;
        let id = seed.id.unwrap_or(self.next_id);
        let (lon, lat) = Self::seed_position(seed, self.grid.resolution())?;
        let state = &seed.state;
        let text = |key: &str| state.get(key).and_then(|v| v.as_str()).map(str::to_owned);
        let number = |key: &str| state.get(key).and_then(serde_json::Value::as_f64);
        let s = self.settings;

        let mut entity = Entity::new(kind, Pose::at(lon, lat, 0.0));
        entity.pose.alt_mm = self.ground_mm(&entity.pose);
        match kind {
            Kind::Drone => {
                // What it stands on, above mean sea level: a rooftop pad, say.
                // The terrain if not given.
                if let Some(elevation) = number("elevation_m") {
                    entity.pose.alt_mm = flight::to_mm(elevation);
                }
                let sysid = match number("sysid") {
                    Some(n) if (1.0..=f64::from(HIGHEST_SYSID)).contains(&n) => n as u8,
                    Some(n) => {
                        anyhow::bail!("drone {id}: sysid {n} is outside 1..={HIGHEST_SYSID}")
                    }
                    None => lowest_free(used_sysids)?,
                };
                if !used_sysids.insert(sysid) {
                    anyhow::bail!("drone {id}: sysid {sysid} is used twice");
                }
                let cruise = number("cruise_ms").map_or(s.cruise_cms, cms);
                let mut drone = Drone::new(sysid, entity.pose.point(), cruise);
                if let Some(items) = state.get("mission") {
                    let items: Vec<MissionItem> = serde_json::from_value(items.clone())
                        .map_err(|e| anyhow::anyhow!("drone {id}: bad mission: {e}"))?;
                    self.check_mission(&items)
                        .map_err(|e| anyhow::anyhow!("drone {id}: {e}"))?;
                    drone.upload(items);
                }
                if state.get("autostart").and_then(|v| v.as_bool()) == Some(true) {
                    if drone.mission.is_empty() {
                        anyhow::bail!("drone {id}: autostart needs a mission");
                    }
                    drone.mode = Mode::Auto;
                }
                entity.callsign = text("callsign").unwrap_or_else(|| format!("UAV-{sysid}"));
                if let Some(depot) = text("depot") {
                    let index = self.book.depot_index(&depot).ok_or_else(|| {
                        anyhow::anyhow!(
                            "drone {id}: depot '{depot}' is not one of the address book's {:?}",
                            self.book.depots
                        )
                    })?;
                    entity.courier = Some(Courier {
                        depot: index,
                        cruise_agl_m: number("cruise_agl_m").unwrap_or(60.0) as f32,
                        order: None,
                        delivered: false,
                        landed_tick: None,
                    });
                }
                entity.drone = Some(drone);
            }
            Kind::Intruder => {
                let route: Vec<[f64; 3]> = state
                    .get("route")
                    .map(|r| serde_json::from_value(r.clone()))
                    .transpose()
                    .map_err(|e| anyhow::anyhow!("intruder {id}: bad route: {e}"))?
                    .unwrap_or_default();
                if route.len() < 2 {
                    anyhow::bail!(
                        "intruder {id}: a route needs at least two [lon, lat, alt_m] points"
                    );
                }
                // Route altitudes are above the ground at each point.
                let waypoints: Vec<Point> = route
                    .iter()
                    .map(|[lon, lat, alt]| {
                        let p = Point::at(*lon, *lat, *alt);
                        p.with_alt_mm(p.alt_mm + self.terrain.ground_mm(p.lat_e7, p.lon_e7))
                    })
                    .collect();
                if let Some(bad) = waypoints.iter().position(|p| !self.accepts(p)) {
                    anyhow::bail!("intruder {id}: route point {bad} is outside the world");
                }
                entity.pose = Pose::at(lon, lat, route[0][2]);
                entity.pose.alt_mm += self.ground_mm(&entity.pose);
                entity.callsign = text("callsign").unwrap_or_default();
                entity.route = Some(Route {
                    waypoints,
                    leg: 0,
                    speed_cms: number("speed_ms").map_or(s.intruder_speed_cms, cms),
                });
            }
            Kind::Person | Kind::Vehicle => {
                let speed = if kind == Kind::Person {
                    s.person_speed_cms
                } else {
                    s.vehicle_speed_cms
                };
                entity.callsign = text("callsign").unwrap_or_default();
                entity.wander = Some(Wander {
                    anchor: entity.pose.point(),
                    radius_m: number("radius_m").map_or(s.wander_radius_m, |r| r.max(0.0) as u32),
                    speed_cms: number("speed_ms").map_or(speed, cms),
                    dest: entity.pose.point(),
                    pause_ms: 0,
                });
            }
            Kind::Poi => {
                entity.callsign = text("name")
                    .or_else(|| text("callsign"))
                    .or_else(|| text("note"))
                    .unwrap_or_else(|| format!("POI {id}"));
            }
        }
        self.insert(id, entity)
    }

    /// Scatter `count` wanderers of `kind` within the wander radius of `origin`.
    fn scatter(&mut self, kind: Kind, count: u64, origin: Point) -> anyhow::Result<()> {
        let s = self.settings;
        let speed = if kind == Kind::Person {
            s.person_speed_cms
        } else {
            s.vehicle_speed_cms
        };
        for _ in 0..count {
            let id = self.next_id;
            let mut placed = None;
            for attempt in 0..16u64 {
                let (a, b) = draw_pair(self.seed ^ SPAWN_SALT, id, attempt);
                let angle = a * core::f64::consts::TAU;
                let reach = libm::sqrt(b) * f64::from(s.wander_radius_m);
                let (lat, lon) = flight::displace(
                    origin.lat_e7,
                    origin.lon_e7,
                    libm::sin(angle) * reach,
                    libm::cos(angle) * reach,
                );
                let at = Point {
                    lat_e7: lat,
                    lon_e7: lon,
                    alt_mm: 0,
                };
                if self.accepts(&at) {
                    placed = Some(at);
                    break;
                }
            }
            let Some(at) = placed else {
                anyhow::bail!(
                    "could not place {} {id} inside the world within {} m of the origin",
                    kind.name(),
                    s.wander_radius_m
                );
            };
            let mut entity = Entity::new(
                kind,
                Pose {
                    lat_e7: at.lat_e7,
                    lon_e7: at.lon_e7,
                    alt_mm: self.terrain.ground_mm(at.lat_e7, at.lon_e7),
                    ..Pose::default()
                },
            );
            entity.wander = Some(Wander {
                anchor: at,
                radius_m: s.wander_radius_m,
                speed_cms: speed,
                dest: at,
                pause_ms: 0,
            });
            self.insert(id, entity)?;
        }
        Ok(())
    }

    fn check_mission(&self, items: &[MissionItem]) -> Result<(), String> {
        if items.len() > self.settings.max_mission_items {
            return Err(format!(
                "{} items is more than this world allows ({})",
                items.len(),
                self.settings.max_mission_items
            ));
        }
        for (index, item) in items.iter().enumerate() {
            item.check(index)?;
            if let Some(at) = item.destination() {
                if !self.accepts(&at) {
                    return Err(format!("item {index} is outside the world"));
                }
            }
        }
        Ok(())
    }

    fn drone_mut(&mut self, intent: &Intent) -> Result<&mut Entity, RejectReason> {
        drone_in(&mut self.entities, intent)
    }

    fn ground_mm(&self, pose: &Pose) -> i32 {
        self.terrain.ground_mm(pose.lat_e7, pose.lon_e7)
    }

    fn apply_drone_intent(&mut self, code: u16, intent: &Intent) -> Result<(), RejectReason> {
        match code {
            codes::MISSION_UPLOAD => {
                let items: Vec<MissionItem> = intent
                    .payload
                    .get("items")
                    .map(|v| serde_json::from_value(v.clone()))
                    .transpose()
                    .map_err(|e| rejected(format!("mission items did not parse: {e}")))?
                    .ok_or_else(|| rejected("mission upload needs an items array"))?;
                self.check_mission(&items).map_err(rejected)?;
                let entity = self.drone_mut(intent)?;
                entity.drone.as_mut().expect("checked").upload(items);
                Ok(())
            }
            codes::MISSION_CLEAR => {
                let entity = self.drone_mut(intent)?;
                entity.drone.as_mut().expect("checked").clear();
                Ok(())
            }
            codes::MISSION_SET_CURRENT => {
                let seq = intent
                    .payload
                    .get("seq")
                    .and_then(serde_json::Value::as_u64)
                    .ok_or_else(|| rejected("set-current needs a seq"))?;
                let entity = self.drone_mut(intent)?;
                entity
                    .drone
                    .as_mut()
                    .expect("checked")
                    .set_current(seq.min(u64::from(u16::MAX)) as u16)
                    .map_err(rejected)
            }
            codes::COMMAND => {
                let command = intent
                    .payload
                    .get("command")
                    .and_then(serde_json::Value::as_u64)
                    .ok_or_else(|| rejected("a command needs a command number"))?;
                let mut params = [f32::NAN; 7];
                if let Some(list) = intent.payload.get("params").and_then(|v| v.as_array()) {
                    for (slot, value) in params.iter_mut().zip(list) {
                        *slot = value.as_f64().map_or(f32::NAN, |v| v as f32);
                    }
                }
                // A NaN arm flag or mission start index means "not given".
                for p in params.iter_mut().take(4) {
                    if p.is_nan() {
                        *p = 0.0;
                    }
                }
                let tuning = self.settings.tuning;
                if command == u64::from(mission::DO_REPOSITION) {
                    let lat = f64::from(params[4]);
                    let lon = f64::from(params[5]);
                    if lat.is_finite()
                        && lon.is_finite()
                        && !self.accepts(&Point::at(lon, lat, 0.0))
                    {
                        return Err(RejectReason::OutOfBounds);
                    }
                }
                let entity = drone_in(&mut self.entities, intent)?;
                let pose = entity.pose;
                entity
                    .drone
                    .as_mut()
                    .expect("checked")
                    .command(
                        command.min(u64::from(u16::MAX)) as u16,
                        params,
                        &pose,
                        &tuning,
                        &self.terrain,
                    )
                    .map_err(rejected)
            }
            other => Err(rejected(format!(
                "the drones model has no handler for code {other}"
            ))),
        }
    }

    fn spawn_targets(&mut self, intent: &Intent) -> Result<(), RejectReason> {
        let kind_name = intent
            .payload
            .get("kind")
            .and_then(|v| v.as_str())
            .unwrap_or("person");
        let kind = match Kind::parse(kind_name) {
            Some(k @ (Kind::Person | Kind::Vehicle | Kind::Poi)) => k,
            _ => {
                return Err(rejected(format!(
                    "can spawn person, vehicle or poi here, not '{kind_name}'"
                )))
            }
        };
        let count = intent
            .payload
            .get("count")
            .and_then(serde_json::Value::as_u64)
            .unwrap_or(1)
            .clamp(1, 1_000);
        let (lon, lat) = intent
            .cell
            .centroid()
            .map_err(|_| RejectReason::OutOfBounds)?;
        for _ in 0..count {
            let id = self.next_id;
            let mut entity = Entity::new(kind, Pose::at(lon, lat, 0.0));
            entity.pose.alt_mm = self.ground_mm(&entity.pose);
            if kind == Kind::Poi {
                entity.callsign = intent
                    .payload
                    .get("name")
                    .and_then(|v| v.as_str())
                    .map_or_else(|| format!("POI {id}"), str::to_owned);
            } else {
                entity.wander = Some(Wander {
                    anchor: entity.pose.point(),
                    radius_m: self.settings.wander_radius_m,
                    speed_cms: if kind == Kind::Person {
                        self.settings.person_speed_cms
                    } else {
                        self.settings.vehicle_speed_cms
                    },
                    dest: entity.pose.point(),
                    pause_ms: 0,
                });
            }
            self.insert(id, entity)
                .map_err(|_| RejectReason::OutOfBounds)?;
        }
        Ok(())
    }

    fn sighting_report(&self, tick: u64, observer: AgentId, sighting: &Sighting) -> Option<Report> {
        let target = self.entities.get(&sighting.target)?;
        let sysid = self.entities.get(&observer)?.drone.as_ref()?.sysid;
        let mut detail = serde_json::json!({
            "sysid": sysid,
            "range_m": libm::round(sighting.range_m) as i64,
        });
        if !target.callsign.is_empty() {
            detail["callsign"] = serde_json::Value::String(target.callsign.clone());
        }
        if target.pose.spd_cms > 0 {
            detail["course_cdeg"] = target.pose.hdg_cdeg.into();
            detail["speed_cms"] = target.pose.spd_cms.into();
        }
        Some(Report {
            seq: 0,
            tick,
            kind: ReportKind::Detection,
            uid: format!("det-{}", sighting.target),
            observer: Some(observer),
            target: Some(sighting.target),
            target_kind: target.kind.name().to_owned(),
            lat_e7: target.pose.lat_e7,
            lon_e7: target.pose.lon_e7,
            alt_mm: target.pose.alt_mm,
            detail,
        })
    }

    /// Couriers that landed give up or finish their order; new orders join
    /// their depot's queue; ready couriers take the oldest order there.
    fn run_dispatch(&mut self, tick: u64) {
        if self.book.depots.is_empty() {
            return;
        }
        let launch_ppm = self.settings.tuning.launch_ppm;
        let ds = self.settings.dispatch;
        for entity in self.entities.values_mut() {
            let (Some(courier), Some(drone)) = (entity.courier.as_mut(), entity.drone.as_ref())
            else {
                continue;
            };
            if courier.order.is_some() && drone.mode == Mode::Disarmed {
                let mut order = courier.order.take().expect("checked");
                if !courier.delivered {
                    // Back without delivering: first in line again.
                    order.dispatched_tick = None;
                    self.dispatch.queues[usize::from(courier.depot)].push_front(order);
                }
                courier.delivered = false;
                courier.landed_tick = Some(tick);
            }
        }

        let seed = self.seed ^ ORDER_SALT;
        self.dispatch
            .place_orders(tick, &self.book, &ds, |n| draw_pair(seed, n, 0));
        if self.dispatch.queued() == 0 {
            return;
        }

        let terrain = &self.terrain;
        for entity in self.entities.values_mut() {
            let (Some(courier), Some(drone)) = (entity.courier.as_mut(), entity.drone.as_mut())
            else {
                continue;
            };
            let ready = courier.order.is_none()
                && matches!(drone.mode, Mode::Disarmed | Mode::Idle)
                && drone.on_home_pad(&entity.pose, terrain)
                && drone.battery_ppm >= launch_ppm
                && courier
                    .landed_tick
                    .is_none_or(|landed| tick.saturating_sub(landed) >= ds.load_ticks);
            if !ready {
                continue;
            }
            let Some(mut order) = self.dispatch.queues[usize::from(courier.depot)].pop_front()
            else {
                continue;
            };
            let to = &self.book.addresses[order.address as usize];
            drone.upload(dispatch::delivery_mission(
                &drone.home,
                to,
                courier.cruise_agl_m,
                &ds,
            ));
            drone.mode = Mode::Auto;
            order.dispatched_tick = Some(tick);
            courier.order = Some(order);
            courier.delivered = false;
        }
    }

    fn scan(&mut self, tick: u64) {
        let sensor = self.settings.sensor;
        let observers: Vec<(AgentId, Point, i32)> = self
            .entities
            .iter()
            .filter(|(_, e)| {
                e.drone
                    .as_ref()
                    .is_some_and(|d| !d.grounded(&e.pose, &self.terrain))
            })
            .map(|(id, e)| (*id, e.pose.point(), self.ground_mm(&e.pose)))
            .collect();
        for (observer, from, ground_mm) in observers {
            let mut sightings = std::mem::take(&mut self.sightings);
            let candidates = self
                .entities
                .iter()
                .filter(|(_, e)| e.kind != Kind::Drone)
                .map(|(id, e)| Candidate {
                    id: *id,
                    at: e.pose.point(),
                    airborne: e.kind == Kind::Intruder,
                });
            sensor.scan(
                self.seed,
                tick,
                observer,
                &from,
                ground_mm,
                candidates,
                &mut self.last_seen,
                &mut sightings,
            );
            for sighting in sightings.drain(..) {
                if let Some(report) = self.sighting_report(tick, observer, &sighting) {
                    self.reports.push(report);
                }
            }
            self.sightings = sightings;
        }
    }
}

/// A drone as a gateway sees it: parsed back out of the published view.
///
/// This is the read side of the contract between the model and anything that
/// presents drones to the outside world. It only uses what the snapshot
/// carries, so it works on any `Published`, including one decoded from JSON.
#[derive(Debug, Clone, PartialEq)]
pub struct DroneStatus {
    pub id: AgentId,
    pub cell: GeoCell,
    pub sysid: u8,
    pub callsign: String,
    pub pose: Pose,
    pub mode: Mode,
    pub home: Point,
    pub mission: Vec<MissionItem>,
    pub mission_seq: u16,
    pub mission_rev: u32,
    /// Battery remaining, percent.
    pub battery_pct: f32,
}

impl DroneStatus {
    pub fn armed(&self) -> bool {
        !matches!(self.mode, Mode::Disarmed)
    }
}

/// Every drone in a published snapshot, by MAVLink system id.
pub fn drones_in(cells: &[CellBag]) -> BTreeMap<u8, DroneStatus> {
    let mut out = BTreeMap::new();
    for bag in cells {
        for object in bag.objects.iter().filter(|o| o.kind == "drone") {
            let Ok(saved) = serde_json::from_value::<Saved>(object.state.clone()) else {
                continue;
            };
            let Some(drone) = saved.drone else { continue };
            out.insert(
                drone.sysid,
                DroneStatus {
                    id: object.id,
                    cell: bag.cell,
                    sysid: drone.sysid,
                    callsign: saved.callsign,
                    pose: saved.pose,
                    mode: drone.mode,
                    home: drone.home,
                    mission: drone.mission,
                    mission_seq: drone.mission_seq,
                    mission_rev: drone.mission_rev,
                    battery_pct: drone.battery_ppm as f32 / 10_000.0,
                },
            );
        }
    }
    out
}

#[allow(clippy::too_many_arguments)]
fn delivery_report(
    tick: u64,
    observer: AgentId,
    sysid: u8,
    callsign: &str,
    order: &dispatch::Order,
    book: &AddressBook,
    pose: &Pose,
    dt: f64,
) -> Report {
    let address = &book.addresses[order.address as usize];
    let seconds = |from: u64, to: u64| to.saturating_sub(from) as f64 * dt;
    let dispatched = order.dispatched_tick.unwrap_or(tick);
    Report {
        seq: 0,
        tick,
        kind: ReportKind::Delivery,
        uid: format!("order-{}", order.id),
        observer: Some(observer),
        target: None,
        target_kind: String::new(),
        lat_e7: address.lat_e7,
        lon_e7: address.lon_e7,
        alt_mm: pose.alt_mm,
        detail: serde_json::json!({
            "order": order.id,
            "address": order.address,
            "label": address.label,
            "depot": book.depots[usize::from(address.depot)],
            "sysid": sysid,
            "callsign": callsign,
            "ordered_tick": order.ordered_tick,
            "dispatched_tick": dispatched,
            "delivered_tick": tick,
            "wait_s": seconds(order.ordered_tick, dispatched),
            "flight_s": seconds(dispatched, tick),
            "total_s": seconds(order.ordered_tick, tick),
        }),
    }
}

fn lowest_free(used: &BTreeSet<u8>) -> anyhow::Result<u8> {
    (1..=HIGHEST_SYSID)
        .find(|id| !used.contains(id))
        .ok_or_else(|| anyhow::anyhow!("all {HIGHEST_SYSID} MAVLink system ids are taken"))
}

impl Model for Drones {
    fn kind(&self) -> &'static str {
        "drones"
    }

    fn reset(&mut self, blob: &ScenarioBlob) -> anyhow::Result<()> {
        let params = &blob.config.model;
        self.grid
            .reconfigure(blob.config.grid.resolution, blob.config.grid.root);
        self.entities.clear();
        self.last_seen.clear();
        self.reports.clear();
        self.settings = settings(params);
        self.terrain = Terrain::from_fields(&blob.fields)?;
        self.book = AddressBook::from_fields(&blob.fields)?;
        self.dispatch = Dispatch::new(self.book.depots.len());
        self.settings.dispatch.orders_per_tick = self.settings.order_rate_per_address_s
            * self.book.addresses.len() as f64
            * self.settings.tuning.dt;
        self.seed = params
            .get("seed")
            .and_then(Param::as_i64)
            .map_or(0x0d0e_5eed_0000_0001, |s| s as u64);
        self.gold = blob.config.run.profile == Profile::Gold;
        self.next_id = 1;

        let mut sysids = BTreeSet::new();
        for seed in &blob.agents {
            self.seed_entity(seed, &mut sysids)?;
        }

        let origin = self.origin(blob)?;
        let count = |key: &str| params.get(key).and_then(Param::as_i64).unwrap_or(0).max(0) as u64;
        self.scatter(Kind::Person, count("persons"), origin)?;
        self.scatter(Kind::Vehicle, count("vehicles"), origin)?;

        self.grid.commit_now();
        Ok(())
    }

    fn apply_intent(&mut self, intent: &Intent) -> Result<(), RejectReason> {
        if !self.grid.accepts(intent.cell) {
            return Err(RejectReason::OutOfBounds);
        }
        match intent.verb {
            IntentVerb::Custom(code) => self.apply_drone_intent(code, intent),
            IntentVerb::Spawn => self.spawn_targets(intent),
            IntentVerb::Despawn => match intent.agent {
                Some(agent) => {
                    if self.entities.remove(&agent).is_none() {
                        return Err(rejected(format!("no agent {agent}")));
                    }
                    self.grid.remove(agent);
                    Ok(())
                }
                None => {
                    for agent in self.grid.occupants(intent.cell).to_vec() {
                        self.entities.remove(&agent);
                        self.grid.remove(agent);
                    }
                    Ok(())
                }
            },
            IntentVerb::Move | IntentVerb::SetField => Err(rejected(
                "drones move by mission (codes 100–103); targets move on their own",
            )),
        }
    }

    fn step(&mut self, tick: u64) {
        let tuning = self.settings.tuning;
        let seed = self.seed;
        let terrain = &self.terrain;
        let mut moved: Vec<AgentId> = Vec::new();

        for (id, entity) in self.entities.iter_mut() {
            if let Some(drone) = entity.drone.as_mut() {
                drone.step(&mut entity.pose, &tuning, terrain, &mut self.events);
                for event in self.events.drain(..) {
                    if let Event::Reached(item) = event {
                        let winched = drone
                            .mission
                            .get(usize::from(item))
                            .is_some_and(|i| i.command == mission::DO_WINCH);
                        if let Some(courier) = entity.courier.as_mut().filter(|_| winched) {
                            if let Some(order) =
                                courier.order.as_ref().filter(|_| !courier.delivered)
                            {
                                courier.delivered = true;
                                self.dispatch.delivered += 1;
                                self.reports.push(delivery_report(
                                    tick,
                                    *id,
                                    drone.sysid,
                                    &entity.callsign,
                                    order,
                                    &self.book,
                                    &entity.pose,
                                    tuning.dt,
                                ));
                            }
                        }
                        self.reports.push(Report {
                            seq: 0,
                            tick,
                            kind: ReportKind::Waypoint,
                            uid: format!("wp-{id}-{item}"),
                            observer: Some(*id),
                            target: None,
                            target_kind: String::new(),
                            lat_e7: entity.pose.lat_e7,
                            lon_e7: entity.pose.lon_e7,
                            alt_mm: entity.pose.alt_mm,
                            detail: serde_json::json!({ "seq": item, "sysid": drone.sysid }),
                        });
                    }
                }
            } else if let Some(route) = entity.route.as_mut() {
                route.step(&mut entity.pose, tuning.dt);
            } else if let Some(wander) = entity.wander.as_mut() {
                let draw = draw_pair(seed ^ WANDER_SALT, *id, tick);
                let grid = &self.grid;
                wander.step(&mut entity.pose, tuning.dt, tuning.dt_ms, draw, |p| {
                    GeoCell::from_lonlat(p.lon(), p.lat(), grid.resolution())
                        .is_ok_and(|c| grid.accepts(c))
                });
                entity.pose.alt_mm = terrain.ground_mm(entity.pose.lat_e7, entity.pose.lon_e7);
            }
            let key = refresh_key(&entity.pose);
            if key != entity.cell_key {
                entity.cell_key = key;
                moved.push(*id);
            }
        }

        for id in moved {
            let pose = self.entities[&id].pose;
            let (lat, lon) = (pose.lat_e7, pose.lon_e7);
            // Off the edge of the world (a drone following a stale plan): keep
            // the last cell rather than drop the entity from occupancy.
            if let Some(cell) = self.cell_of(lat, lon) {
                self.grid.set_location(id, cell);
            }
        }

        self.run_dispatch(tick);

        if self.settings.sensor.scans_on(tick) {
            self.scan(tick);
        }
        self.grid.lazy_update();
    }

    fn snapshot(&self, out: &mut Vec<CellBag>, limit: usize) -> Truncation {
        out.clear();
        let mut truncation = Truncation::default();
        for cell in self.grid.sorted_cells() {
            if out.len() >= limit {
                truncation.cells_omitted += 1;
                continue;
            }
            let mut objects: Vec<WorldObject> = self
                .grid
                .occupants(cell)
                .iter()
                .filter_map(|id| {
                    let entity = self.entities.get(id)?;
                    Some(WorldObject {
                        id: *id,
                        kind: entity.kind.name().to_owned(),
                        state: serde_json::to_value(entity.saved()).unwrap_or_default(),
                    })
                })
                .collect();
            objects.sort_by_key(|o| o.id);
            out.push(CellBag { cell, objects });
            truncation.cells_written += 1;
        }
        truncation
    }

    fn agent_count(&self) -> u64 {
        self.entities.len() as u64
    }

    fn occupied_cells(&self) -> u64 {
        self.grid.occupied_cells()
    }

    fn fields(&self) -> serde_json::Value {
        let last_seen: Vec<[u64; 3]> = self
            .last_seen
            .iter()
            .map(|((observer, target), tick)| [*observer, *target, *tick])
            .collect();
        serde_json::json!({
            "next_id": self.next_id,
            "seed": self.seed,
            "gold": self.gold,
            "last_seen": last_seen,
            "dispatch": self.dispatch,
        })
    }

    fn restore_fields(&mut self, fields: &serde_json::Value) -> anyhow::Result<()> {
        if let Some(next) = fields.get("next_id").and_then(serde_json::Value::as_u64) {
            self.next_id = self.next_id.max(next);
        }
        if let Some(seed) = fields.get("seed").and_then(serde_json::Value::as_u64) {
            self.seed = seed;
        }
        if let Some(gold) = fields.get("gold").and_then(serde_json::Value::as_bool) {
            self.gold = gold;
        }
        if let Some(dispatch) = fields.get("dispatch") {
            self.dispatch = serde_json::from_value(dispatch.clone())?;
        }
        if let Some(list) = fields.get("last_seen") {
            let pairs: Vec<[u64; 3]> = serde_json::from_value(list.clone())?;
            self.last_seen = pairs.into_iter().map(|[o, t, at]| ((o, t), at)).collect();
        }
        Ok(())
    }

    fn rng_state(&self) -> u64 {
        // Every draw comes from a (seed, entity, tick) stream, so the seed is
        // the whole generator state.
        self.seed
    }

    fn set_rng_state(&mut self, state: u64) {
        self.seed = state;
    }

    fn load_cells(&mut self, cells: Vec<CellBag>) -> anyhow::Result<()> {
        self.grid.clear();
        self.entities.clear();
        for bag in cells {
            if !self.grid.accepts(bag.cell) {
                anyhow::bail!(
                    "checkpoint cell {} does not belong to this world (resolution {}, root {:?})",
                    bag.cell,
                    self.grid.resolution(),
                    self.grid.root()
                );
            }
            for object in bag.objects {
                let kind = Kind::parse(&object.kind).ok_or_else(|| {
                    anyhow::anyhow!(
                        "checkpoint agent {} has unknown kind '{}'",
                        object.id,
                        object.kind
                    )
                })?;
                let saved: Saved = serde_json::from_value(object.state).map_err(|e| {
                    anyhow::anyhow!("checkpoint agent {} did not parse: {e}", object.id)
                })?;
                self.entities
                    .insert(object.id, Entity::from_saved(kind, saved));
                self.grid.set_location(object.id, bag.cell);
                self.next_id = self.next_id.max(object.id + 1);
            }
        }
        self.grid.commit_now();
        Ok(())
    }

    fn drain_reports(&mut self, out: &mut Vec<Report>) {
        out.append(&mut self.reports);
    }
}

#[cfg(test)]
mod tests;
