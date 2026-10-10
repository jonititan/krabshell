//! The simulation state the agents step against.
//!
//! [`World`] owns what agents share: krABMaga's `SparseA5Grid` (occupancy of
//! record), the published board of every agent's state, the terrain, the
//! address book and order queues, and the run's seed. Agents themselves live
//! on krABMaga's [`Schedule`].
//!
//! A step goes:
//!
//! 1. [`State::before_step`]: the tick advances and agents spawned by intents
//!    join the schedule.
//! 2. Every agent steps, in id order, with the [`agents::Dispatcher`] first.
//!    Each reads the committed side (the grid and board as they were at the
//!    end of the last step) and writes itself to the other side.
//! 3. [`State::after_step`]: the grid's `lazy_update` and the board swap
//!    commit what the agents wrote.
//!
//! Intents are checked against the committed board as they arrive and
//! applied by the agent they address at the start of its next step, so
//! nothing an intent does is visible until the step that applies it commits.
//!
//! [`agents::Dispatcher`]: crate::agents::Dispatcher

use std::any::Any;
use std::collections::{BTreeMap, BTreeSet, HashMap, HashSet, VecDeque};
use std::io::Write;
use std::sync::Arc;

use krabmaga::bevy_a5::prelude::GeoCell as A5Cell;
use krabmaga::engine::fields::field::Field;
use krabmaga::engine::fields::sparse_a5_grid::SparseA5Grid;
use krabmaga::engine::schedule::Schedule;
use krabmaga::engine::state::State;
use serde::{Deserialize, Serialize};
use sim_core::config::Param;
use sim_core::rng::gold_draw;
use sim_core::{
    AgentId, CellBag, GeoCell, Intent, IntentVerb, Profile, RejectReason, Report, ReportKind,
    ScenarioBlob, SeedAgent, WorldObject,
};
use sim_core::{Wind, WindField};

use crate::agents::{
    AircraftAgent, Body, Dispatcher, DroneAgent, GroundAgent, IntruderAgent, Member, PoiAgent,
};
use crate::airspace::{Airspace, Band, Fence, SavedAirspace, Shape};
use crate::behaviour::Brain;
use crate::dispatch::{AddressBook, Courier, Dispatch};
use crate::flight::{self, Point, Pose};
use crate::mission::{self, Drone, MissionItem, Mode, Tuning};
use crate::operator::{self, Ops};
use crate::regions::{Region, Regions, Survey};
use crate::settings::{cms, settings, Settings};
use crate::stations::{Planner, Range, Station, Stations};
use crate::targets::{Fix, Flight, Route, VehicleType, Wander};
use crate::terrain::Terrain;

/// `IntentVerb::Custom` codes the drones handle. The MAVLink gateway builds
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
    /// Open a geofence. Payload `{"circle": {"lat": deg, "lon": deg,
    /// "radius_m": m}}` or `{"polygon": [[lon, lat], ...]}`, and optionally
    /// `"name"`, `"minutes"` it stays open (-1: until closed),
    /// `"after_minutes"` before it opens (it waits, scheduled, until then),
    /// and `"floor_m"` and `"ceiling_m"`, metres above the ground (absent:
    /// the surface, unlimited; `"altitude_ref": "amsl"` for above sea
    /// level). Refused if it would keep a drone off its home pad.
    pub const GEOFENCE_OPEN: u16 = 110;
    /// Close a geofence, or cancel one waiting to open. Payload `{"id": n}`.
    pub const GEOFENCE_CLOSE: u16 = 111;
    /// A ground station's whole fence plan. Payload `{"fences": [shape, ...]}`
    /// with each shape as for [`GEOFENCE_OPEN`]. Replaces the fences ground
    /// stations set before; leaves the scenario's, the dashboard's and other
    /// intents' alone.
    pub const GEOFENCE_SET: u16 = 112;
    /// Change how long an open geofence stays open. Payload `{"id": n,
    /// "minutes": m}`: `m` minutes from now, or -1 until it is closed.
    pub const GEOFENCE_EXTEND: u16 = 113;
    /// Rename a geofence: open, waiting to open, or closed (so it reopens
    /// under the new name). Payload `{"id": n, "name": "..."}`; an empty
    /// name clears it.
    pub const GEOFENCE_RENAME: u16 = 114;
    /// Add a launch and recovery station. Payload `{"lat": deg, "lon": deg,
    /// "slots": n}` (1 to 100 launch and recovery slots) and optionally
    /// `"name"`. Refused under a geofence that cannot be flown beneath, or
    /// on top of another station.
    pub const STATION_ADD: u16 = 120;
    /// Remove a station. Payload `{"id": n}`.
    pub const STATION_REMOVE: u16 = 121;
    /// Designate a region of interest for the drones the simulation flies
    /// to survey. Payload `{"lat": deg, "lon": deg, "radius_m": m}` (10 to
    /// 5000) and optionally `"name"`.
    pub const ROI_ADD: u16 = 130;
    /// Remove a region of interest. Payload `{"id": n}`.
    pub const ROI_REMOVE: u16 = 131;
    /// Replace the wind. Payload a whole [`sim_core::WindField`], at a
    /// resolution no finer than the world's. The live weather feed sends
    /// these as it refreshes; logged like any intent, so a gold replay flies
    /// the same air.
    pub const WEATHER_SET: u16 = 140;
}

/// Salts that keep each use of the seed independent.
const SPAWN_SALT: u64 = 0x0b5e_55ed_5a17_0001;
pub(crate) const WANDER_SALT: u64 = 0x0b5e_55ed_5a17_0002;
const ORDER_SALT: u64 = 0x0b5e_55ed_5a17_0003;
const VEHICLE_SALT: u64 = 0x0b5e_55ed_5a17_0004;

/// Occupancy is re-resolved when an entity crosses into a new square of this
/// many degrees × 10⁷ (≈4.5 m of latitude). Cells are tens of metres across or
/// more, so occupancy lags a boundary crossing by a few metres at most, and
/// nobody pays for an A5 lookup every tick. Keying on a fixed grid rather than
/// "distance since the last lookup" keeps it a pure function of the pose, so a
/// restored checkpoint refreshes on exactly the same ticks.
const CELL_REFRESH_E7: i32 = 400;

pub(crate) fn refresh_key(pose: &Pose) -> (i32, i32) {
    (
        pose.lat_e7.div_euclid(CELL_REFRESH_E7),
        pose.lon_e7.div_euclid(CELL_REFRESH_E7),
    )
}

const HIGHEST_SYSID: u8 = 250;

/// Most cells whose sensor neighbourhood is remembered at once.
const CAP_CACHE: usize = 1 << 16;

/// krABMaga orders agents within a step by an `i32`; agents use their id, so
/// ids have to fit. The dispatcher goes first.
const DISPATCHER_ORDER: i32 = i32::MIN;

fn ordering(id: AgentId) -> anyhow::Result<i32> {
    i32::try_from(id)
        .ok()
        .filter(|o| *o > 0)
        .ok_or_else(|| anyhow::anyhow!("agent id {id} is outside 1..={}", i32::MAX))
}

/// Two uniform numbers from one (seed, entity, tick) stream.
pub(crate) fn draw_pair(seed: u64, id: AgentId, tick: u64) -> (f64, f64) {
    let (a, b) = gold_draw(seed, id, tick);
    (a, (b >> 11) as f64 / (1u64 << 53) as f64)
}

fn to_a5(cell: GeoCell) -> A5Cell {
    A5Cell::new(cell.raw())
}

fn new_grid(resolution: i32, root: Option<GeoCell>) -> SparseA5Grid<AgentId> {
    match root {
        Some(root) => SparseA5Grid::new_with_root(to_a5(root), resolution),
        None => SparseA5Grid::new(resolution),
    }
}

/// A schedule for a [`World`].
///
/// Under the `parallel` feature krABMaga's default schedule reads its thread
/// count from the process's command line, which belongs to whoever embeds the
/// model. One thread keeps gold runs in id order anyway.
pub fn schedule() -> Schedule {
    #[cfg(feature = "parallel")]
    {
        Schedule::with_threads(1)
    }
    #[cfg(not(feature = "parallel"))]
    {
        Schedule::new()
    }
}

/// Airspeed of a seed aircraft that sets none, m/s: about 135 knots, an
/// airliner on final approach.
const DEFAULT_AIRCRAFT_SPEED_MS: f64 = 70.0;

/// One point of a seed aircraft's `route`.
#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct SeedFix {
    lon: f64,
    lat: f64,
    /// Above mean sea level.
    alt_m: f64,
    /// Airspeed by this point; the one before it if absent.
    #[serde(default)]
    speed_ms: Option<f64>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Kind {
    Drone,
    Intruder,
    /// A manned aircraft on a published route; see [`Flight`].
    Aircraft,
    Person,
    Vehicle,
    Poi,
}

impl Kind {
    pub fn name(self) -> &'static str {
        match self {
            Kind::Drone => "drone",
            Kind::Intruder => "intruder",
            Kind::Aircraft => "aircraft",
            Kind::Person => "person",
            Kind::Vehicle => "vehicle",
            Kind::Poi => "poi",
        }
    }

    pub fn parse(name: &str) -> Option<Kind> {
        Some(match name {
            "drone" => Kind::Drone,
            "intruder" => Kind::Intruder,
            "aircraft" => Kind::Aircraft,
            "person" => Kind::Person,
            "vehicle" => Kind::Vehicle,
            "poi" => Kind::Poi,
            _ => return None,
        })
    }
}

/// The published shape of an agent, and what a checkpoint restores from.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Saved {
    #[serde(flatten)]
    pub pose: Pose,
    #[serde(default, skip_serializing_if = "String::is_empty")]
    pub callsign: String,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub drone: Option<Drone>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub route: Option<Route>,
    /// A manned aircraft's route and timetable.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub flight: Option<Flight>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub wander: Option<Wander>,
    /// What sort of vehicle a vehicle is.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub vehicle_type: Option<VehicleType>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub courier: Option<Courier>,
    /// A survey of a region of interest it is flying.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub survey: Option<Survey>,
    /// What a drone is doing, for people: `delivering`, `surveying`,
    /// `returning`, `charging`, `flying`, `idle`. Derived from the rest each
    /// step; a restore keeps it so the board hashes the same until then.
    #[serde(default, skip_serializing_if = "String::is_empty")]
    pub activity: String,
    /// Holding off this station until one of its slots is free.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub waiting_for_slot: Option<u32>,
    /// Holding a slot at this station: coming in to land on it, or down on
    /// it, until it takes off again.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub slot: Option<u32>,
}

/// One agent as it was at the end of the last step.
#[derive(Debug, Clone, PartialEq)]
pub struct Record {
    pub id: AgentId,
    pub kind: Kind,
    /// The cell it occupies on the grid.
    pub cell: GeoCell,
    pub saved: Saved,
}

/// A change to a drone's mission or mode, waiting for its next step.
#[derive(Debug, Clone)]
pub(crate) enum Instruction {
    Upload(Vec<MissionItem>),
    Clear,
    SetCurrent(u16),
    Command { command: u16, params: [f32; 7] },
}

impl Instruction {
    pub(crate) fn apply(
        &self,
        drone: &mut Drone,
        pose: &Pose,
        tuning: &Tuning,
        terrain: &Terrain,
    ) -> Result<(), String> {
        match self {
            Instruction::Upload(items) => {
                drone.upload(items.clone());
                Ok(())
            }
            Instruction::Clear => {
                drone.clear();
                Ok(())
            }
            Instruction::SetCurrent(seq) => drone.set_current(*seq),
            Instruction::Command { command, params } => {
                drone.command(*command, *params, pose, tuning, terrain)
            }
        }
    }
}

/// Closed fences remembered to open again, at most.
const MAX_PREVIOUS_FENCES: usize = 100;

/// A fence that has closed: what it was, when, and how ("expired",
/// "closed", "cancelled" before it opened). Closed by a ground station's
/// fence plan counts as "closed".
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Previous {
    #[serde(flatten)]
    pub fence: Fence,
    pub closed_tick: u64,
    pub closed: String,
}

/// Open orders a published view lists at most.
const MAX_PUBLISHED_ORDERS: usize = 2_000;

/// The `source` of a fence a ground station uploaded.
const GCS_SOURCE: &str = "gcs";
/// The `source` of a fence the scenario pack starts with.
const SCENARIO_SOURCE: &str = "scenario";

/// How long a fence stays open, from its payload: absent, null or -1 for
/// until it is closed, otherwise more than 0 minutes.
fn fence_minutes(p: &serde_json::Value, key: &str) -> Result<Option<f64>, RejectReason> {
    match p.get(key) {
        None | Some(serde_json::Value::Null) => Ok(None),
        Some(v) => match v.as_f64() {
            Some(-1.0) => Ok(None),
            Some(m) if m > 0.0 && m.is_finite() => Ok(Some(m)),
            _ => Err(rejected(format!(
                "{key} must be more than 0, or -1 to stay open until closed"
            ))),
        },
    }
}

/// A fence's name from its payload: trimmed, at most 80 characters.
fn fence_name(p: &serde_json::Value) -> String {
    p.get("name")
        .and_then(|v| v.as_str())
        .map(|n| n.trim().chars().take(80).collect())
        .unwrap_or_default()
}

/// When an order was turned away for its address being inside a geofence.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum Stage {
    /// As it was placed.
    Placed,
    /// While it waited in the queue: a fence opened over its address.
    Queued,
    /// When its drone came back (or crashed) without delivering it, a fence
    /// having opened over its address on the way.
    Returned,
}

impl Stage {
    fn name(self) -> &'static str {
        match self {
            Stage::Placed => "placed",
            Stage::Queued => "queued",
            Stage::Returned => "returned",
        }
    }

    fn since(self) -> &'static str {
        match self {
            Stage::Placed => "",
            Stage::Queued => ", which opened while the order waited",
            Stage::Returned => ", which opened while the order was on its way",
        }
    }
}

/// Where an address in the book is, on the ground.
fn address_point(book: &AddressBook, address: u32) -> Point {
    let a = &book.addresses[address as usize];
    Point {
        lat_e7: a.lat_e7,
        lon_e7: a.lon_e7,
        alt_mm: 0,
    }
}

fn reason_text(reason: &RejectReason) -> String {
    match reason {
        RejectReason::Rejected { message } => message.clone(),
        other => other.to_string(),
    }
}

/// Same place and size, to within what MAVLink carries: a circle's radius
/// travels as an `f32`.
fn same_shape(a: &Shape, b: &Shape) -> bool {
    match (a, b) {
        (
            Shape::Circle {
                lat_e7: a1,
                lon_e7: a2,
                radius_m: ar,
            },
            Shape::Circle {
                lat_e7: b1,
                lon_e7: b2,
                radius_m: br,
            },
        ) => a1 == b1 && a2 == b2 && (ar - br).abs() < 0.01,
        (Shape::Polygon { points: a }, Shape::Polygon { points: b }) => a == b,
        _ => false,
    }
}

/// A geofence change asked for by intent, made at the start of the next step.
#[derive(Debug, Clone)]
enum FenceChange {
    Open(Fence, Option<f64>),
    /// Open after this many minutes, for the given minutes (or until closed).
    Schedule(Fence, f64, Option<f64>),
    /// Stay open this many minutes from now, or until closed.
    Extend(u32, Option<f64>),
    Rename(u32, String),
    Close(u32),
}

/// A station or region change asked for by intent, made at the start of the
/// next step.
#[derive(Debug, Clone)]
enum SiteChange {
    AddStation(Station),
    RemoveStation(u32),
    AddRegion(Region),
    RemoveRegion(u32),
}

/// Who asked, for the log: "dashboard", say; "intent" if it does not say.
/// The scenario's, ground stations' and crashes' names are their own.
fn source_of(p: &serde_json::Value) -> String {
    p.get("source")
        .and_then(|v| v.as_str())
        .filter(|s| {
            !s.is_empty()
                && s.len() <= 24
                && !matches!(*s, SCENARIO_SOURCE | GCS_SOURCE | CRASH_SOURCE)
                && s.bytes()
                    .all(|b| b.is_ascii_alphanumeric() || b == b'-' || b == b'_')
        })
        .unwrap_or("intent")
        .to_owned()
}

/// The `source` of a region of interest left where a drone crashed.
const CRASH_SOURCE: &str = "crash";

fn rejected(message: impl Into<String>) -> RejectReason {
    RejectReason::Rejected {
        message: message.into(),
    }
}

/// The drones model's krABMaga state.
pub struct World {
    id: String,
    resolution: i32,
    root: Option<GeoCell>,
    /// The agents a fresh run starts with, built once from the pack.
    roster: Vec<Member>,
    roster_next_id: AgentId,
    roster_spares: Vec<Vec<AgentId>>,
    roster_next_sysid: u16,
    /// Drones the operator flies from the start: couriers, and drones the
    /// pack starts on a mission.
    roster_ops: BTreeSet<AgentId>,
    /// The geofences a fresh run starts with, from the pack's
    /// `world/geofences.json`, and how many minutes each stays open.
    roster_fences: Vec<(Fence, Option<f64>)>,
    /// The stations and regions of interest a fresh run starts with, from
    /// the pack's `world/stations.json` and `world/regions.json`.
    roster_stations: Vec<Station>,
    roster_regions: Vec<Region>,
    /// The wind a fresh run starts with, from the pack's fields (put there
    /// by whoever loaded it: a weather fetch, or `world/weather.json`).
    roster_weather: Option<Arc<WindField>>,
    pack_seed: u64,

    pub(crate) grid: SparseA5Grid<AgentId>,
    pub(crate) settings: Settings,
    pub(crate) terrain: Terrain,
    pub(crate) book: AddressBook,
    pub(crate) dispatch: Dispatch,
    pub(crate) seed: u64,
    gold: bool,
    pub(crate) tick: u64,
    /// The tick the next step runs as, when an embedder sets it.
    pinned: Option<u64>,
    next_id: AgentId,

    /// Committed agent state, sorted by id. Shared with snapshots in flight.
    board: Arc<Vec<Record>>,
    /// What agents publish during a step.
    board_next: Vec<Record>,
    occupied: u64,
    occupied_next: HashSet<GeoCell>,

    /// When each (observer, target) pair was last reported.
    pub(crate) last_seen: BTreeMap<(AgentId, AgentId), u64>,
    keep_reports: bool,
    reports: Vec<Report>,
    report_counts: BTreeMap<&'static str, u64>,
    sink: Option<Box<dyn Write + Send + Sync>>,

    mail: BTreeMap<AgentId, Vec<Instruction>>,
    /// Each addressed drone as its queued instructions leave it, to check the
    /// next one against.
    scratch: BTreeMap<AgentId, Drone>,
    arrivals: Vec<Member>,
    retiring: BTreeSet<AgentId>,

    /// Spares standing by at each depot, lowest id first, until one is
    /// called up to replace a courier that crashed.
    spares: Vec<Vec<AgentId>>,
    /// The MAVLink system id the next new drone gets. It only goes up, so a
    /// crashed drone's id is never handed out again.
    next_sysid: u16,
    /// Courier duties waiting for the spare that takes each over.
    pub(crate) call_ups: BTreeMap<AgentId, Courier>,
    /// New drones placed this step, to join the schedule on the next.
    recruits: Vec<Member>,

    /// The geofences open now.
    pub(crate) airspace: Airspace,
    fence_mail: Vec<FenceChange>,
    /// Fences waiting to open, each with `opened_tick` set to when it will
    /// (and `closes_tick` to when it then closes), soonest first.
    scheduled: Vec<Fence>,
    /// Fences that have closed (or were cancelled before opening), newest
    /// last, at most [`MAX_PREVIOUS_FENCES`]: what can be opened again.
    previous: VecDeque<Previous>,
    /// The drones the operator flies, and its notes on each.
    pub(crate) ops: BTreeMap<AgentId, Ops>,
    /// Launch and recovery stations, and regions of interest.
    pub(crate) stations: Stations,
    pub(crate) regions: Regions,
    site_mail: Vec<SiteChange>,
    /// Station slots taken this step, by station, by drones that set off
    /// to land on one: the board shows them only from the next.
    pub(crate) slot_claims: BTreeMap<u32, u32>,
    /// How far couriers can deliver ([`World::delivery_range`]), and the
    /// station and airspace revisions it was worked out for.
    range: serde_json::Value,
    range_key: Option<(u64, u64, u64)>,
    /// The wind now. Checkpointed, since live updates change it mid-run.
    /// Change it only through [`World::set_weather`].
    pub(crate) weather: Option<Arc<WindField>>,
    /// Its [`WindField::rev`], worked out once per change.
    weather_rev: u64,

    /// The behaviour tree every drone thinks with. Taken out while a drone
    /// ticks it, so the drone can reach the rest of the world.
    pub(crate) brain: Option<Brain>,

    /// Grid cells a drone in a given cell could see into.
    caps: HashMap<GeoCell, Vec<A5Cell>>,
    scan_radius_m: f64,
}

impl World {
    /// Build a world from a pack. Fails if the pack does not make sense for
    /// this model; a world that builds always initialises.
    pub fn new(blob: &ScenarioBlob) -> anyhow::Result<World> {
        let params = &blob.config.model;
        let mut settings = settings(params);
        let terrain = Terrain::from_fields(&blob.fields)?;
        let book = AddressBook::from_fields(&blob.fields)?;
        settings.dispatch.orders_per_tick =
            settings.order_rate_per_address_s * book.addresses.len() as f64 * settings.tuning.dt;
        let seed = params
            .get("seed")
            .and_then(Param::as_i64)
            .map_or(0x0d0e_5eed_0000_0001, |s| s as u64);
        let resolution = blob.config.grid.resolution;
        let root = blob.config.grid.root;
        // A drone's camera reaches into any cell holding a target within
        // range. Cells are pentagons around their centroids, and both ends lag
        // their poses by up to a refresh square, hence the slack.
        let scan_radius_m = settings.sensor.max_range_m
            + 2.0 * sim_core::cell::nominal_cell_edge_meters(resolution)
            + 20.0;
        let mut world = World {
            id: blob.manifest.id.clone(),
            resolution,
            root,
            roster: Vec::new(),
            roster_next_id: 1,
            roster_spares: Vec::new(),
            roster_next_sysid: 1,
            roster_ops: BTreeSet::new(),
            roster_fences: Vec::new(),
            roster_stations: Vec::new(),
            roster_regions: Vec::new(),
            roster_weather: None,
            pack_seed: seed,
            grid: new_grid(resolution, root),
            dispatch: Dispatch::new(book.depots.len()),
            settings,
            terrain,
            book,
            seed,
            gold: blob.config.run.profile == Profile::Gold,
            tick: 0,
            pinned: None,
            next_id: 1,
            board: Arc::default(),
            board_next: Vec::new(),
            occupied: 0,
            occupied_next: HashSet::new(),
            last_seen: BTreeMap::new(),
            keep_reports: true,
            reports: Vec::new(),
            report_counts: BTreeMap::new(),
            sink: None,
            mail: BTreeMap::new(),
            scratch: BTreeMap::new(),
            arrivals: Vec::new(),
            retiring: BTreeSet::new(),
            spares: Vec::new(),
            next_sysid: 1,
            call_ups: BTreeMap::new(),
            recruits: Vec::new(),
            airspace: Airspace::default(),
            fence_mail: Vec::new(),
            scheduled: Vec::new(),
            previous: VecDeque::new(),
            ops: BTreeMap::new(),
            stations: Stations::default(),
            regions: Regions::default(),
            site_mail: Vec::new(),
            slot_claims: BTreeMap::new(),
            range: serde_json::Value::Null,
            range_key: None,
            weather: None,
            weather_rev: 0,
            brain: Some(Brain::from_fields(&blob.fields)?),
            caps: HashMap::new(),
            scan_radius_m,
        };
        let origin = world.origin(blob)?;
        world.airspace = Airspace::new(&origin, world.settings.geofence_margin_m);
        world.airspace.vertical_margin_m = world.settings.geofence_vertical_margin_m;
        world.airspace.min_agl_m = world.settings.beneath_min_agl_m;
        world.enrol(blob)?;
        world.roster_fences = world.pack_fences(&blob.fields)?;
        world.roster_stations = world.pack_sites(&blob.fields, "stations", |w, v| {
            w.station_from(v, SCENARIO_SOURCE)
        })?;
        world.roster_regions = world.pack_sites(&blob.fields, "regions", |w, v| {
            w.region_from(v, SCENARIO_SOURCE)
        })?;
        world.roster_weather = match blob.fields.get("weather") {
            None | Some(serde_json::Value::Null) => None,
            Some(v) => Some(Arc::new(world.weather_from(v)?)),
        };
        world.begin();
        Ok(world)
    }

    // ------------------------------------------------------------ building

    /// A seed drone's `depot` or `spare` names a depot the address book does
    /// not have. Say where depots come from, since a pack without deliveries
    /// has none and the fix is then to drop the key.
    fn no_such_depot(&self, id: AgentId, key: &str, depot: &str) -> anyhow::Error {
        if self.book.depots.is_empty() {
            anyhow::anyhow!(
                "drone {id}: `{key}: \"{depot}\"` makes it a delivery drone, but this pack has no \
                 address book (`addresses` in world/fields.json). Remove `{key}` for a drone \
                 that does not deliver"
            )
        } else {
            anyhow::anyhow!(
                "drone {id}: `{key}: \"{depot}\"` is not one of the depots in world/fields.json \
                 `addresses.depots`: {:?}",
                self.book.depots
            )
        }
    }

    /// Build the roster: the pack's seed agents, then scattered people and
    /// vehicles.
    fn enrol(&mut self, blob: &ScenarioBlob) -> anyhow::Result<()> {
        let mut sysids = BTreeSet::new();
        let mut ids = BTreeSet::new();
        self.roster_spares = vec![Vec::new(); self.book.depots.len()];
        for seed in &blob.agents {
            let member = self.seed_member(seed, &mut sysids)?;
            if let Some(depot) = seed.state.get("spare").and_then(|v| v.as_str()) {
                let id = member.body().id;
                let index = self
                    .book
                    .depot_index(depot)
                    .ok_or_else(|| self.no_such_depot(id, "spare", depot))?;
                match &member {
                    Member::Drone(d) if d.courier.is_none() => {}
                    _ => anyhow::bail!(
                        "agent {id}: a spare must be a drone without a depot of its own"
                    ),
                }
                self.roster_spares[usize::from(index)].push(id);
            }
            if let Member::Drone(d) = &member {
                let autostart = seed.state.get("autostart").and_then(|v| v.as_bool()) == Some(true);
                if autostart || d.courier.is_some() {
                    self.roster_ops.insert(d.body.id);
                }
            }
            self.admit(member, &mut ids)?;
        }
        for spares in &mut self.roster_spares {
            spares.sort_unstable();
        }
        self.roster_next_sysid = sysids.last().map_or(1, |s| u16::from(*s) + 1);
        let params = &blob.config.model;
        let origin = self.origin(blob)?;
        let count = |key: &str| params.get(key).and_then(Param::as_i64).unwrap_or(0).max(0) as u64;
        for kind in [Kind::Person, Kind::Vehicle] {
            let key = if kind == Kind::Person {
                "persons"
            } else {
                "vehicles"
            };
            for _ in 0..count(key) {
                let member = self.scatter(kind, origin)?;
                self.admit(member, &mut ids)?;
            }
        }
        self.roster_next_id = self.next_id;
        Ok(())
    }

    fn admit(&mut self, member: Member, ids: &mut BTreeSet<AgentId>) -> anyhow::Result<()> {
        let id = member.body().id;
        ordering(id)?;
        if !ids.insert(id) {
            anyhow::bail!("agent id {id} is used twice");
        }
        self.next_id = self.next_id.max(id + 1);
        self.roster.push(member);
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

    fn body(&self, id: AgentId, kind: Kind, pose: Pose) -> anyhow::Result<Body> {
        let cell = self.cell_of(pose.lat_e7, pose.lon_e7).ok_or_else(|| {
            anyhow::anyhow!(
                "{} {id} at {:.6}, {:.6} is outside the world",
                kind.name(),
                pose.lat(),
                pose.lon()
            )
        })?;
        Ok(Body::new(id, kind, pose, cell))
    }

    fn seed_member(
        &mut self,
        seed: &SeedAgent,
        used_sysids: &mut BTreeSet<u8>,
    ) -> anyhow::Result<Member> {
        let kind = Kind::parse(&seed.kind).ok_or_else(|| {
            anyhow::anyhow!(
                "seed agent kind '{}' is not one of drone, intruder, aircraft, person, vehicle, poi",
                seed.kind
            )
        })?;
        let id = seed.id.unwrap_or(self.next_id);
        self.next_id = self.next_id.max(id + 1);
        let (lon, lat) = match (seed.lon, seed.lat) {
            (Some(lon), Some(lat)) => (lon, lat),
            // An aircraft starts at the first fix of its route.
            _ if kind == Kind::Aircraft => (0.0, 0.0),
            _ => seed.resolve_cell(self.resolution)?.centroid()?,
        };
        let state = &seed.state;
        let text = |key: &str| state.get(key).and_then(|v| v.as_str()).map(str::to_owned);
        let number = |key: &str| state.get(key).and_then(serde_json::Value::as_f64);
        let s = self.settings;

        let mut pose = Pose::at(lon, lat, 0.0);
        pose.alt_mm = self.terrain.ground_mm(pose.lat_e7, pose.lon_e7);
        Ok(match kind {
            Kind::Drone => {
                // What it stands on, above mean sea level: a rooftop pad, say.
                // The terrain if not given.
                if let Some(elevation) = number("elevation_m") {
                    pose.alt_mm = flight::to_mm(elevation);
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
                let mut drone = Drone::new(sysid, pose.point(), cruise);
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
                let mut body = self.body(id, kind, pose)?;
                body.callsign = text("callsign").unwrap_or_else(|| format!("UAV-{sysid}"));
                let courier = match text("depot") {
                    Some(depot) => {
                        let index = self
                            .book
                            .depot_index(&depot)
                            .ok_or_else(|| self.no_such_depot(id, "depot", &depot))?;
                        Some(Courier {
                            depot: index,
                            cruise_agl_m: number("cruise_agl_m").unwrap_or(60.0) as f32,
                            order: None,
                            delivered: false,
                            landed_tick: None,
                        })
                    }
                    None => None,
                };
                Member::Drone(Box::new(DroneAgent {
                    body,
                    drone,
                    courier,
                    survey: None,
                    activity: "",
                    waiting_for_slot: None,
                    slot: None,
                    wrecked: false,
                }))
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
                let mut pose = Pose::at(lon, lat, route[0][2]);
                pose.alt_mm += self.terrain.ground_mm(pose.lat_e7, pose.lon_e7);
                let mut body = self.body(id, kind, pose)?;
                body.callsign = text("callsign").unwrap_or_default();
                Member::Intruder(IntruderAgent {
                    body,
                    route: Route {
                        waypoints,
                        leg: 0,
                        speed_cms: number("speed_ms").map_or(s.intruder_speed_cms, cms),
                    },
                })
            }
            Kind::Aircraft => {
                let route: Vec<SeedFix> = state
                    .get("route")
                    .map(|r| serde_json::from_value(r.clone()))
                    .transpose()
                    .map_err(|e| anyhow::anyhow!("aircraft {id}: bad route: {e}"))?
                    .unwrap_or_default();
                if route.len() < 2 {
                    anyhow::bail!(
                        "aircraft {id}: a route needs at least two {{lon, lat, alt_m}} points"
                    );
                }
                let mut speed_ms = number("speed_ms").unwrap_or(DEFAULT_AIRCRAFT_SPEED_MS);
                let mut fixes = Vec::with_capacity(route.len());
                for (n, fix) in route.iter().enumerate() {
                    speed_ms = fix.speed_ms.unwrap_or(speed_ms);
                    if !(speed_ms > 0.0 && speed_ms.is_finite()) {
                        anyhow::bail!("aircraft {id}: route point {n} has no positive speed_ms");
                    }
                    // Altitudes are above mean sea level, as ATM gives them.
                    let at = Point::at(fix.lon, fix.lat, fix.alt_m);
                    if !self.accepts(&at) {
                        anyhow::bail!("aircraft {id}: route point {n} is outside the world");
                    }
                    fixes.push(Fix {
                        at,
                        speed_cms: cms(speed_ms),
                    });
                }
                let ticks = |key: &str| -> anyhow::Result<Option<u64>> {
                    match number(key) {
                        None => Ok(None),
                        Some(v) if v >= 0.0 && v.is_finite() => {
                            Ok(Some(libm::round(v / s.tuning.dt) as u64))
                        }
                        Some(v) => anyhow::bail!("aircraft {id}: {key} {v} is not a time"),
                    }
                };
                let repeat_ticks = ticks("repeat_s")?.unwrap_or(0);
                let first = fixes[0].at;
                let parked = Pose {
                    lat_e7: first.lat_e7,
                    lon_e7: first.lon_e7,
                    alt_mm: first.alt_mm,
                    ..Pose::default()
                };
                let mut body = self.body(id, kind, parked)?;
                body.callsign = text("callsign").unwrap_or_default();
                Member::Aircraft(AircraftAgent {
                    body,
                    flight: Flight {
                        fixes,
                        leg: 0,
                        active: false,
                        next_start: ticks("start_s")?.unwrap_or(0),
                        repeat_ticks,
                    },
                })
            }
            Kind::Person | Kind::Vehicle => {
                let speed = if kind == Kind::Person {
                    s.person_speed_cms
                } else {
                    s.vehicle_speed_cms
                };
                let vehicle_type = match (kind, text("type")) {
                    (Kind::Vehicle, Some(t)) => Some(VehicleType::parse(&t).ok_or_else(|| {
                        anyhow::anyhow!(
                            "vehicle {id}: type '{t}' is not one of car, van, pickup, bus, lorry"
                        )
                    })?),
                    _ => self.vehicle_type(kind, id),
                };
                let mut body = self.body(id, kind, pose)?;
                body.callsign = text("callsign").unwrap_or_default();
                Member::Ground(GroundAgent {
                    body,
                    vehicle_type,
                    wander: Wander {
                        anchor: pose.point(),
                        radius_m: number("radius_m")
                            .map_or(s.wander_radius_m, |r| r.max(0.0) as u32),
                        speed_cms: number("speed_ms").map_or(speed, cms),
                        dest: pose.point(),
                        pause_ms: 0,
                    },
                })
            }
            Kind::Poi => {
                let mut body = self.body(id, kind, pose)?;
                body.callsign = text("name")
                    .or_else(|| text("callsign"))
                    .or_else(|| text("note"))
                    .unwrap_or_else(|| format!("POI {id}"));
                Member::Poi(PoiAgent { body })
            }
        })
    }

    /// One wanderer of `kind` within the wander radius of `origin`.
    fn scatter(&mut self, kind: Kind, origin: Point) -> anyhow::Result<Member> {
        let s = self.settings;
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
        let pose = Pose {
            lat_e7: at.lat_e7,
            lon_e7: at.lon_e7,
            alt_mm: self.terrain.ground_mm(at.lat_e7, at.lon_e7),
            ..Pose::default()
        };
        self.next_id = id + 1;
        Ok(Member::Ground(GroundAgent {
            body: self.body(id, kind, pose)?,
            vehicle_type: self.vehicle_type(kind, id),
            wander: self.wander_from(kind, at),
        }))
    }

    /// A vehicle's type, drawn from the seed for its id; none for anything
    /// else.
    fn vehicle_type(&self, kind: Kind, id: AgentId) -> Option<VehicleType> {
        (kind == Kind::Vehicle)
            .then(|| VehicleType::draw(draw_pair(self.seed ^ VEHICLE_SALT, id, 0).0))
    }

    fn wander_from(&self, kind: Kind, at: Point) -> Wander {
        Wander {
            anchor: at,
            radius_m: self.settings.wander_radius_m,
            speed_cms: if kind == Kind::Person {
                self.settings.person_speed_cms
            } else {
                self.settings.vehicle_speed_cms
            },
            dest: at,
            pause_ms: 0,
        }
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

    // ---------------------------------------------------------- the grid

    /// The grid cell at a position, if it is inside the world.
    pub fn cell_of(&self, lat_e7: i32, lon_e7: i32) -> Option<GeoCell> {
        let cell = GeoCell::from_lonlat(
            f64::from(lon_e7) * 1e-7,
            f64::from(lat_e7) * 1e-7,
            self.resolution,
        )
        .ok()?;
        self.contains(cell).then_some(cell)
    }

    pub fn contains(&self, cell: GeoCell) -> bool {
        self.grid.contains(&to_a5(cell))
    }

    pub(crate) fn accepts(&self, point: &Point) -> bool {
        self.cell_of(point.lat_e7, point.lon_e7).is_some()
    }

    /// Write an agent to the uncommitted side. Every agent does this every
    /// step: the commit clears what nobody wrote.
    pub(crate) fn place(&mut self, record: Record) {
        self.grid
            .set_object_location(record.id, &to_a5(record.cell));
        self.occupied_next.insert(record.cell);
        self.board_next.push(record);
    }

    /// Commit what agents wrote since the last commit.
    fn commit(&mut self) {
        self.grid.lazy_update();
        let mut board = std::mem::take(&mut self.board_next);
        // Agents step in id order, so this only sorts after a restore that
        // scheduled out of order.
        if !board.is_sorted_by_key(|r| r.id) {
            board.sort_by_key(|r| r.id);
        }
        let capacity = board.len();
        let old = std::mem::replace(&mut self.board, Arc::new(board));
        // Reuse the old board's allocation unless a snapshot still holds it.
        self.board_next = match Arc::try_unwrap(old) {
            Ok(mut old) => {
                old.clear();
                old
            }
            Err(_) => Vec::with_capacity(capacity),
        };
        self.occupied = self.occupied_next.len() as u64;
        self.occupied_next.clear();
    }

    /// Ids of every agent committed to a cell a drone in `cell` could see
    /// into, sorted.
    pub(crate) fn nearby(&mut self, cell: GeoCell) -> Vec<AgentId> {
        if !self.caps.contains_key(&cell) {
            if self.caps.len() >= CAP_CACHE {
                self.caps.clear();
            }
            let cells = self
                .grid
                .cell_spherical_cap(&to_a5(cell), self.scan_radius_m)
                .unwrap_or_default();
            self.caps.insert(cell, cells);
        }
        let mut ids = Vec::new();
        for c in &self.caps[&cell] {
            if let Some(bag) = self.grid.get_objects(c) {
                ids.extend(bag);
            }
        }
        ids.sort_unstable();
        ids.dedup();
        ids
    }

    // ------------------------------------------------------- shared state

    /// An agent as it was committed at the end of the last step.
    pub fn record(&self, id: AgentId) -> Option<&Record> {
        self.board
            .binary_search_by_key(&id, |r| r.id)
            .ok()
            .map(|i| &self.board[i])
    }

    /// Every committed agent, by id.
    pub fn records(&self) -> &[Record] {
        &self.board
    }

    /// The committed agents, to snapshot elsewhere: on another thread, while
    /// this world steps on.
    pub fn committed(&self) -> Committed {
        Committed {
            board: Arc::clone(&self.board),
        }
    }

    pub(crate) fn retiring(&self, id: AgentId) -> bool {
        self.retiring.contains(&id)
    }

    pub(crate) fn take_mail(&mut self, id: AgentId) -> Vec<Instruction> {
        self.mail.remove(&id).unwrap_or_default()
    }

    pub(crate) fn report(&mut self, report: Report) {
        let kind = match report.kind {
            ReportKind::Detection => "detection",
            ReportKind::Waypoint => "waypoint",
            ReportKind::Delivery => "delivery",
            ReportKind::Landing => "landing",
            ReportKind::Crash => "crash",
            ReportKind::Geofence => "geofence",
            ReportKind::Breach => "breach",
            ReportKind::Reroute => "reroute",
            ReportKind::OrderRejected => "order_rejected",
            ReportKind::Station => "station",
            ReportKind::Roi => "roi",
            ReportKind::Survey => "survey",
        };
        *self.report_counts.entry(kind).or_default() += 1;
        if let Some(sink) = self.sink.as_mut() {
            let written = serde_json::to_writer(&mut *sink, &report)
                .map_err(std::io::Error::from)
                .and_then(|()| sink.write_all(b"\n"));
            if let Err(e) = written {
                eprintln!("could not write a report, no longer writing them: {e}");
                self.sink = None;
            }
        }
        if self.keep_reports {
            self.reports.push(report);
        }
    }

    pub(crate) fn place_orders(&mut self) {
        if self.book.depots.is_empty() {
            return;
        }
        let seed = self.seed ^ ORDER_SALT;
        let reject = self.settings.reject_orders_in_geofences && !self.airspace.is_empty();
        let (book, airspace) = (&self.book, &self.airspace);
        let rejected = self.dispatch.place_orders(
            self.tick,
            book,
            &self.settings.dispatch,
            |n| draw_pair(seed, n, 0),
            |address| !reject || !airspace.covers(&address_point(book, address)),
        );
        for order in rejected {
            self.order_rejected(&order, Stage::Placed);
        }
    }

    /// With `reject_orders_in_geofences`, turn away an order whose address
    /// a geofence now covers, logging why. True if it was turned away; the
    /// caller then forgets it rather than queue it.
    pub(crate) fn reject_if_fenced(
        &mut self,
        order: &crate::dispatch::Order,
        stage: Stage,
    ) -> bool {
        if !self.settings.reject_orders_in_geofences
            || !self
                .airspace
                .covers(&address_point(&self.book, order.address))
        {
            return false;
        }
        self.dispatch.rejected += 1;
        self.order_rejected(order, stage);
        true
    }

    /// An order back from a trip undelivered (its drone came home without
    /// it, or crashed): turned away if a fence now covers its address and
    /// such orders are, or if it has failed `MAX_FAILED_TRIPS` trips; else
    /// back to the front of its depot's queue.
    pub(crate) fn return_order(&mut self, mut order: crate::dispatch::Order, depot: u16) {
        order.dispatched_tick = None;
        order.failed_trips = order.failed_trips.saturating_add(1);
        if self.reject_if_fenced(&order, Stage::Returned) {
            return;
        }
        if order.failed_trips >= crate::dispatch::MAX_FAILED_TRIPS {
            self.dispatch.rejected += 1;
            let reason = format!(
                "no courier could reach the address: {} trips came back without delivering it",
                order.failed_trips
            );
            self.rejection_report(&order, Stage::Returned, None, reason);
            return;
        }
        self.dispatch.queues[usize::from(depot)].push_front(order);
    }

    /// Turn away every queued order a newly opened geofence covers.
    fn reject_newly_fenced(&mut self) {
        if !self.settings.reject_orders_in_geofences {
            return;
        }
        let mut queued: Vec<crate::dispatch::Order> = Vec::new();
        for queue in self.dispatch.queues.iter_mut() {
            let (keep, drop): (VecDeque<_>, VecDeque<_>) = std::mem::take(queue)
                .into_iter()
                .partition(|o| !self.airspace.covers(&address_point(&self.book, o.address)));
            *queue = keep;
            queued.extend(drop);
        }
        for order in queued {
            self.dispatch.rejected += 1;
            self.order_rejected(&order, Stage::Queued);
        }
    }

    /// Log an order turned away because its address is inside a geofence.
    fn order_rejected(&mut self, order: &crate::dispatch::Order, stage: Stage) {
        let at = address_point(&self.book, order.address);
        let fence = self
            .airspace
            .covering(&at)
            .first()
            .and_then(|id| self.airspace.fences().iter().find(|f| f.id == *id))
            .cloned();
        let (fence_id, fence_name) = fence
            .as_ref()
            .map_or((None, String::new()), |f| (Some(f.id), f.name.clone()));
        let fence_text = match (&fence_id, fence_name.is_empty()) {
            (Some(id), true) => format!("geofence #{id}"),
            (Some(id), false) => format!("geofence #{id} ({fence_name})"),
            (None, _) => "a geofence".to_owned(),
        };
        let reason = format!(
            "the delivery address is inside {fence_text}{}",
            stage.since()
        );
        self.rejection_report(order, stage, fence_id.map(|id| (id, fence_name)), reason);
    }

    /// Report an order turned away, with why; `fence` is the one covering
    /// its address, if that was the reason.
    fn rejection_report(
        &mut self,
        order: &crate::dispatch::Order,
        stage: Stage,
        fence: Option<(u32, String)>,
        reason: String,
    ) {
        let at = address_point(&self.book, order.address);
        let address = &self.book.addresses[order.address as usize];
        let (fence_id, fence_name) = fence.map_or((None, String::new()), |(id, n)| (Some(id), n));
        let detail = serde_json::json!({
            "order": order.id,
            "address": order.address,
            "label": address.label,
            "depot": self.book.depots[usize::from(address.depot)],
            "ordered_tick": order.ordered_tick,
            "fence": fence_id,
            "fence_name": fence_name,
            "stage": stage.name(),
            "reason": reason,
        });
        self.report(Report {
            seq: 0,
            tick: self.tick,
            kind: ReportKind::OrderRejected,
            uid: format!("order-{}", order.id),
            observer: None,
            target: None,
            target_kind: String::new(),
            lat_e7: at.lat_e7,
            lon_e7: at.lon_e7,
            alt_mm: 0,
            detail,
        });
    }

    /// A drone crashed: put the order it was carrying back at the front of
    /// its depot's queue, and bring in what replaces it. A courier's duty
    /// goes to its depot's spare while one stands by; anything else (a
    /// courier once the spares are gone, a spare, a drone of a ground
    /// station's) is replaced by a new drone on the wreck's home pad, in the
    /// same role. A new drone has an agent id and a MAVLink system id nothing
    /// has had before. `None` once there are no system ids left.
    pub(crate) fn replace(&mut self, wreck: &DroneAgent) -> Option<Replacement> {
        let id = wreck.body.id;
        let operated = self.ops.remove(&id).is_some();
        if let Some(courier) = &wreck.courier {
            if let Some(order) = courier.order.clone().filter(|_| !courier.delivered) {
                self.return_order(order, courier.depot);
            }
        }
        // A crashed spare leaves the stand-by list; its replacement joins it.
        let spare_for = self.spares.iter().position(|s| s.contains(&id));
        if let Some(depot) = spare_for {
            self.spares[depot].retain(|s| *s != id);
        }

        if let Some(courier) = &wreck.courier {
            let depot = usize::from(courier.depot);
            while !self.spares[depot].is_empty() {
                let spare = self.spares[depot].remove(0);
                let Some(drone) = self
                    .record(spare)
                    .filter(|_| !self.retiring(spare))
                    .and_then(|r| Some((r.saved.drone.as_ref()?.sysid, r.saved.callsign.clone())))
                else {
                    continue;
                };
                self.ops.insert(spare, Ops::default());
                self.call_ups.insert(
                    spare,
                    Courier {
                        order: None,
                        delivered: false,
                        landed_tick: None,
                        ..courier.clone()
                    },
                );
                return Some(Replacement {
                    id: spare,
                    sysid: drone.0,
                    callsign: drone.1,
                    spare: true,
                });
            }
        }

        let sysid = u8::try_from(self.next_sysid)
            .ok()
            .filter(|s| *s <= HIGHEST_SYSID)?;
        let new_id = self.next_id;
        ordering(new_id).ok()?;
        let home = wreck.drone.home;
        let pose = Pose {
            lat_e7: home.lat_e7,
            lon_e7: home.lon_e7,
            alt_mm: home.alt_mm,
            ..Pose::default()
        };
        let mut body = self.body(new_id, Kind::Drone, pose).ok()?;
        body.callsign = successor(&wreck.body.callsign, sysid);
        self.next_sysid += 1;
        self.next_id += 1;
        let recruit = DroneAgent {
            body,
            drone: Drone::new(sysid, home, wreck.drone.cruise_cms),
            courier: wreck.courier.clone().map(|c| Courier {
                order: None,
                delivered: false,
                landed_tick: None,
                ..c
            }),
            survey: None,
            activity: "",
            waiting_for_slot: None,
            slot: None,
            wrecked: false,
        };
        if let Some(depot) = spare_for {
            self.spares[depot].push(new_id);
        }
        if operated {
            self.ops.insert(new_id, Ops::default());
        }
        let replacement = Replacement {
            id: new_id,
            sysid,
            callsign: recruit.body.callsign.clone(),
            spare: false,
        };
        // On the board from this step, on the schedule from the next.
        self.place(recruit.record());
        self.recruits.push(Member::Drone(Box::new(recruit)));
        Some(replacement)
    }

    // -------------------------------------------------------- run control

    /// Clear everything a run changes, ready to populate a schedule.
    fn begin(&mut self) {
        self.grid = new_grid(self.resolution, self.root);
        self.board = Arc::default();
        self.board_next.clear();
        self.occupied = 0;
        self.occupied_next.clear();
        self.dispatch = Dispatch::new(self.book.depots.len());
        self.seed = self.pack_seed;
        self.tick = 0;
        self.pinned = None;
        self.next_id = self.roster_next_id;
        self.spares = self.roster_spares.clone();
        self.next_sysid = self.roster_next_sysid;
        self.call_ups.clear();
        self.recruits.clear();
        self.airspace.clear();
        self.fence_mail.clear();
        self.scheduled.clear();
        self.previous.clear();
        self.stations = Stations::default();
        self.regions = Regions::default();
        self.site_mail.clear();
        self.set_weather(self.roster_weather.clone());
        self.ops = self
            .roster_ops
            .iter()
            .map(|id| (*id, Ops::default()))
            .collect();
        self.last_seen.clear();
        self.reports.clear();
        self.report_counts.clear();
        self.clear_pending();
        // The pack's own fences, open from the first tick.
        for (mut fence, minutes) in self.roster_fences.clone() {
            fence.id = self.airspace.take_id();
            fence.closes_tick = minutes.map(|m| self.ticks_for(m));
            self.fence_report(&fence, "open", "scenario");
            self.airspace.open(fence);
        }
        for mut station in self.roster_stations.clone() {
            station.id = self.stations.take_id();
            self.site_report(ReportKind::Station, &station, "add", station.point());
            self.stations.add(station);
        }
        for mut region in self.roster_regions.clone() {
            region.id = self.regions.take_id();
            self.site_report(ReportKind::Roi, &region, "add", region.centre());
            self.regions.add(region);
        }
    }

    fn clear_pending(&mut self) {
        self.slot_claims.clear();
        self.mail.clear();
        self.scratch.clear();
        self.arrivals.clear();
        self.retiring.clear();
    }

    /// Place `members` and the dispatcher on an empty schedule and commit.
    fn populate(&mut self, schedule: &mut Schedule, members: Vec<Member>) {
        schedule.schedule_repeating(Box::new(Dispatcher), 0.0, DISPATCHER_ORDER);
        for member in members {
            let order = ordering(member.body().id).expect("ids are checked on the way in");
            self.place(member.record());
            schedule.schedule_repeating(member.boxed(), 0.0, order);
        }
        self.commit();
        self.range_key = None;
        self.refresh_range();
    }

    /// Step once as `tick`. For embedders that keep their own clock.
    pub fn step_at(&mut self, schedule: &mut Schedule, tick: u64) {
        self.pinned = Some(tick);
        schedule.step(self);
    }

    /// Keep reports for [`World::drain_reports`]. On by default; a long run
    /// that nobody drains should turn it off.
    pub fn keep_reports(&mut self, keep: bool) {
        self.keep_reports = keep;
        if !keep {
            self.reports.clear();
        }
    }

    /// Also write every report to `sink` as a line of JSON.
    pub fn report_to(&mut self, sink: Box<dyn Write + Send + Sync>) {
        self.sink = Some(sink);
    }

    pub fn drain_reports(&mut self, out: &mut Vec<Report>) {
        out.append(&mut self.reports);
    }

    /// Reports raised this run, by kind.
    pub fn report_counts(&self) -> &BTreeMap<&'static str, u64> {
        &self.report_counts
    }

    pub fn flush_reports(&mut self) -> std::io::Result<()> {
        match self.sink.as_mut() {
            Some(sink) => sink.flush(),
            None => Ok(()),
        }
    }

    pub fn scenario_id(&self) -> &str {
        &self.id
    }

    /// The last tick stepped.
    pub fn tick(&self) -> u64 {
        self.tick
    }

    pub fn agent_count(&self) -> u64 {
        self.board.len() as u64
    }

    pub fn occupied_cells(&self) -> u64 {
        self.occupied
    }

    pub fn dispatch(&self) -> &Dispatch {
        &self.dispatch
    }

    pub fn book(&self) -> &AddressBook {
        &self.book
    }

    pub fn terrain(&self) -> &Terrain {
        &self.terrain
    }

    // ------------------------------------------------------------ weather

    /// Check a wind field for this world.
    fn weather_from(&self, value: &serde_json::Value) -> anyhow::Result<WindField> {
        let field = WindField::from_value(value)?;
        if field.resolution > self.resolution {
            anyhow::bail!(
                "weather at resolution {} is finer than the world's {}",
                field.resolution,
                self.resolution
            );
        }
        Ok(field)
    }

    /// The wind now, if there is any.
    pub fn weather(&self) -> Option<&WindField> {
        self.weather.as_deref()
    }

    /// The UTC time now in the simulation, Unix seconds, when there is
    /// weather to give the clock a start.
    pub fn sim_utc(&self) -> Option<f64> {
        let w = self.weather.as_ref()?;
        Some(w.utc_at(self.tick as f64 * self.settings.tuning.dt))
    }

    /// The wind in `cell` (or its ancestor at the weather resolution),
    /// `agl_m` above the ground, now. Calm without weather, or outside it.
    pub(crate) fn wind_at(&self, cell: GeoCell, agl_m: f64) -> Wind {
        let (Some(w), Some(utc)) = (self.weather.as_ref(), self.sim_utc()) else {
            return Wind::CALM;
        };
        w.wind_at(cell, agl_m, utc).unwrap_or(Wind::CALM)
    }

    /// The wind over a point, through the cell it is in.
    pub(crate) fn wind_over(&self, point: &Point, agl_m: f64) -> Wind {
        if self.weather.is_none() {
            return Wind::CALM;
        }
        self.cell_of(point.lat_e7, point.lon_e7)
            .map_or(Wind::CALM, |cell| self.wind_at(cell, agl_m))
    }

    /// Changes whenever the wind the planners see could: a new field, or
    /// the next frame.
    fn weather_epoch(&self) -> u64 {
        let (Some(w), Some(utc)) = (self.weather.as_ref(), self.sim_utc()) else {
            return 0;
        };
        let frame = ((utc - w.t0_unix as f64) / f64::from(w.step_s)).max(0.0) as u64;
        self.weather_rev ^ frame.wrapping_mul(0x9e37_79b9_7f4a_7c15)
    }

    fn set_weather(&mut self, weather: Option<Arc<WindField>>) {
        self.weather_rev = weather.as_ref().map_or(0, |w| w.rev());
        self.weather = weather;
    }

    // ------------------------------------------------------------ intents

    /// Check an intent against the committed world and queue it for the
    /// step that applies it.
    pub fn apply_intent(&mut self, intent: &Intent) -> Result<(), RejectReason> {
        if !self.contains(intent.cell) {
            return Err(RejectReason::OutOfBounds);
        }
        match intent.verb {
            IntentVerb::Custom(codes::WEATHER_SET) => {
                let field = self
                    .weather_from(&intent.payload)
                    .map_err(|e| rejected(e.to_string()))?;
                self.set_weather(Some(Arc::new(field)));
                Ok(())
            }
            IntentVerb::Custom(codes::GEOFENCE_OPEN) => self.open_fence(intent),
            IntentVerb::Custom(codes::GEOFENCE_CLOSE) => self.close_fence(intent),
            IntentVerb::Custom(codes::GEOFENCE_SET) => self.set_fences(intent),
            IntentVerb::Custom(codes::GEOFENCE_EXTEND) => self.extend_fence(intent),
            IntentVerb::Custom(codes::GEOFENCE_RENAME) => self.rename_fence(intent),
            IntentVerb::Custom(codes::STATION_ADD) => {
                let station = self.station_from(&intent.payload, &source_of(&intent.payload))?;
                self.site_mail.push(SiteChange::AddStation(station));
                Ok(())
            }
            IntentVerb::Custom(codes::STATION_REMOVE) => {
                let id = self.site_id(&intent.payload, "station", |w, id| {
                    w.stations.list.iter().any(|s| s.id == id)
                })?;
                self.site_mail.push(SiteChange::RemoveStation(id));
                Ok(())
            }
            IntentVerb::Custom(codes::ROI_ADD) => {
                let region = self.region_from(&intent.payload, &source_of(&intent.payload))?;
                self.site_mail.push(SiteChange::AddRegion(region));
                Ok(())
            }
            IntentVerb::Custom(codes::ROI_REMOVE) => {
                let id = self.site_id(&intent.payload, "region of interest", |w, id| {
                    w.regions.list.iter().any(|r| r.id == id)
                })?;
                self.site_mail.push(SiteChange::RemoveRegion(id));
                Ok(())
            }
            IntentVerb::Custom(code) => self.drone_intent(code, intent),
            IntentVerb::Spawn => self.spawn(intent),
            IntentVerb::Despawn => self.despawn(intent),
            IntentVerb::Move | IntentVerb::SetField => Err(rejected(
                "drones move by mission (codes 100–103); targets move on their own",
            )),
        }
    }

    fn drone_intent(&mut self, code: u16, intent: &Intent) -> Result<(), RejectReason> {
        let instruction = match code {
            codes::MISSION_UPLOAD => {
                let items: Vec<MissionItem> = intent
                    .payload
                    .get("items")
                    .map(|v| serde_json::from_value(v.clone()))
                    .transpose()
                    .map_err(|e| rejected(format!("mission items did not parse: {e}")))?
                    .ok_or_else(|| rejected("mission upload needs an items array"))?;
                self.check_mission(&items).map_err(rejected)?;
                Instruction::Upload(items)
            }
            codes::MISSION_CLEAR => Instruction::Clear,
            codes::MISSION_SET_CURRENT => {
                let seq = intent
                    .payload
                    .get("seq")
                    .and_then(serde_json::Value::as_u64)
                    .ok_or_else(|| rejected("set-current needs a seq"))?;
                Instruction::SetCurrent(seq.min(u64::from(u16::MAX)) as u16)
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
                Instruction::Command {
                    command: command.min(u64::from(u16::MAX)) as u16,
                    params,
                }
            }
            other => {
                return Err(rejected(format!(
                    "the drones model has no handler for code {other}"
                )))
            }
        };

        let agent = intent
            .agent
            .ok_or_else(|| rejected("drone commands need an agent id"))?;
        let record = match self.record(agent) {
            Some(record) if !self.retiring.contains(&agent) => record,
            _ => return Err(rejected(format!("no agent {agent}"))),
        };
        let Some(drone) = record.saved.drone.as_ref() else {
            return Err(rejected(format!(
                "agent {agent} is a {}, not a drone",
                record.kind.name()
            )));
        };
        // Run it on a copy of the drone as earlier instructions this step
        // will leave it. The agent applies the same instructions to the same
        // state, so what passes here cannot fail there.
        let mut copy = self
            .scratch
            .get(&agent)
            .cloned()
            .unwrap_or_else(|| drone.clone());
        instruction
            .apply(
                &mut copy,
                &record.saved.pose,
                &self.settings.tuning,
                &self.terrain,
            )
            .map_err(rejected)?;
        self.scratch.insert(agent, copy);
        self.mail.entry(agent).or_default().push(instruction);
        Ok(())
    }

    fn open_fence(&mut self, intent: &Intent) -> Result<(), RejectReason> {
        let p = &intent.payload;
        let shape = self.shape_from(p)?;
        let band = self.band_from(p, &shape)?;
        let minutes = fence_minutes(p, "minutes")?;
        let after = match p.get("after_minutes") {
            None | Some(serde_json::Value::Null) => 0.0,
            Some(v) => v
                .as_f64()
                .filter(|m| *m >= 0.0 && m.is_finite())
                .ok_or_else(|| rejected("after_minutes must be 0 or more"))?,
        };
        // Who asked, for the log: "dashboard", say. The scenario's and
        // ground stations' names are their own.
        let source = p
            .get("source")
            .and_then(|v| v.as_str())
            .filter(|s| {
                !s.is_empty()
                    && s.len() <= 24
                    && *s != SCENARIO_SOURCE
                    && *s != GCS_SOURCE
                    && s.bytes()
                        .all(|b| b.is_ascii_alphanumeric() || b == b'-' || b == b'_')
            })
            .unwrap_or("intent");
        let fence = Fence {
            id: self.airspace.take_id(),
            shape,
            band,
            name: fence_name(p),
            source: source.into(),
            opened_tick: 0,
            closes_tick: None,
        };
        self.fence_mail.push(if after > 0.0 {
            FenceChange::Schedule(fence, after, minutes)
        } else {
            FenceChange::Open(fence, minutes)
        });
        Ok(())
    }

    /// Replace the geofences ground stations have set with a whole fence
    /// plan, as a ground station uploads one. Fences in the plan that match
    /// one the scenario, the dashboard or another intent opened (a ground
    /// station sends back what it downloaded) are left as they are: they are
    /// not a ground station's to close. All or nothing: one bad fence
    /// refuses the plan.
    fn set_fences(&mut self, intent: &Intent) -> Result<(), RejectReason> {
        let list = intent
            .payload
            .get("fences")
            .and_then(|v| v.as_array())
            .ok_or_else(|| rejected("a fence plan needs a fences array"))?;
        let shapes = list
            .iter()
            .enumerate()
            .map(|(i, v)| {
                self.shape_from(v).map_err(|e| match e {
                    RejectReason::Rejected { message } => rejected(format!("fence {i}: {message}")),
                    other => other,
                })
            })
            .collect::<Result<Vec<Shape>, RejectReason>>()?;

        // The fences as they will stand once what is already queued applies.
        let mut current: Vec<(u32, Shape, bool)> = self
            .airspace
            .fences()
            .iter()
            .map(|f| (f.id, f.shape.clone(), f.source == GCS_SOURCE))
            .collect();
        for change in &self.fence_mail {
            match change {
                FenceChange::Open(f, _) => {
                    current.push((f.id, f.shape.clone(), f.source == GCS_SOURCE))
                }
                FenceChange::Schedule(..) | FenceChange::Extend(..) | FenceChange::Rename(..) => {}
                FenceChange::Close(id) => current.retain(|c| c.0 != *id),
            }
        }
        let wanted: Vec<Shape> = shapes
            .into_iter()
            .filter(|s| !current.iter().any(|(_, c, gcs)| !gcs && same_shape(c, s)))
            .collect();
        let mut kept = vec![false; wanted.len()];
        for (id, shape, gcs) in &current {
            if !gcs {
                continue;
            }
            match (0..wanted.len()).find(|i| !kept[*i] && same_shape(&wanted[*i], shape)) {
                Some(i) => kept[i] = true,
                None => self.fence_mail.push(FenceChange::Close(*id)),
            }
        }
        for (shape, kept) in wanted.into_iter().zip(kept) {
            if kept {
                continue;
            }
            let fence = Fence {
                id: self.airspace.take_id(),
                shape,
                band: Band::default(),
                name: String::new(),
                source: GCS_SOURCE.into(),
                opened_tick: 0,
                closes_tick: None,
            };
            self.fence_mail.push(FenceChange::Open(fence, None));
        }
        Ok(())
    }

    /// A fence's shape from `{"circle": {"lat", "lon", "radius_m"}}` or
    /// `{"polygon": [[lon, lat], ...]}`, in degrees, checked: inside the
    /// world, enclosing some area, and clear of every drone's pad.
    fn shape_from(&self, p: &serde_json::Value) -> Result<Shape, RejectReason> {
        let deg = |v: &serde_json::Value, k: &str| v.get(k).and_then(serde_json::Value::as_f64);
        let shape = if let Some(c) = p.get("circle") {
            let (Some(lat), Some(lon), Some(radius_m)) =
                (deg(c, "lat"), deg(c, "lon"), deg(c, "radius_m"))
            else {
                return Err(rejected("a circle needs lat, lon and radius_m"));
            };
            if !(1.0..=50_000.0).contains(&radius_m) {
                return Err(rejected("radius_m must be between 1 and 50000"));
            }
            let at = Point::at(lon, lat, 0.0);
            if !self.accepts(&at) {
                return Err(RejectReason::OutOfBounds);
            }
            Shape::Circle {
                lat_e7: at.lat_e7,
                lon_e7: at.lon_e7,
                radius_m,
            }
        } else if let Some(list) = p.get("polygon").and_then(|v| v.as_array()) {
            let mut points = Vec::new();
            for corner in list {
                let pair = corner.as_array().map(|c| (c.first(), c.get(1)));
                let Some((Some(lon), Some(lat))) = pair else {
                    return Err(rejected("polygon corners are [lon, lat] pairs"));
                };
                let (Some(lon), Some(lat)) = (lon.as_f64(), lat.as_f64()) else {
                    return Err(rejected("polygon corners are [lon, lat] pairs"));
                };
                let at = Point::at(lon, lat, 0.0);
                if !self.accepts(&at) {
                    return Err(RejectReason::OutOfBounds);
                }
                points.push([at.lat_e7, at.lon_e7]);
            }
            Shape::Polygon { points }
        } else {
            return Err(rejected("a geofence needs a circle or a polygon"));
        };
        if !self.airspace.usable(&shape) {
            return Err(rejected("the geofence encloses no area"));
        }
        if let Some(pad) = self
            .pads()
            .find(|pad| self.airspace.would_cover(&shape, pad))
        {
            return Err(rejected(format!(
                "the geofence would keep drones off the pad at {:.6}, {:.6}",
                pad.lat(),
                pad.lon()
            )));
        }
        Ok(shape)
    }

    /// A fence's band from `"floor_m"` and `"ceiling_m"` (absent or null:
    /// the surface, unlimited), metres above the ground, or above mean sea
    /// level with `"altitude_ref": "amsl"`. Heights above sea level are
    /// taken above the ground at the fence's centre.
    fn band_from(&self, p: &serde_json::Value, shape: &Shape) -> Result<Band, RejectReason> {
        let height = |key: &str| -> Result<Option<f64>, RejectReason> {
            match p.get(key) {
                None | Some(serde_json::Value::Null) => Ok(None),
                Some(v) => v
                    .as_f64()
                    .filter(|m| m.is_finite() && (-1_000.0..=20_000.0).contains(m))
                    .map(Some)
                    .ok_or_else(|| rejected(format!("{key} must be a height in metres"))),
            }
        };
        let (mut floor, mut ceiling) = (height("floor_m")?, height("ceiling_m")?);
        match p
            .get("altitude_ref")
            .and_then(|v| v.as_str())
            .unwrap_or("agl")
        {
            "agl" => {}
            "amsl" => {
                let centre = match shape {
                    Shape::Circle { lat_e7, lon_e7, .. } => (*lat_e7, *lon_e7),
                    Shape::Polygon { points } => {
                        let n = points.len().max(1) as i64;
                        let lat = points.iter().map(|q| i64::from(q[0])).sum::<i64>() / n;
                        let lon = points.iter().map(|q| i64::from(q[1])).sum::<i64>() / n;
                        (lat as i32, lon as i32)
                    }
                };
                let ground = f64::from(self.terrain.ground_mm(centre.0, centre.1)) / 1000.0;
                floor = floor.map(|f| f - ground);
                ceiling = ceiling.map(|c| c - ground);
            }
            other => {
                return Err(rejected(format!(
                    "altitude_ref is agl or amsl, not '{other}'"
                )))
            }
        }
        let floor = floor.map(|f| f.max(0.0)).filter(|f| *f > 0.0);
        if let Some(c) = ceiling {
            if c <= floor.unwrap_or(0.0) {
                return Err(rejected("ceiling_m must be above floor_m and the ground"));
            }
        }
        Ok(Band {
            floor_m: floor,
            ceiling_m: ceiling,
        })
    }

    fn extend_fence(&mut self, intent: &Intent) -> Result<(), RejectReason> {
        let id = intent
            .payload
            .get("id")
            .and_then(serde_json::Value::as_u64)
            .and_then(|id| u32::try_from(id).ok())
            .ok_or_else(|| rejected("extending a geofence needs its id"))?;
        if intent.payload.get("minutes").is_none() {
            return Err(rejected(
                "extending a geofence needs minutes: more than 0, or -1 until closed",
            ));
        }
        let minutes = fence_minutes(&intent.payload, "minutes")?;
        if !self.airspace.fences().iter().any(|f| f.id == id) {
            return Err(rejected(format!("no open geofence {id}")));
        }
        self.fence_mail.push(FenceChange::Extend(id, minutes));
        Ok(())
    }

    fn rename_fence(&mut self, intent: &Intent) -> Result<(), RejectReason> {
        let id = intent
            .payload
            .get("id")
            .and_then(serde_json::Value::as_u64)
            .and_then(|id| u32::try_from(id).ok())
            .ok_or_else(|| rejected("renaming a geofence needs its id"))?;
        if !intent
            .payload
            .get("name")
            .is_some_and(serde_json::Value::is_string)
        {
            return Err(rejected(
                "renaming a geofence needs a name (empty clears it)",
            ));
        }
        let known = self.airspace.fences().iter().any(|f| f.id == id)
            || self.scheduled.iter().any(|f| f.id == id)
            || self.previous.iter().any(|p| p.fence.id == id)
            || self.fence_mail.iter().any(|c| {
                matches!(c, FenceChange::Open(f, _) | FenceChange::Schedule(f, _, _) if f.id == id)
            });
        if !known {
            return Err(rejected(format!("no geofence {id}")));
        }
        self.fence_mail
            .push(FenceChange::Rename(id, fence_name(&intent.payload)));
        Ok(())
    }

    fn close_fence(&mut self, intent: &Intent) -> Result<(), RejectReason> {
        let id = intent
            .payload
            .get("id")
            .and_then(serde_json::Value::as_u64)
            .ok_or_else(|| rejected("closing a geofence needs its id"))?;
        let id = u32::try_from(id).map_err(|_| rejected(format!("no geofence {id}")))?;
        let open = self.airspace.fences().iter().any(|f| f.id == id)
            || self.scheduled.iter().any(|f| f.id == id);
        let pending = self.fence_mail.iter().any(
            |c| matches!(c, FenceChange::Open(f, _) | FenceChange::Schedule(f, _, _) if f.id == id),
        );
        if !open && !pending {
            return Err(rejected(format!("no geofence {id}")));
        }
        self.fence_mail.push(FenceChange::Close(id));
        Ok(())
    }

    /// A station from `{"lat", "lon"}` and optionally `"name"`, checked:
    /// inside the world, not under a geofence too low to fly beneath, and
    /// not on top of another station. Its id is given when it is added.
    fn station_from(&self, p: &serde_json::Value, source: &str) -> Result<Station, RejectReason> {
        let at = self.site_point(p, "a station")?;
        if self.airspace.covers(&at) {
            return Err(rejected("the station would be inside a geofence"));
        }
        let pending = self.site_mail.iter().filter_map(|c| match c {
            SiteChange::AddStation(s) => Some(s.point()),
            _ => None,
        });
        let taken = self.stations.list.iter().map(Station::point).chain(pending);
        if taken
            .into_iter()
            .any(|s| flight::ground_distance_m(&s, &at) <= 2.0 * mission::PAD_RADIUS_M)
        {
            return Err(rejected("there is a station there already"));
        }
        let slots = p
            .get("slots")
            .and_then(serde_json::Value::as_u64)
            .filter(|n| (1..=100).contains(n))
            .ok_or_else(|| rejected("a station needs slots: 1 to 100 launch and recovery slots"))?;
        Ok(Station {
            id: 0,
            name: fence_name(p),
            lat_e7: at.lat_e7,
            lon_e7: at.lon_e7,
            slots: slots as u32,
            source: source.to_owned(),
            added_tick: self.tick,
        })
    }

    /// A region of interest from `{"lat", "lon", "radius_m"}` and optionally
    /// `"name"`, checked. Its id is given when it is added.
    fn region_from(&self, p: &serde_json::Value, source: &str) -> Result<Region, RejectReason> {
        let at = self.site_point(p, "a region of interest")?;
        let radius_m = p
            .get("radius_m")
            .and_then(serde_json::Value::as_f64)
            .filter(|r| (10.0..=5_000.0).contains(r))
            .ok_or_else(|| rejected("radius_m must be between 10 and 5000"))?;
        Ok(Region {
            id: 0,
            name: fence_name(p),
            lat_e7: at.lat_e7,
            lon_e7: at.lon_e7,
            radius_m,
            source: source.to_owned(),
            added_tick: self.tick,
            surveys: 0,
            last_surveyed_tick: None,
            assigned: Vec::new(),
        })
    }

    fn site_point(&self, p: &serde_json::Value, what: &str) -> Result<Point, RejectReason> {
        let deg = |k: &str| p.get(k).and_then(serde_json::Value::as_f64);
        let (Some(lat), Some(lon)) = (deg("lat"), deg("lon")) else {
            return Err(rejected(format!("{what} needs lat and lon")));
        };
        let at = Point::at(lon, lat, 0.0);
        if !self.accepts(&at) {
            return Err(RejectReason::OutOfBounds);
        }
        Ok(at)
    }

    /// The id in `{"id": n}`, of a station or region that exists or is about
    /// to.
    fn site_id(
        &self,
        p: &serde_json::Value,
        what: &str,
        exists: impl Fn(&World, u32) -> bool,
    ) -> Result<u32, RejectReason> {
        let id = p
            .get("id")
            .and_then(serde_json::Value::as_u64)
            .and_then(|id| u32::try_from(id).ok())
            .ok_or_else(|| rejected(format!("removing a {what} needs its id")))?;
        if !exists(self, id) {
            return Err(rejected(format!("no {what} {id}")));
        }
        Ok(id)
    }

    /// A pack's `world/<key>.json`: a list of sites, each read by `read`.
    fn pack_sites<T>(
        &self,
        fields: &serde_json::Value,
        key: &str,
        read: impl Fn(&World, &serde_json::Value) -> Result<T, RejectReason>,
    ) -> anyhow::Result<Vec<T>> {
        let Some(list) = fields.get(key) else {
            return Ok(Vec::new());
        };
        let list = list
            .as_array()
            .ok_or_else(|| anyhow::anyhow!("world/{key}.json must be a list"))?;
        list.iter()
            .enumerate()
            .map(|(i, v)| {
                read(self, v)
                    .map_err(|e| anyhow::anyhow!("world/{key}.json entry {i}: {}", reason_text(&e)))
            })
            .collect()
    }

    fn site_report(&mut self, kind: ReportKind, site: &impl Serialize, action: &str, at: Point) {
        let mut detail = serde_json::to_value(site).unwrap_or_default();
        detail["action"] = action.into();
        let id = detail["id"].as_u64().unwrap_or(0);
        let what = if kind == ReportKind::Station {
            "station"
        } else {
            "roi"
        };
        self.report(Report {
            seq: 0,
            tick: self.tick,
            kind,
            uid: format!("{what}-{id}-{action}"),
            observer: None,
            target: None,
            target_kind: String::new(),
            lat_e7: at.lat_e7,
            lon_e7: at.lon_e7,
            alt_mm: 0,
            detail,
        });
    }

    /// Add and remove the stations and regions asked for by intent. First
    /// thing in a step, with the geofences.
    pub(crate) fn update_sites(&mut self) {
        for change in std::mem::take(&mut self.site_mail) {
            match change {
                SiteChange::AddStation(mut station) => {
                    station.id = self.stations.take_id();
                    station.added_tick = self.tick;
                    self.site_report(ReportKind::Station, &station, "add", station.point());
                    self.stations.add(station);
                }
                SiteChange::RemoveStation(id) => {
                    if let Some(station) = self.stations.remove(id) {
                        self.site_report(ReportKind::Station, &station, "remove", station.point());
                    }
                }
                SiteChange::AddRegion(mut region) => {
                    region.id = self.regions.take_id();
                    region.added_tick = self.tick;
                    self.site_report(ReportKind::Roi, &region, "add", region.centre());
                    self.regions.add(region);
                }
                SiteChange::RemoveRegion(id) => {
                    if let Some(region) = self.regions.remove(id) {
                        self.site_report(ReportKind::Roi, &region, "remove", region.centre());
                    }
                }
            }
        }
    }

    /// Leave a region of interest where a drone crashed, so another goes to
    /// look. None if `crash_roi_radius_m` is 0.
    pub(crate) fn crash_region(&mut self, at: &Point, callsign: &str) -> Option<u32> {
        let radius_m = self.settings.crash_roi_radius_m;
        if radius_m <= 0.0 {
            return None;
        }
        let region = Region {
            id: self.regions.take_id(),
            name: format!("Crash of {callsign}"),
            lat_e7: at.lat_e7,
            lon_e7: at.lon_e7,
            radius_m,
            source: CRASH_SOURCE.into(),
            added_tick: self.tick,
            surveys: 0,
            last_surveyed_tick: None,
            assigned: Vec::new(),
        };
        let id = region.id;
        self.site_report(ReportKind::Roi, &region, "crash", region.centre());
        self.regions.add(region);
        Some(id)
    }

    /// Stations a drone may stop at now: those not under a geofence it
    /// could not get down beneath.
    pub(crate) fn usable_stations(&self) -> Vec<Point> {
        self.stations
            .list
            .iter()
            .map(Station::point)
            .filter(|p| !self.airspace.covers(p))
            .collect()
    }

    pub fn stations(&self) -> &Stations {
        &self.stations
    }

    pub fn regions(&self) -> &Regions {
        &self.regions
    }

    /// Every drone's home pad: those the pack starts with (so a pack's own
    /// fences are checked before the run begins) and any since.
    fn pads(&self) -> impl Iterator<Item = Point> + '_ {
        let roster = self.roster.iter().filter_map(|m| match m {
            Member::Drone(d) => Some(d.drone.home),
            _ => None,
        });
        let board = self
            .board
            .iter()
            .filter_map(|r| r.saved.drone.as_ref().map(|d| d.home));
        roster.chain(board)
    }

    /// Ticks in `minutes` of simulated time, at least one.
    fn ticks_for(&self, minutes: f64) -> u64 {
        (libm::round(minutes * 60.0 / self.settings.tuning.dt) as u64).max(1)
    }

    /// The fences a pack starts with: `world/geofences.json`, a list of
    /// shapes as [`codes::GEOFENCE_OPEN`] takes them, each with an optional
    /// `"name"` and `"minutes"`.
    fn pack_fences(&self, fields: &serde_json::Value) -> anyhow::Result<Vec<(Fence, Option<f64>)>> {
        let Some(list) = fields.get("geofences") else {
            return Ok(Vec::new());
        };
        let list = list
            .as_array()
            .ok_or_else(|| anyhow::anyhow!("world/geofences.json must be a list of fences"))?;
        list.iter()
            .enumerate()
            .map(|(i, v)| {
                let shape = self.shape_from(v).map_err(|e| {
                    anyhow::anyhow!("world/geofences.json fence {i}: {}", reason_text(&e))
                })?;
                let minutes = fence_minutes(v, "minutes").map_err(|e| {
                    anyhow::anyhow!("world/geofences.json fence {i}: {}", reason_text(&e))
                })?;
                let band = self.band_from(v, &shape).map_err(|e| {
                    anyhow::anyhow!("world/geofences.json fence {i}: {}", reason_text(&e))
                })?;
                let fence = Fence {
                    id: 0,
                    shape,
                    band,
                    name: fence_name(v),
                    source: SCENARIO_SOURCE.into(),
                    opened_tick: 0,
                    closes_tick: None,
                };
                Ok((fence, minutes))
            })
            .collect()
    }

    /// Remember a fence that closed, to open again later.
    fn remember(&mut self, fence: Fence, how: &str) {
        if self.previous.len() >= MAX_PREVIOUS_FENCES {
            self.previous.pop_front();
        }
        self.previous.push_back(Previous {
            fence,
            closed_tick: self.tick,
            closed: how.to_owned(),
        });
    }

    fn fence_report(&mut self, fence: &Fence, action: &str, reason: &str) {
        let mut detail = serde_json::to_value(fence).unwrap_or_default();
        detail["action"] = action.into();
        detail["reason"] = reason.into();
        let at = fence.centre();
        self.report(Report {
            seq: 0,
            tick: self.tick,
            kind: ReportKind::Geofence,
            uid: format!("fence-{}-{action}", fence.id),
            observer: None,
            target: None,
            target_kind: String::new(),
            lat_e7: at.lat_e7,
            lon_e7: at.lon_e7,
            alt_mm: 0,
            detail,
        });
    }

    /// Open and close the geofences due this tick: those whose time is up,
    /// and those asked for by intent. First thing in a step, so every drone
    /// sees the same airspace.
    pub(crate) fn update_airspace(&mut self) {
        let tick = self.tick;
        for fence in self.airspace.expire(tick) {
            self.fence_report(&fence, "close", "expired");
            self.remember(fence, "expired");
        }
        for change in std::mem::take(&mut self.fence_mail) {
            match change {
                FenceChange::Open(mut fence, minutes) => {
                    fence.opened_tick = tick;
                    fence.closes_tick = minutes.map(|m| tick + self.ticks_for(m));
                    self.fence_report(&fence, "open", "requested");
                    self.airspace.open(fence);
                }
                FenceChange::Schedule(mut fence, after, minutes) => {
                    fence.opened_tick = tick + self.ticks_for(after);
                    fence.closes_tick = minutes.map(|m| fence.opened_tick + self.ticks_for(m));
                    self.fence_report(&fence, "schedule", "requested");
                    let at = self
                        .scheduled
                        .partition_point(|f| (f.opened_tick, f.id) < (fence.opened_tick, fence.id));
                    self.scheduled.insert(at, fence);
                }
                FenceChange::Rename(id, name) => {
                    if let Some(fence) = self.airspace.rename(id, &name) {
                        self.fence_report(&fence, "rename", "requested");
                    } else if let Some(f) = self.scheduled.iter_mut().find(|f| f.id == id) {
                        f.name = name;
                        let fence = f.clone();
                        self.fence_report(&fence, "rename", "requested");
                    } else if let Some(p) = self.previous.iter_mut().find(|p| p.fence.id == id) {
                        p.fence.name = name;
                    }
                }
                FenceChange::Extend(id, minutes) => {
                    let closes = minutes.map(|m| tick + self.ticks_for(m));
                    if let Some(fence) = self.airspace.set_closes(id, closes) {
                        self.fence_report(&fence, "extend", "requested");
                    }
                }
                FenceChange::Close(id) => {
                    if let Some(fence) = self.airspace.close(id) {
                        self.fence_report(&fence, "close", "requested");
                        self.remember(fence, "closed");
                    } else if let Some(at) = self.scheduled.iter().position(|f| f.id == id) {
                        let fence = self.scheduled.remove(at);
                        self.fence_report(&fence, "cancel", "requested");
                        self.remember(fence, "cancelled");
                    }
                }
            }
        }
        // Those whose time has come.
        while self
            .scheduled
            .first()
            .is_some_and(|f| f.opened_tick <= tick)
        {
            let fence = self.scheduled.remove(0);
            self.fence_report(&fence, "open", "scheduled");
            self.airspace.open(fence);
        }
        // A fence opened this tick (by request, on schedule, or in a ground
        // station's plan) turns away the queued orders it covers.
        if self.airspace.fences().iter().any(|f| f.opened_tick == tick) {
            self.reject_newly_fenced();
        }
    }

    /// Keep a drone the simulation flies clear of the geofences, the way a
    /// ground station would: what it sends joins the drone's mail, after any
    /// commands that arrived since the last tick, and is obeyed with them.
    /// Its own committed state is what it plans from.
    pub(crate) fn operate_drone(
        &mut self,
        id: AgentId,
        callsign: &str,
        drone: &Drone,
        pose: &Pose,
        courier: Option<&Courier>,
    ) {
        let Some(ops) = self.ops.get(&id) else {
            return;
        };
        if self.airspace.is_empty() && ops.is_clean() && ops.checked.is_none() {
            return;
        }
        let airborne = !drone.grounded(pose, &self.terrain);
        let address = courier
            .filter(|c| !c.delivered)
            .and_then(|c| c.order.as_ref())
            .map(|o| address_point(&self.book, o.address));
        let mut ops = ops.clone();
        let orders = operator::orders(
            drone,
            pose,
            airborne,
            address,
            &mut ops,
            &self.airspace,
            &self.settings.tuning,
            &self.terrain,
            self.settings.max_mission_items,
        );
        self.ops.insert(id, ops);
        let Some(orders) = orders else { return };
        self.report(Report {
            seq: 0,
            tick: self.tick,
            kind: ReportKind::Reroute,
            uid: format!("reroute-{id}-{}", self.tick),
            observer: Some(id),
            target: None,
            target_kind: String::new(),
            lat_e7: pose.lat_e7,
            lon_e7: pose.lon_e7,
            alt_mm: pose.alt_mm,
            detail: serde_json::json!({
                "sysid": drone.sysid,
                "callsign": callsign,
                "reason": orders.reason,
                "messages": orders.messages,
                "detours": orders.detours,
                "airspace_rev": self.airspace.rev(),
            }),
        });
        self.mail.entry(id).or_default().extend(orders.instructions);
    }

    /// What the published view carries beside the agents: the geofences
    /// open now (what the MAVLink gateway serves as the fence plan) and the
    /// addresses of open orders, `[lat_e7, lon_e7, in_flight]`, waiting ones
    /// first, at most [`MAX_PUBLISHED_ORDERS`]. The dashboard draws both.
    pub fn overlays(&self) -> serde_json::Value {
        let at = |o: &crate::dispatch::Order| {
            let a = &self.book.addresses[o.address as usize];
            (a.lat_e7, a.lon_e7)
        };
        let waiting = self.dispatch.queues.iter().flatten().map(|o| (at(o), 0));
        let flying = self
            .board
            .iter()
            .filter_map(|r| r.saved.courier.as_ref())
            .filter(|c| !c.delivered)
            .filter_map(|c| c.order.as_ref())
            .map(|o| (at(o), 1));
        let orders: Vec<[i32; 3]> = waiting
            .chain(flying)
            .take(MAX_PUBLISHED_ORDERS)
            .map(|((lat, lon), state)| [lat, lon, state])
            .collect();
        serde_json::json!({
            "geofences": {
                "rev": self.airspace.rev(),
                "margin_m": self.airspace.margin_m,
                "fences": self.airspace.fences(),
                "scheduled": self.scheduled,
                "previous": self.previous.iter().rev().collect::<Vec<_>>(),
            },
            "orders": orders,
            "stations": self.stations.list.iter().map(|s| {
                let mut v = serde_json::to_value(s).unwrap_or_default();
                v["drones"] = self.drones_at(&s.point()).into();
                v["occupied"] = self.slots_taken(s, None).into();
                v["waiting"] = self.waiting_for(s.id).into();
                v
            }).collect::<Vec<_>>(),
            "rois": self.regions.list,
            "range": self.range,
            "weather": self.weather.as_ref().map(|w| serde_json::json!({
                "rev": self.weather_rev.to_string(),
                "mode": w.mode,
                "resolution": w.resolution,
                "start_unix": w.start_unix,
                "cells": w.cells.len(),
                "utc": self.sim_utc(),
            })),
        })
    }

    /// The behaviour tree every drone in this world thinks with.
    pub fn brain(&self) -> &Brain {
        self.brain.as_ref().expect("a world always has a brain")
    }

    /// Slots taken at a station: drones holding one, and anything else down
    /// on it, as of the last step, and the slots taken this step. `but` is
    /// left out of the count.
    pub(crate) fn slots_taken(&self, station: &Station, but: Option<AgentId>) -> u32 {
        let at = station.point();
        let committed = self
            .board
            .iter()
            .filter(|r| Some(r.id) != but)
            .filter(|r| {
                r.saved.slot == Some(station.id)
                    || r.saved.drone.as_ref().is_some_and(|d| {
                        flight::ground_distance_m(&r.saved.pose.point(), &at)
                            <= mission::PAD_RADIUS_M
                            && d.grounded(&r.saved.pose, &self.terrain)
                    })
            })
            .count() as u32;
        committed + self.slot_claims.get(&station.id).copied().unwrap_or(0)
    }

    /// Drones holding off a station until one of its slots is free.
    fn waiting_for(&self, station: u32) -> usize {
        self.board
            .iter()
            .filter(|r| r.saved.waiting_for_slot == Some(station))
            .count()
    }

    /// Work out again how far the couriers can deliver, if the stations, the
    /// airspace (which stations can be used) or the wind have changed.
    pub(crate) fn refresh_range(&mut self) {
        let key = (self.stations.rev, self.airspace.rev(), self.weather_epoch());
        if self.range_key == Some(key) {
            return;
        }
        self.range_key = Some(key);
        self.range = self.delivery_range();
    }

    /// How far the couriers at each depot can deliver, from its home pad,
    /// by way of stations as they plan trips: leaving home charged to the
    /// launch level (full if there is none), winching at the cruise height,
    /// with the courier there that reaches furthest. Each depot's `areas`
    /// are ellipses (`a` and `b` its foci, `[lat_e7, lon_e7]`, and `sum_m`
    /// the most their distances to a point may add up to), and the reach
    /// is everywhere inside any of them; `reach_m` is the furthest from
    /// home, `direct_m` the furthest without stopping. Geofences are not
    /// subtracted. A null `reach_m` means anywhere.
    pub fn delivery_range(&self) -> serde_json::Value {
        let tuning = self.settings.tuning;
        let ds = self.settings.dispatch;
        let charge = if tuning.launch_ppm > 0 {
            f64::from(tuning.launch_ppm)
        } else {
            1_000_000.0
        };
        let stations = self.usable_stations();
        let e7 = |p: &Point| [p.lat_e7, p.lon_e7];
        let hubs: Vec<serde_json::Value> = (0..self.book.depots.len())
            .filter_map(|depot| {
                let best = self
                    .board
                    .iter()
                    .filter_map(|r| {
                        let c = r.saved.courier.as_ref()?;
                        (usize::from(c.depot) == depot).then_some((r.saved.drone.as_ref()?, c))
                    })
                    .map(|(d, c)| {
                        let wind = self.wind_over(&d.home, f64::from(c.cruise_agl_m));
                        let range = Range::new(&tuning, d.cruise_cms).with_wind(&wind);
                        let climb = f64::from(c.cruise_agl_m + ds.takeoff_m);
                        let work = f64::from(tuning.winch.hold_ms(
                            mission::WINCH_DELIVER,
                            0.0,
                            0.0,
                            f64::from(c.cruise_agl_m),
                        )) / 1000.0;
                        (range.reach_m(charge, climb, work), d, range, climb, work)
                    })
                    .max_by(|a, b| a.0.total_cmp(&b.0))?;
                let (direct, drone, range, climb_m, work) = best;
                let planner = Planner {
                    range,
                    stations: stations.clone(),
                    climb_m,
                    home: &drone.home,
                    charge,
                };
                let (areas, reach) = match planner.extent(work) {
                    None => (Vec::new(), None),
                    Some(areas) => {
                        let reach = areas
                            .iter()
                            .map(|a| a.farthest_m(&drone.home))
                            .fold(0.0, f64::max);
                        (areas, Some(libm::round(reach)))
                    }
                };
                Some(serde_json::json!({
                    "depot": self.book.depots[depot],
                    "home": e7(&drone.home),
                    "direct_m": direct.is_finite().then(|| libm::round(direct / 2.0)),
                    "reach_m": reach,
                    "areas": areas.iter().map(|a| serde_json::json!({
                        "a": e7(&a.a), "b": e7(&a.b), "sum_m": libm::round(a.sum_m),
                    })).collect::<Vec<_>>(),
                }))
            })
            .collect();
        serde_json::json!({
            "stations_rev": self.stations.rev,
            "airspace_rev": self.airspace.rev(),
            "hubs": hubs,
        })
    }

    /// Drones on the ground on the pad at `point`.
    fn drones_at(&self, point: &Point) -> usize {
        self.board
            .iter()
            .filter_map(|r| Some((r.saved.drone.as_ref()?, &r.saved.pose)))
            .filter(|(d, pose)| {
                flight::ground_distance_m(&pose.point(), point) <= mission::PAD_RADIUS_M
                    && d.grounded(pose, &self.terrain)
            })
            .count()
    }

    pub fn airspace(&self) -> &Airspace {
        &self.airspace
    }

    fn spawn(&mut self, intent: &Intent) -> Result<(), RejectReason> {
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
        let mut pose = Pose::at(lon, lat, 0.0);
        pose.alt_mm = self.terrain.ground_mm(pose.lat_e7, pose.lon_e7);
        let first = self.next_id;
        if ordering(first + count - 1).is_err() {
            return Err(rejected("this world has run out of agent ids"));
        }
        let mut members = Vec::new();
        for id in first..first + count {
            let mut body = self
                .body(id, kind, pose)
                .map_err(|_| RejectReason::OutOfBounds)?;
            members.push(if kind == Kind::Poi {
                body.callsign = intent
                    .payload
                    .get("name")
                    .and_then(|v| v.as_str())
                    .map_or_else(|| format!("POI {id}"), str::to_owned);
                Member::Poi(PoiAgent { body })
            } else {
                let vehicle_type = match intent.payload.get("type").and_then(|v| v.as_str()) {
                    Some(t) if kind == Kind::Vehicle => Some(
                        VehicleType::parse(t)
                            .ok_or_else(|| rejected(format!("no vehicle type '{t}'")))?,
                    ),
                    _ => self.vehicle_type(kind, id),
                };
                Member::Ground(GroundAgent {
                    body,
                    vehicle_type,
                    wander: self.wander_from(kind, pose.point()),
                })
            });
        }
        self.next_id = first + count;
        self.arrivals.extend(members);
        Ok(())
    }

    fn despawn(&mut self, intent: &Intent) -> Result<(), RejectReason> {
        match intent.agent {
            Some(agent) => {
                if let Some(at) = self.arrivals.iter().position(|m| m.body().id == agent) {
                    self.arrivals.remove(at);
                    return Ok(());
                }
                if self.record(agent).is_none() || !self.retiring.insert(agent) {
                    return Err(rejected(format!("no agent {agent}")));
                }
                self.mail.remove(&agent);
                self.scratch.remove(&agent);
            }
            None => {
                for agent in self
                    .grid
                    .get_objects(&to_a5(intent.cell))
                    .unwrap_or_default()
                {
                    self.retiring.insert(agent);
                    self.mail.remove(&agent);
                    self.scratch.remove(&agent);
                }
                self.arrivals.retain(|m| m.body().cell != intent.cell);
            }
        }
        Ok(())
    }

    // --------------------------------------------------------- publishing

    /// Write the committed occupancy into `out`, sorted by cell and by agent
    /// id within each cell, stopping after `limit` cells. Returns how many
    /// cells were left out.
    pub fn snapshot(&self, out: &mut Vec<CellBag>, limit: usize) -> u64 {
        self.committed().snapshot(out, limit)
    }

    // -------------------------------------------------------- checkpoints

    /// State that does not live in the agents.
    pub fn fields(&self) -> serde_json::Value {
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
            "spares": self.spares,
            "next_sysid": self.next_sysid,
            "call_ups": self.call_ups.iter().collect::<Vec<_>>(),
            "airspace": self.airspace.saved(),
            "scheduled_fences": self.scheduled,
            "previous_fences": self.previous,
            "ops": self.ops.iter().collect::<Vec<_>>(),
            "stations": self.stations,
            "regions": self.regions,
            "weather": self.weather.as_deref(),
        })
    }

    pub fn restore_fields(&mut self, fields: &serde_json::Value) -> anyhow::Result<()> {
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
        if let Some(spares) = fields.get("spares") {
            self.spares = serde_json::from_value(spares.clone())?;
        }
        if let Some(next) = fields.get("next_sysid").and_then(serde_json::Value::as_u64) {
            self.next_sysid = self.next_sysid.max(next.min(u64::from(u16::MAX)) as u16);
        }
        if let Some(list) = fields.get("call_ups") {
            let list: Vec<(AgentId, Courier)> = serde_json::from_value(list.clone())?;
            self.call_ups = list.into_iter().collect();
        }
        if let Some(previous) = fields.get("previous_fences") {
            self.previous = serde_json::from_value(previous.clone())?;
        }
        if let Some(scheduled) = fields.get("scheduled_fences") {
            self.scheduled = serde_json::from_value(scheduled.clone())?;
        }
        if let Some(airspace) = fields.get("airspace") {
            let saved: SavedAirspace = serde_json::from_value(airspace.clone())?;
            self.airspace.restore(saved);
        }
        if let Some(list) = fields.get("ops") {
            let list: Vec<(AgentId, Ops)> = serde_json::from_value(list.clone())?;
            self.ops = list.into_iter().collect();
        }
        if let Some(stations) = fields.get("stations") {
            self.stations = serde_json::from_value(stations.clone())?;
        }
        if let Some(regions) = fields.get("regions") {
            self.regions = serde_json::from_value(regions.clone())?;
        }
        if let Some(list) = fields.get("last_seen") {
            let pairs: Vec<[u64; 3]> = serde_json::from_value(list.clone())?;
            self.last_seen = pairs.into_iter().map(|[o, t, at]| ((o, t), at)).collect();
        }
        match fields.get("weather") {
            // A checkpoint from before weather existed keeps the pack's.
            None => {}
            Some(serde_json::Value::Null) => self.set_weather(None),
            Some(v) => {
                let field = self.weather_from(v)?;
                self.set_weather(Some(Arc::new(field)));
            }
        }
        self.refresh_range();
        Ok(())
    }

    /// Every draw comes from a (seed, entity, tick) stream, so the seed is the
    /// whole generator state.
    pub fn rng_state(&self) -> u64 {
        self.seed
    }

    pub fn set_rng_state(&mut self, state: u64) {
        self.seed = state;
    }

    /// Replace every agent with the ones in `cells`, on `schedule`, which
    /// should be new. The order queues, sightings and seed are left for
    /// [`World::restore_fields`].
    pub fn restore(&mut self, cells: Vec<CellBag>, schedule: &mut Schedule) -> anyhow::Result<()> {
        let mut members = Vec::new();
        for bag in cells {
            if !self.contains(bag.cell) {
                anyhow::bail!(
                    "checkpoint cell {} does not belong to this world (resolution {}, root {:?})",
                    bag.cell,
                    self.resolution,
                    self.root
                );
            }
            for object in bag.objects {
                let id = object.id;
                let kind = Kind::parse(&object.kind).ok_or_else(|| {
                    anyhow::anyhow!("checkpoint agent {id} has unknown kind '{}'", object.kind)
                })?;
                let saved: Saved = serde_json::from_value(object.state)
                    .map_err(|e| anyhow::anyhow!("checkpoint agent {id} did not parse: {e}"))?;
                ordering(id)?;
                let member = Member::from_record(Record {
                    id,
                    kind,
                    cell: bag.cell,
                    saved,
                })
                .map_err(|e| anyhow::anyhow!("checkpoint agent {id}: {e}"))?;
                members.push(member);
            }
        }
        members.sort_by_key(|m| m.body().id);

        self.grid = new_grid(self.resolution, self.root);
        self.board = Arc::default();
        self.board_next.clear();
        self.occupied_next.clear();
        self.reports.clear();
        self.clear_pending();
        self.call_ups.clear();
        self.recruits.clear();
        self.fence_mail.clear();
        self.site_mail.clear();
        for member in &members {
            self.next_id = self.next_id.max(member.body().id + 1);
            if let Member::Drone(d) = member {
                self.next_sysid = self.next_sysid.max(u16::from(d.drone.sysid) + 1);
            }
        }
        self.populate(schedule, members);
        Ok(())
    }
}

/// The agents a world committed at the end of a step, detached from it.
///
/// Each agent's record carries the cell it placed itself in on the grid, so
/// this is the grid's committed occupancy, with every agent's state beside it.
#[derive(Debug, Clone)]
pub struct Committed {
    board: Arc<Vec<Record>>,
}

impl Committed {
    pub fn records(&self) -> &[Record] {
        &self.board
    }

    /// As [`World::snapshot`], for the step this was taken after.
    pub fn snapshot(&self, out: &mut Vec<CellBag>, limit: usize) -> u64 {
        out.clear();
        let mut order: Vec<(GeoCell, usize)> = self
            .board
            .iter()
            .enumerate()
            .map(|(at, r)| (r.cell, at))
            .collect();
        // The board is in id order, so within a cell this is too.
        order.sort_unstable();

        let mut omitted = 0;
        for group in order.chunk_by(|a, b| a.0 == b.0) {
            if out.len() >= limit {
                omitted += 1;
                continue;
            }
            let objects = group
                .iter()
                .map(|(_, at)| {
                    let record = &self.board[*at];
                    WorldObject {
                        id: record.id,
                        kind: record.kind.name().to_owned(),
                        state: serde_json::to_value(&record.saved).unwrap_or_default(),
                    }
                })
                .collect();
            out.push(CellBag {
                cell: group[0].0,
                objects,
            });
        }
        omitted
    }
}

impl State for World {
    fn init(&mut self, schedule: &mut Schedule) {
        self.begin();
        let roster = self.roster.clone();
        self.populate(schedule, roster);
    }

    fn as_any_mut(&mut self) -> &mut dyn Any {
        self
    }

    fn as_any(&self) -> &dyn Any {
        self
    }

    fn as_state_mut(&mut self) -> &mut dyn State {
        self
    }

    fn as_state(&self) -> &dyn State {
        self
    }

    fn reset(&mut self) {
        self.begin();
    }

    fn update(&mut self, _step: u64) {}

    fn before_step(&mut self, schedule: &mut Schedule) {
        self.tick = self.pinned.take().unwrap_or(self.tick + 1);
        self.arrivals.append(&mut self.recruits);
        if self.arrivals.is_empty() {
            return;
        }
        // Agents already on the schedule run at its next time; arrivals join
        // them so they step this tick.
        let now = if schedule.step == 0 {
            0.0
        } else {
            schedule.time + 1.0
        };
        for member in std::mem::take(&mut self.arrivals) {
            let order = ordering(member.body().id).expect("checked when spawned");
            schedule.schedule_repeating(member.boxed(), now, order);
        }
    }

    fn after_step(&mut self, _schedule: &mut Schedule) {
        self.commit();
        self.clear_pending();
    }
}

/// What took over from a crashed drone.
#[derive(Debug, Clone, PartialEq)]
pub(crate) struct Replacement {
    pub(crate) id: AgentId,
    pub(crate) sysid: u8,
    pub(crate) callsign: String,
    /// A spare that was standing by, rather than a new drone.
    pub(crate) spare: bool,
}

/// A new drone's callsign: the wreck's, with its trailing number (if any)
/// swapped for the new system id. `MEGA-7` is followed by `MEGA-43`,
/// `MEGA-SPARE` by `MEGA-SPARE-44`.
fn successor(callsign: &str, sysid: u8) -> String {
    let stem = match callsign.rsplit_once('-') {
        Some((stem, n)) if !n.is_empty() && n.bytes().all(|b| b.is_ascii_digit()) => stem,
        _ => callsign,
    };
    if stem.is_empty() {
        format!("UAV-{sysid}")
    } else {
        format!("{stem}-{sysid}")
    }
}

fn lowest_free(used: &BTreeSet<u8>) -> anyhow::Result<u8> {
    (1..=HIGHEST_SYSID)
        .find(|id| !used.contains(id))
        .ok_or_else(|| anyhow::anyhow!("all {HIGHEST_SYSID} MAVLink system ids are taken"))
}
