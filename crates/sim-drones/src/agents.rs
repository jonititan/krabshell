//! One krABMaga agent per drone, intruder, person, vehicle and point of
//! interest, plus the dispatcher that places delivery orders.
//!
//! Each agent owns its state. On every step it reads the [`World`]'s
//! committed side (where everything was at the end of the last step), acts,
//! and writes itself back to the uncommitted side: its cell to the grid and
//! its full state to the board.

use krabmaga::engine::agent::Agent;
use krabmaga::engine::state::State;
use sim_core::{AgentId, GeoCell, Report, ReportKind};

use crate::dispatch::{self, AddressBook, Courier, Order};
use crate::flight::Pose;
use crate::mission::{self, Drone, Event, Mode};
use crate::sensor::{Candidate, Sighting};
use crate::settings::Settings;
use crate::targets::{Route, Wander};
use crate::terrain::Terrain;
use crate::world::{
    draw_pair, refresh_key, Bounds, Instruction, Kind, Record, Saved, World, WANDER_SALT,
};

fn world(state: &mut dyn State) -> &mut World {
    state
        .as_any_mut()
        .downcast_mut::<World>()
        .expect("drones agents step in a drones World")
}

/// What an agent reads while it works alone: nothing another agent writes
/// during a step. Shared across a [`crate::crowd::Crowd`]'s threads.
#[derive(Clone, Copy)]
pub(crate) struct Env<'a> {
    pub settings: &'a Settings,
    pub terrain: &'a Terrain,
    pub bounds: Bounds,
    pub seed: u64,
    pub tick: u64,
}

impl World {
    pub(crate) fn env(&self) -> Env<'_> {
        Env {
            settings: &self.settings,
            terrain: &self.terrain,
            bounds: self.bounds,
            seed: self.seed,
            tick: self.tick,
        }
    }
}

/// An agent as `board` (sorted by id) has it.
pub(crate) fn record_in(board: &[Record], id: AgentId) -> Option<&Record> {
    board
        .binary_search_by_key(&id, |r| r.id)
        .ok()
        .map(|i| &board[i])
}

/// What every agent has: who it is and where.
#[derive(Debug, Clone)]
pub struct Body {
    pub id: AgentId,
    pub kind: Kind,
    pub pose: Pose,
    pub callsign: String,
    /// The cell it occupies on the grid.
    pub cell: GeoCell,
    /// The refresh square the pose is in; see [`refresh_key`].
    cell_key: (i32, i32),
}

impl Body {
    pub(crate) fn new(id: AgentId, kind: Kind, pose: Pose, cell: GeoCell) -> Self {
        Body {
            id,
            kind,
            pose,
            callsign: String::new(),
            cell,
            cell_key: refresh_key(&pose),
        }
    }

    /// Follow the pose into a new cell, if it has moved far enough to be
    /// worth an A5 lookup.
    pub(crate) fn refresh(&mut self, bounds: &Bounds) {
        let key = refresh_key(&self.pose);
        if key == self.cell_key {
            return;
        }
        self.cell_key = key;
        // Off the edge of the world (a drone following a stale plan): keep
        // the last cell rather than drop the agent from occupancy.
        if let Some(cell) = bounds.cell_of(self.pose.lat_e7, self.pose.lon_e7) {
            self.cell = cell;
        }
    }

    fn saved(&self) -> Saved {
        Saved {
            pose: self.pose,
            callsign: self.callsign.clone(),
            drone: None,
            route: None,
            wander: None,
            courier: None,
        }
    }

    fn record(&self, saved: Saved) -> Record {
        Record {
            id: self.id,
            kind: self.kind,
            cell: self.cell,
            saved,
        }
    }
}

/// A drone flown by a ground station, or by its courier duty.
#[derive(Debug, Clone)]
pub struct DroneAgent {
    pub body: Body,
    pub drone: Drone,
    pub courier: Option<Courier>,
}

/// A person or vehicle wandering near where it started.
#[derive(Debug, Clone)]
pub struct GroundAgent {
    pub body: Body,
    pub wander: Wander,
}

/// An uncontrolled drone flying a closed circuit.
#[derive(Debug, Clone)]
pub struct IntruderAgent {
    pub body: Body,
    pub route: Route,
}

/// A named point of interest. It never moves, but it is on the grid like
/// everything else, so it steps to stay there.
#[derive(Debug, Clone)]
pub struct PoiAgent {
    pub body: Body,
}

/// Places the delivery orders due this tick, before any courier looks.
#[derive(Debug, Clone, Copy)]
pub struct Dispatcher;

impl Agent for Dispatcher {
    fn step(&mut self, state: &mut dyn State) {
        world(state).place_orders();
    }
}

impl Agent for DroneAgent {
    fn step(&mut self, state: &mut dyn State) {
        let world = world(state);
        if world.retiring(self.body.id) {
            return;
        }
        let tick = world.tick;
        let mail = world.take_mail(self.body.id);
        let mut events = Vec::new();
        self.fly(&world.env(), mail, &mut events);
        let mut reports = Vec::new();
        self.settle(world, tick, &events, &mut reports);
        for report in reports {
            world.report(report);
        }

        if self.scans(&world.env()) {
            self.scan(world, tick);
        }

        self.body.refresh(&world.bounds);
        world.place(self.record());
    }

    fn is_stopped(&mut self, state: &mut dyn State) -> bool {
        world(state).retiring(self.body.id)
    }
}

impl DroneAgent {
    pub(crate) fn record(&self) -> Record {
        let mut saved = self.body.saved();
        saved.drone = Some(self.drone.clone());
        saved.courier = self.courier.clone();
        self.body.record(saved)
    }

    /// Apply the mail, then fly a tick. Touches nothing but the drone.
    pub(crate) fn fly(&mut self, env: &Env, mail: Vec<Instruction>, events: &mut Vec<Event>) {
        let tuning = env.settings.tuning;
        // Instructions were checked against this same state when they
        // arrived, so they cannot fail now.
        for instruction in mail {
            let _ = instruction.apply(&mut self.drone, &self.body.pose, &tuning, env.terrain);
        }
        self.drone
            .step(&mut self.body.pose, &tuning, env.terrain, events);
    }

    /// The part of a drone's step that touches the order queues: deliveries
    /// for the waypoints reached this tick, then courier duty. Agents must
    /// settle in id order.
    pub(crate) fn settle(
        &mut self,
        world: &mut World,
        tick: u64,
        events: &[Event],
        reports: &mut Vec<Report>,
    ) {
        for event in events {
            if let Event::Reached(item) = *event {
                self.reached(world, tick, item, reports);
            }
        }
        self.courier_duty(world, tick);
    }

    /// Whether the camera is on this tick.
    pub(crate) fn scans(&self, env: &Env) -> bool {
        env.settings.sensor.scans_on(env.tick) && !self.drone.grounded(&self.body.pose, env.terrain)
    }

    fn reached(&mut self, world: &mut World, tick: u64, item: u16, reports: &mut Vec<Report>) {
        let winched = self
            .drone
            .mission
            .get(usize::from(item))
            .is_some_and(|i| i.command == mission::DO_WINCH);
        if let Some(courier) = self.courier.as_mut().filter(|_| winched) {
            if let Some(order) = courier.order.as_ref().filter(|_| !courier.delivered) {
                courier.delivered = true;
                world.dispatch.delivered += 1;
                let report = delivery_report(
                    tick,
                    self.body.id,
                    self.drone.sysid,
                    &self.body.callsign,
                    order,
                    &world.book,
                    &self.body.pose,
                    world.settings.tuning.dt,
                );
                reports.push(report);
            }
        }
        reports.push(Report {
            seq: 0,
            tick,
            kind: ReportKind::Waypoint,
            uid: format!("wp-{}-{item}", self.body.id),
            observer: Some(self.body.id),
            target: None,
            target_kind: String::new(),
            lat_e7: self.body.pose.lat_e7,
            lon_e7: self.body.pose.lon_e7,
            alt_mm: self.body.pose.alt_mm,
            detail: serde_json::json!({ "seq": item, "sysid": self.drone.sysid }),
        });
    }

    /// A courier that landed gives up or finishes its order; a ready one takes
    /// the oldest order at its depot.
    fn courier_duty(&mut self, world: &mut World, tick: u64) {
        let DroneAgent {
            body,
            drone,
            courier,
        } = self;
        let Some(courier) = courier.as_mut() else {
            return;
        };
        if world.book.depots.is_empty() {
            return;
        }
        let depot = usize::from(courier.depot);
        if courier.order.is_some() && drone.mode == Mode::Disarmed {
            let mut order = courier.order.take().expect("checked");
            if !courier.delivered {
                // Back without delivering: first in line again.
                order.dispatched_tick = None;
                world.dispatch.queues[depot].push_front(order);
            }
            courier.delivered = false;
            courier.landed_tick = Some(tick);
        }

        let ds = world.settings.dispatch;
        let ready = courier.order.is_none()
            && matches!(drone.mode, Mode::Disarmed | Mode::Idle)
            && drone.on_home_pad(&body.pose, &world.terrain)
            && drone.battery_ppm >= world.settings.tuning.launch_ppm
            && courier
                .landed_tick
                .is_none_or(|landed| tick.saturating_sub(landed) >= ds.load_ticks);
        if !ready {
            return;
        }
        let Some(mut order) = world.dispatch.queues[depot].pop_front() else {
            return;
        };
        let to = &world.book.addresses[order.address as usize];
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

    /// Look at everything committed to the cells around, through the grid.
    fn scan(&self, world: &mut World, tick: u64) {
        let pose = &self.body.pose;
        let ground_mm = world.terrain.ground_mm(pose.lat_e7, pose.lon_e7);
        let candidates: Vec<Candidate> = world
            .nearby(self.body.cell)
            .into_iter()
            .filter_map(|id| world.record(id))
            .filter(|r| r.kind != Kind::Drone)
            .map(|r| Candidate {
                id: r.id,
                at: r.saved.pose.point(),
                airborne: r.kind == Kind::Intruder,
            })
            .collect();
        let mut sightings = Vec::new();
        let sensor = world.settings.sensor;
        sensor.scan(
            world.seed,
            tick,
            self.body.id,
            &pose.point(),
            ground_mm,
            candidates.into_iter(),
            &mut world.last_seen,
            &mut sightings,
        );
        for sighting in sightings {
            if let Some(report) = self.sighting_report(&world.board, tick, &sighting) {
                world.report(report);
            }
        }
    }

    pub(crate) fn sighting_report(
        &self,
        board: &[Record],
        tick: u64,
        sighting: &Sighting,
    ) -> Option<Report> {
        let target = record_in(board, sighting.target)?;
        let pose = &target.saved.pose;
        let mut detail = serde_json::json!({
            "sysid": self.drone.sysid,
            "range_m": libm::round(sighting.range_m) as i64,
        });
        if !target.saved.callsign.is_empty() {
            detail["callsign"] = serde_json::Value::String(target.saved.callsign.clone());
        }
        if pose.spd_cms > 0 {
            detail["course_cdeg"] = pose.hdg_cdeg.into();
            detail["speed_cms"] = pose.spd_cms.into();
        }
        Some(Report {
            seq: 0,
            tick,
            kind: ReportKind::Detection,
            uid: format!("det-{}", sighting.target),
            observer: Some(self.body.id),
            target: Some(sighting.target),
            target_kind: target.kind.name().to_owned(),
            lat_e7: pose.lat_e7,
            lon_e7: pose.lon_e7,
            alt_mm: pose.alt_mm,
            detail,
        })
    }
}

impl Agent for GroundAgent {
    fn step(&mut self, state: &mut dyn State) {
        let world = world(state);
        if world.retiring(self.body.id) {
            return;
        }
        self.advance(&world.env());
        world.place(self.record());
    }

    fn is_stopped(&mut self, state: &mut dyn State) -> bool {
        world(state).retiring(self.body.id)
    }
}

impl GroundAgent {
    /// Wander a tick. Touches nothing but the agent.
    pub(crate) fn advance(&mut self, env: &Env) {
        let tuning = env.settings.tuning;
        let draw = draw_pair(env.seed ^ WANDER_SALT, self.body.id, env.tick);
        let bounds = env.bounds;
        self.wander
            .step(&mut self.body.pose, tuning.dt, tuning.dt_ms, draw, |p| {
                bounds.cell_of(p.lat_e7, p.lon_e7).is_some()
            });
        let pose = &mut self.body.pose;
        pose.alt_mm = env.terrain.ground_mm(pose.lat_e7, pose.lon_e7);
        self.body.refresh(&env.bounds);
    }

    pub(crate) fn record(&self) -> Record {
        let mut saved = self.body.saved();
        saved.wander = Some(self.wander.clone());
        self.body.record(saved)
    }
}

impl Agent for IntruderAgent {
    fn step(&mut self, state: &mut dyn State) {
        let world = world(state);
        if world.retiring(self.body.id) {
            return;
        }
        self.advance(&world.env());
        world.place(self.record());
    }

    fn is_stopped(&mut self, state: &mut dyn State) -> bool {
        world(state).retiring(self.body.id)
    }
}

impl IntruderAgent {
    /// Fly the circuit a tick. Touches nothing but the agent.
    pub(crate) fn advance(&mut self, env: &Env) {
        self.route.step(&mut self.body.pose, env.settings.tuning.dt);
        self.body.refresh(&env.bounds);
    }

    pub(crate) fn record(&self) -> Record {
        let mut saved = self.body.saved();
        saved.route = Some(self.route.clone());
        self.body.record(saved)
    }
}

impl Agent for PoiAgent {
    fn step(&mut self, state: &mut dyn State) {
        let world = world(state);
        if world.retiring(self.body.id) {
            return;
        }
        world.place(self.record());
    }

    fn is_stopped(&mut self, state: &mut dyn State) -> bool {
        world(state).retiring(self.body.id)
    }
}

impl PoiAgent {
    pub(crate) fn record(&self) -> Record {
        self.body.record(self.body.saved())
    }
}

/// Any agent but the dispatcher, before it joins a schedule.
#[derive(Debug, Clone)]
pub(crate) enum Member {
    Drone(DroneAgent),
    Ground(GroundAgent),
    Intruder(IntruderAgent),
    Poi(PoiAgent),
}

impl Member {
    pub(crate) fn body(&self) -> &Body {
        match self {
            Member::Drone(a) => &a.body,
            Member::Ground(a) => &a.body,
            Member::Intruder(a) => &a.body,
            Member::Poi(a) => &a.body,
        }
    }

    pub(crate) fn record(&self) -> Record {
        match self {
            Member::Drone(a) => a.record(),
            Member::Ground(a) => a.record(),
            Member::Intruder(a) => a.record(),
            Member::Poi(a) => a.record(),
        }
    }

    pub(crate) fn boxed(self) -> Box<dyn Agent> {
        match self {
            Member::Drone(a) => Box::new(a),
            Member::Ground(a) => Box::new(a),
            Member::Intruder(a) => Box::new(a),
            Member::Poi(a) => Box::new(a),
        }
    }

    /// The agent a published record describes.
    pub(crate) fn from_record(record: Record) -> Result<Member, String> {
        let Record {
            id,
            kind,
            cell,
            saved,
        } = record;
        let mut body = Body::new(id, kind, saved.pose, cell);
        body.callsign = saved.callsign;
        let missing = |what: &str| format!("a {} without {what} state", kind.name());
        Ok(match kind {
            Kind::Drone => Member::Drone(DroneAgent {
                body,
                drone: saved.drone.ok_or_else(|| missing("drone"))?,
                courier: saved.courier,
            }),
            Kind::Intruder => Member::Intruder(IntruderAgent {
                body,
                route: saved.route.ok_or_else(|| missing("route"))?,
            }),
            Kind::Person | Kind::Vehicle => Member::Ground(GroundAgent {
                body,
                wander: saved.wander.ok_or_else(|| missing("wander"))?,
            }),
            Kind::Poi => Member::Poi(PoiAgent { body }),
        })
    }
}

#[allow(clippy::too_many_arguments)]
fn delivery_report(
    tick: u64,
    observer: AgentId,
    sysid: u8,
    callsign: &str,
    order: &Order,
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
