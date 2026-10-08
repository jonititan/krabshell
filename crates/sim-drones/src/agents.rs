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

use crate::behaviour::Node;
use crate::dispatch::{self, AddressBook, Courier, Order};
use crate::flight::Pose;
use crate::mission::{self, Drone, Event, Mode};
use crate::sensor::{Candidate, Sighting};
use crate::targets::{Route, Wander};
use crate::world::{draw_pair, refresh_key, Kind, Record, Saved, World, WANDER_SALT};

fn world(state: &mut dyn State) -> &mut World {
    state
        .as_any_mut()
        .downcast_mut::<World>()
        .expect("drones agents step in a drones World")
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
    fn refresh(&mut self, world: &World) {
        let key = refresh_key(&self.pose);
        if key == self.cell_key {
            return;
        }
        self.cell_key = key;
        // Off the edge of the world (a drone following a stale plan): keep
        // the last cell rather than drop the agent from occupancy.
        if let Some(cell) = world.cell_of(self.pose.lat_e7, self.pose.lon_e7) {
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
    /// Crashed: off the board, and off the schedule after this step.
    pub wrecked: bool,
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

/// First on every step: opens and closes the geofences due and places the
/// delivery orders due, before any drone moves.
#[derive(Debug, Clone, Copy)]
pub struct Dispatcher;

impl Agent for Dispatcher {
    fn step(&mut self, state: &mut dyn State) {
        let world = world(state);
        world.update_airspace();
        world.place_orders();
    }
}

impl Agent for DroneAgent {
    fn step(&mut self, state: &mut dyn State) {
        let world = world(state);
        if world.retiring(self.body.id) {
            return;
        }
        let tick = world.tick;

        // A spare called up to replace a courier that crashed.
        if let Some(courier) = world.call_ups.remove(&self.body.id) {
            self.courier = Some(courier);
        }

        // What it does this tick is up to its brain.
        let mut brain = world.brain.take().expect("a world always has a brain");
        brain.tick(world.settings.tuning.dt, |node| {
            self.leaf(node, world, tick)
        });
        world.brain = Some(brain);
        if self.wrecked {
            return;
        }

        self.body.refresh(world);
        world.place(self.record());
    }

    fn is_stopped(&mut self, state: &mut dyn State) -> bool {
        self.wrecked || world(state).retiring(self.body.id)
    }
}

impl DroneAgent {
    pub(crate) fn record(&self) -> Record {
        let mut saved = self.body.saved();
        saved.drone = Some(self.drone.clone());
        saved.courier = self.courier.clone();
        self.body.record(saved)
    }

    fn reached(&mut self, world: &mut World, tick: u64, item: u16) {
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
                world.report(report);
            }
        }
        world.report(Report {
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

    /// Report each geofence it is in the air inside, once on the way in.
    fn check_fences(&mut self, world: &mut World, tick: u64) {
        if world.airspace.is_empty() && self.drone.fenced.is_empty() {
            return;
        }
        let inside = if self.drone.grounded(&self.body.pose, &world.terrain) {
            Vec::new()
        } else {
            let pose = &self.body.pose;
            let ground = world.terrain.ground_mm(pose.lat_e7, pose.lon_e7);
            world
                .airspace
                .inside(&pose.point(), f64::from(pose.alt_mm - ground) / 1000.0)
        };
        for &id in inside.iter().filter(|id| !self.drone.fenced.contains(id)) {
            let opened_round = world
                .airspace
                .fences()
                .iter()
                .any(|f| f.id == id && f.opened_tick == tick);
            let mut detail = self.flight_detail(world);
            detail["fence"] = id.into();
            detail["how"] = if opened_round {
                "fence opened round it"
            } else {
                "flew in"
            }
            .into();
            detail["flown_by"] = if world.ops.contains_key(&self.body.id) {
                "operator"
            } else {
                "ground station"
            }
            .into();
            detail["mode"] = self.drone.mode.name().into();
            world.report(self.flight_report(
                ReportKind::Breach,
                format!("breach-{}-{id}-{tick}", self.body.id),
                tick,
                detail,
            ));
        }
        self.drone.fenced = inside;
    }

    /// The end of a flight, with what is left in the battery.
    fn landed(&self, world: &mut World, tick: u64) {
        let mut detail = self.flight_detail(world);
        detail["battery_pct"] = battery_pct(self.drone.battery_ppm).into();
        world.report(self.flight_report(
            ReportKind::Landing,
            format!("land-{}-{tick}", self.body.id),
            tick,
            detail,
        ));
    }

    /// Out of battery in the air. The wreck leaves the board; its order goes
    /// back in the queue and a spare or a new drone takes its place.
    fn crashed(&mut self, world: &mut World, tick: u64) {
        self.wrecked = true;
        let mut detail = self.flight_detail(world);
        detail["cause"] = "battery exhausted".into();
        if let Some(order) = self
            .courier
            .as_ref()
            .and_then(|c| c.order.as_ref().filter(|_| !c.delivered))
        {
            detail["requeued_order"] = order.id.into();
        }
        detail["replaced_by"] = match world.replace(self) {
            Some(r) => serde_json::json!({
                "id": r.id, "sysid": r.sysid, "callsign": r.callsign, "spare": r.spare,
            }),
            None => serde_json::Value::Null,
        };
        world.report(self.flight_report(
            ReportKind::Crash,
            format!("crash-{}", self.body.id),
            tick,
            detail,
        ));
    }

    fn flight_detail(&self, world: &World) -> serde_json::Value {
        let mut detail = serde_json::json!({
            "sysid": self.drone.sysid,
            "callsign": self.body.callsign,
        });
        if let Some(courier) = &self.courier {
            detail["depot"] = world.book.depots[usize::from(courier.depot)].clone().into();
        }
        detail
    }

    fn flight_report(
        &self,
        kind: ReportKind,
        uid: String,
        tick: u64,
        detail: serde_json::Value,
    ) -> Report {
        Report {
            seq: 0,
            tick,
            kind,
            uid,
            observer: Some(self.body.id),
            target: None,
            target_kind: String::new(),
            lat_e7: self.body.pose.lat_e7,
            lon_e7: self.body.pose.lon_e7,
            alt_mm: self.body.pose.alt_mm,
            detail,
        }
    }

    /// Answer a sense or carry out an act of the behaviour tree: true for
    /// success, false for failure, `None` for still running.
    fn leaf(&mut self, node: Node, world: &mut World, tick: u64) -> Option<bool> {
        let grounded = || self.drone.grounded(&self.body.pose, &world.terrain);
        Some(match node {
            Node::Airborne => !grounded(),
            Node::IsCourier => self.courier.is_some() && !world.book.depots.is_empty(),
            Node::FlownBySimulation => world.ops.contains_key(&self.body.id),
            Node::BackFromTrip => {
                self.courier.as_ref().is_some_and(|c| c.order.is_some())
                    && self.drone.mode == Mode::Disarmed
            }
            Node::ReadyForWork => self.ready_for_work(world, tick),
            Node::SensorDue => world.settings.sensor.scans_on(tick),
            Node::AtStation => false,
            Node::LowBattery => self.drone.failsafe,
            Node::AvoidGeofences => {
                world.operate_drone(
                    self.body.id,
                    &self.body.callsign,
                    &self.drone,
                    &self.body.pose,
                    self.courier.as_ref(),
                );
                true
            }
            Node::ObeyCommands => {
                let tuning = world.settings.tuning;
                // Instructions were checked against this same state when
                // they arrived, so they cannot fail now.
                for instruction in world.take_mail(self.body.id) {
                    let _ = instruction.apply(
                        &mut self.drone,
                        &self.body.pose,
                        &tuning,
                        &world.terrain,
                    );
                }
                true
            }
            Node::Fly => self.fly(world, tick),
            Node::FinishTrip => {
                self.finish_trip(world, tick);
                true
            }
            Node::TakeOrder => self.take_order(world, tick),
            Node::Survey | Node::DivertToStation | Node::ReturnToBase => false,
            Node::LogBreaches => {
                self.check_fences(world, tick);
                true
            }
            Node::Scan => {
                self.scan(world, tick);
                true
            }
        })
    }

    /// One tick of the autopilot, and what came of it. False if it crashed.
    fn fly(&mut self, world: &mut World, tick: u64) -> bool {
        let tuning = world.settings.tuning;
        let mut events = Vec::new();
        self.drone
            .step(&mut self.body.pose, &tuning, &world.terrain, &mut events);
        for event in events {
            match event {
                Event::Reached(item) => self.reached(world, tick, item),
                Event::Landed => self.landed(world, tick),
                Event::Crashed => {
                    self.crashed(world, tick);
                    return false;
                }
                Event::LowBattery => {}
            }
        }
        true
    }

    /// Back from a trip without delivering: the order goes back to the front
    /// of the queue, unless a geofence opened over the address and such
    /// orders are turned away. Loading starts now either way.
    fn finish_trip(&mut self, world: &mut World, tick: u64) {
        let Some(courier) = self.courier.as_mut() else {
            return;
        };
        let Some(mut order) = courier.order.take() else {
            return;
        };
        if !courier.delivered {
            order.dispatched_tick = None;
            if !world.reject_if_fenced(&order, crate::world::Stage::Returned) {
                world.dispatch.queues[usize::from(courier.depot)].push_front(order);
            }
        }
        courier.delivered = false;
        courier.landed_tick = Some(tick);
    }

    /// On its home pad with nothing to do and charged to launch; a courier
    /// also loaded, and a drone that is not one flown by the simulation.
    fn ready_for_work(&self, world: &World, tick: u64) -> bool {
        let on_pad = matches!(self.drone.mode, Mode::Disarmed | Mode::Idle)
            && self.drone.on_home_pad(&self.body.pose, &world.terrain)
            && self.drone.battery_ppm >= world.settings.tuning.launch_ppm;
        match &self.courier {
            Some(courier) => {
                on_pad
                    && !world.book.depots.is_empty()
                    && courier.order.is_none()
                    && courier.landed_tick.is_none_or(|landed| {
                        tick.saturating_sub(landed) >= world.settings.dispatch.load_ticks
                    })
            }
            None => on_pad && world.ops.contains_key(&self.body.id),
        }
    }

    /// Take the oldest order at its depot whose address is not inside a
    /// geofence, and set off with it. False if there is none.
    fn take_order(&mut self, world: &mut World, tick: u64) -> bool {
        let DroneAgent { drone, courier, .. } = self;
        let Some(courier) = courier.as_mut() else {
            return false;
        };
        if world.book.depots.is_empty() {
            return false;
        }
        let depot = usize::from(courier.depot);
        let next = world.dispatch.queues[depot].iter().position(|o| {
            world.airspace.is_empty() || {
                let a = &world.book.addresses[o.address as usize];
                !world.airspace.covers(&crate::flight::Point {
                    lat_e7: a.lat_e7,
                    lon_e7: a.lon_e7,
                    alt_mm: 0,
                })
            }
        });
        let Some(mut order) = next.and_then(|i| world.dispatch.queues[depot].remove(i)) else {
            return false;
        };
        let ds = world.settings.dispatch;
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
        true
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
            if let Some(report) = self.sighting_report(world, tick, &sighting) {
                world.report(report);
            }
        }
    }

    fn sighting_report(&self, world: &World, tick: u64, sighting: &Sighting) -> Option<Report> {
        let target = world.record(sighting.target)?;
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
        let tuning = world.settings.tuning;
        let draw = draw_pair(world.seed ^ WANDER_SALT, self.body.id, world.tick);
        self.wander
            .step(&mut self.body.pose, tuning.dt, tuning.dt_ms, draw, |p| {
                world.accepts(p)
            });
        let pose = &mut self.body.pose;
        pose.alt_mm = world.terrain.ground_mm(pose.lat_e7, pose.lon_e7);
        self.body.refresh(world);
        world.place(self.record());
    }

    fn is_stopped(&mut self, state: &mut dyn State) -> bool {
        world(state).retiring(self.body.id)
    }
}

impl GroundAgent {
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
        self.route
            .step(&mut self.body.pose, world.settings.tuning.dt);
        self.body.refresh(world);
        world.place(self.record());
    }

    fn is_stopped(&mut self, state: &mut dyn State) -> bool {
        world(state).retiring(self.body.id)
    }
}

impl IntruderAgent {
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
                wrecked: false,
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

/// Battery charge as a percentage, to two decimal places.
fn battery_pct(ppm: u32) -> f64 {
    f64::from(ppm / 100) / 100.0
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
