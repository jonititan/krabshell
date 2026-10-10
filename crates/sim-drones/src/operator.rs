//! The operator: whoever flies the drones the simulation flies itself
//! (couriers, and drones a pack starts on a mission), keeping them clear of
//! geofences the way a ground station would, with MAVLink.
//!
//! The autopilot knows nothing of fences. When the fences change, or a drone
//! it flies gets a new mission or turns for home, the operator replans that
//! drone's route round them and sends what a ground station would send:
//!
//! - the mission with detour waypoints inserted wherever a leg crosses a
//!   fence (`MISSION_COUNT`/`MISSION_ITEM_INT`), `DO_JUMP` targets
//!   renumbered to match, and `MISSION_SET_CURRENT` to carry on along it;
//! - when the drone cannot see its way onto the new route from where it is,
//!   `DO_REPOSITION` to each corner of the way there in turn, then
//!   `MISSION_START`;
//! - for a drone returning home, a mission of detour waypoints at the return
//!   height ending in `NAV_LAND` on its pad, and `MISSION_START`;
//! - for a drone a fence opened round, `DO_REPOSITION` to just outside it,
//!   then the rerouted mission (or `NAV_RETURN_TO_LAUNCH`) once it is out;
//! - for a courier whose delivery address is now fenced,
//!   `NAV_RETURN_TO_LAUNCH`; the order goes back in the queue when it lands.
//!
//! Each leg's detour is planned from the item before it, as the mission
//! flies it, so a mission that loops is safe on every lap. The operator
//! remembers which items it inserted (and any it moved to the edge of a fence
//! their destination was inside), so the next replan starts from the mission
//! as it was, and a closed fence takes its detours with it.
//!
//! Drones flown by ground stations are left to them.

use serde::{Deserialize, Serialize};

use crate::airspace::Airspace;
use crate::flight::{Point, Pose};
use crate::mission::{
    self, Drone, MissionItem, Mode, Tuning, DO_JUMP, NAV_LAND, NAV_LOITER_TIME, NAV_LOITER_UNLIM,
    NAV_RETURN_TO_LAUNCH, NAV_TAKEOFF, NAV_WAYPOINT,
};
use crate::terrain::Terrain;
use crate::world::Instruction;

const FRAME_GLOBAL: u8 = 0;
const FRAME_RELATIVE: u8 = 3;
const FRAME_GLOBAL_INT: u8 = 5;
const FRAME_TERRAIN: u8 = 10;
const FRAME_TERRAIN_INT: u8 = 11;

/// Acceptance radius of a detour waypoint, metres: a corner to pass, not a
/// point to hit.
const DETOUR_ACCEPT_M: f32 = 5.0;

/// What a drone the operator moved with `DO_REPOSITION` goes back to once
/// it gets there.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Resume {
    Mission,
    Home,
}

/// The operator's notes on one drone it flies. Checkpointed.
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct Ops {
    /// The `mission_rev` of the last mission the operator uploaded. The
    /// notes on inserted and moved items describe that mission, and only
    /// while it is still the drone's.
    #[serde(default)]
    pub rev: u32,
    /// Seqs of the detour waypoints it inserted.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub inserted: Vec<u16>,
    /// Items it moved to the edge of a fence their destination was inside,
    /// with where they were going, `[lat_e7, lon_e7]`.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub moved: Vec<(u16, [i32; 2])>,
    /// Items it brought down to fly beneath a fence over where they go,
    /// with the frame and height they had.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub lowered: Vec<(u16, u8, f32)>,
    /// The mission is a way home it made in place of RTL.
    #[serde(default)]
    pub homing: bool,
    /// Moved off its mission by `DO_REPOSITION`: what to go back to.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub resume: Option<Resume>,
    /// Corners still to reposition to before resuming, `[lat_e7, lon_e7]`.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub hops: Vec<[i32; 2]>,
    /// Airspace revision, mission revision and mode it last planned for.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub checked: Option<(u64, u32, String)>,
}

impl Ops {
    /// Nothing of the operator's is in play on the drone.
    pub fn is_clean(&self) -> bool {
        self.inserted.is_empty()
            && self.moved.is_empty()
            && self.lowered.is_empty()
            && !self.homing
            && self.resume.is_none()
            && self.hops.is_empty()
    }

    fn mine(&self, drone: &Drone) -> bool {
        self.rev == drone.mission_rev
            && (!self.inserted.is_empty() || !self.moved.is_empty() || !self.lowered.is_empty())
    }
}

/// What the operator sends one drone, as MAVLink would carry it.
#[derive(Debug, Clone)]
pub(crate) struct Orders {
    pub instructions: Vec<Instruction>,
    /// The messages, for the log: `MISSION_ITEM_INT ×9`, `DO_REPOSITION`…
    pub messages: Vec<String>,
    pub reason: &'static str,
    pub detours: usize,
}

/// A rerouted mission.
#[derive(Debug, Clone)]
pub struct Plan {
    pub items: Vec<MissionItem>,
    /// Where to carry on from.
    pub seq: u16,
    pub notes: Ops,
    /// Corners to reposition through first, when the drone cannot see its
    /// way to `seq` from where it is.
    pub hops: Vec<[i32; 2]>,
}

fn waypoint(at: [i32; 2], frame: u8, z: f32) -> MissionItem {
    MissionItem {
        seq: 0,
        frame,
        command: NAV_WAYPOINT,
        autocontinue: 1,
        param1: 0.0,
        param2: DETOUR_ACCEPT_M,
        param3: 0.0,
        param4: f32::NAN,
        x: at[0],
        y: at[1],
        z,
    }
}

fn point(at: [i32; 2]) -> Point {
    Point {
        lat_e7: at[0],
        lon_e7: at[1],
        alt_mm: 0,
    }
}

/// The same missions, NaN parameters ("leave as is") matching.
pub fn same(a: &[MissionItem], b: &[MissionItem]) -> bool {
    let f = |x: f32, y: f32| x == y || (x.is_nan() && y.is_nan());
    a.len() == b.len()
        && a.iter().zip(b).all(|(a, b)| {
            (a.seq, a.frame, a.command, a.autocontinue, a.x, a.y)
                == (b.seq, b.frame, b.command, b.autocontinue, b.x, b.y)
                && f(a.param1, b.param1)
                && f(a.param2, b.param2)
                && f(a.param3, b.param3)
                && f(a.param4, b.param4)
                && f(a.z, b.z)
        })
}

/// Where an item takes the drone, horizontally.
fn position(item: &MissionItem, home: &Point) -> Option<[i32; 2]> {
    if item.command == NAV_RETURN_TO_LAUNCH {
        return Some([home.lat_e7, home.lon_e7]);
    }
    item.destination().map(|p| [p.lat_e7, p.lon_e7])
}

/// The height an item flies at, as its frame and `z`, for detours on the
/// way to it.
fn height(item: &MissionItem) -> Option<(u8, f32)> {
    matches!(
        item.command,
        NAV_WAYPOINT | NAV_LOITER_TIME | NAV_LOITER_UNLIM | NAV_TAKEOFF
    )
    .then_some((item.frame, item.z))
    .filter(|(_, z)| z.is_finite())
}

/// The drone's mission without the operator's detours, items it moved put
/// back, `DO_JUMP`s renumbered to match; and the index the drone is at in it.
fn base(drone: &Drone, ops: &Ops) -> (Vec<MissionItem>, usize) {
    let mine = ops.mine(drone);
    let inserted = |i: usize| mine && ops.inserted.contains(&(i as u16));
    // Index in the base mission of item `i` of the drone's (or of the next
    // base item, for a detour).
    let to_base = |i: usize| (0..i).filter(|j| !inserted(*j)).count();
    let mut items = Vec::new();
    for (i, item) in drone.mission.iter().enumerate() {
        if inserted(i) {
            continue;
        }
        let mut item = *item;
        if mine {
            if let Some((_, to)) = ops.moved.iter().find(|(seq, _)| usize::from(*seq) == i) {
                (item.x, item.y) = (to[0], to[1]);
            }
            if let Some((_, frame, z)) = ops.lowered.iter().find(|(seq, ..)| usize::from(*seq) == i)
            {
                (item.frame, item.z) = (*frame, *z);
            }
            if item.command == DO_JUMP {
                item.param1 = to_base(item.param1.max(0.0) as usize) as f32;
            }
        }
        items.push(item);
    }
    let at = to_base(usize::from(drone.mission_seq).min(drone.mission.len()));
    (items, at)
}

/// Metres above the ground an item's height puts a drone at `at`: terrain
/// frames say so; absolute ones are above mean sea level, the rest above
/// home. Where a leg's ground rises or falls, it is judged at its end.
fn agl(h: (u8, f32), at: [i32; 2], home: &Point, terrain: &Terrain) -> f64 {
    let z = f64::from(h.1);
    let ground = f64::from(terrain.ground_mm(at[0], at[1])) / 1000.0;
    let metres = match h.0 {
        FRAME_TERRAIN | FRAME_TERRAIN_INT => z,
        FRAME_GLOBAL | FRAME_GLOBAL_INT => z - ground,
        _ => f64::from(home.alt_mm) / 1000.0 + z - ground,
    };
    metres.max(0.0)
}

/// The drone's mission rerouted round the fences, and beneath those it can
/// pass under when that is shorter.
pub fn reroute(
    drone: &Drone,
    pose: &Pose,
    airborne: bool,
    ops: &Ops,
    airspace: &Airspace,
    tuning: &Tuning,
    terrain: &Terrain,
) -> Plan {
    let (base, at) = base(drone, ops);
    let home = drone.home;
    let rtl_height = (FRAME_RELATIVE, (tuning.rtl_alt_mm as f32) / 1000.0);
    let mut out: Vec<MissionItem> = Vec::with_capacity(base.len());
    let mut index = vec![0usize; base.len()];
    let mut notes = Ops::default();
    // The items that lead to the base item the drone is on: its detours,
    // then itself.
    let mut approach = None;
    let mut prev = [home.lat_e7, home.lon_e7];
    let mut prev_height = rtl_height;

    // The way to `to` at height `h`, its corners inserted as waypoints. A
    // way beneath a fence drops to the low height where the leg starts and
    // arrives low; the end, and the low height if it went beneath, are
    // returned.
    let detour = |out: &mut Vec<MissionItem>,
                  notes: &mut Ops,
                  from: [i32; 2],
                  to: [i32; 2],
                  h: (u8, f32)| {
        let passage = airspace.passage(&point(from), &point(to), agl(h, to, &home, terrain));
        let route = passage.path;
        let end = *route.last().expect("a route ends somewhere");
        let (low, corner) = if passage.beneath {
            let low = (FRAME_TERRAIN_INT, passage.agl_m as f32);
            notes.inserted.push(out.len() as u16);
            out.push(waypoint(from, low.0, low.1));
            (Some(low), low)
        } else {
            (None, h)
        };
        for p in &route[..route.len() - 1] {
            notes.inserted.push(out.len() as u16);
            out.push(waypoint(*p, corner.0, corner.1));
        }
        if let Some(low) = low {
            notes.inserted.push(out.len() as u16);
            out.push(waypoint(end, low.0, low.1));
        }
        (end, low)
    };

    for (b, item) in base.iter().enumerate() {
        let mut item = *item;
        let first = out.len();
        if let Some(dest) = position(&item, &home) {
            let h = if item.command == NAV_RETURN_TO_LAUNCH {
                rtl_height
            } else {
                height(&item).unwrap_or(prev_height)
            };
            let (end, low) = detour(&mut out, &mut notes, prev, dest, h);
            if end != dest && item.command != NAV_RETURN_TO_LAUNCH {
                // Inside a fence: go as far as its edge.
                notes.moved.push((out.len() as u16, dest));
                (item.x, item.y) = (end[0], end[1]);
            }
            // Arrived beneath a fence over where it goes: stay down there
            // rather than climb into it.
            if let (Some(low), Some(own)) = (low, height(&item)) {
                if airspace.blocks(&point(end), agl(own, end, &home, terrain)) {
                    notes.lowered.push((out.len() as u16, item.frame, item.z));
                    (item.frame, item.z) = low;
                }
            }
            prev = end;
        } else if item.command == DO_JUMP {
            // The leg the jump makes, from here to the next place on from its
            // target.
            let target = item.param1.max(0.0) as usize;
            if let Some((to, h)) = base.get(target..).and_then(|rest| {
                rest.iter()
                    .find_map(|i| position(i, &home).map(|p| (p, height(i).unwrap_or(prev_height))))
            }) {
                detour(&mut out, &mut notes, prev, to, h);
            }
        }
        if let Some(h) = height(&item) {
            prev_height = h;
        }
        if b == at {
            approach = Some(first..out.len() + 1);
        }
        index[b] = out.len();
        out.push(item);
    }
    for item in &mut out {
        if item.command == DO_JUMP {
            if let Some(i) = index.get(item.param1.max(0.0) as usize) {
                item.param1 = *i as f32;
            }
        }
    }
    for (i, item) in out.iter_mut().enumerate() {
        item.seq = i as u16;
    }

    // Join the route at the furthest point along it the drone can fly
    // straight to; failing that, reposition there corner by corner.
    let mut hops = Vec::new();
    let seq = match approach {
        None => out.len(),
        Some(range) => {
            let place = |i: usize| position(&out[i], &home).map(point);
            let here = pose.point();
            let here_agl = f64::from(pose.alt_mm - terrain.ground_mm(pose.lat_e7, pose.lon_e7))
                .max(0.0)
                / 1000.0;
            // Flown at the height of the item it heads for.
            let leg_agl = |i: usize, p: &Point| {
                height(&out[i]).map_or(here_agl, |h| agl(h, [p.lat_e7, p.lon_e7], &home, terrain))
            };
            let seen = range.clone().rev().find(|i| {
                place(*i).is_none_or(|p| !airborne || airspace.is_clear(&here, &p, leg_agl(*i, &p)))
            });
            match seen {
                Some(i) => i,
                None => {
                    let first = range.start;
                    if let Some(p) = place(first) {
                        hops = airspace.route(&here, &p, here_agl);
                    }
                    first
                }
            }
        }
    };
    Plan {
        items: out,
        seq: seq as u16,
        notes,
        hops,
    }
}

/// A way home round the fences at the return height, ending in a landing on
/// the pad. `None` when the straight way is clear.
pub fn way_home(
    drone: &Drone,
    pose: &Pose,
    airspace: &Airspace,
    tuning: &Tuning,
    terrain: &Terrain,
) -> Option<Vec<MissionItem>> {
    let home = [drone.home.lat_e7, drone.home.lon_e7];
    let rtl = (FRAME_RELATIVE, (tuning.rtl_alt_mm as f32) / 1000.0);
    let route = airspace.route(
        &pose.point(),
        &point(home),
        agl(rtl, home, &drone.home, terrain),
    );
    if route.len() <= 1 {
        return None;
    }
    Some(home_mission(drone, &route, tuning))
}

/// Waypoints at the return height through `route` (ending at home), then a
/// landing on the pad.
fn home_mission(drone: &Drone, route: &[[i32; 2]], tuning: &Tuning) -> Vec<MissionItem> {
    let home = [drone.home.lat_e7, drone.home.lon_e7];
    let z = (tuning.rtl_alt_mm as f32) / 1000.0;
    let mut items: Vec<MissionItem> = route
        .iter()
        .map(|p| waypoint(*p, FRAME_RELATIVE, z))
        .collect();
    let mut land = waypoint(home, FRAME_RELATIVE, 0.0);
    land.command = NAV_LAND;
    land.param2 = 0.0;
    items.push(land);
    for (i, item) in items.iter_mut().enumerate() {
        item.seq = i as u16;
    }
    items
}

/// `DO_REPOSITION` to a point, holding the height it is at.
fn reposition(drone: &Drone, pose: &Pose, to: [i32; 2]) -> Instruction {
    let to = point(to);
    let mut params = [f32::NAN; 7];
    params[0] = -1.0;
    params[4] = to.lat() as f32;
    params[5] = to.lon() as f32;
    params[6] = ((pose.alt_mm - drone.home.alt_mm) as f32) / 1000.0;
    Instruction::Command {
        command: mission::DO_REPOSITION,
        params,
    }
}

fn command(command: u16) -> Instruction {
    Instruction::Command {
        command,
        params: [0.0; 7],
    }
}

fn single(instruction: Instruction, message: &str, reason: &'static str) -> Orders {
    Orders {
        instructions: vec![instruction],
        messages: vec![message.to_owned()],
        reason,
        detours: 0,
    }
}

/// Decide what, if anything, to send a drone the operator flies.
/// `address` is where a courier's undelivered order is going.
#[allow(clippy::too_many_arguments)]
pub(crate) fn orders(
    drone: &Drone,
    pose: &Pose,
    airborne: bool,
    address: Option<Point>,
    ops: &mut Ops,
    airspace: &Airspace,
    tuning: &Tuning,
    terrain: &Terrain,
    max_items: usize,
) -> Option<Orders> {
    if !airborne && drone.mode == Mode::Disarmed {
        // Down and done: nothing of the operator's is left in play.
        if !ops.is_clean() || ops.checked.is_some() {
            *ops = Ops::default();
        }
        return None;
    }
    if airspace.is_empty() && ops.is_clean() {
        ops.checked = None;
        return None;
    }

    // A fence opened round it: out the nearest side first.
    if airborne && !matches!(drone.mode, Mode::Goto { .. }) {
        let agl = f64::from(pose.alt_mm - terrain.ground_mm(pose.lat_e7, pose.lon_e7)) / 1000.0;
        if let Some(out) = airspace.way_out(&pose.point(), agl.max(0.0)) {
            ops.resume = match drone.mode {
                Mode::Rtl => Some(Resume::Home),
                Mode::Auto if ops.homing => Some(Resume::Home),
                Mode::Auto => Some(Resume::Mission),
                _ => ops.resume,
            };
            ops.hops.clear();
            ops.checked = None;
            return Some(single(
                reposition(drone, pose, [out.lat_e7, out.lon_e7]),
                "COMMAND_INT DO_REPOSITION",
                "inside a geofence",
            ));
        }
    }

    // A courier whose address is now fenced off turns for home. Not one on
    // its low-battery failsafe: that one is already making for home or, if
    // home is out of reach, for a station, and sending it home again would
    // undo the divert every tick until it ran flat.
    if let Some(to) = address {
        if drone.mode == Mode::Auto && !ops.homing && !drone.failsafe && airspace.covers(&to) {
            ops.checked = None;
            return Some(single(
                command(mission::NAV_RETURN_TO_LAUNCH),
                "COMMAND_LONG NAV_RETURN_TO_LAUNCH",
                "delivery address geofenced",
            ));
        }
    }

    // Repositioned where it was sent: the next corner, or back to work.
    if let Some(resume) = ops.resume {
        if drone.mode != Mode::Loiter {
            // Still on its way.
            return None;
        }
        ops.checked = None;
        if !ops.hops.is_empty() {
            let next = ops.hops.remove(0);
            return Some(single(
                reposition(drone, pose, next),
                "COMMAND_INT DO_REPOSITION",
                "on the way round a geofence",
            ));
        }
        ops.resume = None;
        if resume == Resume::Home {
            return Some(single(
                command(mission::NAV_RETURN_TO_LAUNCH),
                "COMMAND_LONG NAV_RETURN_TO_LAUNCH",
                "round the geofence, home",
            ));
        }
        let plan = reroute(drone, pose, airborne, ops, airspace, tuning, terrain);
        if plan.items.len() > max_items {
            return None;
        }
        return Some(send(
            drone,
            pose,
            airspace.rev(),
            ops,
            plan,
            "round the geofence, mission",
        ));
    }

    let key = (
        airspace.rev(),
        drone.mission_rev,
        drone.mode.name().to_owned(),
    );
    if ops.checked.as_ref() == Some(&key) {
        return None;
    }
    match drone.mode {
        Mode::Rtl => {
            ops.checked = Some(key);
            let items = way_home(drone, pose, airspace, tuning, terrain)?;
            Some(send_home(drone, pose, airspace.rev(), ops, items))
        }
        Mode::Auto if ops.homing && drone.mission_rev == ops.rev => {
            ops.checked = Some(key);
            let home = [drone.home.lat_e7, drone.home.lon_e7];
            let items = way_home(drone, pose, airspace, tuning, terrain)
                .unwrap_or_else(|| home_mission(drone, &[home], tuning));
            if same(&items, &drone.mission) {
                return None;
            }
            Some(send_home(drone, pose, airspace.rev(), ops, items))
        }
        Mode::Auto => {
            // Mid-hold (a winch, a delay, a landing): wait until it is done,
            // so a new upload does not start the hold again.
            if drone.arrived {
                return None;
            }
            ops.checked = Some(key);
            let plan = reroute(drone, pose, airborne, ops, airspace, tuning, terrain);
            let unchanged = same(&plan.items, &drone.mission) && plan.seq == drone.mission_seq;
            if (unchanged && plan.hops.is_empty()) || plan.items.len() > max_items {
                return None;
            }
            Some(send(
                drone,
                pose,
                airspace.rev(),
                ops,
                plan,
                "route round a geofence",
            ))
        }
        _ => {
            ops.checked = Some(key);
            None
        }
    }
}

fn send_home(
    drone: &Drone,
    pose: &Pose,
    airspace_rev: u64,
    ops: &mut Ops,
    items: Vec<MissionItem>,
) -> Orders {
    let notes = Ops {
        homing: true,
        inserted: (0..items.len() as u16 - 1).collect(),
        ..Ops::default()
    };
    let plan = Plan {
        items,
        seq: 0,
        notes,
        hops: Vec::new(),
    };
    send(
        drone,
        pose,
        airspace_rev,
        ops,
        plan,
        "way home round a geofence",
    )
}

/// Upload a plan and set the drone going on it: `MISSION_SET_CURRENT` to
/// where it joins, then `MISSION_START`, or, if it has to get round a fence
/// to join it, `DO_REPOSITION` to the first corner of the way there.
fn send(
    drone: &Drone,
    pose: &Pose,
    airspace_rev: u64,
    ops: &mut Ops,
    plan: Plan,
    reason: &'static str,
) -> Orders {
    let Plan {
        items,
        seq,
        mut notes,
        mut hops,
    } = plan;
    // The upload bumps the mission revision; what it was planned for is that.
    notes.rev = drone.mission_rev.wrapping_add(1);
    notes.checked = Some((airspace_rev, notes.rev, Mode::Auto.name().to_owned()));
    let detours = notes.inserted.len();
    let mut messages = vec![format!("MISSION_COUNT/MISSION_ITEM_INT ×{}", items.len())];
    let mut instructions = vec![Instruction::Upload(items)];
    if seq > 0 {
        instructions.push(Instruction::SetCurrent(seq));
        messages.push(format!("MISSION_SET_CURRENT {seq}"));
    }
    if hops.is_empty() {
        if drone.mode != Mode::Auto {
            instructions.push(command(mission::MISSION_START));
            messages.push("COMMAND_LONG MISSION_START".into());
        }
    } else {
        let first = hops.remove(0);
        instructions.push(reposition(drone, pose, first));
        messages.push("COMMAND_INT DO_REPOSITION".into());
        notes.resume = Some(Resume::Mission);
        notes.hops = hops;
        notes.checked = None;
    }
    *ops = notes;
    Orders {
        instructions,
        messages,
        reason,
        detours,
    }
}
