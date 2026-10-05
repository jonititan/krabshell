//! Every agent but the dispatcher, stepped as one krABMaga agent on a pool of
//! threads.
//!
//! Most of a drones step is agents working alone: people and vehicles
//! wander, intruders fly their circuits, drones fly and look around. Only a
//! few things are shared and written: the order queues (deliveries and
//! courier duty), the reports, the sensor's memory of what it reported, and
//! the grid and board every agent places itself on. A [`Crowd`] steps in four
//! phases:
//!
//! 1. **Alone, in parallel.** Everyone but drones does their whole step;
//!    drones apply their mail and fly.
//! 2. **Settle, in id order.** Drones handle the waypoints they reached and
//!    their courier duty against the order queues.
//! 3. **Look, in parallel.** Drones whose camera is on scan the committed
//!    board, without writing the sensor's memory, and every drone finds its
//!    cell.
//! 4. **Publish, in id order.** Each agent's reports go out, its sightings are
//!    remembered, and it places itself.
//!
//! Each agent sees exactly what it would have seen stepping alone on the
//! schedule, in the same order, so a crowd's gold run matches the
//! schedule's on any number of threads.

use std::collections::HashMap;
use std::sync::Arc;

use krabmaga::bevy_a5::prelude::GeoCell as A5Cell;
use krabmaga::engine::agent::Agent;
use krabmaga::engine::state::State;
use rayon::prelude::*;
use sim_core::{AgentId, Report};

use crate::agents::{DroneAgent, Env, Member};
use crate::mission::Event;
use crate::sensor::Candidate;
use crate::world::{to_a5, Instruction, Kind, Record, World};

/// Fewest agents a thread takes at a time. Steps are a few hundred
/// nanoseconds; smaller batches spend more on stealing than on work.
const MIN_BATCH: usize = 64;

/// The agents, in id order, and their scratch for a step.
#[derive(Clone, Default)]
pub(crate) struct Crowd {
    members: Vec<Member>,
    slots: Vec<Slot>,
}

/// What one agent carries between phases.
#[derive(Clone, Default)]
struct Slot {
    mail: Vec<Instruction>,
    events: Vec<Event>,
    scans: bool,
    reports: Vec<Report>,
    seen: Vec<AgentId>,
    record: Option<Record>,
}

impl Crowd {
    pub(crate) fn new(mut members: Vec<Member>) -> Crowd {
        members.sort_by_key(|m| m.body().id);
        Crowd {
            members,
            slots: Vec::new(),
        }
    }

    fn step_in(&mut self, world: &mut World) {
        if !world.arrivals.is_empty() {
            self.members.append(&mut world.arrivals);
            self.members.sort_by_key(|m| m.body().id);
        }
        if !world.retiring.is_empty() {
            let retiring = &world.retiring;
            self.members.retain(|m| !retiring.contains(&m.body().id));
        }
        let pool = Arc::clone(world.crowd.as_ref().expect("a crowd steps on a pool"));
        let Crowd { members, slots } = self;
        slots.resize_with(members.len(), Slot::default);
        if world.has_mail() {
            for (member, slot) in members.iter().zip(slots.iter_mut()) {
                if let Member::Drone(d) = member {
                    slot.mail = world.take_mail(d.body.id);
                }
            }
        }

        // 1. Alone.
        let env = world.env();
        pool.install(|| {
            members
                .par_iter_mut()
                .zip(slots.par_iter_mut())
                .with_min_len(MIN_BATCH)
                .for_each(|(member, slot)| alone(member, slot, &env));
        });

        // 2. Settle.
        let tick = world.tick;
        let mut scanning = false;
        for (member, slot) in members.iter_mut().zip(slots.iter_mut()) {
            if let Member::Drone(d) = member {
                d.settle(world, tick, &slot.events, &mut slot.reports);
                slot.scans = d.scans(&world.env());
                if slot.scans {
                    world.warm_cap(d.body.cell);
                    scanning = true;
                }
            }
        }

        // 3. Look.
        let index = if scanning {
            occupancy(&world.board)
        } else {
            HashMap::new()
        };
        let look = Look {
            env: world.env(),
            board: &world.board,
            caps: &world.caps,
            index: &index,
            last_seen: &world.last_seen,
        };
        pool.install(|| {
            members
                .par_iter_mut()
                .zip(slots.par_iter_mut())
                .with_min_len(MIN_BATCH)
                .for_each(|(member, slot)| {
                    if let Member::Drone(d) = member {
                        if slot.scans {
                            look.scan(d, slot);
                        }
                        d.body.refresh(&look.env.bounds);
                        slot.record = Some(d.record());
                    }
                });
        });

        // 4. Publish.
        for (member, slot) in members.iter().zip(slots.iter_mut()) {
            let id = member.body().id;
            for report in slot.reports.drain(..) {
                world.report(report);
            }
            for target in slot.seen.drain(..) {
                world.last_seen.insert((id, target), tick);
            }
            slot.events.clear();
            slot.scans = false;
            world.place(slot.record.take().expect("every agent made its record"));
        }
    }
}

impl Agent for Crowd {
    fn step(&mut self, state: &mut dyn State) {
        let world = state
            .as_any_mut()
            .downcast_mut::<World>()
            .expect("a crowd steps in a drones World");
        self.step_in(world);
    }
}

/// Phase 1 for one agent.
fn alone(member: &mut Member, slot: &mut Slot, env: &Env) {
    match member {
        Member::Drone(d) => {
            let mail = std::mem::take(&mut slot.mail);
            d.fly(env, mail, &mut slot.events);
        }
        Member::Ground(g) => {
            g.advance(env);
            slot.record = Some(g.record());
        }
        Member::Intruder(i) => {
            i.advance(env);
            slot.record = Some(i.record());
        }
        Member::Poi(p) => slot.record = Some(p.record()),
    }
}

/// Who is in each cell, as committed: the grid's read side, from the board.
fn occupancy(board: &[Record]) -> HashMap<A5Cell, Vec<AgentId>> {
    let mut index: HashMap<A5Cell, Vec<AgentId>> = HashMap::new();
    for record in board {
        index.entry(to_a5(record.cell)).or_default().push(record.id);
    }
    index
}

/// What phase 3 reads.
struct Look<'a> {
    env: Env<'a>,
    board: &'a [Record],
    caps: &'a HashMap<sim_core::GeoCell, Vec<A5Cell>>,
    index: &'a HashMap<A5Cell, Vec<AgentId>>,
    last_seen: &'a std::collections::BTreeMap<(AgentId, AgentId), u64>,
}

impl Look<'_> {
    /// [`DroneAgent`]'s scan, reading the occupancy index for the grid and
    /// leaving the sensor's memory to phase 4.
    fn scan(&self, d: &DroneAgent, slot: &mut Slot) {
        let mut ids = Vec::new();
        for c in &self.caps[&d.body.cell] {
            if let Some(bag) = self.index.get(c) {
                ids.extend(bag);
            }
        }
        ids.sort_unstable();
        ids.dedup();

        let pose = &d.body.pose;
        let ground_mm = self.env.terrain.ground_mm(pose.lat_e7, pose.lon_e7);
        let candidates = ids
            .into_iter()
            .filter_map(|id| crate::agents::record_in(self.board, id))
            .filter(|r| r.kind != Kind::Drone)
            .map(|r| Candidate {
                id: r.id,
                at: r.saved.pose.point(),
                airborne: r.kind == Kind::Intruder,
            });
        let mut sightings = Vec::new();
        self.env.settings.sensor.scan_seen(
            self.env.seed,
            self.env.tick,
            d.body.id,
            &pose.point(),
            ground_mm,
            candidates,
            self.last_seen,
            &mut sightings,
        );
        for sighting in sightings {
            slot.seen.push(sighting.target);
            if let Some(report) = d.sighting_report(self.board, self.env.tick, &sighting) {
                slot.reports.push(report);
            }
        }
    }
}
