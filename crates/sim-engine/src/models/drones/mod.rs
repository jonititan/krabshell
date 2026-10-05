//! Drones flying MAVLink missions over a world of things to find.
//!
//! The model itself is [`sim_drones`]: every drone, intruder, person, vehicle
//! and point of interest is a krABMaga agent on a krABMaga schedule, stepping
//! against a [`World`] whose occupancy of record is krABMaga's
//! `SparseA5Grid`. This module is the seam to the driver: [`Drones`] owns a
//! world and its schedule and answers the driver's [`Model`] calls, and
//! [`drones_in`] is how gateways read drones back out of a published view.
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
pub mod heatmap;
pub mod runlog;

pub use sim_drones::{codes, dispatch, flight, mission, sensor, targets, terrain};

use std::collections::BTreeMap;

use sim_core::{AgentId, CellBag, GeoCell, Intent, RejectReason, Report, ScenarioBlob};
use sim_drones::krabmaga::engine::schedule::Schedule;
use sim_drones::krabmaga::engine::state::State;
use sim_drones::{Saved, World};

use crate::model::{Model, Truncation};
use flight::{Point, Pose};
use mission::{MissionItem, Mode};

/// The drones world and the krABMaga schedule its agents step on.
pub struct Drones {
    world: World,
    schedule: Schedule,
}

/// Registered as `drones` in the model registry.
pub fn build(blob: &ScenarioBlob) -> anyhow::Result<Box<dyn Model>> {
    Ok(Box::new(Drones::new(blob)?))
}

impl Drones {
    pub fn new(blob: &ScenarioBlob) -> anyhow::Result<Drones> {
        let mut world = World::new(blob)?;
        let mut schedule = sim_drones::schedule();
        world.init(&mut schedule);
        Ok(Drones { world, schedule })
    }

    pub fn world(&self) -> &World {
        &self.world
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

impl Model for Drones {
    fn kind(&self) -> &'static str {
        "drones"
    }

    fn reset(&mut self, blob: &ScenarioBlob) -> anyhow::Result<()> {
        *self = Drones::new(blob)?;
        Ok(())
    }

    fn apply_intent(&mut self, intent: &Intent) -> Result<(), RejectReason> {
        self.world.apply_intent(intent)
    }

    fn step(&mut self, tick: u64) {
        self.world.step_at(&mut self.schedule, tick);
    }

    fn snapshot(&self, out: &mut Vec<CellBag>, limit: usize) -> Truncation {
        let cells_omitted = self.world.snapshot(out, limit);
        Truncation {
            cells_written: out.len() as u64,
            cells_omitted,
        }
    }

    fn agent_count(&self) -> u64 {
        self.world.agent_count()
    }

    fn occupied_cells(&self) -> u64 {
        self.world.occupied_cells()
    }

    fn fields(&self) -> serde_json::Value {
        self.world.fields()
    }

    fn restore_fields(&mut self, fields: &serde_json::Value) -> anyhow::Result<()> {
        self.world.restore_fields(fields)
    }

    fn rng_state(&self) -> u64 {
        self.world.rng_state()
    }

    fn set_rng_state(&mut self, state: u64) {
        self.world.set_rng_state(state);
    }

    fn load_cells(&mut self, cells: Vec<CellBag>) -> anyhow::Result<()> {
        // The agents are rebuilt from the checkpoint, on a schedule of their own.
        self.schedule = sim_drones::schedule();
        self.world.restore(cells, &mut self.schedule)
    }

    fn drain_reports(&mut self, out: &mut Vec<Report>) {
        self.world.drain_reports(out);
    }
}

#[cfg(test)]
mod tests;
