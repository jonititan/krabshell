//! Drones flying MAVLink missions over a world of things to find, as a
//! krABMaga model.
//!
//! Every entity is a krABMaga [`Agent`](krabmaga::engine::agent::Agent) on the
//! [`Schedule`](krabmaga::engine::schedule::Schedule), and [`World`] is the
//! simulation [`State`](krabmaga::engine::state::State) they step against:
//!
//! - [`agents::DroneAgent`]: flown by ground stations through krabshell's
//!   MAVLink gateway, or by its own courier duty. Missions and commands arrive
//!   as intents (see [`codes`]) carrying the same fields as `MISSION_ITEM_INT`
//!   and `COMMAND_LONG`. Its camera scans the grid around it on a fixed
//!   cadence.
//! - [`agents::IntruderAgent`]: an uncontrolled drone flying a fixed circuit.
//! - [`agents::GroundAgent`]: a person or vehicle wandering near where it
//!   started.
//! - [`agents::PoiAgent`]: a named point of interest that never moves.
//! - [`agents::Dispatcher`]: first on every step, opens and closes geofences
//!   (see [`airspace`]), places delivery orders, and lets the [`operator`]
//!   reroute the drones it flies round the fences, by MAVLink mission upload
//!   and command.
//!
//! Occupancy of record is krABMaga's `SparseA5Grid`, double-buffered the
//! krABMaga way: during a step, agents read the committed side (where
//! everything was at the end of the last step) and write themselves to the
//! other, and [`World`] commits it once every agent has stepped. Each agent
//! publishes its full state alongside, so other agents, gateways and
//! checkpoints read the same committed view.
//!
//! The model runs inside krabshell (`sim-engine` wraps [`World`] as its
//! `drones` model) or on its own with the `krab-drones` binary, which loads a
//! scenario pack and hands it to krABMaga's `simulate!`.
//!
//! Poses are integers and every random choice comes from a (seed, entity,
//! tick) stream; agents step in id order. `gold` runs replay bit for bit.

pub mod agents;
pub mod airspace;
pub mod dispatch;
pub mod flight;
pub mod mission;
pub mod operator;
pub mod sensor;
mod settings;
pub mod targets;
pub mod terrain;
pub mod world;

pub use krabmaga;
pub use world::{codes, schedule, Committed, Kind, Record, Saved, World};

#[cfg(test)]
mod tests;
