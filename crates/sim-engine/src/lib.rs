//! The simulation side of krabshell.
//!
//! One thread owns the world and runs it as fast as the model allows. Nothing
//! in this crate awaits, opens a socket, or reads a file during a tick: the
//! HTTP edge hands the driver decoded work and reads published snapshots back
//! (spec §4.1, §6.3).
//!
//! ```no_run
//! use sim_engine::{Driver, ModelRegistry};
//! use sim_core::ScenarioBlob;
//!
//! let driver = Driver::spawn(ModelRegistry::with_builtins(), ScenarioBlob::empty())?;
//! let handle = driver.handle();
//! handle.command(sim_engine::Control::Step { n: 100 })?;
//! println!("tick {}", handle.view().meta.tick);
//! # Ok::<(), anyhow::Error>(())
//! ```

pub mod admission;
pub mod checkpoint;
pub mod driver;
pub mod edge;
pub mod grid;
pub mod model;
pub mod models;
pub mod scenario;

pub use admission::{Admission, AdmissionOutcome};
pub use driver::{
    Command, Control, Driver, DriverError, DriverHandle, IntentAck, IntentRecord, Published,
    Submission,
};
pub use edge::{check_intent_cell, ActorLimiter, CellCheck};
pub use grid::SparseCellGrid;
pub use model::{Model, ModelFactory, ModelRegistry, Truncation};
pub use scenario::{read_pack, ScenarioStore};

/// Whether this build links krABMaga. It always does now: the drones model
/// runs on it.
///
/// The dashboard reports it, because "is the real engine underneath?" is the
/// first question anyone asks of a running server.
pub const HAS_KRABMAGA: bool = true;

/// Whether this build enables krABMaga's parallel feature.
pub const HAS_PARALLEL: bool = cfg!(feature = "parallel");

/// A one-line description of the engine backing this build.
pub fn engine_description() -> &'static str {
    if HAS_PARALLEL {
        "krABMaga (gis, parallel)"
    } else {
        "krABMaga (gis)"
    }
}
