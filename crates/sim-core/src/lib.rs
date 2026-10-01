//! Types only. No Bevy, no Axum, no krABMaga.
//!
//! `sim-core` is the vocabulary every other layer speaks: the A5 address space
//! ([`GeoCell`]), the intent schema clients submit, the snapshot shape the
//! driver publishes, and the run configuration an operator can download.
//!
//! The one dependency of substance is [`a5`], the pure-maths crate that
//! `bevy_a5` itself wraps. Depending on it directly is what lets the HTTP edge
//! answer cell queries without linking Bevy — see `docs/architecture.md`.

pub mod cell;
pub mod config;
pub mod error;
pub mod intent;
pub mod report;
pub mod rng;
pub mod scenario;
pub mod snapshot;
pub mod wire;

pub use cell::{CellError, GeoCell, MAX_RESOLUTION};
pub use config::{Config, GridConfig, Profile, RunConfig};
pub use error::{ErrorCode, Problem};
pub use intent::{ActorId, AgentId, Intent, IntentVerb, RejectReason};
pub use report::{Report, ReportKind};
pub use scenario::{ScenarioBlob, ScenarioError, ScenarioManifest, ScenarioSummary, SeedAgent};
pub use snapshot::{
    CellBag, Checkpoint, DriverHealth, Metrics, RunMode, SnapshotMeta, WorldObject,
};
pub use wire::{ClientMsg, ServerMsg};
