//! What the driver needs from a simulation, and nothing more.
//!
//! Everything specific to a world — agents, fields, rules — lives behind
//! [`Model`]. The driver knows only how to reset it, feed it admitted intents,
//! step it, and read a committed view out of it. That is the seam krABMaga
//! plugs into: `drones` wraps a krABMaga state and schedule (see
//! `models/drones`), and `walkers-krabmaga` a krABMaga `SparseA5Grid`.
//!
//! A `Model` is stepped on the tick thread and must never block, allocate
//! unboundedly, or touch Tokio (spec §6.3).

use std::collections::BTreeMap;

use sim_core::{CellBag, Intent, RejectReason, Report, ScenarioBlob};

/// How much of the world made it into a published view.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct Truncation {
    pub cells_written: u64,
    /// Cells left out because the view hit its cell cap.
    pub cells_omitted: u64,
}

impl Truncation {
    pub fn is_truncated(self) -> bool {
        self.cells_omitted > 0
    }
}

/// A simulation the driver can own.
pub trait Model: Send + 'static {
    /// Stable name, matching the `model` key in a scenario manifest.
    fn kind(&self) -> &'static str;

    /// Rebuild the world from a pack. Called on load and on reset.
    fn reset(&mut self, blob: &ScenarioBlob) -> anyhow::Result<()>;

    /// Apply one admitted intent to the write side, before the next step.
    ///
    /// Admission (dedup, horizon, bounds) has already happened; this is where
    /// game rules get a veto.
    fn apply_intent(&mut self, intent: &Intent) -> Result<(), RejectReason>;

    /// Advance exactly one tick, including the field commit.
    fn step(&mut self, tick: u64);

    /// A copy of the committed world that the driver can turn into a
    /// published view on another thread, so building it (every agent's state
    /// as JSON) does not hold up the next tick.
    ///
    /// It must be cheap to make, an `Arc` clone say, and its
    /// [`Frame::snapshot`] must write exactly what [`Model::snapshot`] would
    /// have at the moment it was taken. Models without one are snapshotted on
    /// the tick thread.
    fn frame(&self) -> Option<Box<dyn Frame>> {
        None
    }

    /// Write the committed occupancy into `out`, sorted by cell and by agent id
    /// within each cell.
    ///
    /// `out` is reused between publishes so the hot path does not reallocate.
    /// Stop after `limit` cells and report the remainder rather than growing
    /// without bound.
    fn snapshot(&self, out: &mut Vec<CellBag>, limit: usize) -> Truncation;

    fn agent_count(&self) -> u64;

    /// Anything else a view should carry that is not an agent in a cell,
    /// such as the drones' geofences. Built on the tick thread at every
    /// publish, so it must stay small. Null by default.
    fn overlays(&self) -> serde_json::Value {
        serde_json::Value::Null
    }

    fn occupied_cells(&self) -> u64;

    /// State that does not live in the grid, for checkpoints.
    fn fields(&self) -> serde_json::Value {
        serde_json::Value::Null
    }

    /// Counterpart of [`Model::fields`], called during a checkpoint restore.
    fn restore_fields(&mut self, _fields: &serde_json::Value) -> anyhow::Result<()> {
        Ok(())
    }

    /// The model's generator state, so a `gold` run resumes its exact sequence.
    fn rng_state(&self) -> u64;

    fn set_rng_state(&mut self, state: u64);

    /// Replace occupancy wholesale during a checkpoint restore.
    fn load_cells(&mut self, cells: Vec<CellBag>) -> anyhow::Result<()>;

    /// Move the events raised since the last call into `out`.
    ///
    /// Called by the driver after every successful step. Models that raise no
    /// events keep the default.
    fn drain_reports(&mut self, _out: &mut Vec<Report>) {}
}

/// A committed world, detached from the model it came from. See
/// [`Model::frame`].
pub trait Frame: Send + 'static {
    /// As [`Model::snapshot`], for the world as it was when the frame was taken.
    fn snapshot(&self, out: &mut Vec<CellBag>, limit: usize) -> Truncation;
}

/// Builds a model for a scenario. Registered by name at start-up.
pub type ModelFactory = fn(&ScenarioBlob) -> anyhow::Result<Box<dyn Model>>;

/// Name-to-factory map consulted when a scenario is loaded.
#[derive(Default, Clone)]
pub struct ModelRegistry {
    factories: BTreeMap<String, ModelFactory>,
}

impl ModelRegistry {
    /// The models this build ships with.
    pub fn with_builtins() -> Self {
        let mut registry = ModelRegistry::default();
        registry.register("drones", crate::models::drones::build);
        registry.register("walkers", crate::models::walkers::build);
        registry.register("walkers-krabmaga", crate::models::krabmaga_walkers::build);
        registry
    }

    pub fn register(&mut self, kind: impl Into<String>, factory: ModelFactory) {
        self.factories.insert(kind.into(), factory);
    }

    pub fn build(&self, blob: &ScenarioBlob) -> anyhow::Result<Box<dyn Model>> {
        let kind = blob.manifest.model.as_str();
        let factory = self.factories.get(kind).ok_or_else(|| {
            anyhow::anyhow!(
                "scenario '{}' wants model '{kind}', which this build does not have (available: {})",
                blob.manifest.id,
                self.kinds().join(", ")
            )
        })?;
        factory(blob)
    }

    pub fn kinds(&self) -> Vec<&str> {
        self.factories.keys().map(String::as_str).collect()
    }
}
