//! Run configuration (spec §10.2).
//!
//! This is both the file an operator downloads and the object the driver reads.
//! `GET /config/download` serialises the *effective* config, not whatever is on
//! disk, so this type has to be lossless in both TOML and JSON.

use std::collections::BTreeMap;

use serde::{Deserialize, Serialize};

use crate::cell::{GeoCell, MAX_RESOLUTION};

/// Determinism profile (spec §6.5).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize, Default)]
#[serde(rename_all = "snake_case")]
pub enum Profile {
    /// Science and replay: sequential, sorted bags, seeded RNG, hashable.
    Gold,
    /// Games and visualisation: parallel allowed, iteration order undefined.
    #[default]
    Fast,
}

impl Profile {
    pub fn is_gold(self) -> bool {
        matches!(self, Profile::Gold)
    }
}

/// A model parameter. Small closed set so the same value survives a TOML file,
/// a JSON `PUT /config`, and a checkpoint without a lossy hop through `Value`.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(untagged)]
pub enum Param {
    Bool(bool),
    Int(i64),
    Float(f64),
    Text(String),
    List(Vec<Param>),
}

impl Param {
    pub fn as_i64(&self) -> Option<i64> {
        match self {
            Param::Int(v) => Some(*v),
            Param::Float(v) => Some(*v as i64),
            _ => None,
        }
    }

    pub fn as_f64(&self) -> Option<f64> {
        match self {
            Param::Int(v) => Some(*v as f64),
            Param::Float(v) => Some(*v),
            _ => None,
        }
    }

    pub fn as_bool(&self) -> Option<bool> {
        match self {
            Param::Bool(v) => Some(*v),
            _ => None,
        }
    }

    pub fn as_str(&self) -> Option<&str> {
        match self {
            Param::Text(v) => Some(v),
            _ => None,
        }
    }
}

/// Free-form parameters handed to the model on reset. `sim-core` never reads them.
pub type ModelParams = BTreeMap<String, Param>;

/// Everything about how the loop runs, as opposed to what it simulates.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct RunConfig {
    #[serde(default)]
    pub profile: Profile,
    /// Enable krABMaga's `parallel` schedule. Changing it requires a reset.
    #[serde(default)]
    pub parallel: bool,
    #[serde(default = "default_threads")]
    pub threads: u32,
    /// Ticks per second to aim for. Absent means unconstrained — maximum speed.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub target_tps: Option<f64>,
    /// Upper bound on snapshot publishes per second, whatever the tick rate.
    #[serde(default = "default_publish_hz")]
    pub publish_hz: f64,
    /// How many ticks ahead a client may schedule an intent.
    #[serde(default = "default_intent_horizon")]
    pub intent_horizon: u64,
    #[serde(default = "default_max_pending")]
    pub max_pending_intents: usize,
    #[serde(default = "default_max_applied")]
    pub max_applied_per_tick: usize,
    /// Ceiling on cells in one published view. Protects the wire from a world
    /// that grows faster than anyone can read it.
    #[serde(default = "default_max_published_cells")]
    pub max_published_cells: usize,
    /// Sustained intents per second one actor may submit (spec §13). Bursts of
    /// twice this are allowed. Enforced at the edge, before the driver's queue.
    #[serde(default = "default_intent_rate_per_actor")]
    pub intent_rate_per_actor: f64,
    /// Restart the driver automatically after a tick panic (spec §12).
    #[serde(default)]
    pub auto_recover: bool,
}

/// The address space this run occupies.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct GridConfig {
    /// Occupancy resolution, `R_work`.
    #[serde(default = "default_resolution")]
    pub resolution: i32,
    /// Island root. Absent means a global grid.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub root: Option<GeoCell>,
    /// Coarser resolution the dashboard aggregates to, `R_view`.
    #[serde(default = "default_view_resolution")]
    pub view_resolution: i32,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize, Default)]
#[serde(deny_unknown_fields)]
pub struct Config {
    #[serde(default)]
    pub run: RunConfig,
    #[serde(default)]
    pub grid: GridConfig,
    /// Passed verbatim to the model on reset.
    #[serde(default)]
    pub model: ModelParams,
}

#[derive(Debug, Clone, PartialEq, Eq, thiserror::Error)]
pub enum ConfigError {
    #[error("{field} must be {expected}, got {got}")]
    OutOfRange {
        field: &'static str,
        expected: &'static str,
        got: String,
    },
    #[error("view_resolution {view} must be coarser than occupancy resolution {work}")]
    ViewNotCoarser { view: i32, work: i32 },
    #[error("root cell is at resolution {root}, which is finer than the grid's {work}")]
    RootTooFine { root: i32, work: i32 },
}

impl Config {
    /// Reject a config the driver could not honour. Called before any config is
    /// accepted from the network and again before the driver adopts it.
    pub fn validate(&self) -> Result<(), ConfigError> {
        let run = &self.run;
        if run.threads == 0 {
            return Err(out_of_range("run.threads", "at least 1", run.threads));
        }
        if !(run.publish_hz > 0.0 && run.publish_hz <= 1000.0) {
            return Err(out_of_range(
                "run.publish_hz",
                "in 0..=1000",
                run.publish_hz,
            ));
        }
        if let Some(tps) = run.target_tps {
            if !(tps > 0.0 && tps.is_finite()) {
                return Err(out_of_range("run.target_tps", "positive or absent", tps));
            }
        }
        if run.max_pending_intents == 0 {
            return Err(out_of_range(
                "run.max_pending_intents",
                "at least 1",
                run.max_pending_intents,
            ));
        }
        if run.max_applied_per_tick == 0 {
            return Err(out_of_range(
                "run.max_applied_per_tick",
                "at least 1",
                run.max_applied_per_tick,
            ));
        }
        if !(run.intent_rate_per_actor > 0.0 && run.intent_rate_per_actor.is_finite()) {
            return Err(out_of_range(
                "run.intent_rate_per_actor",
                "positive",
                run.intent_rate_per_actor,
            ));
        }
        if run.max_published_cells == 0 {
            return Err(out_of_range(
                "run.max_published_cells",
                "at least 1",
                run.max_published_cells,
            ));
        }

        let grid = &self.grid;
        check_is_resolution("grid.resolution", grid.resolution)?;
        check_is_resolution("grid.view_resolution", grid.view_resolution)?;
        if grid.view_resolution > grid.resolution {
            return Err(ConfigError::ViewNotCoarser {
                view: grid.view_resolution,
                work: grid.resolution,
            });
        }
        if let Some(root) = grid.root {
            let root_res = root.resolution();
            if root_res > grid.resolution {
                return Err(ConfigError::RootTooFine {
                    root: root_res,
                    work: grid.resolution,
                });
            }
        }
        Ok(())
    }

    /// Fields that cannot change under a running world. The dashboard uses this
    /// to decide between "apply" and "apply and reset".
    pub fn changes_requiring_reset(&self, next: &Config) -> Vec<&'static str> {
        let mut fields = Vec::new();
        if self.run.profile != next.run.profile {
            fields.push("run.profile");
        }
        if self.run.parallel != next.run.parallel {
            fields.push("run.parallel");
        }
        if self.run.threads != next.run.threads {
            fields.push("run.threads");
        }
        if self.grid.resolution != next.grid.resolution {
            fields.push("grid.resolution");
        }
        if self.grid.root != next.grid.root {
            fields.push("grid.root");
        }
        if self.model != next.model {
            fields.push("model");
        }
        fields
    }

    /// Interval between publishes, honouring `publish_hz`.
    pub fn publish_interval(&self) -> std::time::Duration {
        std::time::Duration::from_secs_f64(1.0 / self.run.publish_hz)
    }

    /// Interval between ticks, or `None` at maximum speed.
    pub fn tick_interval(&self) -> Option<std::time::Duration> {
        self.run
            .target_tps
            .map(|tps| std::time::Duration::from_secs_f64(1.0 / tps))
    }
}

fn out_of_range(field: &'static str, expected: &'static str, got: impl ToString) -> ConfigError {
    ConfigError::OutOfRange {
        field,
        expected,
        got: got.to_string(),
    }
}

fn check_is_resolution(field: &'static str, value: i32) -> Result<(), ConfigError> {
    if (0..=MAX_RESOLUTION).contains(&value) {
        Ok(())
    } else {
        Err(out_of_range(field, "a valid A5 resolution", value))
    }
}

impl Default for RunConfig {
    fn default() -> Self {
        RunConfig {
            profile: Profile::default(),
            parallel: false,
            threads: default_threads(),
            target_tps: None,
            publish_hz: default_publish_hz(),
            intent_horizon: default_intent_horizon(),
            max_pending_intents: default_max_pending(),
            max_applied_per_tick: default_max_applied(),
            max_published_cells: default_max_published_cells(),
            intent_rate_per_actor: default_intent_rate_per_actor(),
            auto_recover: false,
        }
    }
}

impl Default for GridConfig {
    fn default() -> Self {
        GridConfig {
            resolution: default_resolution(),
            root: None,
            view_resolution: default_view_resolution(),
        }
    }
}

fn default_threads() -> u32 {
    std::thread::available_parallelism()
        .map(|n| n.get() as u32)
        .unwrap_or(4)
}
fn default_publish_hz() -> f64 {
    20.0
}
fn default_intent_horizon() -> u64 {
    8
}
fn default_max_pending() -> usize {
    10_000
}
fn default_max_applied() -> usize {
    1_000
}
fn default_intent_rate_per_actor() -> f64 {
    200.0
}
fn default_max_published_cells() -> usize {
    50_000
}
/// About 180 m per cell, the same figure the spec's example config uses.
fn default_resolution() -> i32 {
    15
}
/// About 2.8 km per cell: coarse enough that a dashboard map of a city is a
/// few hundred polygons rather than a few hundred thousand.
fn default_view_resolution() -> i32 {
    11
}

const _: () = assert!(MAX_RESOLUTION > 0);

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn defaults_are_valid_and_mean_maximum_speed() {
        let config = Config::default();
        config.validate().unwrap();
        assert!(config.run.target_tps.is_none());
        assert!(config.tick_interval().is_none());
        assert_eq!(config.publish_interval().as_millis(), 50);
    }

    #[test]
    fn the_downloadable_file_round_trips_through_toml() {
        let mut config = Config::default();
        config.run.profile = Profile::Gold;
        config.run.target_tps = Some(60.0);
        config.grid.root = Some(GeoCell::from_lonlat(-0.1276, 51.5072, 3).unwrap());
        config
            .model
            .insert("infection_rate".into(), Param::Float(0.25));
        config.model.insert("walkers".into(), Param::Int(10_000));

        let text = toml::to_string_pretty(&config).unwrap();
        let back: Config = toml::from_str(&text).unwrap();
        assert_eq!(back, config);
    }

    #[test]
    fn the_same_config_round_trips_through_json() {
        let config = Config::default();
        let back: Config = serde_json::from_str(&serde_json::to_string(&config).unwrap()).unwrap();
        assert_eq!(back, config);
    }

    #[test]
    fn a_view_finer_than_occupancy_is_refused() {
        let mut config = Config::default();
        config.grid.view_resolution = config.grid.resolution + 1;
        assert!(matches!(
            config.validate(),
            Err(ConfigError::ViewNotCoarser { .. })
        ));
    }

    #[test]
    fn structural_changes_are_flagged_as_needing_a_reset() {
        let before = Config::default();
        let mut after = before.clone();
        after.run.publish_hz = 5.0;
        assert!(before.changes_requiring_reset(&after).is_empty());

        after.run.parallel = true;
        after.grid.resolution = 12;
        assert_eq!(
            before.changes_requiring_reset(&after),
            vec!["run.parallel", "grid.resolution"]
        );
    }

    #[test]
    fn unknown_keys_are_rejected_rather_than_silently_dropped() {
        let err = toml::from_str::<Config>("[run]\nprofil = \"gold\"\n").unwrap_err();
        assert!(err.to_string().contains("profil"), "{err}");
    }
}
