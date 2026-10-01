//! Scenario packs (spec §10.1).
//!
//! A pack is a directory (or zip) of `manifest.toml`, `config.toml`,
//! `params.toml` and an optional `world/`. Decoding happens on Tokio; the
//! driver only ever receives a fully decoded [`ScenarioBlob`].

use serde::{Deserialize, Serialize};

use crate::cell::{CellError, GeoCell};
use crate::config::{Config, ModelParams, Profile};
use crate::intent::AgentId;

/// `manifest.toml`. Identity and address space; everything else lives in
/// `config.toml`.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ScenarioManifest {
    /// Load key. Must be filesystem-safe: it names the pack directory.
    pub id: String,
    pub name: String,
    #[serde(default)]
    pub description: String,
    /// Which registered model to instantiate.
    pub model: String,
    /// krABMaga revision the pack was authored against, for a load-time warning.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub engine_rev: Option<String>,
    /// Occupancy resolution. Overrides `config.toml`'s `[grid]`.
    pub resolution: i32,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub root: Option<GeoCell>,
    #[serde(default)]
    pub profile: Profile,
}

impl ScenarioManifest {
    /// Ids name a directory, so they may not contain path syntax.
    pub fn validate_id(&self) -> Result<(), ScenarioError> {
        let ok = !self.id.is_empty()
            && self.id.len() <= 64
            && self
                .id
                .chars()
                .all(|c| c.is_ascii_alphanumeric() || c == '-' || c == '_');
        if ok {
            Ok(())
        } else {
            Err(ScenarioError::BadId(self.id.clone()))
        }
    }

    /// Fold manifest-owned fields into a config loaded from the same pack.
    /// The manifest wins: it is the pack's identity, the config is its defaults.
    pub fn apply_to(&self, config: &mut Config) {
        config.grid.resolution = self.resolution;
        config.grid.root = self.root;
        config.run.profile = self.profile;
        if config.grid.view_resolution > config.grid.resolution {
            config.grid.view_resolution = (config.grid.resolution - 4).max(0);
        }
    }
}

/// One pre-seeded agent from `world/agents.json`.
///
/// Position may be given as a cell or as lon/lat; lon/lat is compiled to a cell
/// at the pack's resolution on load, so authored packs stay readable.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SeedAgent {
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub id: Option<AgentId>,
    pub kind: String,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub cell: Option<GeoCell>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub lon: Option<f64>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub lat: Option<f64>,
    #[serde(default, skip_serializing_if = "serde_json::Value::is_null")]
    pub state: serde_json::Value,
}

impl SeedAgent {
    /// Where this agent starts, at the given occupancy resolution.
    pub fn resolve_cell(&self, resolution: i32) -> Result<GeoCell, ScenarioError> {
        match (self.cell, self.lon, self.lat) {
            (Some(cell), _, _) => Ok(cell),
            (None, Some(lon), Some(lat)) => {
                GeoCell::from_lonlat(lon, lat, resolution).map_err(ScenarioError::Cell)
            }
            _ => Err(ScenarioError::AgentWithoutPosition {
                kind: self.kind.clone(),
            }),
        }
    }
}

/// A decoded pack, ready to hand to the driver over the control channel.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ScenarioBlob {
    pub manifest: ScenarioManifest,
    /// Effective config: `config.toml` with the manifest folded in.
    pub config: Config,
    #[serde(default)]
    pub agents: Vec<SeedAgent>,
    /// Non-grid model state from `world/fields.json`.
    #[serde(default)]
    pub fields: serde_json::Value,
}

impl ScenarioBlob {
    /// An empty world at default settings, used by `POST /run/reset` before any
    /// pack has been loaded.
    pub fn empty() -> Self {
        let manifest = ScenarioManifest {
            id: "empty".to_owned(),
            name: "Empty world".to_owned(),
            description: "No agents. Loaded when the server starts with no scenario.".to_owned(),
            model: "walkers".to_owned(),
            engine_rev: None,
            resolution: Config::default().grid.resolution,
            root: None,
            profile: Profile::default(),
        };
        let mut config = Config::default();
        manifest.apply_to(&mut config);
        ScenarioBlob {
            manifest,
            config,
            agents: Vec::new(),
            fields: serde_json::Value::Null,
        }
    }

    /// Merge model parameters from `params.toml`.
    pub fn with_params(mut self, params: ModelParams) -> Self {
        self.config.model.extend(params);
        self
    }
}

/// A pack as it appears in `GET /scenarios`.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ScenarioSummary {
    pub id: String,
    pub name: String,
    pub description: String,
    pub model: String,
    pub resolution: i32,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub root: Option<GeoCell>,
    pub profile: Profile,
    pub seed_agents: u64,
    /// Where the pack was found, relative to the scenarios root.
    pub source: String,
}

#[derive(Debug, Clone, PartialEq, thiserror::Error)]
pub enum ScenarioError {
    #[error("scenario id '{0}' must be 1-64 chars of [A-Za-z0-9_-]")]
    BadId(String),
    #[error("no scenario with id '{0}'")]
    Unknown(String),
    #[error("seed agent of kind '{kind}' has neither a cell nor lon/lat")]
    AgentWithoutPosition { kind: String },
    #[error("{file}: {message}")]
    Malformed { file: String, message: String },
    #[error(transparent)]
    Cell(#[from] CellError),
}

#[cfg(test)]
mod tests {
    use super::*;

    fn manifest() -> ScenarioManifest {
        ScenarioManifest {
            id: "london-walkers".into(),
            name: "London walkers".into(),
            description: String::new(),
            model: "walkers".into(),
            engine_rev: None,
            resolution: 11,
            root: Some(GeoCell::from_lonlat(-0.1276, 51.5072, 3).unwrap()),
            profile: Profile::Gold,
        }
    }

    #[test]
    fn the_manifest_owns_the_address_space() {
        let mut config = Config::default();
        config.grid.resolution = 2;
        manifest().apply_to(&mut config);
        assert_eq!(config.grid.resolution, 11);
        assert_eq!(config.run.profile, Profile::Gold);
        assert!(config.grid.view_resolution <= config.grid.resolution);
        config.validate().unwrap();
    }

    #[test]
    fn a_view_resolution_finer_than_the_pack_is_pulled_back() {
        let mut config = Config::default();
        config.grid.view_resolution = 8;
        let mut coarse = manifest();
        coarse.resolution = 4;
        coarse.apply_to(&mut config);
        assert_eq!(config.grid.view_resolution, 0);
        config.validate().unwrap();
    }

    #[test]
    fn path_syntax_in_an_id_is_refused() {
        let mut evil = manifest();
        evil.id = "../../etc".into();
        assert!(matches!(evil.validate_id(), Err(ScenarioError::BadId(_))));
        assert!(manifest().validate_id().is_ok());
    }

    #[test]
    fn seed_agents_may_be_authored_in_lon_lat() {
        let agent = SeedAgent {
            id: None,
            kind: "walker".into(),
            cell: None,
            lon: Some(-0.1276),
            lat: Some(51.5072),
            state: serde_json::Value::Null,
        };
        let cell = agent.resolve_cell(11).unwrap();
        assert_eq!(cell.resolution(), 11);

        let floating = SeedAgent { lon: None, ..agent };
        assert!(matches!(
            floating.resolve_cell(11),
            Err(ScenarioError::AgentWithoutPosition { .. })
        ));
    }

    #[test]
    fn the_empty_world_is_a_valid_starting_point() {
        let blob = ScenarioBlob::empty();
        blob.config.validate().unwrap();
        assert!(blob.agents.is_empty());
    }
}
