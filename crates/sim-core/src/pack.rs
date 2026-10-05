//! Reading a scenario pack directory.
//!
//! A pack is plain files (`manifest.toml`, optional `config.toml`,
//! `params.toml`, `world/agents.json`, `world/fields.json`), so anything that
//! runs a model can load one: the krabshell driver, `simctl`, or a krABMaga
//! binary with no server at all. Zip uploads and the store of packs stay in
//! `sim-engine`.

use std::collections::BTreeMap;
use std::path::Path;

use crate::config::{Config, ModelParams};
use crate::{ScenarioBlob, ScenarioError, ScenarioManifest, SeedAgent};

/// Read a pack directory into a blob.
pub fn read_pack(dir: &Path) -> Result<ScenarioBlob, ScenarioError> {
    let manifest_text = read_to_string(dir, "manifest.toml")?;
    let manifest: ScenarioManifest =
        toml::from_str(&manifest_text).map_err(|e| malformed("manifest.toml", e))?;
    manifest.validate_id()?;

    let mut config: Config = match optional_string(dir, "config.toml")? {
        Some(text) => toml::from_str(&text).map_err(|e| malformed("config.toml", e))?,
        None => Config::default(),
    };
    manifest.apply_to(&mut config);

    let params: ModelParams = match optional_string(dir, "params.toml")? {
        Some(text) => toml::from_str(&text).map_err(|e| malformed("params.toml", e))?,
        None => BTreeMap::new(),
    };

    let agents: Vec<SeedAgent> = match optional_string(dir, "world/agents.json")? {
        Some(text) => serde_json::from_str(&text).map_err(|e| malformed("world/agents.json", e))?,
        None => Vec::new(),
    };

    let fields: serde_json::Value = match optional_string(dir, "world/fields.json")? {
        Some(text) => serde_json::from_str(&text).map_err(|e| malformed("world/fields.json", e))?,
        None => serde_json::Value::Null,
    };

    let blob = ScenarioBlob {
        manifest,
        config,
        agents,
        fields,
    }
    .with_params(params);

    blob.config
        .validate()
        .map_err(|e| ScenarioError::Malformed {
            file: "config.toml".to_owned(),
            message: e.to_string(),
        })?;
    Ok(blob)
}

fn read_to_string(dir: &Path, name: &str) -> Result<String, ScenarioError> {
    std::fs::read_to_string(dir.join(name)).map_err(|e| ScenarioError::Malformed {
        file: name.to_owned(),
        message: e.to_string(),
    })
}

fn optional_string(dir: &Path, name: &str) -> Result<Option<String>, ScenarioError> {
    let path = dir.join(name);
    if !path.is_file() {
        return Ok(None);
    }
    read_to_string(dir, name).map(Some)
}

fn malformed(file: &str, message: impl ToString) -> ScenarioError {
    ScenarioError::Malformed {
        file: file.to_owned(),
        message: message.to_string(),
    }
}
