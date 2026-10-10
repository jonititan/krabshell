//! Reading a scenario pack directory.
//!
//! A pack is plain files (`manifest.toml`, optional `config.toml`,
//! `params.toml`, `world/agents.json`, `world/fields.json`, and any other
//! `world/<name>.json`, which joins the fields as `<name>`), so anything that
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

    let mut fields: serde_json::Value = match optional_string(dir, "world/fields.json")? {
        Some(text) => serde_json::from_str(&text).map_err(|e| malformed("world/fields.json", e))?,
        None => serde_json::Value::Null,
    };
    extra_fields(dir, &mut fields)?;

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

/// Every other `world/<name>.json` joins the fields as `<name>`, so a model
/// can take hand-written data (a drones pack's `world/geofences.json`, say)
/// beside a generated `fields.json` without the two sharing a file.
fn extra_fields(dir: &Path, fields: &mut serde_json::Value) -> Result<(), ScenarioError> {
    let Ok(entries) = std::fs::read_dir(dir.join("world")) else {
        return Ok(());
    };
    let mut names: Vec<String> = entries
        .filter_map(Result::ok)
        .filter_map(|e| e.file_name().into_string().ok())
        .filter(|n| n.ends_with(".json") && n != "agents.json" && n != "fields.json")
        .collect();
    names.sort();
    for name in names {
        let file = format!("world/{name}");
        let text = read_to_string(dir, &file)?;
        let value: serde_json::Value =
            serde_json::from_str(&text).map_err(|e| malformed(&file, e))?;
        if fields.is_null() {
            *fields = serde_json::Value::Object(Default::default());
        }
        let key = name.trim_end_matches(".json");
        let Some(map) = fields.as_object_mut() else {
            return Err(malformed(
                &file,
                "world/fields.json is not an object to add it to",
            ));
        };
        if map.contains_key(key) {
            return Err(malformed(
                &file,
                format!("world/fields.json already has a \"{key}\" entry"),
            ));
        }
        map.insert(key.to_owned(), value);
    }
    Ok(())
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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn other_world_files_join_the_fields_under_their_names() {
        let dir = std::env::temp_dir().join(format!("krabshell-pack-{}", std::process::id()));
        std::fs::create_dir_all(dir.join("world")).unwrap();
        std::fs::write(
            dir.join("manifest.toml"),
            "id = \"t\"\nname = \"t\"\nmodel = \"walkers\"\nresolution = 4\n",
        )
        .unwrap();
        std::fs::write(dir.join("world/fields.json"), r#"{"terrain": 1}"#).unwrap();
        std::fs::write(dir.join("world/geofences.json"), r#"[{"name": "x"}]"#).unwrap();
        let blob = read_pack(&dir).unwrap();
        assert_eq!(blob.fields["terrain"], 1);
        assert_eq!(blob.fields["geofences"][0]["name"], "x");

        // A name fields.json already has is refused, not silently replaced.
        std::fs::write(dir.join("world/terrain.json"), "2").unwrap();
        assert!(read_pack(&dir).is_err());
        std::fs::remove_dir_all(&dir).unwrap();
    }
}
