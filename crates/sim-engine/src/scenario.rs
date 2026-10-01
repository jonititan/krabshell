//! Reading scenario packs off disk (spec §10.1).
//!
//! All of this runs on Tokio or a blocking pool, never on the tick thread: the
//! driver is only ever handed a fully decoded [`ScenarioBlob`] (spec §6.3).
//!
//! Uploads are treated as hostile. A pack is operator-supplied, but "operator"
//! is a role, not a guarantee, so the zip reader enforces the limits in §13.

use std::collections::BTreeMap;
use std::io::Read;
use std::path::{Path, PathBuf};

use sim_core::config::{Config, ModelParams};
use sim_core::{ScenarioBlob, ScenarioError, ScenarioManifest, ScenarioSummary, SeedAgent};

/// Largest a single pack may expand to. Guards against a zip bomb.
pub const MAX_PACK_BYTES: u64 = 256 * 1024 * 1024;

/// Largest number of files a pack may contain.
pub const MAX_PACK_ENTRIES: usize = 4_096;

/// A directory of scenario packs.
#[derive(Debug, Clone)]
pub struct ScenarioStore {
    root: PathBuf,
}

impl ScenarioStore {
    pub fn new(root: impl Into<PathBuf>) -> Self {
        ScenarioStore { root: root.into() }
    }

    pub fn root(&self) -> &Path {
        &self.root
    }

    /// Every readable pack, sorted by id. Unreadable packs are logged and
    /// skipped rather than failing the whole listing.
    pub fn list(&self) -> Vec<ScenarioSummary> {
        let Ok(entries) = std::fs::read_dir(&self.root) else {
            return Vec::new();
        };
        let mut packs: Vec<ScenarioSummary> = entries
            .flatten()
            .filter(|e| e.path().is_dir())
            .filter_map(|entry| match read_pack(&entry.path()) {
                Ok(blob) => Some(summarise(&blob, &entry.file_name().to_string_lossy())),
                Err(err) => {
                    tracing::warn!(path = %entry.path().display(), %err, "skipping unreadable scenario pack");
                    None
                }
            })
            .collect();
        packs.sort_by(|a, b| a.id.cmp(&b.id));
        packs
    }

    /// Load one pack by manifest id.
    pub fn load(&self, id: &str) -> Result<ScenarioBlob, ScenarioError> {
        let dir = self.resolve(id)?;
        read_pack(&dir)
    }

    pub fn exists(&self, id: &str) -> bool {
        self.resolve(id)
            .is_ok_and(|dir| dir.join("manifest.toml").is_file())
    }

    /// Unpack an uploaded zip into the store as `id`.
    ///
    /// Refuses anything that would write outside the pack directory, and
    /// anything that expands past [`MAX_PACK_BYTES`].
    pub fn import_zip(&self, id: &str, bytes: &[u8]) -> Result<ScenarioSummary, ScenarioError> {
        validate_id(id)?;
        let target = self.root.join(id);
        let staging = self.root.join(format!(".{id}.incoming"));
        let _ = std::fs::remove_dir_all(&staging);
        std::fs::create_dir_all(&staging).map_err(|e| io_error("staging directory", e))?;

        let result = extract_zip(bytes, &staging).and_then(|()| read_pack(&staging));
        match result {
            Ok(blob) => {
                if blob.manifest.id != id {
                    let _ = std::fs::remove_dir_all(&staging);
                    return Err(ScenarioError::Malformed {
                        file: "manifest.toml".to_owned(),
                        message: format!(
                            "manifest id '{}' does not match the upload id '{id}'",
                            blob.manifest.id
                        ),
                    });
                }
                let _ = std::fs::remove_dir_all(&target);
                std::fs::rename(&staging, &target).map_err(|e| io_error("installing pack", e))?;
                Ok(summarise(&blob, id))
            }
            Err(err) => {
                let _ = std::fs::remove_dir_all(&staging);
                Err(err)
            }
        }
    }

    /// Map an id to a directory, refusing anything that escapes the store.
    fn resolve(&self, id: &str) -> Result<PathBuf, ScenarioError> {
        validate_id(id)?;
        let dir = self.root.join(id);
        if !dir.is_dir() {
            return Err(ScenarioError::Unknown(id.to_owned()));
        }
        Ok(dir)
    }
}

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

fn summarise(blob: &ScenarioBlob, source: &str) -> ScenarioSummary {
    ScenarioSummary {
        id: blob.manifest.id.clone(),
        name: blob.manifest.name.clone(),
        description: blob.manifest.description.clone(),
        model: blob.manifest.model.clone(),
        resolution: blob.manifest.resolution,
        root: blob.manifest.root,
        profile: blob.manifest.profile,
        seed_agents: blob.agents.len() as u64,
        source: source.to_owned(),
    }
}

fn extract_zip(bytes: &[u8], into: &Path) -> Result<(), ScenarioError> {
    let reader = std::io::Cursor::new(bytes);
    let mut archive =
        zip::ZipArchive::new(reader).map_err(|e| malformed("upload", format!("not a zip: {e}")))?;

    if archive.len() > MAX_PACK_ENTRIES {
        return Err(malformed(
            "upload",
            format!(
                "{} entries exceeds the limit of {MAX_PACK_ENTRIES}",
                archive.len()
            ),
        ));
    }

    let mut written: u64 = 0;
    for index in 0..archive.len() {
        let mut entry = archive
            .by_index(index)
            .map_err(|e| malformed("upload", format!("entry {index}: {e}")))?;

        // `enclosed_name` is None for absolute paths and for anything with a
        // `..` component, which is exactly the zip-slip case (spec §13).
        let Some(relative) = entry.enclosed_name() else {
            return Err(malformed(
                "upload",
                format!("entry '{}' escapes the pack directory", entry.name()),
            ));
        };

        written = written.saturating_add(entry.size());
        if written > MAX_PACK_BYTES {
            return Err(malformed(
                "upload",
                format!("pack expands past the {MAX_PACK_BYTES} byte limit"),
            ));
        }

        let destination = into.join(&relative);
        if entry.is_dir() {
            std::fs::create_dir_all(&destination).map_err(|e| io_error("creating directory", e))?;
            continue;
        }
        if let Some(parent) = destination.parent() {
            std::fs::create_dir_all(parent).map_err(|e| io_error("creating directory", e))?;
        }

        let mut body = Vec::with_capacity(entry.size().min(1 << 20) as usize);
        entry
            .read_to_end(&mut body)
            .map_err(|e| malformed("upload", format!("reading '{}': {e}", relative.display())))?;
        std::fs::write(&destination, &body).map_err(|e| io_error("writing pack file", e))?;
    }
    Ok(())
}

fn validate_id(id: &str) -> Result<(), ScenarioError> {
    let ok = !id.is_empty()
        && id.len() <= 64
        && id
            .chars()
            .all(|c| c.is_ascii_alphanumeric() || c == '-' || c == '_');
    if ok {
        Ok(())
    } else {
        Err(ScenarioError::BadId(id.to_owned()))
    }
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

fn io_error(what: &str, err: std::io::Error) -> ScenarioError {
    ScenarioError::Malformed {
        file: what.to_owned(),
        message: err.to_string(),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::io::Write;

    fn temp_dir(tag: &str) -> PathBuf {
        let dir = std::env::temp_dir().join(format!(
            "krabshell-{tag}-{}-{}",
            std::process::id(),
            std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap()
                .as_nanos()
        ));
        std::fs::create_dir_all(&dir).unwrap();
        dir
    }

    fn write_pack(root: &Path, id: &str) -> PathBuf {
        let dir = root.join(id);
        std::fs::create_dir_all(dir.join("world")).unwrap();
        std::fs::write(
            dir.join("manifest.toml"),
            format!(
                r#"
id = "{id}"
name = "Test pack"
model = "walkers"
resolution = 6
profile = "gold"
"#
            ),
        )
        .unwrap();
        std::fs::write(
            dir.join("config.toml"),
            "[run]\npublish_hz = 10.0\n\n[grid]\nview_resolution = 3\n",
        )
        .unwrap();
        std::fs::write(dir.join("params.toml"), "walkers = 25\nspread_k = 2\n").unwrap();
        std::fs::write(
            dir.join("world/agents.json"),
            r#"[{"kind":"walker","lon":-0.1276,"lat":51.5072}]"#,
        )
        .unwrap();
        dir
    }

    #[test]
    fn a_pack_loads_with_the_manifest_overriding_the_config() {
        let root = temp_dir("pack");
        write_pack(&root, "demo");
        let store = ScenarioStore::new(&root);

        let blob = store.load("demo").unwrap();
        assert_eq!(blob.manifest.model, "walkers");
        assert_eq!(blob.config.grid.resolution, 6, "manifest wins");
        assert_eq!(blob.config.run.publish_hz, 10.0, "config.toml is kept");
        assert_eq!(blob.config.run.profile, sim_core::Profile::Gold);
        assert_eq!(
            blob.config.model.get("walkers").and_then(|p| p.as_i64()),
            Some(25),
            "params.toml is merged in"
        );
        assert_eq!(blob.agents.len(), 1);

        let listed = store.list();
        assert_eq!(listed.len(), 1);
        assert_eq!(listed[0].seed_agents, 1);

        std::fs::remove_dir_all(&root).unwrap();
    }

    #[test]
    fn an_id_that_walks_up_the_tree_is_refused() {
        let root = temp_dir("escape");
        let store = ScenarioStore::new(&root);
        assert!(matches!(
            store.load("../../etc"),
            Err(ScenarioError::BadId(_))
        ));
        assert!(matches!(store.load("nope"), Err(ScenarioError::Unknown(_))));
        std::fs::remove_dir_all(&root).unwrap();
    }

    #[test]
    fn a_zip_upload_installs_a_pack() {
        let root = temp_dir("zip");
        let store = ScenarioStore::new(&root);

        let mut buffer = Vec::new();
        {
            let mut zip = zip::ZipWriter::new(std::io::Cursor::new(&mut buffer));
            let options: zip::write::FileOptions<'_, ()> = zip::write::FileOptions::default();
            zip.start_file("manifest.toml", options).unwrap();
            zip.write_all(
                b"id = \"uploaded\"\nname = \"Uploaded\"\nmodel = \"walkers\"\nresolution = 5\n",
            )
            .unwrap();
            zip.start_file("params.toml", options).unwrap();
            zip.write_all(b"walkers = 3\n").unwrap();
            zip.finish().unwrap();
        }

        let summary = store.import_zip("uploaded", &buffer).unwrap();
        assert_eq!(summary.id, "uploaded");
        assert!(store.exists("uploaded"));
        assert_eq!(store.load("uploaded").unwrap().config.grid.resolution, 5);

        std::fs::remove_dir_all(&root).unwrap();
    }

    #[test]
    fn a_zip_that_escapes_the_pack_directory_is_refused() {
        let root = temp_dir("slip");
        let store = ScenarioStore::new(&root);

        let mut buffer = Vec::new();
        {
            let mut zip = zip::ZipWriter::new(std::io::Cursor::new(&mut buffer));
            let options: zip::write::FileOptions<'_, ()> = zip::write::FileOptions::default();
            zip.start_file("../../escaped.toml", options).unwrap();
            zip.write_all(b"pwned = true\n").unwrap();
            zip.finish().unwrap();
        }

        let err = store.import_zip("evil", &buffer).unwrap_err();
        assert!(err.to_string().contains("escapes"), "{err}");
        assert!(!root.join("evil").exists(), "staging must be cleaned up");
        assert!(!root.parent().unwrap().join("escaped.toml").exists());

        std::fs::remove_dir_all(&root).unwrap();
    }

    #[test]
    fn a_manifest_that_disagrees_with_the_upload_id_is_refused() {
        let root = temp_dir("mismatch");
        let store = ScenarioStore::new(&root);

        let mut buffer = Vec::new();
        {
            let mut zip = zip::ZipWriter::new(std::io::Cursor::new(&mut buffer));
            let options: zip::write::FileOptions<'_, ()> = zip::write::FileOptions::default();
            zip.start_file("manifest.toml", options).unwrap();
            zip.write_all(b"id = \"other\"\nname = \"X\"\nmodel = \"walkers\"\nresolution = 5\n")
                .unwrap();
            zip.finish().unwrap();
        }

        assert!(store.import_zip("claimed", &buffer).is_err());
        assert!(!store.exists("claimed"));
        std::fs::remove_dir_all(&root).unwrap();
    }
}
