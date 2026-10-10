//! Fetched fields on disk, keyed by the request, so running a scenario again
//! does not ask Open-Meteo again.
//!
//! One JSON file per request, named by a fingerprint of everything that
//! decides the answer: the host, the model, the cells, the hours, and, for a
//! window reaching past now, the hour it was fetched in. That last part is
//! what expires forecasts: in the next hour the key is new, so the next fetch
//! misses and gets the newer forecast. Windows wholly in the past never
//! change, so their files are good for ever.
//!
//! Files are written to a temporary name and renamed, so a crash or two
//! servers sharing a directory never leave half a file to be read.

use std::path::{Path, PathBuf};

use sim_core::WindField;

#[derive(Debug, Clone)]
pub struct Cache {
    dir: PathBuf,
}

impl Cache {
    pub fn new(dir: impl Into<PathBuf>) -> Cache {
        Cache { dir: dir.into() }
    }

    pub fn dir(&self) -> &Path {
        &self.dir
    }

    fn path(&self, key: &str) -> PathBuf {
        self.dir.join(format!("{key}.json"))
    }

    /// The field cached under `key`, if there is a readable one.
    pub fn get(&self, key: &str) -> Option<WindField> {
        let bytes = std::fs::read(self.path(key)).ok()?;
        let value: serde_json::Value = serde_json::from_slice(&bytes).ok()?;
        match WindField::from_value(&value) {
            Ok(field) => Some(field),
            Err(err) => {
                tracing::warn!(key, %err, "ignoring an unreadable weather cache entry");
                None
            }
        }
    }

    pub fn put(&self, key: &str, field: &WindField) -> std::io::Result<()> {
        std::fs::create_dir_all(&self.dir)?;
        let tmp = self.dir.join(format!(".{key}.{}.tmp", std::process::id()));
        std::fs::write(&tmp, serde_json::to_vec(field)?)?;
        std::fs::rename(&tmp, self.path(key))
    }

    /// How many entries, and their total size in bytes.
    pub fn usage(&self) -> (usize, u64) {
        let Ok(entries) = std::fs::read_dir(&self.dir) else {
            return (0, 0);
        };
        entries
            .filter_map(Result::ok)
            .filter(|e| e.path().extension().is_some_and(|x| x == "json"))
            .fold((0, 0), |(n, bytes), e| {
                (n + 1, bytes + e.metadata().map_or(0, |m| m.len()))
            })
    }
}
