//! Checkpoint files (spec §9.4).
//!
//! v1 is JSON, for interoperability and because an operator should be able to
//! read one. The round-trip contract is the part that matters: uploading a
//! checkpoint restores the same tick, and in the `gold` profile the same hash.

use std::path::Path;

use sim_core::{Checkpoint, Profile};

#[derive(Debug, thiserror::Error)]
pub enum CheckpointError {
    #[error("checkpoint is not valid JSON: {0}")]
    Malformed(#[from] serde_json::Error),
    #[error("checkpoint io: {0}")]
    Io(#[from] std::io::Error),
    #[error("checkpoint hash mismatch: file says {expected:x}, contents hash to {actual:x}")]
    HashMismatch { expected: u64, actual: u64 },
}

/// Serialise a checkpoint for download. Pretty-printed: these get read by people.
pub fn encode(checkpoint: &Checkpoint) -> Result<Vec<u8>, CheckpointError> {
    Ok(serde_json::to_vec_pretty(checkpoint)?)
}

/// Parse an uploaded checkpoint and verify what can be verified before the
/// driver is asked to adopt it.
pub fn decode(bytes: &[u8]) -> Result<Checkpoint, CheckpointError> {
    let checkpoint: Checkpoint = serde_json::from_slice(bytes)?;
    verify(&checkpoint)?;
    Ok(checkpoint)
}

/// Check a `gold` checkpoint against its own hash. `fast` checkpoints carry no
/// hash, so there is nothing to check.
pub fn verify(checkpoint: &Checkpoint) -> Result<(), CheckpointError> {
    if checkpoint.meta.profile != Profile::Gold {
        return Ok(());
    }
    let Some(expected) = checkpoint.meta.hash else {
        return Ok(());
    };
    let actual = sim_core::snapshot::stable_hash(&checkpoint.cells);
    if expected == actual {
        Ok(())
    } else {
        Err(CheckpointError::HashMismatch { expected, actual })
    }
}

/// A filename an operator will still recognise a month later.
pub fn suggested_filename(checkpoint: &Checkpoint) -> String {
    format!(
        "{}-tick{}.ckpt.json",
        checkpoint.meta.scenario_id, checkpoint.meta.tick
    )
}

pub fn write_to(path: &Path, checkpoint: &Checkpoint) -> Result<(), CheckpointError> {
    std::fs::write(path, encode(checkpoint)?)?;
    Ok(())
}

pub fn read_from(path: &Path) -> Result<Checkpoint, CheckpointError> {
    decode(&std::fs::read(path)?)
}

#[cfg(test)]
mod tests {
    use super::*;
    use sim_core::{CellBag, Config, GeoCell, SnapshotMeta, WorldObject};

    fn checkpoint(profile: Profile) -> Checkpoint {
        let cells = vec![CellBag {
            cell: GeoCell::from_lonlat(-0.1276, 51.5072, 7).unwrap(),
            objects: vec![WorldObject {
                id: 1,
                kind: "walker".into(),
                state: serde_json::json!({"steps": 4}),
            }],
        }];
        let mut config = Config::default();
        config.run.profile = profile;
        config.grid.resolution = 7;
        Checkpoint {
            meta: SnapshotMeta {
                scenario_id: "demo".into(),
                tick: 128,
                profile,
                root: None,
                resolution: 7,
                agent_count: 1,
                occupied_cells: 1,
                hash: (profile == Profile::Gold).then(|| sim_core::snapshot::stable_hash(&cells)),
            },
            config,
            cells,
            fields: serde_json::json!({"next_id": 2}),
            rng: 0xdead_beef,
            intent_log: None,
        }
    }

    #[test]
    fn a_checkpoint_round_trips_and_keeps_its_tick() {
        let original = checkpoint(Profile::Gold);
        let back = decode(&encode(&original).unwrap()).unwrap();
        assert_eq!(back.meta.tick, 128);
        assert_eq!(back.meta.hash, original.meta.hash);
        assert_eq!(back.cells, original.cells);
        assert_eq!(back.rng, 0xdead_beef);
    }

    #[test]
    fn a_tampered_gold_checkpoint_is_rejected() {
        let mut tampered = checkpoint(Profile::Gold);
        tampered.cells[0].objects[0].id = 99;
        let err = decode(&encode(&tampered).unwrap()).unwrap_err();
        assert!(matches!(err, CheckpointError::HashMismatch { .. }));
    }

    #[test]
    fn a_fast_checkpoint_carries_no_hash_to_check() {
        let fast = checkpoint(Profile::Fast);
        assert!(fast.meta.hash.is_none());
        assert!(decode(&encode(&fast).unwrap()).is_ok());
    }

    #[test]
    fn the_filename_says_what_it_is() {
        assert_eq!(
            suggested_filename(&checkpoint(Profile::Fast)),
            "demo-tick128.ckpt.json"
        );
    }
}
