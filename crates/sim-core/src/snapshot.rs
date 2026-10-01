//! What the driver publishes and what an operator can download (spec §9).

use serde::{Deserialize, Serialize};

use crate::cell::GeoCell;
use crate::config::{Config, Profile};
use crate::intent::{AgentId, LoggedIntent};

/// What the driver is doing right now.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize, Default)]
#[serde(rename_all = "snake_case")]
pub enum RunMode {
    #[default]
    Paused,
    Running,
    /// Working through a bounded burst of steps, then back to `Paused`.
    Stepping,
    /// Swapping the world for a scenario or checkpoint.
    Loading,
    /// The tick body panicked. The world is frozen at the last good tick.
    Faulted,
}

impl RunMode {
    pub fn is_advancing(self) -> bool {
        matches!(self, RunMode::Running | RunMode::Stepping)
    }
}

/// Liveness of the tick thread, watched by the HTTP edge so that a dead driver
/// turns into a 503 rather than a hang (spec §12).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct DriverHealth {
    pub alive: bool,
    pub mode: RunMode,
    /// Milliseconds since the driver last published. Grows without bound if it wedges.
    pub last_publish_age_ms: u64,
    /// Panic message from the tick body, if any.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub fault: Option<String>,
    /// Last tick committed before the fault.
    pub last_good_tick: u64,
}

impl Default for DriverHealth {
    fn default() -> Self {
        DriverHealth {
            alive: true,
            mode: RunMode::Paused,
            last_publish_age_ms: 0,
            fault: None,
            last_good_tick: 0,
        }
    }
}

/// Counters the driver maintains in the hot path. All cheap to update: no
/// strings, no allocation (spec §6.3 rule 4).
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize, Default)]
pub struct Metrics {
    pub tick: u64,
    pub agent_count: u64,
    pub occupied_cells: u64,
    /// Ticks per second, measured over the last publish window.
    pub sim_tps: f64,
    /// Snapshots published per second.
    pub publish_hz: f64,
    pub last_step_ns: u64,
    pub intent_queue_depth: u64,
    pub intents_applied: u64,
    pub intents_rejected: u64,
    /// Snapshots the driver produced that no subscriber ever read.
    pub dropped_views: u64,
    /// Occupancy cells omitted from the published view because of the cell cap.
    pub truncated_cells: u64,
}

/// Header of any snapshot, download or WebSocket frame.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct SnapshotMeta {
    pub scenario_id: String,
    pub tick: u64,
    pub profile: Profile,
    /// `None` for a global grid, `Some` for an island run.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub root: Option<GeoCell>,
    /// Occupancy resolution, `R_work`.
    pub resolution: i32,
    pub agent_count: u64,
    pub occupied_cells: u64,
    /// Order-independent hash of the committed bags. Only produced in the
    /// `gold` profile, where it is the replay equality check.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub hash: Option<u64>,
}

/// One agent as it appears on the wire.
///
/// Deliberately schemaless past `kind`: v1 carries model state as JSON so a new
/// model needs no change to `sim-core` (spec §18.3).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct WorldObject {
    pub id: AgentId,
    pub kind: String,
    #[serde(default, skip_serializing_if = "serde_json::Value::is_null")]
    pub state: serde_json::Value,
}

/// The occupants of one cell.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct CellBag {
    pub cell: GeoCell,
    pub objects: Vec<WorldObject>,
}

impl CellBag {
    /// Canonical ordering for hashing and for diffing against a previous frame.
    pub fn sort(&mut self) {
        self.objects.sort_by_key(|o| o.id);
    }
}

/// A restorable world (spec §9.4). JSON in v1 for interoperability; a binary
/// encoding can be added later without changing this shape.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Checkpoint {
    pub meta: SnapshotMeta,
    pub config: Config,
    pub cells: Vec<CellBag>,
    /// Model state that does not live in the grid.
    #[serde(default)]
    pub fields: serde_json::Value,
    /// Driver RNG state, so a restored `gold` run continues the same sequence.
    #[serde(default)]
    pub rng: u64,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub intent_log: Option<Vec<LoggedIntent>>,
}

/// Order-independent hash of committed bags, for `gold` replay equality.
///
/// Per-cell contributions are combined with wrapping addition so the result
/// does not depend on `HashMap` iteration order (spec §16, "HashMap bag order
/// breaks gold"). Within a cell, objects must already be sorted by id.
pub fn stable_hash(cells: &[CellBag]) -> u64 {
    let mut total: u64 = 0;
    for bag in cells {
        let mut h = fnv_offset();
        fnv_u64(&mut h, bag.cell.raw());
        for object in &bag.objects {
            fnv_u64(&mut h, object.id);
            fnv_bytes(&mut h, object.kind.as_bytes());
            if !object.state.is_null() {
                // to_string is canonical enough here: serde_json preserves
                // insertion order and the models build state maps in fixed order.
                fnv_bytes(&mut h, object.state.to_string().as_bytes());
            }
        }
        total = total.wrapping_add(h);
    }
    total
}

const fn fnv_offset() -> u64 {
    0xcbf2_9ce4_8422_2325
}

fn fnv_bytes(h: &mut u64, bytes: &[u8]) {
    for b in bytes {
        *h ^= *b as u64;
        *h = h.wrapping_mul(0x0000_0100_0000_01b3);
    }
}

fn fnv_u64(h: &mut u64, v: u64) {
    fnv_bytes(h, &v.to_le_bytes());
}

#[cfg(test)]
mod tests {
    use super::*;

    fn bag(raw: u64, ids: &[u64]) -> CellBag {
        CellBag {
            cell: GeoCell::from_raw(raw),
            objects: ids
                .iter()
                .map(|id| WorldObject {
                    id: *id,
                    kind: "walker".to_owned(),
                    state: serde_json::json!({"heading": id % 5}),
                })
                .collect(),
        }
    }

    #[test]
    fn the_gold_hash_ignores_cell_order() {
        let forward = vec![bag(10, &[1, 2]), bag(20, &[3])];
        let reversed = vec![bag(20, &[3]), bag(10, &[1, 2])];
        assert_eq!(stable_hash(&forward), stable_hash(&reversed));
    }

    #[test]
    fn the_gold_hash_notices_a_moved_agent() {
        let before = vec![bag(10, &[1, 2]), bag(20, &[3])];
        let after = vec![bag(10, &[1]), bag(20, &[2, 3])];
        assert_ne!(stable_hash(&before), stable_hash(&after));
    }

    #[test]
    fn sorting_a_bag_makes_it_hashable() {
        let mut scrambled = bag(10, &[3, 1, 2]);
        scrambled.sort();
        assert_eq!(
            scrambled.objects.iter().map(|o| o.id).collect::<Vec<_>>(),
            vec![1, 2, 3]
        );
    }
}
