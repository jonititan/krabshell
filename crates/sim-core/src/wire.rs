//! The WebSocket protocol (spec §9.3).
//!
//! The dashboard and any other viewer speak exactly this. Keeping it in
//! `sim-core` — with no Axum and no Bevy in scope — is what stops the wire
//! format drifting towards whichever client was written most recently.

use serde::{Deserialize, Serialize};
use uuid::Uuid;

use crate::cell::GeoCell;
use crate::error::ErrorCode;
use crate::intent::{AgentId, Intent, RejectReason};
use crate::snapshot::{CellBag, Metrics, SnapshotMeta, WorldObject};

/// Client to server.
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(tag = "type", rename_all = "snake_case")]
pub enum ClientMsg {
    /// Add cells to this connection's area of interest.
    Subscribe {
        cells: Vec<GeoCell>,
        /// Aggregate to this resolution instead of sending occupancy cells.
        #[serde(default, skip_serializing_if = "Option::is_none")]
        res: Option<i32>,
    },
    Unsubscribe {
        cells: Vec<GeoCell>,
    },
    /// Replace the area of interest with a disk. The server expands it.
    SetAoi {
        center: GeoCell,
        k: u32,
        #[serde(default, skip_serializing_if = "Option::is_none")]
        res: Option<i32>,
    },
    /// Submit an intent on the live connection. Acknowledged either way.
    Intent {
        intent: Intent,
    },
    Ping,
}

/// One cell's worth of change.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Removal {
    pub cell: GeoCell,
    /// Agents that left. Empty means the whole cell emptied.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub objects: Vec<AgentId>,
}

/// Server to client.
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(tag = "type", rename_all = "snake_case")]
pub enum ServerMsg {
    /// Sent once on connect, before anything else.
    Hello {
        tick: u64,
        meta: SnapshotMeta,
        /// Protocol version. Bumped whenever a client would have to change.
        protocol: u32,
    },
    /// Full contents of the subscribed cells. Sent when an AOI changes.
    Snapshot {
        tick: u64,
        cells: Vec<CellBag>,
    },
    /// Change since the last frame sent to *this* connection.
    Delta {
        tick: u64,
        #[serde(default, skip_serializing_if = "Vec::is_empty")]
        upsert: Vec<CellBag>,
        #[serde(default, skip_serializing_if = "Vec::is_empty")]
        remove: Vec<Removal>,
    },
    /// The intent was queued for a tick.
    Accepted {
        intent_id: Uuid,
    },
    /// The intent will never be applied.
    Rejected {
        intent_id: Uuid,
        #[serde(flatten)]
        reason: RejectReason,
    },
    /// Rate-limited heartbeat, so a viewer can show progress without deltas.
    Tick {
        tick: u64,
        #[serde(flatten)]
        metrics: Metrics,
    },
    Error {
        code: ErrorCode,
        message: String,
    },
    Pong,
}

/// Current protocol version, sent in [`ServerMsg::Hello`].
pub const PROTOCOL_VERSION: u32 = 1;

/// Diff two sorted cell views into the upsert/remove pair a client needs.
///
/// Both sides must be sorted by cell, and each bag's objects sorted by id —
/// [`CellBag::sort`] and a sort of the outer vec. Doing the diff here rather
/// than in the driver keeps per-client work off the tick thread (spec §6.3).
pub fn diff(previous: &[CellBag], current: &[CellBag]) -> (Vec<CellBag>, Vec<Removal>) {
    let mut upsert = Vec::new();
    let mut remove = Vec::new();
    let (mut i, mut j) = (0usize, 0usize);

    while i < previous.len() || j < current.len() {
        match (previous.get(i), current.get(j)) {
            (Some(old), Some(new)) if old.cell == new.cell => {
                if old.objects != new.objects {
                    let gone = departed(&old.objects, &new.objects);
                    if !new.objects.is_empty() {
                        upsert.push(new.clone());
                    }
                    if !gone.is_empty() {
                        remove.push(Removal {
                            cell: old.cell,
                            objects: gone,
                        });
                    }
                }
                i += 1;
                j += 1;
            }
            (Some(old), Some(new)) if old.cell < new.cell => {
                remove.push(Removal {
                    cell: old.cell,
                    objects: Vec::new(),
                });
                i += 1;
            }
            (Some(_), Some(new)) => {
                upsert.push(new.clone());
                j += 1;
            }
            (Some(old), None) => {
                remove.push(Removal {
                    cell: old.cell,
                    objects: Vec::new(),
                });
                i += 1;
            }
            (None, Some(new)) => {
                upsert.push(new.clone());
                j += 1;
            }
            (None, None) => unreachable!("loop condition guarantees one side is non-empty"),
        }
    }

    (upsert, remove)
}

fn departed(old: &[WorldObject], new: &[WorldObject]) -> Vec<AgentId> {
    old.iter()
        .map(|o| o.id)
        .filter(|id| new.binary_search_by_key(id, |o| o.id).is_err())
        .collect()
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
                    kind: "walker".into(),
                    state: serde_json::Value::Null,
                })
                .collect(),
        }
    }

    #[test]
    fn an_unchanged_world_produces_no_frame() {
        let world = vec![bag(1, &[1, 2]), bag(2, &[3])];
        let (upsert, remove) = diff(&world, &world);
        assert!(upsert.is_empty() && remove.is_empty());
    }

    #[test]
    fn an_agent_moving_cells_shows_up_on_both_sides() {
        let before = vec![bag(1, &[1, 2]), bag(2, &[3])];
        let after = vec![bag(1, &[1]), bag(2, &[2, 3])];
        let (upsert, remove) = diff(&before, &after);

        assert_eq!(upsert.len(), 2);
        assert_eq!(remove.len(), 1);
        assert_eq!(remove[0].cell, GeoCell::from_raw(1));
        assert_eq!(remove[0].objects, vec![2]);
    }

    #[test]
    fn an_emptied_cell_is_removed_whole() {
        let before = vec![bag(1, &[1]), bag(2, &[3])];
        let after = vec![bag(2, &[3])];
        let (upsert, remove) = diff(&before, &after);
        assert!(upsert.is_empty());
        assert_eq!(
            remove,
            vec![Removal {
                cell: GeoCell::from_raw(1),
                objects: vec![]
            }]
        );
    }

    #[test]
    fn a_new_cell_is_a_pure_upsert() {
        let before = vec![bag(2, &[3])];
        let after = vec![bag(1, &[9]), bag(2, &[3])];
        let (upsert, remove) = diff(&before, &after);
        assert_eq!(upsert.len(), 1);
        assert_eq!(upsert[0].cell, GeoCell::from_raw(1));
        assert!(remove.is_empty());
    }

    #[test]
    fn client_messages_use_a_type_tag() {
        let msg: ClientMsg = serde_json::from_value(serde_json::json!({
            "type": "set_aoi",
            "center": "0000000000001234",
            "k": 2
        }))
        .unwrap();
        assert!(matches!(msg, ClientMsg::SetAoi { k: 2, .. }));
    }

    #[test]
    fn a_rejection_flattens_its_reason_next_to_the_intent_id() {
        let msg = ServerMsg::Rejected {
            intent_id: Uuid::nil(),
            reason: RejectReason::OutOfBounds,
        };
        let json = serde_json::to_value(&msg).unwrap();
        assert_eq!(json["type"], "rejected");
        assert_eq!(json["reason"], "out_of_bounds");
    }
}
