//! The intention protocol (spec §8).
//!
//! An intent is a request to change the world, addressed to a cell. It is not
//! a command: the driver may reject it, and a rejection is final — intents are
//! never retried on the client's behalf.

use serde::{Deserialize, Serialize};
use uuid::Uuid;

use crate::cell::GeoCell;

/// Identifies an agent within a run. Allocated by the model, not by clients.
pub type AgentId = u64;

/// Who submitted an intent. Used for per-actor rate limiting and the audit log.
#[derive(Debug, Clone, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(transparent)]
pub struct ActorId(pub String);

impl ActorId {
    /// The operator acting through the dashboard.
    pub fn operator() -> Self {
        ActorId("operator".to_owned())
    }
}

impl std::fmt::Display for ActorId {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(&self.0)
    }
}

/// What the intent asks for. `Custom` is the extension point every game-specific
/// action goes through; the engine dispatches on the code to a registered handler.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum IntentVerb {
    Spawn,
    Despawn,
    Move,
    SetField,
    Custom(u16),
}

/// A client-submitted world change.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Intent {
    /// Client-generated idempotency key. Duplicates are dropped, not applied twice.
    pub id: Uuid,
    /// Target tick. `None` means "the next tick the driver commits".
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub tick: Option<u64>,
    pub cell: GeoCell,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub agent: Option<AgentId>,
    pub actor: ActorId,
    pub verb: IntentVerb,
    /// Verb-defined body. Kept opaque here so `sim-core` never learns game rules.
    #[serde(default)]
    pub payload: serde_json::Value,
    /// Client wall clock in milliseconds. Diagnostic only — never trusted for ordering.
    #[serde(default)]
    pub issued_at: u64,
}

impl Intent {
    /// Build an operator intent with a fresh id and no tick constraint.
    pub fn operator(cell: GeoCell, verb: IntentVerb, payload: serde_json::Value) -> Self {
        Intent {
            id: Uuid::new_v4(),
            tick: None,
            cell,
            agent: None,
            actor: ActorId::operator(),
            verb,
            payload,
            issued_at: 0,
        }
    }
}

/// Why an intent was not applied. Every rejection is reported back to the
/// submitting connection with one of these codes.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize, thiserror::Error)]
#[serde(tag = "reason", rename_all = "snake_case")]
pub enum RejectReason {
    /// Target tick has already been committed.
    #[error("intent targets tick {target}, already committed through {committed}")]
    Stale { target: u64, committed: u64 },
    /// Target tick is further ahead than the driver is willing to hold.
    #[error("intent targets tick {target}, beyond horizon {horizon}")]
    TooFar { target: u64, horizon: u64 },
    /// The pending queue is full. The tick loop never stalls to make room.
    #[error("intent queue is full ({pending} pending)")]
    Backpressured { pending: usize },
    /// Already seen this id.
    #[error("duplicate intent id")]
    Duplicate,
    /// Cell lies outside the run's root, or at the wrong resolution.
    #[error("cell is outside the current world")]
    OutOfBounds,
    /// No handler registered for the verb, or the payload did not parse.
    #[error("model rejected the intent: {message}")]
    Rejected { message: String },
    /// Actor exceeded its submission rate.
    #[error("actor is submitting too fast")]
    RateLimited,
}

impl RejectReason {
    /// Stable machine code, matching the problem-response vocabulary.
    pub fn code(&self) -> &'static str {
        match self {
            RejectReason::Stale { .. } => "stale_intent",
            RejectReason::TooFar { .. } => "intent_too_far",
            RejectReason::Backpressured { .. } => "backpressured",
            RejectReason::Duplicate => "duplicate_intent",
            RejectReason::OutOfBounds => "cell_out_of_bounds",
            RejectReason::Rejected { .. } => "intent_rejected",
            RejectReason::RateLimited => "rate_limited",
        }
    }
}

/// One line of the append-only intent log used for `gold` replay (spec §8.4).
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct LoggedIntent {
    /// The tick the intent was actually applied on.
    pub tick: u64,
    pub intent: Intent,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn custom_verbs_survive_a_json_round_trip() {
        let cell = GeoCell::from_lonlat(-0.1276, 51.5072, 9).unwrap();
        let intent = Intent::operator(cell, IntentVerb::Custom(7), serde_json::json!({"n": 3}));
        let back: Intent = serde_json::from_str(&serde_json::to_string(&intent).unwrap()).unwrap();
        assert_eq!(back.verb, IntentVerb::Custom(7));
        assert_eq!(back.cell, cell);
        assert_eq!(back.id, intent.id);
    }

    #[test]
    fn a_minimal_client_intent_parses() {
        let intent: Intent = serde_json::from_value(serde_json::json!({
            "id": "6f1a1e2c-0000-4000-8000-000000000001",
            "cell": "0000000000001234",
            "actor": "player-9",
            "verb": "spawn"
        }))
        .unwrap();
        assert!(intent.tick.is_none());
        assert_eq!(intent.actor.0, "player-9");
    }

    #[test]
    fn rejections_carry_a_stable_code() {
        let reason = RejectReason::Stale {
            target: 4,
            committed: 9,
        };
        assert_eq!(reason.code(), "stale_intent");
        let json = serde_json::to_value(&reason).unwrap();
        assert_eq!(json["reason"], "stale");
        assert_eq!(json["committed"], 9);
    }
}
