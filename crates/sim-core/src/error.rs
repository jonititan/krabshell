//! Error bodies (spec §9.2): `application/problem+json` with a stable `code`.
//!
//! Clients switch on `code`, never on the human `detail`. Adding a variant is
//! additive; renaming one is a breaking protocol change.

use serde::{Deserialize, Serialize};

use crate::cell::CellError;
use crate::config::ConfigError;
use crate::intent::RejectReason;
use crate::scenario::ScenarioError;

/// The closed vocabulary of machine-readable failures.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ErrorCode {
    BadRequest,
    Unauthorized,
    Forbidden,
    NotFound,
    PayloadTooLarge,
    UnknownScenario,
    MalformedScenario,
    BadCell,
    BadConfig,
    StaleIntent,
    IntentTooFar,
    Backpressured,
    DuplicateIntent,
    CellOutOfBounds,
    IntentRejected,
    /// A queued intent was thrown away when the world was replaced.
    IntentDiscarded,
    RateLimited,
    /// The requested change needs a paused driver.
    RequiresPause,
    /// The requested change needs a full reset.
    RequiresReset,
    /// The driver thread is gone or wedged.
    DriverUnavailable,
    /// The weather service could not supply the weather a scenario asks for.
    WeatherUnavailable,
    /// The tick body panicked.
    SimPanic,
    Internal,
}

impl ErrorCode {
    /// HTTP status this code is served with.
    pub fn status(self) -> u16 {
        match self {
            ErrorCode::BadRequest
            | ErrorCode::BadCell
            | ErrorCode::BadConfig
            | ErrorCode::MalformedScenario
            | ErrorCode::StaleIntent
            | ErrorCode::IntentTooFar
            | ErrorCode::DuplicateIntent
            | ErrorCode::CellOutOfBounds
            | ErrorCode::IntentRejected => 400,
            ErrorCode::Unauthorized => 401,
            ErrorCode::Forbidden => 403,
            ErrorCode::NotFound | ErrorCode::UnknownScenario => 404,
            ErrorCode::RequiresPause | ErrorCode::RequiresReset | ErrorCode::IntentDiscarded => 409,
            ErrorCode::PayloadTooLarge => 413,
            ErrorCode::RateLimited | ErrorCode::Backpressured => 429,
            ErrorCode::SimPanic | ErrorCode::Internal => 500,
            ErrorCode::DriverUnavailable => 503,
            ErrorCode::WeatherUnavailable => 502,
        }
    }

    pub fn as_str(self) -> &'static str {
        match self {
            ErrorCode::BadRequest => "bad_request",
            ErrorCode::Unauthorized => "unauthorized",
            ErrorCode::Forbidden => "forbidden",
            ErrorCode::NotFound => "not_found",
            ErrorCode::PayloadTooLarge => "payload_too_large",
            ErrorCode::UnknownScenario => "unknown_scenario",
            ErrorCode::MalformedScenario => "malformed_scenario",
            ErrorCode::BadCell => "bad_cell",
            ErrorCode::BadConfig => "bad_config",
            ErrorCode::StaleIntent => "stale_intent",
            ErrorCode::IntentTooFar => "intent_too_far",
            ErrorCode::Backpressured => "backpressured",
            ErrorCode::DuplicateIntent => "duplicate_intent",
            ErrorCode::CellOutOfBounds => "cell_out_of_bounds",
            ErrorCode::IntentRejected => "intent_rejected",
            ErrorCode::IntentDiscarded => "intent_discarded",
            ErrorCode::RateLimited => "rate_limited",
            ErrorCode::RequiresPause => "requires_pause",
            ErrorCode::RequiresReset => "requires_reset",
            ErrorCode::DriverUnavailable => "driver_unavailable",
            ErrorCode::WeatherUnavailable => "weather_unavailable",
            ErrorCode::SimPanic => "sim_panic",
            ErrorCode::Internal => "internal",
        }
    }
}

/// An RFC 7807-shaped error body.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Problem {
    /// Machine-readable code. The only field a client should branch on.
    pub code: ErrorCode,
    /// Short human title.
    pub title: String,
    /// Human explanation. Never parsed.
    pub detail: String,
    pub status: u16,
    /// Extra machine-readable context, e.g. the rejected field name.
    #[serde(default, skip_serializing_if = "serde_json::Value::is_null")]
    pub meta: serde_json::Value,
}

impl Problem {
    pub fn new(code: ErrorCode, detail: impl Into<String>) -> Self {
        Problem {
            code,
            title: code.as_str().replace('_', " "),
            detail: detail.into(),
            status: code.status(),
            meta: serde_json::Value::Null,
        }
    }

    pub fn with_meta(mut self, meta: serde_json::Value) -> Self {
        self.meta = meta;
        self
    }
}

impl std::fmt::Display for Problem {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}: {}", self.code.as_str(), self.detail)
    }
}

impl std::error::Error for Problem {}

impl From<CellError> for Problem {
    fn from(err: CellError) -> Self {
        Problem::new(ErrorCode::BadCell, err.to_string())
    }
}

impl From<ConfigError> for Problem {
    fn from(err: ConfigError) -> Self {
        Problem::new(ErrorCode::BadConfig, err.to_string())
    }
}

impl From<ScenarioError> for Problem {
    fn from(err: ScenarioError) -> Self {
        let code = match err {
            ScenarioError::Unknown(_) => ErrorCode::UnknownScenario,
            ScenarioError::Cell(_) => ErrorCode::BadCell,
            _ => ErrorCode::MalformedScenario,
        };
        Problem::new(code, err.to_string())
    }
}

impl From<RejectReason> for Problem {
    fn from(reason: RejectReason) -> Self {
        let code = match reason {
            RejectReason::Stale { .. } => ErrorCode::StaleIntent,
            RejectReason::TooFar { .. } => ErrorCode::IntentTooFar,
            RejectReason::Backpressured { .. } => ErrorCode::Backpressured,
            RejectReason::Duplicate => ErrorCode::DuplicateIntent,
            RejectReason::OutOfBounds => ErrorCode::CellOutOfBounds,
            RejectReason::Rejected { .. } => ErrorCode::IntentRejected,
            RejectReason::RateLimited => ErrorCode::RateLimited,
            RejectReason::Discarded { .. } => ErrorCode::IntentDiscarded,
        };
        let detail = reason.to_string();
        Problem::new(code, detail).with_meta(serde_json::to_value(&reason).unwrap_or_default())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_rejected_intent_becomes_a_problem_with_the_matching_code() {
        let problem: Problem = RejectReason::TooFar {
            target: 100,
            horizon: 8,
        }
        .into();
        assert_eq!(problem.code, ErrorCode::IntentTooFar);
        assert_eq!(problem.status, 400);
        assert_eq!(problem.meta["horizon"], 8);
    }

    #[test]
    fn codes_and_statuses_agree_after_serialisation() {
        let problem = Problem::new(ErrorCode::DriverUnavailable, "driver thread is gone");
        let json = serde_json::to_value(&problem).unwrap();
        assert_eq!(json["code"], "driver_unavailable");
        assert_eq!(json["status"], 503);
    }
}
