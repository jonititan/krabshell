//! Things a model tells the outside world about, as opposed to state it holds.
//!
//! A snapshot answers "where is everything now". A report answers "what just
//! happened": a sensor saw a target, a vehicle reached a waypoint. Gateways
//! turn reports into their own protocols (CoT sightings, MAVLink
//! `MISSION_ITEM_REACHED`) without learning the model's internals.
//!
//! The driver stamps every report with a sequence number that only ever grows,
//! across resets and scenario loads, so a consumer that remembers the last
//! `seq` it handled can pick up exactly the new ones from each publish.

use serde::{Deserialize, Serialize};

use crate::intent::AgentId;

/// What kind of event a report describes.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ReportKind {
    /// `observer` sensed `target` at the given position.
    Detection,
    /// `observer` reached mission item `detail.seq`.
    Waypoint,
}

/// One model-generated event.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Report {
    /// Assigned by the driver; models leave it at 0.
    #[serde(default)]
    pub seq: u64,
    pub tick: u64,
    pub kind: ReportKind,
    /// Stable identity of the thing reported on, for de-duplicating downstream.
    pub uid: String,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub observer: Option<AgentId>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub target: Option<AgentId>,
    /// The target's kind, e.g. `person`, `vehicle`, `intruder`, `poi`.
    #[serde(default, skip_serializing_if = "String::is_empty")]
    pub target_kind: String,
    /// Position in degrees × 10⁷ and millimetres above the home datum, the
    /// same integer units MAVLink's `_INT` messages use.
    pub lat_e7: i32,
    pub lon_e7: i32,
    pub alt_mm: i32,
    #[serde(default, skip_serializing_if = "serde_json::Value::is_null")]
    pub detail: serde_json::Value,
}

impl Report {
    pub fn lat(&self) -> f64 {
        f64::from(self.lat_e7) * 1e-7
    }

    pub fn lon(&self) -> f64 {
        f64::from(self.lon_e7) * 1e-7
    }
}
