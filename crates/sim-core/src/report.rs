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
    /// `observer` delivered an order at the given position. `detail` carries
    /// the order and when it was placed, dispatched and delivered.
    Delivery,
    /// `observer` touched down at the end of a flight. `detail.battery_pct`
    /// is the charge it landed with.
    Landing,
    /// `observer` ran out of battery in the air and fell at the given
    /// position. `detail` says what replaced it, if anything.
    Crash,
    /// A geofence opened or closed. `detail` carries its id, shape and who
    /// asked for it; the position is its centre.
    Geofence,
    /// `observer` was inside geofence `detail.fence` in the air: it flew in,
    /// or the fence opened round it.
    Breach,
    /// The operator sent `observer` MAVLink messages to take it round a
    /// geofence: `detail.messages` lists them.
    Reroute,
    /// An order was turned away when it was placed, at the given address:
    /// `detail.reason` says why (inside a geofence, say).
    OrderRejected,
    /// A launch and recovery station was added or removed. `detail` carries
    /// the station and `action`; the position is the station.
    Station,
    /// A region of interest was designated or removed (`detail.action`), or
    /// added by itself where a drone crashed. The position is its centre.
    Roi,
    /// `observer` finished surveying region of interest `detail.roi`.
    Survey,
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
