//! Everything logged about one run of a drones pack: the flight heatmap and
//! the delivered orders. Fed from published views, so the server and
//! `simctl` log the same way; writing the files is left to the caller.
//!
//! A pack asks for a log by setting `heatmap_resolution` or
//! `order_rate_per_address_day` in its params. The files are:
//!
//! - `heatmap.geojson`: drone-seconds of flight per A5 cell (see
//!   [`super::heatmap`]), when `heatmap_resolution` is set;
//! - `deliveries.csv`: every delivered order; `rejected-orders.csv`, every
//!   order turned away when placed (its address inside a geofence) and why;
//! - `delivery-times.csv` and `delivery-times.svg`: a histogram of time from
//!   order to delivery;
//! - `landings.csv`: every flight's end, landed or crashed, with the battery
//!   percentage it touched down with (-1 for a crash);
//! - `landing-battery.csv` and `landing-battery.svg`: a histogram of those
//!   percentages, crashes in a bin of their own;
//! - `geofences.geojson`: every geofence opened during the run and every
//!   breach of one; `breaches.csv`, every breach; `reroutes.csv`, the MAVLink
//!   messages the operator sent to take drones round them;
//! - `summary.json`: the run's span and totals.

use sim_core::config::{Config, Param};

use super::deliveries::DeliveryLog;
use super::drones_in;
use super::geofences::GeofenceLog;
use super::heatmap::{self, Heatmap};
use super::landings::LandingLog;
use crate::Published;

/// Does this config ask for a run log?
pub fn wanted(config: &Config) -> bool {
    heatmap::settings(config).is_some()
        || config
            .model
            .get("order_rate_per_address_day")
            .and_then(Param::as_f64)
            .is_some_and(|r| r > 0.0)
}

pub struct RunLog {
    pub scenario: String,
    pub heatmap: Option<Heatmap>,
    pub deliveries: DeliveryLog,
    pub landings: LandingLog,
    pub geofences: GeofenceLog,
    last_tick: Option<u64>,
    last_report: u64,
    /// Reports that fell out of the driver's buffer before they were read.
    pub missed_reports: u64,
}

impl RunLog {
    pub fn new(scenario: &str, config: &Config, view: &Published) -> Self {
        RunLog {
            scenario: scenario.to_owned(),
            heatmap: heatmap::settings(config).map(|(res, dt)| Heatmap::new(res, dt)),
            deliveries: DeliveryLog::default(),
            landings: LandingLog::default(),
            geofences: GeofenceLog::default(),
            last_tick: None,
            // Only what happens from now on.
            last_report: view.reports.last().map_or(0, |r| r.seq),
            missed_reports: 0,
        }
    }

    /// Fold in a published view. Views of a tick already seen add nothing.
    pub fn observe(&mut self, view: &Published) {
        let tick = view.meta.tick;
        if self.last_tick != Some(tick) {
            self.last_tick = Some(tick);
            if let Some(map) = self.heatmap.as_mut() {
                map.observe(tick, &drones_in(&view.cells));
            }
        }
        let new = view.reports_since(self.last_report);
        if let Some(first) = new.first() {
            self.missed_reports += first.seq.saturating_sub(self.last_report + 1);
        }
        for report in new {
            self.deliveries.add(report);
            self.landings.add(report);
            self.geofences.add(report);
            self.last_report = report.seq;
        }
    }

    /// The last tick seen, to spot a reset.
    pub fn last_tick(&self) -> Option<u64> {
        self.last_tick
    }

    /// File name and contents of everything worth writing so far.
    pub fn files(&self) -> Vec<(&'static str, String)> {
        let mut out = Vec::new();
        if let Some(map) = self.heatmap.as_ref().filter(|m| m.drone_seconds() > 0.0) {
            out.push((
                "heatmap.geojson",
                map.to_geojson(&self.scenario).to_string(),
            ));
        }
        if !self.deliveries.is_empty() {
            out.push(("deliveries.csv", self.deliveries.to_csv()));
            out.push(("delivery-times.csv", self.deliveries.histogram_csv()));
            out.push((
                "delivery-times.svg",
                self.deliveries
                    .histogram_svg(&format!("{}: time from order to delivery", self.scenario)),
            ));
        }
        if !self.deliveries.rejected.is_empty() {
            out.push(("rejected-orders.csv", self.deliveries.rejected_csv()));
        }
        if !self.landings.is_empty() {
            out.push(("landings.csv", self.landings.to_csv()));
            out.push(("landing-battery.csv", self.landings.histogram_csv()));
            out.push((
                "landing-battery.svg",
                self.landings.histogram_svg(&format!(
                    "{}: battery left at the end of each flight",
                    self.scenario
                )),
            ));
        }
        if !self.geofences.is_empty() {
            out.push((
                "geofences.geojson",
                self.geofences.to_geojson(&self.scenario).to_string(),
            ));
            out.push(("breaches.csv", self.geofences.breaches_csv()));
            out.push(("reroutes.csv", self.geofences.reroutes_csv()));
        }
        if !out.is_empty() {
            let summary = self.deliveries.summary();
            out.push((
                "summary.json",
                serde_json::to_string_pretty(&serde_json::json!({
                    "scenario": self.scenario,
                    "last_tick": self.last_tick,
                    "flight_drone_seconds": self.heatmap.as_ref().map(Heatmap::drone_seconds),
                    "deliveries": self.deliveries.len(),
                    "rejected_orders": self.deliveries.rejected.len(),
                    "order_to_delivery_s": summary.map(|s| serde_json::json!({
                        "mean": s.mean_s, "median": s.median_s, "p90": s.p90_s, "max": s.max_s,
                        "mean_wait_for_drone": s.mean_wait_s, "mean_flight": s.mean_flight_s,
                    })),
                    "flights": self.landings.len(),
                    "crashes": self.landings.crashes(),
                    "geofences": self.geofences.fences.len(),
                    "breaches": self.geofences.breaches.len(),
                    "reroutes": self.geofences.reroutes.len(),
                    "missed_reports": self.missed_reports,
                }))
                .unwrap_or_default(),
            ));
        }
        out
    }
}
