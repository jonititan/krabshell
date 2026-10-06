//! Geofences, breaches of them and the reroutes round them, from `Geofence`,
//! `Breach` and `Reroute` reports.
//!
//! [`GeofenceLog`] writes every fence that opened, with when it opened and
//! closed, and every breach as GeoJSON (styled for geojson.io and other
//! simplestyle viewers), every breach as CSV, and every reroute the operator
//! sent as CSV. Nothing here touches the simulation.

use std::collections::BTreeMap;

use sim_core::{Report, ReportKind};

use super::airspace::Fence;
use super::chart::csv_field;

/// A fence, and when it was open.
#[derive(Debug, Clone, PartialEq)]
pub struct Opened {
    pub fence: Fence,
    pub closed_tick: Option<u64>,
    pub close_reason: String,
}

/// A drone found inside a fence.
#[derive(Debug, Clone, PartialEq)]
pub struct Breach {
    pub tick: u64,
    pub drone: u64,
    pub sysid: u64,
    pub callsign: String,
    pub fence: u64,
    pub how: String,
    pub flown_by: String,
    pub lat: f64,
    pub lon: f64,
}

/// MAVLink messages the operator sent one drone to take it round a fence.
#[derive(Debug, Clone, PartialEq)]
pub struct Reroute {
    pub tick: u64,
    pub drone: u64,
    pub callsign: String,
    pub reason: String,
    pub detours: u64,
    pub messages: Vec<String>,
}

#[derive(Debug, Clone, Default)]
pub struct GeofenceLog {
    pub fences: BTreeMap<u32, Opened>,
    pub breaches: Vec<Breach>,
    pub reroutes: Vec<Reroute>,
}

impl GeofenceLog {
    /// Keep a report if it is about geofences. True if it was.
    pub fn add(&mut self, report: &Report) -> bool {
        let d = &report.detail;
        let text = |k: &str| {
            d.get(k)
                .and_then(|v| v.as_str())
                .unwrap_or_default()
                .to_owned()
        };
        let int = |k: &str| d.get(k).and_then(serde_json::Value::as_u64).unwrap_or(0);
        match report.kind {
            ReportKind::Geofence => {
                let Ok(fence) = serde_json::from_value::<Fence>(d.clone()) else {
                    return false;
                };
                if text("action") == "close" {
                    if let Some(open) = self.fences.get_mut(&fence.id) {
                        open.closed_tick = Some(report.tick);
                        open.close_reason = text("reason");
                    }
                } else {
                    self.fences.insert(
                        fence.id,
                        Opened {
                            fence,
                            closed_tick: None,
                            close_reason: String::new(),
                        },
                    );
                }
            }
            ReportKind::Breach => self.breaches.push(Breach {
                tick: report.tick,
                drone: report.observer.unwrap_or(0),
                sysid: int("sysid"),
                callsign: text("callsign"),
                fence: int("fence"),
                how: text("how"),
                flown_by: text("flown_by"),
                lat: report.lat(),
                lon: report.lon(),
            }),
            ReportKind::Reroute => self.reroutes.push(Reroute {
                tick: report.tick,
                drone: report.observer.unwrap_or(0),
                callsign: text("callsign"),
                reason: text("reason"),
                detours: int("detours"),
                messages: d
                    .get("messages")
                    .and_then(|m| m.as_array())
                    .map(|m| {
                        m.iter()
                            .filter_map(|s| s.as_str().map(str::to_owned))
                            .collect()
                    })
                    .unwrap_or_default(),
            }),
            _ => return false,
        }
        true
    }

    pub fn is_empty(&self) -> bool {
        self.fences.is_empty() && self.breaches.is_empty() && self.reroutes.is_empty()
    }

    /// Every fence as a polygon, and every breach as a point.
    pub fn to_geojson(&self, scenario_id: &str) -> serde_json::Value {
        let mut features: Vec<serde_json::Value> = self
            .fences
            .values()
            .map(|o| {
                let f = &o.fence;
                serde_json::json!({
                    "type": "Feature",
                    "geometry": f.geometry(),
                    "properties": {
                        "fence": f.id,
                        "source": f.source,
                        "opened_tick": f.opened_tick,
                        "closed_tick": o.closed_tick,
                        "closed": if o.close_reason.is_empty() { "still open" } else { o.close_reason.as_str() },
                        "breaches": self.breaches.iter().filter(|b| b.fence == u64::from(f.id)).count(),
                        "fill": "#c53030",
                        "fill-opacity": 0.2,
                        "stroke": "#c53030",
                        "stroke-width": 1.5,
                    },
                })
            })
            .collect();
        features.extend(self.breaches.iter().map(|b| {
            serde_json::json!({
                "type": "Feature",
                "geometry": { "type": "Point", "coordinates": [b.lon, b.lat] },
                "properties": {
                    "breach_of": b.fence,
                    "drone": b.callsign,
                    "tick": b.tick,
                    "how": b.how,
                    "flown_by": b.flown_by,
                    "marker-color": "#16181d",
                    "marker-symbol": "cross",
                },
            })
        }));
        serde_json::json!({
            "type": "FeatureCollection",
            "properties": {
                "scenario": scenario_id,
                "what": "geofences opened during the run, and breaches of them",
                "fences": self.fences.len(),
                "breaches": self.breaches.len(),
                "reroutes": self.reroutes.len(),
            },
            "features": features,
        })
    }

    pub fn breaches_csv(&self) -> String {
        let mut out = String::from("tick,drone,sysid,callsign,fence,how,flown_by,lat,lon\n");
        for b in &self.breaches {
            out.push_str(&format!(
                "{},{},{},{},{},{},{},{:.7},{:.7}\n",
                b.tick,
                b.drone,
                b.sysid,
                csv_field(&b.callsign),
                b.fence,
                csv_field(&b.how),
                csv_field(&b.flown_by),
                b.lat,
                b.lon
            ));
        }
        out
    }

    pub fn reroutes_csv(&self) -> String {
        let mut out = String::from("tick,drone,callsign,reason,detours,messages\n");
        for r in &self.reroutes {
            out.push_str(&format!(
                "{},{},{},{},{},{}\n",
                r.tick,
                r.drone,
                csv_field(&r.callsign),
                csv_field(&r.reason),
                r.detours,
                csv_field(&r.messages.join("; "))
            ));
        }
        out
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn report(kind: ReportKind, tick: u64, detail: serde_json::Value) -> Report {
        Report {
            seq: 0,
            tick,
            kind,
            uid: String::new(),
            observer: Some(4),
            target: None,
            target_kind: String::new(),
            lat_e7: 549_000_000,
            lon_e7: 239_000_000,
            alt_mm: 0,
            detail,
        }
    }

    #[test]
    fn fences_breaches_and_reroutes_are_logged() {
        let fence = serde_json::json!({
            "id": 3, "shape": "circle", "lat_e7": 549_000_000, "lon_e7": 239_000_000,
            "radius_m": 200.0, "source": "atm", "opened_tick": 10, "closes_tick": 500,
        });
        let mut log = GeofenceLog::default();
        let mut open = fence.clone();
        open["action"] = "open".into();
        assert!(log.add(&report(ReportKind::Geofence, 10, open)));
        assert!(log.add(&report(
            ReportKind::Breach,
            10,
            serde_json::json!({ "fence": 3, "callsign": "MEGA-1", "sysid": 1, "how": "fence opened round it", "flown_by": "operator" }),
        )));
        assert!(log.add(&report(
            ReportKind::Reroute,
            11,
            serde_json::json!({ "callsign": "MEGA-1", "reason": "inside a geofence", "detours": 0, "messages": ["COMMAND_INT DO_REPOSITION"] }),
        )));
        let mut close = fence;
        close["action"] = "close".into();
        close["reason"] = "expired".into();
        assert!(log.add(&report(ReportKind::Geofence, 500, close)));
        assert!(!log.add(&report(ReportKind::Landing, 1, serde_json::Value::Null)));

        assert_eq!(log.fences[&3].closed_tick, Some(500));
        let json = log.to_geojson("t");
        assert_eq!(json["features"].as_array().unwrap().len(), 2);
        assert_eq!(json["features"][0]["properties"]["breaches"], 1);
        assert_eq!(json["features"][0]["geometry"]["type"], "Polygon");
        assert!(log
            .breaches_csv()
            .contains(",MEGA-1,3,fence opened round it,operator,"));
        assert!(log.reroutes_csv().contains("COMMAND_INT DO_REPOSITION"));
    }
}
