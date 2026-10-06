//! How much battery each flight came home with, from `Landing` and `Crash`
//! reports.
//!
//! Every flight is one sample: the battery percentage the drone touched down
//! with, or -1 if it ran flat in the air and crashed. [`LandingLog`] writes
//! every sample as CSV, a histogram of them (crashes in a bin of their own,
//! then 5 % bins up to 100 %) as CSV, and the same histogram as an SVG chart.
//! Nothing here touches the simulation.

use sim_core::{Report, ReportKind};

use super::chart::{self, csv_field, Bar, Chart};

/// The battery percentage a crash counts as.
pub const CRASHED: f64 = -1.0;

/// Width of a histogram bin, percent.
const BIN_PCT: f64 = 5.0;
const BINS: usize = 20;

/// The end of one flight.
#[derive(Debug, Clone, PartialEq)]
pub struct Landing {
    pub tick: u64,
    pub drone: u64,
    pub sysid: u64,
    pub callsign: String,
    pub depot: String,
    /// Percent of a full charge, or [`CRASHED`].
    pub battery_pct: f64,
    pub lat: f64,
    pub lon: f64,
    /// For a crash, what took over: its callsign, and whether it was a spare.
    pub replaced_by: Option<(String, bool)>,
}

impl Landing {
    pub fn crashed(&self) -> bool {
        self.battery_pct < 0.0
    }
}

#[derive(Debug, Clone, Default)]
pub struct LandingLog {
    pub landings: Vec<Landing>,
}

impl LandingLog {
    /// Keep a report if it ends a flight. True if it did.
    pub fn add(&mut self, report: &Report) -> bool {
        if !matches!(report.kind, ReportKind::Landing | ReportKind::Crash) {
            return false;
        }
        let d = &report.detail;
        let text = |k: &str| {
            d.get(k)
                .and_then(|v| v.as_str())
                .unwrap_or_default()
                .to_owned()
        };
        let battery_pct = match report.kind {
            ReportKind::Crash => CRASHED,
            _ => d
                .get("battery_pct")
                .and_then(serde_json::Value::as_f64)
                .unwrap_or(0.0),
        };
        let replaced_by = d.get("replaced_by").and_then(|r| {
            Some((
                r.get("callsign")?.as_str()?.to_owned(),
                r.get("spare")?.as_bool()?,
            ))
        });
        self.landings.push(Landing {
            tick: report.tick,
            drone: report.observer.unwrap_or(0),
            sysid: d
                .get("sysid")
                .and_then(serde_json::Value::as_u64)
                .unwrap_or(0),
            callsign: text("callsign"),
            depot: text("depot"),
            battery_pct,
            lat: report.lat(),
            lon: report.lon(),
            replaced_by,
        });
        true
    }

    pub fn len(&self) -> usize {
        self.landings.len()
    }

    pub fn is_empty(&self) -> bool {
        self.landings.is_empty()
    }

    pub fn crashes(&self) -> usize {
        self.landings.iter().filter(|l| l.crashed()).count()
    }

    /// Every flight, one row each, in the order they ended.
    pub fn to_csv(&self) -> String {
        let mut out = String::from(
            "tick,drone,sysid,callsign,depot,outcome,battery_pct,lat,lon,replaced_by,replacement\n",
        );
        for l in &self.landings {
            let (by, how) = match &l.replaced_by {
                Some((callsign, spare)) => {
                    (csv_field(callsign), if *spare { "spare" } else { "new" })
                }
                None => (String::new(), ""),
            };
            out.push_str(&format!(
                "{},{},{},{},{},{},{},{:.7},{:.7},{by},{how}\n",
                l.tick,
                l.drone,
                l.sysid,
                csv_field(&l.callsign),
                csv_field(&l.depot),
                if l.crashed() { "crashed" } else { "landed" },
                l.battery_pct,
                l.lat,
                l.lon,
            ));
        }
        out
    }

    /// Crashes, then flights landed with 0–5 %, 5–10 % … 95–100 % left.
    pub fn histogram(&self) -> (usize, [usize; BINS]) {
        let mut bins = [0; BINS];
        for l in self.landings.iter().filter(|l| !l.crashed()) {
            bins[((l.battery_pct / BIN_PCT) as usize).min(BINS - 1)] += 1;
        }
        (self.crashes(), bins)
    }

    /// The histogram as CSV. Crashes are the row from -1 to -1.
    pub fn histogram_csv(&self) -> String {
        let (crashes, bins) = self.histogram();
        let mut out = format!("from_pct,to_pct,flights\n-1,-1,{crashes}\n");
        for (i, n) in bins.iter().enumerate() {
            out.push_str(&format!(
                "{},{},{n}\n",
                i as f64 * BIN_PCT,
                (i + 1) as f64 * BIN_PCT
            ));
        }
        out
    }

    /// The histogram as a standalone SVG bar chart: crashes in red on the
    /// left, then battery percentage at touchdown.
    pub fn histogram_svg(&self, title: &str) -> String {
        let (crashes, bins) = self.histogram();
        let mut landed: Vec<f64> = self
            .landings
            .iter()
            .filter(|l| !l.crashed())
            .map(|l| l.battery_pct)
            .collect();
        landed.sort_by(f64::total_cmp);
        let mut notes = vec![format!(
            "{} flights · {} landed · {} crashed (battery -1)",
            self.len(),
            landed.len(),
            crashes
        )];
        if !landed.is_empty() {
            let at = |q: f64| landed[((landed.len() - 1) as f64 * q).round() as usize];
            notes.push(format!(
                "battery at touchdown: mean {:.1} % · median {:.1} % · lowest {:.1} %",
                landed.iter().sum::<f64>() / landed.len() as f64,
                at(0.5),
                landed[0]
            ));
        }
        let mut bars = vec![Bar {
            count: crashes,
            fill: chart::ALERT,
            tooltip: format!("crashed: {crashes}"),
        }];
        bars.extend(bins.iter().enumerate().map(|(i, n)| Bar {
            count: *n,
            fill: chart::BAR,
            tooltip: format!("{}–{} %: {n}", i as f64 * BIN_PCT, (i + 1) as f64 * BIN_PCT),
        }));
        let mut ticks = vec![(0.5, "-1".to_owned())];
        ticks.extend(
            (0..=BINS)
                .step_by(2)
                .map(|i| (1.0 + i as f64, format!("{}", i as f64 * BIN_PCT))),
        );
        Chart {
            title,
            notes,
            bars,
            ticks,
            x_label: "battery % at touchdown (crashes count as -1)",
            y_label: "flights",
        }
        .svg()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn report(kind: ReportKind, battery_pct: f64) -> Report {
        Report {
            seq: 0,
            tick: 10,
            kind,
            uid: String::new(),
            observer: Some(3),
            target: None,
            target_kind: String::new(),
            lat_e7: 549_000_000,
            lon_e7: 239_000_000,
            alt_mm: 0,
            detail: serde_json::json!({
                "sysid": 3, "callsign": "MEGA-3", "depot": "MEGA", "battery_pct": battery_pct,
                "replaced_by": { "id": 50, "sysid": 43, "callsign": "MEGA-SPARE", "spare": true },
            }),
        }
    }

    #[test]
    fn landings_bin_by_battery_and_crashes_count_as_minus_one() {
        let mut log = LandingLog::default();
        for pct in [0.4, 4.99, 5.0, 62.5, 100.0] {
            assert!(log.add(&report(ReportKind::Landing, pct)));
        }
        assert!(log.add(&report(ReportKind::Crash, 0.0)));
        assert!(!log.add(&report(ReportKind::Delivery, 50.0)));

        let (crashes, bins) = log.histogram();
        assert_eq!(crashes, 1);
        assert_eq!(bins[0], 2);
        assert_eq!(bins[1], 1);
        assert_eq!(bins[12], 1);
        assert_eq!(bins[19], 1, "a full battery is in the top bin");
        assert_eq!(log.landings[5].battery_pct, CRASHED);

        let csv = log.histogram_csv();
        assert!(csv.starts_with("from_pct,to_pct,flights\n-1,-1,1\n0,5,2\n"));
        assert_eq!(csv.lines().count(), 22);
        let rows = log.to_csv();
        assert!(rows.contains(",crashed,-1,"));
        assert!(rows.trim_end().ends_with("MEGA-SPARE,spare"));
        let svg = log.histogram_svg("Kaunas");
        assert!(svg.starts_with("<svg") && svg.contains("6 flights · 5 landed · 1 crashed"));
    }
}
