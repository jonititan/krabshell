//! Delivered orders, and how long they took, from `Delivery` reports.
//!
//! [`DeliveryLog`] collects the reports the dispatcher emits and writes them
//! out three ways: every delivery as CSV, a histogram of time from order to
//! delivery as CSV, and the same histogram as a self-contained SVG chart.
//! Nothing here touches the simulation.

use sim_core::{Report, ReportKind};

use super::chart::{self, csv_field, Bar, Chart};

/// One delivered order.
#[derive(Debug, Clone, PartialEq)]
pub struct Delivery {
    pub order: u64,
    pub depot: String,
    pub callsign: String,
    pub address: String,
    pub lat: f64,
    pub lon: f64,
    pub ordered_tick: u64,
    pub delivered_tick: u64,
    /// Order to dispatch: waiting in the queue for a ready drone.
    pub wait_s: f64,
    /// Dispatch to the parcel touching down.
    pub flight_s: f64,
    /// Order to delivery.
    pub total_s: f64,
}

/// An order turned away when it was placed.
#[derive(Debug, Clone, PartialEq)]
pub struct Rejected {
    pub order: u64,
    pub depot: String,
    pub address: String,
    pub lat: f64,
    pub lon: f64,
    pub ordered_tick: u64,
    pub fence: Option<u64>,
    pub fence_name: String,
    /// When it was turned away: as it was `placed`, while `queued`, or when
    /// its drone `returned` without delivering it.
    pub stage: String,
    pub reason: String,
}

#[derive(Debug, Clone, Default)]
pub struct DeliveryLog {
    pub deliveries: Vec<Delivery>,
    pub rejected: Vec<Rejected>,
}

/// Bin widths a histogram picks from, seconds: whole, readable minutes.
const NICE_BINS_S: [f64; 12] = [
    60.0, 120.0, 300.0, 600.0, 900.0, 1200.0, 1800.0, 3600.0, 7200.0, 10800.0, 21600.0, 43200.0,
];
const MAX_BINS: f64 = 30.0;

impl DeliveryLog {
    /// Keep a report if it is a delivery. True if it was.
    pub fn add(&mut self, report: &Report) -> bool {
        let d = &report.detail;
        if report.kind == ReportKind::OrderRejected {
            let text = |k: &str| {
                d.get(k)
                    .and_then(|v| v.as_str())
                    .unwrap_or_default()
                    .to_owned()
            };
            let int = |k: &str| d.get(k).and_then(serde_json::Value::as_u64);
            self.rejected.push(Rejected {
                order: int("order").unwrap_or(0),
                depot: text("depot"),
                address: text("label"),
                lat: report.lat(),
                lon: report.lon(),
                ordered_tick: int("ordered_tick").unwrap_or(report.tick),
                fence: int("fence"),
                fence_name: text("fence_name"),
                stage: text("stage"),
                reason: text("reason"),
            });
            return true;
        }
        if report.kind != ReportKind::Delivery {
            return false;
        }
        let text = |k: &str| {
            d.get(k)
                .and_then(|v| v.as_str())
                .unwrap_or_default()
                .to_owned()
        };
        let int = |k: &str| d.get(k).and_then(serde_json::Value::as_u64).unwrap_or(0);
        let num = |k: &str| d.get(k).and_then(serde_json::Value::as_f64).unwrap_or(0.0);
        self.deliveries.push(Delivery {
            order: int("order"),
            depot: text("depot"),
            callsign: text("callsign"),
            address: text("label"),
            lat: report.lat(),
            lon: report.lon(),
            ordered_tick: int("ordered_tick"),
            delivered_tick: int("delivered_tick"),
            wait_s: num("wait_s"),
            flight_s: num("flight_s"),
            total_s: num("total_s"),
        });
        true
    }

    pub fn len(&self) -> usize {
        self.deliveries.len()
    }

    pub fn is_empty(&self) -> bool {
        self.deliveries.is_empty()
    }

    /// Every delivery, one row each.
    pub fn to_csv(&self) -> String {
        let mut out = String::from(
            "order,depot,drone,address,lat,lon,ordered_tick,delivered_tick,wait_s,flight_s,total_s\n",
        );
        for d in &self.deliveries {
            out.push_str(&format!(
                "{},{},{},{},{:.7},{:.7},{},{},{:.1},{:.1},{:.1}\n",
                d.order,
                csv_field(&d.depot),
                csv_field(&d.callsign),
                csv_field(&d.address),
                d.lat,
                d.lon,
                d.ordered_tick,
                d.delivered_tick,
                d.wait_s,
                d.flight_s,
                d.total_s
            ));
        }
        out
    }

    /// Every order turned away, one row each, with why.
    pub fn rejected_csv(&self) -> String {
        let mut out =
            String::from("order,depot,address,lat,lon,ordered_tick,fence,fence_name,stage,reason\n");
        for r in &self.rejected {
            out.push_str(&format!(
                "{},{},{},{:.7},{:.7},{},{},{},{},{}\n",
                r.order,
                csv_field(&r.depot),
                csv_field(&r.address),
                r.lat,
                r.lon,
                r.ordered_tick,
                r.fence.map(|f| f.to_string()).unwrap_or_default(),
                csv_field(&r.fence_name),
                csv_field(&r.stage),
                csv_field(&r.reason),
            ));
        }
        out
    }

    /// Order-to-delivery times in equal bins of a readable width: `(from_s,
    /// to_s, count)`, from zero to the slowest delivery.
    pub fn histogram(&self) -> Vec<(f64, f64, usize)> {
        let slowest = self
            .deliveries
            .iter()
            .map(|d| d.total_s)
            .fold(0.0, f64::max);
        if self.deliveries.is_empty() {
            return Vec::new();
        }
        let width = NICE_BINS_S
            .iter()
            .copied()
            .find(|w| slowest / w < MAX_BINS)
            .unwrap_or(NICE_BINS_S[NICE_BINS_S.len() - 1]);
        let bins = ((slowest / width).floor() as usize) + 1;
        let mut counts = vec![0usize; bins];
        for d in &self.deliveries {
            counts[((d.total_s / width) as usize).min(bins - 1)] += 1;
        }
        counts
            .into_iter()
            .enumerate()
            .map(|(i, n)| (i as f64 * width, (i + 1) as f64 * width, n))
            .collect()
    }

    pub fn histogram_csv(&self) -> String {
        let mut out = String::from("from_min,to_min,deliveries\n");
        for (from, to, n) in self.histogram() {
            out.push_str(&format!("{},{},{n}\n", from / 60.0, to / 60.0));
        }
        out
    }

    /// Count, mean, median, 90th percentile and maximum order-to-delivery
    /// time, seconds.
    pub fn summary(&self) -> Option<Summary> {
        if self.deliveries.is_empty() {
            return None;
        }
        let mut t: Vec<f64> = self.deliveries.iter().map(|d| d.total_s).collect();
        t.sort_by(f64::total_cmp);
        let at = |q: f64| t[((t.len() - 1) as f64 * q).round() as usize];
        let mean_of = |f: fn(&Delivery) -> f64| {
            self.deliveries.iter().map(f).sum::<f64>() / self.deliveries.len() as f64
        };
        Some(Summary {
            count: t.len(),
            mean_s: mean_of(|d| d.total_s),
            median_s: at(0.5),
            p90_s: at(0.9),
            max_s: t[t.len() - 1],
            mean_wait_s: mean_of(|d| d.wait_s),
            mean_flight_s: mean_of(|d| d.flight_s),
        })
    }

    /// The histogram as a standalone SVG bar chart, minutes along the bottom.
    pub fn histogram_svg(&self, title: &str) -> String {
        let bins = self.histogram();
        let notes = self
            .summary()
            .map(|s| {
                vec![
                    format!(
                        "{} deliveries · mean {} · median {} · 90% within {} · slowest {}",
                        s.count,
                        minutes(s.mean_s),
                        minutes(s.median_s),
                        minutes(s.p90_s),
                        minutes(s.max_s)
                    ),
                    format!(
                        "of which, on average: waiting for a drone {} · flying to the address and winching {}",
                        minutes(s.mean_wait_s),
                        minutes(s.mean_flight_s)
                    ),
                ]
            })
            .unwrap_or_default();
        let label_every = (bins.len() as f64 / 10.0).ceil().max(1.0) as usize;
        let mut ticks: Vec<(f64, String)> = bins
            .iter()
            .enumerate()
            .filter(|(i, _)| i % label_every == 0)
            .map(|(i, (from, _, _))| (i as f64, format!("{}", from / 60.0)))
            .collect();
        if let Some(last) = bins.last() {
            ticks.push((bins.len() as f64, format!("{}", last.1 / 60.0)));
        }
        Chart {
            title,
            notes,
            bars: bins
                .iter()
                .map(|(from, _, n)| Bar {
                    count: *n,
                    fill: chart::BAR,
                    tooltip: format!("{}–{} min: {n}", from / 60.0, (from + bins[0].1) / 60.0),
                })
                .collect(),
            ticks,
            x_label: "minutes from order to delivery",
            y_label: "deliveries",
        }
        .svg()
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Summary {
    pub count: usize,
    pub mean_s: f64,
    pub median_s: f64,
    pub p90_s: f64,
    pub max_s: f64,
    pub mean_wait_s: f64,
    pub mean_flight_s: f64,
}

fn minutes(s: f64) -> String {
    format!("{:.1} min", s / 60.0)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn report(order: u64, total_s: f64) -> Report {
        Report {
            seq: 0,
            tick: 0,
            kind: ReportKind::Delivery,
            uid: format!("order-{order}"),
            observer: Some(1),
            target: None,
            target_kind: String::new(),
            lat_e7: 549_000_000,
            lon_e7: 239_000_000,
            alt_mm: 0,
            detail: serde_json::json!({
                "order": order, "depot": "AKRO", "callsign": "AKRO-1",
                "label": "Laisvės al. 1, Kaunas", "total_s": total_s,
                "wait_s": total_s / 2.0, "flight_s": total_s / 2.0,
            }),
        }
    }

    #[test]
    fn deliveries_bin_into_readable_minutes_and_summarise() {
        let mut log = DeliveryLog::default();
        for (i, t) in [100.0, 200.0, 700.0, 1_000.0, 1_900.0]
            .into_iter()
            .enumerate()
        {
            assert!(log.add(&report(i as u64 + 1, t)));
        }
        let mut detection = report(9, 1.0);
        detection.kind = ReportKind::Detection;
        assert!(!log.add(&detection));

        // Slowest 1900 s: 2-minute bins would be 16 bins, under the cap of 30.
        let bins = log.histogram();
        assert_eq!(bins[0], (0.0, 120.0, 1));
        assert_eq!(bins.len(), 16);
        assert_eq!(bins.iter().map(|b| b.2).sum::<usize>(), 5);

        let s = log.summary().unwrap();
        assert_eq!(s.count, 5);
        assert_eq!(s.median_s, 700.0);
        assert_eq!(s.max_s, 1_900.0);

        // A comma in an address is quoted, not a new column.
        assert!(log.to_csv().contains("\"Laisvės al. 1, Kaunas\""));
        let svg = log.histogram_svg("Kaunas");
        assert!(svg.starts_with("<svg") && svg.contains("5 deliveries"));
    }
}
