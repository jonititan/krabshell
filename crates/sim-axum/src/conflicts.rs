//! Conflict assessment of the running world: where its aircraft could meet
//! in the next minute, and what each should do about it.
//!
//! The drones and intruders in the air are read out of the published view
//! and handed to `sim-conflict`, with the run's wind. The assessment runs
//! off the tick thread, at most once per published tick, and only while
//! someone is looking (the API or the Conflicts screen asked in the last
//! half minute), so a run nobody watches pays nothing for it. Times are
//! simulated seconds.

use std::collections::BTreeMap;
use std::sync::atomic::{AtomicI64, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};

use arc_swap::ArcSwapOption;
use serde::Serialize;
use sim_conflict::{
    assess, hotspots, AircraftState, Calm, Config, FieldWind, Fleet, Manoeuvre, Multirotor,
    Performance, Prediction, Targets, WindSource,
};
use sim_core::config::Param;
use sim_core::{GeoCell, WindField};
use sim_engine::{DriverHandle, Published};
use tokio::sync::{watch, Mutex};

use crate::weather::WeatherService;

/// Most cells a report lists.
const MAX_CELLS: usize = 500;

/// How long after the last look the background keeps assessing, seconds.
const WATCHED_S: i64 = 30;

#[derive(Debug, Clone, Serialize)]
pub struct Contributor {
    pub id: u64,
    pub callsign: String,
    pub probability: f64,
}

/// One A5 cell at its worst moment.
#[derive(Debug, Clone, Serialize)]
pub struct CellView {
    pub cell: GeoCell,
    /// Its outline, `[lon, lat]` pairs.
    pub boundary: Vec<[f64; 2]>,
    pub layer: i32,
    pub alt_from_m: f64,
    pub alt_to_m: f64,
    /// Seconds ahead of the assessed tick, simulated.
    pub t_s: f64,
    /// Summed probability: the aircraft expected there. A conflict at or
    /// over the threshold.
    pub summed: f64,
    /// The chance two or more are there together.
    pub collision: f64,
    pub conflict: bool,
    pub aircraft: Vec<Contributor>,
}

/// A request for one aircraft to resolve a conflict.
#[derive(Debug, Clone, Serialize)]
pub struct AdvisoryView {
    pub id: u64,
    pub callsign: String,
    pub manoeuvre: Manoeuvre,
    pub targets: Targets,
    pub against: Vec<Contributor>,
    pub first_conflict_s: f64,
    pub risk_before: f64,
    pub risk_after: f64,
    /// In no conflict once every advisory is followed.
    pub clear: bool,
}

/// Two aircraft still in conflict once every advisory is followed.
#[derive(Debug, Clone, Serialize)]
pub struct UnresolvedView {
    pub a: Contributor,
    pub b: Contributor,
    pub first_s: f64,
    pub peak_total: f64,
}

#[derive(Debug, Clone, Serialize)]
pub struct ConflictReport {
    /// The tick assessed.
    pub tick: u64,
    pub sim_utc: Option<String>,
    /// Wall-clock milliseconds the assessment took.
    pub took_ms: f64,
    /// Aircraft in the air that were assessed.
    pub aircraft: usize,
    pub resolution: i32,
    pub threshold: f64,
    pub step_s: f64,
    pub horizon_s: f64,
    pub layer_m: f64,
    pub risk_before: f64,
    pub risk_after: f64,
    pub advisories: Vec<AdvisoryView>,
    pub unresolved: Vec<UnresolvedView>,
    /// Cells two or more aircraft could share, most aircraft expected first.
    pub cells: Vec<CellView>,
}

pub struct ConflictService {
    latest: ArcSwapOption<ConflictReport>,
    busy: Mutex<()>,
    /// Unix seconds of the last look.
    looked: AtomicI64,
    started: Instant,
}

impl Default for ConflictService {
    fn default() -> Self {
        ConflictService {
            latest: ArcSwapOption::empty(),
            busy: Mutex::new(()),
            looked: AtomicI64::new(i64::MIN),
            started: Instant::now(),
        }
    }
}

impl ConflictService {
    fn now_s(&self) -> i64 {
        self.started.elapsed().as_secs() as i64
    }

    /// The assessment of the current tick, worked out if it is not already.
    pub async fn report(
        &self,
        driver: &DriverHandle,
        weather: &WeatherService,
    ) -> anyhow::Result<Arc<ConflictReport>> {
        self.looked.store(self.now_s(), Ordering::Relaxed);
        let tick = driver.view().meta.tick;
        if let Some(r) = self.latest.load_full().filter(|r| r.tick == tick) {
            return Ok(r);
        }
        let _busy = self.busy.lock().await;
        let view = driver.view();
        if let Some(r) = self.latest.load_full().filter(|r| r.tick == view.meta.tick) {
            return Ok(r);
        }
        let field = weather.current().and_then(|w| w.field.clone());
        let report =
            tokio::task::spawn_blocking(move || compute(&view, field.as_deref())).await??;
        let report = Arc::new(report);
        self.latest.store(Some(Arc::clone(&report)));
        Ok(report)
    }

    /// The last report, whatever tick it is of.
    pub fn latest(&self) -> Option<Arc<ConflictReport>> {
        self.latest.load_full()
    }

    /// Keep the report fresh while someone is looking. Runs until `stop`.
    pub async fn run(
        self: Arc<Self>,
        driver: DriverHandle,
        weather: Arc<WeatherService>,
        mut stop: watch::Receiver<bool>,
    ) {
        let mut ticker = tokio::time::interval(Duration::from_secs(2));
        ticker.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Delay);
        loop {
            tokio::select! {
                _ = ticker.tick() => {}
                _ = stop.wait_for(|s| *s) => return,
            }
            let looked = self.looked.load(Ordering::Relaxed);
            if self.now_s().saturating_sub(looked) > WATCHED_S {
                continue;
            }
            let before = self.looked.load(Ordering::Relaxed);
            if let Err(err) = self.report(&driver, &weather).await {
                tracing::warn!(%err, "conflict assessment");
            }
            // Working it out is not a look.
            self.looked.store(before, Ordering::Relaxed);
        }
    }
}

/// A drone or intruder in the air, as the view publishes it.
struct Flying {
    state: AircraftState,
    callsign: String,
}

fn param(view: &Published, key: &str, default: f64) -> f64 {
    view.config
        .model
        .get(key)
        .and_then(Param::as_f64)
        .filter(|v| v.is_finite() && *v > 0.0)
        .unwrap_or(default)
}

/// The performance the pack's drones and intruders fly with.
fn fleet(view: &Published) -> Fleet {
    let quad = |name: &str, cruise: f64| {
        let Performance::Multirotor(base) = Performance::generic_quadcopter() else {
            unreachable!()
        };
        Performance::Multirotor(Multirotor {
            name: name.into(),
            cruise_ms: cruise,
            max_speed_ms: (cruise * 1.5).max(base.max_speed_ms),
            max_climb_ms: param(view, "climb_ms", base.max_climb_ms),
            advisory_vs_ms: param(view, "climb_ms", base.advisory_vs_ms),
            ..base
        })
    };
    Fleet::from([
        (
            "drone".to_owned(),
            quad("krabshell drone", param(view, "cruise_ms", 10.0)),
        ),
        (
            "intruder".to_owned(),
            quad("intruder", param(view, "intruder_speed_ms", 12.0)),
        ),
    ])
}

/// The aircraft in the air: drones not on the ground or charging, and every
/// intruder.
fn flying(view: &Published) -> Vec<Flying> {
    let int = |v: &serde_json::Value, k: &str| v.get(k).and_then(serde_json::Value::as_i64);
    let mut out = Vec::new();
    for bag in &view.cells {
        for object in &bag.objects {
            let s = &object.state;
            let kind = object.kind.as_str();
            if kind != "drone" && kind != "intruder" {
                continue;
            }
            if kind == "drone" {
                let mode = s
                    .pointer("/drone/mode")
                    .and_then(|m| m.as_str())
                    .unwrap_or("");
                let activity = s.get("activity").and_then(|a| a.as_str()).unwrap_or("");
                if matches!(mode, "disarmed" | "idle" | "")
                    || matches!(activity, "charging" | "idle")
                {
                    continue;
                }
            }
            let (Some(lat), Some(lon)) = (int(s, "lat_e7"), int(s, "lon_e7")) else {
                continue;
            };
            let speed = int(s, "air_cms").or(int(s, "spd_cms")).unwrap_or(0) as f64 * 0.01;
            let heading = int(s, "yaw_cdeg").or(int(s, "hdg_cdeg")).unwrap_or(0) as f64 * 0.01;
            out.push(Flying {
                callsign: s
                    .get("callsign")
                    .and_then(|c| c.as_str())
                    .map_or_else(|| format!("#{}", object.id), str::to_owned),
                state: AircraftState {
                    id: object.id,
                    lat: lat as f64 * 1e-7,
                    lon: lon as f64 * 1e-7,
                    alt_m: int(s, "alt_mm").unwrap_or(0) as f64 * 1e-3,
                    tas_ms: speed,
                    heading_deg: heading,
                    vs_ms: int(s, "vs_cms").unwrap_or(0) as f64 * 0.01,
                    turn_rate_dps: 0.0,
                    mass_kg: None,
                    performance: kind.to_owned(),
                },
            });
        }
    }
    out
}

/// The configuration for a world: the drones preset, on its cells.
pub fn config_for(view: &Published) -> Config {
    Config {
        resolution: view.meta.resolution.clamp(0, sim_core::MAX_RESOLUTION),
        ..Config::drones()
    }
}

/// Assess one published view.
pub fn compute(view: &Published, field: Option<&WindField>) -> anyhow::Result<ConflictReport> {
    let started = Instant::now();
    let cfg = config_for(view);
    let aircraft = flying(view);
    let callsigns: BTreeMap<u64, String> = aircraft
        .iter()
        .map(|a| (a.state.id, a.callsign.clone()))
        .collect();
    let who = |id: u64, probability: f64| Contributor {
        id,
        callsign: callsigns
            .get(&id)
            .cloned()
            .unwrap_or_else(|| format!("#{id}")),
        probability,
    };
    let states: Vec<AircraftState> = aircraft.into_iter().map(|a| a.state).collect();
    let utc = view.overlays["weather"]["utc"].as_f64();
    let wind: Box<dyn WindSource + '_> = match (field, utc) {
        (Some(field), Some(utc_now)) => Box::new(FieldWind {
            field,
            utc_now,
            cell_resolution: view.meta.resolution,
        }),
        _ => Box::new(Calm),
    };
    let a = assess(&states, &fleet(view), wind.as_ref(), &cfg)?;
    // Where they could meet with nobody manoeuvring: what the advisories are
    // for.
    let before_refs: Vec<&Prediction> = a.unadvised.iter().collect();
    let cells = hotspots(&before_refs, &cfg)
        .into_iter()
        .take(MAX_CELLS)
        .map(|h| CellView {
            boundary: h
                .cell
                .boundary()
                .unwrap_or_default()
                .into_iter()
                .map(|(lon, lat)| [lon, lat])
                .collect(),
            cell: h.cell,
            layer: h.layer,
            alt_from_m: h.alt_from_m,
            alt_to_m: h.alt_to_m,
            t_s: h.t_s,
            summed: h.summed,
            collision: h.collision,
            conflict: h.conflict,
            aircraft: h.aircraft.iter().map(|(id, p)| who(*id, *p)).collect(),
        })
        .collect();
    Ok(ConflictReport {
        tick: view.meta.tick,
        sim_utc: utc.map(|u| sim_weather::format_utc(u as i64)),
        took_ms: started.elapsed().as_secs_f64() * 1e3,
        aircraft: states.len(),
        resolution: cfg.resolution,
        threshold: cfg.threshold,
        step_s: cfg.step_s,
        horizon_s: cfg.step_s * cfg.steps as f64,
        layer_m: cfg.layers.bands[0].depth_m,
        risk_before: a.risk_before,
        risk_after: a.risk_after,
        advisories: a
            .advisories
            .iter()
            .map(|adv| AdvisoryView {
                id: adv.aircraft,
                callsign: who(adv.aircraft, 0.0).callsign,
                manoeuvre: adv.manoeuvre,
                targets: adv.targets,
                against: adv.against.iter().map(|id| who(*id, 0.0)).collect(),
                first_conflict_s: adv.first_conflict_s,
                risk_before: adv.risk_before,
                risk_after: adv.risk_after,
                clear: adv.clear,
            })
            .collect(),
        unresolved: sim_conflict::encounters(&a.residual)
            .into_iter()
            .map(|e| UnresolvedView {
                a: who(e.a, 0.0),
                b: who(e.b, 0.0),
                first_s: e.first_s,
                peak_total: e.peak_total,
            })
            .collect(),
        cells,
    })
}
