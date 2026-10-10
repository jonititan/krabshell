//! The weather a run flies in: fetched before a scenario loads, kept fresh
//! while a live one runs, and mirrored here for the weather API and screen.
//!
//! The world holds the wind its drones fly in (it arrives in the pack's
//! fields, and later as `WEATHER_SET` intents) and checkpoints it. This
//! service keeps a copy of what it last gave the world, so the API can answer
//! questions about the whole field without asking the tick thread; the world
//! publishes its field's fingerprint, and the status says whether the two
//! agree.
//!
//! A fetch that fails refuses a `recorded` load: the point of recorded
//! weather is that the run is reproducible, and quietly flying it in still
//! air would not be. A `live` load goes ahead in still air, says why, and the
//! refresher keeps trying.

use std::sync::Arc;
use std::time::Duration;

use arc_swap::ArcSwapOption;
use serde::Serialize;
use sim_core::{GeoCell, Intent, IntentVerb, ScenarioBlob, WeatherMode, WindField};
use sim_engine::{DriverHandle, Submission};
use sim_weather::{format_utc, now_unix, spec_from, Fetcher, WeatherSpec};
use tokio::sync::{watch, Mutex};

/// `codes::WEATHER_SET` in the drones model.
pub const WEATHER_SET: u16 = 140;

/// Why a scenario could not be given its weather.
#[derive(Debug, thiserror::Error)]
pub enum PrepareError {
    /// The pack asks for weather in a way that cannot work.
    #[error("{0:#}")]
    Config(anyhow::Error),
    /// Recorded weather could not be fetched.
    #[error("{0:#}")]
    Fetch(anyhow::Error),
}

impl From<PrepareError> for crate::problem::ApiError {
    fn from(err: PrepareError) -> Self {
        let code = match err {
            PrepareError::Config(_) => sim_core::ErrorCode::BadConfig,
            PrepareError::Fetch(_) => sim_core::ErrorCode::WeatherUnavailable,
        };
        crate::problem::ApiError::new(code, err.to_string())
    }
}

/// Where the field came from.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum Origin {
    /// Fetched from Open-Meteo (or read from the cache of a fetch).
    OpenMeteo,
    /// The pack's own `world/weather.json`.
    Pack,
    /// A restored checkpoint's.
    Checkpoint,
}

/// What the service knows about the loaded scenario's weather.
#[derive(Debug, Clone)]
pub struct WeatherState {
    pub scenario_id: String,
    pub origin: Origin,
    /// The request, for fetched weather.
    pub spec: Option<WeatherSpec>,
    /// What the world was last given; `None` if a live fetch failed.
    pub field: Option<Arc<WindField>>,
    pub rev: u64,
    /// The last fetch came from the cache.
    pub cache_hit: bool,
    /// Open-Meteo locations the last fetch asked for.
    pub locations: usize,
    /// Wall-clock time of the last fetch attempt, Unix seconds.
    pub attempted_unix: i64,
    pub error: Option<String>,
}

impl WeatherState {
    pub fn mode(&self) -> Option<WeatherMode> {
        self.spec
            .as_ref()
            .map(|s| s.mode)
            .or_else(|| self.field.as_ref().map(|f| f.mode))
    }
}

pub struct WeatherService {
    fetcher: Option<Fetcher>,
    current: ArcSwapOption<WeatherState>,
    /// One fetch at a time.
    busy: Mutex<()>,
}

impl WeatherService {
    pub fn new(fetcher: Option<Fetcher>) -> WeatherService {
        WeatherService {
            fetcher,
            current: ArcSwapOption::empty(),
            busy: Mutex::new(()),
        }
    }

    /// Weather fetching switched off: packs with their own field still work.
    pub fn offline() -> WeatherService {
        WeatherService::new(None)
    }

    pub fn fetcher(&self) -> Option<&Fetcher> {
        self.fetcher.as_ref()
    }

    pub fn current(&self) -> Option<Arc<WeatherState>> {
        self.current.load_full()
    }

    /// Give a scenario its weather before the driver loads it: fetch what its
    /// params ask for (or take the pack's own field) and put it in the
    /// blob's fields, where the model reads it.
    pub async fn prepare(&self, blob: &mut ScenarioBlob) -> Result<(), PrepareError> {
        let _busy = self.busy.lock().await;
        let id = blob.manifest.id.clone();
        let now = now_unix();
        let Some(spec) = spec_from(blob, now).map_err(PrepareError::Config)? else {
            // No fetch asked for. A pack may carry its own field.
            let state = match blob.fields.get("weather") {
                Some(v) if !v.is_null() => {
                    let field = WindField::from_value(v).map_err(|e| {
                        PrepareError::Config(anyhow::anyhow!("world/weather.json: {e}"))
                    })?;
                    Some(Arc::new(WeatherState {
                        scenario_id: id,
                        origin: Origin::Pack,
                        spec: None,
                        rev: field.rev(),
                        field: Some(Arc::new(field)),
                        cache_hit: false,
                        locations: 0,
                        attempted_unix: now,
                        error: None,
                    }))
                }
                _ => None,
            };
            self.current.store(state);
            return Ok(());
        };

        let fetched = match &self.fetcher {
            Some(fetcher) => fetcher.fetch(&spec, now).await,
            None => Err(anyhow::anyhow!(
                "weather fetching is switched off (--no-weather)"
            )),
        };
        let mut state = WeatherState {
            scenario_id: id,
            origin: Origin::OpenMeteo,
            spec: Some(spec.clone()),
            field: None,
            rev: 0,
            cache_hit: false,
            locations: 0,
            attempted_unix: now,
            error: None,
        };
        match fetched {
            Ok(f) => {
                tracing::info!(
                    scenario = %state.scenario_id,
                    cells = f.field.cells.len(),
                    frames = f.field.frames,
                    cache_hit = f.cache_hit,
                    locations = f.locations,
                    "weather ready"
                );
                if !blob.fields.is_object() {
                    blob.fields = serde_json::json!({});
                }
                blob.fields["weather"] =
                    serde_json::to_value(&f.field).map_err(|e| PrepareError::Fetch(e.into()))?;
                state.rev = f.field.rev();
                state.cache_hit = f.cache_hit;
                state.locations = f.locations;
                state.field = Some(Arc::new(f.field));
            }
            Err(err) if spec.mode == WeatherMode::Recorded => {
                return Err(PrepareError::Fetch(
                    err.context("recorded weather could not be fetched"),
                ));
            }
            Err(err) => {
                tracing::warn!(%err, "live weather could not be fetched; flying in still air until it can");
                if blob.fields.is_object() {
                    blob.fields["weather"] = serde_json::Value::Null;
                }
                state.error = Some(format!("{err:#}"));
            }
        }
        self.current.store(Some(Arc::new(state)));
        Ok(())
    }

    /// Take a restored checkpoint's wind as what the world flies in.
    pub fn adopt(&self, scenario_id: &str, field: Option<WindField>) {
        let previous = self.current();
        let Some(field) = field else {
            self.current.store(None);
            return;
        };
        self.current.store(Some(Arc::new(WeatherState {
            scenario_id: scenario_id.to_owned(),
            origin: Origin::Checkpoint,
            spec: previous.and_then(|p| p.spec.clone()),
            rev: field.rev(),
            field: Some(Arc::new(field)),
            cache_hit: false,
            locations: 0,
            attempted_unix: now_unix(),
            error: None,
        })));
    }

    /// Fetch a live run's weather again and hand it to the world. The run's
    /// clock (its tick 0) stays where it was; the window starts now.
    pub async fn refresh(&self, driver: &DriverHandle) -> anyhow::Result<Arc<WeatherState>> {
        let _busy = self.busy.lock().await;
        let Some(current) = self.current() else {
            anyhow::bail!("this run has no weather");
        };
        let Some(spec) = current.spec.clone().filter(|s| s.mode == WeatherMode::Live) else {
            anyhow::bail!("only live weather is refreshed; recorded weather is fixed");
        };
        let Some(fetcher) = &self.fetcher else {
            anyhow::bail!("weather fetching is switched off (--no-weather)");
        };
        let now = now_unix();
        let window = WeatherSpec {
            start_unix: now - sim_weather::HOUR_S,
            ..spec.clone()
        };
        let mut state = WeatherState {
            attempted_unix: now,
            ..(*current).clone()
        };
        match fetcher.fetch(&window, now).await {
            Ok(f) => {
                let mut field = f.field;
                field.start_unix = spec.start_unix;
                submit(driver, &field)?;
                state.rev = field.rev();
                state.cache_hit = f.cache_hit;
                state.locations = f.locations;
                state.field = Some(Arc::new(field));
                state.error = None;
            }
            Err(err) => state.error = Some(format!("{err:#}")),
        }
        let state = Arc::new(state);
        self.current.store(Some(Arc::clone(&state)));
        Ok(state)
    }

    /// Keep a live run's weather fresh, and put it back if the world lost it
    /// (a reset rebuilds from the pack as it was loaded). Runs until `stop`.
    pub async fn run(self: Arc<Self>, driver: DriverHandle, mut stop: watch::Receiver<bool>) {
        let mut ticker = tokio::time::interval(Duration::from_secs(20));
        ticker.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Delay);
        loop {
            tokio::select! {
                _ = ticker.tick() => {}
                _ = stop.wait_for(|s| *s) => return,
            }
            let Some(current) = self.current() else {
                continue;
            };
            let Some(spec) = current
                .spec
                .as_ref()
                .filter(|s| s.mode == WeatherMode::Live)
            else {
                continue;
            };
            let due = now_unix() - current.attempted_unix >= i64::from(spec.refresh_min) * 60;
            let retry = current.error.is_some() && now_unix() - current.attempted_unix >= 60;
            if due || retry {
                if let Err(err) = self.refresh(&driver).await {
                    tracing::warn!(%err, "weather refresh");
                }
                continue;
            }
            // Out of step with the world: give it the field again.
            if let Some(field) = &current.field {
                if world_rev(&driver) != Some(current.rev) {
                    if let Err(err) = submit(&driver, field) {
                        tracing::warn!(%err, "could not resend the weather");
                    }
                }
            }
        }
    }
}

/// The fingerprint of the wind the world is flying in, from its overlays.
pub fn world_rev(driver: &DriverHandle) -> Option<u64> {
    driver.view().overlays["weather"]["rev"]
        .as_str()
        .and_then(|r| r.parse().ok())
}

/// Hand a field to the world as a `WEATHER_SET` intent, addressed to a cell
/// inside it.
fn submit(driver: &DriverHandle, field: &WindField) -> anyhow::Result<()> {
    let view = driver.view();
    let cell = view
        .cells
        .first()
        .map(|bag| bag.cell)
        .or_else(|| {
            let (lon, lat) = field.cells.first()?.cell.centroid().ok()?;
            GeoCell::from_lonlat(lon, lat, view.meta.resolution).ok()
        })
        .ok_or_else(|| anyhow::anyhow!("no cell to address the weather to"))?;
    let intent = Intent::operator(
        cell,
        IntentVerb::Custom(WEATHER_SET),
        serde_json::to_value(field)?,
    );
    driver
        .submit(Submission { intent, ack: None })
        .map_err(|err| anyhow::anyhow!("{err:?}"))?;
    Ok(())
}

/// RFC 3339 for an optional Unix time.
pub fn utc(unix: Option<i64>) -> Option<String> {
    unix.map(format_utc)
}
