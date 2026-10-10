//! Fetching a [`WindField`] from Open-Meteo.

use openmeteo_rs::{Client, HourlyVar, TimeFormat, TowerLevel, WeatherModel, WindSpeedUnit};
use sim_core::cell::{cells_in_bbox, CellError};
use sim_core::weather::{components_cms, CellWind};
use sim_core::{GeoCell, WindField};

use crate::{fnv, Cache, WeatherSpec, ATTRIBUTION, HOUR_S, LEVELS_M, MAX_CELLS};

/// Locations per request. Open-Meteo takes up to a thousand, but a hundred
/// keeps the URL a sane length.
const CHUNK: usize = 100;

/// Most world cells an area is sampled into, finding its weather cells.
const MAX_WORLD_CELLS: usize = 2_000_000;

/// The forecast API keeps about three months behind it; older windows come
/// from the historical forecast archive, which has the same variables.
const FORECAST_PAST_S: i64 = 85 * 86_400;

const FORECAST_HOST: &str = "https://api.open-meteo.com/";
const HISTORICAL_HOST: &str = "https://historical-forecast-api.open-meteo.com/";

/// Where to fetch from, if not Open-Meteo's free hosts.
#[derive(Debug, Clone, Default)]
pub struct Hosts {
    /// A commercial API key (needed for commercial use).
    pub api_key: Option<String>,
    /// The forecast host's root, e.g. a self-hosted Open-Meteo.
    pub forecast: Option<String>,
    /// The historical forecast host's root.
    pub historical: Option<String>,
}

/// A fetched field and where it came from.
#[derive(Debug, Clone)]
pub struct Fetched {
    pub field: WindField,
    /// Read from the cache rather than fetched.
    pub cache_hit: bool,
    /// Open-Meteo locations asked for; 0 on a cache hit.
    pub locations: usize,
    pub key: String,
}

pub struct Fetcher {
    forecast: Client,
    historical: Client,
    hosts: (String, String),
    cache: Cache,
}

impl Fetcher {
    pub fn new(cache: Cache, hosts: Hosts) -> anyhow::Result<Fetcher> {
        let forecast_host = hosts
            .forecast
            .clone()
            .unwrap_or_else(|| FORECAST_HOST.into());
        let historical_host = hosts
            .historical
            .clone()
            .unwrap_or_else(|| HISTORICAL_HOST.into());
        let client = |host: &str| -> anyhow::Result<Client> {
            let mut b = Client::builder()
                .user_agent(concat!("krabshell/", env!("CARGO_PKG_VERSION")))
                .timeout(std::time::Duration::from_secs(30));
            if let Some(key) = &hosts.api_key {
                b = b.api_key(key.clone());
            }
            Ok(b.forecast_base_url(host)?.build()?)
        };
        Ok(Fetcher {
            forecast: client(&forecast_host)?,
            historical: client(&historical_host)?,
            hosts: (forecast_host, historical_host),
            cache,
        })
    }

    pub fn cache(&self) -> &Cache {
        &self.cache
    }

    /// The weather cells covering a spec's area: the ancestors, at the
    /// weather resolution, of every world cell in it. A drone finds its wind
    /// through its cell's ancestor, and A5 ancestors do not always cover
    /// their descendants, so the area's own cells at the weather resolution
    /// would not do.
    pub fn cells(spec: &WeatherSpec) -> anyhow::Result<Vec<GeoCell>> {
        let [w, s, e, n] = spec.bbox;
        let too_many = |limit| {
            anyhow::anyhow!(
                "the area needs more than {limit} weather cells at resolution {}: \
                 lower weather_resolution or shrink weather_bbox",
                spec.resolution
            )
        };
        let world =
            cells_in_bbox(w, s, e, n, spec.world_resolution, MAX_WORLD_CELLS).map_err(|err| {
                match err {
                    CellError::TooManyCells { .. } => anyhow::anyhow!(
                        "the weather area is too big for resolution {} cells: shrink weather_bbox",
                        spec.world_resolution
                    ),
                    other => anyhow::anyhow!("weather area: {other}"),
                }
            })?;
        let mut cells = std::collections::BTreeSet::new();
        for c in world {
            cells.insert(c.parent(Some(spec.resolution))?);
            if cells.len() > MAX_CELLS {
                return Err(too_many(MAX_CELLS));
            }
        }
        Ok(cells.into_iter().collect())
    }

    /// The wind a spec asks for, from the cache if it holds it, else from
    /// Open-Meteo (and then cached). `now` is the wall clock.
    pub async fn fetch(&self, spec: &WeatherSpec, now: i64) -> anyhow::Result<Fetched> {
        let cells = Self::cells(spec)?;
        let (first, last) = spec.window();
        let historical = first < now - FORECAST_PAST_S;
        let (client, host) = if historical {
            (&self.historical, &self.hosts.1)
        } else {
            (&self.forecast, &self.hosts.0)
        };
        let cell_list: String = cells
            .iter()
            .map(|c| c.to_hex())
            .collect::<Vec<_>>()
            .join(",");
        let mut key = format!(
            "v1|{host}|{}|{}|{:016x}|{first}|{last}",
            spec.model,
            spec.resolution,
            fnv(&cell_list)
        );
        if last > now {
            // A forecast: good until the hour turns.
            key.push_str(&format!("|issued={}", now.div_euclid(HOUR_S)));
        }
        let key = format!("{:016x}", fnv(&key));

        if let Some(mut field) = self.cache.get(&key) {
            field.start_unix = spec.start_unix;
            field.mode = spec.mode;
            return Ok(Fetched {
                field,
                cache_hit: true,
                locations: 0,
                key,
            });
        }

        let frames = ((last - first) / HOUR_S + 1) as usize;
        let mut rows: Vec<Columns> = Vec::with_capacity(cells.len());
        for chunk in cells.chunks(CHUNK) {
            let points: Vec<(f64, f64)> = chunk
                .iter()
                .map(|c| c.centroid().map(|(lon, lat)| (lat, lon)))
                .collect::<Result<_, _>>()?;
            let mut request = client
                .forecast_batch(points)
                .hourly(variables())
                .wind_speed_unit(WindSpeedUnit::Ms)
                .timeformat(TimeFormat::UnixTime)
                .hour_range(primitive(first)?, primitive(last)?);
            if spec.model != "best_match" {
                request = request.models([WeatherModel::other(spec.model.clone())]);
            }
            let responses = request.send().await?;
            if responses.len() != chunk.len() {
                anyhow::bail!(
                    "Open-Meteo answered {} locations for {} asked",
                    responses.len(),
                    chunk.len()
                );
            }
            for response in responses {
                let hourly = response
                    .hourly
                    .ok_or_else(|| anyhow::anyhow!("Open-Meteo sent no hourly data"))?;
                rows.push(Columns::read(&hourly, first, frames));
            }
        }

        let field = assemble(spec, &cells, rows, first, frames, now, historical)?;
        if let Err(err) = self.cache.put(&key, &field) {
            tracing::warn!(%err, dir = %self.cache.dir().display(), "could not cache the weather");
        }
        Ok(Fetched {
            field,
            cache_hit: false,
            locations: cells.len(),
            key,
        })
    }
}

fn variables() -> Vec<HourlyVar> {
    let mut vars = vec![
        HourlyVar::WindSpeed10m,
        HourlyVar::WindDirection10m,
        HourlyVar::WindGusts10m,
    ];
    for level in [TowerLevel::M80, TowerLevel::M120, TowerLevel::M180] {
        vars.push(HourlyVar::WindSpeedAtTower(level));
        vars.push(HourlyVar::WindDirectionAtTower(level));
    }
    vars
}

fn primitive(unix: i64) -> anyhow::Result<time::PrimitiveDateTime> {
    let t = time::OffsetDateTime::from_unix_timestamp(unix)?;
    Ok(time::PrimitiveDateTime::new(t.date(), t.time()))
}

/// One location's values on the field's frames: `(east, north)` per level,
/// and gusts, each `None` where Open-Meteo had nothing.
struct Columns {
    wind: Vec<Vec<Option<(i16, i16)>>>,
    gust: Vec<Option<i16>>,
}

impl Columns {
    fn read(hourly: &openmeteo_rs::HourlyData, first: i64, frames: usize) -> Columns {
        // Which row each frame is, by its timestamp.
        let row_of: std::collections::BTreeMap<i64, usize> = hourly
            .time
            .iter()
            .enumerate()
            .map(|(i, t)| (t.unix_timestamp(), i))
            .collect();
        let rows: Vec<Option<usize>> = (0..frames)
            .map(|f| row_of.get(&(first + f as i64 * HOUR_S)).copied())
            .collect();
        let column = |name: &str| -> Vec<Option<f32>> {
            let values = hourly.get(name).and_then(|s| s.values_f32());
            rows.iter()
                .map(|r| values.and_then(|v| v.get((*r)?).copied().flatten()))
                .collect()
        };
        let wind = LEVELS_M
            .iter()
            .map(|m| {
                let speed = column(&format!("wind_speed_{m}m"));
                let from = column(&format!("wind_direction_{m}m"));
                speed
                    .iter()
                    .zip(&from)
                    .map(|(s, d)| Some(components_cms(f64::from((*s)?), f64::from((*d)?))))
                    .collect()
            })
            .collect();
        let gust = column("wind_gusts_10m")
            .into_iter()
            .map(|g| g.map(|g| (f64::from(g) * 100.0).round().clamp(0.0, 32_767.0) as i16))
            .collect();
        Columns { wind, gust }
    }
}

/// Put the locations' columns together into a field, keeping the levels any
/// location has data for and filling gaps: a missing level from the nearest
/// level that has a value, a missing frame from the one before (or after).
fn assemble(
    spec: &WeatherSpec,
    cells: &[GeoCell],
    rows: Vec<Columns>,
    first: i64,
    frames: usize,
    now: i64,
    historical: bool,
) -> anyhow::Result<WindField> {
    let kept: Vec<usize> = (0..LEVELS_M.len())
        .filter(|&l| rows.iter().any(|r| r.wind[l].iter().any(Option::is_some)))
        .collect();
    if kept.is_empty() {
        anyhow::bail!("Open-Meteo has no wind for this area and time");
    }
    let levels = kept.len();
    let mut out = Vec::with_capacity(cells.len());
    for (cell, row) in cells.iter().zip(rows) {
        let mut frame_values: Vec<Option<Vec<(i16, i16)>>> = (0..frames)
            .map(|f| {
                let have: Vec<Option<(i16, i16)>> = kept.iter().map(|&l| row.wind[l][f]).collect();
                if have.iter().all(Option::is_none) {
                    return None;
                }
                Some(
                    (0..levels)
                        .map(|k| {
                            have[k].unwrap_or_else(|| {
                                // The nearest level with a value.
                                (1..levels)
                                    .flat_map(|d| [k.checked_sub(d), Some(k + d)])
                                    .flatten()
                                    .find_map(|j| have.get(j).copied().flatten())
                                    .unwrap_or((0, 0))
                            })
                        })
                        .collect(),
                )
            })
            .collect();
        fill_frames(&mut frame_values, vec![(0, 0); levels]);
        let mut gusts: Vec<Option<i16>> = row.gust.clone();
        fill_frames(&mut gusts, 0);
        let (mut u, mut v) = (
            Vec::with_capacity(frames * levels),
            Vec::with_capacity(frames * levels),
        );
        for values in frame_values.into_iter().flatten() {
            for (east, north) in values {
                u.push(east);
                v.push(north);
            }
        }
        out.push(CellWind {
            cell: *cell,
            u_cms: u,
            v_cms: v,
            gust_cms: gusts.into_iter().flatten().collect(),
        });
    }
    let field = WindField {
        resolution: spec.resolution,
        mode: spec.mode,
        start_unix: spec.start_unix,
        t0_unix: first,
        step_s: HOUR_S as u32,
        frames,
        levels_m: kept.iter().map(|&l| LEVELS_M[l]).collect(),
        cells: out,
        source: format!(
            "Open-Meteo {} {}",
            spec.model,
            if historical {
                "historical forecast"
            } else {
                "forecast"
            }
        ),
        attribution: ATTRIBUTION.into(),
        fetched_unix: now,
    };
    Ok(field.validate()?)
}

/// Fill `None`s from the frame before, or for leading ones the first frame
/// that has a value, or `empty` if none does.
fn fill_frames<T: Clone>(values: &mut [Option<T>], empty: T) {
    let first = values.iter().flatten().next().cloned().unwrap_or(empty);
    let mut last = first;
    for v in values.iter_mut() {
        match v {
            Some(x) => last = x.clone(),
            None => *v = Some(last.clone()),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn gaps_are_filled_from_the_frame_before_or_the_first_with_a_value() {
        let mut v = vec![None, Some(3), None, Some(5), None];
        fill_frames(&mut v, 0);
        assert_eq!(v, vec![Some(3), Some(3), Some(3), Some(5), Some(5)]);
        let mut none: Vec<Option<i32>> = vec![None, None];
        fill_frames(&mut none, 7);
        assert_eq!(none, vec![Some(7), Some(7)]);
    }
}
