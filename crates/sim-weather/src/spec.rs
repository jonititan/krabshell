//! What weather a scenario asks for, read from its params and fields.

use std::collections::BTreeMap;

use sim_core::config::Param;
use sim_core::{ScenarioBlob, WeatherMode};

use crate::{parse_utc, HOUR_S};

/// Everything a fetch needs.
#[derive(Debug, Clone, PartialEq)]
pub struct WeatherSpec {
    pub mode: WeatherMode,
    /// The UTC time of tick 0, Unix seconds.
    pub start_unix: i64,
    pub hours: u32,
    /// Resolution of the weather cells.
    pub resolution: i32,
    /// The world's occupancy resolution: weather cells are the ancestors of
    /// its cells, which is how a drone finds its wind.
    pub world_resolution: i32,
    /// `[min_lon, min_lat, max_lon, max_lat]`, degrees.
    pub bbox: [f64; 4],
    /// An Open-Meteo model token; `best_match` lets it choose.
    pub model: String,
    /// How often a live run fetches again, minutes.
    pub refresh_min: u32,
}

impl WeatherSpec {
    /// The first and last frame times, on the hour, covering the run.
    pub fn window(&self) -> (i64, i64) {
        let first = self.start_unix.div_euclid(HOUR_S) * HOUR_S;
        let end = self.start_unix + i64::from(self.hours) * HOUR_S;
        let last = (end + HOUR_S - 1).div_euclid(HOUR_S) * HOUR_S;
        (first, last.max(first + HOUR_S))
    }
}

fn text<'a>(params: &'a BTreeMap<String, Param>, key: &str) -> Option<&'a str> {
    match params.get(key) {
        Some(Param::Text(t)) => Some(t.as_str()),
        _ => None,
    }
}

fn number(params: &BTreeMap<String, Param>, key: &str) -> Option<f64> {
    params
        .get(key)
        .and_then(Param::as_f64)
        .filter(|v| v.is_finite())
}

/// The weather a scenario asks for, or `None` if it asks for none. `now` is
/// the wall clock, which a live run starts at.
pub fn spec_from(blob: &ScenarioBlob, now: i64) -> anyhow::Result<Option<WeatherSpec>> {
    let params = &blob.config.model;
    let mode = match text(params, "weather").unwrap_or("off") {
        "off" | "" => return Ok(None),
        "recorded" => WeatherMode::Recorded,
        "live" => WeatherMode::Live,
        other => anyhow::bail!("weather is \"off\", \"recorded\" or \"live\", not {other:?}"),
    };
    let start_unix = match mode {
        WeatherMode::Live => now,
        WeatherMode::Recorded => {
            let Some(start) = text(params, "weather_start_utc") else {
                anyhow::bail!(
                    "recorded weather needs weather_start_utc, e.g. \"2026-03-14T06:00:00Z\""
                );
            };
            parse_utc(start)?
        }
    };
    let hours = number(params, "weather_hours").unwrap_or(24.0);
    if !(1.0..=384.0).contains(&hours) {
        anyhow::bail!("weather_hours is 1 to 384 (16 days), not {hours}");
    }
    let world = blob.manifest.resolution;
    let resolution = number(params, "weather_resolution").map_or(12.min(world), |r| r as i32);
    if !(0..=world).contains(&resolution) {
        anyhow::bail!(
            "weather_resolution {resolution} must be no finer than the world's resolution {world}"
        );
    }
    let model = text(params, "weather_model")
        .unwrap_or("best_match")
        .to_owned();
    if model.is_empty() || !model.chars().all(|c| c.is_ascii_alphanumeric() || c == '_') {
        anyhow::bail!("weather_model {model:?} is not an Open-Meteo model token");
    }
    let refresh_min = number(params, "weather_refresh_min")
        .unwrap_or(60.0)
        .clamp(5.0, 1_440.0) as u32;
    Ok(Some(WeatherSpec {
        mode,
        start_unix,
        hours: hours as u32,
        resolution,
        world_resolution: world,
        bbox: area(blob)?,
        model,
        refresh_min,
    }))
}

/// The area to fetch: `weather_bbox`, else the pack's terrain, else its
/// agents, sites and addresses with `weather_margin_km` round them.
fn area(blob: &ScenarioBlob) -> anyhow::Result<[f64; 4]> {
    let params = &blob.config.model;
    if let Some(Param::List(list)) = params.get("weather_bbox") {
        let v: Vec<f64> = list.iter().filter_map(Param::as_f64).collect();
        let [w, s, e, n] = v[..] else {
            anyhow::bail!("weather_bbox is [min_lon, min_lat, max_lon, max_lat]");
        };
        if !(w < e && s < n && (-180.0..=180.0).contains(&w) && (-90.0..=90.0).contains(&s)) {
            anyhow::bail!("weather_bbox [{w}, {s}, {e}, {n}] is not a box");
        }
        return Ok([w, s, e, n]);
    }
    let fields = &blob.fields;
    if let Some(t) = fields.get("terrain") {
        let int = |k: &str| t.get(k).and_then(serde_json::Value::as_f64).unwrap_or(0.0);
        let (lat0, lon0) = (int("lat0_e7") * 1e-7, int("lon0_e7") * 1e-7);
        let lat1 = lat0 + int("dlat_e7") * 1e-7 * (int("rows") - 1.0);
        let lon1 = lon0 + int("dlon_e7") * 1e-7 * (int("cols") - 1.0);
        if lat1 > lat0 && lon1 > lon0 {
            return Ok([lon0, lat0, lon1, lat1]);
        }
    }

    let mut points: Vec<(f64, f64)> = Vec::new();
    for agent in &blob.agents {
        match (agent.lon, agent.lat, agent.cell) {
            (Some(lon), Some(lat), _) => points.push((lon, lat)),
            (_, _, Some(cell)) => points.extend(cell.centroid().ok()),
            _ => {}
        }
    }
    let lonlat = |v: &serde_json::Value| Some((v.get("lon")?.as_f64()?, v.get("lat")?.as_f64()?));
    for key in ["stations", "regions"] {
        if let Some(list) = fields.get(key).and_then(serde_json::Value::as_array) {
            points.extend(list.iter().filter_map(lonlat));
        }
    }
    // Addresses are `[lat, lon, ...]`.
    if let Some(items) = fields
        .pointer("/addresses/items")
        .and_then(serde_json::Value::as_array)
    {
        points.extend(
            items
                .iter()
                .filter_map(|i| Some((i.get(1)?.as_f64()?, i.get(0)?.as_f64()?))),
        );
    }
    if points.is_empty() {
        anyhow::bail!("nothing in the pack says where to fetch weather: give weather_bbox");
    }
    let margin_km = number(params, "weather_margin_km")
        .unwrap_or(3.0)
        .clamp(0.0, 200.0);
    let (mut w, mut s, mut e, mut n) = (f64::MAX, f64::MAX, f64::MIN, f64::MIN);
    for (lon, lat) in points {
        w = w.min(lon);
        e = e.max(lon);
        s = s.min(lat);
        n = n.max(lat);
    }
    let dlat = margin_km / 111.32;
    let dlon = dlat / ((s + n) / 2.0).to_radians().cos().max(0.01);
    Ok([w - dlon, s - dlat, e + dlon, n + dlat])
}

#[cfg(test)]
mod tests {
    use super::*;
    use sim_core::{Config, ScenarioManifest, SeedAgent};

    fn blob(params: &[(&str, Param)]) -> ScenarioBlob {
        let manifest = ScenarioManifest {
            id: "t".into(),
            name: "t".into(),
            description: String::new(),
            model: "drones".into(),
            engine_rev: None,
            resolution: 15,
            root: None,
            profile: Default::default(),
        };
        let mut config = Config::default();
        for (k, v) in params {
            config.model.insert((*k).into(), v.clone());
        }
        ScenarioBlob {
            manifest,
            config,
            agents: vec![SeedAgent {
                id: None,
                kind: "drone".into(),
                cell: None,
                lon: Some(23.9),
                lat: Some(54.9),
                state: serde_json::Value::Null,
            }],
            fields: serde_json::Value::Null,
        }
    }

    #[test]
    fn no_weather_param_means_no_weather() {
        assert_eq!(spec_from(&blob(&[]), 0).unwrap(), None);
    }

    #[test]
    fn recorded_needs_a_start_and_live_starts_now() {
        let b = blob(&[("weather", Param::Text("recorded".into()))]);
        assert!(spec_from(&b, 0).is_err());
        let b = blob(&[
            ("weather", Param::Text("recorded".into())),
            (
                "weather_start_utc",
                Param::Text("2026-03-14T06:30:00Z".into()),
            ),
            ("weather_hours", Param::Int(2)),
        ]);
        let s = spec_from(&b, 0).unwrap().unwrap();
        assert_eq!(s.start_unix, 1_773_469_800);
        assert_eq!(s.resolution, 12);
        // 06:00 to 09:00 covers 06:30 + 2 h.
        assert_eq!(s.window(), (1_773_468_000, 1_773_478_800));
        let live = blob(&[("weather", Param::Text("live".into()))]);
        assert_eq!(spec_from(&live, 1_000).unwrap().unwrap().start_unix, 1_000);
    }

    #[test]
    fn the_area_is_the_agents_with_a_margin_or_the_box_given() {
        let b = blob(&[("weather", Param::Text("live".into()))]);
        let [w, s, e, n] = spec_from(&b, 0).unwrap().unwrap().bbox;
        assert!(w < 23.9 && e > 23.9 && s < 54.9 && n > 54.9);
        assert!(((n - s) * 111.32 - 6.0).abs() < 0.01);
        let b = blob(&[
            ("weather", Param::Text("live".into())),
            (
                "weather_bbox",
                Param::List(vec![
                    Param::Float(23.0),
                    Param::Float(54.0),
                    Param::Float(24.0),
                    Param::Float(55.0),
                ]),
            ),
        ]);
        assert_eq!(
            spec_from(&b, 0).unwrap().unwrap().bbox,
            [23.0, 54.0, 24.0, 55.0]
        );
    }

    #[test]
    fn weather_finer_than_the_world_is_refused() {
        let b = blob(&[
            ("weather", Param::Text("live".into())),
            ("weather_resolution", Param::Int(16)),
        ]);
        assert!(spec_from(&b, 0).is_err());
    }
}
