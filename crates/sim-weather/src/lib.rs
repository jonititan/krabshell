//! Wind for a krabshell world, from Open-Meteo, one series per A5 cell.
//!
//! A pack asks for weather in its `params.toml`:
//!
//! ```toml
//! weather = "recorded"                     # or "live", or "off" (the default)
//! weather_start_utc = "2026-03-14T06:00:00Z"  # recorded: what tick 0 is
//! weather_hours = 24                       # how much to fetch
//! weather_resolution = 12                  # A5 resolution of the weather cells
//! weather_model = "best_match"             # an Open-Meteo model token
//! weather_bbox = [23.8, 54.85, 24.05, 54.97]  # optional: min lon, min lat, max lon, max lat
//! ```
//!
//! `recorded` fixes tick 0 at `weather_start_utc` and uses whatever Open-Meteo
//! holds for that window: its forecast archive for anything older than three
//! months, the forecast API otherwise (which includes forecasts for a start
//! still to come). `live` fixes tick 0 at the moment of loading and fetches the
//! latest forecast from the hour before; the server refreshes it while the run
//! goes on.
//!
//! Every weather cell is fetched at its centroid, at 10, 80, 120 and 180 m
//! above the ground, with gusts at 10 m, one frame an hour. The area is the
//! pack's terrain if it has any, else its agents and sites with a margin, or
//! `weather_bbox`.
//!
//! Fetched fields are cached on disk, keyed by what was asked: the same
//! scenario run again does not call Open-Meteo again. Windows wholly in the
//! past are kept for good; windows that reach into the future (forecasts) are
//! fetched again once an hour has turned, since the forecast will have moved.

pub mod cache;
pub mod fetch;
pub mod spec;

pub use cache::Cache;
pub use fetch::{Fetched, Fetcher};
pub use spec::{spec_from, WeatherSpec};

/// What Open-Meteo's licence asks users of its data to say.
pub const ATTRIBUTION: &str = "Weather data by Open-Meteo.com (CC BY 4.0)";

/// Heights above the ground fetched for every cell, metres.
pub const LEVELS_M: [u16; 4] = [10, 80, 120, 180];

/// Most weather cells one area may hold. Open-Meteo counts each location as
/// a call, so this also bounds what one load costs.
pub const MAX_CELLS: usize = 1_000;

/// Seconds in an hour, the frame step.
pub const HOUR_S: i64 = 3_600;

/// FNV-1a, 64-bit: a stable fingerprint for cache keys.
pub(crate) fn fnv(text: &str) -> u64 {
    let mut h: u64 = 0xcbf2_9ce4_8422_2325;
    for b in text.bytes() {
        h ^= u64::from(b);
        h = h.wrapping_mul(0x0100_0000_01b3);
    }
    h
}

/// Unix seconds now.
pub fn now_unix() -> i64 {
    std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map_or(0, |d| d.as_secs() as i64)
}

/// RFC 3339 for a Unix time, to the second.
pub fn format_utc(unix: i64) -> String {
    time::OffsetDateTime::from_unix_timestamp(unix)
        .ok()
        .and_then(|t| {
            t.format(&time::format_description::well_known::Rfc3339)
                .ok()
        })
        .unwrap_or_default()
}

/// Read an RFC 3339 time (`2026-03-14T06:00:00Z`) as Unix seconds.
pub fn parse_utc(text: &str) -> anyhow::Result<i64> {
    let t =
        time::OffsetDateTime::parse(text.trim(), &time::format_description::well_known::Rfc3339)
            .map_err(|e| anyhow::anyhow!("{text:?} is not an RFC 3339 time: {e}"))?;
    Ok(t.unix_timestamp())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn times_round_trip() {
        let t = parse_utc("2026-03-14T06:00:00Z").unwrap();
        assert_eq!(t, 1_773_468_000);
        assert_eq!(format_utc(t), "2026-03-14T06:00:00Z");
        assert!(parse_utc("yesterday").is_err());
    }
}
