//! Wind over the world, one series per A5 cell.
//!
//! The wind is held at one resolution, the *weather resolution*, coarser than
//! the occupancy grid: a weather model's grid is a kilometre or two, so finer
//! cells would only repeat it. Every cell finer than that takes the wind of
//! its ancestor there, so anything that knows its cell knows its wind.
//!
//! Each weather cell carries a series of frames (one an hour, usually), and
//! each frame the wind at a few heights above the ground. Between frames the
//! wind is linear in time, between heights linear in height, below the lowest
//! height it falls off as a power law toward the ground, and above the highest
//! it holds. Values are stored as whole centimetres per second, so a field
//! read from a pack or a checkpoint gives the same bits on every machine, and
//! the interpolation uses only IEEE arithmetic and `libm`.
//!
//! Who fills it (a fetch from a weather service, a file in a pack) is not this
//! module's business; it only says what the field is and how to read it.

use serde::{Deserialize, Serialize};

use crate::cell::GeoCell;

/// The exponent of the power-law profile below the lowest level: the classic
/// one-seventh for open ground.
const SHEAR_EXPONENT: f64 = 1.0 / 7.0;

/// Where a field's numbers came from.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum WeatherMode {
    /// A fixed start time, from recorded or forecast data fetched once.
    #[default]
    Recorded,
    /// Started at the wall-clock time of loading, refreshed while it runs.
    Live,
}

/// One weather cell's series.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct CellWind {
    pub cell: GeoCell,
    /// Wind toward the east, cm/s, `frames × levels`, frame-major.
    pub u_cms: Vec<i16>,
    /// Wind toward the north, cm/s, laid out as `u_cms`.
    pub v_cms: Vec<i16>,
    /// Gusts near the ground, cm/s, one per frame.
    pub gust_cms: Vec<i16>,
}

/// The wind over a world.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct WindField {
    /// Resolution every cell in `cells` is at.
    pub resolution: i32,
    pub mode: WeatherMode,
    /// The UTC time of tick 0, Unix seconds.
    pub start_unix: i64,
    /// The UTC time of frame 0, Unix seconds.
    pub t0_unix: i64,
    /// Seconds between frames.
    pub step_s: u32,
    pub frames: usize,
    /// Heights above the ground of each level, metres, ascending.
    pub levels_m: Vec<u16>,
    /// Sorted by cell.
    pub cells: Vec<CellWind>,
    /// What produced it, for people: the service and model.
    #[serde(default)]
    pub source: String,
    /// The credit the data's licence asks for.
    #[serde(default)]
    pub attribution: String,
    /// When it was fetched, Unix seconds.
    #[serde(default)]
    pub fetched_unix: i64,
}

/// The wind somewhere, in metres per second.
#[derive(Debug, Clone, Copy, PartialEq, Default, Serialize, Deserialize)]
pub struct Wind {
    /// Toward the east.
    pub east_ms: f64,
    /// Toward the north.
    pub north_ms: f64,
    /// Gusts near the ground.
    pub gust_ms: f64,
}

impl Wind {
    pub const CALM: Wind = Wind {
        east_ms: 0.0,
        north_ms: 0.0,
        gust_ms: 0.0,
    };

    pub fn speed_ms(&self) -> f64 {
        libm::sqrt(self.east_ms * self.east_ms + self.north_ms * self.north_ms)
    }

    /// The direction it blows *from*, degrees clockwise from north, as
    /// forecasts and windsocks give it.
    pub fn from_deg(&self) -> f64 {
        if self.east_ms == 0.0 && self.north_ms == 0.0 {
            return 0.0;
        }
        let toward = libm::atan2(self.east_ms, self.north_ms).to_degrees();
        (toward + 180.0).rem_euclid(360.0)
    }

    pub fn is_calm(&self) -> bool {
        self.east_ms == 0.0 && self.north_ms == 0.0
    }
}

#[derive(Debug, Clone, PartialEq, Eq, thiserror::Error)]
pub enum WeatherError {
    #[error("weather: {0}")]
    Invalid(String),
}

impl WindField {
    /// Check a field built or read from outside, and sort its cells, which
    /// lookups rely on.
    pub fn validate(mut self) -> Result<WindField, WeatherError> {
        let bad = |m: String| Err(WeatherError::Invalid(m));
        if self.frames == 0 || self.step_s == 0 {
            return bad("a field needs at least one frame and a step".into());
        }
        if self.levels_m.is_empty() || self.levels_m.windows(2).any(|w| w[0] >= w[1]) {
            return bad("levels_m must be non-empty and ascending".into());
        }
        if self.levels_m[0] == 0 {
            return bad("the lowest level must be above the ground".into());
        }
        let per = self.frames * self.levels_m.len();
        for c in &self.cells {
            if c.cell.resolution() != self.resolution {
                return bad(format!(
                    "cell {} is at resolution {}, not {}",
                    c.cell,
                    c.cell.resolution(),
                    self.resolution
                ));
            }
            if c.u_cms.len() != per || c.v_cms.len() != per || c.gust_cms.len() != self.frames {
                return bad(format!("cell {} has the wrong number of values", c.cell));
            }
        }
        self.cells.sort_by_key(|c| c.cell);
        self.cells.dedup_by_key(|c| c.cell);
        Ok(self)
    }

    /// Read a field from JSON, as a pack or checkpoint holds it.
    pub fn from_value(value: &serde_json::Value) -> Result<WindField, WeatherError> {
        let field =
            WindField::deserialize(value).map_err(|e| WeatherError::Invalid(e.to_string()))?;
        field.validate()
    }

    /// A stable fingerprint of the numbers and the clock, so two copies can
    /// be told apart cheaply. FNV-1a over everything that changes the wind.
    pub fn rev(&self) -> u64 {
        let mut h: u64 = 0xcbf2_9ce4_8422_2325;
        let mut eat = |bytes: &[u8]| {
            for b in bytes {
                h ^= u64::from(*b);
                h = h.wrapping_mul(0x0100_0000_01b3);
            }
        };
        eat(&self.resolution.to_le_bytes());
        eat(&self.start_unix.to_le_bytes());
        eat(&self.t0_unix.to_le_bytes());
        eat(&self.step_s.to_le_bytes());
        eat(&(self.frames as u64).to_le_bytes());
        for l in &self.levels_m {
            eat(&l.to_le_bytes());
        }
        for c in &self.cells {
            eat(&c.cell.raw().to_le_bytes());
            for v in c.u_cms.iter().chain(&c.v_cms).chain(&c.gust_cms) {
                eat(&v.to_le_bytes());
            }
        }
        h
    }

    /// The weather cell a cell of any resolution at least as fine lies in.
    pub fn cell_for(&self, cell: GeoCell) -> Option<GeoCell> {
        let r = cell.resolution();
        if r == self.resolution {
            Some(cell)
        } else if r > self.resolution {
            cell.parent(Some(self.resolution)).ok()
        } else {
            None
        }
    }

    pub fn series(&self, weather_cell: GeoCell) -> Option<&CellWind> {
        self.cells
            .binary_search_by_key(&weather_cell, |c| c.cell)
            .ok()
            .map(|i| &self.cells[i])
    }

    /// UTC Unix seconds at `sim_s` simulated seconds after tick 0.
    pub fn utc_at(&self, sim_s: f64) -> f64 {
        self.start_unix as f64 + sim_s
    }

    /// Frame position (fractional) at a UTC time, clamped to the field.
    fn frame_at(&self, unix: f64) -> (usize, f64) {
        let pos = ((unix - self.t0_unix as f64) / f64::from(self.step_s))
            .clamp(0.0, (self.frames - 1) as f64);
        let i = (libm::floor(pos) as usize).min(self.frames.saturating_sub(2));
        if self.frames == 1 {
            return (0, 0.0);
        }
        (i, pos - i as f64)
    }

    /// The wind in `cell` (any resolution at least as fine as the field's),
    /// `agl_m` above the ground, at UTC `unix`. `None` outside the field.
    pub fn wind_at(&self, cell: GeoCell, agl_m: f64, unix: f64) -> Option<Wind> {
        let series = self.series(self.cell_for(cell)?)?;
        Some(self.sample(series, agl_m, unix))
    }

    /// The wind of one series at a height and time.
    pub fn sample(&self, series: &CellWind, agl_m: f64, unix: f64) -> Wind {
        let (f, ft) = self.frame_at(unix);
        let levels = self.levels_m.len();
        let at = |values: &[i16], frame: usize, level: usize| -> f64 {
            f64::from(values[frame * levels + level]) * 0.01
        };
        let in_time = |values: &[i16], level: usize| -> f64 {
            let a = at(values, f, level);
            if self.frames == 1 {
                return a;
            }
            a + (at(values, f + 1, level) - a) * ft
        };
        // Height: which levels bracket it, and how far between.
        let lowest = f64::from(self.levels_m[0]);
        let (lo, hi, t, scale) = if agl_m <= lowest {
            let h = agl_m.max(0.0);
            (0, 0, 0.0, libm::pow(h / lowest, SHEAR_EXPONENT))
        } else if let Some(k) = self
            .levels_m
            .windows(2)
            .position(|w| agl_m <= f64::from(w[1]))
        {
            let (a, b) = (f64::from(self.levels_m[k]), f64::from(self.levels_m[k + 1]));
            (k, k + 1, (agl_m - a) / (b - a), 1.0)
        } else {
            (levels - 1, levels - 1, 0.0, 1.0)
        };
        let in_height = |values: &[i16]| -> f64 {
            let a = in_time(values, lo);
            (a + (in_time(values, hi) - a) * t) * scale
        };
        let gust = {
            let g = |frame: usize| f64::from(series.gust_cms[frame]) * 0.01;
            if self.frames == 1 {
                g(0)
            } else {
                g(f) + (g(f + 1) - g(f)) * ft
            }
        };
        Wind {
            east_ms: in_height(&series.u_cms),
            north_ms: in_height(&series.v_cms),
            gust_ms: gust,
        }
    }

    /// The UTC time of each frame.
    pub fn frame_times(&self) -> impl Iterator<Item = i64> + '_ {
        (0..self.frames).map(|i| self.t0_unix + i as i64 * i64::from(self.step_s))
    }
}

/// Eastward and northward components, cm/s, of a wind given as a speed in
/// m/s and the direction it blows from in degrees, as weather services give it.
pub fn components_cms(speed_ms: f64, from_deg: f64) -> (i16, i16) {
    let rad = from_deg.to_radians();
    let east = -speed_ms * libm::sin(rad) * 100.0;
    let north = -speed_ms * libm::cos(rad) * 100.0;
    let clamp = |v: f64| libm::round(v).clamp(f64::from(i16::MIN), f64::from(i16::MAX)) as i16;
    (clamp(east), clamp(north))
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A resolution-15 cell, as a drone would be in.
    fn fine() -> GeoCell {
        GeoCell::from_lonlat(23.9, 54.9, 15).unwrap()
    }

    /// Its weather cell. Not always the cell the same point falls in at
    /// resolution 12: A5's hierarchy, not the point, decides.
    fn cell() -> GeoCell {
        fine().parent(Some(12)).unwrap()
    }

    /// Two frames an hour apart, levels 10 and 100 m: 2 → 4 m/s east at
    /// 10 m and 6 → 8 m/s at 100 m, gusts 5 → 7.
    fn field() -> WindField {
        WindField {
            resolution: 12,
            mode: WeatherMode::Recorded,
            start_unix: 1_000_000,
            t0_unix: 1_000_000,
            step_s: 3600,
            frames: 2,
            levels_m: vec![10, 100],
            cells: vec![CellWind {
                cell: cell(),
                u_cms: vec![200, 600, 400, 800],
                v_cms: vec![0, 0, 0, 0],
                gust_cms: vec![500, 700],
            }],
            source: String::new(),
            attribution: String::new(),
            fetched_unix: 0,
        }
        .validate()
        .unwrap()
    }

    #[test]
    fn a_fine_cell_takes_its_ancestors_wind() {
        let f = field();
        let fine = fine();
        assert_eq!(f.cell_for(fine), Some(cell()));
        let w = f.wind_at(fine, 10.0, 1_000_000.0).unwrap();
        assert!((w.east_ms - 2.0).abs() < 1e-9);
        let coarse = GeoCell::from_lonlat(23.9, 54.9, 9).unwrap();
        assert_eq!(f.wind_at(coarse, 10.0, 1_000_000.0), None);
    }

    #[test]
    fn wind_is_linear_in_time_and_height_and_holds_past_the_ends() {
        let f = field();
        let c = cell();
        let east = |agl: f64, t: f64| f.wind_at(c, agl, t).unwrap().east_ms;
        // Half an hour in, half way up: (3 + 7) / 2.
        assert!((east(55.0, 1_001_800.0) - 5.0).abs() < 1e-9);
        // Before the first frame and after the last, the ends hold.
        assert!((east(10.0, 0.0) - 2.0).abs() < 1e-9);
        assert!((east(100.0, 9e9) - 8.0).abs() < 1e-9);
        // Above the top level, the top holds.
        assert!((east(500.0, 1_000_000.0) - 6.0).abs() < 1e-9);
        assert!((f.wind_at(c, 0.0, 1_001_800.0).unwrap().gust_ms - 6.0).abs() < 1e-9);
    }

    #[test]
    fn below_the_lowest_level_wind_falls_toward_the_ground() {
        let f = field();
        let w = |agl| f.wind_at(cell(), agl, 1_000_000.0).unwrap().east_ms;
        assert_eq!(w(0.0), 0.0);
        assert!(w(1.0) > 0.0 && w(1.0) < w(5.0) && w(5.0) < w(10.0));
    }

    #[test]
    fn components_follow_the_meteorological_convention() {
        // From the west blows toward the east.
        assert_eq!(components_cms(5.0, 270.0), (500, 0));
        // From the north blows toward the south.
        assert_eq!(components_cms(5.0, 0.0), (0, -500));
        let w = Wind {
            east_ms: 5.0,
            north_ms: 0.0,
            gust_ms: 0.0,
        };
        assert!((w.from_deg() - 270.0).abs() < 1e-9);
    }

    #[test]
    fn a_field_round_trips_through_json_with_the_same_rev() {
        let f = field();
        let back = WindField::from_value(&serde_json::to_value(&f).unwrap()).unwrap();
        assert_eq!(back.rev(), f.rev());
        assert_eq!(
            back.wind_at(cell(), 10.0, 1_000_000.0),
            f.wind_at(cell(), 10.0, 1_000_000.0)
        );
    }

    #[test]
    fn a_bad_field_is_refused() {
        let mut f = field();
        f.cells[0].u_cms.pop();
        assert!(f.validate().is_err());
    }
}
