//! Ground elevation, from a regular latitude/longitude grid in the pack.
//!
//! Altitudes in the drones model are above mean sea level. A pack without
//! terrain has flat ground at 0 m, which is what every altitude meant before
//! terrain existed, so such packs behave exactly as they always did.
//!
//! The grid lives in `world/fields.json` under `"terrain"`:
//!
//! ```json
//! { "terrain": {
//!     "lat0_e7": 548400000, "lon0_e7": 238000000,
//!     "dlat_e7": 2778, "dlon_e7": 4167,
//!     "rows": 540, "cols": 504,
//!     "elev_dm": [ ... rows × cols, row 0 southmost, west to east ... ],
//!     "source": "free text, carried for attribution"
//! } }
//! ```
//!
//! Sample `(r, c)` is at `lat0 + r·dlat`, `lon0 + c·dlon`, in decimetres above
//! mean sea level. Between samples the ground is bilinear; beyond the edge it
//! is the nearest edge sample. Interpolation is integer arithmetic, so a
//! `gold` run gets the same millimetres on every machine.

use serde::Deserialize;

#[derive(Debug, Clone, Default, PartialEq)]
pub struct Terrain {
    lat0_e7: i64,
    lon0_e7: i64,
    dlat_e7: i64,
    dlon_e7: i64,
    rows: usize,
    cols: usize,
    elev_dm: Vec<i32>,
}

#[derive(Deserialize)]
struct Grid {
    lat0_e7: i64,
    lon0_e7: i64,
    dlat_e7: i64,
    dlon_e7: i64,
    rows: usize,
    cols: usize,
    elev_dm: Vec<i32>,
}

impl Terrain {
    /// Flat ground at sea level.
    pub const FLAT: Terrain = Terrain {
        lat0_e7: 0,
        lon0_e7: 0,
        dlat_e7: 1,
        dlon_e7: 1,
        rows: 0,
        cols: 0,
        elev_dm: Vec::new(),
    };

    /// Read the `"terrain"` entry of a pack's fields, or flat ground if there
    /// is none.
    pub fn from_fields(fields: &serde_json::Value) -> anyhow::Result<Terrain> {
        let Some(value) = fields.get("terrain") else {
            return Ok(Terrain::FLAT);
        };
        let grid = Grid::deserialize(value)
            .map_err(|e| anyhow::anyhow!("world/fields.json terrain: {e}"))?;
        if grid.rows < 2 || grid.cols < 2 {
            anyhow::bail!("world/fields.json terrain: needs at least 2 × 2 samples");
        }
        if grid.dlat_e7 <= 0 || grid.dlon_e7 <= 0 {
            anyhow::bail!("world/fields.json terrain: spacing must be positive");
        }
        if grid.elev_dm.len() != grid.rows * grid.cols {
            anyhow::bail!(
                "world/fields.json terrain: {} samples for a {} × {} grid",
                grid.elev_dm.len(),
                grid.rows,
                grid.cols
            );
        }
        Ok(Terrain {
            lat0_e7: grid.lat0_e7,
            lon0_e7: grid.lon0_e7,
            dlat_e7: grid.dlat_e7,
            dlon_e7: grid.dlon_e7,
            rows: grid.rows,
            cols: grid.cols,
            elev_dm: grid.elev_dm,
        })
    }

    pub fn is_flat(&self) -> bool {
        self.elev_dm.is_empty()
    }

    /// Ground elevation above mean sea level, millimetres.
    pub fn ground_mm(&self, lat_e7: i32, lon_e7: i32) -> i32 {
        if self.is_flat() {
            return 0;
        }
        // Grid position in 1/65536ths of a sample, clamped to the grid.
        const ONE: i64 = 1 << 16;
        let along = |at: i32, origin: i64, step: i64, count: usize| -> (usize, i64) {
            let max = (count as i64 - 1) * ONE;
            let pos = ((i64::from(at) - origin) * ONE / step).clamp(0, max);
            let index = ((pos / ONE) as usize).min(count - 2);
            (index, pos - index as i64 * ONE)
        };
        let (r, fr) = along(lat_e7, self.lat0_e7, self.dlat_e7, self.rows);
        let (c, fc) = along(lon_e7, self.lon0_e7, self.dlon_e7, self.cols);
        let at = |r: usize, c: usize| i64::from(self.elev_dm[r * self.cols + c]);
        let south = at(r, c) * (ONE - fc) + at(r, c + 1) * fc;
        let north = at(r + 1, c) * (ONE - fc) + at(r + 1, c + 1) * fc;
        let dm_q32 = south * (ONE - fr) + north * fr;
        // Decimetres × 2³² to millimetres, rounding half up.
        ((dm_q32 * 100 + (1 << 31)) >> 32) as i32
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn grid() -> Terrain {
        Terrain::from_fields(&serde_json::json!({ "terrain": {
            "lat0_e7": 0, "lon0_e7": 0, "dlat_e7": 1000, "dlon_e7": 1000,
            "rows": 2, "cols": 3,
            "elev_dm": [100, 200, 300,
                        300, 400, 500],
        }}))
        .unwrap()
    }

    #[test]
    fn samples_are_exact_and_between_them_is_bilinear() {
        let t = grid();
        assert_eq!(t.ground_mm(0, 0), 10_000);
        assert_eq!(t.ground_mm(1000, 2000), 50_000);
        assert_eq!(t.ground_mm(500, 500), 25_000);
        assert_eq!(t.ground_mm(0, 1500), 25_000);
    }

    #[test]
    fn beyond_the_edge_is_the_nearest_edge() {
        let t = grid();
        assert_eq!(t.ground_mm(-5000, -5000), 10_000);
        assert_eq!(t.ground_mm(9000, 9000), 50_000);
        assert_eq!(t.ground_mm(9000, 1000), 40_000);
    }

    #[test]
    fn no_terrain_is_flat_sea_level() {
        let t = Terrain::from_fields(&serde_json::Value::Null).unwrap();
        assert!(t.is_flat());
        assert_eq!(t.ground_mm(515_000_000, -1_000_000), 0);
    }

    #[test]
    fn a_short_sample_list_is_refused() {
        let bad = serde_json::json!({ "terrain": {
            "lat0_e7": 0, "lon0_e7": 0, "dlat_e7": 1, "dlon_e7": 1,
            "rows": 2, "cols": 2, "elev_dm": [1, 2, 3],
        }});
        assert!(Terrain::from_fields(&bad).is_err());
    }
}
