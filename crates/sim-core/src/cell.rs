//! The A5 address space.
//!
//! [`GeoCell`] is a newtype over the raw `u64` A5 index. It is deliberately
//! *not* `bevy_a5::GeoCell`: the wire format and the HTTP edge must not drag
//! Bevy into the server process (spec §4.4, §16). Both wrap the same `u64`
//! produced by the same version of the [`a5`] crate, so
//! `bevy_a5::GeoCell::new(cell.raw())` round-trips exactly.

use std::fmt;

use serde::{de::Error as _, Deserialize, Deserializer, Serialize, Serializer};

/// Highest resolution the A5 index can represent. Mirrors
/// `a5::core::serialization::MAX_RESOLUTION`, which is not re-exported.
pub const MAX_RESOLUTION: i32 = 30;

/// Failure modes of the cell maths. Every one of these is a client error, not
/// a server fault, so they map onto 400-class problem responses.
#[derive(Debug, Clone, PartialEq, Eq, thiserror::Error)]
pub enum CellError {
    #[error("resolution {0} is outside 0..={MAX_RESOLUTION}")]
    BadResolution(i32),
    #[error("'{0}' is not a cell id (expected hex, 0x-hex or decimal u64)")]
    BadCellId(String),
    #[error("bounding box is empty or inverted")]
    BadBbox,
    #[error("query would return more than {limit} cells")]
    TooManyCells { limit: usize },
    #[error("a5: {0}")]
    A5(String),
}

/// An A5 cell index.
///
/// Ordering is by raw index, which is also hierarchical: children of a cell
/// sort contiguously. Snapshots rely on that for stable `gold` hashing.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord, Default)]
pub struct GeoCell(u64);

impl GeoCell {
    /// The whole planet. Used as "no root" in grid construction.
    pub const WORLD: GeoCell = GeoCell(a5::WORLD_CELL);

    pub const fn from_raw(raw: u64) -> Self {
        GeoCell(raw)
    }

    pub const fn raw(self) -> u64 {
        self.0
    }

    /// Locate a lon/lat (degrees) at `resolution`.
    pub fn from_lonlat(longitude: f64, latitude: f64, resolution: i32) -> Result<Self, CellError> {
        check_resolution(resolution)?;
        a5::lonlat_to_cell(a5::LonLat::new(longitude, latitude), resolution)
            .map(GeoCell)
            .map_err(CellError::A5)
    }

    /// Centroid of the cell as `(longitude, latitude)` in degrees.
    pub fn centroid(self) -> Result<(f64, f64), CellError> {
        let ll = a5::cell_to_lonlat(self.0).map_err(CellError::A5)?;
        Ok((ll.longitude(), ll.latitude()))
    }

    pub fn resolution(self) -> i32 {
        a5::get_resolution(self.0)
    }

    /// Parent at `resolution`, or the immediate parent when `None`.
    pub fn parent(self, resolution: Option<i32>) -> Result<Self, CellError> {
        if let Some(r) = resolution {
            check_resolution(r)?;
        }
        a5::cell_to_parent(self.0, resolution)
            .map(GeoCell)
            .map_err(CellError::A5)
    }

    /// Children at `resolution`, or the immediate children when `None`.
    pub fn children(self, resolution: Option<i32>) -> Result<Vec<Self>, CellError> {
        if let Some(r) = resolution {
            check_resolution(r)?;
        }
        a5::cell_to_children(self.0, resolution)
            .map(wrap_all)
            .map_err(CellError::A5)
    }

    /// Edge-adjacent cells, excluding `self`.
    pub fn neighbors(self) -> Result<Vec<Self>, CellError> {
        let mut cells = self.grid_disk(1)?;
        cells.retain(|c| *c != self);
        Ok(cells)
    }

    /// All cells within `k` steps, including `self`, at `self`'s resolution.
    ///
    /// `a5::grid_disk` returns a *compacted* cover: runs of sibling cells are
    /// collapsed into their parent, so the centre cell itself may not appear in
    /// the raw result. Subscriptions and bags are keyed at one resolution, so
    /// the cover is expanded back before it is returned. Use
    /// [`GeoCell::grid_disk_compact`] when a mixed-resolution cover is what you
    /// actually want, such as drawing an outline.
    pub fn grid_disk(self, k: usize) -> Result<Vec<Self>, CellError> {
        let cover = a5::grid_disk(self.0, k).map_err(CellError::A5)?;
        expand_to(cover, self.resolution())
    }

    /// The raw compacted cover, cells possibly coarser than `self`.
    pub fn grid_disk_compact(self, k: usize) -> Result<Vec<Self>, CellError> {
        a5::grid_disk(self.0, k)
            .map(wrap_all)
            .map_err(CellError::A5)
    }

    /// All cells within `radius_meters` of this cell's centre, at `self`'s
    /// resolution. Compacted for the same reason as [`GeoCell::grid_disk`].
    pub fn spherical_cap(self, radius_meters: f64) -> Result<Vec<Self>, CellError> {
        let cover = a5::spherical_cap(self.0, radius_meters).map_err(CellError::A5)?;
        expand_to(cover, self.resolution())
    }

    /// Boundary ring as `(longitude, latitude)` pairs, for drawing.
    pub fn boundary(self) -> Result<Vec<(f64, f64)>, CellError> {
        let ring = a5::cell_to_boundary(self.0, None).map_err(CellError::A5)?;
        Ok(ring
            .into_iter()
            .map(|ll| (ll.longitude(), ll.latitude()))
            .collect())
    }

    /// True when `self` is `ancestor` or lies inside it.
    ///
    /// This is the admission check for island runs: an intent addressed outside
    /// the configured root is rejected before it ever reaches the driver.
    pub fn is_within(self, ancestor: GeoCell) -> bool {
        if ancestor == GeoCell::WORLD || ancestor == self {
            return true;
        }
        let ancestor_res = ancestor.resolution();
        if self.resolution() < ancestor_res {
            return false;
        }
        self.parent(Some(ancestor_res)).is_ok_and(|p| p == ancestor)
    }

    /// Zero-padded lowercase hex. Padding keeps ids the same width so they
    /// line up in the dashboard and sort lexicographically like their `u64`.
    pub fn to_hex(self) -> String {
        format!("{:016x}", self.0)
    }

    /// Accepts `"0x1f.."`, bare hex, or a decimal `u64`.
    ///
    /// Decimal is accepted because the spec allows either on the wire, but a
    /// bare 16-digit string is ambiguous, so hex wins when both parse.
    pub fn parse(s: &str) -> Result<Self, CellError> {
        let s = s.trim();
        if let Some(hex) = s.strip_prefix("0x").or_else(|| s.strip_prefix("0X")) {
            return a5::hex_to_u64(hex).map(GeoCell).map_err(CellError::A5);
        }
        if let Ok(v) = u64::from_str_radix(s, 16) {
            return Ok(GeoCell(v));
        }
        s.parse::<u64>()
            .map(GeoCell)
            .map_err(|_| CellError::BadCellId(s.to_owned()))
    }
}

/// Cells at `resolution` whose boundary overlaps the bounding box.
///
/// A convenience only: storage and subscriptions are cell sets (spec §7.2).
/// The walk descends the A5 hierarchy from resolution 0, so cost scales with
/// the *answer*, not with the resolution — but a wide box at a fine resolution
/// is still unbounded, hence `limit`.
///
/// Two documented approximations: the overlap test uses each cell's
/// axis-aligned vertex bounds, so a few cells just outside the box may be
/// returned; and boxes crossing the antimeridian are rejected rather than
/// split.
pub fn cells_in_bbox(
    min_lon: f64,
    min_lat: f64,
    max_lon: f64,
    max_lat: f64,
    resolution: i32,
    limit: usize,
) -> Result<Vec<GeoCell>, CellError> {
    check_resolution(resolution)?;
    if !(min_lon < max_lon && min_lat < max_lat) {
        return Err(CellError::BadBbox);
    }

    let mut out = Vec::new();
    let mut frontier: Vec<GeoCell> = a5::get_res0_cells().map(wrap_all).map_err(CellError::A5)?;

    while let Some(cell) = frontier.pop() {
        if !cell_overlaps(cell, min_lon, min_lat, max_lon, max_lat)? {
            continue;
        }
        if cell.resolution() >= resolution {
            out.push(cell);
            if out.len() > limit {
                return Err(CellError::TooManyCells { limit });
            }
            continue;
        }
        frontier.extend(cell.children(None)?);
    }

    out.sort_unstable();
    out.dedup();
    Ok(out)
}

/// Nominal edge length of a cell at `resolution`, in metres. Used by the
/// dashboard to pick a display resolution for the current viewport.
pub fn nominal_cell_edge_meters(resolution: i32) -> f64 {
    a5::cell_area(resolution).sqrt()
}

fn cell_overlaps(
    cell: GeoCell,
    min_lon: f64,
    min_lat: f64,
    max_lon: f64,
    max_lat: f64,
) -> Result<bool, CellError> {
    let ring = cell.boundary()?;
    if ring.is_empty() {
        // WORLD_CELL is unbounded and reports an empty ring.
        return Ok(true);
    }
    let (mut lo_lon, mut hi_lon) = (f64::MAX, f64::MIN);
    let (mut lo_lat, mut hi_lat) = (f64::MAX, f64::MIN);
    for (lon, lat) in ring {
        lo_lon = lo_lon.min(lon);
        hi_lon = hi_lon.max(lon);
        lo_lat = lo_lat.min(lat);
        hi_lat = hi_lat.max(lat);
    }
    Ok(lo_lon <= max_lon && hi_lon >= min_lon && lo_lat <= max_lat && hi_lat >= min_lat)
}

fn check_resolution(resolution: i32) -> Result<(), CellError> {
    if (0..=MAX_RESOLUTION).contains(&resolution) {
        Ok(())
    } else {
        Err(CellError::BadResolution(resolution))
    }
}

fn wrap_all(raw: Vec<u64>) -> Vec<GeoCell> {
    raw.into_iter().map(GeoCell).collect()
}

/// Expand a compacted cover so every cell sits at `resolution`.
fn expand_to(cover: Vec<u64>, resolution: i32) -> Result<Vec<GeoCell>, CellError> {
    let mut cells = a5::uncompact(&cover, resolution).map_err(CellError::A5)?;
    cells.sort_unstable();
    cells.dedup();
    Ok(wrap_all(cells))
}

impl fmt::Display for GeoCell {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(&self.to_hex())
    }
}

impl From<u64> for GeoCell {
    fn from(raw: u64) -> Self {
        GeoCell(raw)
    }
}

impl From<GeoCell> for u64 {
    fn from(cell: GeoCell) -> Self {
        cell.0
    }
}

/// Cells travel as hex strings. JSON numbers are IEEE doubles in every browser,
/// which would silently corrupt indices above 2^53.
impl Serialize for GeoCell {
    fn serialize<S: Serializer>(&self, s: S) -> Result<S::Ok, S::Error> {
        s.serialize_str(&self.to_hex())
    }
}

impl<'de> Deserialize<'de> for GeoCell {
    fn deserialize<D: Deserializer<'de>>(d: D) -> Result<Self, D::Error> {
        #[derive(Deserialize)]
        #[serde(untagged)]
        enum Repr {
            Text(String),
            Raw(u64),
        }
        match Repr::deserialize(d)? {
            Repr::Text(s) => GeoCell::parse(&s).map_err(D::Error::custom),
            Repr::Raw(v) => Ok(GeoCell(v)),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const LONDON: (f64, f64) = (-0.1276, 51.5072);

    #[test]
    fn lonlat_round_trips_through_the_cell_centroid() {
        let cell = GeoCell::from_lonlat(LONDON.0, LONDON.1, 9).unwrap();
        let (lon, lat) = cell.centroid().unwrap();
        assert_eq!(GeoCell::from_lonlat(lon, lat, 9).unwrap(), cell);
        assert_eq!(cell.resolution(), 9);
    }

    #[test]
    fn hex_is_the_wire_format_and_survives_json() {
        let cell = GeoCell::from_lonlat(LONDON.0, LONDON.1, 12).unwrap();
        let json = serde_json::to_string(&cell).unwrap();
        assert!(json.starts_with('"'), "cells must not be JSON numbers");
        assert_eq!(serde_json::from_str::<GeoCell>(&json).unwrap(), cell);
        assert_eq!(
            GeoCell::parse(&format!("0x{}", cell.to_hex())).unwrap(),
            cell
        );
    }

    #[test]
    fn containment_follows_the_hierarchy() {
        let fine = GeoCell::from_lonlat(LONDON.0, LONDON.1, 10).unwrap();
        let coarse = fine.parent(Some(4)).unwrap();
        let elsewhere = GeoCell::from_lonlat(151.2093, -33.8688, 10).unwrap();

        assert!(fine.is_within(coarse));
        assert!(fine.is_within(GeoCell::WORLD));
        assert!(!elsewhere.is_within(coarse));
        assert!(!coarse.is_within(fine), "a parent is not inside its child");
    }

    #[test]
    fn grid_disk_grows_and_contains_its_centre() {
        let centre = GeoCell::from_lonlat(LONDON.0, LONDON.1, 8).unwrap();
        let k1 = centre.grid_disk(1).unwrap();
        let k2 = centre.grid_disk(2).unwrap();
        assert!(k1.contains(&centre));
        assert!(k2.len() > k1.len());
        assert!(!centre.neighbors().unwrap().contains(&centre));
    }

    #[test]
    fn a_disk_is_uniform_resolution_even_though_a5_compacts_it() {
        let centre = GeoCell::from_lonlat(LONDON.0, LONDON.1, 8).unwrap();
        let disk = centre.grid_disk(3).unwrap();
        assert!(disk.iter().all(|c| c.resolution() == 8));
        assert!(
            disk.len() >= centre.grid_disk_compact(3).unwrap().len(),
            "expanding a compacted cover can only add cells"
        );
    }

    #[test]
    fn bbox_compiles_to_cells_that_actually_sit_in_the_box() {
        let cells = cells_in_bbox(-0.3, 51.3, 0.1, 51.7, 6, 10_000).unwrap();
        assert!(!cells.is_empty());
        assert!(cells.contains(&GeoCell::from_lonlat(LONDON.0, LONDON.1, 6).unwrap()));
        for c in &cells {
            assert_eq!(c.resolution(), 6);
        }
    }

    #[test]
    fn bbox_refuses_to_blow_up() {
        assert!(matches!(
            cells_in_bbox(-180.0, -85.0, 180.0, 85.0, 12, 1_000),
            Err(CellError::TooManyCells { .. })
        ));
        assert!(matches!(
            cells_in_bbox(1.0, 1.0, 0.0, 0.0, 6, 10),
            Err(CellError::BadBbox)
        ));
        assert!(matches!(
            GeoCell::from_lonlat(0.0, 0.0, 99),
            Err(CellError::BadResolution(99))
        ));
    }
}
