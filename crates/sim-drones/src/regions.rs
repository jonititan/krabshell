//! Regions of interest: places the drones the simulation flies go and look
//! at with their cameras.
//!
//! A [`Region`] is a circle. A pack lists those it starts with in
//! `world/regions.json`, each `{"name", "lat", "lon", "radius_m"}`; while it
//! runs, intents designate and remove them (see [`crate::codes`]), and every
//! drone that crashes leaves one where it fell, so someone goes to look.
//!
//! A drone free for work (on its pad, charged, and for a courier, with no
//! order it can take) surveys the region most overdue a look that it can
//! reach: one nobody has surveyed yet, oldest first, then the one surveyed
//! longest ago, once `roi_revisit_min` have passed. One drone at a time
//! goes to each. It flies there, circles inside it at its survey height
//! with the camera looking, and comes home; the region is surveyed when the
//! circle closes.

use serde::{Deserialize, Serialize};
use sim_core::AgentId;

use crate::flight::{self, Point};
use crate::mission::{MissionItem, NAV_WAYPOINT};

const FRAME_TERRAIN: u8 = 11;

/// Corners of the circle a survey flies.
const SURVEY_SIDES: usize = 8;

/// A place to go and look at.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Region {
    pub id: u32,
    #[serde(default, skip_serializing_if = "String::is_empty")]
    pub name: String,
    pub lat_e7: i32,
    pub lon_e7: i32,
    pub radius_m: f64,
    /// Who designated it: `scenario`, `dashboard`, `intent`, or `crash` for
    /// one left where a drone fell.
    pub source: String,
    pub added_tick: u64,
    /// Surveys finished.
    #[serde(default)]
    pub surveys: u32,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub last_surveyed_tick: Option<u64>,
    /// Drones on their way to survey it.
    #[serde(default)]
    pub assigned: Vec<AgentId>,
}

impl Region {
    pub fn centre(&self) -> Point {
        Point {
            lat_e7: self.lat_e7,
            lon_e7: self.lon_e7,
            alt_mm: 0,
        }
    }

    /// Due a look at `tick`: nobody on the way, and never surveyed or not
    /// for `revisit_ticks`.
    pub fn due(&self, tick: u64, revisit_ticks: u64) -> bool {
        self.assigned.is_empty()
            && self
                .last_surveyed_tick
                .is_none_or(|t| tick.saturating_sub(t) >= revisit_ticks)
    }

    /// How overdue, for picking the most overdue first: never surveyed
    /// before surveyed, then the longest waiting, then the lowest id.
    fn urgency(&self) -> (bool, u64, u32) {
        match self.last_surveyed_tick {
            None => (false, self.added_tick, self.id),
            Some(t) => (true, t, self.id),
        }
    }

    /// The corners of the circle a survey flies round, inside the region.
    pub fn circuit(&self) -> Vec<Point> {
        let r = (self.radius_m * 0.6).clamp(10.0, self.radius_m.max(10.0));
        (0..=SURVEY_SIDES)
            .map(|i| {
                let a = i as f64 / SURVEY_SIDES as f64 * core::f64::consts::TAU;
                let (lat, lon) =
                    flight::displace(self.lat_e7, self.lon_e7, r * libm::sin(a), r * libm::cos(a));
                Point {
                    lat_e7: lat,
                    lon_e7: lon,
                    alt_mm: 0,
                }
            })
            .collect()
    }

    /// Metres round the circuit.
    pub fn circuit_m(&self) -> f64 {
        let c = self.circuit();
        c.windows(2)
            .map(|w| flight::ground_distance_m(&w[0], &w[1]))
            .sum()
    }
}

/// Every region of interest in a world. Checkpointed through the model's
/// fields.
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct Regions {
    pub list: Vec<Region>,
    pub next_id: u32,
}

impl Regions {
    pub fn is_empty(&self) -> bool {
        self.list.is_empty()
    }

    pub fn take_id(&mut self) -> u32 {
        self.next_id = self.next_id.max(1);
        let id = self.next_id;
        self.next_id += 1;
        id
    }

    pub fn add(&mut self, region: Region) {
        self.list.push(region);
        self.list.sort_by_key(|r| r.id);
    }

    pub fn remove(&mut self, id: u32) -> Option<Region> {
        let at = self.list.iter().position(|r| r.id == id)?;
        Some(self.list.remove(at))
    }

    pub fn get_mut(&mut self, id: u32) -> Option<&mut Region> {
        self.list.iter_mut().find(|r| r.id == id)
    }

    /// Regions due a look, most overdue first.
    pub fn due(&self, tick: u64, revisit_ticks: u64) -> Vec<&Region> {
        let mut due: Vec<&Region> = self
            .list
            .iter()
            .filter(|r| r.due(tick, revisit_ticks))
            .collect();
        due.sort_by_key(|r| r.urgency());
        due
    }

    /// The drone is no longer on its way to any region.
    pub fn release(&mut self, drone: AgentId) {
        for r in &mut self.list {
            r.assigned.retain(|d| *d != drone);
        }
    }
}

/// A survey a drone is flying. Lives in the drone's published state.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Survey {
    pub roi: u32,
    /// The mission item that closes the circle.
    pub last_item: u16,
    pub started_tick: u64,
    /// The circle has been flown.
    #[serde(default)]
    pub done: bool,
}

/// Mission items for flying the circuit: to its first corner and round.
pub fn circuit_items(region: &Region, agl_m: f32) -> Vec<MissionItem> {
    region
        .circuit()
        .iter()
        .map(|p| MissionItem {
            seq: 0,
            frame: FRAME_TERRAIN,
            command: NAV_WAYPOINT,
            autocontinue: 1,
            param1: 0.0,
            param2: 5.0,
            param3: 0.0,
            param4: f32::NAN,
            x: p.lat_e7,
            y: p.lon_e7,
            z: agl_m,
        })
        .collect()
}
