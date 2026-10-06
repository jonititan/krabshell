//! Delivery orders, and dispatching drones to them.
//!
//! A pack lists the addresses that can order in `world/fields.json` under
//! `"addresses"`, each tied to the depot (launch and recovery site) that
//! serves it:
//!
//! ```json
//! { "addresses": {
//!     "depots": ["MEGA", "AKRO"],
//!     "items": [[lat_e7, lon_e7, depot_index, "label"], ...]
//! } }
//! ```
//!
//! Every address orders at random, independently and rarely
//! (`order_rate_per_address_day`), which together is one Poisson stream at
//! the summed rate with each order going to an address chosen uniformly. The
//! stream is drawn from the run's seed, so `gold` runs place the same orders
//! at the same ticks.
//!
//! Orders wait in a first-come first-served queue at their depot. A drone
//! with a `depot` (a courier) takes the oldest order there once it is on its
//! home pad, disarmed, loaded (`dispatch_load_s` after landing) and charged
//! to the launch level. It gets a fresh mission for that one order: take off,
//! fly to the address at its cruise height above the terrain, lower the
//! parcel with DO_WINCH / WINCH_DELIVER, fly home and land. The order counts
//! as delivered when the winch item completes. A courier that lands without
//! delivering (sent home on low battery, or taken over by a ground station)
//! puts its order back at the front of the queue.

use std::collections::VecDeque;

use serde::{Deserialize, Serialize};

use crate::flight::Point;
use crate::mission::{MissionItem, DO_WINCH, NAV_LAND, NAV_TAKEOFF, NAV_WAYPOINT, WINCH_DELIVER};

const FRAME_MISSION: u8 = 2;
const FRAME_RELATIVE: u8 = 3;
const FRAME_TERRAIN: u8 = 11;

/// One place that can order.
#[derive(Debug, Clone, PartialEq)]
pub struct Address {
    pub lat_e7: i32,
    pub lon_e7: i32,
    pub depot: u16,
    pub label: String,
}

/// Every address that can order, and the depots that serve them.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct AddressBook {
    pub depots: Vec<String>,
    pub addresses: Vec<Address>,
}

#[derive(Deserialize)]
struct Book {
    depots: Vec<String>,
    items: Vec<(i32, i32, u16, String)>,
}

impl AddressBook {
    /// Read the `"addresses"` entry of a pack's fields, or an empty book.
    pub fn from_fields(fields: &serde_json::Value) -> anyhow::Result<AddressBook> {
        let Some(value) = fields.get("addresses") else {
            return Ok(AddressBook::default());
        };
        let book = Book::deserialize(value)
            .map_err(|e| anyhow::anyhow!("world/fields.json addresses: {e}"))?;
        if let Some((i, bad)) = book
            .items
            .iter()
            .enumerate()
            .find(|(_, item)| usize::from(item.2) >= book.depots.len())
        {
            anyhow::bail!(
                "world/fields.json addresses: item {i} names depot {} of {}",
                bad.2,
                book.depots.len()
            );
        }
        Ok(AddressBook {
            depots: book.depots,
            addresses: book
                .items
                .into_iter()
                .map(|(lat_e7, lon_e7, depot, label)| Address {
                    lat_e7,
                    lon_e7,
                    depot,
                    label,
                })
                .collect(),
        })
    }

    pub fn depot_index(&self, name: &str) -> Option<u16> {
        self.depots.iter().position(|d| d == name).map(|i| i as u16)
    }
}

/// An order placed and not yet delivered.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Order {
    pub id: u64,
    pub address: u32,
    pub ordered_tick: u64,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub dispatched_tick: Option<u64>,
}

/// A drone that delivers from a depot. Lives in the drone's published state.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Courier {
    pub depot: u16,
    pub cruise_agl_m: f32,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub order: Option<Order>,
    #[serde(default)]
    pub delivered: bool,
    /// When it last landed, so loading time can be counted from it. None
    /// means loaded and ready from the start.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub landed_tick: Option<u64>,
}

/// The order stream and the depot queues. Checkpointed through the model's
/// fields.
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct Dispatch {
    pub queues: Vec<VecDeque<Order>>,
    /// Orders placed so far; also the last order's id (ids start at 1).
    pub placed: u64,
    pub delivered: u64,
    /// Orders turned away when placed (see [`Dispatch::place_orders`]).
    #[serde(default)]
    pub rejected: u64,
    /// When the next order comes. None until the first draw.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub next_order_tick: Option<u64>,
}

/// Mission heights and timings from `params.toml`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct DispatchSettings {
    /// Orders per tick across the whole book. Zero means no orders.
    pub orders_per_tick: f64,
    pub takeoff_m: f32,
    pub approach_m: f32,
    pub load_ticks: u64,
}

impl Dispatch {
    pub fn new(depots: usize) -> Self {
        Dispatch {
            queues: vec![VecDeque::new(); depots],
            ..Dispatch::default()
        }
    }

    pub fn queued(&self) -> usize {
        self.queues.iter().map(VecDeque::len).sum()
    }

    /// Place every order due at or before `tick`. `draw(n)` gives two
    /// uniform numbers for order `n`: the gap after it, and which address
    /// places it. Draw 0 only sets when the first order comes. An order
    /// `accept` turns down (by address index) joins no queue; those are
    /// returned. The draws are the same either way, so turning orders away
    /// changes no other order.
    pub fn place_orders(
        &mut self,
        tick: u64,
        book: &AddressBook,
        settings: &DispatchSettings,
        draw: impl Fn(u64) -> (f64, f64),
        mut accept: impl FnMut(u32) -> bool,
    ) -> Vec<Order> {
        let mut rejected = Vec::new();
        if settings.orders_per_tick <= 0.0 || book.addresses.is_empty() {
            return rejected;
        }
        let gap =
            |u: f64| libm::round(-libm::log(1.0 - u) / settings.orders_per_tick).max(0.0) as u64;
        let mut next = self
            .next_order_tick
            .unwrap_or_else(|| tick + gap(draw(0).0));
        while next <= tick {
            self.placed += 1;
            let (after, pick) = draw(self.placed);
            let address =
                ((pick * book.addresses.len() as f64) as usize).min(book.addresses.len() - 1);
            let order = Order {
                id: self.placed,
                address: address as u32,
                ordered_tick: next,
                dispatched_tick: None,
            };
            if accept(order.address) {
                self.queues[usize::from(book.addresses[address].depot)].push_back(order);
            } else {
                self.rejected += 1;
                rejected.push(order);
            }
            next += gap(after);
        }
        self.next_order_tick = Some(next);
        rejected
    }
}

fn item(seq: usize, command: u16, frame: u8) -> MissionItem {
    MissionItem {
        seq: seq as u16,
        frame,
        command,
        autocontinue: 1,
        param1: 0.0,
        param2: 0.0,
        param3: 0.0,
        param4: f32::NAN,
        x: 0,
        y: 0,
        z: 0.0,
    }
}

/// The mission for one delivery from `home` to `to`.
pub fn delivery_mission(
    home: &Point,
    to: &Address,
    cruise_agl_m: f32,
    settings: &DispatchSettings,
) -> Vec<MissionItem> {
    let mut items = Vec::with_capacity(6);
    let mut takeoff = item(0, NAV_TAKEOFF, FRAME_RELATIVE);
    takeoff.z = settings.takeoff_m;
    items.push(takeoff);

    let mut out = item(1, NAV_WAYPOINT, FRAME_TERRAIN);
    (out.x, out.y, out.z) = (to.lat_e7, to.lon_e7, cruise_agl_m);
    items.push(out);

    let mut winch = item(2, DO_WINCH, FRAME_MISSION);
    winch.param1 = 1.0;
    winch.param2 = f32::from(WINCH_DELIVER);
    winch.param4 = 0.0;
    items.push(winch);

    let mut back = item(3, NAV_WAYPOINT, FRAME_TERRAIN);
    (back.x, back.y, back.z) = (home.lat_e7, home.lon_e7, cruise_agl_m);
    back.param2 = 5.0;
    items.push(back);

    let mut approach = item(4, NAV_WAYPOINT, FRAME_RELATIVE);
    (approach.x, approach.y, approach.z) = (home.lat_e7, home.lon_e7, settings.approach_m);
    items.push(approach);

    let mut land = item(5, NAV_LAND, FRAME_RELATIVE);
    (land.x, land.y) = (home.lat_e7, home.lon_e7);
    items.push(land);
    items
}

#[cfg(test)]
mod tests {
    use super::*;

    fn book(n: usize) -> AddressBook {
        AddressBook {
            depots: vec!["A".into(), "B".into()],
            addresses: (0..n)
                .map(|i| Address {
                    lat_e7: 549_000_000 + i as i32,
                    lon_e7: 239_000_000,
                    depot: (i % 2) as u16,
                    label: format!("Street {i}"),
                })
                .collect(),
        }
    }

    #[test]
    fn orders_arrive_at_the_configured_rate_and_split_by_depot() {
        let book = book(1_000);
        let settings = DispatchSettings {
            orders_per_tick: 0.01,
            takeoff_m: 30.0,
            approach_m: 15.0,
            load_ticks: 0,
        };
        let mut d = Dispatch::new(2);
        let mut rng = sim_core::rng::Rng::new(3);
        let draws: Vec<(f64, f64)> = (0..10_000).map(|_| (rng.unit(), rng.unit())).collect();
        // The first call schedules the first order from tick 0.
        d.place_orders(0, &book, &settings, |n| draws[n as usize], |_| true);
        // Every tenth address turned away: the rest are placed as before.
        let rejected = d.place_orders(
            100_000,
            &book,
            &settings,
            |n| draws[n as usize],
            |a| a % 10 != 0,
        );
        assert!(!rejected.is_empty() && rejected.iter().all(|o| o.address % 10 == 0));
        assert_eq!(d.rejected as usize, rejected.len());
        assert_eq!(d.placed as usize, d.queued() + rejected.len());
        // About 1000 expected; Poisson spread is about ±32.
        let placed = d.placed as f64;
        assert!((850.0..1150.0).contains(&placed), "{placed}");
        assert!(d.queues[0].len() > 300 && d.queues[1].len() > 300);
        // Each queue is oldest first.
        for q in &d.queues {
            assert!(q
                .iter()
                .zip(q.iter().skip(1))
                .all(|(a, b)| a.ordered_tick <= b.ordered_tick));
        }
    }

    #[test]
    fn a_delivery_mission_takes_off_winches_and_lands_at_home() {
        let home = Point::at(23.89, 54.94, 81.0);
        let to = &book(1).addresses[0];
        let settings = DispatchSettings {
            orders_per_tick: 0.0,
            takeoff_m: 30.0,
            approach_m: 15.0,
            load_ticks: 0,
        };
        let m = delivery_mission(&home, to, 70.0, &settings);
        let commands: Vec<u16> = m.iter().map(|i| i.command).collect();
        assert_eq!(
            commands,
            [
                NAV_TAKEOFF,
                NAV_WAYPOINT,
                DO_WINCH,
                NAV_WAYPOINT,
                NAV_WAYPOINT,
                NAV_LAND
            ]
        );
        for (i, item) in m.iter().enumerate() {
            item.check(i).unwrap();
        }
        assert_eq!((m[1].x, m[1].y), (to.lat_e7, to.lon_e7));
        assert_eq!((m[5].x, m[5].y), (home.lat_e7, home.lon_e7));
    }

    #[test]
    fn an_address_book_reads_from_fields_and_refuses_unknown_depots() {
        let ok = serde_json::json!({ "addresses": {
            "depots": ["MEGA", "AKRO"],
            "items": [[549000000, 239000000, 1, "Lankų g. 7"]],
        }});
        let book = AddressBook::from_fields(&ok).unwrap();
        assert_eq!(book.addresses[0].depot, 1);
        assert_eq!(book.depot_index("AKRO"), Some(1));
        let bad = serde_json::json!({ "addresses": {
            "depots": ["MEGA"], "items": [[0, 0, 3, "x"]],
        }});
        assert!(AddressBook::from_fields(&bad).is_err());
        assert!(AddressBook::from_fields(&serde_json::Value::Null)
            .unwrap()
            .addresses
            .is_empty());
    }
}
