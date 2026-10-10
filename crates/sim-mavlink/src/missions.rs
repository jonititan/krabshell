//! The mission upload handshake and conversion between wire and model items.
//!
//! An upload is `MISSION_COUNT` from the ground station, then one
//! `MISSION_REQUEST_INT` from us and one `MISSION_ITEM_INT` back per item, then
//! `MISSION_ACK`. The gateway holds the partial mission here until the last item
//! arrives, and only then hands the whole thing to the simulation as one intent:
//! the drone never flies half a plan.
//!
//! A fence plan (`MAV_MISSION_TYPE_FENCE`) uploads the same way, and becomes
//! the geofences ground stations have set (see [`fence_shapes`]).

use std::time::{Duration, Instant};

use mavlink::dialects::common::{MavCmd, MavFrame, MavMissionType, MISSION_ITEM_INT_DATA};
use num_traits::FromPrimitive;
use sim_engine::models::drones::airspace::{Fence, Shape};
use sim_engine::models::drones::mission::MissionItem;

/// How long to wait for the next item before asking again (MAVLink default).
pub const ITEM_TIMEOUT: Duration = Duration::from_millis(1500);
/// Requests for one item before giving up on the whole upload.
pub const MAX_RETRIES: u8 = 5;
/// Anything larger is refused before the handshake starts.
pub const MAX_ITEMS: u16 = 4096;

/// An upload in progress, from one ground station to one drone.
#[derive(Debug)]
pub struct Upload {
    pub count: u16,
    pub items: Vec<MissionItem>,
    /// A mission, or a fence plan.
    pub mission_type: MavMissionType,
    /// The ground station's own system and component ids, for replies.
    pub gcs: (u8, u8),
    pub last_io: Instant,
    pub retries: u8,
}

impl Upload {
    pub fn new(count: u16, mission_type: MavMissionType, gcs: (u8, u8), now: Instant) -> Self {
        Upload {
            count,
            items: Vec::with_capacity(usize::from(count)),
            mission_type,
            gcs,
            last_io: now,
            retries: 0,
        }
    }

    /// The seq we are waiting for.
    pub fn next(&self) -> u16 {
        self.items.len() as u16
    }

    pub fn complete(&self) -> bool {
        self.items.len() == usize::from(self.count)
    }

    /// Take an item. Returns false for anything but the one we asked for; the
    /// caller re-requests, which is how MAVLink recovers from a lost packet.
    pub fn accept(&mut self, item: &MISSION_ITEM_INT_DATA, now: Instant) -> bool {
        if item.seq != self.next() || self.complete() {
            return false;
        }
        self.items.push(from_wire(item));
        self.last_io = now;
        self.retries = 0;
        true
    }

    /// Whether it is time to ask again, and whether we have asked too often.
    pub fn overdue(&self, now: Instant) -> bool {
        now.duration_since(self.last_io) >= ITEM_TIMEOUT
    }
}

pub fn from_wire(item: &MISSION_ITEM_INT_DATA) -> MissionItem {
    MissionItem {
        seq: item.seq,
        frame: item.frame as u8,
        command: item.command as u16,
        autocontinue: item.autocontinue,
        param1: item.param1,
        param2: item.param2,
        param3: item.param3,
        param4: item.param4,
        x: item.x,
        y: item.y,
        z: item.z,
    }
}

/// The wire form of a stored item, or `None` if the model holds a command or
/// frame number this dialect does not know (it cannot be sent back).
pub fn to_wire(item: &MissionItem, current: bool, gcs: (u8, u8)) -> Option<MISSION_ITEM_INT_DATA> {
    Some(MISSION_ITEM_INT_DATA {
        param1: item.param1,
        param2: item.param2,
        param3: item.param3,
        param4: item.param4,
        x: item.x,
        y: item.y,
        z: item.z,
        seq: item.seq,
        command: MavCmd::from_u16(item.command)?,
        target_system: gcs.0,
        target_component: gcs.1,
        frame: MavFrame::from_u8(item.frame)?,
        current: u8::from(current),
        autocontinue: item.autocontinue,
        mission_type: MavMissionType::MAV_MISSION_TYPE_MISSION,
    })
}

/// An uploaded fence plan as the geofence shapes the simulation takes (see
/// `codes::GEOFENCE_SET`): a circle per `NAV_FENCE_CIRCLE_EXCLUSION`, a
/// polygon per run of `NAV_FENCE_POLYGON_VERTEX_EXCLUSION` items (param1 is
/// the corner count). A return point is ignored. Inclusion fences are not
/// simulated, so a plan with one is refused rather than half applied.
pub fn fence_shapes(items: &[MissionItem]) -> Result<Vec<serde_json::Value>, String> {
    const RETURN_POINT: u16 = 5000;
    const POLYGON_INCLUSION: u16 = 5001;
    const POLYGON_EXCLUSION: u16 = 5002;
    const CIRCLE_INCLUSION: u16 = 5003;
    const CIRCLE_EXCLUSION: u16 = 5004;
    let deg = |e7: i32| f64::from(e7) * 1e-7;
    let mut out = Vec::new();
    let mut i = 0;
    while i < items.len() {
        let item = &items[i];
        match item.command {
            CIRCLE_EXCLUSION => {
                out.push(serde_json::json!({ "circle": {
                    "lat": deg(item.x), "lon": deg(item.y), "radius_m": f64::from(item.param1),
                }}));
                i += 1;
            }
            POLYGON_EXCLUSION => {
                let n = item.param1 as usize;
                if n < 3 || i + n > items.len() {
                    return Err(format!(
                        "fence item {i}: a polygon needs at least 3 corners and all of them"
                    ));
                }
                let corners = &items[i..i + n];
                if corners
                    .iter()
                    .any(|c| c.command != POLYGON_EXCLUSION || c.param1 as usize != n)
                {
                    return Err(format!(
                        "fence item {i}: the polygon's {n} corners do not follow one another"
                    ));
                }
                let points: Vec<[f64; 2]> = corners.iter().map(|c| [deg(c.y), deg(c.x)]).collect();
                out.push(serde_json::json!({ "polygon": points }));
                i += n;
            }
            RETURN_POINT => i += 1,
            POLYGON_INCLUSION | CIRCLE_INCLUSION => {
                return Err(
                    "inclusion fences are not simulated: only keep-out (exclusion) zones".into(),
                )
            }
            other => return Err(format!("fence item {i}: command {other} is not a fence")),
        }
    }
    Ok(out)
}

/// The geofences as a fence plan (`MAV_MISSION_TYPE_FENCE`), the way
/// ArduPilot and PX4 hold one: a `NAV_FENCE_CIRCLE_EXCLUSION` per circle,
/// and a `NAV_FENCE_POLYGON_VERTEX_EXCLUSION` per corner of each polygon,
/// each carrying the polygon's corner count.
pub fn fence_plan(fences: &[Fence], gcs: (u8, u8)) -> Vec<MISSION_ITEM_INT_DATA> {
    let item = |command, param1: f32, x, y| MISSION_ITEM_INT_DATA {
        param1,
        param2: 0.0,
        param3: 0.0,
        param4: 0.0,
        x,
        y,
        z: 0.0,
        seq: 0,
        command,
        target_system: gcs.0,
        target_component: gcs.1,
        frame: MavFrame::MAV_FRAME_GLOBAL,
        current: 0,
        autocontinue: 0,
        mission_type: MavMissionType::MAV_MISSION_TYPE_FENCE,
    };
    let mut out = Vec::new();
    for fence in fences {
        match &fence.shape {
            Shape::Circle {
                lat_e7,
                lon_e7,
                radius_m,
            } => out.push(item(
                MavCmd::MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION,
                *radius_m as f32,
                *lat_e7,
                *lon_e7,
            )),
            Shape::Polygon { points } => {
                for p in points {
                    out.push(item(
                        MavCmd::MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION,
                        points.len() as f32,
                        p[0],
                        p[1],
                    ));
                }
            }
        }
    }
    for (i, item) in out.iter_mut().enumerate() {
        item.seq = i as u16;
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn an_uploaded_fence_plan_reads_back_as_the_shapes_it_was_made_from() {
        let circle = Fence {
            id: 1,
            shape: Shape::Circle {
                lat_e7: 549_000_000,
                lon_e7: 239_000_000,
                radius_m: 150.0,
            },
            band: Default::default(),
            name: String::new(),
            source: "scenario".into(),
            opened_tick: 0,
            closes_tick: None,
        };
        let triangle = Fence {
            id: 2,
            shape: Shape::Polygon {
                points: vec![
                    [549_000_000, 239_000_000],
                    [549_010_000, 239_000_000],
                    [549_000_000, 239_010_000],
                ],
            },
            ..circle.clone()
        };
        let items: Vec<MissionItem> = fence_plan(&[circle, triangle], (255, 190))
            .iter()
            .map(from_wire)
            .collect();
        let shapes = fence_shapes(&items).unwrap();
        assert_eq!(shapes.len(), 2);
        assert_eq!(shapes[0]["circle"]["radius_m"], 150.0);
        assert_eq!(shapes[1]["polygon"].as_array().unwrap().len(), 3);
        assert!((shapes[1]["polygon"][1][1].as_f64().unwrap() - 54.901).abs() < 1e-9);

        let mut inclusion = items.clone();
        inclusion[0].command = 5003;
        assert!(fence_shapes(&inclusion).unwrap_err().contains("inclusion"));
        assert!(fence_shapes(&items[..2]).is_err(), "a polygon cut short");
    }

    #[test]
    fn a_fence_plan_has_one_item_per_circle_and_per_polygon_corner() {
        let circle = Fence {
            id: 1,
            shape: Shape::Circle {
                lat_e7: 549_000_000,
                lon_e7: 239_000_000,
                radius_m: 150.0,
            },
            band: Default::default(),
            name: String::new(),
            source: "scenario".into(),
            opened_tick: 0,
            closes_tick: None,
        };
        let square = Fence {
            id: 2,
            shape: Shape::Polygon {
                points: vec![[1, 1], [1, 2], [2, 2], [2, 1]],
            },
            ..circle.clone()
        };
        let plan = fence_plan(&[circle, square], (255, 190));
        assert_eq!(plan.len(), 5);
        assert_eq!(plan[0].command, MavCmd::MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION);
        assert_eq!(plan[0].param1, 150.0);
        assert!(plan[1..].iter().all(|i| i.command
            == MavCmd::MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION
            && i.param1 == 4.0
            && i.mission_type == MavMissionType::MAV_MISSION_TYPE_FENCE));
        assert_eq!(
            plan.iter().map(|i| i.seq).collect::<Vec<_>>(),
            [0, 1, 2, 3, 4]
        );
    }

    #[allow(deprecated)]
    fn wire(seq: u16) -> MISSION_ITEM_INT_DATA {
        MISSION_ITEM_INT_DATA {
            param1: 0.0,
            param2: 2.0,
            param3: 0.0,
            param4: f32::NAN,
            x: 515_000_000,
            y: -1_200_000,
            z: 30.0,
            seq,
            command: MavCmd::MAV_CMD_NAV_WAYPOINT,
            target_system: 1,
            target_component: 1,
            frame: MavFrame::MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            current: 0,
            autocontinue: 1,
            mission_type: MavMissionType::MAV_MISSION_TYPE_MISSION,
        }
    }

    #[test]
    fn items_round_trip_through_the_model_form() {
        let original = wire(4);
        let stored = from_wire(&original);
        assert_eq!(stored.command, 16);
        assert_eq!(stored.frame, 6);
        let back = to_wire(&stored, true, (255, 190)).unwrap();
        assert_eq!(back.command, original.command);
        assert_eq!(back.frame, original.frame);
        assert_eq!(back.x, original.x);
        assert!(back.param4.is_nan());
        assert_eq!(back.current, 1);
        assert_eq!(back.target_system, 255);
    }

    #[test]
    fn an_upload_takes_items_in_order_only() {
        let now = Instant::now();
        let mut upload = Upload::new(2, MavMissionType::MAV_MISSION_TYPE_MISSION, (255, 190), now);
        assert!(!upload.accept(&wire(1), now), "out of order");
        assert!(upload.accept(&wire(0), now));
        assert!(!upload.accept(&wire(0), now), "duplicate");
        assert!(upload.accept(&wire(1), now));
        assert!(upload.complete());
        assert!(!upload.overdue(now));
        assert!(upload.overdue(now + ITEM_TIMEOUT));
    }
}
