//! The mission upload handshake and conversion between wire and model items.
//!
//! An upload is `MISSION_COUNT` from the ground station, then one
//! `MISSION_REQUEST_INT` from us and one `MISSION_ITEM_INT` back per item, then
//! `MISSION_ACK`. The gateway holds the partial mission here until the last item
//! arrives, and only then hands the whole thing to the simulation as one intent:
//! the drone never flies half a plan.

use std::time::{Duration, Instant};

use mavlink::dialects::common::{MavCmd, MavFrame, MavMissionType, MISSION_ITEM_INT_DATA};
use num_traits::FromPrimitive;
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
    /// The ground station's own system and component ids, for replies.
    pub gcs: (u8, u8),
    pub last_io: Instant,
    pub retries: u8,
}

impl Upload {
    pub fn new(count: u16, gcs: (u8, u8), now: Instant) -> Self {
        Upload {
            count,
            items: Vec::with_capacity(usize::from(count)),
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

#[cfg(test)]
mod tests {
    use super::*;

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
        let mut upload = Upload::new(2, (255, 190), now);
        assert!(!upload.accept(&wire(1), now), "out of order");
        assert!(upload.accept(&wire(0), now));
        assert!(!upload.accept(&wire(0), now), "duplicate");
        assert!(upload.accept(&wire(1), now));
        assert!(upload.complete());
        assert!(!upload.overdue(now));
        assert!(upload.overdue(now + ITEM_TIMEOUT));
    }
}
