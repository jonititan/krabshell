//! What a drone says about itself, built from its published state.
//!
//! These are pure functions from a [`DroneStatus`] to MAVLink messages, so the
//! mapping (modes to heartbeat flags, integer pose to telemetry units) can be
//! read and tested without a socket.

use mavlink::dialects::common::{
    MavAutopilot, MavMessage, MavMissionType, MavModeFlag, MavProtocolCapability, MavSeverity,
    MavState, MavSysStatusSensor, MavSysStatusSensorExtended, MavType, MissionState, ATTITUDE_DATA,
    AUTOPILOT_VERSION_DATA, GLOBAL_POSITION_INT_DATA, HEARTBEAT_DATA, HOME_POSITION_DATA,
    MISSION_COUNT_DATA, MISSION_CURRENT_DATA, PARAM_VALUE_DATA, STATUSTEXT_DATA, SYS_STATUS_DATA,
    VFR_HUD_DATA,
};
use sim_engine::models::drones::mission::Mode;
use sim_engine::models::drones::DroneStatus;

/// Custom mode numbers reported in HEARTBEAT, one per simulated flight mode.
pub fn custom_mode(mode: &Mode) -> u32 {
    match mode {
        Mode::Disarmed => 0,
        Mode::Idle => 1,
        Mode::Takeoff { .. } => 2,
        Mode::Auto => 3,
        Mode::Loiter => 4,
        Mode::Goto { .. } => 5,
        Mode::Rtl => 6,
        Mode::Land => 7,
    }
}

pub fn heartbeat(drone: &DroneStatus, sim_healthy: bool) -> MavMessage {
    let mut base = MavModeFlag::MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
    if drone.armed() {
        base |=
            MavModeFlag::MAV_MODE_FLAG_SAFETY_ARMED | MavModeFlag::MAV_MODE_FLAG_STABILIZE_ENABLED;
    }
    match drone.mode {
        Mode::Auto | Mode::Rtl | Mode::Land => base |= MavModeFlag::MAV_MODE_FLAG_AUTO_ENABLED,
        Mode::Takeoff { .. } | Mode::Loiter | Mode::Goto { .. } => {
            base |= MavModeFlag::MAV_MODE_FLAG_GUIDED_ENABLED
        }
        Mode::Disarmed | Mode::Idle => {}
    }
    let status = if !sim_healthy {
        MavState::MAV_STATE_CRITICAL
    } else if drone.armed() {
        MavState::MAV_STATE_ACTIVE
    } else {
        MavState::MAV_STATE_STANDBY
    };
    MavMessage::HEARTBEAT(HEARTBEAT_DATA {
        custom_mode: custom_mode(&drone.mode),
        mavtype: MavType::MAV_TYPE_QUADROTOR,
        autopilot: MavAutopilot::MAV_AUTOPILOT_GENERIC,
        base_mode: base,
        system_status: status,
        mavlink_version: 3,
    })
}

/// North/east velocity in cm/s from heading and ground speed.
fn velocity(drone: &DroneStatus) -> (i16, i16) {
    let hdg = f64::from(drone.pose.hdg_cdeg) / 100.0;
    let speed = f64::from(drone.pose.spd_cms);
    let rad = hdg.to_radians();
    (
        (speed * rad.cos()).round().clamp(-32_768.0, 32_767.0) as i16,
        (speed * rad.sin()).round().clamp(-32_768.0, 32_767.0) as i16,
    )
}

pub fn global_position(drone: &DroneStatus, time_boot_ms: u32) -> MavMessage {
    let (vx, vy) = velocity(drone);
    MavMessage::GLOBAL_POSITION_INT(GLOBAL_POSITION_INT_DATA {
        time_boot_ms,
        lat: drone.pose.lat_e7,
        lon: drone.pose.lon_e7,
        // The scenario has no terrain or MSL datum: altitude is relative to home.
        alt: drone.pose.alt_mm,
        relative_alt: drone.pose.alt_mm,
        vx,
        vy,
        vz: (-drone.pose.vs_cms).clamp(-32_768, 32_767) as i16,
        hdg: drone.pose.hdg_cdeg.clamp(0, 35_999) as u16,
    })
}

pub fn attitude(drone: &DroneStatus, time_boot_ms: u32) -> MavMessage {
    let mut yaw = (f64::from(drone.pose.hdg_cdeg) / 100.0).to_radians();
    if yaw > std::f64::consts::PI {
        yaw -= std::f64::consts::TAU;
    }
    MavMessage::ATTITUDE(ATTITUDE_DATA {
        time_boot_ms,
        roll: 0.0,
        pitch: 0.0,
        yaw: yaw as f32,
        rollspeed: 0.0,
        pitchspeed: 0.0,
        yawspeed: 0.0,
    })
}

pub fn vfr_hud(drone: &DroneStatus) -> MavMessage {
    let speed = drone.pose.spd_cms as f32 / 100.0;
    MavMessage::VFR_HUD(VFR_HUD_DATA {
        airspeed: speed,
        groundspeed: speed,
        alt: drone.pose.alt_mm as f32 / 1000.0,
        climb: drone.pose.vs_cms as f32 / 100.0,
        heading: (drone.pose.hdg_cdeg / 100) as i16,
        throttle: if drone.pose.alt_mm > 200 { 50 } else { 0 },
    })
}

pub fn sys_status(drone: &DroneStatus) -> MavMessage {
    let sensors = MavSysStatusSensor::MAV_SYS_STATUS_SENSOR_GPS
        | MavSysStatusSensor::MAV_SYS_STATUS_SENSOR_3D_GYRO
        | MavSysStatusSensor::MAV_SYS_STATUS_SENSOR_3D_ACCEL
        | MavSysStatusSensor::MAV_SYS_STATUS_SENSOR_3D_MAG;
    let pct = drone.battery_pct.clamp(0.0, 100.0);
    MavMessage::SYS_STATUS(SYS_STATUS_DATA {
        onboard_control_sensors_present: sensors,
        onboard_control_sensors_enabled: sensors,
        onboard_control_sensors_health: sensors,
        load: 0,
        // A 4S pack between 14.0 and 16.8 V.
        voltage_battery: (14_000.0 + pct * 28.0) as u16,
        current_battery: -1,
        drop_rate_comm: 0,
        errors_comm: 0,
        errors_count1: 0,
        errors_count2: 0,
        errors_count3: 0,
        errors_count4: 0,
        battery_remaining: pct.round() as i8,
        onboard_control_sensors_present_extended: MavSysStatusSensorExtended::empty(),
        onboard_control_sensors_enabled_extended: MavSysStatusSensorExtended::empty(),
        onboard_control_sensors_health_extended: MavSysStatusSensorExtended::empty(),
    })
}

pub fn home_position(drone: &DroneStatus) -> MavMessage {
    MavMessage::HOME_POSITION(HOME_POSITION_DATA {
        latitude: drone.home.lat_e7,
        longitude: drone.home.lon_e7,
        altitude: drone.home.alt_mm,
        x: 0.0,
        y: 0.0,
        z: 0.0,
        q: [1.0, 0.0, 0.0, 0.0],
        approach_x: 0.0,
        approach_y: 0.0,
        approach_z: 0.0,
        time_usec: 0,
    })
}

pub fn mission_current(drone: &DroneStatus) -> MavMessage {
    let total = drone.mission.len() as u16;
    let state = if total == 0 {
        MissionState::MISSION_STATE_NO_MISSION
    } else if drone.mission_seq >= total {
        MissionState::MISSION_STATE_COMPLETE
    } else if drone.mode == Mode::Auto {
        MissionState::MISSION_STATE_ACTIVE
    } else if drone.mission_seq == 0 {
        MissionState::MISSION_STATE_NOT_STARTED
    } else {
        MissionState::MISSION_STATE_PAUSED
    };
    MavMessage::MISSION_CURRENT(MISSION_CURRENT_DATA {
        seq: drone.mission_seq.min(total.saturating_sub(1)),
        total,
        mission_state: state,
        mission_mode: u8::from(drone.mode == Mode::Auto),
        mission_id: drone.mission_rev,
        fence_id: 0,
        rally_points_id: 0,
    })
}

pub fn mission_count(
    count: u16,
    mission_type: MavMissionType,
    opaque_id: u32,
    gcs: (u8, u8),
) -> MavMessage {
    MavMessage::MISSION_COUNT(MISSION_COUNT_DATA {
        count,
        target_system: gcs.0,
        target_component: gcs.1,
        mission_type,
        opaque_id,
    })
}

/// Tells a ground station this is a MAVLink 2 vehicle that speaks the
/// `_INT` mission and command messages. Without it QGroundControl treats a
/// generic autopilot as MAVLink-1-only and will not use `MISSION_ITEM_INT`.
pub fn autopilot_version(sysid: u8) -> MavMessage {
    MavMessage::AUTOPILOT_VERSION(AUTOPILOT_VERSION_DATA {
        capabilities: MavProtocolCapability::MAV_PROTOCOL_CAPABILITY_MISSION_INT
            | MavProtocolCapability::MAV_PROTOCOL_CAPABILITY_COMMAND_INT
            | MavProtocolCapability::MAV_PROTOCOL_CAPABILITY_MAVLINK2,
        uid: u64::from(sysid),
        flight_sw_version: 0x0001_0000,
        middleware_sw_version: 0,
        os_sw_version: 0,
        board_version: 0,
        vendor_id: 0,
        product_id: 0,
        flight_custom_version: *b"krabshel",
        middleware_custom_version: [0; 8],
        os_custom_version: [0; 8],
        uid2: [0; 18],
    })
}

/// The one parameter a simulated drone exposes, so a ground station's
/// parameter download finishes instead of timing out.
pub fn only_param(sysid: u8) -> MavMessage {
    MavMessage::PARAM_VALUE(PARAM_VALUE_DATA {
        param_value: f32::from(sysid),
        param_count: 1,
        param_index: 0,
        param_id: "SYSID_THISMAV".into(),
        param_type: mavlink::dialects::common::MavParamType::MAV_PARAM_TYPE_UINT8,
    })
}

pub fn status_text(severity: MavSeverity, text: &str) -> MavMessage {
    // STATUSTEXT carries at most 50 bytes; cut on a character boundary.
    let mut end = text.len().min(50);
    while !text.is_char_boundary(end) {
        end -= 1;
    }
    MavMessage::STATUSTEXT(STATUSTEXT_DATA {
        severity,
        text: text[..end].into(),
        id: 0,
        chunk_seq: 0,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use sim_core::GeoCell;
    use sim_engine::models::drones::flight::{Point, Pose};

    fn drone(mode: Mode) -> DroneStatus {
        DroneStatus {
            id: 1,
            cell: GeoCell::from_lonlat(-0.1276, 51.5072, 15).unwrap(),
            sysid: 3,
            callsign: "UAV-3".into(),
            pose: Pose {
                lat_e7: 515_072_000,
                lon_e7: -1_276_000,
                alt_mm: 40_000,
                hdg_cdeg: 9_000,
                spd_cms: 1_000,
                vs_cms: 0,
            },
            mode,
            home: Point::default(),
            mission: Vec::new(),
            mission_seq: 0,
            mission_rev: 0,
            battery_pct: 87.5,
        }
    }

    #[test]
    fn an_armed_auto_drone_heartbeats_as_armed_and_auto() {
        let MavMessage::HEARTBEAT(hb) = heartbeat(&drone(Mode::Auto), true) else {
            panic!()
        };
        assert!(hb
            .base_mode
            .contains(MavModeFlag::MAV_MODE_FLAG_SAFETY_ARMED));
        assert!(hb
            .base_mode
            .contains(MavModeFlag::MAV_MODE_FLAG_AUTO_ENABLED));
        assert_eq!(hb.system_status, MavState::MAV_STATE_ACTIVE);
        assert_eq!(hb.mavlink_version, 3);

        let MavMessage::HEARTBEAT(hb) = heartbeat(&drone(Mode::Disarmed), true) else {
            panic!()
        };
        assert!(!hb
            .base_mode
            .contains(MavModeFlag::MAV_MODE_FLAG_SAFETY_ARMED));
        assert_eq!(hb.system_status, MavState::MAV_STATE_STANDBY);
    }

    #[test]
    fn flying_east_is_positive_vy() {
        let MavMessage::GLOBAL_POSITION_INT(pos) = global_position(&drone(Mode::Auto), 5) else {
            panic!()
        };
        assert_eq!(pos.vx, 0);
        assert_eq!(pos.vy, 1_000);
        assert_eq!(pos.hdg, 9_000);
        assert_eq!(pos.relative_alt, 40_000);
    }

    #[test]
    fn status_text_is_cut_to_fifty_bytes() {
        let MavMessage::STATUSTEXT(st) =
            status_text(MavSeverity::MAV_SEVERITY_INFO, &"é".repeat(40))
        else {
            panic!()
        };
        assert!(st.text.to_str().unwrap().len() <= 50);
    }
}
