//! A fake ground station against the real gateway and a running simulation.
//!
//! Everything here goes over loopback UDP in MAVLink v2, exactly as
//! QGroundControl would talk to krabshell: heartbeats in, the mission upload and
//! download handshakes, commands and their acks, and `MISSION_ITEM_REACHED` as
//! the simulated drone flies.

use std::collections::BTreeSet;
use std::net::SocketAddr;
use std::path::Path;
use std::sync::Arc;
use std::time::{Duration, Instant};

use mavlink::dialects::common::{
    MavCmd, MavFrame, MavMessage, MavMissionResult, MavMissionType, MavResult, COMMAND_LONG_DATA,
    MISSION_COUNT_DATA, MISSION_ITEM_INT_DATA, MISSION_REQUEST_INT_DATA, MISSION_REQUEST_LIST_DATA,
};
use mavlink::{MavHeader, MavlinkVersion};
use sim_engine::{ActorLimiter, Control, Driver, ModelRegistry};
use sim_mavlink::{Gateway, MavlinkConfig};
use tokio::net::UdpSocket;
use tokio::sync::watch;

/// QGroundControl's own ids.
const GCS: (u8, u8) = (255, 190);
/// The idle drone in `london-patrol`, waiting for a ground station.
const UAV: u8 = 4;

struct Station {
    socket: UdpSocket,
    gateway: SocketAddr,
    seq: u8,
}

impl Station {
    async fn send(&mut self, msg: MavMessage) {
        let mut buf = Vec::new();
        let header = MavHeader {
            system_id: GCS.0,
            component_id: GCS.1,
            sequence: self.seq,
        };
        self.seq = self.seq.wrapping_add(1);
        mavlink::write_versioned_msg(&mut buf, MavlinkVersion::V2, header, &msg).unwrap();
        self.socket.send_to(&buf, self.gateway).await.unwrap();
    }

    /// Wait for the first message from `sysid` that `pick` accepts.
    async fn expect<T>(
        &mut self,
        sysid: u8,
        what: &str,
        mut pick: impl FnMut(&MavMessage) -> Option<T>,
    ) -> T {
        let deadline = Instant::now() + Duration::from_secs(10);
        let mut buf = vec![0u8; 65_536];
        loop {
            let left = deadline.saturating_duration_since(Instant::now());
            assert!(!left.is_zero(), "timed out waiting for {what}");
            let Ok(Ok((n, _))) = tokio::time::timeout(left, self.socket.recv_from(&mut buf)).await
            else {
                panic!("timed out waiting for {what}");
            };
            let mut reader = mavlink::peek_reader::PeekReader::new(&buf[..n]);
            while let Ok((header, msg)) = mavlink::read_any_msg::<MavMessage, _>(&mut reader) {
                if header.system_id == sysid && header.component_id == 1 {
                    if let Some(found) = pick(&msg) {
                        return found;
                    }
                }
            }
        }
    }

    async fn command(&mut self, command: MavCmd, params: [f32; 7]) -> MavResult {
        self.send(MavMessage::COMMAND_LONG(COMMAND_LONG_DATA {
            param1: params[0],
            param2: params[1],
            param3: params[2],
            param4: params[3],
            param5: params[4],
            param6: params[5],
            param7: params[6],
            command,
            target_system: UAV,
            target_component: 1,
            confirmation: 0,
        }))
        .await;
        self.expect(UAV, &format!("ack for {command:?}"), |m| match m {
            MavMessage::COMMAND_ACK(a) if a.command == command => Some(a.result),
            _ => None,
        })
        .await
    }
}

#[allow(deprecated)]
fn item(seq: u16, command: MavCmd, x: i32, y: i32, z: f32) -> MISSION_ITEM_INT_DATA {
    MISSION_ITEM_INT_DATA {
        param1: 0.0,
        param2: 2.0,
        param3: 0.0,
        param4: f32::NAN,
        x,
        y,
        z,
        seq,
        command,
        target_system: UAV,
        target_component: 1,
        frame: MavFrame::MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
        current: 0,
        autocontinue: 1,
        mission_type: MavMissionType::MAV_MISSION_TYPE_MISSION,
    }
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn a_ground_station_can_upload_download_and_fly_a_mission() {
    let pack = Path::new(env!("CARGO_MANIFEST_DIR")).join("../../scenarios/london-patrol");
    let blob = sim_engine::read_pack(&pack).expect("london-patrol pack");
    let driver = Driver::spawn(ModelRegistry::with_builtins(), blob).unwrap();
    let handle = driver.handle();
    handle.command(Control::Start { target_tps: None }).unwrap();

    let station_socket = UdpSocket::bind("127.0.0.1:0").await.unwrap();
    let cfg = MavlinkConfig {
        bind: "127.0.0.1:0".parse().unwrap(),
        gcs: vec![station_socket.local_addr().unwrap()],
        telemetry_hz: 10.0,
    };
    let gateway = Gateway::bind(cfg, handle.clone(), Arc::new(ActorLimiter::new()))
        .await
        .unwrap();
    let mut station = Station {
        gateway: gateway.local_addr().unwrap(),
        socket: station_socket,
        seq: 0,
    };
    let (stop_tx, stop_rx) = watch::channel(false);
    let task = tokio::spawn(gateway.run(stop_rx));

    // Every drone in the pack announces itself as its own system.
    let mut seen = BTreeSet::new();
    let deadline = Instant::now() + Duration::from_secs(5);
    while seen.len() < 4 && Instant::now() < deadline {
        for sysid in 1..=4u8 {
            if !seen.contains(&sysid) {
                let got = tokio::time::timeout(
                    Duration::from_millis(1500),
                    station.expect(sysid, "heartbeat", |m| {
                        matches!(m, MavMessage::HEARTBEAT(_)).then_some(())
                    }),
                )
                .await;
                if got.is_ok() {
                    seen.insert(sysid);
                }
            }
        }
    }
    assert_eq!(seen, BTreeSet::from([1, 2, 3, 4]));

    // QGroundControl's first question of a generic autopilot.
    let result = station
        .command(
            MavCmd::MAV_CMD_REQUEST_MESSAGE,
            [148.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        )
        .await;
    assert_eq!(result, MavResult::MAV_RESULT_ACCEPTED);
    let caps = station
        .expect(UAV, "AUTOPILOT_VERSION", |m| match m {
            MavMessage::AUTOPILOT_VERSION(v) => Some(v.capabilities),
            _ => None,
        })
        .await;
    assert!(caps.contains(
        mavlink::dialects::common::MavProtocolCapability::MAV_PROTOCOL_CAPABILITY_MISSION_INT
    ));

    // Upload: takeoff, a waypoint 150 m north of UAV-4's home, land.
    let north = 515_080_000 + 13_500;
    let items = [
        item(0, MavCmd::MAV_CMD_NAV_TAKEOFF, 0, 0, 30.0),
        item(1, MavCmd::MAV_CMD_NAV_WAYPOINT, north, -1_281_000, 30.0),
        item(2, MavCmd::MAV_CMD_NAV_LAND, 0, 0, 0.0),
    ];
    station
        .send(MavMessage::MISSION_COUNT(MISSION_COUNT_DATA {
            count: items.len() as u16,
            target_system: UAV,
            target_component: 1,
            mission_type: MavMissionType::MAV_MISSION_TYPE_MISSION,
            opaque_id: 0,
        }))
        .await;
    for expected in 0..items.len() as u16 {
        let seq = station
            .expect(UAV, "MISSION_REQUEST_INT", |m| match m {
                MavMessage::MISSION_REQUEST_INT(r) => Some(r.seq),
                _ => None,
            })
            .await;
        assert_eq!(seq, expected);
        station
            .send(MavMessage::MISSION_ITEM_INT(
                items[usize::from(seq)].clone(),
            ))
            .await;
    }
    let ack = station
        .expect(UAV, "MISSION_ACK", |m| match m {
            MavMessage::MISSION_ACK(a) => Some(a.mavtype),
            _ => None,
        })
        .await;
    assert_eq!(ack, MavMissionResult::MAV_MISSION_ACCEPTED);

    // Download: the simulation's copy matches what was sent.
    station
        .send(MavMessage::MISSION_REQUEST_LIST(
            MISSION_REQUEST_LIST_DATA {
                target_system: UAV,
                target_component: 1,
                mission_type: MavMissionType::MAV_MISSION_TYPE_MISSION,
            },
        ))
        .await;
    let count = station
        .expect(UAV, "MISSION_COUNT", |m| match m {
            MavMessage::MISSION_COUNT(c) => Some(c.count),
            _ => None,
        })
        .await;
    assert_eq!(count, 3);
    for seq in 0..count {
        station
            .send(MavMessage::MISSION_REQUEST_INT(MISSION_REQUEST_INT_DATA {
                seq,
                target_system: UAV,
                target_component: 1,
                mission_type: MavMissionType::MAV_MISSION_TYPE_MISSION,
            }))
            .await;
        let got = station
            .expect(UAV, "MISSION_ITEM_INT", |m| match m {
                MavMessage::MISSION_ITEM_INT(i) if i.seq == seq => Some(i.clone()),
                _ => None,
            })
            .await;
        let sent = &items[usize::from(seq)];
        assert_eq!(
            (got.command, got.x, got.y, got.z),
            (sent.command, sent.x, sent.y, sent.z)
        );
    }

    // Commands the simulation does not have are refused, not ignored.
    let result = station
        .command(
            MavCmd::MAV_CMD_DO_SET_SERVO,
            [1.0, 1500.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        )
        .await;
    assert_eq!(result, MavResult::MAV_RESULT_UNSUPPORTED);

    // Fly it.
    let armed = station
        .command(
            MavCmd::MAV_CMD_COMPONENT_ARM_DISARM,
            [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        )
        .await;
    assert_eq!(armed, MavResult::MAV_RESULT_ACCEPTED);
    let started = station
        .command(MavCmd::MAV_CMD_MISSION_START, [0.0; 7])
        .await;
    assert_eq!(started, MavResult::MAV_RESULT_ACCEPTED);

    let mut reached = Vec::new();
    while reached.len() < 3 {
        let seq = station
            .expect(UAV, "MISSION_ITEM_REACHED", |m| match m {
                MavMessage::MISSION_ITEM_REACHED(r) => Some(r.seq),
                _ => None,
            })
            .await;
        reached.push(seq);
    }
    assert_eq!(reached, vec![0, 1, 2]);

    // A paused simulation cannot act on a command, and says so at once.
    handle.command(Control::Pause).unwrap();
    tokio::time::sleep(Duration::from_millis(300)).await;
    let result = station
        .command(
            MavCmd::MAV_CMD_COMPONENT_ARM_DISARM,
            [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        )
        .await;
    assert_eq!(result, MavResult::MAV_RESULT_TEMPORARILY_REJECTED);

    stop_tx.send(true).unwrap();
    task.await.unwrap().unwrap();
    driver.shutdown();
}
