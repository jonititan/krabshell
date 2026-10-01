//! krabshell's MAVLink gateway: every simulated drone, presented to ground
//! stations as a real vehicle on a UDP link.
//!
//! The simulation runs as fast as it can and knows nothing about MAVLink. This
//! gateway is the in-between layer. It speaks the real-time half of the
//! protocol itself — heartbeats, telemetry cadence, the mission handshake and
//! its timeouts, all on the wall clock — and turns what ground stations ask
//! for into intents:
//!
//! - A completed mission upload becomes one `MISSION_UPLOAD` intent.
//! - `COMMAND_LONG`/`COMMAND_INT` (arm, takeoff, mission start, RTL, land,
//!   reposition, pause) become `COMMAND` intents.
//! - Downloads, telemetry and `MISSION_ITEM_REACHED` are served from the
//!   driver's published view and reports.
//!
//! The simulation stays the authority. The gateway never touches model state,
//! never steps the schedule, and never blocks on the driver: acks come back
//! over a channel it polls on its own timer.
//!
//! One UDP socket carries every drone. Each is its own MAVLink system (its
//! `sysid`, component 1), which is how QGroundControl and MAVSDK tell vehicles
//! apart on a shared link. Telemetry goes to the configured ground stations and
//! to any peer that has sent us a packet recently.
//!
//! MAVLink has no authentication. Bind to loopback unless the network is trusted.

pub mod missions;
pub mod telemetry;

use std::collections::{BTreeMap, HashMap};
use std::net::SocketAddr;
use std::sync::Arc;
use std::time::{Duration, Instant};

use crossbeam_channel::{Receiver, TryRecvError};
use mavlink::dialects::common::{
    MavCmd, MavMessage, MavMissionResult, MavMissionType, MavResult, MavSeverity, COMMAND_ACK_DATA,
    MISSION_ACK_DATA, MISSION_ITEM_REACHED_DATA, MISSION_REQUEST_INT_DATA,
};
use mavlink::{MavHeader, MavlinkVersion};
use sim_core::{ActorId, Intent, IntentVerb, ReportKind};
use sim_engine::models::drones::{codes, drones_in, DroneStatus};
use sim_engine::{ActorLimiter, DriverHandle, IntentAck, Published, Submission};
use tokio::net::UdpSocket;
use tokio::sync::watch;
use uuid::Uuid;

use missions::Upload;

/// The component id every simulated drone speaks as (MAV_COMP_ID_AUTOPILOT1).
/// QGroundControl only creates a vehicle for heartbeats from component 1.
const COMPONENT: u8 = 1;

/// Forget a peer that has sent nothing for this long.
const PEER_TTL: Duration = Duration::from_secs(60);

/// How long to wait for the simulation to apply a mission before answering the
/// ground station anyway. Under QGroundControl's 1.5 s retry timer.
const MISSION_ACK_WAIT: Duration = Duration::from_millis(1000);
const COMMAND_ACK_WAIT: Duration = Duration::from_millis(1000);

// MAV_CMD numbers the gateway answers itself rather than passing to the sim.
const REQUEST_MESSAGE: u16 = 512;
const REQUEST_AUTOPILOT_CAPABILITIES: u16 = 520;
const SET_MESSAGE_INTERVAL: u16 = 511;
const MSG_AUTOPILOT_VERSION: u32 = 148;
const MSG_HOME_POSITION: u32 = 242;

/// Commands the drone model understands; see `drones::mission`.
const SIM_COMMANDS: [u16; 9] = [400, 22, 300, 20, 21, 176, 192, 193, 224];

#[derive(Debug, Clone)]
pub struct MavlinkConfig {
    pub bind: SocketAddr,
    /// Ground stations to send telemetry to whether or not they have spoken
    /// first. QGroundControl listens on 14550 by default.
    pub gcs: Vec<SocketAddr>,
    /// Position, attitude and HUD rate, per drone.
    pub telemetry_hz: f64,
}

impl Default for MavlinkConfig {
    fn default() -> Self {
        MavlinkConfig {
            bind: "127.0.0.1:14551".parse().expect("valid"),
            gcs: vec!["127.0.0.1:14550".parse().expect("valid")],
            telemetry_hz: 5.0,
        }
    }
}

/// What a pending intent is waiting to tell a ground station.
#[derive(Debug)]
enum Awaiting {
    Mission,
    Command(MavCmd),
}

#[derive(Debug)]
struct Pending {
    rx: Receiver<IntentAck>,
    deadline: Instant,
    peer: SocketAddr,
    sysid: u8,
    gcs: (u8, u8),
    what: Awaiting,
    /// The drone's `mission_rev` when the upload was submitted.
    rev_before: u32,
    /// The simulation has applied it; waiting for a publish to show it.
    applied: bool,
}

pub struct Gateway {
    socket: UdpSocket,
    cfg: MavlinkConfig,
    handle: DriverHandle,
    limiter: Arc<ActorLimiter>,
    started: Instant,

    view: Arc<Published>,
    drones: BTreeMap<u8, DroneStatus>,
    peers: HashMap<SocketAddr, Instant>,
    sequence: [u8; 256],
    uploads: HashMap<(SocketAddr, u8), Upload>,
    pending: Vec<Pending>,
    /// Last (seq, rev) announced in MISSION_CURRENT, per drone.
    announced: BTreeMap<u8, (u16, u32)>,
    last_report: u64,
    world: (String, u64),
    buf: Vec<u8>,
}

impl Gateway {
    /// Bind the socket. Separate from [`Gateway::run`] so a caller can learn
    /// the bound port first (tests bind port 0).
    pub async fn bind(
        cfg: MavlinkConfig,
        handle: DriverHandle,
        limiter: Arc<ActorLimiter>,
    ) -> std::io::Result<Self> {
        let socket = UdpSocket::bind(cfg.bind).await?;
        let view = handle.view();
        let last_report = view.reports.last().map_or(0, |r| r.seq);
        let world = (view.scenario_id.clone(), view.meta.tick);
        Ok(Gateway {
            socket,
            cfg,
            handle,
            limiter,
            started: Instant::now(),
            drones: drones_in(&view.cells),
            view,
            peers: HashMap::new(),
            sequence: [0; 256],
            uploads: HashMap::new(),
            pending: Vec::new(),
            announced: BTreeMap::new(),
            last_report,
            world,
            buf: Vec::with_capacity(512),
        })
    }

    pub fn local_addr(&self) -> std::io::Result<SocketAddr> {
        self.socket.local_addr()
    }

    /// Serve until `shutdown` flips to true.
    pub async fn run(mut self, mut shutdown: watch::Receiver<bool>) -> anyhow::Result<()> {
        tracing::info!(
            bind = %self.local_addr()?,
            gcs = ?self.cfg.gcs,
            drones = self.drones.len(),
            "MAVLink gateway up"
        );
        let mut heartbeat = tokio::time::interval(Duration::from_secs(1));
        let mut telemetry = tokio::time::interval(Duration::from_secs_f64(
            1.0 / self.cfg.telemetry_hz.clamp(0.1, 50.0),
        ));
        let mut poll = tokio::time::interval(Duration::from_millis(20));
        for timer in [&mut heartbeat, &mut telemetry, &mut poll] {
            timer.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Delay);
        }
        let mut datagram = vec![0u8; 65_536];

        loop {
            tokio::select! {
                changed = shutdown.changed() => {
                    if changed.is_err() || *shutdown.borrow() {
                        break;
                    }
                }
                received = self.socket.recv_from(&mut datagram) => {
                    match received {
                        Ok((n, peer)) => self.on_datagram(&datagram[..n], peer).await,
                        // ICMP "port unreachable" from a GCS that went away
                        // surfaces here on some platforms; it is not fatal.
                        Err(err) => tracing::debug!(%err, "MAVLink receive error"),
                    }
                }
                _ = heartbeat.tick() => {
                    self.refresh();
                    self.send_heartbeats().await;
                }
                _ = telemetry.tick() => {
                    self.refresh();
                    self.send_telemetry().await;
                }
                _ = poll.tick() => {
                    self.poll_pending().await;
                    self.poll_uploads().await;
                }
            }
        }
        tracing::info!("MAVLink gateway stopped");
        Ok(())
    }

    // ------------------------------------------------------------- state

    fn time_boot_ms(&self) -> u32 {
        self.started.elapsed().as_millis() as u32
    }

    fn sim_running(&self) -> bool {
        self.view.health.mode.is_advancing()
    }

    /// Take the latest published view. A reload or restore invalidates any
    /// half-finished upload, since the drone it was for may not exist.
    fn refresh(&mut self) {
        let view = self.handle.view();
        let world = (view.scenario_id.clone(), view.meta.tick);
        if world.0 != self.world.0 || world.1 < self.world.1 {
            if !self.uploads.is_empty() {
                tracing::info!(
                    "world reloaded; dropping {} mission upload(s)",
                    self.uploads.len()
                );
            }
            self.uploads.clear();
            self.announced.clear();
        }
        self.world = world;
        self.drones = drones_in(&view.cells);
        self.view = view;
        let now = Instant::now();
        self.peers
            .retain(|_, seen| now.duration_since(*seen) < PEER_TTL);
    }

    fn destinations(&self) -> Vec<SocketAddr> {
        let mut out = self.cfg.gcs.clone();
        for peer in self.peers.keys() {
            if !out.contains(peer) {
                out.push(*peer);
            }
        }
        out
    }

    // -------------------------------------------------------------- send

    fn encode(&mut self, sysid: u8, msg: &MavMessage) -> bool {
        let seq = &mut self.sequence[usize::from(sysid)];
        let header = MavHeader {
            system_id: sysid,
            component_id: COMPONENT,
            sequence: *seq,
        };
        *seq = seq.wrapping_add(1);
        self.buf.clear();
        mavlink::write_versioned_msg(&mut self.buf, MavlinkVersion::V2, header, msg).is_ok()
    }

    async fn send_to(&mut self, sysid: u8, msg: &MavMessage, to: SocketAddr) {
        if self.encode(sysid, msg) {
            if let Err(err) = self.socket.send_to(&self.buf, to).await {
                tracing::debug!(%err, %to, "MAVLink send failed");
            }
        }
    }

    async fn broadcast(&mut self, sysid: u8, msg: &MavMessage) {
        if !self.encode(sysid, msg) {
            return;
        }
        for to in self.destinations() {
            if let Err(err) = self.socket.send_to(&self.buf, to).await {
                tracing::debug!(%err, %to, "MAVLink send failed");
            }
        }
    }

    async fn send_heartbeats(&mut self) {
        let healthy = self.view.health.fault.is_none() && self.view.health.alive;
        let drones: Vec<DroneStatus> = self.drones.values().cloned().collect();
        for drone in &drones {
            self.broadcast(drone.sysid, &telemetry::heartbeat(drone, healthy))
                .await;
            self.broadcast(drone.sysid, &telemetry::sys_status(drone))
                .await;
            self.broadcast(drone.sysid, &telemetry::mission_current(drone))
                .await;
            self.announced
                .insert(drone.sysid, (drone.mission_seq, drone.mission_rev));
        }
    }

    async fn send_telemetry(&mut self) {
        let t = self.time_boot_ms();
        let drones: Vec<DroneStatus> = self.drones.values().cloned().collect();
        for drone in &drones {
            self.broadcast(drone.sysid, &telemetry::global_position(drone, t))
                .await;
            self.broadcast(drone.sysid, &telemetry::attitude(drone, t))
                .await;
            self.broadcast(drone.sysid, &telemetry::vfr_hud(drone))
                .await;
            // Announce a new current item as soon as it changes, not at 1 Hz.
            let now = (drone.mission_seq, drone.mission_rev);
            if self.announced.get(&drone.sysid) != Some(&now) {
                self.announced.insert(drone.sysid, now);
                self.broadcast(drone.sysid, &telemetry::mission_current(drone))
                    .await;
            }
        }

        // Mission items reached since the last pass. Reports hold the model's
        // agent id; map it back to the drone's sysid.
        let reached: Vec<(u8, u16, u64)> = self
            .view
            .reports_since(self.last_report)
            .iter()
            .filter(|r| r.kind == ReportKind::Waypoint)
            .filter_map(|r| {
                let sysid = r.detail.get("sysid")?.as_u64()? as u8;
                let seq = r.detail.get("seq")?.as_u64()? as u16;
                Some((sysid, seq, r.seq))
            })
            .collect();
        if let Some(last) = self.view.reports.last() {
            if last.seq > self.last_report {
                let first = self.view.reports.first().map_or(0, |r| r.seq);
                if first > self.last_report + 1 && self.last_report > 0 {
                    tracing::warn!(
                        missed = first - self.last_report - 1,
                        "MAVLink gateway fell behind the report ring"
                    );
                }
                self.last_report = last.seq;
            }
        }
        for (sysid, seq, _) in reached {
            let msg = MavMessage::MISSION_ITEM_REACHED(MISSION_ITEM_REACHED_DATA { seq });
            self.broadcast(sysid, &msg).await;
        }
    }

    async fn status(&mut self, sysid: u8, to: SocketAddr, severity: MavSeverity, text: &str) {
        self.send_to(sysid, &telemetry::status_text(severity, text), to)
            .await;
    }

    async fn mission_ack(
        &mut self,
        sysid: u8,
        to: SocketAddr,
        gcs: (u8, u8),
        result: MavMissionResult,
    ) {
        let rev = self.drones.get(&sysid).map_or(0, |d| d.mission_rev);
        let msg = MavMessage::MISSION_ACK(MISSION_ACK_DATA {
            target_system: gcs.0,
            target_component: gcs.1,
            mavtype: result,
            mission_type: MavMissionType::MAV_MISSION_TYPE_MISSION,
            opaque_id: rev,
        });
        self.send_to(sysid, &msg, to).await;
    }

    async fn command_ack(
        &mut self,
        sysid: u8,
        to: SocketAddr,
        gcs: (u8, u8),
        command: MavCmd,
        result: MavResult,
    ) {
        let msg = MavMessage::COMMAND_ACK(COMMAND_ACK_DATA {
            command,
            result,
            progress: 0,
            result_param2: 0,
            target_system: gcs.0,
            target_component: gcs.1,
        });
        self.send_to(sysid, &msg, to).await;
    }

    async fn request_item(&mut self, sysid: u8, to: SocketAddr, gcs: (u8, u8), seq: u16) {
        let msg = MavMessage::MISSION_REQUEST_INT(MISSION_REQUEST_INT_DATA {
            seq,
            target_system: gcs.0,
            target_component: gcs.1,
            mission_type: MavMissionType::MAV_MISSION_TYPE_MISSION,
        });
        self.send_to(sysid, &msg, to).await;
    }

    // ------------------------------------------------------------ intents

    /// Queue an intent for `sysid`'s drone. On success, returns the channel its
    /// outcome will arrive on.
    fn submit(
        &mut self,
        sysid: u8,
        peer: SocketAddr,
        code: u16,
        payload: serde_json::Value,
    ) -> Result<Receiver<IntentAck>, String> {
        let drone = self
            .drones
            .get(&sysid)
            .ok_or_else(|| format!("no drone with system id {sysid}"))?;
        sim_engine::check_intent_cell(&self.view.meta, drone.cell).map_err(|e| e.to_string())?;
        let actor = format!("mavlink:{peer}");
        if !self
            .limiter
            .allow(&actor, self.view.config.run.intent_rate_per_actor)
        {
            return Err("too many requests; slow down".into());
        }
        let intent = Intent {
            id: Uuid::new_v4(),
            tick: None,
            cell: drone.cell,
            agent: Some(drone.id),
            actor: ActorId(actor),
            verb: IntentVerb::Custom(code),
            payload,
            issued_at: std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .map_or(0, |d| d.as_millis() as u64),
        };
        let (tx, rx) = crossbeam_channel::bounded(1);
        self.handle
            .submit(Submission {
                intent,
                ack: Some(tx),
            })
            .map_err(|reason| reason.to_string())?;
        Ok(rx)
    }

    async fn poll_pending(&mut self) {
        if self.pending.is_empty() {
            return;
        }
        self.refresh();
        let now = Instant::now();
        let mut still = Vec::with_capacity(self.pending.len());
        let mut done = Vec::new();
        for mut p in std::mem::take(&mut self.pending) {
            // A mission is acknowledged only once a published view carries it,
            // so a download straight after the ack returns the new mission, not
            // the one it replaced.
            let published = |p: &Pending| {
                self.drones
                    .get(&p.sysid)
                    .is_some_and(|d| d.mission_rev != p.rev_before)
            };
            if p.applied {
                if published(&p) || now >= p.deadline {
                    done.push((p, Some(IntentAck::Applied { tick: 0 })));
                } else {
                    still.push(p);
                }
                continue;
            }
            match p.rx.try_recv() {
                Ok(IntentAck::Applied { .. })
                    if matches!(p.what, Awaiting::Mission) && !published(&p) =>
                {
                    p.applied = true;
                    still.push(p);
                }
                Ok(ack) => done.push((p, Some(ack))),
                Err(TryRecvError::Disconnected) => done.push((p, None)),
                Err(TryRecvError::Empty) if now >= p.deadline => done.push((p, None)),
                Err(TryRecvError::Empty) => still.push(p),
            }
        }
        self.pending = still;
        for (p, ack) in done {
            self.answer(p, ack).await;
        }
    }

    async fn answer(&mut self, p: Pending, ack: Option<IntentAck>) {
        match (p.what, ack) {
            (Awaiting::Mission, Some(IntentAck::Applied { .. })) => {
                self.mission_ack(
                    p.sysid,
                    p.peer,
                    p.gcs,
                    MavMissionResult::MAV_MISSION_ACCEPTED,
                )
                .await;
            }
            (Awaiting::Mission, Some(IntentAck::Rejected(reason))) => {
                let text = reason.to_string();
                let result = if text.contains("frame") {
                    MavMissionResult::MAV_MISSION_UNSUPPORTED_FRAME
                } else if text.contains("allows") {
                    MavMissionResult::MAV_MISSION_NO_SPACE
                } else if text.contains("outside the world") {
                    MavMissionResult::MAV_MISSION_DENIED
                } else {
                    MavMissionResult::MAV_MISSION_ERROR
                };
                self.status(p.sysid, p.peer, MavSeverity::MAV_SEVERITY_ERROR, &text)
                    .await;
                self.mission_ack(p.sysid, p.peer, p.gcs, result).await;
            }
            (Awaiting::Mission, None) => {
                // Queued but not yet applied: the simulation is paused or busy.
                // The upload is complete on our side and will apply on the next
                // tick; say so rather than make the ground station retry.
                self.status(
                    p.sysid,
                    p.peer,
                    MavSeverity::MAV_SEVERITY_WARNING,
                    "Mission queued: simulation paused",
                )
                .await;
                self.mission_ack(
                    p.sysid,
                    p.peer,
                    p.gcs,
                    MavMissionResult::MAV_MISSION_ACCEPTED,
                )
                .await;
            }
            (Awaiting::Command(cmd), Some(IntentAck::Applied { .. })) => {
                self.command_ack(p.sysid, p.peer, p.gcs, cmd, MavResult::MAV_RESULT_ACCEPTED)
                    .await;
            }
            (Awaiting::Command(cmd), Some(IntentAck::Rejected(reason))) => {
                let text = reason.to_string();
                let text = text
                    .strip_prefix("model rejected the intent: ")
                    .unwrap_or(&text);
                self.status(p.sysid, p.peer, MavSeverity::MAV_SEVERITY_WARNING, text)
                    .await;
                self.command_ack(p.sysid, p.peer, p.gcs, cmd, MavResult::MAV_RESULT_DENIED)
                    .await;
            }
            (Awaiting::Command(cmd), None) => {
                self.command_ack(
                    p.sysid,
                    p.peer,
                    p.gcs,
                    cmd,
                    MavResult::MAV_RESULT_TEMPORARILY_REJECTED,
                )
                .await;
            }
        }
    }

    async fn poll_uploads(&mut self) {
        let now = Instant::now();
        let overdue: Vec<(SocketAddr, u8)> = self
            .uploads
            .iter()
            .filter(|(_, u)| u.overdue(now))
            .map(|(k, _)| *k)
            .collect();
        for (peer, sysid) in overdue {
            let Some(upload) = self.uploads.get_mut(&(peer, sysid)) else {
                continue;
            };
            if upload.retries >= missions::MAX_RETRIES {
                let gcs = upload.gcs;
                self.uploads.remove(&(peer, sysid));
                self.mission_ack(
                    sysid,
                    peer,
                    gcs,
                    MavMissionResult::MAV_MISSION_OPERATION_CANCELLED,
                )
                .await;
                continue;
            }
            upload.retries += 1;
            upload.last_io = now;
            let (gcs, next) = (upload.gcs, upload.next());
            self.request_item(sysid, peer, gcs, next).await;
        }
    }

    // ------------------------------------------------------------ receive

    async fn on_datagram(&mut self, data: &[u8], peer: SocketAddr) {
        self.peers.insert(peer, Instant::now());
        let mut reader = mavlink::peek_reader::PeekReader::new(data);
        // A datagram is finite, so the reader always reaches the end; the cap
        // is there so a pathological frame can never spin this loop.
        for _ in 0..256 {
            match mavlink::read_any_msg::<MavMessage, _>(&mut reader) {
                Ok((header, msg)) => self.on_message(header, msg, peer).await,
                Err(mavlink::error::MessageReadError::Io(_)) => break,
                // An unknown message or enum value: skip it, keep the rest.
                Err(err) => {
                    tracing::debug!(%err, %peer, "unparseable MAVLink frame");
                    continue;
                }
            }
        }
    }

    async fn on_message(&mut self, header: MavHeader, msg: MavMessage, peer: SocketAddr) {
        let gcs = (header.system_id, header.component_id);
        let target = match mavlink::Message::target_system_id(&msg) {
            Some(t) => t,
            None => return, // heartbeats and broadcasts: the peer is noted, nothing to answer
        };
        if target == 0 {
            return;
        }
        self.refresh();
        if !self.drones.contains_key(&target) {
            return; // someone else's vehicle on a shared link
        }
        let sysid = target;

        match msg {
            MavMessage::MISSION_COUNT(m) => {
                if m.mission_type != MavMissionType::MAV_MISSION_TYPE_MISSION {
                    // Fences and rally points are not simulated; an empty one
                    // is harmless to accept, anything else is not supported.
                    let result = if m.count == 0 {
                        MavMissionResult::MAV_MISSION_ACCEPTED
                    } else {
                        MavMissionResult::MAV_MISSION_UNSUPPORTED
                    };
                    self.send_to(
                        sysid,
                        &MavMessage::MISSION_ACK(MISSION_ACK_DATA {
                            target_system: gcs.0,
                            target_component: gcs.1,
                            mavtype: result,
                            mission_type: m.mission_type,
                            opaque_id: 0,
                        }),
                        peer,
                    )
                    .await;
                    return;
                }
                if m.count > missions::MAX_ITEMS {
                    self.mission_ack(sysid, peer, gcs, MavMissionResult::MAV_MISSION_NO_SPACE)
                        .await;
                    return;
                }
                if m.count == 0 {
                    self.finish_upload(sysid, peer, gcs, Vec::new()).await;
                    return;
                }
                self.uploads
                    .insert((peer, sysid), Upload::new(m.count, gcs, Instant::now()));
                self.request_item(sysid, peer, gcs, 0).await;
            }
            MavMessage::MISSION_ITEM_INT(item) => {
                let Some(upload) = self.uploads.get_mut(&(peer, sysid)) else {
                    return;
                };
                // A duplicate or out-of-order item is not taken; either way the
                // reply is a request for the item we still need.
                upload.accept(&item, Instant::now());
                let next = upload.next();
                if upload.complete() {
                    let upload = self.uploads.remove(&(peer, sysid)).expect("present");
                    self.finish_upload(sysid, peer, gcs, upload.items).await;
                } else {
                    self.request_item(sysid, peer, gcs, next).await;
                }
            }
            MavMessage::MISSION_REQUEST_LIST(m) => {
                let (count, rev) = if m.mission_type == MavMissionType::MAV_MISSION_TYPE_MISSION {
                    let d = &self.drones[&sysid];
                    (d.mission.len() as u16, d.mission_rev)
                } else {
                    (0, 0)
                };
                let msg = telemetry::mission_count(count, m.mission_type, rev, gcs);
                self.send_to(sysid, &msg, peer).await;
            }
            MavMessage::MISSION_REQUEST_INT(m) => self.send_item(sysid, peer, gcs, m.seq).await,
            #[allow(deprecated)]
            MavMessage::MISSION_REQUEST(m) => self.send_item(sysid, peer, gcs, m.seq).await,
            MavMessage::MISSION_CLEAR_ALL(m) => {
                if m.mission_type == MavMissionType::MAV_MISSION_TYPE_MISSION {
                    self.finish_upload(sysid, peer, gcs, Vec::new()).await;
                } else {
                    self.send_to(
                        sysid,
                        &MavMessage::MISSION_ACK(MISSION_ACK_DATA {
                            target_system: gcs.0,
                            target_component: gcs.1,
                            mavtype: MavMissionResult::MAV_MISSION_ACCEPTED,
                            mission_type: m.mission_type,
                            opaque_id: 0,
                        }),
                        peer,
                    )
                    .await;
                }
            }
            #[allow(deprecated)]
            MavMessage::MISSION_SET_CURRENT(m) => {
                let payload = serde_json::json!({ "seq": m.seq });
                if let Err(err) = self.submit(sysid, peer, codes::MISSION_SET_CURRENT, payload) {
                    self.status(sysid, peer, MavSeverity::MAV_SEVERITY_WARNING, &err)
                        .await;
                }
            }
            MavMessage::COMMAND_LONG(c) => {
                let params = [
                    c.param1, c.param2, c.param3, c.param4, c.param5, c.param6, c.param7,
                ];
                self.on_command(sysid, peer, gcs, c.command, params).await;
            }
            MavMessage::COMMAND_INT(c) => {
                let params = [
                    c.param1,
                    c.param2,
                    c.param3,
                    c.param4,
                    c.x as f32 * 1e-7,
                    c.y as f32 * 1e-7,
                    c.z,
                ];
                self.on_command(sysid, peer, gcs, c.command, params).await;
            }
            MavMessage::PARAM_REQUEST_LIST(_) | MavMessage::PARAM_REQUEST_READ(_) => {
                self.send_to(sysid, &telemetry::only_param(sysid), peer)
                    .await;
            }
            MavMessage::PARAM_SET(_) => {
                // Nothing is settable; echo the one value so the GCS stops retrying.
                self.send_to(sysid, &telemetry::only_param(sysid), peer)
                    .await;
            }
            // Superseded by DO_SET_MODE, but QGroundControl still sends it to
            // change flight mode on a generic autopilot.
            #[allow(deprecated)]
            MavMessage::SET_MODE(m) => {
                let flags = m.base_mode as u32;
                let params = [flags as f32, m.custom_mode as f32, 0.0, 0.0, 0.0, 0.0, 0.0];
                let payload = serde_json::json!({ "command": 176, "params": params });
                if let Err(err) = self.submit(sysid, peer, codes::COMMAND, payload) {
                    self.status(sysid, peer, MavSeverity::MAV_SEVERITY_WARNING, &err)
                        .await;
                }
            }
            _ => {}
        }
    }

    async fn send_item(&mut self, sysid: u8, peer: SocketAddr, gcs: (u8, u8), seq: u16) {
        let drone = &self.drones[&sysid];
        let current = drone.mission_seq == seq;
        match drone
            .mission
            .get(usize::from(seq))
            .and_then(|item| missions::to_wire(item, current, gcs))
        {
            Some(item) => {
                self.send_to(sysid, &MavMessage::MISSION_ITEM_INT(item), peer)
                    .await
            }
            None => {
                self.mission_ack(
                    sysid,
                    peer,
                    gcs,
                    MavMissionResult::MAV_MISSION_INVALID_SEQUENCE,
                )
                .await
            }
        }
    }

    async fn finish_upload(
        &mut self,
        sysid: u8,
        peer: SocketAddr,
        gcs: (u8, u8),
        items: Vec<sim_engine::models::drones::mission::MissionItem>,
    ) {
        let code = if items.is_empty() {
            codes::MISSION_CLEAR
        } else {
            codes::MISSION_UPLOAD
        };
        let payload = serde_json::json!({ "items": items });
        let rev_before = self.drones.get(&sysid).map_or(0, |d| d.mission_rev);
        match self.submit(sysid, peer, code, payload) {
            Ok(rx) => self.pending.push(Pending {
                rx,
                deadline: Instant::now() + MISSION_ACK_WAIT,
                peer,
                sysid,
                gcs,
                what: Awaiting::Mission,
                rev_before,
                applied: false,
            }),
            Err(err) => {
                self.status(sysid, peer, MavSeverity::MAV_SEVERITY_ERROR, &err)
                    .await;
                self.mission_ack(sysid, peer, gcs, MavMissionResult::MAV_MISSION_ERROR)
                    .await;
            }
        }
    }

    async fn on_command(
        &mut self,
        sysid: u8,
        peer: SocketAddr,
        gcs: (u8, u8),
        command: MavCmd,
        params: [f32; 7],
    ) {
        let number = command as u16;
        match number {
            REQUEST_MESSAGE | REQUEST_AUTOPILOT_CAPABILITIES => {
                let wanted = if number == REQUEST_MESSAGE {
                    params[0] as u32
                } else {
                    MSG_AUTOPILOT_VERSION
                };
                let reply = match wanted {
                    MSG_AUTOPILOT_VERSION => Some(telemetry::autopilot_version(sysid)),
                    MSG_HOME_POSITION => Some(telemetry::home_position(&self.drones[&sysid])),
                    _ => None,
                };
                match reply {
                    Some(msg) => {
                        self.command_ack(sysid, peer, gcs, command, MavResult::MAV_RESULT_ACCEPTED)
                            .await;
                        self.send_to(sysid, &msg, peer).await;
                    }
                    None => {
                        self.command_ack(
                            sysid,
                            peer,
                            gcs,
                            command,
                            MavResult::MAV_RESULT_UNSUPPORTED,
                        )
                        .await
                    }
                }
            }
            SET_MESSAGE_INTERVAL => {
                // Rates are the gateway's own; accept so the GCS moves on.
                self.command_ack(sysid, peer, gcs, command, MavResult::MAV_RESULT_ACCEPTED)
                    .await;
            }
            n if SIM_COMMANDS.contains(&n) => {
                if !self.sim_running() {
                    self.status(
                        sysid,
                        peer,
                        MavSeverity::MAV_SEVERITY_WARNING,
                        "Simulation is paused",
                    )
                    .await;
                    self.command_ack(
                        sysid,
                        peer,
                        gcs,
                        command,
                        MavResult::MAV_RESULT_TEMPORARILY_REJECTED,
                    )
                    .await;
                    return;
                }
                let json_params: Vec<serde_json::Value> = params
                    .iter()
                    .map(|p| {
                        if p.is_finite() {
                            serde_json::json!(p)
                        } else {
                            serde_json::Value::Null
                        }
                    })
                    .collect();
                let payload = serde_json::json!({ "command": n, "params": json_params });
                match self.submit(sysid, peer, codes::COMMAND, payload) {
                    Ok(rx) => self.pending.push(Pending {
                        rx,
                        deadline: Instant::now() + COMMAND_ACK_WAIT,
                        peer,
                        sysid,
                        gcs,
                        what: Awaiting::Command(command),
                        rev_before: 0,
                        applied: false,
                    }),
                    Err(err) => {
                        self.status(sysid, peer, MavSeverity::MAV_SEVERITY_WARNING, &err)
                            .await;
                        self.command_ack(sysid, peer, gcs, command, MavResult::MAV_RESULT_FAILED)
                            .await;
                    }
                }
            }
            _ => {
                self.command_ack(sysid, peer, gcs, command, MavResult::MAV_RESULT_UNSUPPORTED)
                    .await;
            }
        }
    }
}
