//! Cursor-on-Target out of the simulation, for ATAK, WinTAK and TAK Server.
//!
//! Three things go on the wire, one XML `<event>` per UDP datagram:
//!
//! - **Drones**, as friendly rotary-wing UAVs (`a-f-A-M-H-Q`), once a second.
//! - **Sightings**, as the model reports them: people (`a-u-G-U-C-I`),
//!   vehicles (`a-u-G-E-V`) and intruder drones (`a-u-A-M-H-Q`), each linked
//!   to the drone that saw it.
//! - **Points of interest**, as spot markers (`b-m-p-s-m`), every 15 seconds,
//!   so a client that joins late sees them quickly.
//!
//! The default destination is the SA multicast group ATAK listens on,
//! 239.2.3.1:6969, with legacy XML (TAK protocol version 0), which every TAK
//! client accepts. A unicast address works too, e.g. a TAK Server's UDP input.
//!
//! Event times are wall-clock UTC, because TAK clients age markers out against
//! their own clock. The simulation tick goes in `<remarks>`: when the sim runs
//! faster than real time, the markers move faster than real time too, but they
//! never arrive already stale.

use std::net::{Ipv4Addr, SocketAddr};
use std::time::{Duration, SystemTime, UNIX_EPOCH};

use sim_core::{Report, ReportKind};
use sim_engine::models::drones::drones_in;
use sim_engine::{DriverHandle, Published};
use tokio::net::UdpSocket;
use tokio::sync::watch;

/// Prefix on every uid, so krabshell's markers never collide with real ones.
const UID_PREFIX: &str = "krabshell";

#[derive(Debug, Clone)]
pub struct CotConfig {
    /// Where to send. A multicast group or a unicast host.
    pub dest: SocketAddr,
    /// Multicast hops. 1 keeps it on the local network.
    pub ttl: u32,
    /// Local interface to send multicast from; `None` lets the OS choose.
    pub iface: Option<Ipv4Addr>,
    /// Drone position updates per second.
    pub drone_hz: f64,
}

impl Default for CotConfig {
    fn default() -> Self {
        CotConfig {
            dest: "239.2.3.1:6969".parse().expect("valid"),
            ttl: 1,
            iface: None,
            drone_hz: 1.0,
        }
    }
}

/// One CoT event, before it is rendered.
#[derive(Debug, Clone, PartialEq)]
pub struct CotEvent {
    pub uid: String,
    pub cot_type: String,
    pub how: &'static str,
    pub time: SystemTime,
    pub stale: Duration,
    pub lat: f64,
    pub lon: f64,
    /// Metres. The simulation's altitudes are above mean sea level, which
    /// stands in for the ellipsoid: no geoid separation is applied.
    pub hae: f64,
    pub ce: f64,
    pub le: f64,
    pub callsign: Option<String>,
    /// Degrees true and metres per second.
    pub track: Option<(f64, f64)>,
    pub remarks: Option<String>,
    /// (uid, type) of the parent, e.g. the drone that made a sighting.
    pub link: Option<(String, String)>,
}

/// `YYYY-MM-DDThh:mm:ss.sssZ`, the form CoT uses.
pub fn cot_time(t: SystemTime) -> String {
    uas_cot::Iso8601(micros(t)).to_string()
}

fn micros(t: SystemTime) -> u64 {
    t.duration_since(UNIX_EPOCH).unwrap_or_default().as_micros() as u64
}

/// Render an event as a standalone CoT XML document.
pub fn event_xml(e: &CotEvent) -> String {
    let time_us = micros(e.time);
    let event = uas_cot::Event {
        how: e.how.into(),
        stale_us: micros(e.time + e.stale),
        point: uas_cot::Point {
            lat: e.lat,
            lon: e.lon,
            hae: e.hae,
            ce: e.ce,
            le: e.le,
        },
        callsign: e.callsign.clone(),
        track: e
            .track
            .map(|(course, speed)| uas_cot::Track { course, speed }),
        links: e
            .link
            .iter()
            .map(|(uid, kind)| uas_cot::Link {
                uid: uid.clone(),
                kind: kind.clone(),
                relation: "p-p".into(),
            })
            .collect(),
        remarks: e.remarks.clone(),
        ..uas_cot::Event::new(e.uid.as_str(), e.cot_type.as_str(), time_us, 0)
    };
    event.to_xml()
}

pub const DRONE_TYPE: &str = "a-f-A-M-H-Q";
pub const POI_TYPE: &str = "b-m-p-s-m";
/// A manned aircraft: neutral, air, civilian fixed-wing.
pub const AIRCRAFT_TYPE: &str = "a-n-A-C-F";

/// The CoT type for something a drone saw.
pub fn sighting_type(target_kind: &str) -> &'static str {
    match target_kind {
        "person" => "a-u-G-U-C-I",
        "vehicle" => "a-u-G-E-V",
        "intruder" => "a-u-A-M-H-Q",
        "poi" => POI_TYPE,
        _ => "a-u-G",
    }
}

pub fn drone_uid(sysid: u8) -> String {
    format!("{UID_PREFIX}-uav-{sysid}")
}

/// Own-position events for every drone in a view.
pub fn drone_events(view: &Published, now: SystemTime, stale: Duration) -> Vec<CotEvent> {
    drones_in(&view.cells)
        .into_values()
        .map(|d| CotEvent {
            uid: drone_uid(d.sysid),
            cot_type: DRONE_TYPE.into(),
            how: "m-g",
            time: now,
            stale,
            lat: d.pose.lat(),
            lon: d.pose.lon(),
            hae: d.pose.alt_m(),
            ce: 5.0,
            le: 5.0,
            callsign: Some(d.callsign.clone()),
            track: Some((
                f64::from(d.pose.hdg_cdeg) / 100.0,
                f64::from(d.pose.spd_cms) / 100.0,
            )),
            remarks: Some(format!(
                "{} · {} · battery {:.0}% · mission item {}/{} · sim tick {}",
                d.callsign,
                d.mode.name(),
                d.battery_pct,
                d.mission_seq,
                d.mission.len(),
                view.meta.tick
            )),
            link: None,
        })
        .collect()
}

/// Tracks for every manned aircraft flying its route in a view. One parked
/// between flights sends nothing, and its last track goes stale.
pub fn aircraft_events(view: &Published, now: SystemTime, stale: Duration) -> Vec<CotEvent> {
    view.cells
        .iter()
        .flat_map(|bag| &bag.objects)
        .filter(|o| {
            o.kind == "aircraft"
                && o.state.pointer("/flight/active") == Some(&serde_json::Value::Bool(true))
        })
        .map(|o| {
            let int = |key: &str| o.state.get(key).and_then(|v| v.as_i64()).unwrap_or(0) as f64;
            let callsign = o
                .state
                .get("callsign")
                .and_then(|v| v.as_str())
                .filter(|c| !c.is_empty())
                .map_or_else(|| format!("Aircraft {}", o.id), str::to_owned);
            CotEvent {
                uid: format!("{UID_PREFIX}-ac-{}", o.id),
                cot_type: AIRCRAFT_TYPE.into(),
                how: "m-g",
                time: now,
                stale,
                lat: int("lat_e7") * 1e-7,
                lon: int("lon_e7") * 1e-7,
                hae: int("alt_mm") * 1e-3,
                ce: 10.0,
                le: 10.0,
                callsign: Some(callsign.clone()),
                track: Some((int("hdg_cdeg") / 100.0, int("spd_cms") / 100.0)),
                remarks: Some(format!(
                    "{callsign} · manned aircraft · sim tick {}",
                    view.meta.tick
                )),
                link: None,
            }
        })
        .collect()
}

/// Spot markers for every point of interest in a view.
pub fn poi_events(view: &Published, now: SystemTime, stale: Duration) -> Vec<CotEvent> {
    view.cells
        .iter()
        .flat_map(|bag| &bag.objects)
        .filter(|o| o.kind == "poi")
        .map(|o| {
            let e7 =
                |key: &str| o.state.get(key).and_then(|v| v.as_i64()).unwrap_or(0) as f64 * 1e-7;
            let name = o
                .state
                .get("callsign")
                .and_then(|v| v.as_str())
                .map_or_else(|| format!("POI {}", o.id), str::to_owned);
            CotEvent {
                uid: format!("{UID_PREFIX}-poi-{}", o.id),
                cot_type: POI_TYPE.into(),
                how: "h-g-i-g-o",
                time: now,
                stale,
                lat: e7("lat_e7"),
                lon: e7("lon_e7"),
                hae: 0.0,
                ce: 1.0,
                le: 1.0,
                callsign: Some(name),
                track: None,
                remarks: Some("Point of interest".into()),
                link: None,
            }
        })
        .collect()
}

/// A sighting report as a CoT event, linked to the drone that made it.
pub fn sighting_event(
    report: &Report,
    observer: Option<(u8, &str)>,
    now: SystemTime,
    stale: Duration,
) -> Option<CotEvent> {
    if report.kind != ReportKind::Detection {
        return None;
    }
    let target = report.target?;
    let detail = &report.detail;
    let range = detail.get("range_m").and_then(|v| v.as_i64());
    let name = detail.get("callsign").and_then(|v| v.as_str());
    let (uid, callsign) = if report.target_kind == "poi" {
        // A POI already has a marker; a sighting refreshes it.
        (
            format!("{UID_PREFIX}-poi-{target}"),
            name.map_or_else(|| format!("POI {target}"), str::to_owned),
        )
    } else {
        let label = match report.target_kind.as_str() {
            "person" => "Person",
            "vehicle" => "Vehicle",
            "intruder" => "Unknown UAV",
            _ => "Contact",
        };
        (
            format!("{UID_PREFIX}-det-{target}"),
            name.map_or_else(|| format!("{label} {target}"), str::to_owned),
        )
    };
    let seen_by = observer.map_or_else(|| "a drone".to_owned(), |(_, c)| c.to_owned());
    let remarks = match range {
        Some(r) => format!("Seen by {seen_by} at {r} m · sim tick {}", report.tick),
        None => format!("Seen by {seen_by} · sim tick {}", report.tick),
    };
    let track = match (
        detail.get("course_cdeg").and_then(|v| v.as_f64()),
        detail.get("speed_cms").and_then(|v| v.as_f64()),
    ) {
        (Some(c), Some(s)) => Some((c / 100.0, s / 100.0)),
        _ => None,
    };
    Some(CotEvent {
        uid,
        cot_type: sighting_type(&report.target_kind).into(),
        how: "m-s",
        time: now,
        stale,
        lat: report.lat(),
        lon: report.lon(),
        hae: f64::from(report.alt_mm) / 1000.0,
        ce: 10.0,
        le: 10.0,
        callsign: Some(callsign),
        track,
        remarks: Some(remarks),
        link: observer.map(|(sysid, _)| (drone_uid(sysid), DRONE_TYPE.to_owned())),
    })
}

pub struct Emitter {
    socket: UdpSocket,
    cfg: CotConfig,
    handle: DriverHandle,
    last_report: u64,
}

impl Emitter {
    pub async fn bind(cfg: CotConfig, handle: DriverHandle) -> std::io::Result<Self> {
        let local = SocketAddr::from((cfg.iface.unwrap_or(Ipv4Addr::UNSPECIFIED), 0));
        let socket = UdpSocket::bind(local).await?;
        if cfg.dest.ip().is_multicast() {
            socket.set_multicast_ttl_v4(cfg.ttl)?;
            socket.set_multicast_loop_v4(true)?;
        }
        let last_report = handle.view().reports.last().map_or(0, |r| r.seq);
        Ok(Emitter {
            socket,
            cfg,
            handle,
            last_report,
        })
    }

    async fn send(&self, events: impl IntoIterator<Item = CotEvent>) {
        for event in events {
            let xml = event_xml(&event);
            if let Err(err) = self.socket.send_to(xml.as_bytes(), self.cfg.dest).await {
                tracing::debug!(%err, dest = %self.cfg.dest, "CoT send failed");
            }
        }
    }

    /// Serve until `shutdown` flips to true.
    pub async fn run(mut self, mut shutdown: watch::Receiver<bool>) -> anyhow::Result<()> {
        tracing::info!(dest = %self.cfg.dest, ttl = self.cfg.ttl, "CoT emitter up");
        let drone_period = Duration::from_secs_f64(1.0 / self.cfg.drone_hz.clamp(0.05, 20.0));
        let mut drones = tokio::time::interval(drone_period);
        let mut pois = tokio::time::interval(Duration::from_secs(15));
        let mut sightings = tokio::time::interval(Duration::from_millis(200));
        for timer in [&mut drones, &mut pois, &mut sightings] {
            timer.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Delay);
        }
        let drone_stale = (drone_period * 5).max(Duration::from_secs(10));

        loop {
            tokio::select! {
                changed = shutdown.changed() => {
                    if changed.is_err() || *shutdown.borrow() {
                        break;
                    }
                }
                _ = drones.tick() => {
                    let view = self.handle.view();
                    self.send(drone_events(&view, SystemTime::now(), drone_stale)).await;
                    self.send(aircraft_events(&view, SystemTime::now(), drone_stale)).await;
                }
                _ = pois.tick() => {
                    let view = self.handle.view();
                    self.send(poi_events(&view, SystemTime::now(), Duration::from_secs(600))).await;
                }
                _ = sightings.tick() => self.send_sightings().await,
            }
        }
        tracing::info!("CoT emitter stopped");
        Ok(())
    }

    async fn send_sightings(&mut self) {
        let view = self.handle.view();
        let fresh = view.reports_since(self.last_report);
        let Some(last) = fresh.last() else { return };
        if let Some(first) = fresh.first() {
            if first.seq > self.last_report + 1 && self.last_report > 0 {
                tracing::warn!(
                    missed = first.seq - self.last_report - 1,
                    "CoT emitter fell behind the report ring"
                );
            }
        }
        self.last_report = last.seq;

        let drones = drones_in(&view.cells);
        let callsign_of = |agent: u64| {
            drones
                .values()
                .find(|d| d.id == agent)
                .map(|d| (d.sysid, d.callsign.as_str()))
        };
        let now = SystemTime::now();
        let events: Vec<CotEvent> = fresh
            .iter()
            .filter_map(|r| {
                let observer = r.observer.and_then(callsign_of);
                let stale = if r.target_kind == "poi" {
                    Duration::from_secs(600)
                } else {
                    Duration::from_secs(30)
                };
                sighting_event(r, observer, now, stale)
            })
            .collect();
        self.send(events).await;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn times_are_utc_with_milliseconds() {
        assert_eq!(cot_time(UNIX_EPOCH), "1970-01-01T00:00:00.000Z");
        let t = UNIX_EPOCH + Duration::from_millis(1_790_246_096_789);
        assert_eq!(cot_time(t), "2026-09-24T10:34:56.789Z");
        // 2000-02-29, a leap day in a century year.
        let t = UNIX_EPOCH + Duration::from_secs(951_782_400);
        assert_eq!(cot_time(t), "2000-02-29T00:00:00.000Z");
    }

    #[test]
    fn a_sighting_renders_as_the_expected_cot_document() {
        let event = CotEvent {
            uid: "krabshell-det-42".into(),
            cot_type: "a-u-G-U-C-I".into(),
            how: "m-s",
            time: UNIX_EPOCH + Duration::from_secs(1_790_246_096),
            stale: Duration::from_secs(30),
            lat: 51.5072,
            lon: -0.1276,
            hae: 0.0,
            ce: 10.0,
            le: 10.0,
            callsign: Some("Person <42> & co".into()),
            track: Some((90.0, 1.4)),
            remarks: Some("Seen by HAWK-1 at 38 m".into()),
            link: Some(("krabshell-uav-1".into(), DRONE_TYPE.into())),
        };
        assert_eq!(
            event_xml(&event),
            concat!(
                r#"<?xml version="1.0" encoding="UTF-8"?>"#,
                r#"<event version="2.0" uid="krabshell-det-42" type="a-u-G-U-C-I" how="m-s" "#,
                r#"time="2026-09-24T10:34:56.000Z" start="2026-09-24T10:34:56.000Z" stale="2026-09-24T10:35:26.000Z">"#,
                r#"<point lat="51.5072000" lon="-0.1276000" hae="0.0" ce="10.0" le="10.0"/>"#,
                r#"<detail><contact callsign="Person &lt;42&gt; &amp; co"/>"#,
                r#"<track course="90.0" speed="1.40"/>"#,
                r#"<link uid="krabshell-uav-1" type="a-f-A-M-H-Q" relation="p-p"/>"#,
                r#"<remarks>Seen by HAWK-1 at 38 m</remarks></detail></event>"#,
            )
        );
    }

    #[test]
    fn a_heading_just_short_of_north_stays_inside_the_track_schema() {
        // hdg_cdeg 35_999 is 359.99 degrees. CoT's track.xsd caps course
        // below 360, so it must go on the wire as 0.0, not 360.0.
        let event = CotEvent {
            uid: "krabshell-uav-1".into(),
            cot_type: DRONE_TYPE.into(),
            how: "m-g",
            time: UNIX_EPOCH,
            stale: Duration::from_secs(10),
            lat: 51.5,
            lon: -0.1,
            hae: 100.0,
            ce: 5.0,
            le: 5.0,
            callsign: None,
            track: Some((35_999.0 / 100.0, 3.0)),
            remarks: None,
            link: None,
        };
        let xml = event_xml(&event);
        assert!(
            xml.contains(r#"<track course="0.0" speed="3.00"/>"#),
            "{xml}"
        );
    }

    #[test]
    fn each_target_kind_has_its_own_type() {
        assert_eq!(sighting_type("person"), "a-u-G-U-C-I");
        assert_eq!(sighting_type("vehicle"), "a-u-G-E-V");
        assert_eq!(sighting_type("intruder"), "a-u-A-M-H-Q");
        assert_eq!(sighting_type("poi"), "b-m-p-s-m");
    }
}
