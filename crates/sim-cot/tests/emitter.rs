//! The emitter against a running simulation, received on a loopback socket.

use std::collections::BTreeSet;
use std::path::Path;
use std::time::{Duration, Instant};

use sim_cot::{CotConfig, Emitter};
use sim_engine::{Control, Driver, ModelRegistry};
use tokio::net::UdpSocket;
use tokio::sync::watch;

fn attr<'a>(xml: &'a str, name: &str) -> Option<&'a str> {
    let key = format!(r#" {name}=""#);
    let start = xml.find(&key)? + key.len();
    let end = xml[start..].find('"')?;
    Some(&xml[start..start + end])
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn drones_pois_and_sightings_arrive_as_cot_events() {
    let pack = Path::new(env!("CARGO_MANIFEST_DIR")).join("../../scenarios/london-patrol");
    let blob = sim_engine::read_pack(&pack).expect("london-patrol pack");
    let driver = Driver::spawn(ModelRegistry::with_builtins(), blob).unwrap();
    let handle = driver.handle();
    handle.command(Control::Start { target_tps: None }).unwrap();

    let receiver = UdpSocket::bind("127.0.0.1:0").await.unwrap();
    let cfg = CotConfig {
        dest: receiver.local_addr().unwrap(),
        drone_hz: 5.0,
        ..CotConfig::default()
    };
    let emitter = Emitter::bind(cfg, handle.clone()).await.unwrap();
    let (stop_tx, stop_rx) = watch::channel(false);
    let task = tokio::spawn(emitter.run(stop_rx));

    let mut drones = BTreeSet::new();
    let mut pois = BTreeSet::new();
    let mut sighting_types = BTreeSet::new();
    let mut linked = false;
    let deadline = Instant::now() + Duration::from_secs(10);
    let mut buf = vec![0u8; 65_536];
    while Instant::now() < deadline && !(drones.len() == 4 && pois.len() == 10 && linked) {
        let Ok(Ok(n)) = tokio::time::timeout(Duration::from_secs(2), receiver.recv(&mut buf)).await
        else {
            continue;
        };
        let xml = std::str::from_utf8(&buf[..n]).expect("utf-8");
        assert!(xml.starts_with("<?xml"), "{xml}");
        assert!(xml.ends_with("</event>"), "{xml}");
        let cot_type = attr(xml, "type").unwrap();
        let callsign = attr(xml, "callsign").unwrap_or_default().to_owned();
        match cot_type {
            "a-f-A-M-H-Q" => {
                drones.insert(callsign);
            }
            "b-m-p-s-m" => {
                pois.insert(callsign);
            }
            other if other.starts_with("a-u-") => {
                sighting_types.insert(other.to_owned());
                if xml.contains(r#"relation="p-p""#) && xml.contains("krabshell-uav-") {
                    linked = true;
                }
            }
            other => panic!("unexpected type {other}"),
        }
    }

    assert_eq!(
        drones,
        BTreeSet::from(["HAWK-1", "HAWK-2", "HAWK-3", "UAV-4"].map(String::from))
    );
    assert!(pois.contains("Big Ben"), "{pois:?}");
    assert_eq!(pois.len(), 10, "{pois:?}");
    assert!(
        linked,
        "no sighting linked to a drone; saw {sighting_types:?}"
    );

    stop_tx.send(true).unwrap();
    task.await.unwrap().unwrap();
    driver.shutdown();
}
