//! `krabshell`: the simulation driver and its HTTP edge in one process.
//!
//! Start-up order matters. The driver is spawned first and the world is built
//! before the listener binds, so the server never serves a half-loaded scenario.
//! Shutdown runs the same order backwards: stop accepting, pause, optionally
//! checkpoint, then join the tick thread (spec §12).
//!
//! Beside the HTTP edge run the drone gateways: MAVLink over UDP for ground
//! stations, and Cursor-on-Target over UDP multicast for TAK. Both only read
//! the published view and submit intents, like every other edge. They stop
//! with the HTTP server, before the driver pauses.

use std::net::{IpAddr, Ipv4Addr, SocketAddr};
use std::path::PathBuf;

use clap::Parser;
use sim_axum::auth::Auth;
use sim_axum::{router, AppState};
use sim_core::ScenarioBlob;
use sim_engine::{checkpoint, Control, Driver, ModelRegistry, ScenarioStore};
use tokio::sync::watch;

#[derive(Parser)]
#[command(
    name = "krabshell",
    version,
    about = "Operator server for krABMaga simulations"
)]
struct Cli {
    /// Address to bind. Anything but loopback requires DASHBOARD_TOKEN.
    #[arg(long, default_value = "127.0.0.1:8080", env = "KRABSHELL_BIND")]
    bind: SocketAddr,

    /// Directory of scenario packs.
    #[arg(long, default_value = "scenarios", env = "KRABSHELL_SCENARIOS")]
    scenarios: PathBuf,

    /// Scenario to load at start-up. Without it the server starts empty.
    #[arg(long, env = "KRABSHELL_SCENARIO")]
    scenario: Option<String>,

    /// Run the start-up scenario on a different model than its manifest names.
    #[arg(long)]
    model: Option<String>,

    /// Start running immediately instead of paused.
    #[arg(long)]
    run: bool,

    /// Write a checkpoint here on a clean shutdown.
    #[arg(long)]
    checkpoint_on_exit: Option<PathBuf>,

    /// UDP address the MAVLink gateway listens on. MAVLink has no
    /// authentication: anything that can reach this port can fly the drones.
    #[arg(
        long,
        default_value = "127.0.0.1:14551",
        env = "KRABSHELL_MAVLINK_BIND"
    )]
    mavlink_bind: SocketAddr,

    /// Ground station to send telemetry to without waiting to hear from it.
    /// Repeatable. QGroundControl listens on 14550.
    #[arg(
        long,
        default_value = "127.0.0.1:14550",
        env = "KRABSHELL_MAVLINK_GCS",
        value_delimiter = ','
    )]
    mavlink_gcs: Vec<SocketAddr>,

    /// Position and attitude telemetry rate per drone, Hz.
    #[arg(long, default_value_t = 5.0)]
    mavlink_hz: f64,

    /// Do not start the MAVLink gateway.
    #[arg(long)]
    no_mavlink: bool,

    /// Where Cursor-on-Target events go: ATAK's SA multicast group by default,
    /// or any unicast host:port.
    #[arg(long, default_value = "239.2.3.1:6969", env = "KRABSHELL_COT_DEST")]
    cot_dest: SocketAddr,

    /// Multicast TTL for CoT. 1 keeps it on the local network.
    #[arg(long, default_value_t = 1)]
    cot_ttl: u32,

    /// Local IPv4 address of the interface to send CoT from.
    #[arg(long, env = "KRABSHELL_COT_IFACE")]
    cot_iface: Option<Ipv4Addr>,

    /// Do not start the CoT emitter.
    #[arg(long)]
    no_cot: bool,
}

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    tracing_subscriber::fmt()
        .with_env_filter(
            tracing_subscriber::EnvFilter::try_from_default_env()
                .unwrap_or_else(|_| "info,tower_http=warn".into()),
        )
        .init();

    let cli = Cli::parse();
    let auth = Auth::from_env();

    // Refusing to expose an unauthenticated server is a hard rule, not a
    // warning: the dashboard can reset a world and upload scenario packs.
    if auth.is_open() && !is_loopback(cli.bind.ip()) {
        anyhow::bail!(
            "refusing to bind {} without a token: set DASHBOARD_TOKEN, or bind 127.0.0.1",
            cli.bind
        );
    }

    let store = ScenarioStore::new(&cli.scenarios);
    let blob = match &cli.scenario {
        Some(id) => store.load(id).map_err(|err| {
            anyhow::anyhow!(
                "could not load scenario '{id}' from {}: {err}",
                cli.scenarios.display()
            )
        })?,
        None => ScenarioBlob::empty(),
    };
    let blob = match &cli.model {
        Some(model) => ScenarioBlob {
            manifest: sim_core::ScenarioManifest {
                model: model.clone(),
                ..blob.manifest
            },
            ..blob
        },
        None => blob,
    };

    tracing::info!(
        scenario = %blob.manifest.id,
        model = %blob.manifest.model,
        resolution = blob.config.grid.resolution,
        engine = sim_engine::engine_description(),
        "building the world"
    );

    let driver = Driver::spawn(ModelRegistry::with_builtins(), blob)?;
    let handle = driver.handle();

    if cli.run {
        handle.command(Control::Start { target_tps: None })?;
    }

    let state = AppState::new(handle.clone(), store, auth);

    // One signal stops the HTTP server and both gateways.
    let (stop_tx, stop_rx) = watch::channel(false);
    let mut edges = Vec::new();

    if !cli.no_mavlink {
        if !is_loopback(cli.mavlink_bind.ip()) {
            tracing::warn!(
                bind = %cli.mavlink_bind,
                "MAVLink is unauthenticated: anything that can reach this port can command the drones"
            );
        }
        let cfg = sim_mavlink::MavlinkConfig {
            bind: cli.mavlink_bind,
            gcs: cli.mavlink_gcs.clone(),
            telemetry_hz: cli.mavlink_hz,
        };
        let gateway = sim_mavlink::Gateway::bind(cfg, handle.clone(), state.limiter.clone())
            .await
            .map_err(|err| {
                anyhow::anyhow!(
                    "could not bind the MAVLink gateway to {}: {err} (use --mavlink-bind or --no-mavlink)",
                    cli.mavlink_bind
                )
            })?;
        edges.push(tokio::spawn(gateway.run(stop_rx.clone())));
    }

    if !cli.no_cot {
        let cfg = sim_cot::CotConfig {
            dest: cli.cot_dest,
            ttl: cli.cot_ttl,
            iface: cli.cot_iface,
            ..sim_cot::CotConfig::default()
        };
        let emitter = sim_cot::Emitter::bind(cfg, handle.clone())
            .await
            .map_err(|err| {
                anyhow::anyhow!("could not start the CoT emitter: {err} (use --no-cot)")
            })?;
        edges.push(tokio::spawn(emitter.run(stop_rx.clone())));
    }

    let listener = tokio::net::TcpListener::bind(cli.bind).await?;
    let bound = listener.local_addr()?;
    tracing::info!(%bound, "dashboard on http://{bound}/");

    tokio::spawn(async move {
        shutdown_signal().await;
        let _ = stop_tx.send(true);
    });
    let mut http_stop = stop_rx.clone();
    let server = axum::serve(listener, router(state)).with_graceful_shutdown(async move {
        let _ = http_stop.wait_for(|stop| *stop).await;
    });
    if let Err(err) = server.await {
        tracing::error!(%err, "the http server stopped");
    }

    for edge in edges {
        match edge.await {
            Ok(Ok(())) => {}
            Ok(Err(err)) => tracing::error!(%err, "a gateway stopped with an error"),
            Err(err) => tracing::error!(%err, "a gateway task panicked"),
        }
    }

    // Pause first: a checkpoint of a moving world would be a different world by
    // the time it is written.
    tracing::info!("pausing the driver");
    let _ = handle.command(Control::Pause);

    if let Some(path) = cli.checkpoint_on_exit {
        match handle.checkpoint() {
            Ok(snapshot) => match checkpoint::write_to(&path, &snapshot) {
                Ok(()) => {
                    tracing::info!(path = %path.display(), tick = snapshot.meta.tick, "checkpoint written")
                }
                Err(err) => tracing::error!(%err, "could not write the shutdown checkpoint"),
            },
            Err(err) => tracing::error!(%err, "could not take a shutdown checkpoint"),
        }
    }

    tracing::info!("joining the driver");
    driver.shutdown();
    Ok(())
}

fn is_loopback(ip: IpAddr) -> bool {
    match ip {
        IpAddr::V4(v4) => v4.is_loopback(),
        IpAddr::V6(v6) => v6.is_loopback(),
    }
}

async fn shutdown_signal() {
    let ctrl_c = async {
        tokio::signal::ctrl_c()
            .await
            .expect("failed to listen for ctrl-c");
    };

    #[cfg(unix)]
    let terminate = async {
        tokio::signal::unix::signal(tokio::signal::unix::SignalKind::terminate())
            .expect("failed to listen for SIGTERM")
            .recv()
            .await;
    };

    #[cfg(not(unix))]
    let terminate = std::future::pending::<()>();

    tokio::select! {
        _ = ctrl_c => tracing::info!("interrupt received"),
        _ = terminate => tracing::info!("terminate received"),
    }
}
