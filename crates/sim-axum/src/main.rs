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
//! with the HTTP server, before the driver pauses. So does the run logger
//! (flight heatmap, deliveries) for packs that ask for one.

use std::net::{IpAddr, Ipv4Addr, SocketAddr};
use std::path::PathBuf;
use std::sync::Arc;

use clap::Parser;
use sim_axum::auth::Auth;
use sim_axum::weather::WeatherService;
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

    /// Another origin that may change things, as `scheme://host[:port]`:
    /// the public address when a reverse proxy rewrites the Host. Repeatable.
    /// The server's own origin needs no listing.
    #[arg(long, env = "KRABSHELL_ALLOWED_ORIGINS", value_delimiter = ',')]
    allowed_origin: Vec<String>,

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

    /// Where run logs (flight heatmap, deliveries) go, one folder per run, for
    /// scenarios that set `heatmap_resolution` or `order_rate_per_address_day`.
    #[arg(long, default_value = "var/logs", env = "KRABSHELL_LOG_DIR")]
    log_dir: PathBuf,

    /// How often run logs are rewritten, seconds of wall-clock time.
    #[arg(long, default_value_t = 60)]
    log_every: u64,

    /// Do not write run logs.
    #[arg(long)]
    no_logs: bool,

    /// Where fetched weather is cached, so a scenario run again does not
    /// fetch it again.
    #[arg(
        long,
        default_value = "var/weather-cache",
        env = "KRABSHELL_WEATHER_CACHE"
    )]
    weather_cache: PathBuf,

    /// Never fetch weather. Scenarios asking for recorded weather then refuse
    /// to load; live ones fly in still air; a pack's own `world/weather.json`
    /// still works.
    #[arg(long)]
    no_weather: bool,

    /// Open-Meteo API key. The free API is for non-commercial use only.
    #[arg(long, env = "OPEN_METEO_API_KEY", hide_env_values = true)]
    open_meteo_key: Option<String>,

    /// Root URL of an Open-Meteo forecast API to use instead of the public
    /// one (a self-hosted instance, say).
    #[arg(long, env = "KRABSHELL_OPEN_METEO_URL")]
    open_meteo_url: Option<String>,

    /// Root URL of an Open-Meteo historical forecast API, likewise.
    #[arg(long, env = "KRABSHELL_OPEN_METEO_HISTORICAL_URL")]
    open_meteo_historical_url: Option<String>,
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
    let auth = Auth::from_env().with_origins(cli.allowed_origin.clone());

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
    let mut blob = match &cli.model {
        Some(model) => ScenarioBlob {
            manifest: sim_core::ScenarioManifest {
                model: model.clone(),
                ..blob.manifest
            },
            ..blob
        },
        None => blob,
    };

    let weather = Arc::new(WeatherService::new(if cli.no_weather {
        None
    } else {
        let hosts = sim_weather::fetch::Hosts {
            api_key: cli.open_meteo_key.clone(),
            forecast: cli.open_meteo_url.clone(),
            historical: cli.open_meteo_historical_url.clone(),
        };
        Some(sim_weather::Fetcher::new(
            sim_weather::Cache::new(&cli.weather_cache),
            hosts,
        )?)
    }));
    weather
        .prepare(&mut blob)
        .await
        .map_err(|err| anyhow::anyhow!("scenario '{}': {err}", blob.manifest.id))?;

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

    let state = AppState::new(handle.clone(), store, auth).with_weather(Arc::clone(&weather));

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

    let weather_task = tokio::spawn(Arc::clone(&weather).run(handle.clone(), stop_rx.clone()));
    let conflict_task = tokio::spawn(Arc::clone(&state.conflicts).run(
        handle.clone(),
        Arc::clone(&weather),
        stop_rx.clone(),
    ));

    if !cli.no_logs {
        let cfg = sim_axum::runlog::RunLogConfig {
            dir: cli.log_dir.clone(),
            every: std::time::Duration::from_secs(cli.log_every),
        };
        edges.push(tokio::spawn(sim_axum::runlog::run(
            handle.clone(),
            cfg,
            stop_rx.clone(),
        )));
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

    let _ = weather_task.await;
    let _ = conflict_task.await;
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
