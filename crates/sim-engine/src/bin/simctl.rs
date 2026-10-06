//! Headless driver CLI (spec §14, Phase 1).
//!
//! This is how the speed baseline gets measured. It runs the same driver the
//! server runs, with no HTTP and no subscribers, so the difference between
//! `simctl bench` and the same scenario under `krabshell` is exactly the cost
//! the edge adds.

use std::path::PathBuf;
use std::time::{Duration, Instant};

use clap::{Parser, Subcommand};
use sim_core::{Profile, ScenarioBlob};
use sim_engine::{checkpoint, Control, Driver, DriverHandle, ModelRegistry, ScenarioStore};

#[derive(Parser)]
#[command(name = "simctl", about = "Run a krabshell scenario without a server")]
struct Cli {
    /// Directory of scenario packs.
    #[arg(long, default_value = "scenarios", global = true)]
    scenarios: PathBuf,

    /// Run the pack on a different model than its manifest names, e.g.
    /// `walkers-krabmaga` to compare occupancy backends on the same world.
    #[arg(long, global = true)]
    model: Option<String>,

    #[command(subcommand)]
    command: Command,
}

#[derive(Subcommand)]
enum Command {
    /// List the packs in the scenario directory.
    List,
    /// Run a scenario for a fixed number of ticks and report the rate.
    Run {
        /// Manifest id of the pack to run.
        scenario: String,
        #[arg(long, default_value_t = 10_000)]
        ticks: u64,
        /// Cap the tick rate. Omit for maximum speed.
        #[arg(long)]
        tps: Option<f64>,
        /// Write a checkpoint here when the run finishes.
        #[arg(long)]
        checkpoint: Option<PathBuf>,
        /// Force a determinism profile, overriding the pack.
        #[arg(long, value_parser = parse_profile)]
        profile: Option<Profile>,
    },
    /// Measure ticks per second, discarding a warm-up.
    Bench {
        scenario: String,
        #[arg(long, default_value_t = 1_000)]
        warmup: u64,
        #[arg(long, default_value_t = 20_000)]
        ticks: u64,
        /// Run at maximum speed for this many seconds instead of a fixed
        /// number of ticks. Big worlds tick slowly, so a fixed wall-clock
        /// budget keeps a sweep over world sizes to a predictable length.
        #[arg(long, conflicts_with = "ticks")]
        seconds: Option<f64>,
    },
    /// Run a scenario twice and check the two runs agree. Requires `gold`.
    Replay {
        scenario: String,
        #[arg(long, default_value_t = 500)]
        ticks: u64,
    },
    /// Fly a drones scenario for some simulated hours as fast as it goes and
    /// write its run log: flight heatmap, deliveries, a histogram of time
    /// from order to delivery, and one of battery left at each landing.
    #[command(alias = "heatmap")]
    Fly {
        scenario: String,
        /// Simulated hours to run.
        #[arg(long, default_value_t = 8.0)]
        hours: f64,
        /// Simulated seconds between samples. The heatmap follows the straight
        /// line between samples, so this only blurs corners at waypoints.
        #[arg(long, default_value_t = 5.0)]
        sample_s: f64,
        /// Output folder. Defaults to var/logs/<scenario>-<hours>h.
        #[arg(long)]
        out: Option<PathBuf>,
    },
    /// Look up the A5 cell at a coordinate. Use it to fill in a manifest `root`.
    Cell {
        #[arg(long, allow_hyphen_values = true)]
        lon: f64,
        #[arg(long, allow_hyphen_values = true)]
        lat: f64,
        #[arg(long, default_value_t = 9)]
        res: i32,
    },
}

fn parse_profile(text: &str) -> Result<Profile, String> {
    match text {
        "gold" => Ok(Profile::Gold),
        "fast" => Ok(Profile::Fast),
        other => Err(format!("expected 'gold' or 'fast', got '{other}'")),
    }
}

fn main() -> anyhow::Result<()> {
    let cli = Cli::parse();
    let store = ScenarioStore::new(&cli.scenarios);
    let model_override = cli.model.clone();
    let load = |id: &str| -> anyhow::Result<ScenarioBlob> {
        let mut blob = store.load(id)?;
        if let Some(model) = &model_override {
            blob.manifest.model = model.clone();
        }
        Ok(blob)
    };

    match cli.command {
        Command::List => {
            let packs = store.list();
            if packs.is_empty() {
                println!("no scenario packs under {}", cli.scenarios.display());
                return Ok(());
            }
            println!("{:<20} {:<16} {:>4}  NAME", "ID", "MODEL", "RES");
            for pack in packs {
                println!(
                    "{:<20} {:<16} {:>4}  {}",
                    pack.id, pack.model, pack.resolution, pack.name
                );
            }
        }

        Command::Fly {
            scenario,
            hours,
            sample_s,
            out,
        } => {
            use sim_engine::models::drones::runlog::{self, RunLog};
            let blob = load(&scenario)?;
            if !runlog::wanted(&blob.config) {
                anyhow::bail!(
                    "'{scenario}' asks for no run log: set heatmap_resolution or order_rate_per_address_day in its params.toml"
                );
            }
            let tick_s = blob
                .config
                .model
                .get("tick_seconds")
                .and_then(sim_core::config::Param::as_f64)
                .filter(|s| *s > 0.0)
                .unwrap_or(0.1);
            let total = (hours * 3600.0 / tick_s).round().max(1.0) as u64;
            let every = (sample_s / tick_s).round().max(1.0) as u64;
            let out = out
                .unwrap_or_else(|| PathBuf::from("var/logs").join(format!("{scenario}-{hours}h")));
            let config = blob.config.clone();
            let (driver, handle) = start(blob)?;
            println!(
                "flying '{scenario}' for {hours} simulated hours ({total} ticks), sampling every {every} ticks"
            );
            let started = Instant::now();
            let mut log = RunLog::new(&scenario, &config, &handle.view());
            log.observe(&handle.view());
            let mut tick = 0;
            let mut next_report = total / 10;
            while tick < total {
                let n = every.min(total - tick);
                handle.command(Control::Step { n })?;
                tick += n;
                // The driver publishes as soon as a step batch ends.
                let view = loop {
                    let view = handle.view();
                    if view.meta.tick >= tick && !view.health.mode.is_advancing() {
                        break view;
                    }
                    if !view.health.alive {
                        anyhow::bail!("the driver stopped at tick {}", view.meta.tick);
                    }
                    std::thread::sleep(Duration::from_micros(50));
                };
                log.observe(&view);
                if tick >= next_report {
                    println!(
                        "  {:>3.0}%  {} deliveries, {} crashes so far",
                        tick as f64 * 100.0 / total as f64,
                        log.deliveries.len(),
                        log.landings.crashes()
                    );
                    next_report += total / 10;
                }
            }
            driver.shutdown();

            std::fs::create_dir_all(&out)?;
            for (name, contents) in log.files() {
                std::fs::write(out.join(name), contents)?;
            }
            println!("wall clock:    {:.1} s", started.elapsed().as_secs_f64());
            if let Some(map) = &log.heatmap {
                println!(
                    "flight:        {:.1} drone-hours",
                    map.drone_seconds() / 3600.0
                );
            }
            if let Some(s) = log.deliveries.summary() {
                println!(
                    "deliveries:    {} (order to delivery: mean {:.1} min, median {:.1}, 90% within {:.1}, slowest {:.1})",
                    s.count,
                    s.mean_s / 60.0,
                    s.median_s / 60.0,
                    s.p90_s / 60.0,
                    s.max_s / 60.0
                );
            }
            if !log.deliveries.rejected.is_empty() {
                println!(
                    "rejected:      {} orders (address inside a geofence)",
                    log.deliveries.rejected.len()
                );
            }
            if !log.landings.is_empty() {
                println!(
                    "flights:       {} ({} crashed)",
                    log.landings.len(),
                    log.landings.crashes()
                );
            }
            if !log.geofences.is_empty() {
                println!(
                    "geofences:     {} opened, {} breaches, {} reroutes",
                    log.geofences.fences.len(),
                    log.geofences.breaches.len(),
                    log.geofences.reroutes.len()
                );
            }
            if log.missed_reports > 0 {
                println!(
                    "warning:       {} reports were dropped before they were read",
                    log.missed_reports
                );
            }
            println!("wrote          {}/", out.display());
        }

        Command::Run {
            scenario,
            ticks,
            tps,
            checkpoint: checkpoint_path,
            profile,
        } => {
            let mut blob = load(&scenario)?;
            if let Some(profile) = profile {
                blob.config.run.profile = profile;
                blob.manifest.profile = profile;
            }
            let (driver, handle) = start(blob)?;

            println!(
                "running '{scenario}' for {ticks} ticks on {}",
                sim_engine::engine_description()
            );
            let started = Instant::now();
            handle.command(Control::SetRate(tps))?;
            handle.command(Control::Step { n: ticks })?;
            await_tick(&handle, ticks)?;
            let elapsed = started.elapsed();

            report(&handle, elapsed);

            if let Some(path) = checkpoint_path {
                let snapshot = handle.checkpoint()?;
                checkpoint::write_to(&path, &snapshot)?;
                println!("checkpoint -> {}", path.display());
            }
            driver.shutdown();
        }

        Command::Bench {
            scenario,
            warmup,
            ticks,
            seconds,
        } => {
            let blob = load(&scenario)?;
            let agents = blob
                .config
                .model
                .get("walkers")
                .and_then(sim_core::config::Param::as_i64)
                .unwrap_or(0);
            let (driver, handle) = start(blob)?;

            handle.command(Control::Step { n: warmup })?;
            await_tick(&handle, warmup)?;

            let (ticks, elapsed) = match seconds {
                None => {
                    let started = Instant::now();
                    handle.command(Control::Step { n: ticks })?;
                    await_tick(&handle, warmup + ticks)?;
                    (ticks, started.elapsed())
                }
                Some(seconds) => {
                    let started = Instant::now();
                    handle.command(Control::Start { target_tps: None })?;
                    std::thread::sleep(Duration::from_secs_f64(seconds));
                    handle.command(Control::Pause)?;
                    // The pause lands between ticks, so the tick in flight when
                    // it was sent is counted; time it to when the driver stops.
                    let (done, stopped) = await_paused(&handle)?;
                    (done - warmup, stopped - started)
                }
            };

            let view = handle.view();
            println!("engine:        {}", sim_engine::engine_description());
            println!("model:         {}", view.model_kind);
            println!("scenario:      {scenario}");
            println!("profile:       {:?}", view.meta.profile);
            println!("agents:        {}", view.meta.agent_count);
            println!("occupied cells: {}", view.meta.occupied_cells);
            println!("seeded from:   walkers = {agents}");
            println!("ticks:         {ticks} (after {warmup} warm-up)");
            println!("wall clock:    {:.3} s", elapsed.as_secs_f64());
            println!("ticks/second:  {:.0}", ticks as f64 / elapsed.as_secs_f64());
            println!(
                "per tick:      {:.1} us",
                elapsed.as_secs_f64() * 1e6 / ticks as f64
            );
            if view.meta.agent_count > 0 {
                println!(
                    "per agent-tick: {:.1} ns",
                    elapsed.as_nanos() as f64 / (ticks * view.meta.agent_count) as f64
                );
            }
            driver.shutdown();
        }

        Command::Replay { scenario, ticks } => {
            let blob = load(&scenario)?;
            if blob.config.run.profile != Profile::Gold {
                anyhow::bail!(
                    "'{scenario}' runs in the fast profile; replay equality only holds for gold"
                );
            }
            let first = run_to_hash(blob.clone(), ticks)?;
            let second = run_to_hash(blob, ticks)?;
            match (first, second) {
                (Some(a), Some(b)) if a == b => {
                    println!("replay matches after {ticks} ticks: hash {a:016x}");
                }
                (Some(a), Some(b)) => {
                    anyhow::bail!("replay diverged: {a:016x} != {b:016x}");
                }
                _ => anyhow::bail!("no hash was produced; is the world larger than the cell cap?"),
            }
        }

        Command::Cell { lon, lat, res } => {
            let cell = sim_core::GeoCell::from_lonlat(lon, lat, res)?;
            let (clon, clat) = cell.centroid()?;
            println!("cell:      {cell}");
            println!("resolution: {}", cell.resolution());
            println!("centroid:  {clon:.6}, {clat:.6}");
            for parent_res in (0..res).rev().take(6) {
                println!("parent r{parent_res}: {}", cell.parent(Some(parent_res))?);
            }
            println!("neighbours: {}", cell.neighbors()?.len());
            println!(
                "edge:      ~{:.0} m",
                sim_core::cell::nominal_cell_edge_meters(res)
            );
        }
    }

    Ok(())
}

fn start(blob: ScenarioBlob) -> anyhow::Result<(Driver, DriverHandle)> {
    let driver = Driver::spawn(ModelRegistry::with_builtins(), blob)?;
    let handle = driver.handle();
    Ok((driver, handle))
}

fn run_to_hash(blob: ScenarioBlob, ticks: u64) -> anyhow::Result<Option<u64>> {
    let (driver, handle) = start(blob)?;
    handle.command(Control::Step { n: ticks })?;
    await_tick(&handle, ticks)?;
    let hash = handle.checkpoint()?.meta.hash;
    driver.shutdown();
    Ok(hash)
}

/// Block until the driver has stopped advancing, and return the tick it
/// stopped on and when it was seen to stop.
fn await_paused(handle: &DriverHandle) -> anyhow::Result<(u64, Instant)> {
    let deadline = Instant::now() + Duration::from_secs(120);
    loop {
        let view = handle.view();
        if !view.health.alive {
            anyhow::bail!("the driver stopped at tick {}", view.meta.tick);
        }
        if !view.health.mode.is_advancing() {
            let stopped = Instant::now();
            // The view is published on a timer; give it one more publish so
            // the tick it reports is the one the driver paused on.
            std::thread::sleep(Duration::from_millis(250));
            return Ok((handle.view().meta.tick, stopped));
        }
        if Instant::now() > deadline {
            anyhow::bail!("the driver did not pause within 120s");
        }
        std::thread::sleep(Duration::from_millis(5));
    }
}

/// Block until the driver reaches `target`, or give up with a useful message.
fn await_tick(handle: &DriverHandle, target: u64) -> anyhow::Result<()> {
    let mut idle_since = Instant::now();
    let mut last_tick = handle.view().meta.tick;
    loop {
        let view = handle.view();
        if view.meta.tick >= target {
            return Ok(());
        }
        if !view.health.alive {
            anyhow::bail!("the driver stopped at tick {}", view.meta.tick);
        }
        if let Some(fault) = &view.health.fault {
            anyhow::bail!(
                "the model panicked at tick {}: {fault}",
                view.health.last_good_tick
            );
        }
        if view.meta.tick != last_tick {
            last_tick = view.meta.tick;
            idle_since = Instant::now();
        } else if idle_since.elapsed() > Duration::from_secs(60) {
            anyhow::bail!("the driver made no progress for 60s at tick {last_tick}");
        }
        std::thread::sleep(Duration::from_millis(2));
    }
}

fn report(handle: &DriverHandle, elapsed: Duration) {
    let view = handle.view();
    println!("tick:          {}", view.meta.tick);
    println!("agents:        {}", view.meta.agent_count);
    println!("occupied:      {} cells", view.meta.occupied_cells);
    println!("wall clock:    {:.3} s", elapsed.as_secs_f64());
    println!(
        "ticks/second:  {:.0}",
        view.meta.tick as f64 / elapsed.as_secs_f64().max(f64::EPSILON)
    );
    println!("last step:     {} ns", view.metrics.last_step_ns);
    if let Some(hash) = view.meta.hash {
        println!("gold hash:     {hash:016x}");
    }
}
