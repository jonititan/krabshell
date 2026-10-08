//! Run a drones scenario pack on krABMaga alone: no krabshell driver, server
//! or gateways. The pack is read from disk, built into a [`World`] (a krABMaga
//! `State`) and stepped on a krABMaga `Schedule` the way `simulate!` does it.
//! `--tui` hands it to `simulate!` itself, with krABMaga's terminal monitor.
//!
//! ```text
//! cargo run --release -p sim-drones --bin krab-drones -- scenarios/kaunas-deliveries --steps 36000
//! ```

use std::fs::File;
use std::io::BufWriter;
use std::path::PathBuf;
use std::time::Instant;

use anyhow::Context;
use clap::Parser;
use krabmaga::engine::state::State;
use sim_drones::World;

#[derive(Parser)]
#[command(name = "krab-drones", about = "Run a drones scenario pack on krABMaga")]
struct Cli {
    /// Scenario pack directory, e.g. scenarios/london-patrol.
    pack: PathBuf,
    /// Steps per repetition. A step is the pack's `tick_seconds` of
    /// simulated time.
    #[arg(long, default_value_t = 10_000)]
    steps: u64,
    /// Repetitions. Each starts the pack afresh; the summary is of the last.
    #[arg(long, default_value_t = 1)]
    reps: u64,
    /// Run under krABMaga's `simulate!` with its terminal monitor, and print
    /// no summary.
    #[arg(long)]
    tui: bool,
    /// Write every report (detections, waypoints, deliveries) here, one JSON
    /// object per line.
    #[arg(long)]
    reports: Option<PathBuf>,
}

fn main() -> anyhow::Result<()> {
    let cli = Cli::parse();
    let blob = sim_core::read_pack(&cli.pack)
        .with_context(|| format!("reading the pack at {}", cli.pack.display()))?;
    if blob.manifest.model != "drones" {
        anyhow::bail!(
            "'{}' is a '{}' pack; this runs 'drones' packs",
            blob.manifest.id,
            blob.manifest.model
        );
    }
    let mut world = World::new(&blob)?;
    // Nobody drains them here; they are counted, and written out if asked.
    world.keep_reports(false);
    if let Some(path) = &cli.reports {
        let file = File::create(path).with_context(|| format!("creating {}", path.display()))?;
        world.report_to(Box::new(BufWriter::new(file)));
    }

    println!(
        "running '{}' for {} steps × {} on krABMaga",
        blob.manifest.id, cli.steps, cli.reps
    );
    println!(
        "brain:         {} nodes ({})",
        world.brain().size(),
        if blob.fields.get("behaviour").is_some() {
            "the pack's world/behaviour.json"
        } else {
            "the default behaviour tree"
        }
    );
    if cli.tui {
        #[cfg(not(feature = "parallel"))]
        {
            run_with_monitor(world, cli.steps, cli.reps);
            return Ok(());
        }
        #[cfg(feature = "parallel")]
        anyhow::bail!("--tui needs a build without the parallel feature");
    }
    let started = Instant::now();
    run(&mut world, cli.steps, cli.reps);
    let wall = started.elapsed().as_secs_f64();
    world.flush_reports()?;

    let mut cells = Vec::new();
    world.snapshot(&mut cells, usize::MAX);
    let steps = cli.steps * cli.reps;
    println!("tick:          {}", world.tick());
    println!("agents:        {}", world.agent_count());
    println!("occupied:      {} cells", world.occupied_cells());
    println!("wall clock:    {wall:.3} s");
    println!("steps/second:  {:.0}", steps as f64 / wall.max(1e-9));
    let reports: Vec<String> = world
        .report_counts()
        .iter()
        .map(|(kind, count)| format!("{count} {kind}"))
        .collect();
    if !reports.is_empty() {
        println!("reports:       {}", reports.join(", "));
    }
    let dispatch = world.dispatch();
    if !world.book().depots.is_empty() {
        println!(
            "orders:        {} placed, {} delivered, {} queued",
            dispatch.placed,
            dispatch.delivered,
            dispatch.queued()
        );
    }
    if !world.stations().is_empty() {
        println!("stations:      {}", world.stations().list.len());
    }
    if !world.regions().is_empty() {
        let surveys: u32 = world.regions().list.iter().map(|r| r.surveys).sum();
        println!(
            "regions:       {} of interest, {surveys} surveys",
            world.regions().list.len()
        );
    }
    println!(
        "gold hash:     {:016x}",
        sim_core::snapshot::stable_hash(&cells)
    );
    Ok(())
}

/// What `simulate!` does without its terminal monitor: initialise, then step
/// the schedule, for each repetition.
fn run(world: &mut World, steps: u64, reps: u64) {
    for _ in 0..reps {
        let mut schedule = sim_drones::schedule();
        world.init(&mut schedule);
        for _ in 0..steps {
            schedule.step(world);
            if world.end_condition(&mut schedule) {
                break;
            }
        }
    }
}

/// krABMaga's `simulate!` with its terminal monitor. It runs the world on a
/// thread of its own and keeps it, so there is no summary afterwards.
#[cfg(not(feature = "parallel"))]
fn run_with_monitor(world: World, steps: u64, reps: u64) {
    use krabmaga::simulate;
    simulate!(world, steps, reps, true);
}
