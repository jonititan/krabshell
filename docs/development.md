# Development

krabshell is a Cargo workspace of eight crates plus a directory of dashboard templates and assets, all compiled into one `krabshell` binary and two command-line tools. This page describes what each crate is for, how to build and test the workspace, how to run the drones model on krABMaga alone, and the things that must be kept in step by hand: the OpenAPI document, the dashboard's compiled-in assets and these docs. The design reasoning behind the crate boundaries is in [Architecture notes](architecture.md).

## Workspace layout

```text
crates/
  sim-core/       types and A5 cell maths shared by everything
  sim-drones/     the drones model as krABMaga agents; krab-drones
  sim-engine/     the driver thread, the model seam, built-in models; simctl
  sim-mavlink/    MAVLink gateway
  sim-cot/        Cursor-on-Target emitter
  sim-weather/    wind from Open-Meteo per A5 cell, cached on disk
  sim-conflict/   conflict detection and resolution advisories
  sim-axum/       HTTP and WebSocket edge, dashboard routes; krabshell
  sim-dashboard/  Askama templates and static assets (not a Cargo crate)
scenarios/        drone packs: iron-wolf, kaunas-deliveries, london-patrol, swarm-survey
bench/scenarios/  random-walker packs: london-walkers, global-drift
docs/             these guides
tools/            Python scripts for building packs and checking the gateways
```

### sim-core

Types only: no Bevy, Axum, Tokio or krABMaga. It is the vocabulary every other layer speaks: the A5 address space (`GeoCell`, over the pure-maths `a5` crate), the intent schema clients submit, the snapshot shape the driver publishes, checkpoints, the run configuration, scenario manifests and reading pack directories (`read_pack`), WebSocket messages, problem codes and the deterministic RNG. Source: [`crates/sim-core/src/lib.rs`](../crates/sim-core/src/lib.rs).

### sim-drones

The drones model, as a krABMaga model. Every drone, intruder, manned aircraft, person, vehicle and point of interest is a krABMaga agent on a krABMaga `Schedule`, and `World` is the `State` they step against, with a `SparseA5Grid` as occupancy of record, double-buffered the krABMaga way. A `Dispatcher` agent steps first each tick to open and close geofences, place delivery orders and, every `atc_every_s`, assess conflicts with `sim-conflict` (`World::run_atc`, [`atc.rs`](../crates/sim-drones/src/atc.rs)); the drones and aircraft advised fly the advisories. What each drone does is decided by a behaviour tree (`behaviour/drone.json` by default). Poses are integers and every random draw comes from a (seed, entity, tick) stream, so gold runs replay bit for bit. It does not depend on the driver, Axum or Tokio (of krabshell's crates it uses only `sim-core` and `sim-conflict`), and has its own binary, `krab-drones`. Source: [`crates/sim-drones/src/lib.rs`](../crates/sim-drones/src/lib.rs).

### sim-engine

The simulation side of krabshell. One thread owns the world and runs it as fast as the model allows; nothing in this crate awaits, opens a socket or reads a file during a tick. It holds the `Driver` and its control, intent and snapshot channels, the publisher thread, the `Model` trait and `ModelRegistry`, the built-in models (`drones`, wrapping `sim_drones::World`; `walkers` on a built-in grid; `walkers-krabmaga` on krABMaga's), intent admission, the rewind history (a `sim-history` thread that keeps keyframes in the `gold` profile and rebuilds past ticks; [`history.rs`](../crates/sim-engine/src/history.rs)), the edge checks every transport shares (cell bounds and the per-actor rate limiter), the scenario store, checkpoint encoding, the drones run log, and the `simctl` binary. Source: [`crates/sim-engine/src/lib.rs`](../crates/sim-engine/src/lib.rs).

### sim-mavlink

The MAVLink gateway: every simulated drone presented to ground stations as its own vehicle (its `sysid`, component 1) on one UDP socket. It speaks the real-time half of the protocol itself (heartbeats, telemetry cadence, the mission handshake and its timeouts, on the wall clock) and turns what ground stations ask for into intents. It never touches model state, never steps the schedule and never blocks on the driver. Source: [`crates/sim-mavlink/src/lib.rs`](../crates/sim-mavlink/src/lib.rs).

### sim-cot

The Cursor-on-Target emitter, for ATAK, WinTAK and TAK Server: drones and manned aircraft once a second, sightings as the model reports them, points of interest every 15 seconds, one XML event per UDP datagram. Event times are wall-clock UTC so TAK clients never see them arrive stale; the simulation tick goes in `<remarks>`. Source: [`crates/sim-cot/src/lib.rs`](../crates/sim-cot/src/lib.rs).

### sim-weather

Wind for a world, from Open-Meteo, one series per A5 cell, at 10, 80, 120 and 180 m with gusts at 10 m, hourly. `recorded` weather is fetched once for a fixed window; `live` weather is the latest forecast, refreshed by the server. Fetched fields are cached on disk, keyed by what was asked. The `openmeteo-rs` dependency is pinned exactly; read its changelog before moving it. Source: [`crates/sim-weather/src/lib.rs`](../crates/sim-weather/src/lib.rs).

### sim-conflict

Probabilistic conflict detection and resolution advisories for aircraft on A5 cells. `assess` predicts each aircraft as a seeded cloud of particles flown under an energy model (BADA's Total Energy Model for fixed-wing aircraft, an excess-power envelope for multirotors), sums the probabilities in each cell and altitude layer, flags cells where two or more aircraft reach the threshold, and tries climbs, descents, level-offs, turns and speed changes to advise what removes the most risk. Each aircraft has a giving-way order (`GiveWay`: advised first, last or never) and may carry a protected volume (`Protection`) that others must keep out of. Predictions are flown in parallel with rayon and collected in order, so it is deterministic. It has no dependency on the rest of krabshell beyond `sim-core`. Source: [`crates/sim-conflict/src/lib.rs`](../crates/sim-conflict/src/lib.rs).

```sh
cargo run --release -p sim-conflict --example head_on
```

The `figures` example writes the data behind the figures in `docs/figures` as JSON; see `conflict_figures.py` under [Tools](#tools).

### sim-axum

The HTTP edge and the operator dashboard, and the `krabshell` binary that wires everything together. Nothing here owns the world: every route reads the driver's published snapshot or posts a message to it, so a slow client, a large download or a wedged browser cannot cost the simulation a tick. It also holds auth, the weather service, the conflict service (which uses `sim_drones::atc` so its assessment matches the world's own), the run logger, the hand-written OpenAPI document and the rendered docs. Source: [`crates/sim-axum/src/lib.rs`](../crates/sim-axum/src/lib.rs), [`main.rs`](../crates/sim-axum/src/main.rs).

### sim-dashboard

Not a Cargo crate: a directory of Askama templates (`templates/`) and static files (`assets/`) that `sim-axum` compiles in. See [Dashboard templates and assets](#dashboard-templates-and-assets).

## Building

Rust 1.95 or later. Every build links krABMaga with its `gis` feature and so all of Bevy, which on Linux needs:

```sh
sudo apt-get install libasound2-dev libudev-dev libwayland-dev libfontconfig1-dev libxkbcommon-dev pkg-config
```

```sh
cargo build --release                                  # everything
cargo build --release -p sim-axum --bin krabshell      # the server
cargo build --release -p sim-engine --bin simctl       # headless runner
cargo build --release -p sim-drones --bin krab-drones  # the model on krABMaga alone
```

Profiles are set in the workspace `Cargo.toml`. Release builds use thin LTO and one codegen unit. Dev builds still compile `sim-engine`, `sim-drones` and `a5` at `opt-level = 3`, so a `cargo run` without `--release` does not ship a debug-speed tick loop by accident.

The `parallel` feature switches on krABMaga's parallel grid and schedule; see [Operating the server](operating.md#cargo-features).

## Tests

```sh
cargo test --workspace
cargo test --workspace --features sim-axum/parallel,sim-drones/parallel
cargo clippy --workspace --all-targets
cargo fmt --all --check
```

Every test binary links Bevy. With full debug info they are large enough to fill a small disk, so CI sets:

```sh
CARGO_PROFILE_DEV_DEBUG=0 CARGO_PROFILE_TEST_DEBUG=0 cargo test --workspace
```

Run one suite with `--test`, for example `cargo test -p sim-axum --test openapi`.

### Notable suites

| Suite | What it checks |
|---|---|
| `sim-engine/tests/driver.rs` | The tick thread end to end over its real channels: a paused world stays put, a panicking model does not take the process with it, an intent lands on the next tick. |
| `sim-engine/tests/history.rs` | Rewind: a tick rebuilt from the history, intents and all, is the world as it was; the window is kept and starts again with a new world; the `fast` profile has none. |
| `sim-engine/src/models/drones/tests.rs` | The drones model through the driver's `Model` seam. |
| `sim-drones/src/tests.rs` | The model on krABMaga alone. |
| `sim-axum/tests/api.rs` | Every route against a real router and a real driver, so a handler that answers from a stale view fails here rather than in a browser. |
| `sim-axum/tests/ws.rs` | The live protocol over a real socket on an ephemeral port. |
| `sim-axum/tests/openapi.rs` | The OpenAPI document against the server; see [Keeping openapi.json in sync](#keeping-openapijson-in-sync). |
| `sim-axum/tests/weather.rs` | Loading packs that ask for weather, against a stand-in for Open-Meteo: recorded weather is fetched once then cached, a failed fetch refuses a recorded load, live weather refreshes into the running world. |
| `sim-weather/tests/fetch.rs` | What is asked of Open-Meteo, how the answer becomes a field, and that the cache spares a second call. |
| `sim-mavlink/tests/gateway.rs` | A fake ground station over loopback UDP in MAVLink v2: heartbeats, mission upload and download, commands and acks, `MISSION_ITEM_REACHED`. |
| `sim-cot/tests/emitter.rs` | The emitter against a running simulation, received on a loopback socket. |
| `sim-conflict/tests/encounters.rs` | Whole encounters through `assess`: what is a conflict, and what is advised. |

Unit tests sit beside the code in most modules.

### CI

[`.github/workflows/ci.yml`](../.github/workflows/ci.yml) runs on every push and pull request on the default build (the `parallel` job is commented out for now), with `RUSTFLAGS=-D warnings`: formatting (default build only), clippy, the workspace tests, and three gold replays:

```sh
cargo run --release -p sim-engine --bin simctl -- replay london-patrol --ticks 600
cargo run --release -p sim-drones --bin krab-drones -- scenarios/london-patrol --steps 600   # twice; the gold hashes must match
cargo run --release -p sim-engine --bin simctl -- --scenarios bench/scenarios --model walkers-krabmaga replay london-walkers --ticks 300
```

## Benchmarks

There is no `cargo bench`. Speed is measured with `simctl bench` on the walker packs in `bench/scenarios` and with `krab-drones` on the drone packs. The method, the machine and the numbers to regress against are in [Speed baseline](baseline.md). On a shared machine (WSL2 in particular), interleave runs of the things being compared and report medians.

## Running the drones model without krabshell

`krab-drones` runs a drones pack on krABMaga alone: no driver, server, gateways or dashboard. It reads the pack, builds the `World`, and steps it on a krABMaga `Schedule` the way `simulate!` does. Everything the drones do (behaviour trees, geofences, stations, regions of interest, deliveries) is in the model, so it all runs the same here. Source: [`krab-drones.rs`](../crates/sim-drones/src/bin/krab-drones.rs).

```sh
cargo run --release -p sim-drones --bin krab-drones -- scenarios/kaunas-deliveries --steps 36000
cargo run --release -p sim-drones --bin krab-drones -- scenarios/london-patrol --reports reports.jsonl
cargo run --release -p sim-drones --bin krab-drones -- scenarios/london-patrol --tui
```

| Argument | Default | Meaning |
|---|---|---|
| `PACK` | required | Pack directory, such as `scenarios/london-patrol`. Its manifest must name the `drones` model. |
| `--steps N` | 10 000 | Steps per repetition. A step is the pack's `tick_seconds` of simulated time. |
| `--reps N` | 1 | Repetitions, each starting the pack afresh. The summary is of the last. |
| `--reports PATH` | none | Write every report (detections, waypoints, deliveries and the rest) as JSON Lines. |
| `--tui` | off | Hand the world to krABMaga's `simulate!` with its terminal monitor, and print no summary. Not available in a `parallel` build. |

Without `--tui` it prints the behaviour tree's size, the tick, agents, occupied cells, steps per second, reports by kind, orders, stations, each depot's delivery range, regions and surveys, and the gold hash.

To embed the model in your own krABMaga program, depend on `sim-drones`, which re-exports `krabmaga`:

```rust
use sim_drones::krabmaga::engine::state::State;

let blob = sim_core::read_pack("scenarios/london-patrol".as_ref())?;
let mut world = sim_drones::World::new(&blob)?;
let mut schedule = sim_drones::schedule();
world.init(&mut schedule);
for _ in 0..1_000 {
    schedule.step(&mut world);
}
```

## Keeping openapi.json in sync

The API reference at [`/docs/api`](/docs/api), and the document served at `/api/v1/openapi.json`, both come from [`crates/sim-axum/openapi.json`](../crates/sim-axum/openapi.json). It is written by hand, because response bodies are built from types across several crates and some are the model's own JSON, and it is compiled into the binary with `include_str!`.

`crates/sim-axum/tests/openapi.rs` keeps it honest. It fails when:

- a route declared in [`lib.rs`](../crates/sim-axum/src/lib.rs) is not described, or a described operation is not routed. The test finds routes by reading `lib.rs` for `.route("path", method(...))` calls, so API routes must be declared there in that form;
- a described operation does not reach a handler;
- the document or the docs page asks for a token;
- a live response, for every scenario pack, does not match its schema, including a response that has grown a field the schema does not list.

When you add or change an API route: declare it in `router()` in `lib.rs`, describe it in `openapi.json` (path, parameters, request body and response schemas), and run:

```sh
cargo test -p sim-axum --test openapi
```

## Dashboard templates and assets

The dashboard is server-rendered: Askama templates and htmx, with no JavaScript build step and no Node toolchain.

- **Templates** are in [`crates/sim-dashboard/templates/`](../crates/sim-dashboard/templates/), with partials for htmx swaps in `fragments/`. [`crates/sim-axum/askama.toml`](../crates/sim-axum/askama.toml) points Askama there, and they are compiled into `sim-axum` at build time; a template error is a compile error.
- **Assets** are in [`crates/sim-dashboard/assets/`](../crates/sim-dashboard/assets/): `app.css`, the map and page scripts, and vendored `htmx.min.js` and `milsymbol.js` (3.0.4). Each is pulled in with `include_str!` in [`routes/pages.rs`](../crates/sim-axum/src/routes/pages.rs) and served under `/assets/`. A new asset needs both an `include_str!` and a route there. milsymbol is served at a versioned path, `/assets/milsymbol-3.0.4.js`, and cached for good; a new version needs a new path.

Because everything is compiled in, the dashboard works with no internet (except optional basemap tiles, which the browser fetches), and a change to a template or asset takes a rebuild to show.

## Docs

These guides are the Markdown files in `docs/`, plus `krabshell_spec.md`, listed in `SOURCES` in [`crates/sim-axum/src/docs.rs`](../crates/sim-axum/src/docs.rs) and compiled in with `include_str!`. The server renders each at `/docs/<slug>`, the slug being the file name without `.md`. A new page needs an entry in `SOURCES`, and an edit takes a rebuild to show. Link between pages with relative paths (`operating.md`, `operating.md#authentication`) so they work on GitHub too; the server rewrites them to `/docs/...`, and links to source files to the repository.

### As a PDF

`tools/docs_pdf.py` exports the same guides, in the same order and groups, as one PDF: a cover, a contents index of every guide and its sections with page numbers, PDF bookmarks for every heading, and page numbers on every page. Links between guides become links inside the PDF; links to source files go to GitHub; the figures are embedded. It drives the installed Google Chrome through Playwright, so it needs Chrome but no LaTeX or pandoc:

```sh
uv run --with markdown --with playwright --with pypdf tools/docs_pdf.py   # target/krabshell-docs.pdf
uv run --with markdown --with playwright --with pypdf tools/docs_pdf.py docs.pdf
```

It prints twice: the first pass reads the page each heading landed on from the PDF's bookmarks, the second puts those numbers in the contents. The cover gives the date and the commit it was made from.

## Tools

`tools/` holds Python scripts, run with [uv](https://docs.astral.sh/uv/) (`uv run tools/<script>.py`):

| Script | What it does |
|---|---|
| `kaunas_deliveries.py` | Builds the world of `scenarios/kaunas-deliveries`: drones, terrain and the address book. |
| `lt_geozones.py` | Fetches Lithuania's published UAS geographical zones over Kaunas into that pack's `world/geofences.json`, leaving hand-drawn fences alone. |
| `fleet_pareto.py`, `fleet_pareto_chart.py` | Sweep the Kaunas fleet size with `simctl fly` and chart the Pareto frontier of delivery wait against fleet size. |
| `mavlink_check.py` | Acts as a ground station against a running krabshell and flies one drone. |
| `cot_listen.py` | Prints the CoT events krabshell sends, as a TAK client would see them. No dependencies. |
| `docs_pdf.py` | Exports these guides as one PDF with a contents index; see [As a PDF](#as-a-pdf). |
| `conflict_figures.py` | Draws the conflict figures in `docs/figures` from the `sim-conflict` `figures` example. |
