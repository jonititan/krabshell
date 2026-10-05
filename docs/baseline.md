# Speed baseline

Spec §14 Phase 0 asks for a recorded baseline that the server must not destroy.
These are the first numbers. They are a floor to regress against, not a
performance claim.

## Conditions

- 24 logical CPUs, WSL2 (Linux 6.6), shared with an editor and other work.
  Load average during runs was 4–5, and single runs of the same binary varied
  by up to 2×. Every figure below is a **median of interleaved runs**, so both
  backends saw the same noise.
- `cargo build --release` (thin LTO, one codegen unit).
- krABMaga 0.6.2 from crates.io, `a5` 0.7.3, Rust 1.95.
- Model: the reference random walkers (`crates/sim-engine/src/models`).

Re-run with:

```sh
cargo build --release -p sim-engine --bin simctl
target/release/simctl --scenarios bench/scenarios bench london-walkers --warmup 200 --ticks 1500
target/release/simctl --scenarios bench/scenarios --model walkers-krabmaga bench london-walkers --warmup 200 --ticks 1500
```

## Phase 0 gate

`gis` + `parallel` compile and run. A standalone probe moving 10 000 objects on
a `SparseA5Grid` for 200 ticks measured **≈44 ns per move, field only** (no
model, no driver). Confirmed `bevy_a5::GeoCell` is `Copy + Eq + Hash + Send + Sync`.

Enabling `gis` links all of Bevy 0.18 (≈610 crates) even though the grid only
needs `bevy_a5` maths. That is the risk in spec §16. The server's own crates
still use the `a5` crate directly for cell maths, but krABMaga is no longer
behind a feature: the drones model runs on it, so every build links Bevy.

## Driver throughput, headless

| Pack | Agents | Backend | Ticks/s (median) | ns per agent-tick |
|---|---:|---|---:|---:|
| `london-walkers` (gold) | 2 002 | built-in grid | 1 554 (n=5) | ≈320 |
| `london-walkers` (gold) | 2 002 | krABMaga `SparseA5Grid` | 2 312 (n=5) | ≈215 |
| `london-walkers` (gold) | 2 002 | krABMaga, `parallel` feature | 1 710 (n=5) | ≈290 |
| `global-drift` (fast) | 20 000 | built-in grid | 134 (n=3) | ≈375 |
| `global-drift` (fast) | 20 000 | krABMaga `SparseA5Grid` | 196 (n=3) | ≈255 |
| `global-drift` (fast) | 20 000 | krABMaga, `parallel` feature | 130 (n=3) | ≈385 |

Observations:

- **krABMaga's single-threaded grid is ~1.5× faster than the built-in grid.**
  The built-in grid copies both of its maps into the write side on every tick
  so unmoved agents persist; krABMaga clears the write side instead and relies
  on agents re-placing themselves. That copy is the first optimisation target.
- **The `parallel` grid is slower than either** in this workload. It pays for
  sharded, locked maps while the step itself is still sequential — intra-step
  parallelism is spec Phase 6. Do not advertise `parallel` as a speed-up yet.
- Gold replay agrees across backends: both produce hash `41979b9086558a23`
  after 300 ticks of `london-walkers`, and a unit test checks they agree
  move-for-move including intents.
- Per-agent cost is dominated by the walk itself (a cached neighbour lookup
  and two hash-map writes), not by the driver. The driver loop overhead is not
  separately visible at these sizes.

## Cost of the HTTP edge

Spec §17 criterion 2: driver throughput within 10 % of headless with the
dashboard open. Interleaved rounds on `london-walkers`, built-in grid, server
running at maximum speed with `publish_hz = 20` and a client polling
`/api/v1/state/map` once a second:

| Round | Headless ticks/s | Server ticks/s |
|---:|---:|---:|
| 1 | 1 527 | 1 547 |
| 2 | 1 568 | 1 519 |
| 3 | 1 538 | 1 543 |

The difference is inside run-to-run noise (≤3 %). Publishing is a pointer
swap on the tick thread plus one snapshot copy per publish, so its cost scales
with `publish_hz` and world size, not with the number of readers.

Not yet measured (spec Phase 5): 10 and 50 WebSocket subscribers, and a
checkpoint of 10⁵ agents while running.

## Drones on krABMaga

The drones model moved from a hand-rolled loop over a `BTreeMap` of entities
on the built-in grid to krABMaga agents on a `Schedule` and a `SparseA5Grid`.
Medians of three interleaved runs of 3 000 ticks, on 32 logical CPUs (Linux 7.0,
shared with an editor):

| Pack | Agents | Built-in loop (`simctl`, before) | krABMaga (`simctl`) | krABMaga, views built off the tick thread (`simctl`) | krABMaga alone (`krab-drones`) |
|---|---:|---:|---:|---:|---:|
| `london-patrol` (gold) | 1 365 | 2 739 | 2 041 | 2 217 | 2 280 |
| `kaunas-deliveries` (gold) | 1 844 | 1 499 | 1 165 | 1 311 | 1 326 |
| `swarm-survey` (fast) | 5 042 | 491 | 363 | 465 | 476 |

Ticks per second, publishing at the packs' 20 Hz. Observations:

- **What krABMaga costs is 80–100 ns per agent per tick.** With publishing
  at 1 Hz the step alone runs at 2 900 / 1 630 / 599 ticks/s before and
  2 204 / 1 303 / 462 on krABMaga. Timed separately for `swarm-survey`: the
  schedule's heap pop and push ≈ 55 ns (the queue's hasher makes no
  difference), every agent re-placing itself on the grid and the commit
  dropping every cell's bag ≈ 20 ns, cloning the board ≈ 14 ns, the
  occupied-cell set ≈ 5 ns. The agents' own work, ≈ 325 ns, is unchanged.
- **Publishing used to cost about a fifth of the tick thread.** A view of
  `swarm-survey` takes ≈ 3.5 ms to build (every agent's state as JSON),
  and the driver builds 20 a second. The drones now hand the driver a frame
  (an `Arc` of the committed board) and a publisher thread builds the view,
  so the tick thread runs as fast at 20 Hz as at 1 Hz. The old loop paid that
  fifth, which is why the gap narrows most for the biggest pack.
- **`krab-drones` publishes nothing**, so it is the step alone.
- **`parallel` is slower again** (≈990 ticks/s for `london-patrol` alone): it
  pays for the locked grid and a mutex around the state while the schedule
  still runs on one thread, and gives the same gold hash.

Re-run with:

```sh
cargo build --release -p sim-engine -p sim-drones --bins
target/release/simctl run london-patrol --ticks 3000
target/release/krab-drones scenarios/london-patrol --steps 3000
```

## Parallel drones: krABMaga's schedule vs a crowd

Branch `experiment/parallel-threads`. `krab-drones` alone, 3 000 ticks,
32 logical CPUs.

**krABMaga's `parallel` schedule with more threads** (`--features parallel
--threads N`), `kaunas-deliveries`: 1 thread 606 ticks/s, 4 threads 333,
16 threads 276, against 1 339 for the default build. Its step takes one
mutex around the whole `State` for every agent's `step`, so agents still run
one at a time and the extra threads only add contention. It also deals
agents round-robin to threads that interleave freely, so the gold hash
changes from run to run (three different hashes at 4 threads), and it steps
every queued agent regardless of its scheduled time.

**A crowd** (`--crowd N`, [`crowd.rs`](../crates/sim-drones/src/crowd.rs)):
every agent but the dispatcher steps inside one krABMaga agent. Work an agent
does alone (wandering, flying, scanning the committed board) runs on a rayon
pool; the parts that touch shared state (order queues, reports, the sensor's
memory, placing on the grid) run after it in id order. Gold hashes and the
full report stream are byte-identical to the schedule's at every thread
count (checked over 36 000 ticks of `kaunas-deliveries`, and by
`a_crowd_steps_exactly_as_the_schedule_does` with spawns, despawns and
mail).

Medians of three interleaved runs, ticks per second:

| Pack | Agents | Schedule | Crowd 1 | 2 | 4 | 8 | 16 |
|---|---:|---:|---:|---:|---:|---:|---:|
| `kaunas-deliveries` | 1 844 | 1 332 | 1 437 | 2 251 | 2 885 | **3 032** (2.3×) | 2 723 |
| `london-patrol` | 1 365 | 2 244 | 2 354 | 3 511 | 4 292 | **4 837** (2.2×) | 4 542 |
| `swarm-survey` | 5 042 | 473 | 529 | 926 | 1 410 | 1 745 | **1 792** (3.8×) |

- **One crowd thread is already ≈ 8 % faster**: one heap pop per tick
  instead of one per agent.
- **Deliveries flatten past 4–8 threads.** At ≈ 330 µs a tick the id-order
  phases (placing 1 844 agents on the grid, the commit) and two pool
  round-trips are a growing share. The bigger `swarm-survey` keeps scaling to
  16.

Re-run with:

```sh
cargo build --release -p sim-drones --bin krab-drones
target/release/krab-drones scenarios/kaunas-deliveries --steps 3000 --crowd 8
```
