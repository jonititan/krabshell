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
cargo build --release -p sim-engine --features krabmaga --bin simctl
target/release/simctl --scenarios bench/scenarios bench london-walkers --warmup 200 --ticks 1500
target/release/simctl --scenarios bench/scenarios --model walkers-krabmaga bench london-walkers --warmup 200 --ticks 1500
```

## Phase 0 gate

`gis` + `parallel` compile and run. A standalone probe moving 10 000 objects on
a `SparseA5Grid` for 200 ticks measured **≈44 ns per move, field only** (no
model, no driver). Confirmed `bevy_a5::GeoCell` is `Copy + Eq + Hash + Send + Sync`.

Enabling `gis` links all of Bevy 0.18 (≈610 crates) even though the grid only
needs `bevy_a5` maths. That is the risk in spec §16; it is why the server's own
crates use the `a5` crate directly and keep krABMaga behind a feature.

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
