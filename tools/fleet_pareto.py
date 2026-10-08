# /// script
# requires-python = ">=3.11"
# ///
"""Sweep the kaunas-deliveries fleet size and find the Pareto frontier.

Each configuration is a copy of scenarios/kaunas-deliveries with the first
`mega` couriers at Mega and the first `akro` at Akropolis (the spares stay),
flown with `simctl fly` for HOURS simulated hours. Orders come from the
pack's seed, the same orders at the same ticks whatever the fleet, so every
configuration is judged on the same orders: those placed in the first
WINDOW hours that are not turned away for a geofence.

For each configuration:

- wait: order to delivery, mean and 90th percentile, over those orders. An
  order not delivered by the end of the run counts as waiting until the end,
  so for a fleet that falls behind this is a lower bound.
- fleet size: couriers, spares not counted.
- flying hours: drone-hours in the air over the whole run (simctl's
  flight_drone_seconds), and per delivery.

A configuration is on the frontier if no other is at least as good on all
three (mean wait, fleet size, flying hours) and better on one. Writes
results.csv and results.json to the output folder; chart them with
tools/fleet_pareto_chart.py.

    cargo build --release -p sim-engine --bin simctl
    uv run tools/fleet_pareto.py                       # the default grid
    uv run tools/fleet_pareto.py --mega 4,8 --akro 6,12 --jobs 4
"""

import argparse
import csv
import json
import os
import shutil
import subprocess
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
PACK = ROOT / "scenarios" / "kaunas-deliveries"
SIMCTL = ROOT / "target" / "release" / "simctl"
HOURS = 9.0
WINDOW = 8.0
MEGA = [2, 3, 4, 6, 8, 10, 12, 15, 20]
AKRO = [3, 4, 6, 8, 10, 12, 15, 18, 20]


def ints(s: str) -> list[int]:
    return [int(x) for x in s.split(",") if x]


def make_pack(work: Path, mega: int, akro: int) -> str:
    """A copy of the pack with `mega` and `akro` couriers. Its id."""
    pid = f"fleet-m{mega:02d}-a{akro:02d}"
    out = work / "packs" / pid
    if out.exists():
        shutil.rmtree(out)
    (out / "world").mkdir(parents=True)
    for f in ("config.toml", "params.toml"):
        shutil.copy(PACK / f, out / f)
    manifest = (PACK / "manifest.toml").read_text()
    manifest = manifest.replace('id = "kaunas-deliveries"', f'id = "{pid}"')
    (out / "manifest.toml").write_text(manifest)
    for f in ("fields.json", "geofences.json"):
        os.symlink(PACK / "world" / f, out / "world" / f)
    keep = {"MEGA": mega, "AKRO": akro}
    agents = []
    for a in json.loads((PACK / "world" / "agents.json").read_text()):
        if a["kind"] == "drone" and "depot" in a["state"]:
            n = int(a["state"]["callsign"].split("-")[1])
            if n > keep[a["state"]["depot"]]:
                continue
        agents.append(a)
    (out / "world" / "agents.json").write_text(json.dumps(agents))
    return pid


def fly(work: Path, pid: str) -> Path:
    log = work / "runs" / pid
    if (log / "summary.json").exists():
        return log
    subprocess.run(
        [str(SIMCTL), "--scenarios", str(work / "packs"), "fly", pid,
         "--hours", str(HOURS), "--out", str(log)],
        check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
    )
    return log


def deliveries(log: Path) -> dict[int, tuple[int, float]]:
    """order -> (ordered_tick, order to delivery in seconds)."""
    with open(log / "deliveries.csv") as f:
        return {int(r["order"]): (int(r["ordered_tick"]), float(r["total_s"])) for r in csv.DictReader(f)}


def pareto(rows: list[dict], keys: list[str]) -> None:
    for r in rows:
        r["pareto"] = not any(
            all(o[k] <= r[k] for k in keys) and any(o[k] < r[k] for k in keys)
            for o in rows if o is not r
        )


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--mega", type=ints, default=MEGA, help="couriers at Mega, comma-separated")
    ap.add_argument("--akro", type=ints, default=AKRO, help="couriers at Akropolis, comma-separated")
    ap.add_argument("--jobs", type=int, default=max(1, (os.cpu_count() or 2) - 2))
    ap.add_argument("--out", type=Path, default=ROOT / "var" / "fleet-pareto")
    args = ap.parse_args()
    if not SIMCTL.exists():
        raise SystemExit(f"{SIMCTL} missing: cargo build --release -p sim-engine --bin simctl")

    configs = sorted({(m, a) for m in args.mega for a in args.akro} | {(20, 20)})
    args.out.mkdir(parents=True, exist_ok=True)
    pids = {c: make_pack(args.out, *c) for c in configs}
    with ThreadPoolExecutor(args.jobs) as pool:
        logs = dict(zip(configs, pool.map(lambda c: fly(args.out, pids[c]), configs)))

    tick_s = 0.1
    end_tick = round(HOURS * 3600 / tick_s)
    # The orders to judge on: the full fleet delivers every one placed in the
    # window long before the run ends.
    full = deliveries(logs[(20, 20)])
    judged = {o: t for o, (t, _) in full.items() if t < WINDOW * 3600 / tick_s}

    rows = []
    for (mega, akro), log in logs.items():
        s = json.loads((log / "summary.json").read_text())
        got = deliveries(log)
        waits = []
        for o, t in judged.items():
            if o in got:
                assert got[o][0] == t, f"order {o} placed at a different tick in {log.name}"
                waits.append(got[o][1])
            else:
                waits.append((end_tick - t) * tick_s)
        waits.sort()
        flying_h = s["flight_drone_seconds"] / 3600
        rows.append({
            "mega": mega, "akro": akro, "fleet": mega + akro,
            "orders": len(waits),
            "undelivered": sum(o not in got for o in judged),
            "deliveries": s["deliveries"],
            "mean_wait_min": sum(waits) / len(waits) / 60,
            "p90_wait_min": waits[int(0.9 * (len(waits) - 1))] / 60,
            "flying_h": flying_h,
            "flying_min_per_delivery": flying_h * 60 / max(1, s["deliveries"]),
            "crashes": s["crashes"],
        })
    rows.sort(key=lambda r: (r["fleet"], r["mega"]))
    pareto(rows, ["mean_wait_min", "fleet", "flying_h"])

    with open(args.out / "results.csv", "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)
    (args.out / "results.json").write_text(json.dumps(
        {"hours": HOURS, "window_h": WINDOW, "orders": len(judged), "rows": rows}, indent=1))
    print(f"{len(rows)} configurations, {len(judged)} orders judged, results in {args.out}")
    for r in rows:
        if r["pareto"]:
            print(f"  M{r['mega']:>2} A{r['akro']:>2}  fleet {r['fleet']:>2}  "
                  f"wait {r['mean_wait_min']:6.1f} min (p90 {r['p90_wait_min']:6.1f})  "
                  f"flying {r['flying_h']:5.1f} h  undelivered {r['undelivered']}")


if __name__ == "__main__":
    main()
