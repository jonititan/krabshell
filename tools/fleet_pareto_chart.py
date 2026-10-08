# /// script
# requires-python = ">=3.11"
# ///
"""Chart the fleet sweep from tools/fleet_pareto.py as one HTML page.

Reads <out>/results.json and writes <out>/pareto.html: delivery wait against
fleet size and against fleet flying hours, every configuration as a dot, the
Pareto-optimal ones filled, and a table of them. Self-contained (no external
scripts), light and dark.

    uv run tools/fleet_pareto_chart.py
    uv run tools/fleet_pareto_chart.py --out var/fleet-pareto
"""

import argparse
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent

PAGE = r"""<title>Kaunas Fleet Pareto</title>
<style>
/* Layout: one reading column; two charts side by side when there is room. */
:root {
  --bg: #f7f8f7; --surface: #fcfcfb; --ink: #0b0b0b; --ink-2: #52514e; --muted: #898781;
  --grid: #e1e0d9; --axis: #c3c2b7; --front: #2a78d6; --front-line: #2a78d6; --dom: #b9b7ae;
  --ring: rgba(11,11,11,0.10);
  --sans: system-ui, -apple-system, "Segoe UI", Roboto, sans-serif;
  --mono: ui-monospace, "SF Mono", Menlo, Consolas, monospace;
}
@media (prefers-color-scheme: dark) { :root:not([data-theme="light"]) {
  --bg: #0d0d0d; --surface: #1a1a19; --ink: #ffffff; --ink-2: #c3c2b7; --muted: #898781;
  --grid: #2c2c2a; --axis: #383835; --front: #3987e5; --front-line: #3987e5; --dom: #5a5953;
  --ring: rgba(255,255,255,0.10); color-scheme: dark; } }
:root[data-theme="dark"] {
  --bg: #0d0d0d; --surface: #1a1a19; --ink: #ffffff; --ink-2: #c3c2b7; --muted: #898781;
  --grid: #2c2c2a; --axis: #383835; --front: #3987e5; --front-line: #3987e5; --dom: #5a5953;
  --ring: rgba(255,255,255,0.10); color-scheme: dark; }
body { background: var(--bg); color: var(--ink); font: 15px/1.5 var(--sans); }
.wrap { max-width: 1280px; margin: 0 auto; padding-inline: 20px; padding-block: 28px 56px; display: grid; gap: 22px; }
h1 { font-size: 24px; margin: 0; letter-spacing: -0.01em; text-wrap: balance; }
h2 { font-size: 13px; margin: 0; text-transform: uppercase; letter-spacing: 0.07em; color: var(--ink-2); font-weight: 600; }
p { margin: 0; max-width: 72ch; color: var(--ink-2); }
.lede { display: grid; gap: 8px; }
.facts { display: flex; flex-wrap: wrap; gap: 8px 28px; font-size: 14px; color: var(--ink-2); }
.facts b { color: var(--ink); font-variant-numeric: tabular-nums; font-weight: 600; }
.bar { display: flex; flex-wrap: wrap; align-items: center; gap: 10px 22px; }
.seg { display: inline-flex; border: 1px solid var(--axis); border-radius: 7px; overflow: hidden; }
.seg button { font: inherit; font-size: 13px; padding: 5px 12px; border: 0; background: transparent; color: var(--ink-2); cursor: pointer; }
.seg button[aria-pressed="true"] { background: var(--ink); color: var(--surface); }
.seg button:focus-visible { outline: 2px solid var(--front); outline-offset: -2px; }
.legend { display: flex; gap: 18px; font-size: 13px; color: var(--ink-2); align-items: center; flex-wrap: wrap; }
.legend span { display: inline-flex; align-items: center; gap: 6px; }
.charts { display: grid; grid-template-columns: repeat(auto-fit, minmax(min(100%, 520px), 1fr)); gap: 18px; }
figure { margin: 0; background: var(--surface); border: 1px solid var(--ring); border-radius: 10px; padding: 16px 16px 10px; min-width: 0; display: grid; gap: 6px; }
figcaption { font-size: 14px; font-weight: 600; }
figure .sub { font-size: 13px; color: var(--muted); }
svg.chart { width: 100%; height: auto; display: block; overflow: visible; }
svg text { fill: var(--muted); font: 12px var(--sans); font-variant-numeric: tabular-nums; }
svg text.lab { fill: var(--ink-2); }
.gridline { stroke: var(--grid); stroke-width: 1; }
.axisline { stroke: var(--axis); stroke-width: 1; }
.envelope { fill: none; stroke: var(--front-line); stroke-width: 2; stroke-linejoin: round; }
.dot { stroke: var(--surface); stroke-width: 2; cursor: pointer; }
.dot.dom { fill: var(--dom); }
.dot.front { fill: var(--front); }
.dot.hot { stroke: var(--ink); }
.tip { position: fixed; pointer-events: none; background: var(--surface); color: var(--ink); border: 1px solid var(--ring);
  box-shadow: 0 4px 16px rgba(0,0,0,0.12); border-radius: 8px; padding: 8px 10px; font-size: 13px; line-height: 1.45; z-index: 5; }
.tip b { font-weight: 600; }
.tip .k { color: var(--muted); }
.table-box { background: var(--surface); border: 1px solid var(--ring); border-radius: 10px; overflow-x: auto; }
table { border-collapse: collapse; width: 100%; font-size: 14px; font-variant-numeric: tabular-nums; }
th, td { padding: 7px 12px; text-align: right; white-space: nowrap; border-bottom: 1px solid var(--grid); }
th { color: var(--ink-2); font-weight: 600; font-size: 12px; text-transform: uppercase; letter-spacing: 0.05em; }
th:first-child, td:first-child { text-align: left; }
tr:last-child td { border-bottom: 0; }
td.warn { color: var(--ink-2); }
.notes { display: grid; gap: 8px; font-size: 14px; }
.notes p { font-size: 14px; }
code { font-family: var(--mono); font-size: 0.9em; }
</style>

<div class="wrap">
  <header class="lede">
    <h1>Kaunas delivery fleet: wait, size and flying hours</h1>
    <p>Each dot is one fleet: a number of couriers at Mega and at Akropolis, flown through the same
    simulated day of orders. Filled dots are Pareto-optimal: no other fleet is at least as good on mean wait,
    fleet size and flying hours, and better on one.</p>
    <div class="facts" id="facts"></div>
  </header>

  <div class="bar">
    <div class="seg" role="group" aria-label="Wait measure">
      <button type="button" id="m-mean" aria-pressed="true">Mean wait</button>
      <button type="button" id="m-p90" aria-pressed="false">90th percentile</button>
    </div>
    <div class="legend" aria-label="Key">
      <span><svg width="12" height="12"><circle cx="6" cy="6" r="5" fill="var(--front)"/></svg>Pareto-optimal</span>
      <span><svg width="12" height="12"><circle cx="6" cy="6" r="5" fill="var(--dom)"/></svg>Dominated</span>
      <span><svg width="26" height="12"><line x1="1" y1="6" x2="25" y2="6" stroke="var(--front-line)" stroke-width="2"/></svg>Best wait for the size or hours</span>
    </div>
  </div>

  <div class="charts">
    <figure>
      <figcaption>Wait against fleet size</figcaption>
      <div class="sub">Couriers at both hubs, spares not counted. Wait on a log scale.</div>
      <svg class="chart" id="c-fleet" role="img" aria-label="Order to delivery time against fleet size"></svg>
    </figure>
    <figure>
      <figcaption>Wait against fleet flying hours</figcaption>
      <div class="sub">Drone-hours in the air over the run. Wait on a log scale.</div>
      <svg class="chart" id="c-hours" role="img" aria-label="Order to delivery time against fleet flying hours"></svg>
    </figure>
  </div>

  <section style="display:grid; gap:10px">
    <h2>Pareto-optimal fleets</h2>
    <div class="table-box"><table id="front-table"></table></div>
  </section>

  <section class="notes">
    <h2>How this was measured</h2>
    <div id="notes"></div>
  </section>
</div>
<div class="tip" id="tip" hidden></div>

<script>
const DATA = __DATA__;
const rows = DATA.rows;
let measure = 'mean_wait_min';

const fmt = (v, d = 1) => v.toLocaleString('en-GB', { minimumFractionDigits: d, maximumFractionDigits: d });
const front = rows.filter(r => r.pareto);
const full = rows.reduce((a, b) => (b.fleet > a.fleet ? b : a));

document.getElementById('facts').innerHTML = [
  ['Configurations', rows.length],
  ['Pareto-optimal', front.length],
  ['Orders judged', DATA.orders.toLocaleString('en-GB')],
  ['Order window', DATA.window_h + ' h of a ' + DATA.hours + ' h run'],
].map(([k, v]) => `<span>${k} <b>${v}</b></span>`).join('');

document.getElementById('notes').innerHTML = `
  <p>Every fleet is a copy of <code>scenarios/kaunas-deliveries</code> with fewer couriers (the spares stay), run headless with
  <code>simctl fly</code> for ${DATA.hours} simulated hours. Orders come from the pack's seed, so every fleet gets the same
  orders at the same moments. Each is judged on the ${DATA.orders.toLocaleString('en-GB')} orders placed in the first
  ${DATA.window_h} hours that were not turned away for a geofence.</p>
  <p>Wait is order to delivery: queueing for a drone, the flight out and the winch. An order still undelivered when the run
  ends counts as waiting until then, so for fleets that fall behind the figure is a lower bound; the table shows how many
  orders that applies to.</p>
  <p>Flying hours scale with the work done: a fleet that delivers fewer orders flies fewer hours. Per delivery the figure is
  close to constant (${fmt(Math.min(...rows.map(r => r.flying_min_per_delivery)))}–${fmt(Math.max(...rows.map(r => r.flying_min_per_delivery)))} min),
  so among fleets that keep up, size and wait are the real trade.</p>
  <p>Regenerate with <code>uv run tools/fleet_pareto.py</code> then <code>uv run tools/fleet_pareto_chart.py</code>.</p>`;

function table() {
  const head = '<tr><th>Mega + Akropolis</th><th>Fleet</th><th>Mean wait</th><th>90th pct</th><th>Flying h</th><th>Min / delivery</th><th>Undelivered</th></tr>';
  const body = front.slice().sort((a, b) => a.fleet - b.fleet || a.mean_wait_min - b.mean_wait_min).map(r => `<tr>
    <td>${r.mega} + ${r.akro}</td><td>${r.fleet}</td><td>${fmt(r.mean_wait_min)} min</td><td>${fmt(r.p90_wait_min)} min</td>
    <td>${fmt(r.flying_h)}</td><td>${fmt(r.flying_min_per_delivery)}</td>
    <td class="${r.undelivered ? 'warn' : ''}">${r.undelivered ? r.undelivered + ' of ' + r.orders : '0'}</td></tr>`).join('');
  document.getElementById('front-table').innerHTML = head + body;
}

const tip = document.getElementById('tip');
function showTip(e, r) {
  tip.innerHTML = `<b>${r.mega} at Mega + ${r.akro} at Akropolis</b> (${r.fleet})${r.pareto ? ' · Pareto-optimal' : ''}<br>
    <span class="k">Mean wait</span> ${fmt(r.mean_wait_min)} min · <span class="k">90th</span> ${fmt(r.p90_wait_min)} min<br>
    <span class="k">Flying</span> ${fmt(r.flying_h)} h · ${fmt(r.flying_min_per_delivery)} min per delivery<br>
    <span class="k">Undelivered at the end</span> ${r.undelivered} of ${r.orders}`;
  tip.hidden = false;
  const pad = 14, w = tip.offsetWidth, h = tip.offsetHeight;
  let x = e.clientX + pad, y = e.clientY + pad;
  if (x + w > innerWidth - 8) x = e.clientX - w - pad;
  if (y + h > innerHeight - 8) y = e.clientY - h - pad;
  tip.style.left = x + 'px'; tip.style.top = y + 'px';
}

function niceTicks(lo, hi, n) {
  const step0 = (hi - lo) / n, mag = 10 ** Math.floor(Math.log10(step0));
  const step = [1, 2, 2.5, 5, 10].map(m => m * mag).find(s => s >= step0);
  const out = [];
  for (let v = Math.ceil(lo / step) * step; v <= hi + 1e-9; v += step) out.push(+v.toFixed(6));
  return out;
}
function logTicks(lo, hi) {
  const out = [];
  for (let e = Math.floor(Math.log10(lo)); e <= Math.ceil(Math.log10(hi)); e++)
    for (const m of [1, 2, 5]) { const v = m * 10 ** e; if (v >= lo && v <= hi) out.push(v); }
  return out;
}

function draw(svgId, xKey, xLabel, xFmt) {
  const svg = document.getElementById(svgId);
  const W = Math.max(320, svg.clientWidth || 560), H = Math.round(Math.min(420, Math.max(280, W * 0.62)));
  const m = { l: 52, r: 14, t: 10, b: 44 };
  svg.setAttribute('viewBox', `0 0 ${W} ${H}`);
  const ys = rows.map(r => r[measure]), xs = rows.map(r => r[xKey]);
  const yLo = Math.max(0.5, Math.min(...ys) * 0.8), yHi = Math.max(...ys) * 1.25;
  const xT = niceTicks(0, Math.max(...xs) * 1.04, 6);
  const xHi = xT[xT.length - 1];
  const X = v => m.l + (v / xHi) * (W - m.l - m.r);
  const Y = v => m.t + (1 - (Math.log(v) - Math.log(yLo)) / (Math.log(yHi) - Math.log(yLo))) * (H - m.t - m.b);
  let s = '';
  for (const t of logTicks(yLo, yHi)) {
    s += `<line class="gridline" x1="${m.l}" x2="${W - m.r}" y1="${Y(t)}" y2="${Y(t)}"/>`;
    s += `<text x="${m.l - 8}" y="${Y(t) + 4}" text-anchor="end">${t >= 1 ? t : t.toFixed(1)}</text>`;
  }
  for (const t of xT) s += `<text x="${X(t)}" y="${H - m.b + 18}" text-anchor="middle">${xFmt(t)}</text>`;
  s += `<line class="axisline" x1="${m.l}" x2="${W - m.r}" y1="${H - m.b}" y2="${H - m.b}"/>`;
  s += `<text class="lab" x="${(m.l + W - m.r) / 2}" y="${H - 6}" text-anchor="middle">${xLabel}</text>`;
  s += `<text class="lab" transform="translate(13 ${(m.t + H - m.b) / 2}) rotate(-90)" text-anchor="middle">minutes, order to delivery</text>`;
  // Lower envelope: the best wait any fleet reaches at this x or less.
  const sorted = rows.slice().sort((a, b) => a[xKey] - b[xKey] || a[measure] - b[measure]);
  const env = []; let best = Infinity;
  for (const r of sorted) if (r[measure] < best) { best = r[measure]; env.push(r); }
  s += `<path class="envelope" d="${env.map((r, i) => (i ? 'L' : 'M') + X(r[xKey]).toFixed(1) + ' ' + Y(r[measure]).toFixed(1)).join(' ')}"/>`;
  const order = rows.map((r, i) => i).sort((a, b) => rows[a].pareto - rows[b].pareto);
  for (const i of order) {
    const r = rows[i];
    s += `<circle class="dot ${r.pareto ? 'front' : 'dom'}" data-i="${i}" cx="${X(r[xKey]).toFixed(1)}" cy="${Y(r[measure]).toFixed(1)}" r="${r.pareto ? 5.5 : 4.5}"/>`;
  }
  svg.innerHTML = s;
}

function render() {
  draw('c-fleet', 'fleet', 'couriers in the fleet', v => v);
  draw('c-hours', 'flying_h', 'fleet flying hours over the run', v => v);
}

for (const svg of document.querySelectorAll('svg.chart')) {
  svg.addEventListener('pointermove', e => {
    const dots = [...svg.querySelectorAll('.dot')];
    const pt = svg.createSVGPoint(); pt.x = e.clientX; pt.y = e.clientY;
    const p = pt.matrixTransform(svg.getScreenCTM().inverse());
    let bestD = 18 ** 2, hit = null;
    for (const d of dots) {
      const dx = d.cx.baseVal.value - p.x, dy = d.cy.baseVal.value - p.y, dd = dx * dx + dy * dy;
      if (dd < bestD) { bestD = dd; hit = d; }
    }
    document.querySelectorAll('.dot.hot').forEach(d => d.classList.remove('hot'));
    if (!hit) { tip.hidden = true; return; }
    const i = hit.dataset.i;
    document.querySelectorAll(`.dot[data-i="${i}"]`).forEach(d => d.classList.add('hot'));
    showTip(e, rows[i]);
  });
  svg.addEventListener('pointerleave', () => {
    tip.hidden = true;
    document.querySelectorAll('.dot.hot').forEach(d => d.classList.remove('hot'));
  });
}

for (const [id, key] of [['m-mean', 'mean_wait_min'], ['m-p90', 'p90_wait_min']]) {
  document.getElementById(id).addEventListener('click', () => {
    measure = key;
    document.getElementById('m-mean').setAttribute('aria-pressed', key === 'mean_wait_min');
    document.getElementById('m-p90').setAttribute('aria-pressed', key === 'p90_wait_min');
    render();
  });
}
addEventListener('resize', render);
table();
render();
</script>
"""


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--out", type=Path, default=ROOT / "var" / "fleet-pareto")
    args = ap.parse_args()
    data = json.loads((args.out / "results.json").read_text())
    page = args.out / "pareto.html"
    page.write_text(PAGE.replace("__DATA__", json.dumps(data)))
    print(page)


if __name__ == "__main__":
    main()
