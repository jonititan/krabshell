// The Conflicts page: a conflict heatmap over the shared map.
//
// The map (map.js) does the projection, basemap, drones and panning; this
// file hands it a tool that shades every A5 cell any aircraft could be in
// over the next minute, at its highest: by the aircraft expected there (the
// summed probability, up to 1 for one aircraft alone) or by the chance of two
// or more together. It outlines the cells in conflict, numbers the top ten,
// and links the map and the table of top cells: a click on either picks the
// cell out in both. The tables themselves are server-rendered and poll by
// htmx.

(function () {
  'use strict';

  const map = window.krabshellMap;
  if (!map) return;

  const metricSelect = document.getElementById('conflict-metric');
  const layerBox = document.getElementById('conflict-layer');
  const legend = document.getElementById('conflict-legend');

  // Cold to hot, as the other maps.
  const RAMP = [
    [43, 108, 176],
    [79, 178, 134],
    [233, 196, 106],
    [224, 122, 63],
    [164, 35, 43],
  ];

  let report = null; // /api/v1/conflicts
  let picked = null; // a cell id
  let top = 1;       // the hot end of the ramp

  function colour(t) {
    const x = Math.max(0, Math.min(1, t)) * (RAMP.length - 1);
    const i = Math.min(RAMP.length - 2, Math.floor(x));
    const f = x - i;
    const c = RAMP[i].map((v, k) => Math.round(v + (RAMP[i + 1][k] - v) * f));
    return `rgb(${c[0]}, ${c[1]}, ${c[2]})`;
  }

  function value(cell) {
    return metricSelect.value === 'summed' ? cell.summed : cell.collision;
  }

  // The hot end of the ramp: at least two aircraft, so a lone aircraft's
  // certainty is mid-scale and anything shared stands out.
  function scale() {
    if (metricSelect.value !== 'summed' || !report) return 1;
    const most = report.heat.reduce((m, c) => Math.max(m, c.summed), 2 * report.threshold);
    return Math.ceil(most * 2) / 2;
  }

  async function refresh() {
    try {
      const response = await fetch('/api/v1/conflicts', { headers: { accept: 'application/json' } });
      if (!response.ok) return;
      report = await response.json();
    } catch {
      return;
    }
    top = scale();
    renderLegend();
    map.redraw();
  }

  function renderLegend() {
    if (!report || !layerBox.checked) {
      legend.hidden = true;
      return;
    }
    legend.hidden = false;
    const stops = RAMP.map((c, i) => `rgb(${c}) ${(i / (RAMP.length - 1)) * 100}%`).join(', ');
    const label = metricSelect.value === 'summed' ? 'Aircraft expected' : 'P(two or more)';
    const high = metricSelect.value === 'summed' ? `${top}` : '100%';
    legend.innerHTML =
      `<span>${label}</span><span>0</span>` +
      `<span class="ramp" style="background: linear-gradient(to right, ${stops})"></span>` +
      `<span>${high}</span>`;
  }

  function trace(ctx, project, ring) {
    ctx.beginPath();
    ring.forEach(([lon, lat], i) => {
      const [x, y] = project(lon, lat);
      if (i === 0) ctx.moveTo(x, y);
      else ctx.lineTo(x, y);
    });
    ctx.closePath();
  }

  function centre(ring) {
    let lon = 0;
    let lat = 0;
    for (const [x, y] of ring) {
      lon += x;
      lat += y;
    }
    return [lon / ring.length, lat / ring.length];
  }

  // Ray casting, in degrees: cells are small enough.
  function inside(ring, lon, lat) {
    let hit = false;
    for (let i = 0, j = ring.length - 1; i < ring.length; j = i++) {
      const [xi, yi] = ring[i];
      const [xj, yj] = ring[j];
      if ((yi > lat) !== (yj > lat) && lon < ((xj - xi) * (lat - yi)) / (yj - yi) + xi) hit = !hit;
    }
    return hit;
  }

  function markRow() {
    for (const row of document.querySelectorAll('table.conflict-cells tr.picked')) {
      row.classList.remove('picked');
    }
    if (!picked) return;
    const row = document.getElementById(`cell-${picked}`);
    if (row) row.classList.add('picked');
  }

  const tool = {
    draw(ctx, project, style) {
      if (!report || !layerBox.checked) return;
      const ink = style.getPropertyValue('--ink').trim() || '#000';
      const bad = style.getPropertyValue('--bad').trim() || '#a4232b';
      const accent = style.getPropertyValue('--accent').trim() || '#2b6cb0';
      // Every cell anyone could be in, faintest first so the worst are on
      // top; then the conflicts outlined.
      const heat = report.heat.filter((c) => c.boundary.length > 2);
      for (const c of [...heat].reverse()) {
        const t = Math.min(1, value(c) / top);
        trace(ctx, project, c.boundary);
        ctx.fillStyle = colour(t);
        ctx.globalAlpha = 0.2 + 0.5 * t;
        ctx.fill();
        ctx.globalAlpha = 1;
      }
      const cells = report.cells.filter((c) => c.boundary.length > 2);
      for (const c of cells.filter((x) => x.conflict)) {
        trace(ctx, project, c.boundary);
        ctx.strokeStyle = bad;
        ctx.lineWidth = 2;
        ctx.stroke();
      }
      // The top ten, numbered as in the table.
      ctx.font = '600 11px system-ui, sans-serif';
      ctx.textAlign = 'center';
      ctx.textBaseline = 'middle';
      cells.slice(0, 10).forEach((c, i) => {
        const [x, y] = project(...centre(c.boundary));
        ctx.fillStyle = 'rgba(255, 255, 255, 0.85)';
        ctx.beginPath();
        ctx.arc(x, y, 8, 0, Math.PI * 2);
        ctx.fill();
        ctx.fillStyle = ink;
        ctx.fillText(String(i + 1), x, y + 0.5);
      });
      const chosen = picked && heat.find((c) => c.cell === picked);
      if (chosen) {
        trace(ctx, project, chosen.boundary);
        ctx.setLineDash([6, 4]);
        ctx.strokeStyle = accent;
        ctx.lineWidth = 2.5;
        ctx.stroke();
        ctx.setLineDash([]);
      }
    },
    click(lon, lat) {
      if (!report) return false;
      const hit = report.heat.find((c) => c.boundary.length > 2 && inside(c.boundary, lon, lat));
      picked = hit ? hit.cell : null;
      markRow();
      // Every click is this page's: there is no cell inspector here.
      return true;
    },
  };

  // A row of the top cells table picks its cell out and shows it.
  document.addEventListener('click', (event) => {
    const row = event.target.closest('table.conflict-cells tr[data-cell]');
    if (!row) return;
    picked = row.dataset.cell;
    markRow();
    map.showPlace(Number(row.dataset.lon), Number(row.dataset.lat), 400);
  });
  // The tables are swapped every couple of seconds; keep the pick. A new
  // cell resolution means new cells: fetch them now.
  document.body.addEventListener('htmx:afterSwap', markRow);
  document.body.addEventListener('htmx:afterRequest', (event) => {
    if (event.detail.requestConfig && event.detail.requestConfig.path === '/actions/conflicts/resolution') {
      picked = null;
      refresh();
    }
  });

  metricSelect.addEventListener('change', () => {
    top = scale();
    renderLegend();
    map.redraw();
  });
  layerBox.addEventListener('change', () => {
    renderLegend();
    map.redraw();
  });

  map.setTool(tool);
  refresh();
  setInterval(() => {
    if (!document.hidden) refresh();
  }, 2000);
})();
