// The Weather page: a wind layer over the shared map, and a query panel.
//
// The map (map.js) does the projection, basemap, drones and panning; this
// file hands it a tool that shades each A5 weather cell by wind speed and
// draws an arrow the way the wind blows, and that turns a click into a query
// for the wind there. Everything comes from `/api/v1/weather*`; the status
// panel above is server-rendered and polls itself.

(function () {
  'use strict';

  const map = window.krabshellMap;
  if (!map) return;

  const altSelect = document.getElementById('wind-alt');
  const nowBox = document.getElementById('wind-now');
  const frameInput = document.getElementById('wind-frame');
  const timeOut = document.getElementById('wind-time');
  const layerBox = document.getElementById('wind-layer');
  const legend = document.getElementById('wind-legend');
  const form = document.getElementById('wind-form');
  const qLat = document.getElementById('q-lat');
  const qLon = document.getElementById('q-lon');
  const qCell = document.getElementById('q-cell');
  const qAlt = document.getElementById('q-alt');
  const qStatus = document.getElementById('q-status');
  const qResult = document.getElementById('q-result');

  // Cold to hot, as the occupancy map.
  const RAMP = [
    [43, 108, 176],
    [79, 178, 134],
    [233, 196, 106],
    [224, 122, 63],
    [164, 35, 43],
  ];

  let status = null;   // /api/v1/weather
  let field = null;    // /api/v1/weather/cells
  let detail = null;   // /api/v1/weather/cells/{cell}
  let scale = 15;      // m/s at the hot end of the ramp

  function colour(speed) {
    const t = Math.max(0, Math.min(1, speed / scale)) * (RAMP.length - 1);
    const i = Math.min(RAMP.length - 2, Math.floor(t));
    const f = t - i;
    const c = RAMP[i].map((v, k) => Math.round(v + (RAMP[i + 1][k] - v) * f));
    return `rgb(${c[0]}, ${c[1]}, ${c[2]})`;
  }

  async function get(url) {
    const response = await fetch(url, { headers: { accept: 'application/json' } });
    const body = await response.json().catch(() => null);
    if (!response.ok) {
      const err = new Error(body && body.detail ? body.detail : `${response.status}`);
      err.status = response.status;
      throw err;
    }
    return body;
  }

  // ------------------------------------------------------------ time

  function frameTimes() {
    if (!status || !status.window) return [];
    const first = Date.parse(status.window.first_utc) / 1000;
    return Array.from({ length: status.window.frames }, (_, i) => first + i * status.window.step_s);
  }

  function iso(unix) {
    return new Date(unix * 1000).toISOString().replace('.000', '');
  }

  function short(utc) {
    return utc ? utc.replace('T', ' ').replace(/:00Z$|Z$/, '') + ' UTC' : '';
  }

  // The time the map and query show: the simulation's, or the hour picked.
  function shownUtc() {
    if (nowBox.checked || !status || !status.window) return null;
    const times = frameTimes();
    return iso(times[Math.min(times.length - 1, Number(frameInput.value))]);
  }

  function syncSlider() {
    const times = frameTimes();
    frameInput.max = String(Math.max(0, times.length - 1));
    frameInput.disabled = nowBox.checked || times.length < 2;
    if (nowBox.checked && status && status.sim_utc) {
      const sim = Date.parse(status.sim_utc) / 1000;
      let i = 0;
      while (i + 1 < times.length && times[i + 1] <= sim) i++;
      frameInput.value = String(i);
      timeOut.textContent = short(status.sim_utc);
    } else {
      timeOut.textContent = times.length ? short(iso(times[Number(frameInput.value)])) : '';
    }
  }

  // ------------------------------------------------------------ data

  async function refreshStatus() {
    try {
      status = await get('/api/v1/weather');
    } catch {
      return;
    }
    syncSlider();
  }

  async function refreshCells() {
    if (!status || !status.available) {
      field = null;
      renderLegend();
      map.redraw();
      return;
    }
    const params = new URLSearchParams({ alt_m: altSelect.value });
    const utc = shownUtc();
    if (utc) params.set('utc', utc);
    try {
      field = await get(`/api/v1/weather/cells?${params}`);
      scale = Math.max(10, Math.ceil(field.max_speed_ms / 5) * 5);
    } catch {
      field = null;
    }
    renderLegend();
    map.redraw();
  }

  function renderLegend() {
    if (!field || !layerBox.checked) {
      legend.hidden = true;
      return;
    }
    legend.hidden = false;
    const stops = RAMP.map((c, i) => `rgb(${c}) ${(i / (RAMP.length - 1)) * 100}%`).join(', ');
    legend.innerHTML =
      `<span>Wind at ${field.alt_m} m</span><span>0</span>` +
      `<span class="ramp" style="background: linear-gradient(to right, ${stops})"></span>` +
      `<span>${scale} m/s</span>`;
  }

  // ------------------------------------------------------------ drawing

  function centre(ring) {
    let lon = 0;
    let lat = 0;
    for (const [x, y] of ring) {
      lon += x;
      lat += y;
    }
    return [lon / ring.length, lat / ring.length];
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

  // An arrow the way the wind blows, its length by speed.
  function arrow(ctx, project, cell, ink) {
    const [lon, lat] = centre(cell.boundary);
    const [x, y] = project(lon, lat);
    // Screen direction: project a step east and north (Mercator stretches).
    const [ex] = project(lon + 0.001, lat);
    const [, ny] = project(lon, lat + 0.001);
    const sx = (ex - x) / 0.001;
    const sy = (ny - y) / 0.001;
    let dx = cell.east_ms * sx / Math.cos((lat * Math.PI) / 180);
    let dy = cell.north_ms * sy;
    const n = Math.hypot(dx, dy);
    if (n < 1e-9 || cell.speed_ms < 0.05) return;
    // Fit the cell: as wide as the cell at the ramp's hot end.
    const ring = cell.boundary.map(([a, b]) => project(a, b));
    const across = Math.max(...ring.map(([a]) => a)) - Math.min(...ring.map(([a]) => a));
    if (across < 10) return;
    const len = Math.min(across * 0.7, 8 + (across * 0.6 * cell.speed_ms) / scale);
    dx = (dx / n) * len;
    dy = (dy / n) * len;
    const tail = [x - dx / 2, y - dy / 2];
    const head = [x + dx / 2, y + dy / 2];
    ctx.beginPath();
    ctx.moveTo(...tail);
    ctx.lineTo(...head);
    const a = Math.atan2(dy, dx);
    const h = Math.min(8, len * 0.35);
    ctx.lineTo(head[0] - h * Math.cos(a - 0.45), head[1] - h * Math.sin(a - 0.45));
    ctx.moveTo(...head);
    ctx.lineTo(head[0] - h * Math.cos(a + 0.45), head[1] - h * Math.sin(a + 0.45));
    ctx.strokeStyle = ink;
    ctx.lineWidth = 1.8;
    ctx.stroke();
    if (across > 70) {
      ctx.fillStyle = ink;
      ctx.font = '11px system-ui, sans-serif';
      ctx.textAlign = 'center';
      ctx.fillText(`${cell.speed_ms.toFixed(1)}`, x, y + len / 2 + 13);
    }
  }

  const tool = {
    draw(ctx, project, style) {
      const ink = style.getPropertyValue('--ink').trim() || '#000';
      const accent = style.getPropertyValue('--accent').trim() || '#2b6cb0';
      if (field && layerBox.checked) {
        for (const cell of field.cells) {
          if (!cell.boundary || cell.boundary.length < 3) continue;
          trace(ctx, project, cell.boundary);
          ctx.fillStyle = colour(cell.speed_ms);
          ctx.globalAlpha = 0.3;
          ctx.fill();
          ctx.globalAlpha = 0.5;
          ctx.strokeStyle = colour(cell.speed_ms);
          ctx.lineWidth = 0.8;
          ctx.stroke();
          ctx.globalAlpha = 1;
        }
        for (const cell of field.cells) arrow(ctx, project, cell, ink);
      }
      if (detail && detail.boundary && detail.boundary.length > 2) {
        trace(ctx, project, detail.boundary);
        ctx.setLineDash([6, 4]);
        ctx.strokeStyle = accent;
        ctx.lineWidth = 2.5;
        ctx.stroke();
        ctx.setLineDash([]);
      }
    },
    click(lon, lat) {
      qLat.value = lat.toFixed(5);
      qLon.value = lon.toFixed(5);
      qCell.value = '';
      query();
      return true;
    },
  };

  // ------------------------------------------------------------ query

  async function query() {
    const alt = Number(qAlt.value || '50');
    qStatus.textContent = 'Asking…';
    try {
      let cell = qCell.value.trim();
      if (!cell) {
        const lat = Number(qLat.value);
        const lon = Number(qLon.value);
        if (!Number.isFinite(lat) || !Number.isFinite(lon)) {
          throw new Error('give a latitude and longitude, or a cell');
        }
        cell = (await get(`/api/v1/cells/lookup?${new URLSearchParams({ lon, lat })}`)).cell;
      }
      const params = new URLSearchParams({ alt_m: String(alt) });
      const utc = shownUtc();
      if (utc) params.set('utc', utc);
      detail = await get(`/api/v1/weather/cells/${encodeURIComponent(cell)}?${params}`);
      qStatus.textContent = `Cell ${detail.cell}`;
      renderDetail();
    } catch (err) {
      detail = null;
      qResult.hidden = true;
      qStatus.textContent = err.status === 404 && status && !status.available
        ? 'This run has no weather.'
        : `No answer: ${err.message}`;
    }
    map.redraw();
  }

  function compass(deg) {
    const names = ['N', 'NNE', 'NE', 'ENE', 'E', 'ESE', 'SE', 'SSE', 'S', 'SSW', 'SW', 'WSW', 'W', 'WNW', 'NW', 'NNW'];
    return names[Math.round(deg / 22.5) % 16];
  }

  function renderDetail() {
    const d = detail;
    qResult.hidden = false;
    document.getElementById('q-wcell').textContent =
      d.weather_cell === d.cell ? d.weather_cell : `${d.weather_cell} (resolution ${d.resolution})`;
    document.getElementById('q-speed').innerHTML =
      `${d.wind.speed_ms.toFixed(1)}<small>m/s at ${d.alt_m} m</small>`;
    document.getElementById('q-from').textContent =
      `${compass(d.wind.from_deg)} (${d.wind.from_deg.toFixed(0)}°)`;
    document.getElementById('q-gust').textContent = `${d.wind.gust_ms.toFixed(1)} m/s`;
    document.getElementById('q-vec').textContent =
      `${d.wind.east_ms.toFixed(2)}, ${d.wind.north_ms.toFixed(2)} m/s`;
    document.getElementById('q-utc').textContent = short(d.utc);
    const body = document.querySelector('#q-levels tbody');
    body.replaceChildren(...d.levels.map((l) => {
      const tr = document.createElement('tr');
      for (const text of [`${l.alt_m} m`, `${l.speed_ms.toFixed(1)} m/s`, `${compass(l.from_deg)} ${l.from_deg.toFixed(0)}°`]) {
        const td = document.createElement('td');
        td.textContent = text;
        tr.append(td);
      }
      return tr;
    }));
    chart(d);
  }

  // Wind and gusts through the window, with the simulation's time marked.
  function chart(d) {
    const svg = document.getElementById('q-chart');
    const W = 520;
    const H = 170;
    const pad = { l: 34, r: 20, t: 8, b: 22 };
    svg.setAttribute('viewBox', `0 0 ${W} ${H}`);
    const n = d.series.length;
    const top = Math.max(5, ...d.series.map((f) => Math.max(f.speed_ms, f.gust_ms)));
    const ymax = Math.ceil(top / 5) * 5;
    const x = (i) => pad.l + (n < 2 ? 0 : (i / (n - 1)) * (W - pad.l - pad.r));
    const y = (v) => H - pad.b - (v / ymax) * (H - pad.t - pad.b);
    const line = (key) => d.series.map((f, i) => `${x(i).toFixed(1)},${y(f[key]).toFixed(1)}`).join(' ');
    const parts = [];
    for (let v = 0; v <= ymax; v += ymax / 5) {
      parts.push(`<line class="grid" x1="${pad.l}" x2="${W - pad.r}" y1="${y(v)}" y2="${y(v)}"/>`);
      parts.push(`<text class="axis" x="${pad.l - 4}" y="${y(v) + 4}" text-anchor="end">${v}</text>`);
    }
    const every = Math.max(1, Math.ceil(n / 6));
    d.series.forEach((f, i) => {
      if (i % every === 0) {
        parts.push(`<text class="axis" x="${x(i)}" y="${H - 6}" text-anchor="middle">${f.utc.slice(11, 16)}</text>`);
      }
    });
    parts.push(`<polyline class="l-gust" points="${line('gust_ms')}"/>`);
    parts.push(`<polyline class="l-wind" points="${line('speed_ms')}"/>`);
    if (status && status.sim_utc && n > 1) {
      const first = Date.parse(d.series[0].utc);
      const last = Date.parse(d.series[n - 1].utc);
      const sim = Date.parse(status.sim_utc);
      if (sim >= first && sim <= last) {
        const sx = pad.l + ((sim - first) / (last - first)) * (W - pad.l - pad.r);
        parts.push(`<line class="l-now" x1="${sx}" x2="${sx}" y1="${pad.t}" y2="${H - pad.b}"/>`);
      }
    }
    svg.innerHTML = parts.join('');
  }

  // ------------------------------------------------------------ wiring

  form.addEventListener('submit', (event) => {
    event.preventDefault();
    query();
  });
  qCell.addEventListener('input', () => {
    if (qCell.value.trim()) {
      qLat.value = '';
      qLon.value = '';
    }
  });
  altSelect.addEventListener('change', () => {
    qAlt.value = altSelect.value;
    refreshCells();
    if (detail) query();
  });
  nowBox.addEventListener('change', () => {
    syncSlider();
    refreshCells();
    if (detail) query();
  });
  frameInput.addEventListener('input', () => {
    syncSlider();
    refreshCells();
  });
  frameInput.addEventListener('change', () => {
    if (detail) query();
  });
  layerBox.addEventListener('change', () => {
    renderLegend();
    map.redraw();
  });

  map.setTool(tool);

  async function tick() {
    await refreshStatus();
    await refreshCells();
  }
  tick();
  // Follow the simulation's clock; an hour picked by hand stays put.
  setInterval(() => {
    if (document.hidden) return;
    refreshStatus().then(() => {
      if (nowBox.checked) refreshCells();
    });
  }, 3000);
})();
