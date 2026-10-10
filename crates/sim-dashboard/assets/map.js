// Occupancy map.
//
// The server does the expensive half: `/api/v1/state/map` returns cells already
// aggregated to a display resolution, each with its boundary ring. This file
// projects, draws, optionally lays a raster basemap underneath, and turns a
// click into the display cell to inspect.
//
// Open geofences (`/api/v1/geofences`) are drawn over the cells. Each poll's
// body goes out as a `krabshell:geofences` event, and `window.krabshellMap`
// lets a page hand clicks to a tool of its own (the Geofences page draws
// new fences with one) instead of the cell inspector.
//
// Drones, sightings and stations are drawn as NATO joint military symbols
// (APP-6 / MIL-STD-2525), by the vendored milsymbol library; regions of
// interest as named areas of interest. Without milsymbol the map falls back
// to plain arrows and dots.
//
// Projection is Web Mercator, because that is what every XYZ tile server
// renders; cells and tiles only line up if both use it. It does not wrap the
// antimeridian and clips at ±85°, both fine for an operator view.

(function () {
  'use strict';

  const canvas = document.getElementById('map');
  if (!canvas) return;

  const ctx = canvas.getContext('2d');
  const status = document.getElementById('map-status');
  const legend = document.getElementById('map-legend');
  const resInput = document.getElementById('map-res');
  const resOut = document.getElementById('map-res-out');
  const follow = document.getElementById('map-follow');
  const fitButton = document.getElementById('map-fit');
  const basemapSelect = document.getElementById('map-basemap');
  const attribution = document.getElementById('map-attribution');
  const cellInput = document.getElementById('cell-input');
  const layerInputs = [...document.querySelectorAll('input[data-layer]')];

  // What the map shows, each switched by a `data-layer` checkbox and
  // remembered across pages: drones and intruders; delivery locations
  // (hubs and landmarks, open orders' addresses, recent drop-offs); people
  // and vehicles (where they are, and camera sightings of them); geofences;
  // launch and recovery stations; regions of interest; and the A5 occupancy
  // grid.
  const layers = {
    drones: true, deliveries: true, people: true, fences: true,
    stations: true, rois: true, range: true, grid: true,
  };
  try {
    Object.assign(layers, JSON.parse(localStorage.getItem('krabshell.layers') || '{}'));
  } catch { /* no storage, or nonsense in it: everything on */ }
  // A page may start some layers off for itself (`data-layers-off="grid"` on
  // the canvas) without changing what the other pages remember.
  const pageOff = (canvas.dataset.layersOff || '').split(',').filter(Boolean);
  for (const key of pageOff) layers[key] = false;
  function rememberLayers() {
    try {
      const saved = JSON.parse(localStorage.getItem('krabshell.layers') || '{}');
      const mine = { ...layers };
      for (const key of pageOff) {
        if (key in saved) mine[key] = saved[key];
        else delete mine[key];
      }
      localStorage.setItem('krabshell.layers', JSON.stringify(mine));
    } catch { /* not remembered, still applied */ }
  }

  // Colours the ramp goes through, cold to hot.
  const RAMP = [
    [43, 108, 176],
    [79, 178, 134],
    [233, 196, 106],
    [224, 122, 63],
    [164, 35, 43],
  ];

  const TILE = 256;
  const MAX_LAT = 85.0511287798;

  let cells = [];
  let maxCount = 0;
  let bounds = null;      // { minX, minY, maxX, maxY } of the data, in world units
  let camera = null;      // { cx, cy, scale }: world units (0..1) and pixels per world
  let hover = null;
  let selected = null;    // { cell, lon, lat }: the inspected cell and where it was picked
  let inFlight = false;

  // Geofences open now, the one picked out, and a page's tool, if any: an
  // object with optional click(lon, lat) -> handled, move(lon, lat) and
  // draw(ctx, project, style).
  let fences = [];
  let fenceSelected = null;
  let tool = null;

  // Individual things drawn over the occupancy cells: drones, intruders and
  // points of interest from /state/entities, and recent sightings from /reports.
  let entities = [];
  let sightings = [];     // { lat, lon, kind, at }
  let people = [];        // persons and vehicles, polled while their layer is on
  let orders = [];        // open orders' addresses: [lat_e7, lon_e7, in_flight]
  let drops = [];         // recent deliveries: { lat, lon, at }
  // Launch and recovery stations and regions of interest, as the model
  // publishes them in its overlays. A model that publishes none leaves both
  // empty, and nothing is drawn.
  let stations = [];
  let rois = [];
  // How far the couriers can deliver, per hub: [{ depot, reach_m, rings }],
  // each ring an ellipse as [lon, lat] corners; see drawRange.
  let range = [];
  let rangeKey = '';
  let markSelected = null; // { kind: 'station' | 'roi', id }: picked out by a page
  const DROP_MS = 120_000;
  let lastReport = null;  // seq of the newest report seen; null until primed
  const SIGHTING_MS = 30_000;

  // Smooth, accurate motion between polls, the way netcode does it: each poll
  // brings the authoritative state of every drone at a known tick. Between
  // polls a drone is drawn where that state predicts it to be at the current
  // simulated time (straight on at its reported speed and heading). When the
  // next poll lands, the prediction is thrown away and rebuilt from the new
  // state, so what is drawn is never more than one poll's prediction away
  // from the truth, and is the truth whenever a poll arrives.
  const ENTITY_POLL_MS = 250;
  const FRAME_MS = 33;
  // Never predict further than this many simulated seconds past a sample. At
  // big speed-ups a poll covers minutes of flight; dead reckoning through a
  // turn for that long would be fiction, so the drone waits for the truth.
  const MAX_PREDICT_S = 20;
  const samples = new Map(); // id -> { lon, lat, hdg, speed, tick }
  const clock = { tick: null, at: 0, ticksPerMs: 0, tickSeconds: 0.1, advancing: false };

  // ------------------------------------------------------------ geometry

  function cssSize() {
    const rect = canvas.getBoundingClientRect();
    return { w: rect.width, h: rect.height };
  }

  // Keep the backing store matched to the CSS size and the device pixel ratio,
  // otherwise everything is soft on a high-density display.
  function resize() {
    const { w, h } = cssSize();
    const dpr = window.devicePixelRatio || 1;
    if (canvas.width !== Math.round(w * dpr) || canvas.height !== Math.round(h * dpr)) {
      canvas.width = Math.round(w * dpr);
      canvas.height = Math.round(h * dpr);
    }
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  }

  // Longitude and latitude to Web Mercator world units, 0..1 on both axes with
  // y growing south, the same frame XYZ tiles are cut from.
  function toWorld(lon, lat) {
    const phi = (Math.max(-MAX_LAT, Math.min(MAX_LAT, lat)) * Math.PI) / 180;
    return [
      (lon + 180) / 360,
      (1 - Math.log(Math.tan(phi) + 1 / Math.cos(phi)) / Math.PI) / 2,
    ];
  }

  function fromWorld(x, y) {
    const lat = (Math.atan(Math.sinh(Math.PI * (1 - 2 * y))) * 180) / Math.PI;
    return [x * 360 - 180, lat];
  }

  function worldToScreen(x, y) {
    const { w, h } = cssSize();
    return [w / 2 + (x - camera.cx) * camera.scale, h / 2 + (y - camera.cy) * camera.scale];
  }

  function screenToWorld(x, y) {
    const { w, h } = cssSize();
    return [camera.cx + (x - w / 2) / camera.scale, camera.cy + (y - h / 2) / camera.scale];
  }

  function project(lon, lat) {
    const [x, y] = toWorld(lon, lat);
    return worldToScreen(x, y);
  }

  function unproject(x, y) {
    const [wx, wy] = screenToWorld(x, y);
    return fromWorld(wx, wy);
  }

  function fit() {
    if (!bounds) return;
    const { w, h } = cssSize();
    const spanX = Math.max(bounds.maxX - bounds.minX, 1e-9);
    const spanY = Math.max(bounds.maxY - bounds.minY, 1e-9);
    // 0.88 leaves a margin so edge cells are not clipped by the frame.
    const scale = Math.min(w / spanX, h / spanY) * 0.88;
    camera = {
      cx: (bounds.minX + bounds.maxX) / 2,
      cy: (bounds.minY + bounds.maxY) / 2,
      scale: Math.min(scale, TILE * 2 ** 24),
    };
  }

  function colour(count) {
    if (maxCount <= 0) return 'rgba(120,130,145,0.35)';
    // Counts in an ABM are heavy-tailed: a log scale keeps a single crowded
    // cell from flattening everything else to the same colour.
    const t = Math.log1p(count) / Math.log1p(maxCount);
    const position = Math.min(0.999, Math.max(0, t)) * (RAMP.length - 1);
    const index = Math.floor(position);
    const mix = position - index;
    const a = RAMP[index];
    const b = RAMP[Math.min(index + 1, RAMP.length - 1)];
    const channel = (i) => Math.round(a[i] + (b[i] - a[i]) * mix);
    return `rgb(${channel(0)}, ${channel(1)}, ${channel(2)})`;
  }

  // ------------------------------------------------------------- basemap

  let basemap = null;     // the selected <option>'s settings, or null for none
  const tiles = new Map(); // url -> { img, state: 'loading' | 'ready' | 'failed' }
  let tileFailures = 0;
  let tileSuccesses = 0;

  function readBasemap() {
    const option = basemapSelect && basemapSelect.selectedOptions[0];
    if (!option || !option.value) return null;
    return {
      id: option.value,
      url: option.dataset.url,
      maxZoom: parseInt(option.dataset.maxZoom, 10) || 19,
      attribution: option.dataset.attribution,
      attributionUrl: option.dataset.attributionUrl,
    };
  }

  function tileUrl(z, x, y) {
    return basemap.url
      .replace('{z}', z)
      .replace('{x}', x)
      .replace('{y}', y);
  }

  function tile(z, x, y, load) {
    const url = tileUrl(z, x, y);
    let entry = tiles.get(url);
    if (!entry && load) {
      const img = new Image();
      entry = { img, state: 'loading' };
      img.onload = () => {
        entry.state = 'ready';
        tileSuccesses++;
        scheduleDraw();
      };
      img.onerror = () => {
        entry.state = 'failed';
        tileFailures++;
        updateAttribution();
      };
      img.src = url;
      tiles.set(url, entry);
      // Bound the cache: Map keeps insertion order, so the first key is oldest.
      while (tiles.size > 600) tiles.delete(tiles.keys().next().value);
    }
    return entry;
  }

  function drawTiles(w, h) {
    // Pick the zoom whose tiles are closest to their native size on screen.
    const z = Math.max(0, Math.min(basemap.maxZoom, Math.round(Math.log2(camera.scale / TILE))));
    const n = 2 ** z;
    const size = camera.scale / n;
    const [x0, y0] = screenToWorld(0, 0);
    const [x1, y1] = screenToWorld(w, h);
    const tx0 = Math.floor(x0 * n), tx1 = Math.floor(x1 * n);
    const ty0 = Math.max(0, Math.floor(y0 * n)), ty1 = Math.min(n - 1, Math.floor(y1 * n));
    if ((tx1 - tx0 + 1) * (ty1 - ty0 + 1) > 400) return;

    for (let ty = ty0; ty <= ty1; ty++) {
      for (let tx = tx0; tx <= tx1; tx++) {
        const wrapped = ((tx % n) + n) % n;
        const [sx, sy] = worldToScreen(tx / n, ty / n);
        const entry = tile(z, wrapped, ty, true);
        if (entry && entry.state === 'ready') {
          // +0.5 hides hairline seams between tiles from sub-pixel rounding.
          ctx.drawImage(entry.img, sx, sy, size + 0.5, size + 0.5);
          continue;
        }
        // While a tile loads, stretch the nearest loaded ancestor over its
        // square so zooming does not flash blank.
        for (let up = 1; up <= Math.min(z, 5); up++) {
          const pz = z - up;
          const px = wrapped >> up, py = ty >> up;
          const parent = tile(pz, px, py, false);
          if (!parent || parent.state !== 'ready') continue;
          const k = 2 ** up;
          const src = parent.img.naturalWidth / k;
          ctx.drawImage(
            parent.img,
            (wrapped - px * k) * src, (ty - py * k) * src, src, src,
            sx, sy, size + 0.5, size + 0.5,
          );
          break;
        }
      }
    }
  }

  function updateAttribution() {
    if (!attribution) return;
    if (!basemap) {
      attribution.hidden = true;
      return;
    }
    attribution.hidden = false;
    attribution.textContent = '';
    const link = document.createElement('a');
    link.href = basemap.attributionUrl;
    link.target = '_blank';
    link.rel = 'noopener';
    link.textContent = basemap.attribution;
    attribution.append(link);
    if (tileFailures > 0 && tileSuccesses === 0) {
      attribution.append(' · tiles are not loading; the browser needs internet access');
    }
  }

  function setBasemap() {
    basemap = readBasemap();
    tileFailures = 0;
    tileSuccesses = 0;
    try {
      localStorage.setItem('krabshell.basemap', basemap ? basemap.id : '');
    } catch { /* storage may be blocked; the choice just is not remembered */ }
    updateAttribution();
    draw();
  }

  // ---------------------------------------------------------------- draw

  let drawQueued = false;
  function scheduleDraw() {
    if (drawQueued) return;
    drawQueued = true;
    requestAnimationFrame(() => {
      drawQueued = false;
      draw();
    });
  }

  function trace(ring) {
    ctx.beginPath();
    for (let i = 0; i < ring.length; i++) {
      const [x, y] = project(ring[i][0], ring[i][1]);
      if (i === 0) ctx.moveTo(x, y);
      else ctx.lineTo(x, y);
    }
    ctx.closePath();
  }

  function draw() {
    resize();
    const { w, h } = cssSize();
    ctx.clearRect(0, 0, w, h);
    if (!camera) return;

    if (basemap) drawTiles(w, h);

    const style = getComputedStyle(document.body);
    const line = style.getPropertyValue('--line').trim() || '#ccc';
    const ink = style.getPropertyValue('--ink').trim() || '#000';
    const accent = style.getPropertyValue('--accent').trim() || '#2b6cb0';

    // Let the basemap show through when there is one.
    const alpha = basemap ? 0.62 : 0.82;
    let selectedRing = null;
    for (const cell of layers.grid ? cells : []) {
      const ring = cell.boundary;
      if (!ring || ring.length < 3) continue;
      trace(ring);
      ctx.fillStyle = colour(cell.count);
      ctx.globalAlpha = alpha;
      ctx.fill();
      ctx.globalAlpha = 1;
      ctx.strokeStyle = basemap ? 'rgba(255,255,255,0.7)' : line;
      ctx.lineWidth = 0.5;
      ctx.stroke();

      if (hover && hover.cell === cell.cell) {
        ctx.strokeStyle = ink;
        ctx.lineWidth = 2;
        ctx.stroke();
      }
      if (selected && selected.cell === cell.cell) selectedRing = ring;
    }

    // The inspected cell last, so no neighbour paints over its outline. An
    // empty cell is not in `cells`, so its ring comes from the lookup.
    if (!selectedRing && selected && selected.boundary) selectedRing = selected.boundary;
    if (selectedRing) {
      trace(selectedRing);
      ctx.setLineDash([6, 4]);
      ctx.strokeStyle = accent;
      ctx.lineWidth = 2.5;
      ctx.stroke();
      ctx.setLineDash([]);
    }

    if (layers.range) drawRange(style);
    if (layers.fences) drawFences(style);
    if (layers.rois) drawRois(style);
    if (layers.stations) drawStations(style);

    drawEntities(style, ink);

    if (tool && tool.draw) tool.draw(ctx, project, style);

    if (hover && !tool && layers.grid) {
      const label = `${hover.cell} · ${hover.count} agent${hover.count === 1 ? '' : 's'}`;
      ctx.font = '12px ui-monospace, monospace';
      const width = ctx.measureText(label).width + 12;
      const x = Math.min(hover.x + 12, w - width - 4);
      const y = Math.max(hover.y - 26, 4);
      ctx.fillStyle = style.getPropertyValue('--panel').trim() || '#fff';
      ctx.strokeStyle = line;
      ctx.lineWidth = 1;
      ctx.beginPath();
      ctx.roundRect(x, y, width, 22, 5);
      ctx.fill();
      ctx.stroke();
      ctx.fillStyle = ink;
      ctx.fillText(label, x + 6, y + 15);
    }
  }

  // ----------------------------------------------------------- geofences

  // A circle on the ground as a ring of [lon, lat], for drawing.
  function circleRing(lon, lat, radiusM, sides = 64) {
    const k = 180 / Math.PI / 6_371_000;
    const ring = [];
    for (let i = 0; i < sides; i++) {
      const a = (i / sides) * Math.PI * 2;
      ring.push([
        lon + (Math.sin(a) * radiusM * k) / Math.cos((lat * Math.PI) / 180),
        lat + Math.cos(a) * radiusM * k,
      ]);
    }
    return ring;
  }

  function fenceRing(f) {
    if (f.shape === 'circle') return circleRing(f.lon_e7 * 1e-7, f.lat_e7 * 1e-7, f.radius_m);
    return (f.points || []).map(([lat, lon]) => [lon * 1e-7, lat * 1e-7]);
  }

  // Diagonal hatching in the fence colour, so a fence reads as a fence over
  // any cell shading or basemap, red cells included.
  let hatch = null;
  function hatching(colour) {
    if (hatch && hatch.colour === colour) return hatch.pattern;
    const tile = document.createElement('canvas');
    tile.width = tile.height = 10;
    const t = tile.getContext('2d');
    t.strokeStyle = colour;
    t.lineWidth = 2;
    t.beginPath();
    t.moveTo(-2, 12);
    t.lineTo(12, -2);
    t.moveTo(8, 12);
    t.lineTo(12, 8);
    t.moveTo(-2, 2);
    t.lineTo(2, -2);
    t.stroke();
    hatch = { colour, pattern: ctx.createPattern(tile, 'repeat') };
    return hatch.pattern;
  }

  function drawFences(style) {
    if (fences.length === 0) return;
    const bad = style.getPropertyValue('--bad').trim() || '#a4232b';
    const accent = style.getPropertyValue('--accent').trim() || '#2b6cb0';
    const panel = style.getPropertyValue('--panel').trim() || '#fff';
    const ink = style.getPropertyValue('--ink').trim() || '#000';
    const pxPerKm = camera.scale / 40_075;
    for (const f of fences) {
      const ring = fenceRing(f);
      if (ring.length < 3) continue;
      const picked = fenceSelected === f.id;
      // A fence with a raised floor is lighter and dashed: drones may pass
      // beneath it.
      const raised = floorOf(f) > 0;
      trace(ring);
      ctx.fillStyle = panel;
      ctx.globalAlpha = raised ? 0.2 : 0.35;
      ctx.fill();
      ctx.fillStyle = hatching(bad);
      ctx.globalAlpha = (picked ? 0.9 : 0.6) * (raised ? 0.45 : 1);
      ctx.fill();
      ctx.globalAlpha = 1;
      // A light halo under the outline keeps it clear of what is beneath.
      ctx.strokeStyle = panel;
      ctx.lineWidth = picked ? 6 : 4.5;
      ctx.stroke();
      if (raised) ctx.setLineDash([9, 5]);
      ctx.strokeStyle = picked ? accent : bad;
      ctx.lineWidth = picked ? 3 : 2;
      ctx.stroke();
      ctx.setLineDash([]);
      // Its number and altitude band, once it is big enough on screen to
      // carry them.
      if (pxPerKm > 20 || picked) {
        let x = 0, y = 0;
        for (const [lon, lat] of ring) {
          const [px, py] = project(lon, lat);
          x += px;
          y += py;
        }
        x /= ring.length;
        y /= ring.length;
        const band = fenceBand(f);
        const label = band ? `#${f.id} · ${band}` : `#${f.id}`;
        ctx.font = '600 12px ui-sans-serif, system-ui, sans-serif';
        ctx.lineWidth = 3;
        ctx.strokeStyle = panel;
        ctx.strokeText(label, x - ctx.measureText(label).width / 2, y + 4);
        ctx.fillStyle = ink;
        ctx.fillText(label, x - ctx.measureText(label).width / 2, y + 4);
      }
    }
  }

  // A fence's altitude band, in metres above ground level: an absent floor
  // is the surface, an absent ceiling unlimited.
  function floorOf(f) {
    const v = Number(f.floor_m);
    return f.floor_m != null && Number.isFinite(v) ? v : 0;
  }

  function ceilingOf(f) {
    const v = Number(f.ceiling_m);
    return f.ceiling_m != null && Number.isFinite(v) ? v : null;
  }

  // The band the way an aeronautical chart writes it: "SFC–120 m",
  // "60 m–UNL". Empty for a fence from the surface up without limit, unless
  // `always`.
  function fenceBand(f, always = false) {
    const floor = floorOf(f);
    const ceiling = ceilingOf(f);
    if (!always && floor <= 0 && ceiling === null) return '';
    const metres = (m) => `${Math.round(m).toLocaleString()} m`;
    return `${floor > 0 ? metres(floor) : 'SFC'}–${ceiling === null ? 'UNL' : metres(ceiling)}`;
  }

  function insideRing(ring, x, y) {
    let inside = false;
    for (let i = 0, j = ring.length - 1; i < ring.length; j = i++) {
      const [xi, yi] = project(ring[i][0], ring[i][1]);
      const [xj, yj] = project(ring[j][0], ring[j][1]);
      if ((yi > y) !== (yj > y) && x < ((xj - xi) * (y - yi)) / (yj - yi) + xi) inside = !inside;
    }
    return inside;
  }

  // The topmost fence under a screen point.
  function fenceAt(lon, lat) {
    if (!camera) return null;
    const [x, y] = project(lon, lat);
    for (let i = fences.length - 1; i >= 0; i--) {
      if (insideRing(fenceRing(fences[i]), x, y)) return fences[i];
    }
    return null;
  }

  async function refreshFences() {
    try {
      const response = await fetch('/api/v1/geofences');
      if (!response.ok) return;
      const body = await response.json();
      fences = body.fences || [];
      if (fenceSelected !== null && !fences.some((f) => f.id === fenceSelected)) fenceSelected = null;
      document.dispatchEvent(new CustomEvent('krabshell:geofences', { detail: body }));
      if (camera) draw();
    } catch {
      // The map works without the fences.
    }
  }

  // Track the simulation clock from the ticks the polls report. The rate is
  // smoothed, so one late poll does not lurch every drone.
  function updateClock(tick, mode, tickSeconds, now) {
    clock.advancing = mode === 'running' || mode === 'stepping';
    if (tickSeconds > 0) clock.tickSeconds = tickSeconds;
    if (clock.tick === null || tick < clock.tick) {
      clock.ticksPerMs = 0;
    } else if (tick > clock.tick && now > clock.at) {
      const rate = (tick - clock.tick) / (now - clock.at);
      clock.ticksPerMs = clock.ticksPerMs ? clock.ticksPerMs * 0.6 + rate * 0.4 : rate;
    } else {
      return; // Same tick as before: keep predicting from the older anchor.
    }
    clock.tick = tick;
    clock.at = now;
  }

  function simTickAt(now) {
    if (clock.tick === null) return 0;
    return clock.advancing ? clock.tick + clock.ticksPerMs * (now - clock.at) : clock.tick;
  }

  // Replace every drone's state with the authoritative one just received.
  function recordSamples(tick) {
    samples.clear();
    for (const e of entities) {
      if (e.kind === 'poi') continue;
      samples.set(e.id, { lon: e.lon, lat: e.lat, hdg: e.heading_deg || 0, speed: e.speed_ms || 0, tick });
    }
  }

  // Where a drone is now, predicted from its last authoritative state.
  function predicted(e, now) {
    const s = samples.get(e.id);
    if (!s) return { lon: e.lon, lat: e.lat, hdg: e.heading_deg || 0 };
    const dt = Math.min(Math.max((simTickAt(now) - s.tick) * clock.tickSeconds, 0), MAX_PREDICT_S);
    if (dt === 0 || s.speed === 0) return s;
    const a = (s.hdg * Math.PI) / 180;
    const k = 180 / Math.PI / 6_371_000;
    const north = Math.cos(a) * s.speed * dt;
    const east = Math.sin(a) * s.speed * dt;
    return {
      lon: s.lon + (east * k) / Math.cos((s.lat * Math.PI) / 180),
      lat: s.lat + north * k,
      hdg: s.hdg,
    };
  }

  function anyMoving() {
    if (!clock.advancing) return false;
    for (const s of samples.values()) if (s.speed > 0) return true;
    return false;
  }

  // Redraw while something is moving, at about 30 frames a second. The
  // browser pauses this when the tab is hidden.
  let lastFrame = 0;
  function animate(now) {
    if (camera && now - lastFrame >= FRAME_MS && anyMoving()) {
      lastFrame = now;
      draw();
    }
    requestAnimationFrame(animate);
  }
  requestAnimationFrame(animate);

  // ------------------------------------------------------------- symbols

  // NATO joint military symbology. Drones and intruders are the 2525
  // letter SIDCs of the CoT types TAK gets for them (a-f-A-M-H-Q,
  // a-u-A-M-H-Q), so both show the same thing;
  // the rest are APP-6 D / MIL-STD-2525D 20-digit SIDCs (version 10,
  // identity, symbol set, then the entity), which have the civilian
  // individuals and vehicles the letter codes lack. milsymbol's isValid()
  // accepts each of them.
  const SIDC = {
    drone: 'SFAPMHQ--------',         // friend, air, military rotary-wing UAV
    intruder: 'SUAPMHQ--------',         // unknown, air, military rotary-wing UAV
    person: '10011100001103000000',   // unknown, land civilian unit, individual
    station: 'SFGPIBA---H----',       // friend, ground installation, airport / air base
  };
  // Unknown, land equipment, civilian vehicles, by the model's vehicle type.
  const VEHICLE = {
    car: '10011500001601000000',    // automobile
    pickup: '10011500001602000000', // open-bed truck
    bus: '10011500001603000000',    // multiple passenger vehicle
    van: '10011500001604000000',    // utility vehicle
    lorry: '10011500001606000000',  // tractor trailer truck with box
  };
  // A sighting's symbol: what was seen, and for a vehicle, what sort.
  function sighted(s) {
    if (s.kind === 'vehicle') return VEHICLE[s.vehicleType] || VEHICLE.car;
    return s.kind === 'person' ? SIDC.person : s.kind === 'intruder' ? SIDC.intruder : null;
  }
  // A pale halo round every symbol and line, so the black frames read on a
  // dark theme and over any basemap.
  const HALO = 'rgba(255,255,255,0.85)';

  // Each symbol is built once per SIDC, size and pixel ratio, then stamped
  // as an image: milsymbol takes about a millisecond a symbol, too slow to
  // spend on every drone every frame.
  const symbolCache = new Map();
  function symbol(sidc, size) {
    if (!window.ms) return null;
    const ratio = window.devicePixelRatio || 1;
    const key = `${sidc}|${size}|${ratio}`;
    if (!symbolCache.has(key)) {
      let made = null;
      try {
        // Civilians keep the unknown identity's yellow, not APP-6's
        // optional civilian purple: who they are is not yet known.
        const s = new window.ms.Symbol(sidc, {
          size, outlineWidth: 2, outlineColor: HALO, civilianColor: false,
        });
        if (s.isValid()) {
          const image = s.asCanvas(ratio);
          const anchor = s.getAnchor();
          const box = s.getSize();
          made = { image, ax: anchor.x, ay: anchor.y, w: box.width, h: box.height };
        }
      } catch { /* not a symbol milsymbol knows: the caller falls back */ }
      symbolCache.set(key, made);
    }
    return symbolCache.get(key);
  }

  // Symbol size in CSS pixels: small over a whole city, larger at street
  // scale. Three steps keep the cache to a handful of images.
  function symbolSize(pxPerKm) {
    return pxPerKm > 400 ? 26 : pxPerKm > 60 ? 21 : 17;
  }

  // Stamp a symbol centred on (x, y). With a heading, APP-6's
  // direction-of-movement line goes first, from the centre of the frame
  // out along it, so the frame covers its root. Returns how far right of x
  // the symbol reaches, for a label, or null with no symbol to draw.
  function stamp(g, sidc, size, x, y, hdg = null, alpha = 1) {
    const s = symbol(sidc, size);
    if (!s) return null;
    g.globalAlpha = alpha;
    if (hdg !== null) {
      const a = (hdg * Math.PI) / 180;
      const len = size * 1.7;
      const tip = [x + Math.sin(a) * len, y - Math.cos(a) * len];
      const barb = (turn) => [tip[0] + Math.sin(a + turn) * 6, tip[1] - Math.cos(a + turn) * 6];
      g.lineCap = 'round';
      g.lineJoin = 'round';
      for (const [colour, width] of [[HALO, 4.5], ['#000', 1.75]]) {
        g.strokeStyle = colour;
        g.lineWidth = width;
        g.beginPath();
        g.moveTo(x, y);
        g.lineTo(...tip);
        g.moveTo(...barb(2.7));
        g.lineTo(...tip);
        g.lineTo(...barb(-2.7));
        g.stroke();
      }
      g.lineCap = 'butt';
      g.lineJoin = 'miter';
    }
    g.drawImage(s.image, x - s.ax, y - s.ay, s.w, s.h);
    g.globalAlpha = 1;
    return s.w - s.ax;
  }

  // The map keys: each `canvas[data-sidc]` gets its symbol as the map draws
  // it, with a direction-of-movement line when it has a `data-heading`.
  function drawKeys() {
    const ratio = window.devicePixelRatio || 1;
    for (const key of document.querySelectorAll('canvas[data-sidc]')) {
      const w = key.clientWidth || 28;
      const h = key.clientHeight || 28;
      key.width = Math.round(w * ratio);
      key.height = Math.round(h * ratio);
      const g = key.getContext('2d');
      g.setTransform(ratio, 0, 0, ratio, 0, 0);
      const heading = key.dataset.heading ? Number(key.dataset.heading) : null;
      const [x, y] = heading === null ? [w / 2, h / 2] : [w * 0.4, h * 0.6];
      if (stamp(g, key.dataset.sidc, Number(key.dataset.size) || 14, x, y, heading) === null) {
        key.hidden = true;
      }
    }
  }

  // Text with a halo in the panel colour, so it reads over anything.
  function haloText(text, x, y, panel, ink, align = 'left') {
    const left = align === 'center' ? x - ctx.measureText(text).width / 2 : x;
    ctx.lineWidth = 3;
    ctx.strokeStyle = panel;
    ctx.strokeText(text, left, y);
    ctx.fillStyle = ink;
    ctx.fillText(text, left, y);
  }

  // An arrow pointing along the heading, for a drone when milsymbol is
  // missing; screen y grows downward.
  function arrow(x, y, hdg, fill, panel) {
    const a = (hdg * Math.PI) / 180;
    const tip = [x + Math.sin(a) * 12, y - Math.cos(a) * 12];
    const left = [x + Math.sin(a + 2.5) * 8, y - Math.cos(a + 2.5) * 8];
    const right = [x + Math.sin(a - 2.5) * 8, y - Math.cos(a - 2.5) * 8];
    ctx.beginPath();
    ctx.moveTo(...tip);
    ctx.lineTo(...left);
    ctx.lineTo(x, y);
    ctx.lineTo(...right);
    ctx.closePath();
    ctx.fillStyle = fill;
    ctx.strokeStyle = panel;
    ctx.lineWidth = 1.5;
    ctx.fill();
    ctx.stroke();
  }

  // ------------------------------------------------ stations and regions

  function picked(kind, id) {
    return markSelected !== null && markSelected.kind === kind && markSelected.id === id;
  }

  // Degrees from the model's `_e7` integers, or null when they are missing.
  function position(item) {
    const lat = Number(item.lat_e7) * 1e-7;
    const lon = Number(item.lon_e7) * 1e-7;
    return Number.isFinite(lat) && Number.isFinite(lon) && item.lat_e7 != null ? [lon, lat] : null;
  }

  // ------------------------------------------------------- delivery range

  // The model publishes each hub's reach as ellipses: foci `a` and `b`
  // ([lat_e7, lon_e7]) and the most the distances to them may add up to.
  // They are turned into rings of [lon, lat] once, when the range changes
  // (a station added or removed, a fence changing which can be used).
  function setRange(published) {
    const hubs = (published && Array.isArray(published.hubs)) ? published.hubs : [];
    const key = JSON.stringify(hubs);
    if (key === rangeKey) return;
    rangeKey = key;
    range = hubs.map((h) => ({
      depot: h.depot,
      reach_m: h.reach_m,
      rings: (h.areas || []).map((a) => ellipseRing(a.a, a.b, a.sum_m)),
    }));
  }

  function ellipseRing(a, b, sumM, sides = 72) {
    const k = (Math.PI / 180) * 6_371_000;
    const lat0 = a[0] * 1e-7, lon0 = a[1] * 1e-7;
    const cos0 = Math.cos((lat0 * Math.PI) / 180);
    const bx = (b[1] * 1e-7 - lon0) * k * cos0;
    const by = (b[0] * 1e-7 - lat0) * k;
    const focal = Math.hypot(bx, by) / 2;
    const half = sumM / 2;
    const minor = Math.sqrt(Math.max(half * half - focal * focal, 0));
    const [ux, uy] = focal > 1e-9 ? [bx / (2 * focal), by / (2 * focal)] : [1, 0];
    const ring = [];
    for (let i = 0; i < sides; i++) {
      const t = (i / sides) * Math.PI * 2;
      const u = half * Math.cos(t), v = minor * Math.sin(t);
      const x = bx / 2 + u * ux - v * uy;
      const y = by / 2 + u * uy + v * ux;
      ring.push([lon0 + x / (k * cos0), lat0 + y / k]);
    }
    return ring;
  }

  // Each hub's reach is the union of its ellipses: filled lightly, and
  // outlined along the union's edge only. The outline comes from stroking
  // every ellipse on a scratch canvas and then cutting out their insides,
  // which leaves just the outer half of the strokes on the edge.
  let rangeCanvas = null;
  function drawRange(style) {
    if (range.length === 0) return;
    const ok = style.getPropertyValue('--ok').trim() || '#1f7a4d';
    const { w, h } = cssSize();
    const dpr = window.devicePixelRatio || 1;
    rangeCanvas = rangeCanvas || document.createElement('canvas');
    rangeCanvas.width = Math.round(w * dpr);
    rangeCanvas.height = Math.round(h * dpr);
    const g = rangeCanvas.getContext('2d');
    g.setTransform(dpr, 0, 0, dpr, 0, 0);
    const outline = (rings) => {
      g.beginPath();
      for (const ring of rings) {
        ring.forEach(([lon, lat], i) => {
          const [x, y] = project(lon, lat);
          if (i === 0) g.moveTo(x, y);
          else g.lineTo(x, y);
        });
        g.closePath();
      }
    };
    for (const hub of range) {
      g.globalCompositeOperation = 'source-over';
      g.strokeStyle = ok;
      g.lineWidth = 4;
      g.setLineDash([10, 6]);
      for (const ring of hub.rings) {
        outline([ring]);
        g.stroke();
      }
      g.setLineDash([]);
      g.globalCompositeOperation = 'destination-out';
      outline(hub.rings);
      g.fill('nonzero');
      g.globalCompositeOperation = 'source-over';
      g.globalAlpha = 0.07;
      g.fillStyle = ok;
      outline(hub.rings);
      g.fill('nonzero');
      g.globalAlpha = 1;
    }
    ctx.drawImage(rangeCanvas, 0, 0, w, h);
  }

  // Launch and recovery stations: friendly ground installations (air base),
  // named once the map is zoomed in far enough to have room.
  function drawStations(style) {
    if (stations.length === 0) return;
    const token = (name, fallback) => style.getPropertyValue(name).trim() || fallback;
    const ink = token('--ink', '#000');
    const panel = token('--panel', '#fff');
    const accent = token('--accent', '#2b6cb0');
    const pxPerKm = camera.scale / 40_075;
    const size = symbolSize(pxPerKm);
    ctx.font = '600 11px ui-sans-serif, system-ui, sans-serif';
    for (const s of stations) {
      const at = position(s);
      if (!at) continue;
      const [x, y] = project(...at);
      const mark = picked('station', s.id);
      if (mark) {
        ctx.beginPath();
        ctx.arc(x, y, size * 1.1, 0, Math.PI * 2);
        ctx.strokeStyle = accent;
        ctx.lineWidth = 3;
        ctx.stroke();
      }
      let right = stamp(ctx, SIDC.station, size, x, y);
      if (right === null) {
        ctx.fillStyle = panel;
        ctx.strokeStyle = ink;
        ctx.lineWidth = 1.5;
        ctx.fillRect(x - 6, y - 6, 12, 12);
        ctx.strokeRect(x - 6, y - 6, 12, 12);
        right = 6;
      }
      if (pxPerKm > 20 || mark) {
        const name = s.name || `Station ${s.id}`;
        const used = Number(s.occupied);
        const slots = Number(s.slots);
        const label = Number.isFinite(slots) && slots > 0
          ? `${name} · ${Number.isFinite(used) ? used : 0}/${slots} slots`
          : name;
        haloText(label, x + right + 4, y + 4, panel, ink);
      }
    }
  }

  // Regions of interest as APP-6 named areas of interest: a dashed outline
  // labelled "NAI" and the region's name. One with drones on their way to
  // survey it is heavier, in the accent colour, and says how many; one a
  // crash put there is in the alarm colour.
  function drawRois(style) {
    if (rois.length === 0) return;
    const token = (name, fallback) => style.getPropertyValue(name).trim() || fallback;
    const ink = token('--ink', '#000');
    const panel = token('--panel', '#fff');
    const accent = token('--accent', '#2b6cb0');
    const bad = token('--bad', '#a4232b');
    const pxPerKm = camera.scale / 40_075;
    for (const r of rois) {
      const at = position(r);
      const radius = Number(r.radius_m);
      if (!at || !(radius > 0)) continue;
      const mark = picked('roi', r.id);
      const tasked = Array.isArray(r.assigned) ? r.assigned.length : 0;
      const colour = mark || tasked ? accent : r.source === 'crash' ? bad : ink;
      trace(circleRing(at[0], at[1], radius));
      if (mark || tasked) {
        ctx.fillStyle = accent;
        ctx.globalAlpha = 0.1;
        ctx.fill();
        ctx.globalAlpha = 1;
      }
      ctx.strokeStyle = panel;
      ctx.lineWidth = 4.5;
      ctx.stroke();
      ctx.setLineDash([8, 5]);
      ctx.strokeStyle = colour;
      ctx.lineWidth = mark ? 3 : tasked ? 2.5 : 1.75;
      ctx.stroke();
      ctx.setLineDash([]);
      if (pxPerKm < 8 && !mark) continue;
      // The label sits inside the top of the area, or above it when the
      // area is too small on screen to hold it.
      const [cx, cy] = project(...at);
      const top = project(at[0], at[1] + radius / 111_320)[1];
      const roomy = cy - top > 30;
      ctx.font = '600 11px ui-sans-serif, system-ui, sans-serif';
      const lines = [`NAI ${r.name || `#${r.id}`}`];
      if (tasked) lines.push(`${tasked} drone${tasked === 1 ? '' : 's'} tasked to survey`);
      let y = roomy ? top + 15 : top - 6 - 13 * (lines.length - 1);
      for (const line of lines) {
        haloText(line, cx, y, panel, colour === ink ? ink : colour, 'center');
        ctx.font = '11px ui-sans-serif, system-ui, sans-serif';
        y += 13;
      }
    }
  }

  // The station nearest a point, within a symbol's reach on screen.
  function stationAt(lon, lat) {
    if (!camera) return null;
    const [x, y] = project(lon, lat);
    let best = null;
    let bestPx = 16;
    for (const s of stations) {
      const at = position(s);
      if (!at) continue;
      const [sx, sy] = project(...at);
      const px = Math.hypot(sx - x, sy - y);
      if (px < bestPx) {
        best = s;
        bestPx = px;
      }
    }
    return best;
  }

  // The smallest region of interest containing a point.
  function roiAt(lon, lat) {
    let best = null;
    for (const r of rois) {
      const at = position(r);
      if (!at) continue;
      const east = (lon - at[0]) * 111_320 * Math.cos((at[1] * Math.PI) / 180);
      const north = (lat - at[1]) * 111_320;
      if (Math.hypot(east, north) <= r.radius_m && (!best || r.radius_m < best.radius_m)) best = r;
    }
    return best;
  }

  // People and vehicles (the truth) are faint dots; delivery locations go
  // next; camera sightings are unknown ground (person, vehicle) or air
  // (intruder) symbols fading over SIGHTING_MS; POIs are small diamonds, so
  // a city full of addresses does not hide the drones; and drones go on
  // top, friendly air, intruders unknown air, drawn where they are
  // predicted to be now with a direction-of-movement line while they move.
  // Colours come from the theme so both modes read.
  function drawEntities(style, ink) {
    const token = (name, fallback) => style.getPropertyValue(name).trim() || fallback;
    const accent = token('--accent', '#2b6cb0');
    const bad = token('--bad', '#a4232b');
    const warn = token('--warn', '#8a6100');
    const panel = token('--panel', '#fff');
    const ok = token('--ok', '#1f7a4d');
    const now = performance.now();
    // Camera scale is pixels per Earth circumference (world units 0..1).
    const pxPerKm = camera.scale / 40_075;
    const size = symbolSize(pxPerKm);

    if (layers.people) {
      for (const e of people) {
        const [x, y] = project(e.lon, e.lat);
        ctx.fillStyle = e.kind === 'vehicle' ? accent : warn;
        ctx.globalAlpha = 0.55;
        if (e.kind === 'vehicle') ctx.fillRect(x - 2, y - 2, 4, 4);
        else {
          ctx.beginPath();
          ctx.arc(x, y, 1.8, 0, Math.PI * 2);
          ctx.fill();
        }
      }
      ctx.globalAlpha = 1;
    }

    if (layers.deliveries) {
      // Open orders: a ring while waiting for a drone, filled once one is
      // on its way. Recent drop-offs fade out over two minutes.
      for (const [lat, lon, flying] of orders) {
        const [x, y] = project(lon * 1e-7, lat * 1e-7);
        ctx.beginPath();
        ctx.arc(x, y, 4, 0, Math.PI * 2);
        ctx.strokeStyle = ok;
        ctx.lineWidth = 1.75;
        if (flying) {
          ctx.fillStyle = ok;
          ctx.fill();
        } else {
          ctx.fillStyle = panel;
          ctx.fill();
          ctx.stroke();
        }
      }
      for (const d of drops) {
        const age = (now - d.at) / DROP_MS;
        if (age >= 1) continue;
        const [x, y] = project(d.lon, d.lat);
        ctx.globalAlpha = 0.9 * (1 - age);
        ctx.strokeStyle = ok;
        ctx.lineWidth = 2;
        ctx.beginPath();
        ctx.moveTo(x - 4, y);
        ctx.lineTo(x - 1, y + 3);
        ctx.lineTo(x + 4, y - 3);
        ctx.stroke();
      }
      ctx.globalAlpha = 1;
    }

    for (const s of layers.people ? sightings : []) {
      const age = (now - s.at) / SIGHTING_MS;
      const sidc = sighted(s);
      if (age >= 1 || !sidc) continue;
      const [x, y] = project(s.lon, s.lat);
      if (stamp(ctx, sidc, size - 4, x, y, null, 0.9 * (1 - age)) !== null) continue;
      ctx.globalAlpha = 0.85 * (1 - age);
      ctx.fillStyle = s.kind === 'intruder' ? bad : s.kind === 'vehicle' ? accent : warn;
      ctx.beginPath();
      ctx.arc(x, y, 3.5, 0, Math.PI * 2);
      ctx.fill();
    }
    ctx.globalAlpha = 1;

    ctx.font = '11px ui-sans-serif, system-ui, sans-serif';
    ctx.lineWidth = 1.5;
    // POI names only once zoomed in to street scale; drone callsigns always.
    const namePois = pxPerKm > 400;
    // POIs first, so drones draw on top of them.
    const shown = entities.filter((e) => (e.kind === 'poi' ? layers.deliveries : layers.drones));
    const ordered = shown.sort((a, b) => (a.kind === 'poi' ? 0 : 1) - (b.kind === 'poi' ? 0 : 1));
    for (const e of ordered) {
      const at = e.kind === 'poi' ? null : predicted(e, now);
      const [x, y] = at ? project(at.lon, at.lat) : project(e.lon, e.lat);
      const moving = e.kind !== 'poi' && e.speed_ms > 0.3;
      let right = 9;
      if (e.kind === 'poi') {
        const r = namePois ? 5 : 3.5;
        ctx.globalAlpha = namePois ? 1 : 0.7;
        ctx.beginPath();
        ctx.moveTo(x, y - r);
        ctx.lineTo(x + r * 0.8, y);
        ctx.lineTo(x, y + r);
        ctx.lineTo(x - r * 0.8, y);
        ctx.closePath();
        ctx.fillStyle = panel;
        ctx.strokeStyle = ink;
        ctx.lineWidth = 1;
        ctx.fill();
        ctx.stroke();
        ctx.globalAlpha = 1;
        ctx.lineWidth = 1.5;
      } else {
        const sidc = e.kind === 'intruder' ? SIDC.intruder : SIDC.drone;
        const reach = stamp(ctx, sidc, size, x, y, moving ? at.hdg : null);
        if (reach === null) arrow(x, y, at.hdg, e.kind === 'intruder' ? bad : accent, panel);
        else right = reach + 4;
      }
      // Drones parked together on a roof would print their names on top of
      // each other: name the ones moving, and the rest once zoomed in. A
      // drone's label says what it is doing, or failing that its mode.
      if (e.callsign && (moving || namePois || (e.kind === 'intruder'))) {
        const doing = e.activity || e.mode;
        const label = e.kind === 'drone' && doing ? `${e.callsign} · ${doing}` : e.callsign;
        haloText(label, x + right, y + 4, panel, ink);
        ctx.lineWidth = 1.5;
      }
    }
  }

  async function refreshEntities() {
    try {
      const kinds = [
        ...(layers.drones ? ['drone', 'intruder'] : []),
        ...(layers.deliveries ? ['poi'] : []),
      ];
      const response = kinds.length
        ? await fetch(`/api/v1/state/entities?kind=${kinds.join(',')}`)
        : null;
      if (!response) entities = [];
      if (response && response.ok) {
        entities = await response.json();
        const tick = Number(response.headers.get('x-sim-tick'));
        updateClock(
          tick,
          response.headers.get('x-sim-mode') || '',
          Number(response.headers.get('x-sim-tick-seconds')) || 0,
          performance.now(),
        );
        recordSamples(tick);
      }

      const since = lastReport ?? 0;
      const reports = await fetch(`/api/v1/reports?since=${since}`);
      if (reports.ok) {
        const body = await reports.json();
        // The first poll only finds out where the stream is, so opening the
        // page does not replay minutes of old sightings at once.
        if (lastReport !== null) {
          const now = performance.now();
          for (const r of body.reports) {
            const at = { lat: r.lat_e7 * 1e-7, lon: r.lon_e7 * 1e-7, at: now };
            if (r.kind === 'detection') sightings.push({
              ...at, kind: r.target_kind, vehicleType: r.detail && r.detail.vehicle_type,
            });
            else if (r.kind === 'delivery') drops.push(at);
          }
        }
        lastReport = body.last_seq;
        const cutoff = performance.now() - SIGHTING_MS;
        sightings = sightings.filter((s) => s.at > cutoff).slice(-2000);
        drops = drops.filter((d) => d.at > performance.now() - DROP_MS).slice(-500);
      }
      if (camera) draw();
    } catch {
      // The occupancy map keeps working without the overlay.
    }
  }

  // People, vehicles, open orders, stations and regions of interest change
  // less often and cost more to send, so they come once a second, and only
  // while their layer is on. Each overlay key may be missing (a model that
  // has none, or an older one): that layer is then just empty.
  async function refreshSlow() {
    try {
      if (layers.people) {
        const response = await fetch('/api/v1/state/entities?kind=person,vehicle');
        if (response.ok) people = await response.json();
      } else {
        people = [];
      }
      if (layers.deliveries || layers.stations || layers.rois || layers.range) {
        const response = await fetch('/api/v1/state/overlays');
        if (response.ok) {
          const body = (await response.json()) || {};
          const list = (key, on) => (on && Array.isArray(body[key]) ? body[key] : []);
          orders = list('orders', layers.deliveries);
          stations = list('stations', layers.stations);
          rois = list('rois', layers.rois);
          setRange(layers.range ? body.range : null);
        }
      } else {
        orders = [];
        stations = [];
        rois = [];
        setRange(null);
      }
      if (camera) draw();
    } catch {
      // The map works without them.
    }
  }

  // Point-in-polygon on the projected ring, so hit testing matches what is
  // drawn even when the projection distorts.
  function cellAt(x, y) {
    for (let c = cells.length - 1; c >= 0; c--) {
      const ring = cells[c].boundary;
      if (!ring || ring.length < 3) continue;
      let inside = false;
      for (let i = 0, j = ring.length - 1; i < ring.length; j = i++) {
        const [xi, yi] = project(ring[i][0], ring[i][1]);
        const [xj, yj] = project(ring[j][0], ring[j][1]);
        if ((yi > y) !== (yj > y) && x < ((xj - xi) * (y - yi)) / (yj - yi) + xi) {
          inside = !inside;
        }
      }
      if (inside) return cells[c];
    }
    return null;
  }

  // -------------------------------------------------------------- data

  function displayResolution() {
    return resInput ? resInput.value : '';
  }

  async function refresh() {
    if (inFlight) return;
    inFlight = true;
    try {
      const res = displayResolution();
      const response = await fetch(`/api/v1/state/map${res ? `?res=${res}` : ''}`);
      if (!response.ok) {
        const problem = await response.json().catch(() => ({ detail: response.statusText }));
        status.textContent = problem.detail || 'The map could not be loaded.';
        return;
      }
      const body = await response.json();
      cells = body.cells;
      maxCount = body.max_count;

      if (cells.length === 0) {
        status.textContent = `Tick ${body.tick}: no occupied cells.`;
        draw();
        return;
      }

      let minX = Infinity, minY = Infinity, maxX = -Infinity, maxY = -Infinity;
      for (const cell of cells) {
        for (const [lon, lat] of cell.boundary) {
          const [x, y] = toWorld(lon, lat);
          if (x < minX) minX = x;
          if (x > maxX) maxX = x;
          if (y < minY) minY = y;
          if (y > maxY) maxY = y;
        }
      }
      const first = bounds === null;
      bounds = { minX, minY, maxX, maxY };
      if (first || !camera) fit();

      status.textContent =
        `Tick ${body.tick.toLocaleString()} · ${cells.length.toLocaleString()} cells at ` +
        `resolution ${body.resolution} · busiest holds ${body.max_count.toLocaleString()}` +
        (body.truncated ? ' · view was capped, some cells are missing' : '');
      if (legend) {
        legend.hidden = !layers.grid;
        legend.innerHTML = `<span>0</span><span class="ramp"></span><span>${body.max_count.toLocaleString()}</span>`;
      }
      draw();
    } catch (err) {
      status.textContent = `The map could not be loaded: ${err}`;
    } finally {
      inFlight = false;
    }
  }

  // Show `cell` in the inspector and tell live.js to keep it current.
  function inspect(cell, lon, lat, boundary) {
    selected = { cell, lon, lat, boundary };
    draw();
    if (!cellInput) return;
    cellInput.value = cell;
    if (window.htmx) {
      window.htmx.ajax('GET', `/fragments/cell?cell=${cell}`, { target: '#cell', swap: 'outerHTML' });
    }
    document.dispatchEvent(new CustomEvent('krabshell:cell', { detail: cell }));
  }

  // The display cell containing a point, which may be empty and so not drawn.
  function inspectAt(lon, lat) {
    const res = displayResolution();
    fetch(`/api/v1/cells/lookup?lon=${lon}&lat=${lat}${res ? `&res=${res}` : ''}`)
      .then((r) => (r.ok ? r.json() : null))
      .then((body) => {
        if (body && body.cell) inspect(body.cell, lon, lat, body.boundary);
      })
      .catch(() => {});
  }

  // ------------------------------------------------------- interaction

  let dragging = null;

  canvas.addEventListener('pointerdown', (event) => {
    canvas.setPointerCapture(event.pointerId);
    dragging = { x: event.offsetX, y: event.offsetY, moved: false };
  });

  canvas.addEventListener('pointermove', (event) => {
    if (dragging && camera) {
      const dx = event.offsetX - dragging.x;
      const dy = event.offsetY - dragging.y;
      if (Math.abs(dx) > 2 || Math.abs(dy) > 2) dragging.moved = true;
      camera.cx -= dx / camera.scale;
      camera.cy -= dy / camera.scale;
      dragging.x = event.offsetX;
      dragging.y = event.offsetY;
      draw();
      return;
    }
    if (!camera) return;
    if (tool) {
      if (tool.move) {
        const [lon, lat] = unproject(event.offsetX, event.offsetY);
        tool.move(lon, lat);
        draw();
      }
      return;
    }
    const found = cellAt(event.offsetX, event.offsetY);
    const next = found ? { ...found, x: event.offsetX, y: event.offsetY } : null;
    const changed = (next && next.cell) !== (hover && hover.cell);
    hover = next;
    if (changed || next) draw();
  });

  canvas.addEventListener('pointerup', (event) => {
    const wasDrag = dragging && dragging.moved;
    dragging = null;
    if (wasDrag || !camera) return;

    // Inspect exactly the polygon that was clicked: the cell at the display
    // resolution, whose count is what the map is showing.
    const [lon, lat] = unproject(event.offsetX, event.offsetY);
    if (tool && tool.click && tool.click(lon, lat, event)) {
      draw();
      return;
    }
    const found = cellAt(event.offsetX, event.offsetY);
    if (found) inspect(found.cell, lon, lat, found.boundary);
    else inspectAt(lon, lat);
  });

  canvas.addEventListener('wheel', (event) => {
    if (!camera) return;
    event.preventDefault();
    const before = screenToWorld(event.offsetX, event.offsetY);
    const factor = Math.exp(-event.deltaY * 0.0015);
    camera.scale = Math.min(Math.max(camera.scale * factor, TILE / 2), TILE * 2 ** 24);
    const after = screenToWorld(event.offsetX, event.offsetY);
    // Keep the point under the cursor fixed while zooming.
    camera.cx += before[0] - after[0];
    camera.cy += before[1] - after[1];
    draw();
  }, { passive: false });

  canvas.addEventListener('pointerleave', () => {
    hover = null;
    draw();
  });

  if (fitButton) fitButton.addEventListener('click', () => { fit(); draw(); });

  if (resInput) {
    resInput.addEventListener('input', () => {
      if (resOut) resOut.value = resInput.value;
    });
    resInput.addEventListener('change', async () => {
      // Keep the camera: the operator changed the detail, not the place.
      await refresh();
      // The inspector follows the display resolution: re-pick the cell under
      // the point the operator chose, at the new resolution.
      if (selected && selected.lon !== undefined) inspectAt(selected.lon, selected.lat);
    });
  }

  if (basemapSelect) {
    try {
      const saved = localStorage.getItem('krabshell.basemap');
      if (saved !== null && [...basemapSelect.options].some((o) => o.value === saved)) {
        basemapSelect.value = saved;
      }
    } catch { /* no storage: start with no basemap */ }
    basemapSelect.addEventListener('change', setBasemap);
    basemap = readBasemap();
    updateAttribution();
  }

  // A cell typed into the inspector is outlined too, once it is on the map.
  if (cellInput) {
    cellInput.addEventListener('change', () => {
      selected = { cell: cellInput.value.trim() };
      draw();
    });
    if (cellInput.value.trim()) selected = { cell: cellInput.value.trim() };
  }

  window.addEventListener('resize', draw);
  // The page width can change without the window changing (the Width picker).
  if (window.ResizeObserver) new ResizeObserver(() => draw()).observe(canvas);

  for (const input of layerInputs) {
    input.checked = layers[input.dataset.layer] !== false;
    input.addEventListener('change', () => {
      layers[input.dataset.layer] = input.checked;
      const own = pageOff.indexOf(input.dataset.layer);
      // Switched by hand, it is this page's to remember like any other.
      if (own >= 0) pageOff.splice(own, 1);
      rememberLayers();
      if (legend) legend.hidden = !layers.grid;
      if (!layers.grid) hover = null;
      refreshEntities();
      refreshSlow();
      draw();
    });
  }

  // For pages that build on the map.
  window.krabshellMap = {
    setTool(next) {
      tool = next;
      hover = null;
      canvas.classList.toggle('drawing', Boolean(next && next.drawing));
      draw();
    },
    redraw: scheduleDraw,
    refreshFences,
    fenceAt,
    circleRing,
    // Screen distance in pixels between two [lon, lat] points.
    pixels(a, b) {
      if (!camera) return Infinity;
      const [ax, ay] = project(a[0], a[1]);
      const [bx, by] = project(b[0], b[1]);
      return Math.hypot(ax - bx, ay - by);
    },
    selectFence(id) {
      fenceSelected = id;
      draw();
    },
    // Centre on a fence and zoom to show it whole.
    showFence(id) {
      const f = fences.find((x) => x.id === id);
      if (!f) return;
      fenceSelected = id;
      showRing(fenceRing(f));
    },
    fenceBand,
    refreshOverlays: refreshSlow,
    stationAt,
    roiAt,
    // Pick out a station or region of interest ({ kind: 'station' | 'roi',
    // id }), or nothing (null).
    selectMark(mark) {
      markSelected = mark;
      draw();
    },
    // Centre on a point and zoom to show `radiusM` round it.
    showPlace(lon, lat, radiusM) {
      showRing(circleRing(lon, lat, Math.max(radiusM, 50), 16));
    },
  };

  function showRing(ring) {
    let minX = Infinity, minY = Infinity, maxX = -Infinity, maxY = -Infinity;
    for (const [lon, lat] of ring) {
      const [x, y] = toWorld(lon, lat);
      minX = Math.min(minX, x); maxX = Math.max(maxX, x);
      minY = Math.min(minY, y); maxY = Math.max(maxY, y);
    }
    const { w, h } = cssSize();
    const span = Math.max(maxX - minX, maxY - minY, 1e-9);
    camera = {
      cx: (minX + maxX) / 2,
      cy: (minY + maxY) / 2,
      scale: Math.min(Math.min(w, h) / span / 3, TILE * 2 ** 24),
    };
    draw();
  }

  window.addEventListener('keydown', (event) => {
    if (tool && tool.key && tool.key(event)) draw();
  });

  drawKeys();
  refresh();
  refreshEntities();
  refreshFences();
  refreshSlow();
  // Occupancy once a second; drones several times a second, so a correction
  // is never more than a quarter of a second of prediction.
  setInterval(() => {
    if (!follow || follow.checked) refresh();
    // Fences keep coming whether or not the map follows the run, so the
    // list beside it stays true.
    refreshFences();
    if (!follow || follow.checked) refreshSlow();
  }, 1000);
  let entitiesInFlight = false;
  setInterval(async () => {
    if ((follow && !follow.checked) || entitiesInFlight) return;
    entitiesInFlight = true;
    try {
      await refreshEntities();
    } finally {
      entitiesInFlight = false;
    }
  }, ENTITY_POLL_MS);
})();
