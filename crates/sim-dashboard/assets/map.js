// Occupancy map.
//
// The server does the expensive half: `/api/v1/state/map` returns cells already
// aggregated to a display resolution, each with its boundary ring. This file
// projects, draws, optionally lays a raster basemap underneath, and turns a
// click into the display cell to inspect.
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
  const entitiesToggle = document.getElementById('map-entities');

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

  // Individual things drawn over the occupancy cells: drones, intruders and
  // points of interest from /state/entities, and recent sightings from /reports.
  let entities = [];
  let sightings = [];     // { lat, lon, kind, at }
  let lastReport = null;  // seq of the newest report seen; null until primed
  const SIGHTING_MS = 30_000;

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
    for (const cell of cells) {
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

    if (!entitiesToggle || entitiesToggle.checked) drawEntities(style, ink);

    if (hover) {
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

  // Sightings fade over SIGHTING_MS; drones are arrows along their heading;
  // POIs are diamonds. Colours come from the theme so both modes read.
  function drawEntities(style, ink) {
    const token = (name, fallback) => style.getPropertyValue(name).trim() || fallback;
    const accent = token('--accent', '#2b6cb0');
    const bad = token('--bad', '#a4232b');
    const warn = token('--warn', '#8a6100');
    const panel = token('--panel', '#fff');
    const now = performance.now();

    for (const s of sightings) {
      const age = (now - s.at) / SIGHTING_MS;
      if (age >= 1) continue;
      const [x, y] = project(s.lon, s.lat);
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
    // Camera scale is pixels per Earth circumference (world units 0..1).
    const pxPerKm = camera.scale / 40_075;
    const namePois = pxPerKm > 400;
    for (const e of entities) {
      const [x, y] = project(e.lon, e.lat);
      if (e.kind === 'poi') {
        ctx.beginPath();
        ctx.moveTo(x, y - 6);
        ctx.lineTo(x + 5, y);
        ctx.lineTo(x, y + 6);
        ctx.lineTo(x - 5, y);
        ctx.closePath();
        ctx.fillStyle = panel;
        ctx.strokeStyle = ink;
        ctx.fill();
        ctx.stroke();
      } else {
        // An arrow pointing along the heading; screen y grows downward.
        const a = (e.heading_deg * Math.PI) / 180;
        const tip = [x + Math.sin(a) * 9, y - Math.cos(a) * 9];
        const left = [x + Math.sin(a + 2.5) * 6, y - Math.cos(a + 2.5) * 6];
        const right = [x + Math.sin(a - 2.5) * 6, y - Math.cos(a - 2.5) * 6];
        ctx.beginPath();
        ctx.moveTo(...tip);
        ctx.lineTo(...left);
        ctx.lineTo(x, y);
        ctx.lineTo(...right);
        ctx.closePath();
        ctx.fillStyle = e.kind === 'intruder' ? bad : accent;
        ctx.strokeStyle = panel;
        ctx.fill();
        ctx.stroke();
      }
      if (e.callsign && (e.kind !== 'poi' || namePois)) {
        const label = e.kind === 'drone' && e.mode ? `${e.callsign} · ${e.mode}` : e.callsign;
        ctx.lineWidth = 3;
        ctx.strokeStyle = panel;
        ctx.strokeText(label, x + 9, y + 4);
        ctx.fillStyle = ink;
        ctx.fillText(label, x + 9, y + 4);
        ctx.lineWidth = 1.5;
      }
    }
  }

  async function refreshEntities() {
    if (entitiesToggle && !entitiesToggle.checked) return;
    try {
      const response = await fetch('/api/v1/state/entities?kind=drone,intruder,poi');
      if (response.ok) entities = await response.json();

      const since = lastReport ?? 0;
      const reports = await fetch(`/api/v1/reports?since=${since}`);
      if (reports.ok) {
        const body = await reports.json();
        // The first poll only finds out where the stream is, so opening the
        // page does not replay minutes of old sightings at once.
        if (lastReport !== null) {
          const now = performance.now();
          for (const r of body.reports) {
            if (r.kind !== 'detection') continue;
            sightings.push({ lat: r.lat_e7 * 1e-7, lon: r.lon_e7 * 1e-7, kind: r.target_kind, at: now });
          }
        }
        lastReport = body.last_seq;
        const cutoff = performance.now() - SIGHTING_MS;
        sightings = sightings.filter((s) => s.at > cutoff).slice(-2000);
      }
      if (camera) draw();
    } catch {
      // The occupancy map keeps working without the overlay.
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

  if (entitiesToggle) entitiesToggle.addEventListener('change', () => { refreshEntities(); draw(); });

  refresh();
  refreshEntities();
  setInterval(() => {
    if (!follow || follow.checked) {
      refresh();
      refreshEntities();
    }
  }, 1000);
})();
