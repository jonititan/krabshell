// Geofences page: the list of open fences, recent fence activity, and tools
// to draw new fences on the map.
//
// map.js draws the fences and polls `/api/v1/geofences` once a second; this
// file listens for each poll (`krabshell:geofences`) to keep the tables
// current, and hands map.js a tool that turns clicks into a circle (centre,
// then edge) or a polygon (corners, closed by clicking the first again, by
// Enter, or by Open fence). Fences open with POST /api/v1/geofences and close
// with DELETE /api/v1/geofences/{id}; both take effect on the next tick.

(function () {
  'use strict';

  const map = window.krabshellMap;
  if (!map) return;

  const $ = (id) => document.getElementById(id);
  const hint = $('fence-hint');
  const draftForm = $('fence-draft');
  const draftTitle = $('draft-title');
  const radiusField = $('draft-radius-field');
  const radiusInput = $('draft-radius');
  const minutesInput = $('draft-minutes');
  const summary = $('draft-summary');
  const draftError = $('draft-error');
  const undoButton = $('draft-undo');
  const tableBody = document.querySelector('#fence-table tbody');
  const eventsBody = document.querySelector('#fence-events tbody');
  const flash = $('fence-flash');
  const toolButtons = [...document.querySelectorAll('[data-tool]')];

  const MIN_RADIUS_M = 1;
  const MAX_RADIUS_M = 50_000;
  const CLOSE_PX = 12; // a click this close to the first corner closes a polygon

  const HINTS = {
    select: 'Click a fence to pick it out in the list. Drag to pan, scroll to zoom.',
    circle: 'Click the centre, then click again at the edge (or type the radius). Esc cancels.',
    polygon: 'Click each corner. Click the first corner again, press Enter, or Open fence to finish. Esc cancels.',
  };

  let mode = 'select';
  // The fence being drawn: { kind: 'circle', centre, radius, placed } or
  // { kind: 'polygon', corners: [[lon, lat]...] }, and where the pointer is.
  let draft = null;
  let pointer = null;
  let last = null; // the latest /api/v1/geofences body
  let selected = null;
  // Intents sent while the world was paused, by id, with what they were:
  // the model checks them only when it next steps.
  const pending = new Map();

  // ------------------------------------------------------------ geometry

  // Metres east and north of `from`, for lengths and areas on the ground.
  function metres(from, to) {
    const k = (Math.PI / 180) * 6_371_000;
    return [
      (to[0] - from[0]) * k * Math.cos((from[1] * Math.PI) / 180),
      (to[1] - from[1]) * k,
    ];
  }

  function distance(a, b) {
    const [e, n] = metres(a, b);
    return Math.hypot(e, n);
  }

  function areaKm2(ring) {
    if (ring.length < 3) return 0;
    const pts = ring.map((p) => metres(ring[0], p));
    let a = 0;
    for (let i = 0; i < pts.length; i++) {
      const [x1, y1] = pts[i];
      const [x2, y2] = pts[(i + 1) % pts.length];
      a += x1 * y2 - x2 * y1;
    }
    return Math.abs(a) / 2 / 1e6;
  }

  function fmtArea(km2) {
    return km2 >= 0.1 ? `${km2.toFixed(2)} km²` : `${Math.round(km2 * 1e6).toLocaleString()} m²`;
  }

  function fmtLength(m) {
    return m >= 1000 ? `${(m / 1000).toFixed(2)} km` : `${Math.round(m)} m`;
  }

  // ---------------------------------------------------------------- tool

  function setMode(next) {
    mode = next;
    draft = null;
    pointer = null;
    for (const b of toolButtons) b.setAttribute('aria-pressed', String(b.dataset.tool === mode));
    hint.textContent = HINTS[mode];
    map.setTool(mode === 'select' ? selectTool : { ...drawTool, drawing: true });
    renderDraft();
  }

  const selectTool = {
    click(lon, lat) {
      const f = map.fenceAt(lon, lat);
      select(f ? f.id : null);
      // Clicks here pick fences; there is no cell inspector on this page.
      return true;
    },
  };

  const drawTool = {
    click(lon, lat) {
      const at = [lon, lat];
      if (mode === 'circle') {
        if (!draft || draft.placed) {
          draft = { kind: 'circle', centre: at, radius: 0, placed: false };
        } else {
          draft.radius = Math.max(MIN_RADIUS_M, Math.round(distance(draft.centre, at)));
          draft.placed = true;
          radiusInput.focus();
        }
      } else {
        if (!draft) draft = { kind: 'polygon', corners: [] };
        const first = draft.corners[0];
        if (draft.corners.length >= 3 && first && map.pixels(first, at) <= CLOSE_PX) {
          submit();
          return true;
        }
        draft.corners.push(at);
      }
      renderDraft();
      return true;
    },
    move(lon, lat) {
      pointer = [lon, lat];
      if (draft && draft.kind === 'circle' && !draft.placed) {
        draft.radius = Math.round(distance(draft.centre, pointer));
        renderSummary();
      }
    },
    key(event) {
      if (event.target instanceof HTMLInputElement) return false;
      if (event.key === 'Escape') {
        setMode(mode);
        return true;
      }
      if (event.key === 'Enter' && draft && draft.kind === 'polygon') {
        submit();
        return true;
      }
      if ((event.key === 'Backspace' || event.key === 'Delete') && draft && draft.kind === 'polygon') {
        draft.corners.pop();
        renderDraft();
        return true;
      }
      return false;
    },
    draw(ctx, project, style) {
      if (!draft) return;
      const accent = style.getPropertyValue('--accent').trim() || '#2b6cb0';
      const panel = style.getPropertyValue('--panel').trim() || '#fff';
      let ring = [];
      let open = false;
      if (draft.kind === 'circle') {
        if (draft.radius > 0) ring = map.circleRing(draft.centre[0], draft.centre[1], draft.radius);
      } else {
        ring = [...draft.corners];
        if (pointer) ring.push(pointer);
        open = draft.corners.length < 2;
      }
      if (ring.length >= 2) {
        ctx.beginPath();
        ring.forEach(([lon, lat], i) => {
          const [x, y] = project(lon, lat);
          if (i === 0) ctx.moveTo(x, y);
          else ctx.lineTo(x, y);
        });
        if (!open) ctx.closePath();
        ctx.fillStyle = accent;
        ctx.globalAlpha = 0.15;
        if (!open) ctx.fill();
        ctx.globalAlpha = 1;
        ctx.setLineDash([6, 4]);
        ctx.strokeStyle = accent;
        ctx.lineWidth = 2;
        ctx.stroke();
        ctx.setLineDash([]);
      }
      const dots = draft.kind === 'circle' ? [draft.centre] : draft.corners;
      dots.forEach(([lon, lat], i) => {
        const [x, y] = project(lon, lat);
        ctx.beginPath();
        ctx.arc(x, y, i === 0 && draft.kind === 'polygon' ? 6 : 4, 0, Math.PI * 2);
        ctx.fillStyle = panel;
        ctx.fill();
        ctx.strokeStyle = accent;
        ctx.lineWidth = 2;
        ctx.stroke();
      });
    },
  };

  // --------------------------------------------------------------- draft

  function ready() {
    if (!draft) return false;
    if (draft.kind === 'circle') return draft.placed && draft.radius >= MIN_RADIUS_M;
    return draft.corners.length >= 3;
  }

  function renderSummary() {
    if (!draft) {
      summary.textContent = '';
      return;
    }
    if (draft.kind === 'circle') {
      const [lon, lat] = draft.centre;
      summary.textContent =
        `centre ${lat.toFixed(5)}, ${lon.toFixed(5)}` +
        (draft.radius > 0 ? ` · ${fmtArea(Math.PI * draft.radius ** 2 / 1e6)}` : '');
      if (!draft.placed) radiusInput.value = draft.radius > 0 ? String(draft.radius) : '';
    } else {
      const n = draft.corners.length;
      summary.textContent =
        `${n} corner${n === 1 ? '' : 's'}` + (n >= 3 ? ` · ${fmtArea(areaKm2(draft.corners))}` : ' · at least 3 needed');
    }
    $('draft-open').disabled = !ready();
  }

  function renderDraft() {
    draftForm.hidden = !draft;
    draftError.hidden = true;
    if (!draft) {
      map.redraw();
      return;
    }
    const circle = draft.kind === 'circle';
    draftTitle.textContent = circle ? 'New circular fence' : 'New polygon fence';
    radiusField.hidden = !circle;
    undoButton.hidden = circle;
    if (circle && draft.placed) radiusInput.value = String(draft.radius);
    renderSummary();
    map.redraw();
  }

  radiusInput.addEventListener('input', () => {
    if (!draft || draft.kind !== 'circle') return;
    const r = Number(radiusInput.value);
    if (Number.isFinite(r) && r > 0) {
      draft.radius = Math.round(r);
      draft.placed = true;
      renderSummary();
      map.redraw();
    }
  });

  undoButton.addEventListener('click', () => {
    if (draft && draft.kind === 'polygon') draft.corners.pop();
    renderDraft();
  });

  $('draft-cancel').addEventListener('click', () => setMode(mode));

  draftForm.addEventListener('submit', (event) => {
    event.preventDefault();
    submit();
  });

  // The model's own words, without the driver's preamble.
  function reason(text) {
    return String(text).replace(/^model rejected the intent: /, '');
  }

  function showError(text) {
    draftError.textContent = reason(text);
    draftError.hidden = false;
  }

  async function submit() {
    if (!ready()) {
      showError(draft && draft.kind === 'polygon' ? 'A polygon needs at least 3 corners.' : 'Place the edge, or type a radius.');
      return;
    }
    const body = {};
    if (draft.kind === 'circle') {
      if (draft.radius > MAX_RADIUS_M) {
        showError(`The radius can be at most ${MAX_RADIUS_M.toLocaleString()} m.`);
        return;
      }
      body.circle = { lat: draft.centre[1], lon: draft.centre[0], radius_m: draft.radius };
    } else {
      body.polygon = draft.corners.map(([lon, lat]) => [lon, lat]);
    }
    const name = $('draft-name').value.trim();
    if (name) body.name = name;
    const minutesText = minutesInput.value.trim();
    if (minutesText) {
      const minutes = Number(minutesText);
      if (!Number.isFinite(minutes) || minutes <= 0) {
        showError('Minutes must be a positive number, or blank to stay open until closed.');
        return;
      }
      body.minutes = minutes;
    }
    $('draft-open').disabled = true;
    try {
      const response = await fetch('/api/v1/geofences', {
        method: 'POST',
        headers: { 'content-type': 'application/json' },
        body: JSON.stringify(body),
      });
      const result = await response.json().catch(() => ({}));
      if (!response.ok) {
        showError(result.detail || `The fence was refused (${response.status}).`);
        $('draft-open').disabled = false;
        return;
      }
      if (result.status === 'queued') {
        pending.set(result.id, { what: 'The fence', at: Date.now() });
        say('Fence queued: it opens when the simulation next steps.', 'warn');
      } else {
        say('Fence opened. The drones the simulation flies are being rerouted round it.', 'ok');
      }
      minutesInput.value = '';
      $('draft-name').value = '';
      setMode(mode);
      map.refreshFences();
    } catch (err) {
      showError(`The fence could not be sent: ${err}`);
      $('draft-open').disabled = false;
    }
  }

  // -------------------------------------------------------------- tables

  function say(text, kind) {
    flash.textContent = text;
    flash.className = `banner ${kind}`;
    flash.hidden = false;
    clearTimeout(say.timer);
    say.timer = setTimeout(() => { flash.hidden = true; }, 6000);
  }

  const SOURCES = { scenario: 'Scenario', gcs: 'Ground station', dashboard: 'Dashboard', intent: 'API' };

  function simTime(tick) {
    const s = last && last.tick_seconds ? tick * last.tick_seconds : null;
    if (s === null) return `tick ${tick.toLocaleString()}`;
    const t = Math.floor(s);
    const d = Math.floor(t / 86_400);
    const hms = [Math.floor((t % 86_400) / 3600), Math.floor((t % 3600) / 60), t % 60]
      .map((v) => String(v).padStart(2, '0'))
      .join(':');
    return d > 0 ? `${d} d ${hms}` : hms;
  }

  function closesIn(f) {
    if (f.closes_tick == null) return 'when closed';
    if (!last || !last.tick_seconds) return `tick ${f.closes_tick.toLocaleString()}`;
    const s = Math.max(0, (f.closes_tick - last.tick) * last.tick_seconds);
    if (s < 60) return `in ${Math.round(s)} s`;
    if (s < 3600) return `in ${Math.round(s / 60)} min`;
    return `in ${(s / 3600).toFixed(1)} h`;
  }

  function shapeText(f) {
    if (f.shape === 'circle') {
      return `Circle · r ${fmtLength(f.radius_m)} · ${fmtArea(Math.PI * f.radius_m ** 2 / 1e6)}`;
    }
    const ring = (f.points || []).map(([lat, lon]) => [lon * 1e-7, lat * 1e-7]);
    return `Polygon · ${ring.length} corners · ${fmtArea(areaKm2(ring))}`;
  }

  function cell(text, cls) {
    const td = document.createElement('td');
    td.textContent = text;
    if (cls) td.className = cls;
    return td;
  }

  function nameCell(f) {
    const td = document.createElement('td');
    const id = document.createElement('span');
    id.className = 'mono';
    id.textContent = `#${f.id}`;
    td.append(id);
    if (f.name) td.append(` ${f.name}`);
    return td;
  }

  function select(id) {
    selected = id;
    map.selectFence(id);
    for (const row of tableBody.rows) row.classList.toggle('current', Number(row.dataset.id) === id);
  }

  async function close(id, button) {
    button.disabled = true;
    try {
      const response = await fetch(`/api/v1/geofences/${id}`, { method: 'DELETE' });
      const result = await response.json().catch(() => ({}));
      if (!response.ok) {
        say(reason(result.detail || `Fence ${id} could not be closed (${response.status}).`), 'bad');
        button.disabled = false;
        return;
      }
      if (result.status === 'queued') {
        pending.set(result.id, { what: `Closing fence ${id}`, at: Date.now() });
        say(`Fence ${id} closes when the simulation next steps.`, 'warn');
      } else {
        say(`Fence ${id} closed.`, 'ok');
      }
      map.refreshFences();
    } catch (err) {
      say(`Fence ${id} could not be closed: ${err}`, 'bad');
      button.disabled = false;
    }
  }

  function renderFences(body) {
    const fences = body.fences || [];
    $('fence-empty').hidden = fences.length > 0;
    $('fence-table').hidden = fences.length === 0;
    $('fence-meta').textContent = body.margin_m
      ? `Drones the simulation flies keep ${Math.round(body.margin_m)} m clear · airspace revision ${body.rev}`
      : '';
    // Rebuild only when something changed, so a button being pressed is not
    // swept away by the next poll.
    const key = JSON.stringify(fences) + body.tick_seconds + Math.floor(body.tick / 10);
    if (renderFences.key === key) return;
    renderFences.key = key;
    tableBody.replaceChildren(
      ...fences.map((f) => {
        const tr = document.createElement('tr');
        tr.dataset.id = f.id;
        tr.className = f.id === selected ? 'current' : '';
        tr.tabIndex = 0;
        tr.title = 'Show on the map';
        tr.append(
          nameCell(f),
          cell(shapeText(f)),
          cell(SOURCES[f.source] || f.source),
          cell(simTime(f.opened_tick), 'mono small'),
          cell(closesIn(f)),
          cell(String(f.breaches || 0), f.breaches ? 'breach' : ''),
        );
        const action = document.createElement('td');
        const button = document.createElement('button');
        button.type = 'button';
        button.className = 'danger';
        button.textContent = 'Close';
        button.title = f.source === 'scenario'
          ? 'Close this fence of the scenario\'s (a reset opens it again)'
          : 'Close this fence';
        button.addEventListener('click', (event) => {
          event.stopPropagation();
          close(f.id, button);
        });
        action.append(button);
        tr.append(action);
        const show = () => {
          select(f.id);
          map.showFence(f.id);
        };
        tr.addEventListener('click', show);
        tr.addEventListener('keydown', (event) => {
          if (event.key === 'Enter') show();
        });
        return tr;
      }),
    );
  }

  function describe(r) {
    const d = r.detail || {};
    switch (r.kind) {
      case 'geofence': {
        const opened = d.action === 'open';
        const who = SOURCES[d.source] || d.source;
        const shape = d.shape === 'circle'
          ? `circle r ${fmtLength(d.radius_m)}`
          : `polygon, ${(d.points || []).length} corners`;
        const named = d.name ? `${d.name} · ` : '';
        return [
          opened ? 'Opened' : 'Closed',
          d.id,
          opened
            ? `${named}${who} · ${shape}`
            : `${named}${d.reason === 'expired' ? 'its time was up' : 'on request'}`,
          '',
        ];
      }
      case 'breach':
        return [
          'Breach',
          d.fence,
          `${d.callsign || `drone ${r.observer}`}: ${d.how} · flown by ${d.flown_by}`,
          'bad',
        ];
      case 'reroute':
        return [
          'Reroute',
          null,
          `${d.callsign || `drone ${r.observer}`}: ${d.reason}` +
            (d.detours ? ` · ${d.detours} detour waypoint${d.detours === 1 ? '' : 's'}` : '') +
            (d.messages ? ` · ${d.messages.join(', ')}` : ''),
          '',
        ];
      default:
        return null;
    }
  }

  function renderEvents(body) {
    const rows = (body.recent || []).map((r) => [r, describe(r)]).filter(([, d]) => d);
    $('events-empty').hidden = rows.length > 0;
    $('fence-events').hidden = rows.length === 0;
    const key = rows.length ? rows[0][0].seq : 0;
    if (renderEvents.key === key) return;
    renderEvents.key = key;
    eventsBody.replaceChildren(
      ...rows.map(([r, [what, fence, detail, cls]]) => {
        const tr = document.createElement('tr');
        if (cls === 'bad') tr.className = 'rejected';
        tr.append(
          cell(simTime(r.tick), 'mono small'),
          cell(what),
          cell(fence == null ? '' : `#${fence}`, 'mono'),
          cell(detail, 'small'),
        );
        return tr;
      }),
    );
  }

  // A queued request the model then refused: say so, with its reason.
  function checkPending(body) {
    for (const r of body.refused || []) {
      const p = pending.get(r.id);
      if (!p) continue;
      pending.delete(r.id);
      say(`${p.what} was refused: ${reason(r.detail)}`, 'bad');
    }
    const stale = Date.now() - 10 * 60_000;
    for (const [id, p] of pending) if (p.at < stale) pending.delete(id);
  }

  document.addEventListener('krabshell:geofences', (event) => {
    last = event.detail;
    checkPending(last);
    // A model without geofences publishes no `margin_m`.
    $('fence-unavailable').hidden = Boolean(last.margin_m);
    renderFences(last);
    renderEvents(last);
  });

  for (const b of toolButtons) b.addEventListener('click', () => setMode(b.dataset.tool));
  setMode('select');
})();
