// Stations & regions page: launch and recovery stations, regions of
// interest for the drones to survey, what has happened to both lately, and
// tools to add them on the map.
//
// map.js draws both from the model's overlays; this file polls
// `/api/v1/stations` and `/api/v1/rois` once a second for the tables, and
// hands map.js a tool that turns a click into a station, or a centre and an
// edge into a region. They are added with POST and removed with DELETE on
// the same paths; both take effect on the next tick.

(function () {
  'use strict';

  const map = window.krabshellMap;
  if (!map) return;

  const $ = (id) => document.getElementById(id);
  const hint = $('mark-hint');
  const draftForm = $('mark-draft');
  const draftTitle = $('mark-title');
  const nameInput = $('mark-name');
  const radiusField = $('mark-radius-field');
  const radiusInput = $('mark-radius');
  const summary = $('mark-summary');
  const draftError = $('mark-error');
  const addButton = $('mark-add');
  const flash = $('mark-flash');
  const stationBody = document.querySelector('#station-table tbody');
  const roiBody = document.querySelector('#roi-table tbody');
  const eventsBody = document.querySelector('#mark-events tbody');
  const toolButtons = [...document.querySelectorAll('[data-tool]')];

  const MIN_RADIUS_M = 10;
  const MAX_RADIUS_M = 5000;

  const HINTS = {
    select: 'Click a station or region to pick it out in its list. Drag to pan, scroll to zoom.',
    station: 'Click where the station goes, then name it. Esc cancels.',
    roi: 'Click the centre, then click again at the edge (or type the radius), then name it. Esc cancels.',
  };

  let mode = 'select';
  // What is being placed: { kind: 'station', at } or { kind: 'roi', centre,
  // radius, placed }, all positions [lon, lat].
  let draft = null;
  let stations = null; // the latest /api/v1/stations body
  let rois = null;     // the latest /api/v1/rois body
  let selected = null; // { kind, id }
  let callsigns = new Map(); // drone agent id -> callsign, for tasked drones
  // Intents sent while the world was paused, by id, with what they were:
  // the model checks them only when it next steps.
  const pending = new Map();

  // ------------------------------------------------------------ geometry

  function distance(a, b) {
    const k = (Math.PI / 180) * 6_371_000;
    const east = (b[0] - a[0]) * k * Math.cos((a[1] * Math.PI) / 180);
    const north = (b[1] - a[1]) * k;
    return Math.hypot(east, north);
  }

  function fmtLength(m) {
    return m >= 1000 ? `${(m / 1000).toFixed(2)} km` : `${Math.round(m)} m`;
  }

  function degrees(item) {
    return [Number(item.lon_e7) * 1e-7, Number(item.lat_e7) * 1e-7];
  }

  // ---------------------------------------------------------------- tool

  function setMode(next) {
    mode = next;
    draft = null;
    for (const b of toolButtons) b.setAttribute('aria-pressed', String(b.dataset.tool === mode));
    hint.textContent = HINTS[mode];
    map.setTool(mode === 'select' ? selectTool : { ...placeTool, drawing: true });
    renderDraft();
  }

  const selectTool = {
    click(lon, lat) {
      const station = map.stationAt(lon, lat);
      const roi = station ? null : map.roiAt(lon, lat);
      if (station) select({ kind: 'station', id: station.id });
      else if (roi) select({ kind: 'roi', id: roi.id });
      else select(null);
      // Clicks here pick stations and regions; there is no cell inspector.
      return true;
    },
  };

  const placeTool = {
    click(lon, lat) {
      const at = [lon, lat];
      if (mode === 'station') {
        draft = { kind: 'station', at };
        nameInput.focus();
      } else if (!draft || draft.placed) {
        draft = { kind: 'roi', centre: at, radius: 0, placed: false };
      } else {
        draft.radius = clampRadius(distance(draft.centre, at));
        draft.placed = true;
        nameInput.focus();
      }
      renderDraft();
      return true;
    },
    move(lon, lat) {
      if (draft && draft.kind === 'roi' && !draft.placed) {
        draft.radius = Math.round(distance(draft.centre, [lon, lat]));
        renderSummary();
      }
    },
    key(event) {
      if (event.target instanceof HTMLInputElement) return false;
      if (event.key === 'Escape') {
        setMode(mode);
        return true;
      }
      return false;
    },
    draw(ctx, project, style) {
      if (!draft) return;
      const accent = style.getPropertyValue('--accent').trim() || '#2b6cb0';
      const panel = style.getPropertyValue('--panel').trim() || '#fff';
      const centre = draft.kind === 'station' ? draft.at : draft.centre;
      if (draft.kind === 'roi' && draft.radius > 0) {
        const ring = map.circleRing(centre[0], centre[1], draft.radius);
        ctx.beginPath();
        ring.forEach(([lon, lat], i) => {
          const [x, y] = project(lon, lat);
          if (i === 0) ctx.moveTo(x, y);
          else ctx.lineTo(x, y);
        });
        ctx.closePath();
        ctx.fillStyle = accent;
        ctx.globalAlpha = 0.15;
        ctx.fill();
        ctx.globalAlpha = 1;
        ctx.setLineDash([6, 4]);
        ctx.strokeStyle = accent;
        ctx.lineWidth = 2;
        ctx.stroke();
        ctx.setLineDash([]);
      }
      const [x, y] = project(centre[0], centre[1]);
      ctx.beginPath();
      ctx.arc(x, y, draft.kind === 'station' ? 7 : 4, 0, Math.PI * 2);
      ctx.fillStyle = panel;
      ctx.fill();
      ctx.strokeStyle = accent;
      ctx.lineWidth = 2;
      ctx.stroke();
    },
  };

  function clampRadius(m) {
    return Math.min(MAX_RADIUS_M, Math.max(MIN_RADIUS_M, Math.round(m)));
  }

  // --------------------------------------------------------------- draft

  function ready() {
    if (!draft) return false;
    return draft.kind === 'station' || (draft.placed && draft.radius >= MIN_RADIUS_M);
  }

  function renderSummary() {
    if (!draft) {
      summary.textContent = '';
      return;
    }
    const [lon, lat] = draft.kind === 'station' ? draft.at : draft.centre;
    summary.textContent =
      `${draft.kind === 'station' ? 'at' : 'centre'} ${lat.toFixed(5)}, ${lon.toFixed(5)}` +
      (draft.kind === 'roi' && draft.radius > 0 ? ` · r ${fmtLength(draft.radius)}` : '');
    if (draft.kind === 'roi' && !draft.placed) radiusInput.value = draft.radius > 0 ? String(draft.radius) : '';
    addButton.disabled = !ready();
  }

  function renderDraft() {
    draftForm.hidden = !draft;
    draftError.hidden = true;
    if (!draft) {
      map.redraw();
      return;
    }
    const roi = draft.kind === 'roi';
    draftTitle.textContent = roi ? 'New region of interest' : 'New launch and recovery station';
    addButton.textContent = roi ? 'Add region' : 'Add station';
    radiusField.hidden = !roi;
    if (roi && draft.placed) radiusInput.value = String(draft.radius);
    renderSummary();
    map.redraw();
  }

  radiusInput.addEventListener('input', () => {
    if (!draft || draft.kind !== 'roi') return;
    const r = Number(radiusInput.value);
    if (Number.isFinite(r) && r > 0) {
      draft.radius = Math.round(r);
      draft.placed = true;
      renderSummary();
      map.redraw();
    }
  });

  $('mark-cancel').addEventListener('click', () => setMode(mode));

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
      showError('Place the edge, or type a radius.');
      return;
    }
    const name = nameInput.value.trim();
    if (!name) {
      showError(`Give the ${draft.kind === 'roi' ? 'region' : 'station'} a name.`);
      nameInput.focus();
      return;
    }
    let path;
    let body;
    if (draft.kind === 'station') {
      path = '/api/v1/stations';
      body = { lat: draft.at[1], lon: draft.at[0], name };
    } else {
      const r = Number(radiusInput.value.trim() || draft.radius);
      if (!Number.isFinite(r) || r < MIN_RADIUS_M || r > MAX_RADIUS_M) {
        showError(`The radius is from ${MIN_RADIUS_M} to ${MAX_RADIUS_M.toLocaleString()} m.`);
        return;
      }
      path = '/api/v1/rois';
      body = { lat: draft.centre[1], lon: draft.centre[0], radius_m: r, name };
    }
    const what = draft.kind === 'roi' ? 'region' : 'station';
    addButton.disabled = true;
    try {
      const response = await fetch(path, {
        method: 'POST',
        headers: { 'content-type': 'application/json' },
        body: JSON.stringify(body),
      });
      const result = await response.json().catch(() => ({}));
      if (!response.ok) {
        showError(result.detail || `The ${what} was refused (${response.status}).`);
        addButton.disabled = false;
        return;
      }
      if (result.status === 'queued') {
        pending.set(result.id, { what: `Adding ${what} “${name}”`, at: Date.now() });
        say(`${capital(what)} “${name}” queued: it is added when the simulation next steps.`, 'warn');
      } else {
        say(`${capital(what)} “${name}” added.`, 'ok');
      }
      nameInput.value = '';
      radiusInput.value = '';
      setMode(mode);
      refresh();
      map.refreshOverlays();
    } catch (err) {
      showError(`The ${what} could not be sent: ${err}`);
      addButton.disabled = false;
    }
  }

  function capital(text) {
    return text.charAt(0).toUpperCase() + text.slice(1);
  }

  // -------------------------------------------------------------- tables

  function say(text, kind) {
    flash.textContent = text;
    flash.className = `banner ${kind}`;
    flash.hidden = false;
    clearTimeout(say.timer);
    say.timer = setTimeout(() => { flash.hidden = true; }, 6000);
  }

  const SOURCES = { scenario: 'Scenario', dashboard: 'Dashboard', intent: 'API', crash: 'Crash' };

  function latest() {
    return stations || rois || {};
  }

  function simTime(tick) {
    const seconds = latest().tick_seconds;
    if (tick == null) return '';
    if (!seconds) return `tick ${tick.toLocaleString()}`;
    const t = Math.floor(tick * seconds);
    const d = Math.floor(t / 86_400);
    const hms = [Math.floor((t % 86_400) / 3600), Math.floor((t % 3600) / 60), t % 60]
      .map((v) => String(v).padStart(2, '0'))
      .join(':');
    return d > 0 ? `${d} d ${hms}` : hms;
  }

  // How long ago `tick` was, as "5 min ago".
  function ago(tick) {
    const body = latest();
    if (!body.tick_seconds || body.tick == null) return '';
    const s = Math.max(0, (body.tick - tick) * body.tick_seconds);
    if (s < 60) return `${Math.round(s)} s ago`;
    if (s < 3600) return `${Math.round(s / 60)} min ago`;
    return `${(s / 3600).toFixed(1)} h ago`;
  }

  function cell(text, cls) {
    const td = document.createElement('td');
    td.textContent = text;
    if (cls) td.className = cls;
    return td;
  }

  function nameCell(item, fallback) {
    const td = document.createElement('td');
    const id = document.createElement('span');
    id.className = 'mono';
    id.textContent = `#${item.id}`;
    td.append(id, ` ${item.name || fallback}`);
    return td;
  }

  function sourceCell(item) {
    if (item.source !== 'crash') return cell(SOURCES[item.source] || item.source || '');
    const td = document.createElement('td');
    const tag = document.createElement('span');
    tag.className = 'tag bad';
    tag.textContent = 'crash site';
    tag.title = 'Added by itself where a drone crashed';
    td.append(tag);
    return td;
  }

  function select(mark) {
    selected = mark;
    map.selectMark(mark);
    const pick = (body, kind) => {
      for (const row of body.rows) {
        row.classList.toggle('current', Boolean(mark) && mark.kind === kind && Number(row.dataset.id) === mark.id);
      }
    };
    pick(stationBody, 'station');
    pick(roiBody, 'roi');
  }

  // A row that picks its station or region out on the map when clicked.
  function row(kind, item, radiusM) {
    const tr = document.createElement('tr');
    tr.dataset.id = item.id;
    tr.className = selected && selected.kind === kind && selected.id === item.id ? 'current' : '';
    tr.tabIndex = 0;
    tr.title = 'Show on the map';
    const show = () => {
      select({ kind, id: item.id });
      const [lon, lat] = degrees(item);
      if (Number.isFinite(lon) && Number.isFinite(lat)) map.showPlace(lon, lat, radiusM);
    };
    tr.addEventListener('click', show);
    tr.addEventListener('keydown', (event) => {
      if (event.key === 'Enter') show();
    });
    return tr;
  }

  function removeButton(kind, item) {
    const td = document.createElement('td');
    const button = document.createElement('button');
    button.type = 'button';
    button.className = 'danger';
    button.textContent = 'Remove';
    button.title = item.source === 'scenario'
      ? 'Remove this one of the scenario\'s (a reset adds it again)'
      : 'Remove';
    button.addEventListener('click', (event) => {
      event.stopPropagation();
      remove(kind, item, button);
    });
    td.append(button);
    return td;
  }

  async function remove(kind, item, button) {
    const what = kind === 'roi' ? 'Region' : 'Station';
    const label = item.name ? `${what} “${item.name}”` : `${what} ${item.id}`;
    button.disabled = true;
    try {
      const path = kind === 'roi' ? 'rois' : 'stations';
      const response = await fetch(`/api/v1/${path}/${item.id}`, { method: 'DELETE' });
      const result = await response.json().catch(() => ({}));
      if (!response.ok) {
        say(`${label} could not be removed: ${reason(result.detail || response.status)}`, 'bad');
        button.disabled = false;
        return;
      }
      if (result.status === 'queued') {
        pending.set(result.id, { what: `Removing ${label.charAt(0).toLowerCase()}${label.slice(1)}`, at: Date.now() });
        say(`${label} is removed when the simulation next steps.`, 'warn');
      } else {
        say(`${label} removed.`, 'ok');
      }
      refresh();
      map.refreshOverlays();
    } catch (err) {
      say(`${label} could not be removed: ${err}`, 'bad');
      button.disabled = false;
    }
  }

  function renderStations(body) {
    const list = body.stations || [];
    $('station-empty').hidden = list.length > 0;
    $('station-table').hidden = list.length === 0;
    // Rebuild only when something changed, so a button being pressed is not
    // swept away by the next poll.
    const key = JSON.stringify(list) + body.tick_seconds;
    if (renderStations.key === key) return;
    renderStations.key = key;
    stationBody.replaceChildren(
      ...list.map((s) => {
        const tr = row('station', s, 300);
        const [lon, lat] = degrees(s);
        const n = Number(s.drones);
        tr.append(
          nameCell(s, 'station'),
          cell(Number.isFinite(lat) ? `${lat.toFixed(5)}, ${lon.toFixed(5)}` : '', 'mono small'),
          cell(SOURCES[s.source] || s.source || ''),
          cell(simTime(s.added_tick), 'mono small'),
          cell(Number.isFinite(n) ? String(n) : ''),
          removeButton('station', s),
        );
        return tr;
      }),
    );
  }

  // The drones on their way to survey a region, by callsign where known.
  function tasked(r) {
    const ids = Array.isArray(r.assigned) ? r.assigned : [];
    if (ids.length === 0) return '';
    return ids.map((id) => callsigns.get(id) || `#${id}`).join(', ');
  }

  function renderRois(body) {
    const list = body.rois || [];
    $('roi-empty').hidden = list.length > 0;
    $('roi-table').hidden = list.length === 0;
    const key = JSON.stringify(list) + body.tick_seconds + Math.floor(body.tick / 100) + callsigns.size;
    if (renderRois.key === key) return;
    renderRois.key = key;
    roiBody.replaceChildren(
      ...list.map((r) => {
        const tr = row('roi', r, Number(r.radius_m) || 300);
        if (r.source === 'crash') tr.classList.add('crash');
        const last = r.last_surveyed_tick;
        tr.append(
          nameCell(r, 'region'),
          cell(Number(r.radius_m) > 0 ? fmtLength(Number(r.radius_m)) : ''),
          sourceCell(r),
          cell(simTime(r.added_tick), 'mono small'),
          cell(String(r.surveys || 0)),
          last == null ? cell('never', 'dim') : cell(`${simTime(last)} · ${ago(last)}`, 'small'),
          cell(tasked(r), tasked(r) ? 'tasked' : ''),
          removeButton('roi', r),
        );
        return tr;
      }),
    );
  }

  function describe(r) {
    const d = r.detail || {};
    const named = (fallback) => (d.name ? `“${d.name}”` : fallback);
    switch (r.kind) {
      case 'station':
        return [
          d.action === 'remove' ? 'Station removed' : 'Station added',
          `${named(`station ${d.id ?? ''}`)}` + (d.source ? ` · ${SOURCES[d.source] || d.source}` : ''),
          '',
        ];
      case 'roi':
        if (d.action === 'remove') return ['Region removed', named(`region ${d.id ?? ''}`), ''];
        if (d.source === 'crash') return ['Crash site', `${named('A region')} added where a drone crashed`, 'bad'];
        return [
          'Region designated',
          `${named(`region ${d.id ?? ''}`)}` + (d.source ? ` · ${SOURCES[d.source] || d.source}` : ''),
          '',
        ];
      case 'survey': {
        const who = d.callsign || (d.sysid ? `drone ${d.sysid}` : `drone ${r.observer}`);
        const took = Number(d.duration_s) > 0 ? ` in ${fmtDuration(Number(d.duration_s))}` : '';
        return ['Survey done', `${who} surveyed ${named(`region ${d.roi ?? ''}`)}${took}`, 'ok'];
      }
      default:
        return null;
    }
  }

  function fmtDuration(s) {
    if (s < 60) return `${Math.round(s)} s`;
    if (s < 3600) return `${Math.floor(s / 60)} min ${Math.round(s % 60)} s`;
    return `${(s / 3600).toFixed(1)} h`;
  }

  function renderEvents() {
    const reports = [...((stations && stations.recent) || []), ...((rois && rois.recent) || [])]
      .sort((a, b) => b.seq - a.seq)
      .slice(0, 100);
    const rows = reports.map((r) => [r, describe(r)]).filter(([, d]) => d);
    $('mark-events-empty').hidden = rows.length > 0;
    $('mark-events').hidden = rows.length === 0;
    const key = rows.length ? `${rows[0][0].seq}/${rows.length}` : '';
    if (renderEvents.key === key) return;
    renderEvents.key = key;
    eventsBody.replaceChildren(
      ...rows.map(([r, [what, detail, cls]]) => {
        const tr = document.createElement('tr');
        if (cls === 'bad') tr.className = 'rejected';
        tr.append(cell(simTime(r.tick), 'mono small'), cell(what), cell(detail, 'small'));
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

  // Callsigns for the drones tasked to survey, fetched only while there
  // are any.
  async function refreshCallsigns() {
    const any = ((rois && rois.rois) || []).some((r) => Array.isArray(r.assigned) && r.assigned.length);
    if (!any) return;
    try {
      const response = await fetch('/api/v1/state/entities?kind=drone');
      if (!response.ok) return;
      const drones = await response.json();
      callsigns = new Map(drones.filter((d) => d.callsign).map((d) => [d.id, d.callsign]));
    } catch {
      // The table shows agent ids instead.
    }
  }

  let inFlight = false;
  async function refresh() {
    if (inFlight) return;
    inFlight = true;
    try {
      const [s, r] = await Promise.all([
        fetch('/api/v1/stations').then((x) => (x.ok ? x.json() : null)),
        fetch('/api/v1/rois').then((x) => (x.ok ? x.json() : null)),
      ]);
      if (s) {
        stations = s;
        checkPending(s);
        renderStations(s);
      }
      if (r) {
        rois = r;
        checkPending(r);
        await refreshCallsigns();
        renderRois(r);
      }
      // A model without either publishes neither list.
      $('marks-unavailable').hidden = Boolean((s && s.available) || (r && r.available));
      renderEvents();
    } catch {
      // The next poll tries again.
    } finally {
      inFlight = false;
    }
  }

  for (const b of toolButtons) b.addEventListener('click', () => setMode(b.dataset.tool));
  setMode('select');
  refresh();
  setInterval(refresh, 1000);
})();
