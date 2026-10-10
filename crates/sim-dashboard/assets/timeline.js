// Rewind: step back through what just happened, replay it, and carry on.
//
// The server keeps keyframes of the recent past and rebuilds any tick in it
// on request (GET /api/v1/history, and ?at=<tick> on the state endpoints).
// This file holds the cursor: while reviewing, `window.krabshellRewind.at()`
// is the tick to show and the maps add it to what they fetch; a
// `krabshell:at` event says it moved. The cursor is this tab's own, kept in
// sessionStorage so it survives moving between pages.
//
// Rewind pauses the run, so the past stops growing while you look. Replay
// plays the cursor forward; when it reaches the live tick, review ends and
// the run starts again at the speed it had, so it picks up where it left off.

(function () {
  'use strict';

  const root = document.getElementById('timeline');
  if (!root) return;
  const status = root.querySelector('.timeline-status');
  const scrub = root.querySelector('.timeline-scrub');
  const rateSelect = root.querySelector('[data-act="rate"]');
  const playButton = root.querySelector('[data-act="play"]');
  const KEY = 'krabshell.rewind';
  const TICK_MS = 100; // how often a replay moves the cursor

  const state = {
    available: false,
    first: 0,
    live: 0,
    tickSeconds: 0.1,
    review: false,
    cursor: 0,
    playing: false,
    rate: 1,
    carry: 0, // fractional ticks a replay owes
  };

  function load() {
    try {
      const saved = JSON.parse(sessionStorage.getItem(KEY) || 'null');
      if (saved && saved.review) {
        state.review = true;
        state.cursor = saved.cursor | 0;
        state.rate = saved.rate || 1;
      }
    } catch {
      // Storage may be off; the cursor then lasts as long as the page.
    }
  }

  function save() {
    try {
      sessionStorage.setItem(KEY, JSON.stringify({
        review: state.review, cursor: state.cursor, rate: state.rate,
      }));
    } catch {
      // As above.
    }
  }

  window.krabshellRewind = {
    at: () => (state.review ? state.cursor : null),
  };
  // The conflict tables and the cell inspector are htmx fragments: while
  // reviewing, their requests carry the cursor too.
  document.addEventListener('htmx:configRequest', (event) => {
    if (state.review && /^\/fragments\/(conflicts|cell)\b/.test(event.detail.path)) {
      event.detail.parameters.at = state.cursor;
    }
  });

  function announce() {
    save();
    document.dispatchEvent(new CustomEvent('krabshell:at', { detail: window.krabshellRewind.at() }));
  }

  function clock(tick) {
    const s = Math.floor(tick * state.tickSeconds);
    const pad = (n) => String(n).padStart(2, '0');
    return `${pad(Math.floor(s / 3600))}:${pad(Math.floor(s / 60) % 60)}:${pad(s % 60)}`;
  }

  function render() {
    root.hidden = false;
    root.classList.toggle('reviewing', state.review);
    root.querySelector('.timeline-review').hidden = !state.review || !state.available;
    root.querySelector('[data-act="rewind"]').hidden = state.review || !state.available;
    scrub.hidden = !state.review || !state.available;
    if (!state.available) {
      status.textContent = state.reason ? `Rewind unavailable: ${state.reason}` : '';
      return;
    }
    scrub.min = String(state.first);
    scrub.max = String(state.live);
    scrub.value = String(state.cursor);
    rateSelect.value = String(state.rate);
    playButton.textContent = state.playing ? '❚❚ Pause' : '▶ Replay';
    const span = Math.max(0, state.live - state.first);
    if (state.review) {
      const behind = (state.live - state.cursor) * state.tickSeconds;
      status.textContent = `REVIEW · tick ${state.cursor.toLocaleString()} · ${clock(state.cursor)}`
        + ` · ${behind.toFixed(1)} s behind now`;
    } else {
      status.textContent = `${(span * state.tickSeconds / 60).toFixed(1)} min to rewind`;
    }
  }

  async function refreshRange() {
    try {
      const response = await fetch('/api/v1/history', { headers: { accept: 'application/json' } });
      if (!response.ok) return;
      const h = await response.json();
      state.available = h.available && h.first_tick !== null;
      state.reason = h.reason;
      state.first = h.first_tick ?? 0;
      state.live = h.live_tick;
      if (h.tick_seconds) state.tickSeconds = h.tick_seconds;
      if (state.review) {
        // The window slides on while the run goes on elsewhere.
        const clamped = Math.min(Math.max(state.cursor, state.first), state.live);
        if (clamped !== state.cursor) {
          state.cursor = clamped;
          announce();
        }
      }
      render();
    } catch {
      // Try again on the next poll.
    }
  }

  async function action(path) {
    // The control panel's own actions, as its buttons send them.
    // An empty form: Start with no rate keeps the speed already chosen.
    await fetch(path, {
      method: 'POST',
      headers: { 'HX-Request': 'true', 'Content-Type': 'application/x-www-form-urlencoded' },
      body: '',
    });
  }

  function moveTo(tick) {
    const next = Math.min(Math.max(Math.round(tick), state.first), state.live);
    if (next === state.cursor && state.review) return;
    state.cursor = next;
    render();
    announce();
  }

  async function rewind() {
    await action('/actions/pause');
    await refreshRange();
    state.review = true;
    state.cursor = state.live;
    render();
    announce();
  }

  function stopPlaying() {
    state.playing = false;
    state.carry = 0;
  }

  async function backToNow(start) {
    stopPlaying();
    state.review = false;
    render();
    announce();
    if (start) await action('/actions/start');
  }

  // A replay moves the cursor at `rate` times real time, then hands over to
  // the live run.
  setInterval(async () => {
    if (!state.playing) return;
    state.carry += (state.rate * TICK_MS) / 1000 / state.tickSeconds;
    const whole = Math.floor(state.carry);
    if (whole < 1) return;
    state.carry -= whole;
    if (state.cursor + whole >= state.live) {
      await backToNow(true);
      return;
    }
    moveTo(state.cursor + whole);
  }, TICK_MS);

  const tenSeconds = () => Math.max(1, Math.round(10 / state.tickSeconds));

  root.addEventListener('click', (event) => {
    const act = event.target.closest('[data-act]')?.dataset.act;
    switch (act) {
      case 'rewind': rewind(); break;
      case 'back': stopPlaying(); moveTo(state.cursor - 1); break;
      case 'fwd': stopPlaying(); moveTo(state.cursor + 1); break;
      case 'back-big': stopPlaying(); moveTo(state.cursor - tenSeconds()); break;
      case 'fwd-big': stopPlaying(); moveTo(state.cursor + tenSeconds()); break;
      case 'play': state.playing = !state.playing; state.carry = 0; render(); break;
      case 'live': backToNow(false); break;
      default: break;
    }
  });
  rateSelect.addEventListener('change', () => {
    state.rate = Number(rateSelect.value) || 1;
    save();
  });
  scrub.addEventListener('input', () => {
    stopPlaying();
    moveTo(Number(scrub.value));
  });
  // Starting the run from the control panel ends a review: the live world is
  // what is moving now.
  document.addEventListener('htmx:afterRequest', (event) => {
    const path = event.detail?.requestConfig?.path || '';
    if (state.review && (path === '/actions/start' || path === '/actions/step')) backToNow(false);
  });

  load();
  render();
  refreshRange().then(() => {
    if (state.review) announce();
  });
  setInterval(refreshRange, 2000);
})();
