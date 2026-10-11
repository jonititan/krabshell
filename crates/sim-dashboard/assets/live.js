// Live cell contents over the WebSocket.
//
// This is the same protocol a Bevy client would speak (spec §9.3), used here
// for one small thing: keep the inspected cell's bag current without polling.
// A cell coarser than occupancy, picked from the map, follows the tick
// heartbeat instead of AOI deltas.
// If the socket never connects, the htmx "Look up" button still works — the
// console is not allowed to depend on this file.

(function () {
  'use strict';

  const cellInput = document.getElementById('cell-input');
  const status = document.getElementById('map-status');
  if (!cellInput) return;

  let socket = null;
  let watching = null;
  let backoff = 1000;
  let refreshQueued = false;
  let lastTick = null;
  let lastCoarseRefresh = 0;

  function isCoarse() {
    const cell = document.getElementById('cell');
    return !!cell && cell.hasAttribute('data-coarse');
  }

  function connect() {
    const scheme = location.protocol === 'https:' ? 'wss' : 'ws';
    socket = new WebSocket(`${scheme}://${location.host}/api/v1/ws`);

    socket.addEventListener('open', () => {
      backoff = 1000;
      if (watching) subscribe(watching);
    });

    socket.addEventListener('message', (event) => {
      let msg;
      try {
        msg = JSON.parse(event.data);
      } catch {
        return;
      }
      // Snapshot and delta both mean the watched bag may have changed. Rather
      // than reimplementing the bag view in JavaScript, ask htmx to re-render
      // the fragment the server already knows how to build.
      if (msg.type === 'snapshot' || msg.type === 'delta') queueRefresh();
      // The area of interest is kept at occupancy resolution, so a coarser
      // cell picked from the map never gets deltas of its own. Re-render it on
      // the heartbeat instead, at most once a second.
      if (msg.type === 'tick' && isCoarse() && msg.tick !== lastTick) {
        const now = performance.now();
        if (now - lastCoarseRefresh >= 1000) {
          lastCoarseRefresh = now;
          lastTick = msg.tick;
          queueRefresh();
        }
      }
    });

    socket.addEventListener('close', () => {
      socket = null;
      // Back off to a minute so a stopped server does not turn into a
      // reconnect storm in an abandoned tab.
      setTimeout(connect, backoff);
      backoff = Math.min(backoff * 2, 60000);
    });

    socket.addEventListener('error', () => {
      if (status && !status.textContent.includes('live')) {
        status.textContent += ' · live updates unavailable';
      }
    });
  }

  function subscribe(cell) {
    if (!socket || socket.readyState !== WebSocket.OPEN) return;
    socket.send(JSON.stringify({ type: 'set_aoi', center: cell, k: 1 }));
  }

  // Collapse a burst of deltas into one fragment fetch per frame.
  function queueRefresh() {
    if (refreshQueued || !watching || !window.htmx) return;
    refreshQueued = true;
    requestAnimationFrame(() => {
      refreshQueued = false;
      const at = window.krabshellRewind ? window.krabshellRewind.at() : null;
      window.htmx.ajax('GET', `/fragments/cell?cell=${watching}${at === null ? '' : `&at=${at}`}`, {
        target: '#cell',
        swap: 'outerHTML',
      });
    });
  }

  function watch(cell) {
    if (!cell || cell === watching) return;
    watching = cell;
    subscribe(cell);
  }

  document.addEventListener('krabshell:cell', (event) => watch(event.detail));
  // The rewind cursor moved: show the watched cell at the new tick.
  document.addEventListener('krabshell:at', queueRefresh);
  cellInput.addEventListener('change', () => watch(cellInput.value.trim()));

  if (cellInput.value.trim()) watching = cellInput.value.trim();
  connect();
})();
