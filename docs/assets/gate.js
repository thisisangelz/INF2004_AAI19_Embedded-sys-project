// Password gate. The run data in data.js is encrypted (AES-256-GCM, key from
// PBKDF2-SHA256 of the password), so the page holds neither the password nor
// readable data. The right password is the only thing that decrypts it.
// The derived key, not the password, is kept for this browser tab so a
// reload does not ask again.
(function () {
  'use strict';
  const SESSION_KEY = 'runlog.key.v1';
  const $ = (s) => document.querySelector(s);
  const bytes = (b64) => Uint8Array.from(atob(b64), (c) => c.charCodeAt(0));
  const b64 = (buf) => btoa(String.fromCharCode.apply(null, new Uint8Array(buf)));

  async function deriveKey(password, box) {
    const base = await crypto.subtle.importKey('raw', new TextEncoder().encode(password), 'PBKDF2', false, ['deriveKey']);
    return crypto.subtle.deriveKey({ name: 'PBKDF2', hash: 'SHA-256', salt: bytes(box.salt), iterations: box.iterations },
      base, { name: 'AES-GCM', length: 256 }, true, ['decrypt']);
  }

  async function open(key, box) {
    const zipped = await crypto.subtle.decrypt({ name: 'AES-GCM', iv: bytes(box.iv) }, key, bytes(box.data));
    const stream = new Blob([zipped]).stream().pipeThrough(new DecompressionStream('gzip'));
    return JSON.parse(await new Response(stream).text());
  }

  function unlock(payload) {
    window.RUNS = payload.runs;
    window.LAYOUTS = payload.layouts;
    $('#gate').remove();
    document.body.classList.remove('locked');
    window.startDashboard();
  }

  async function tryStoredKey(box) {
    let raw = null;
    try { raw = sessionStorage.getItem(SESSION_KEY); } catch (e) { return false; }
    if (!raw) return false;
    try {
      const key = await crypto.subtle.importKey('raw', bytes(raw), 'AES-GCM', false, ['decrypt']);
      unlock(await open(key, box));
      return true;
    } catch (e) {
      try { sessionStorage.removeItem(SESSION_KEY); } catch (e2) { /* ignore */ }
      return false;
    }
  }

  async function start() {
    const box = window.DASHBOARD_DATA;
    const msg = $('#gate-msg');
    const form = $('#gate-form');
    const input = $('#gate-pass');
    if (!box) { msg.textContent = 'The dashboard data is missing. Rebuild it with docs/tools/build_runs.js.'; return; }
    if (!window.crypto || !crypto.subtle || typeof DecompressionStream === 'undefined') {
      msg.textContent = 'This browser cannot open the dashboard. Use a current Chrome, Edge, Firefox or Safari over https.';
      form.querySelector('button').disabled = true;
      return;
    }
    if (await tryStoredKey(box)) return;
    form.hidden = false;
    input.focus();
    form.addEventListener('submit', async (e) => {
      e.preventDefault();
      const button = form.querySelector('button');
      button.disabled = true;
      msg.textContent = 'Checking…';
      try {
        const key = await deriveKey(input.value, box);
        const payload = await open(key, box);
        try { sessionStorage.setItem(SESSION_KEY, b64(await crypto.subtle.exportKey('raw', key))); } catch (err) { /* private mode */ }
        unlock(payload);
      } catch (err) {
        msg.textContent = 'Wrong password.';
        input.select();
        button.disabled = false;
      }
    });
  }
  start();
}());
