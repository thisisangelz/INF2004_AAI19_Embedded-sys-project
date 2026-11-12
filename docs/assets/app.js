(function () {
  'use strict';

  const $ = (sel, el) => (el || document).querySelector(sel);
  const esc = (s) => String(s).replace(/[&<>"]/g, (c) => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;' }[c]));
  const NS = 'http://www.w3.org/2000/svg';

  let runs = [];
  let current = 0;
  let view = 'run';
  const eventFilter = { decisions: true, colour: true, problems: true, transfer: true, moves: false };

  // ---------- formatting ----------
  const secs = (ms, d) => (ms / 1000).toFixed(d === undefined ? 1 : d) + ' s';
  const pct = (x, d) => (x * 100).toFixed(d === undefined ? 0 : d) + '%';
  function duration(ms) {
    const s = Math.round(ms / 1000);
    return s >= 60 ? Math.floor(s / 60) + ' min ' + String(s % 60).padStart(2, '0') + ' s' : s + ' s';
  }

  // ---------- chart helpers ----------
  function niceTicks(min, max, count) {
    const span = max - min || 1;
    const raw = span / count;
    const mag = Math.pow(10, Math.floor(Math.log10(raw)));
    const step = [1, 2, 5, 10].map((m) => m * mag).find((s) => s >= raw);
    const out = [];
    for (let v = Math.ceil(min / step) * step; v <= max + 1e-9; v += step) out.push(+v.toFixed(6));
    return out;
  }

  const PHASE_NAME = {
    start: 'Start', follow: 'Wi-Fi line', dip: 'Dip recovery', return: 'Return to best',
    search: 'Direction search', mat: 'Colour steering', scan: 'Colour scan', transfer: 'Transfer',
  };

  // ---------- plain-language summary ----------
  function insights(run) {
    const s = run.stats;
    const out = [];
    if (run.timed) {
      if (run.tailMs > 1000) out.push('The last <b>' + secs(run.tailMs, 0) + '</b> has no timestamps in the log. Its length is estimated from the drives, turns and colour reads recorded there.');
      out.push('The robot was moving for <b>' + secs(s.driveMs) + '</b> of a <b>' + duration(s.wallMs) +
        '</b> run (<b>' + pct(s.driveShare, 1) + '</b>). The rest of the time it was stopped.');
      if (s.medianStepWallMs) {
        out.push('A typical stop took <b>' + secs(s.medianStepWallMs) + '</b> to move about <b>' +
          Math.round(s.medianStepMovedMs) + ' ms</b> forward.');
      }
      out.push('Wi-Fi scanning used <b>' + pct(s.scanMs / s.wallMs) + '</b> of the run (' + s.wifiScans +
        ' scans). A stationary stop takes two batches of scans before the robot may move on.');
      if (s.cruiseSegments) {
        const ends = Object.keys(s.cruiseEnds).map((k) => s.cruiseEnds[k] + ' ended on ' + k).join(', ');
        out.push('<b>Cruise:</b> ' + s.movingScans + ' of ' + s.wifiScans + ' Wi-Fi scans (' + pct(s.movingScans / s.wifiScans) +
          ') were taken while driving, in ' + s.cruiseSegments + ' segment' + (s.cruiseSegments > 1 ? 's' : '') + ' (' + ends +
          '). Largest control-loop gap: <b>' + s.cruiseMaxGapMs + ' ms</b>.');
      }
      if (s.bleChecks) out.push('The robot made <b>' + s.bleChecks + '</b> stationary BLE proximity checks (' + s.bleReadings + ' filtered readings).');
      if (s.dipChecks) out.push('<b>' + s.dipChecks + '</b> dip checks: the signal fell and the robot stopped to re-measure before moving.');
      if (s.imuFaults) out.push('<b>' + s.imuFaults + '</b> IMU read error' + (s.imuFaults > 1 ? 's' : '') + ' during a drive. The robot stopped, recovered the sensor and ended that block early.');
      if (s.directionSearches || s.returns) out.push('Direction searches: <b>' + s.directionSearches + '</b>, returns to the best point: <b>' + s.returns + '</b>.');
    }
    if (s.matScans) {
      const vecs = run.scans.filter((x) => x.result).map((x) => x.result.vector_angle + '°');
      out.push('<b>' + s.matScans + '</b> colour scan' + (s.matScans > 1 ? 's' : '') + (vecs.length ? ', pointing ' + vecs.join(', ') + ' from where each started.' : '.'));
    }
    if (run.failure) {
      out.push('Failure' + (run.failure.t !== null && run.timed ? ' at about <b>' + ((run.failure.t - run.t0) / 1000).toFixed(0) + ' s</b>' : '') + ': <b>' + esc(run.failure.text) + '</b>');
    }
    if (run.result) {
      out.push('Ended with <b>' + esc(run.result) + '</b>' + (run.faults.length ? ' · faults ' + run.faults.map(esc).join(', ') : '') + '.');
    }
    return out;
  }

  // ---------- links to the saved files ----------
  // Notes open on GitHub, which renders Markdown; raw logs sit next to this page in runs/.
  const NOTE_BASE = 'https://github.com/thisisangelz/INF2004_AAI19_Embedded-sys-project/blob/main/docs/runs/';
  function fileLinks(r) {
    if (!r.embedded) return '<span class="muted">opened from this computer</span>';
    return (r.note ? '<a href="' + NOTE_BASE + encodeURIComponent(r.note) + '" target="_blank" rel="noopener">Notes</a>' : '') +
      '<a href="runs/' + encodeURIComponent(r.name) + '" target="_blank" rel="noopener">Raw log</a>';
  }
  const resultLabel = (r) => (r.resultKind === 'ok' ? 'completed' : r.resultKind === 'fail' ? 'no payload' : 'no result');

  // ---------- all runs ----------
  function renderRuns() {
    const byDate = {};
    runs.forEach((r, i) => { (byDate[r.date || 'Undated'] = byDate[r.date || 'Undated'] || []).push(i); });
    const dates = Object.keys(byDate).sort().reverse();
    const fmtDate = (d) => (/^\d{4}-\d{2}-\d{2}$/.test(d) ? new Date(d + 'T00:00:00').toLocaleDateString(undefined, { weekday: 'short', day: 'numeric', month: 'long', year: 'numeric' }) : d);
    $('#main').innerHTML = '<section class="head"><h2>All runs</h2><p class="sub">' + runs.length + ' saved logs, newest first. Open a run to see its timeline, path and colour scans. Notes are the written analysis of each run.</p></section>' +
      dates.map((d) => '<section><h3 class="day">' + esc(fmtDate(d)) + '</h3><div class="runs">' + byDate[d].map((i) => {
        const r = runs[i];
        const share = r.stats.driveShare;
        return '<article class="runcard"><button class="run" data-i="' + i + '">' +
          '<span class="t">' + esc(r.title) + '</span>' +
          '<span class="m"><span class="chip ' + r.resultKind + '">' + resultLabel(r) + '</span>' +
          (r.timed ? '<span>' + duration(r.stats.wallMs) + '</span><span>' + pct(share, 1) + ' moving</span>' : '<span>no timing</span>') + '</span>' +
          (share !== null ? '<span class="minibar" title="Time spent moving"><i style="width:' + Math.max(1, share * 100 * 4) + '%"></i></span>' : '') +
          '</button><div class="links">' + fileLinks(r) + '</div></article>';
      }).join('') + '</div></section>').join('');
    $('#main').querySelectorAll('.run').forEach((b) => b.addEventListener('click', () => { current = +b.dataset.i; setView('run'); }));
  }

  // ---------- run view ----------
  function tiles(run) {
    const s = run.stats;
    const t = [];
    if (!run.timed) {
      t.push(['Colour scans', String(s.matScans), 'full turns on the spot']);
      t.push(['Result', run.resultKind === 'ok' ? 'Completed' : run.resultKind === 'fail' ? 'No payload' : 'n/a', run.faults.length ? run.faults.join(', ') : 'no faults']);
      return '<div class="tiles">' + t.map((x) => '<div class="tile"><span class="k">' + x[0] + '</span><span class="v">' + esc(x[1]) + '</span><span class="s">' + esc(x[2]) + '</span></div>').join('') + '</div>';
    }
    t.push(['Run time', run.timed ? duration(s.wallMs) : 'n/a', run.timed ? 'first Wi-Fi scan to last stop' : 'this log has no timestamps']);
    t.push(['Moving', run.timed ? secs(s.driveMs) : 'n/a', s.blocks + ' drive blocks']);
    t.push(['Moving share', s.driveShare === null ? 'n/a' : pct(s.driveShare, 1), 'of the run time', s.driveShare !== null && s.driveShare < 0.1]);
    t.push(['Stops', String(s.stops), s.medianStepWallMs ? 'median ' + secs(s.medianStepWallMs) + ' each' : '']);
    t.push(['Result', run.resultKind === 'ok' ? 'Completed' : run.resultKind === 'fail' ? 'No payload' : 'n/a', run.faults.length ? run.faults.join(', ') : 'no faults']);
    return '<div class="tiles">' + t.map((x) => '<div class="tile' + (x[3] ? ' hot' : '') + '"><span class="k">' + x[0] + '</span><span class="v">' + esc(x[1]) + '</span><span class="s">' + esc(x[2]) + '</span></div>').join('') + '</div>';
  }

  function timeUse(run) {
    if (!run.timed) return '';
    const s = run.stats;
    const rows = [
      ['Wi-Fi scanning', s.scanMs, 'var(--c-scan)', 'stationary batches, two per stop'],
      ['Driving', s.driveMs, 'var(--c-drive)', 'time the motors ran'],
      ['Turning', s.turnMs, 'var(--c-turn)', 'gyro-controlled turns'],
      ['Colour sensor', s.colourMs, 'var(--c-colour)', 'about 345 ms per arrival check'],
      ['Everything else', s.otherMs, 'var(--c-other)', 'BLE checks, settling, flash log writes, radio waits'],
    ];
    const total = Math.max(s.wallMs, rows.reduce((a, r) => a + r[1], 0));
    return '<section class="card"><h2>Where the time went</h2>' +
      '<p class="sub">Share of the run time, from the first Wi-Fi scan to the last stop.</p>' +
      '<div class="bar-use" role="img" aria-label="Time use">' + rows.map((r) => '<i style="width:' + (r[1] / total * 100) + '%;background:' + r[2] + '" title="' + r[0] + '"></i>').join('') + '</div>' +
      '<div class="legend">' + rows.map((r) => '<div><span class="sw" style="background:' + r[2] + '"></span><span>' + r[0] + '</span><span class="n">' + secs(r[1]) + '</span><span class="p">' + pct(r[1] / total) + '</span><small>' + r[3] + '</small></div>').join('') + '</div>' +
      '<p class="foot">Before the first scan: ' + secs(s.startupMs, 0) + ' (power-up, log erase, gyro calibration, Bluetooth start). Not included above. Scans taken while driving overlap the driving time and are not counted under Wi-Fi scanning. Drive time is what the motors ran when the log reports it (DRIVE OK), otherwise the commanded time. Turn time is the logged turn time. Colour time is an estimate.</p></section>';
  }

  // ---------- timeline ----------
  function buildTimeline(run) {
    const W = 980; const L = 62; const R = 18;
    const T = (run.t1 - run.t0) / 1000;
    const x = (sec) => L + (sec / T) * (W - L - R);
    const rel = (ms) => (ms - run.t0) / 1000;
    const ev = run.events.filter((e) => e.t !== null);
    const parts = [];

    // phase strip
    const strip = { y: 6, h: 22 };
    run.phases.forEach((p) => {
      const x0 = x(rel(p.t0)); const x1 = x(rel(p.t1));
      parts.push('<rect class="ph" x="' + x0 + '" y="' + strip.y + '" width="' + Math.max(1, x1 - x0) + '" height="' + strip.h + '" style="fill:var(--ph-' + p.phase + ')"><title>' + PHASE_NAME[p.phase] + ' ' + secs(p.t1 - p.t0) + '</title></rect>');
      if (x1 - x0 > 74) parts.push('<text class="ph-t" x="' + (x0 + 6) + '" y="' + (strip.y + 15) + '">' + PHASE_NAME[p.phase] + '</text>');
    });

    function panel(y, h, title, ticks, ymap, fmt) {
      parts.push('<text class="ttl" x="' + L + '" y="' + (y - 8) + '">' + title + '</text>');
      ticks.forEach((tk) => {
        const yy = ymap(tk.v);
        parts.push('<line class="grid" x1="' + L + '" x2="' + (W - R) + '" y1="' + yy + '" y2="' + yy + '"/>');
        parts.push('<text x="' + (L - 8) + '" y="' + (yy + 4) + '" text-anchor="end">' + (fmt ? fmt(tk) : tk.v) + '</text>');
      });
      parts.push('<line class="axis" x1="' + L + '" x2="' + L + '" y1="' + y + '" y2="' + (y + h) + '"/>');
    }

    // panel 1: Wi-Fi and BLE signal
    const p1 = { y: 62, h: 190 };
    const raw = run.events.filter((e) => e.type === 'rssi_raw');
    const ble = ev.filter((e) => e.type === 'ble');
    const pts = ev.filter((e) => e.type === 'line_point');
    // BLE readings can be far weaker than any Wi-Fi scan; keep them out of the axis range.
    const vals = raw.map((e) => e.d.dbm).concat(pts.map((e) => Math.min(e.d.dbm1, e.d.dbm2)));
    const lo = Math.floor((Math.min.apply(null, vals) - 3) / 5) * 5;
    const hi = Math.ceil((Math.max.apply(null, vals) + 3) / 5) * 5;
    const y1 = (v) => p1.y + p1.h - ((v - lo) / (hi - lo)) * p1.h;
    panel(p1.y, p1.h, 'Signal strength (dBm): Wi-Fi scans (green = taken while driving), stops, BLE checks', niceTicks(lo, hi, 5).map((v) => ({ v })), y1);
    [[-58, 'BLE gate −58'], [-85, 'with colour −85']].forEach((g) => {
      if (g[0] > lo && g[0] < hi) parts.push('<line class="gate" x1="' + L + '" x2="' + (W - R) + '" y1="' + y1(g[0]) + '" y2="' + y1(g[0]) + '"/><text class="gate-t" x="' + (W - R - 4) + '" y="' + (y1(g[0]) - 4) + '" text-anchor="end">' + g[1] + '</text>');
    });
    raw.forEach((e) => parts.push('<circle class="raw' + (e.d.stopped === 0 ? ' mv' : '') + '" cx="' + x(rel(e.t)) + '" cy="' + y1(e.d.dbm) + '" r="' + (e.d.stopped === 0 ? 2.8 : 1.8) + '"/>'));
    if (pts.length) {
      parts.push('<polyline class="line" points="' + pts.map((e) => x(rel(e.t)) + ',' + y1((e.d.dbm1 + e.d.dbm2) / 2)).join(' ') + '"/>');
      pts.forEach((e) => parts.push('<circle class="pt" cx="' + x(rel(e.t)) + '" cy="' + y1((e.d.dbm1 + e.d.dbm2) / 2) + '" r="3.4"><title>Stop ' + e.d.point + ': ' + e.d.dbm1 + ' / ' + e.d.dbm2 + ' dBm</title></circle>'));
    }
    ble.forEach((e) => { if (e.d.dbm >= lo && e.d.dbm <= hi) parts.push('<circle class="ble" cx="' + x(rel(e.t)) + '" cy="' + y1(e.d.dbm) + '" r="2.2"/>'); });

    // panel 2: colour under the sensor
    const hasColour = ev.some((e) => e.type === 'colour' || e.type === 'colour_scan' || e.type === 'mat_step');
    const p2 = { y: 290, h: hasColour ? 112 : 0 };
    const ymap2 = (d) => p2.y + p2.h - (d / 7) * p2.h;
    const labels = LogParser.DEPTH_LABEL;
    if (hasColour) panel(p2.y, p2.h, 'Colour under the sensor (floor to ring 6)', labels.map((l, d) => ({ v: d, l })), ymap2, (tk) => labels[tk.v]);
    const col = ev.filter((e) => e.type === 'colour');
    const scn = ev.filter((e) => e.type === 'colour_scan' || e.type === 'mat_step');
    const trace = col.map((e) => { const d = e.d.depths.filter((v) => v !== null); return d.length ? [x(rel(e.t)), ymap2(d[d.length - 1])] : null; }).filter(Boolean);
    if (trace.length > 1) parts.push('<polyline class="clink" points="' + trace.map((p) => p.join(',')).join(' ') + '"/>');
    col.forEach((e) => {
      e.d.depths.forEach((d) => { if (d !== null) parts.push('<circle class="cdot" cx="' + x(rel(e.t)) + '" cy="' + ymap2(d) + '" r="2.6"/>'); });
      if (e.d.arrived) parts.push('<circle class="arr" cx="' + x(rel(e.t)) + '" cy="' + ymap2(Math.max.apply(null, e.d.depths.filter((v) => v !== null))) + '" r="7"><title>Arrival confirmed</title></circle>');
    });
    scn.forEach((e) => {
      const d = e.type === 'colour_scan' ? e.d.depth : e.d.depth;
      if (typeof d === 'number' && d >= 0) parts.push('<rect class="cscan" x="' + (x(rel(e.t)) - 2.5) + '" y="' + (ymap2(d) - 2.5) + '" width="5" height="5"/>');
    });

    // panel 3: distance driven
    const p3 = { y: hasColour ? 440 : 290, h: 112 };
    const drives = ev.filter((e) => e.type === 'drive');
    let cum = 0;
    const steps = [[0, 0]];
    drives.forEach((e) => { steps.push([rel(e.t), cum]); cum += e.d.ms / 1000; steps.push([rel(e.t), cum]); });
    steps.push([T, cum]);
    const maxCum = Math.max(cum * 1.15, 1);
    const y3 = (v) => p3.y + p3.h - (v / maxCum) * p3.h;
    panel(p3.y, p3.h, 'Driving time so far (seconds). Flat stretches are the robot standing still', niceTicks(0, maxCum, 4).map((v) => ({ v })), y3);
    const path3 = steps.map((p) => x(p[0]) + ',' + y3(p[1])).join(' ');
    parts.push('<polygon class="cum-area" points="' + x(0) + ',' + y3(0) + ' ' + path3 + ' ' + x(T) + ',' + y3(0) + '"/>');
    parts.push('<polyline class="cum" points="' + path3 + '"/>');

    // time axis
    const axisY = p3.y + p3.h;
    parts.push('<line class="axis" x1="' + L + '" x2="' + (W - R) + '" y1="' + axisY + '" y2="' + axisY + '"/>');
    niceTicks(0, T, 10).forEach((tk) => {
      parts.push('<line class="grid" x1="' + x(tk) + '" x2="' + x(tk) + '" y1="' + p1.y + '" y2="' + axisY + '" opacity=".5"/>');
      parts.push('<text x="' + x(tk) + '" y="' + (axisY + 16) + '" text-anchor="middle">' + tk + ' s</text>');
    });
    parts.push('<text x="' + (W - R) + '" y="' + (axisY + 32) + '" text-anchor="end">Time since the first Wi-Fi scan. Colour and BLE times are estimated between stamped Wi-Fi lines.</text>');

    const H = axisY + 40;
    parts.push('<line class="xh" id="xh" x1="0" x2="0" y1="' + p1.y + '" y2="' + axisY + '" visibility="hidden"/>');
    parts.push('<rect class="hit" id="hit" x="' + L + '" y="' + p1.y + '" width="' + (W - L - R) + '" height="' + (axisY - p1.y) + '"/>');

    const svg = '<svg viewBox="0 0 ' + W + ' ' + H + '" role="img" aria-label="Timeline of signal strength, colour readings and driving time">' + parts.join('') + '</svg>';
    return { svg, W, L, R, T, raw, drives, col, steps };
  }

  function attachCrosshair(wrap, run, tl) {
    const svg = wrap.querySelector('svg');
    const xh = svg.querySelector('#xh');
    const hit = svg.querySelector('#hit');
    const tip = wrap.querySelector('.tip');
    const rel = (ms) => (ms - run.t0) / 1000;
    function phaseAt(sec) {
      const ms = run.t0 + sec * 1000;
      const p = run.phases.find((q) => ms >= q.t0 && ms <= q.t1);
      return p ? PHASE_NAME[p.phase] : '';
    }
    function move(evt) {
      const r = svg.getBoundingClientRect();
      const vx = ((evt.clientX - r.left) / r.width) * tl.W;
      const sec = ((vx - tl.L) / (tl.W - tl.L - tl.R)) * tl.T;
      if (sec < 0 || sec > tl.T) return;
      xh.setAttribute('x1', vx); xh.setAttribute('x2', vx); xh.setAttribute('visibility', 'visible');
      let near = null;
      tl.raw.forEach((e) => { if (!near || Math.abs(rel(e.t) - sec) < Math.abs(rel(near.t) - sec)) near = e; });
      let driven = 0;
      tl.drives.forEach((e) => { if (rel(e.t) <= sec) driven += e.d.ms / 1000; });
      let colour = null;
      tl.col.forEach((e) => { if (rel(e.t) <= sec) colour = e; });
      const depth = colour ? colour.d.depths.filter((v) => v !== null).pop() : null;
      tip.innerHTML = '<b>' + sec.toFixed(1) + ' s</b> · ' + phaseAt(sec) +
        '<br>Driven so far: ' + driven.toFixed(1) + ' s' +
        (near ? '<br>Nearest Wi-Fi scan: ' + near.d.dbm + ' dBm (' + rel(near.t).toFixed(1) + ' s)' : '') +
        (depth !== null && depth !== undefined ? '<br>Last colour: ' + LogParser.DEPTH_LABEL[depth] : '');
      tip.hidden = false;
      const wr = wrap.getBoundingClientRect();
      const left = evt.clientX - wr.left + wrap.scrollLeft + 14;
      tip.style.left = Math.min(left, wrap.scrollWidth - 230) + 'px';
      tip.style.top = (evt.clientY - wr.top + 12) + 'px';
    }
    hit.addEventListener('pointermove', move);
    hit.addEventListener('pointerleave', () => { xh.setAttribute('visibility', 'hidden'); tip.hidden = true; });
  }

  // ---------- path ----------
  function pathSVG(run) {
    if (!run.segs.length) return '<p class="empty">No driving in this run.</p>';
    const S = 400; const pad = 36;
    const xs = [0]; const ys = [0];
    run.segs.forEach((g) => { xs.push(g.x1); ys.push(g.y1); });
    run.pathStops.forEach((p) => { xs.push(p.x); ys.push(p.y); });
    const minx = Math.min.apply(null, xs); const maxx = Math.max.apply(null, xs);
    const miny = Math.min.apply(null, ys); const maxy = Math.max.apply(null, ys);
    const span = Math.max(maxx - minx, maxy - miny, 800);
    const k = (S - 2 * pad) / span;
    const cx = (minx + maxx) / 2; const cy = (miny + maxy) / 2;
    const px = (v) => S / 2 + (v - cx) * k;
    const py = (v) => S / 2 - (v - cy) * k;
    const dbms = run.pathStops.map((p) => p.dbm);
    const dlo = Math.min.apply(null, dbms); const dhi = Math.max.apply(null, dbms);
    const out = [];
    const scaleLen = 1000 * k;
    out.push('<line class="axis" x1="' + pad + '" x2="' + (pad + scaleLen) + '" y1="' + (S - 14) + '" y2="' + (S - 14) + '"/><text x="' + (pad + scaleLen + 6) + '" y="' + (S - 10) + '">1 s of driving</text>');
    run.segs.forEach((g) => out.push('<line class="' + (g.dir > 0 ? 'seg-f' : 'seg-b') + '" x1="' + px(g.x0) + '" y1="' + py(g.y0) + '" x2="' + px(g.x1) + '" y2="' + py(g.y1) + '"/>'));
    run.pathStops.forEach((p) => {
      const n = dhi > dlo ? (p.dbm - dlo) / (dhi - dlo) : 1;
      out.push('<circle class="stop" cx="' + px(p.x) + '" cy="' + py(p.y) + '" r="4.5" style="fill-opacity:' + (0.25 + 0.75 * n).toFixed(2) + '"><title>Stop ' + p.n + ': ' + p.dbm.toFixed(0) + ' dBm</title></circle>');
    });
    out.push('<text class="ttl" x="' + (px(0) + 8) + '" y="' + (py(0) + 4) + '">start</text><circle cx="' + px(0) + '" cy="' + py(0) + '" r="4" fill="none" stroke="currentColor"/>');
    const last = run.segs[run.segs.length - 1];
    out.push('<text class="ttl" x="' + (px(last.x1) + 8) + '" y="' + (py(last.y1) + 4) + '">end</text>');
    return '<svg viewBox="0 0 ' + S + ' ' + S + '" style="min-width:0;max-width:460px;margin:0 auto" role="img" aria-label="Estimated path of the robot">' + out.join('') + '</svg>';
  }

  function stepsTable(run) {
    if (!run.stops.length) return '<p class="empty">No stops with signal readings in this log.</p>';
    const maxWait = Math.max.apply(null, run.stops.map((s) => s.wallMs || 0));
    return '<div class="scroll"><table><thead><tr><th>Stop</th><th>At</th><th>Heading</th><th>Moved</th><th>Wait before it</th><th>Wi-Fi dBm</th></tr></thead><tbody>' +
      run.stops.map((s) => '<tr><td>' + s.n + '</td><td>' + ((s.t - run.t0) / 1000).toFixed(0) + ' s</td><td>' + s.heading + '°</td><td>' +
        (s.movedMs === null ? '–' : s.movedMs + ' ms') + '</td><td class="bar-cell"><i style="width:' + (maxWait ? (s.wallMs || 0) / maxWait * 100 : 0) + '%"></i><span>' +
        (s.wallMs === null ? '–' : secs(s.wallMs)) + '</span></td><td>' + s.dbm1 + ' / ' + s.dbm2 + '</td></tr>').join('') + '</tbody></table></div>';
  }

  // ---------- colour scan radial ----------
  function scanSVG(scan, idx) {
    const S = 220; const c = S / 2 + 14; const R = 78;
    const out = [];
    for (let d = 1; d <= 7; d++) out.push('<circle cx="' + c + '" cy="' + c + '" r="' + (R * d / 7) + '" fill="none" class="grid"/>');
    [2, 4, 6].forEach((d) => out.push('<text x="' + (c + 3) + '" y="' + (c - R * d / 7 - 2) + '" style="font-size:9px">' + LogParser.DEPTH_LABEL[d] + '</text>'));
    const pts = scan.readings.map((r) => {
      const a = (r.angle * Math.PI) / 180;
      const rad = (Math.max(0, r.depth || 0) / 7) * R;
      return [c + rad * Math.sin(a), c - rad * Math.cos(a), r];
    });
    out.push('<polygon class="poly" points="' + pts.map((p) => p[0] + ',' + p[1]).join(' ') + '"/>');
    scan.readings.forEach((r, i) => {
      const a = (r.angle * Math.PI) / 180;
      out.push('<text x="' + (c + (R + 14) * Math.sin(a)) + '" y="' + (c - (R + 14) * Math.cos(a) + 3) + '" text-anchor="middle" style="font-size:9px">' + r.angle + '°</text>');
      out.push('<circle class="cdot" cx="' + pts[i][0] + '" cy="' + pts[i][1] + '" r="3"><title>' + r.angle + '°: ' + esc(r.zone) + '</title></circle>');
    });
    if (scan.result && scan.result.valid) {
      const a = (scan.result.vector_angle * Math.PI) / 180;
      out.push('<line class="vec" x1="' + c + '" y1="' + c + '" x2="' + (c + R * Math.sin(a)) + '" y2="' + (c - R * Math.cos(a)) + '"/><circle cx="' + (c + R * Math.sin(a)) + '" cy="' + (c - R * Math.cos(a)) + '" r="4" fill="var(--accent)"/>');
    }
    const title = 'Scan ' + (idx + 1) + (scan.result ? ' · points ' + scan.result.vector_angle + '°' + (scan.result.valid ? '' : ' (not valid)') : '');
    return '<div class="card" style="padding:8px"><svg viewBox="0 0 ' + S + ' ' + (S + 14) + '" style="min-width:0" role="img" aria-label="' + esc(title) + '"><text class="ttl" x="6" y="14">' + esc(title) + '</text>' + out.join('') + '</svg></div>';
  }

  // ---------- events ----------
  const CATS = {
    decisions: ['follow', 'dip', 'dip_check', 'return', 'trial', 'direction', 'approach', 'route', 'cruise'],
    colour: ['mat', 'mat_step', 'arrival', 'step', 'colour_scan_result'],
    problems: ['failed', 'fault', 'result', 'faultmask', 'imu'],
    transfer: ['xfer'],
    moves: ['drive', 'turn'],
  };
  const CAT_LABEL = { decisions: 'Search decisions', colour: 'Colour and mat', problems: 'Faults and result', transfer: 'Transfer', moves: 'Drives and turns' };

  function eventRows(run) {
    const on = new Set();
    Object.keys(CATS).forEach((c) => { if (eventFilter[c]) CATS[c].forEach((t) => on.add(t)); });
    const rows = run.events.filter((e) => on.has(e.type));
    const shown = rows.slice(0, 700);
    const html = shown.map((e) => {
      const cls = e.type === 'failed' || e.type === 'fault' || (e.type === 'result' && run.resultKind === 'fail') ? ' bad' : e.type === 'result' && run.resultKind === 'ok' ? ' good' : '';
      const tm = e.t === null ? '–' : (e.tEst ? '~' : '') + ((e.t - run.t0) / 1000).toFixed(1);
      return '<div class="ev' + cls + '"><span class="tm">' + tm + '</span><span class="ty">' + e.type + '</span><span class="tx">' + esc(e.raw.length > 240 ? e.raw.slice(0, 240) + '…' : e.raw) + '</span></div>';
    }).join('');
    return (html || '<p class="empty">No events in the selected groups.</p>') + (rows.length > shown.length ? '<p class="empty">Showing the first ' + shown.length + ' of ' + rows.length + '.</p>' : '');
  }

  function renderRun() {
    const run = runs[current];
    const parts = [];
    parts.push('<div class="picker"><label for="pick">Run</label><select id="pick">' + runs.map((r, i) => '<option value="' + i + '"' + (i === current ? ' selected' : '') + '>' + esc((r.date ? r.date + ' · ' : '') + r.title) + '</option>').join('') + '</select>' +
      '<button id="prev" aria-label="Previous run"' + (current === 0 ? ' disabled' : '') + '>‹</button><button id="next" aria-label="Next run"' + (current === runs.length - 1 ? ' disabled' : '') + '>›</button></div>');
    parts.push('<section class="head"><h2>' + esc(run.title) + '</h2><div class="meta"><span class="chip ' + run.resultKind + '">' + esc(run.result || 'no result line') + '</span><span>' + esc(run.date) + '</span><span class="mono">' + esc(run.name) + '</span><span class="links">' + fileLinks(run) + '</span></div>' +
      (run.banner ? '<div class="meta"><span>Firmware:</span><span class="mono">' + esc(run.banner) + '</span></div>' : '') + '</section>');
    parts.push(tiles(run));
    const ins = insights(run);
    if (ins.length) parts.push('<section class="card"><h2>What this run shows</h2><ul class="insights">' + ins.map((i) => '<li>' + i + '</li>').join('') + '</ul></section>');
    parts.push(timeUse(run));
    if (run.timed) {
      parts.push('<section><h2>Timeline</h2><p class="sub">Top strip: what the robot was doing. Hover the charts for values at that moment.</p><div class="card chart" id="tl"></div></section>');
      parts.push('<section class="two"><div><h2>Estimated path</h2><p class="sub">Dead-reckoned from drive commands and headings, relative to the start. Darker stop = stronger Wi-Fi. Dashed red = reversing. Sign of heading may be mirrored.</p><div class="card">' + pathSVG(run) + '</div></div>' +
        '<div><h2>Stops</h2><p class="sub">The wait is the time since the previous stop, almost all scanning and checking.</p><div class="card">' + stepsTable(run) + '</div></div></section>');
    }
    if (run.scans.length) parts.push('<section><h2>Colour scans</h2><p class="sub">Ring depth read at each 30° step while turning on the spot. The arrow is the direction the robot would steer.</p><div class="scans">' + run.scans.map(scanSVG).join('') + '</div></section>');
    parts.push('<section><h2>Event log</h2><div class="chips" id="chips">' + Object.keys(CATS).map((c) => '<button data-cat="' + c + '" aria-pressed="' + eventFilter[c] + '">' + CAT_LABEL[c] + '</button>').join('') + '</div><div class="card events" id="events">' + eventRows(run) + '</div><p class="foot">Times marked ~ are estimated between stamped Wi-Fi lines.</p></section>');
    $('#main').innerHTML = parts.join('');
    if (run.timed) {
      const tl = buildTimeline(run);
      const wrap = $('#tl');
      wrap.innerHTML = tl.svg + '<div class="tip" hidden></div>';
      attachCrosshair(wrap, run, tl);
    }
    $('#pick').addEventListener('change', (e) => { current = +e.target.value; setView('run'); });
    $('#prev').addEventListener('click', () => { current--; setView('run'); });
    $('#next').addEventListener('click', () => { current++; setView('run'); });
    $('#chips').addEventListener('click', (e) => {
      const b = e.target.closest('button');
      if (!b) return;
      eventFilter[b.dataset.cat] = !eventFilter[b.dataset.cat];
      b.setAttribute('aria-pressed', eventFilter[b.dataset.cat]);
      $('#events').innerHTML = eventRows(run);
    });
  }

  function renderCompare() {
    const best = Math.max.apply(null, runs.map((r) => r.stats.driveShare || 0));
    $('#main').innerHTML = '<section><div class="head"><h2>Compare runs</h2><p class="sub">Click a row to open the run. The share column is the fraction of run time spent driving.</p></div>' +
      '<div class="card"><div class="scroll" style="max-height:none"><table><thead><tr><th>Run</th><th>Firmware</th><th>Run time</th><th>Moving</th><th>Moving share</th><th>Stops</th><th>Per stop</th><th>Result</th></tr></thead><tbody>' +
      runs.map((r, i) => {
        const s = r.stats;
        const ver = (/line-search (v\d+)/.exec(r.banner) || [])[1] || '';
        return '<tr class="link" data-i="' + i + '"><td style="white-space:normal;min-width:220px">' + esc(r.title) + '</td><td>' + esc(ver) + '</td><td>' + (r.timed ? duration(s.wallMs) : '–') + '</td><td>' + (r.timed ? secs(s.driveMs) : '–') + '</td>' +
          '<td class="bar-cell">' + (s.driveShare === null ? '–' : '<i style="width:' + (best ? s.driveShare / best * 100 : 0) + '%;background:var(--c-drive)"></i><span>' + pct(s.driveShare, 1) + '</span>') + '</td>' +
          '<td>' + (s.stops || '–') + '</td><td>' + (s.medianStepWallMs ? secs(s.medianStepWallMs) : '–') + '</td>' +
          '<td><span class="chip ' + r.resultKind + '">' + (r.resultKind === 'ok' ? 'completed' : r.resultKind === 'fail' ? 'no payload' : 'n/a') + '</span></td></tr>';
      }).join('') + '</tbody></table></div></div></section>';
    $('#main').querySelectorAll('tr.link').forEach((tr) => tr.addEventListener('click', () => { current = +tr.dataset.i; setView('run'); }));
  }

  function setView(v) {
    view = v;
    ['runs', 'run', 'compare'].forEach((name) => $('#tab-' + name).setAttribute('aria-selected', v === name));
    if (v === 'run') renderRun(); else if (v === 'compare') renderCompare(); else renderRuns();
    // Keep the address shareable: #runs, #compare, or #run=<file name without .txt>.
    const hash = v === 'run' ? 'run=' + runs[current].name.replace(/\.txt$/, '') : v;
    try { history.replaceState(null, '', '#' + hash); } catch (e) { /* file:// in some browsers */ }
    window.scrollTo({ top: 0 });
  }

  // ---------- loading ----------
  function toast(msg) {
    const t = $('#toast');
    t.textContent = msg; t.hidden = false;
    clearTimeout(toast.id);
    toast.id = setTimeout(() => { t.hidden = true; }, 3500);
  }
  function addFiles(files) {
    const list = Array.from(files).filter((f) => /\.txt$|text\//.test(f.name + f.type));
    if (!list.length) { toast('Choose a robot log (.txt)'); return; }
    Promise.all(list.map((f) => f.text().then((text) => ({ name: f.name, text })))).then((loaded) => {
      loaded.forEach((l) => runs.push(LogParser.parse(l.text, { name: l.name, title: l.name.replace(/\.txt$/, '') })));
      current = runs.length - 1;
      toast('Added ' + loaded.length + ' log' + (loaded.length > 1 ? 's' : ''));
      setView('run');
    });
  }

  function init() {
    runs = (window.RUNS || []).map((r) => Object.assign(LogParser.parse(r.text, r), { note: r.note, embedded: true }));
    if (!runs.length) { $('#main').innerHTML = '<p class="empty">No logs embedded. Run <code>node docs/tools/build_runs.js</code> or open a log file.</p>'; }
    // The Run tab starts on the newest run.
    current = Math.max(0, runs.length - 1);
    // Deep links: #run=purple opens the first run whose file name contains "purple"; #compare and #runs open those tabs.
    const hash = decodeURIComponent(location.hash.slice(1));
    const wanted = /^run=(.+)$/.exec(hash);
    const hit = wanted ? runs.findIndex((r) => r.name.includes(wanted[1])) : -1;
    if (hit >= 0) current = hit;
    $('#tab-runs').addEventListener('click', () => setView('runs'));
    $('#tab-run').addEventListener('click', () => setView('run'));
    $('#tab-compare').addEventListener('click', () => setView('compare'));
    $('#file').addEventListener('change', (e) => { addFiles(e.target.files); e.target.value = ''; });
    let depth = 0;
    window.addEventListener('dragenter', (e) => { e.preventDefault(); depth++; $('#drop').hidden = false; });
    window.addEventListener('dragleave', () => { depth = Math.max(0, depth - 1); if (!depth) $('#drop').hidden = true; });
    window.addEventListener('dragover', (e) => e.preventDefault());
    window.addEventListener('drop', (e) => { e.preventDefault(); depth = 0; $('#drop').hidden = true; addFiles(e.dataTransfer.files); });
    if (runs.length) setView(hit >= 0 ? 'run' : hash === 'compare' ? 'compare' : 'runs');
  }
  init();
}());
