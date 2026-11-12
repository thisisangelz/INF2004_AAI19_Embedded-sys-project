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
    if (r.draft) {
      return '<span class="chip draft" title="Only in this browser until it is committed to docs/runs/">draft</span>' +
        '<button class="linkbtn" data-dl="' + esc(r.name) + '">Download .txt</button><button class="linkbtn" data-rm="' + esc(r.name) + '">Remove</button>';
    }
    return (r.note ? '<a href="' + NOTE_BASE + encodeURIComponent(r.note) + '" target="_blank" rel="noopener">Notes</a>' : '<span class="muted">No notes yet</span>') +
      '<a href="runs/' + encodeURIComponent(r.name) + '" target="_blank" rel="noopener">Raw log</a>';
  }

  function download(name, text, type) {
    const a = document.createElement('a');
    a.href = URL.createObjectURL(new Blob([text], { type: type || 'text/plain' }));
    a.download = name;
    document.body.appendChild(a); a.click(); a.remove();
    setTimeout(() => URL.revokeObjectURL(a.href), 1000);
  }

  // ---------- pasted runs, kept in this browser until committed ----------
  const DRAFTS_KEY = 'runlog.drafts.v1';
  const sameLog = (a, b) => a.replace(/\r\n/g, '\n').trim() === b.replace(/\r\n/g, '\n').trim();
  const Drafts = {
    list: [],
    load() { try { this.list = JSON.parse(localStorage.getItem(DRAFTS_KEY) || '[]'); } catch (e) { this.list = []; } },
    save() { try { localStorage.setItem(DRAFTS_KEY, JSON.stringify(this.list)); return true; } catch (e) { return false; } },
  };
  const draftRun = (d) => Object.assign(LogParser.parse(d.text, { name: d.name, title: d.name.replace(/^\d{4}-\d{2}-\d{2}-/, '').replace(/\.txt$/, '').replace(/-/g, ' ') }), { draft: true, source: d.text });

  // Adds pasted or opened logs. Returns how many were added.
  function addLogs(items) {
    let added = 0;
    items.forEach((item) => {
      const parsed = LogParser.parse(item.text, { name: item.name });
      if (!parsed.banner && parsed.events.length < 5) { toast(item.name + ': this does not look like robot output'); return; }
      const dup = runs.findIndex((r) => sameLog(r.source, item.text));
      if (dup >= 0) { current = dup; toast('That log is already here: ' + runs[dup].title); return; }
      let name = item.name;
      for (let k = 2; runs.some((r) => r.name === name); k++) name = item.name.replace(/(-\d+)?\.txt$/, '-' + k + '.txt');
      const d = { name, text: item.text, added: new Date().toISOString() };
      Drafts.list.push(d);
      runs.push(draftRun(d));
      current = runs.length - 1;
      added++;
    });
    if (added && !Drafts.save()) toast('This browser will not keep the run after you close the page. Download it now.');
    else if (added) toast('Added. Download the .txt into docs/runs/ and commit it to keep it.');
    if (added || items.length) setView('run');
    return added;
  }

  function removeDraft(name) {
    if (!confirm('Remove ' + name + ' from this browser? Download it first if you still need it.')) return;
    Drafts.list = Drafts.list.filter((d) => d.name !== name);
    Drafts.save();
    runs = runs.filter((r) => r.name !== name);
    current = Math.min(current, runs.length - 1);
    setView('runs');
  }

  function draftBanner() {
    const n = Drafts.list.length;
    if (!n) return '';
    return '<div class="card notice"><p><b>' + n + ' run' + (n > 1 ? 's are' : ' is') + ' only in this browser.</b> Download ' + (n > 1 ? 'them' : 'it') + ', put the file' + (n > 1 ? 's' : '') + ' in <code>docs/runs/</code>, then commit. Ask Claude to write the notes once the logs are in the repo.</p>' +
      '<div class="actions"><button id="dl-all">Download ' + (n > 1 ? 'all ' + n : 'it') + '</button></div></div>';
  }
  const resultLabel = (r) => (r.resultKind === 'ok' ? 'completed' : r.resultKind === 'fail' ? 'no payload' : 'no result');

  // ---------- all runs ----------
  function renderRuns() {
    const byDate = {};
    runs.forEach((r, i) => { (byDate[r.date || 'Undated'] = byDate[r.date || 'Undated'] || []).push(i); });
    const dates = Object.keys(byDate).sort().reverse();
    const fmtDate = (d) => (/^\d{4}-\d{2}-\d{2}$/.test(d) ? new Date(d + 'T00:00:00').toLocaleDateString(undefined, { weekday: 'short', day: 'numeric', month: 'long', year: 'numeric' }) : d);
    $('#main').innerHTML = draftBanner() + '<section class="head"><h2>All runs</h2><p class="sub">' + runs.length + ' logs, newest first. Open a run to see its timeline, path and colour scans. Notes are the written analysis of each run.</p></section>' +
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
    const all = $('#dl-all');
    if (all) all.addEventListener('click', () => Drafts.list.forEach((d, k) => setTimeout(() => download(d.name, d.text), k * 400)));
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
      parts.push('<section><h2>Field map</h2><p class="sub">Where the robot went on the ' + FieldMap.GRID.cols + ' × ' + FieldMap.GRID.rows + ' grid of ' + FieldMap.GRID.tileCm + ' cm tiles. Enter the start, the way the robot faced, and both beacons.</p><div id="field"></div></section>');
      const stopsCard = '<div><h2>Stops</h2><p class="sub">The wait is the time since the previous stop, almost all scanning and checking.</p><div class="card">' + stepsTable(run) + '</div></div>';
      parts.push(FieldMap.complete(Store.get(run.name)) ? '<section>' + stopsCard + '</section>' :
        '<section class="two"><div><h2>Estimated path</h2><p class="sub">Without field positions: dead-reckoned from motor time and headings, relative to the start. Dashed red = reversing. May be mirrored.</p><div class="card">' + pathSVG(run) + '</div></div>' + stopsCard + '</section>');
    }
    if (run.scans.length) parts.push('<section><h2>Colour scans</h2><p class="sub">Ring depth read at each 30° step while turning on the spot. The arrow is the direction the robot would steer.</p><div class="scans">' + run.scans.map(scanSVG).join('') + '</div></section>');
    parts.push('<section><h2>Event log</h2><div class="chips" id="chips">' + Object.keys(CATS).map((c) => '<button data-cat="' + c + '" aria-pressed="' + eventFilter[c] + '">' + CAT_LABEL[c] + '</button>').join('') + '</div><div class="card events" id="events">' + eventRows(run) + '</div><p class="foot">Times marked ~ are estimated between stamped Wi-Fi lines.</p></section>');
    $('#main').innerHTML = parts.join('');
    if (run.timed) {
      const tl = buildTimeline(run);
      const wrap = $('#tl');
      wrap.innerHTML = tl.svg + '<div class="tip" hidden></div>';
      attachCrosshair(wrap, run, tl);
      renderFieldEditor(run);
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

  // ---------- field positions: committed layouts.json plus this browser's edits ----------
  const DRAFT_KEY = 'runlog.layouts.v1';
  const Store = {
    base: { settings: {}, runs: {} },
    draft: { settings: {}, runs: {} },
    load() {
      try { const d = JSON.parse(localStorage.getItem(DRAFT_KEY) || 'null'); if (d && d.runs) this.draft = { settings: d.settings || {}, runs: d.runs }; } catch (e) { /* storage blocked */ }
    },
    save() { try { localStorage.setItem(DRAFT_KEY, JSON.stringify(this.draft)); } catch (e) { toast('This browser will not keep edits. Export them before leaving.'); } },
    get(name) { return name in this.draft.runs ? this.draft.runs[name] : this.base.runs[name] || null; },
    set(name, layout) { this.draft.runs[name] = layout; this.save(); },
    settings() { return Object.assign({}, this.base.settings, this.draft.settings); },
    setSetting(k, v) { this.draft.settings[k] = v; this.save(); },
    pending() {
      const runsChanged = Object.keys(this.draft.runs).filter((k) => JSON.stringify(this.draft.runs[k]) !== JSON.stringify(this.base.runs[k] || null)).length;
      const settingsChanged = Object.keys(this.draft.settings).filter((k) => JSON.stringify(this.draft.settings[k]) !== JSON.stringify(this.base.settings[k])).length;
      return runsChanged + settingsChanged;
    },
    merged() {
      const runsOut = {};
      const all = Object.assign({}, this.base.runs, this.draft.runs);
      Object.keys(all).sort().forEach((k) => { if (all[k]) runsOut[k] = all[k]; });
      return {
        grid: { cols: FieldMap.GRID.cols, rows: FieldMap.GRID.rows, tile_cm: FieldMap.GRID.tileCm, origin: 'bottom-left corner of A1', x: 'cm to the right', y: 'cm up', bearing: 'degrees clockwise from up' },
        settings: this.settings(), runs: runsOut,
      };
    },
    discard() { this.draft = { settings: {}, runs: {} }; this.save(); },
  };

  function exportLayouts() {
    const blob = new Blob([JSON.stringify(Store.merged(), null, 2) + '\n'], { type: 'application/json' });
    const a = document.createElement('a');
    a.href = URL.createObjectURL(blob);
    a.download = 'layouts.json';
    document.body.appendChild(a); a.click(); a.remove();
    setTimeout(() => URL.revokeObjectURL(a.href), 1000);
    toast('Saved layouts.json. Put it in docs/runs/, rebuild with docs/tools/build_runs.js, then commit.');
  }
  function importLayouts(file) {
    file.text().then((text) => {
      const data = JSON.parse(text);
      if (!data || typeof data.runs !== 'object') throw new Error('no runs');
      Object.keys(data.runs).forEach((k) => { Store.draft.runs[k] = data.runs[k]; });
      Object.assign(Store.draft.settings, data.settings || {});
      Store.save();
      toast('Imported positions for ' + Object.keys(data.runs).length + ' runs');
      setView(view);
    }).catch(() => toast('That file is not a layouts.json'));
  }
  const pendingNote = () => {
    const n = Store.pending();
    return n ? '<span class="warn">' + n + ' change' + (n > 1 ? 's' : '') + ' saved in this browser only.</span> Export, then commit <code>docs/runs/layouts.json</code> so everyone sees ' + (n > 1 ? 'them' : 'it') + '.' : 'All positions match the committed <code>layouts.json</code>.';
  };

  function fieldParams() {
    const s = Store.settings();
    const sign = s.heading_sign === -1 ? -1 : 1;
    const layoutOf = (r) => Store.get(r.name);
    const cal = FieldMap.calibrate(runs, layoutOf, sign);
    const mirrored = FieldMap.calibrate(runs, layoutOf, -sign);
    const manual = typeof s.speed_cm_s === 'number' && s.speed_cm_s > 0 ? s.speed_cm_s : null;
    const speed = manual || cal.speed || FieldMap.DEFAULT_SPEED_CM_S;
    const source = manual ? 'set by hand' : cal.speed ? 'calibrated from ' + cal.fits.length + ' run' + (cal.fits.length > 1 ? 's' : '') + ' that reached AP1' : 'assumed. No placed run that reached AP1 gives a usable fit yet';
    return { speedCmS: speed, headingSign: sign, reanchor: s.reanchor !== false, cal, mirrored, manual, source, calibrated: !!(manual || cal.speed) };
  }

  // ---------- field drawing ----------
  // paths: [{ mapped, layout, cls }] · layout: markers to draw · opts.edit: clickable
  function fieldSVG(layout, paths, opts) {
    opts = opts || {};
    const G = FieldMap.GRID; const size = FieldMap.sizeCm();
    let minx = 0; let miny = 0; let maxx = size.w; let maxy = size.h;
    paths.forEach((p) => p.mapped && p.mapped.trail.forEach((q) => { minx = Math.min(minx, q.x); maxx = Math.max(maxx, q.x); miny = Math.min(miny, q.y); maxy = Math.max(maxy, q.y); }));
    const lim = (v, a, b) => Math.max(a, Math.min(b, v));
    minx = lim(minx, -size.w, 0) - 34; miny = lim(miny, -size.h, 0) - 34;
    maxx = lim(maxx, size.w, 2 * size.w) + 20; maxy = lim(maxy, size.h, 2 * size.h) + 20;
    const X = (x) => x; const Y = (y) => -y;
    const out = [];
    out.push('<rect class="f-out" x="' + minx + '" y="' + (-maxy) + '" width="' + (maxx - minx) + '" height="' + (maxy - miny) + '"/>');
    for (let c = 0; c < G.cols; c++) {
      for (let r = 0; r < G.rows; r++) out.push('<rect class="f-tile' + ((c + r) % 2 ? ' alt' : '') + '" x="' + c * G.tileCm + '" y="' + (-(r + 1) * G.tileCm) + '" width="' + G.tileCm + '" height="' + G.tileCm + '"/>');
      out.push('<text class="f-lab" x="' + (c + 0.5) * G.tileCm + '" y="' + 20 + '" text-anchor="middle">' + String.fromCharCode(65 + c) + '</text>');
    }
    for (let r = 0; r < G.rows; r++) out.push('<text class="f-lab" x="-14" y="' + (-(r + 0.5) * G.tileCm + 5) + '" text-anchor="middle">' + (r + 1) + '</text>');
    paths.forEach((p) => {
      const m = p.mapped; if (!m) return;
      const cls = p.cls || '';
      m.segs.forEach((s) => out.push('<line class="f-seg ' + (s.dir > 0 ? 'fwd' : 'rev') + ' ' + cls + '" x1="' + X(s.x0) + '" y1="' + Y(s.y0) + '" x2="' + X(s.x1) + '" y2="' + Y(s.y1) + '"/>'));
      m.arrivals.forEach((a) => {
        const ap = p.layout && p.layout['ap' + a.ap];
        if (a.anchored && ap && a.errorCm > 1) out.push('<line class="f-jump ' + cls + '" x1="' + X(a.at.x) + '" y1="' + Y(a.at.y) + '" x2="' + X(ap.x) + '" y2="' + Y(ap.y) + '"><title>Path snapped ' + a.errorCm.toFixed(0) + ' cm onto AP' + a.ap + ' at the transfer</title></line>');
      });
      if (!opts.overlay) {
        const dbms = m.stops.map((s) => s.dbm); const lo = Math.min.apply(null, dbms); const hi = Math.max.apply(null, dbms);
        m.stops.forEach((s, i) => out.push('<circle class="f-stop" cx="' + X(s.x) + '" cy="' + Y(s.y) + '" r="3.6" style="fill-opacity:' + (hi > lo ? 0.25 + 0.75 * (s.dbm - lo) / (hi - lo) : 1).toFixed(2) + '"><title>Stop ' + (i + 1) + ': ' + s.dbm.toFixed(0) + ' dBm</title></circle>'));
      }
      if (opts.overlay && p.layout && p.layout.start) out.push('<circle class="f-start small ' + cls + '" cx="' + X(p.layout.start.x) + '" cy="' + Y(p.layout.start.y) + '" r="5"><title>Start of ' + esc(p.title || 'run') + '</title></circle>');
      const e = m.end;
      out.push('<path class="f-end ' + cls + '" d="M' + (X(e.x) - 5) + ' ' + (Y(e.y) - 5) + 'l10 10m0 -10l-10 10"><title>' + esc(p.title || 'End of the run') + '</title></path>');
    });
    if (layout) {
      ['ap1', 'ap2'].forEach((k) => {
        const ap = layout[k]; if (!ap) return;
        out.push('<circle class="f-' + k + '" cx="' + X(ap.x) + '" cy="' + Y(ap.y) + '" r="9"/><text class="f-aplab" x="' + X(ap.x) + '" y="' + (Y(ap.y) - 13) + '" text-anchor="middle">' + k.toUpperCase() + '</text>');
      });
      const s = layout.start;
      if (s) {
        out.push('<circle class="f-start" cx="' + X(s.x) + '" cy="' + Y(s.y) + '" r="7"/>');
        if (typeof s.bearing === 'number') {
          const b = (s.bearing * Math.PI) / 180;
          out.push('<line class="f-arrow" x1="' + X(s.x) + '" y1="' + Y(s.y) + '" x2="' + X(s.x + 26 * Math.sin(b)) + '" y2="' + Y(s.y + 26 * Math.cos(b)) + '" marker-end="url(#f-head)"/>');
        }
        out.push('<text class="f-aplab" x="' + X(s.x) + '" y="' + (Y(s.y) + 21) + '" text-anchor="middle">Start</text>');
      }
    }
    const defs = '<defs><marker id="f-head" viewBox="0 0 10 10" refX="6" refY="5" markerWidth="5" markerHeight="5" orient="auto-start-reverse"><path d="M0 0L10 5L0 10z" class="f-headfill"/></marker></defs>';
    return '<svg class="field' + (opts.edit ? ' edit' : '') + '" viewBox="' + minx + ' ' + (-maxy) + ' ' + (maxx - minx) + ' ' + (maxy - miny) + '" role="img" aria-label="' + esc(opts.label || 'Field map') + '">' + defs + out.join('') + '</svg>';
  }

  // ---------- field editor in the Run tab ----------
  let placing = 'start';
  const BEARINGS = [[0, '↑'], [45, '↗'], [90, '→'], [135, '↘'], [180, '↓'], [225, '↙'], [270, '←'], [315, '↖']];
  const posText = (p) => (p ? FieldMap.tileName(p) + ' · ' + p.x + ', ' + p.y + ' cm' : 'not set');

  function renderFieldEditor(run) {
    const host = $('#field'); if (!host) return;
    const layout = Store.get(run.name) || {};
    const P = fieldParams();
    const mapped = FieldMap.mapRun(run, layout, P);
    const met = FieldMap.metrics(run, layout, mapped);
    const prevIdx = (() => { for (let i = current - 1; i >= 0; i--) if (FieldMap.complete(Store.get(runs[i].name))) return i; for (let i = runs.length - 1; i > current; i--) if (FieldMap.complete(Store.get(runs[i].name))) return i; return -1; })();
    const facts = [];
    if (mapped) {
      facts.push('Drive speed <b>' + P.speedCmS.toFixed(1) + ' cm/s</b> (' + P.source + ').');
      mapped.arrivals.forEach((a) => { if (a.errorCm !== null) facts.push('At the AP' + a.ap + ' transfer the path was <b>' + a.errorCm.toFixed(0) + ' cm</b> from AP' + a.ap + (a.anchored ? ' and was snapped onto it.' : '.')); });
      met.legs.forEach((l) => facts.push('<b>Leg ' + l.leg + '</b> (to AP' + l.leg + ', ' + l.straightCm.toFixed(0) + ' cm straight): ' +
        (l.reached ? 'reached in <b>' + (l.timeMs !== null ? secs(l.timeMs, 0) : '?') + '</b>, ' + l.drivenCm.toFixed(0) + ' cm driven' + (l.efficiency && P.calibrated ? ' (' + pct(Math.min(1, l.efficiency)) + ' direct)' : '') + '.'
          : 'not reached. Closest <b>' + l.closestCm.toFixed(0) + ' cm</b>, ended ' + l.endCm.toFixed(0) + ' cm away after ' + l.drivenCm.toFixed(0) + ' cm driven.')));
      const ownFit = FieldMap.fitRun(run, layout, P.headingSign);
      if (ownFit && !ownFit.speed) facts.unshift('<span class="warn">At the AP1 transfer the path heads away from AP1.</span> Check the start position and the way it faced. This run is left out of the calibration.');
      if (!P.calibrated) facts.push('<span class="warn">Distances are not calibrated yet.</span> Enter positions for a run that reached AP1.');
    } else {
      facts.push('Place the start and pick the way the robot faced to draw the path on the grid.');
    }
    host.innerHTML = '<div class="field-wrap"><div class="card fieldcard">' + fieldSVG(layout, mapped ? [{ mapped, layout }] : [], { edit: true, label: 'Field map of this run' }) + '</div>' +
      '<div class="card editor">' +
      '<p class="sub">Choose what to place, then click the grid. Points snap to ' + FieldMap.SNAP_CM + ' cm.</p>' +
      '<div class="place" role="radiogroup" aria-label="What to place">' +
      [['start', 'Start'], ['ap1', 'AP1 · blue'], ['ap2', 'AP2 · red']].map((m) => '<button role="radio" data-place="' + m[0] + '" aria-checked="' + (placing === m[0]) + '"><span class="dot ' + m[0] + '"></span><span>' + m[1] + '</span><small>' + posText(m[0] === 'start' ? layout.start : layout[m[0]]) + '</small></button>').join('') + '</div>' +
      '<div class="facing"><span class="sub">Start facing</span><div class="compass">' + BEARINGS.map((b) => '<button data-bearing="' + b[0] + '" aria-pressed="' + (!!layout.start && layout.start.bearing === b[0]) + '" title="' + b[0] + '°"' + (layout.start ? '' : ' disabled') + '>' + b[1] + '</button>').join('') +
      '</div><label class="deg">or <input id="bearing" type="number" min="0" max="359" step="5" value="' + (layout.start && typeof layout.start.bearing === 'number' ? layout.start.bearing : '') + '"' + (layout.start ? '' : ' disabled') + '> °</label></div>' +
      '<div class="actions">' + (prevIdx >= 0 ? '<button id="copy-layout">Copy from ' + esc(runs[prevIdx].date + ' · ' + runs[prevIdx].title.slice(0, 40)) + (runs[prevIdx].title.length > 40 ? '…' : '') + '</button>' : '') +
      '<button id="clear-layout"' + (Store.get(run.name) ? '' : ' disabled') + '>Clear</button></div>' +
      '<ul class="insights">' + facts.map((f) => '<li>' + f + '</li>').join('') + '</ul>' +
      '<p class="foot">' + pendingNote() + ' <button class="linkbtn" id="export-layouts">Export layouts.json</button></p></div></div>';

    const update = (fn) => {
      const next = JSON.parse(JSON.stringify(Store.get(run.name) || {}));
      fn(next);
      const wasComplete = FieldMap.complete(layout);
      Store.set(run.name, Object.keys(next).length ? next : null);
      if (wasComplete !== FieldMap.complete(next)) { const y = window.scrollY; renderRun(); window.scrollTo(0, y); } else renderFieldEditor(run);
    };
    const svg = host.querySelector('svg.field');
    svg.addEventListener('click', (evt) => {
      const pt = svg.createSVGPoint(); pt.x = evt.clientX; pt.y = evt.clientY;
      const p = pt.matrixTransform(svg.getScreenCTM().inverse());
      const size = FieldMap.sizeCm();
      const x = Math.max(0, Math.min(size.w, FieldMap.snap(p.x)));
      const y = Math.max(0, Math.min(size.h, FieldMap.snap(-p.y)));
      update((l) => {
        if (placing === 'start') l.start = { x, y, bearing: l.start && typeof l.start.bearing === 'number' ? l.start.bearing : 0 };
        else l[placing] = { x, y };
      });
      // After placing one thing, move on to the next missing one.
      const l = Store.get(run.name) || {};
      placing = !l.start ? 'start' : !l.ap1 ? 'ap1' : !l.ap2 ? 'ap2' : placing;
      renderFieldEditor(run);
    });
    host.querySelectorAll('[data-place]').forEach((b) => b.addEventListener('click', () => { placing = b.dataset.place; renderFieldEditor(run); }));
    host.querySelectorAll('[data-bearing]').forEach((b) => b.addEventListener('click', () => update((l) => { l.start.bearing = +b.dataset.bearing; })));
    const deg = host.querySelector('#bearing');
    deg.addEventListener('change', () => { const v = ((Math.round(+deg.value) % 360) + 360) % 360; if (deg.value !== '' && isFinite(v)) update((l) => { l.start.bearing = v; }); });
    const copy = host.querySelector('#copy-layout');
    if (copy) copy.addEventListener('click', () => update((l) => { Object.assign(l, JSON.parse(JSON.stringify(Store.get(runs[prevIdx].name)))); }));
    host.querySelector('#clear-layout').addEventListener('click', () => update((l) => { delete l.start; delete l.ap1; delete l.ap2; }));
    host.querySelector('#export-layouts').addEventListener('click', exportLayouts);
  }

  // ---------- Field tab: calibration and numbers across runs ----------
  let fieldSetup = null;
  function renderField() {
    const P = fieldParams();
    const rows = runs.map((r, i) => {
      const layout = Store.get(r.name);
      const mapped = FieldMap.mapRun(r, layout, P);
      return { r, i, layout, mapped, met: FieldMap.metrics(r, layout, mapped), key: FieldMap.setupKey(layout) };
    });
    const placed = rows.filter((x) => x.mapped);
    const missing = rows.filter((x) => !x.mapped && x.r.timed);
    const leg1 = placed.map((x) => x.met.legs.find((l) => l.leg === 1)).filter(Boolean);
    const reached = leg1.filter((l) => l.reached);
    const med = (xs) => { const s = xs.filter((v) => v !== null).sort((a, b) => a - b); return s.length ? s[s.length >> 1] : null; };
    const best = reached.filter((l) => l.timeMs !== null).sort((a, b) => a.timeMs - b.timeMs)[0];

    const tiles = [
      ['Runs on the field', placed.length + ' / ' + rows.filter((x) => x.r.timed).length, missing.length ? missing.length + ' still need positions' : 'every timed run placed'],
      ['AP1 reached', leg1.length ? reached.length + ' / ' + leg1.length : '–', leg1.length ? pct(reached.length / leg1.length) + ' of placed runs' : ''],
      ['Time to AP1', med(reached.map((l) => l.timeMs)) !== null ? secs(med(reached.map((l) => l.timeMs)), 0) : '–', best ? 'median · best ' + secs(best.timeMs, 0) : 'median of runs that reached it'],
      ['Path directness', P.calibrated && med(reached.map((l) => l.efficiency)) !== null ? pct(Math.min(1, med(reached.map((l) => l.efficiency)))) : '–', 'straight line ÷ distance driven'],
      ['Closest miss', med(leg1.filter((l) => !l.reached).map((l) => l.closestCm)) !== null ? med(leg1.filter((l) => !l.reached).map((l) => l.closestCm)).toFixed(0) + ' cm' : '–', 'median, runs that missed AP1'],
    ];

    const setups = {};
    placed.forEach((x) => { if (x.key) (setups[x.key] = setups[x.key] || []).push(x); });
    const keys = Object.keys(setups).sort((a, b) => setups[b].length - setups[a].length);
    if (!keys.includes(fieldSetup)) fieldSetup = keys[0] || null;
    const group = fieldSetup ? setups[fieldSetup] : [];
    const overlay = fieldSetup ? fieldSVG({ ap1: group[0].layout.ap1, ap2: group[0].layout.ap2 }, group.map((x) => ({ mapped: x.mapped, layout: x.layout, cls: x.met.legs.some((l) => l.leg === 1 && l.reached) ? 'hit' : 'miss', title: x.r.title })), { overlay: true, label: 'All runs with this beacon setup' }) : '';

    const cm = (v) => (v === null || v === undefined ? '–' : v.toFixed(0) + ' cm');
    const table = '<table><thead><tr><th>Run</th><th>Start</th><th>AP1</th><th>AP2</th><th>Start→AP1</th><th>AP1</th><th>Time to AP1</th><th>Driven</th><th>Direct</th><th>Closest AP1</th><th>Closest AP2</th></tr></thead><tbody>' +
      placed.slice().reverse().map((x) => {
        const l1 = x.met.legs.find((l) => l.leg === 1); const l2 = x.met.legs.find((l) => l.leg === 2);
        return '<tr class="link" data-i="' + x.i + '"><td style="white-space:normal;min-width:220px">' + esc(x.r.date + ' · ' + x.r.title) + '</td><td>' + FieldMap.tileName(x.layout.start) + ' ' + x.layout.start.bearing + '°</td><td>' + FieldMap.tileName(x.layout.ap1) + '</td><td>' + FieldMap.tileName(x.layout.ap2) + '</td><td>' + cm(x.met.startToAp1Cm) + '</td>' +
          '<td>' + (l1 ? '<span class="chip ' + (l1.reached ? 'ok' : 'fail') + '">' + (l1.reached ? 'reached' : 'missed') + '</span>' : '–') + '</td><td>' + (l1 && l1.timeMs !== null ? secs(l1.timeMs, 0) : '–') + '</td><td>' + cm(l1 && l1.drivenCm) + '</td><td>' + (l1 && l1.efficiency && P.calibrated ? pct(Math.min(1, l1.efficiency)) : '–') + '</td>' +
          '<td>' + (l1 ? (l1.reached ? '0 cm' : cm(l1.closestCm)) : '–') + '</td><td>' + (l2 ? (l2.reached ? '0 cm' : cm(l2.closestCm)) : '–') + '</td></tr>';
      }).join('') + '</tbody></table>';

    const s = Store.settings();
    const calRows = P.cal.fits.map((f) => '<tr><td style="white-space:normal">' + esc(f.title) + '</td><td>' + f.speed.toFixed(1) + ' cm/s</td><td>' + f.errorCm.toFixed(0) + ' cm</td></tr>').join('');
    const mirrorHint = P.cal.fits.length >= 2 && P.mirrored.fits.length && P.mirrored.errorCm < P.cal.errorCm * 0.7
      ? '<p class="warn">Turning the headings the other way fits better (median ' + P.mirrored.errorCm.toFixed(0) + ' cm against ' + P.cal.errorCm.toFixed(0) + ' cm). Check the setting below.</p>' : '';

    $('#main').innerHTML = '<section class="head"><h2>Field</h2><p class="sub">Runs placed on the ' + FieldMap.GRID.cols + ' × ' + FieldMap.GRID.rows + ' grid of ' + FieldMap.GRID.tileCm + ' cm tiles. Use this to see how far each run got and what to optimise next. Enter positions in the Run tab.</p></section>' +
      '<div class="tiles">' + tiles.map((t) => '<div class="tile"><span class="k">' + t[0] + '</span><span class="v">' + esc(t[1]) + '</span><span class="s">' + esc(t[2]) + '</span></div>').join('') + '</div>' +
      (keys.length ? '<section><h2>Paths by beacon setup</h2><p class="sub">Every placed run with these beacon positions. Green = reached AP1, red = did not. × marks where each run ended.</p>' +
        '<div class="picker"><label for="setup">Setup</label><select id="setup">' + keys.map((k) => '<option' + (k === fieldSetup ? ' selected' : '') + '>' + esc(k + ' · ' + setups[k].length + ' run' + (setups[k].length > 1 ? 's' : '')) + '</option>').join('') + '</select></div>' +
        '<div class="card fieldcard wide">' + overlay + '</div></section>' : '') +
      '<section><h2>Runs on the field</h2>' + (placed.length ? '<p class="sub">Newest first. Click a row to open the run.</p><div class="card"><div class="scroll" style="max-height:none">' + table + '</div></div>' : '<p class="empty">No run has field positions yet. Open a run, then place the start, its facing and both beacons.</p>') +
      (missing.length ? '<p class="foot">Still to place: ' + missing.map((x) => '<button class="linkbtn" data-open="' + x.i + '">' + esc(x.r.date + ' · ' + x.r.title.slice(0, 48)) + '</button>').join(' · ') + '</p>' : '') + '</section>' +
      '<section class="two"><div><h2>Distance calibration</h2><p class="sub">The log has no position sensor, so distance is motor time × drive speed. Each run that reached AP1 gives the speed that puts its path on AP1. The leftover is the error that speed cannot remove (turns, slip, the arrival ring\'s offset from the beacon).</p>' +
        '<div class="card">' + (calRows ? '<table><thead><tr><th>Run</th><th>Speed</th><th>Leftover</th></tr></thead><tbody>' + calRows + '</tbody></table><p class="foot">Median: ' + P.cal.speed.toFixed(1) + ' cm/s, leftover ' + P.cal.errorCm.toFixed(0) + ' cm. A run used to calibrate itself will look more direct than it was. Trust the numbers once several runs are placed.</p>' : '<p class="empty">No placed run has reached AP1 yet. Until one does, the map assumes ' + FieldMap.DEFAULT_SPEED_CM_S + ' cm/s.</p>') + mirrorHint + '</div></div>' +
        '<div><h2>Settings</h2><p class="sub">Saved with the positions in layouts.json.</p><div class="card settings">' +
        '<label>Drive speed <input id="speed" type="number" min="1" max="200" step="0.5" placeholder="' + (P.cal.speed ? P.cal.speed.toFixed(1) : 'auto') + '" value="' + (P.manual || '') + '"> cm/s <small>Leave empty to use the calibration.</small></label>' +
        '<label><input id="sign" type="checkbox"' + (P.headingSign === 1 ? ' checked' : '') + '> A positive log heading turns the robot anticlockwise <small>From the motor wiring in motion.c. Untick if the paths come out mirrored.</small></label>' +
        '<label><input id="reanchor" type="checkbox"' + (P.reanchor ? ' checked' : '') + '> Snap the path onto AP1 at the transfer <small>The AP2 leg then starts from where the robot really was.</small></label>' +
        '<p class="foot">' + pendingNote() + '</p><div class="actions"><button id="export-layouts">Export layouts.json</button><label class="btn">Import…<input id="import-layouts" type="file" accept=".json,application/json" hidden></label>' + (Store.pending() ? '<button id="discard">Discard browser changes</button>' : '') + '</div></div></div></section>';

    const main = $('#main');
    main.querySelectorAll('tr.link').forEach((tr) => tr.addEventListener('click', () => { current = +tr.dataset.i; setView('run'); }));
    main.querySelectorAll('[data-open]').forEach((b) => b.addEventListener('click', () => { current = +b.dataset.open; setView('run'); }));
    const setup = $('#setup'); if (setup) setup.addEventListener('change', () => { fieldSetup = keys[setup.selectedIndex]; renderField(); });
    $('#speed').addEventListener('change', (e) => { const v = parseFloat(e.target.value); Store.setSetting('speed_cm_s', v > 0 ? v : null); renderField(); });
    $('#sign').addEventListener('change', (e) => { Store.setSetting('heading_sign', e.target.checked ? 1 : -1); renderField(); });
    $('#reanchor').addEventListener('change', (e) => { Store.setSetting('reanchor', e.target.checked); renderField(); });
    $('#export-layouts').addEventListener('click', exportLayouts);
    $('#import-layouts').addEventListener('change', (e) => { if (e.target.files[0]) importLayouts(e.target.files[0]); e.target.value = ''; });
    const discard = $('#discard'); if (discard) discard.addEventListener('click', () => { if (confirm('Discard every position and setting changed in this browser?')) { Store.discard(); renderField(); } });
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
    ['runs', 'run', 'compare', 'field'].forEach((name) => $('#tab-' + name).setAttribute('aria-selected', v === name));
    if (v === 'run') renderRun(); else if (v === 'compare') renderCompare(); else if (v === 'field') renderField(); else renderRuns();
    // Keep the address shareable: #runs, #compare, #field, or #run=<file name without .txt>.
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
    Promise.all(list.map((f) => f.text().then((text) => ({ name: /^\d{4}-\d{2}-\d{2}-/.test(f.name) ? f.name : today() + '-' + slug(f.name.replace(/\.txt$/, '')) + '.txt', text })))).then(addLogs);
  }
  const today = () => { const d = new Date(); return d.getFullYear() + '-' + String(d.getMonth() + 1).padStart(2, '0') + '-' + String(d.getDate()).padStart(2, '0'); };
  const slug = (s) => s.toLowerCase().replace(/[^a-z0-9]+/g, '-').replace(/^-+|-+$/g, '');

  function setupAddDialog() {
    const dlg = $('#add-dialog');
    const open = () => {
      $('#add-date').value = $('#add-date').value || today();
      const t = new Date();
      $('#add-name').placeholder = 'run-' + String(t.getHours()).padStart(2, '0') + String(t.getMinutes()).padStart(2, '0');
      if (dlg.showModal) dlg.showModal(); else dlg.setAttribute('open', '');
      $('#add-text').focus();
    };
    $('#add-run').addEventListener('click', open);
    $('#add-cancel').addEventListener('click', () => dlg.close());
    $('#add-files').addEventListener('click', () => { dlg.close(); $('#file').click(); });
    $('#add-form').addEventListener('submit', (e) => {
      e.preventDefault();
      const text = $('#add-text').value;
      if (!text.trim()) return;
      const name = $('#add-date').value + '-' + (slug($('#add-name').value) || $('#add-name').placeholder) + '.txt';
      dlg.close();
      if (addLogs([{ name, text }])) { $('#add-text').value = ''; $('#add-name').value = ''; }
    });
    // Delegated so the run list, run header and banner can all use them.
    $('#main').addEventListener('click', (e) => {
      const dl = e.target.closest('[data-dl]');
      if (dl) { const d = Drafts.list.find((x) => x.name === dl.dataset.dl); if (d) download(d.name, d.text); }
      const rm = e.target.closest('[data-rm]');
      if (rm) removeDraft(rm.dataset.rm);
    });
  }

  function init() {
    Store.base = window.LAYOUTS || { settings: {}, runs: {} };
    Store.load();
    runs = (window.RUNS || []).map((r) => Object.assign(LogParser.parse(r.text, r), { note: r.note, embedded: true, source: r.text }));
    // Pasted runs from earlier visits. Once a log is committed, its draft is dropped.
    Drafts.load();
    const before = Drafts.list.length;
    Drafts.list = Drafts.list.filter((d) => !runs.some((r) => r.name === d.name || sameLog(r.source, d.text)));
    if (Drafts.list.length !== before) Drafts.save();
    Drafts.list.forEach((d) => runs.push(draftRun(d)));
    if (!runs.length) { $('#main').innerHTML = '<p class="empty">No logs yet. Use <b>Add run…</b> to paste serial monitor output.</p>'; }
    // The Run tab starts on the newest run.
    current = Math.max(0, runs.length - 1);
    // Deep links: #run=purple opens the first run whose file name contains "purple"; #compare and #runs open those tabs.
    const hash = decodeURIComponent(location.hash.slice(1));
    const wanted = /^run=(.+)$/.exec(hash);
    const hit = wanted ? runs.findIndex((r) => r.name.includes(wanted[1])) : -1;
    if (hit >= 0) current = hit;
    $('#tab-runs').addEventListener('click', () => setView('runs'));
    $('#tab-run').addEventListener('click', () => setView('run'));
    $('#tab-field').addEventListener('click', () => setView('field'));
    $('#tab-compare').addEventListener('click', () => setView('compare'));
    setupAddDialog();
    $('#file').addEventListener('change', (e) => { addFiles(e.target.files); e.target.value = ''; });
    let depth = 0;
    window.addEventListener('dragenter', (e) => { e.preventDefault(); depth++; $('#drop').hidden = false; });
    window.addEventListener('dragleave', () => { depth = Math.max(0, depth - 1); if (!depth) $('#drop').hidden = true; });
    window.addEventListener('dragover', (e) => e.preventDefault());
    window.addEventListener('drop', (e) => { e.preventDefault(); depth = 0; $('#drop').hidden = true; addFiles(e.dataTransfer.files); });
    window.addEventListener('hashchange', () => {
      const h = decodeURIComponent(location.hash.slice(1));
      const m = /^run=(.+)$/.exec(h);
      const i = m ? runs.findIndex((r) => r.name.includes(m[1])) : -1;
      if (i >= 0 && !(view === 'run' && i === current)) { current = i; setView('run'); } else if (['runs', 'compare', 'field'].includes(h) && h !== view) setView(h);
    });
    if (runs.length) setView(hit >= 0 ? 'run' : hash === 'compare' || hash === 'field' ? hash : 'runs');
  }
  // gate.js calls this once the password has decrypted the data.
  window.startDashboard = init;
}());
