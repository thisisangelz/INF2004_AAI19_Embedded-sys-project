// Parses a saved robot run log (the text GP21 replays over USB) into events,
// time series and summary numbers. Runs in the browser and in Node.
//
// Only RSSI_RAW and LINE_POINT lines carry a firmware timestamp (t_ms, since
// boot). Every other line gets a time interpolated between the nearest two
// stamped lines, and is flagged tEst. Treat those times as "about when".
(function (root, factory) {
  if (typeof module === 'object' && module.exports) module.exports = factory();
  else root.LogParser = factory();
})(typeof self !== 'undefined' ? self : this, function () {
  'use strict';

  const DEPTH = { FLOOR: 0, WHITE: 1, RING1: 2, RING2: 3, RING3: 4, RING4: 5, RING5: 6, RING6: 7 };
  const DEPTH_LABEL = ['Floor', 'White', 'R1', 'R2', 'R3', 'R4', 'R5', 'R6'];
  // colour_arrival_check() waits one 103 ms integration plus 12 ms per reading, three readings.
  const COLOUR_CHECK_MS = 345;

  const num = (v) => (/^-?\d+(\.\d+)?$/.test(v) ? Number(v) : v);
  function kv(line) {
    const out = {};
    line.split(',').slice(1).forEach((part) => {
      const i = part.indexOf('=');
      if (i > 0) out[part.slice(0, i)] = num(part.slice(i + 1));
    });
    return out;
  }
  const depthOf = (zone) => (zone in DEPTH ? DEPTH[zone] : null);

  function classify(line) {
    const make = (type, d, stamped) => ({ type, raw: line, d: d || {}, t: stamped ? d.t_ms : null, tEst: !stamped });
    let m;
    if (line.startsWith('RSSI_RAW,')) return make('rssi_raw', kv(line), true);
    if (line.startsWith('LINE_POINT,')) return make('line_point', kv(line), true);
    if (line.startsWith('RSSI_SAMPLE,')) return make('rssi_sample', kv(line));
    if (line.startsWith('FOLLOW_POINT,')) return make('follow_point', kv(line));
    if (line.startsWith('DIP_CHECK,')) return make('dip_check', kv(line));
    if (line.startsWith('COLOUR_SCAN_RESULT,')) return make('colour_scan_result', kv(line));
    if (line.startsWith('COLOUR_SCAN,')) {
      const d = kv(line);
      d.depth = typeof d.depth === 'number' ? d.depth : depthOf(d.zone);
      return make('colour_scan', d);
    }
    if (line.startsWith('COLOUR,')) {
      const d = kv(line);
      d.zoneList = String(d.zones || '').split('/');
      d.depths = d.zoneList.map(depthOf);
      d.matList = d.mats ? String(d.mats).split('/') : [];
      return make('colour', d);
    }
    if (line.startsWith('MAT_STEP,')) return make('mat_step', kv(line));
    if ((m = /^DRIVE: (forwards|backwards), base duty (\d+)%, duration (\d+) ms, heading (-?[\d.]+)/.exec(line)))
      return make('drive', { dir: m[1] === 'forwards' ? 1 : -1, duty: +m[2], ms: +m[3], heading: +m[4] });
    if ((m = /^TURN (\w+): commanded (-?[\d.]+) degrees.*?elapsed (\d+) ms/.exec(line)))
      return make('turn', { status: m[1], deg: +m[2], ms: +m[3] });
    if ((m = /^DRIVE OK: (forwards|backwards), powered (d+) ms/.exec(line))) return make('drive_ok', { ms: +m[2] });
    if (line.startsWith('TURN')) return make('turn', { status: 'FAILED', deg: 0, ms: 0 });
    if ((m = /^AP\d BLE: filtered (-?\d+) dBm, need >= (-?\d+), close (\d+)\/(\d+)/.exec(line)))
      return make('ble', { dbm: +m[1], need: +m[2], close: +m[3], of: +m[4] });
    if (/^BLE-only check:/.test(line)) return make('ble_check');
    if (/^AP\d (GATT|PAYLOAD COMPLETE|payload)/.test(line)) return make('xfer');
    if (line.startsWith('FOLLOW: ')) return make('follow');
    if (line.startsWith('DIP: ')) return make('dip');
    if (line.startsWith('RETURN: ')) return make('return');
    if (line.startsWith('TRIAL')) return make('trial');
    if (line.startsWith('DIRECTION ')) return make('direction');
    if (line.startsWith('ROUTE: ')) return make('route');
    if (/^IMU (IO ERROR|RECOVERED|GAP)/.test(line)) return make('imu');
    if (line.startsWith('MAT: ')) return make('mat');
    if (line.startsWith('ARRIVAL: ')) return make('arrival');
    if (line.startsWith('STEP: ')) return make('step');
    if (line.startsWith('APPROACH: ')) return make('approach');
    if (line.startsWith('FAILED: ')) return make('failed');
    if ((m = /^FAULT MASK: (0x[0-9a-f]+)/i.exec(line))) return make('faultmask', { mask: m[1] });
    if ((m = /^FAULT: (\w+)/.exec(line))) return make('fault', { name: m[1] });
    if (line.startsWith('RESULT: ')) return make('result', { text: line.slice(8) });
    if ((m = /^Getting there \| (.*)$/.exec(line))) return make('banner', { text: m[1] });
    return null;
  }

  function assignTimes(events) {
    const stamped = [];
    events.forEach((e, i) => { if (!e.tEst && typeof e.t === 'number') stamped.push(i); });
    if (!stamped.length) return { t0: null, t1: null, tailMs: 0 };
    let last = -Infinity;
    stamped.forEach((i) => { if (events[i].t < last) events[i].t = last; last = events[i].t; });
    for (let k = 0; k < stamped.length; k++) {
      const a = stamped[k];
      const b = stamped[k + 1];
      const end = b === undefined ? events.length : b;
      for (let i = a + 1; i < end; i++) {
        events[i].t = b === undefined ? events[a].t
          : events[a].t + (events[b].t - events[a].t) * (i - a) / (b - a);
      }
    }
    // Nothing is stamped after the last Wi-Fi line, but the robot keeps working (colour scans,
    // steering). Advance a clock by each event's own known duration so the run does not end early.
    const lastStamped = stamped[stamped.length - 1];
    let clock = events[lastStamped].t;
    for (let i = lastStamped + 1; i < events.length; i++) {
      events[i].t = clock;
      clock += knownDurationMs(events[i]);
    }
    return { t0: events[stamped[0]].t, t1: clock, tailMs: clock - events[lastStamped].t };
  }

  // Time an event is known to take, used only past the last stamped line.
  function knownDurationMs(e) {
    switch (e.type) {
      case 'drive': case 'turn': return e.d.ms;
      case 'colour': return COLOUR_CHECK_MS;
      case 'colour_scan': return COLOUR_CHECK_MS + 200; // three readings and the 200 ms settle
      default: return 0;
    }
  }

  // Which part of the run a line announces. null leaves the current phase alone.
  function phaseOf(e, current) {
    const text = e.raw;
    switch (e.type) {
      case 'follow':
        if (/colour sensor sees the mat/.test(text)) return 'mat';
        if (/decline position|decline persisted/.test(text)) return 'dip';
        if (/^FOLLOW: heading/.test(text)) return 'follow';
        return null;
      case 'dip':
        if (/recovery confirmed/.test(text)) return 'follow';
        if (/strengthened/.test(text)) return null;
        return 'dip';
      case 'dip_check': return 'dip';
      case 'return': return 'return';
      case 'trial': return 'search';
      case 'direction': return 'search';
      case 'route': return null;
      case 'mat': case 'mat_step': return 'mat';
      case 'approach': return /resuming|follow current/.test(text) ? 'follow' : null;
      case 'arrival': return /confirms/.test(text) ? 'transfer' : null;
      case 'xfer': return /PAYLOAD COMPLETE/.test(text) ? 'transfer' : null;
      case 'colour_scan': case 'colour_scan_result': return current === 'mat' ? null : 'scan';
      default: return null;
    }
  }

  function median(values) {
    if (!values.length) return null;
    const s = values.slice().sort((a, b) => a - b);
    const mid = s.length >> 1;
    return s.length % 2 ? s[mid] : (s[mid - 1] + s[mid]) / 2;
  }
  const sum = (arr, f) => arr.reduce((acc, x) => acc + f(x), 0);

  function parse(text, meta) {
    meta = meta || {};
    const events = [];
    text.split(/\r?\n/).forEach((raw) => {
      const line = raw.replace(/\s+$/, '');
      if (!line) return;
      const e = classify(line);
      if (!e) return;
      // "DRIVE OK" reports the time the motors really ran; prefer it to the commanded time.
      if (e.type === 'drive_ok') {
        const prev = events[events.length - 1];
        const drive = prev && prev.type === 'drive' ? prev : events.slice().reverse().find((x) => x.type === 'drive');
        if (drive && drive.d.commanded === undefined) { drive.d.commanded = drive.d.ms; drive.d.ms = e.d.ms; }
        return;
      }
      e.i = events.length;
      events.push(e);
    });
    const { t0, t1, tailMs } = assignTimes(events);
    const timed = t0 !== null && t1 > t0;
    const of = (type) => events.filter((e) => e.type === type);

    const drives = of('drive');
    const turns = of('turn');
    const samples = of('rssi_sample');
    const linePoints = of('line_point');
    const colour = of('colour');
    const scanLines = of('colour_scan');

    // ---- time use ----
    const driveMs = sum(drives, (e) => e.d.ms);
    const turnMs = sum(turns, (e) => e.d.ms);
    const scanMs = sum(samples, (e) => Number(e.d.duration_ms) || 0);
    const colourMs = colour.length * COLOUR_CHECK_MS;
    const wallMs = timed ? t1 - t0 : 0;
    const otherMs = Math.max(0, wallMs - driveMs - turnMs - scanMs - colourMs);

    // ---- per stop ----
    const stops = linePoints.map((e, k) => {
      const prev = linePoints[k - 1];
      const sameLine = prev && prev.d.line === e.d.line;
      return {
        n: k + 1, line: e.d.line, t: e.t, heading: e.d.heading, powered: e.d.powered_ms,
        dbm1: e.d.dbm1, dbm2: e.d.dbm2,
        wallMs: prev ? e.t - prev.t : null,
        movedMs: sameLine ? e.d.powered_ms - prev.d.powered_ms : null,
      };
    });
    const stepStops = stops.filter((s) => s.wallMs !== null && s.movedMs > 0);

    // ---- phases ----
    const phases = [];
    if (timed) {
      let current = 'start';
      let from = t0;
      events.forEach((e) => {
        if (e.t === null) return;
        const next = phaseOf(e, current);
        if (next && next !== current) {
          if (e.t > from) phases.push({ phase: current, t0: from, t1: e.t });
          current = next;
          from = e.t;
        }
      });
      phases.push({ phase: current, t0: from, t1 });
    }

    // ---- dead-reckoned path: heading is relative to the start, distance is powered ms ----
    const segs = [];
    const pathStops = [];
    let x = 0;
    let y = 0;
    events.forEach((e) => {
      if (e.type === 'drive') {
        const r = (e.d.heading * Math.PI) / 180;
        const nx = x + e.d.dir * e.d.ms * Math.sin(r);
        const ny = y + e.d.dir * e.d.ms * Math.cos(r);
        segs.push({ x0: x, y0: y, x1: nx, y1: ny, dir: e.d.dir, t: e.t });
        x = nx;
        y = ny;
      } else if (e.type === 'line_point') {
        pathStops.push({ x, y, t: e.t, dbm: (e.d.dbm1 + e.d.dbm2) / 2, n: pathStops.length + 1 });
      }
    });

    // ---- colour scans: a new scan starts when the angle goes back to the start ----
    const scans = [];
    let lastAngle = Infinity;
    events.forEach((e) => {
      if (e.type === 'colour_scan') {
        if (e.d.angle <= lastAngle) scans.push({ round: e.d.round, readings: [], result: null });
        lastAngle = e.d.angle;
        scans[scans.length - 1].readings.push(e.d);
      } else if (e.type === 'colour_scan_result' && scans.length) {
        scans[scans.length - 1].result = e.d;
      }
    });

    const resultEv = of('result').pop();
    const result = resultEv ? resultEv.d.text : null;
    const resultKind = !result ? 'none' : /CONFIRMED|COMPLETE/.test(result) && !/FAILED/.test(result) ? 'ok' : 'fail';
    const banner = (of('banner')[0] || { d: {} }).d.text || '';
    const date = (/^(\d{4}-\d{2}-\d{2})/.exec(meta.name || '') || [])[1] || '';

    return {
      name: meta.name || '', title: meta.title || meta.name || 'Run', date, banner,
      events, timed, t0, t1, tailMs,
      result, resultKind,
      faults: of('fault').map((e) => e.d.name),
      failure: (() => { const f = of('failed')[0]; return f ? { text: f.raw.slice(8), t: f.t } : null; })(),
      stats: {
        wallMs, startupMs: timed ? t0 : null, driveMs, turnMs, scanMs, colourMs, otherMs,
        driveShare: wallMs ? driveMs / wallMs : null,
        stops: stops.length, blocks: drives.length,
        medianStepWallMs: median(stepStops.map((s) => s.wallMs)),
        medianStepMovedMs: median(stepStops.map((s) => s.movedMs)),
        dipChecks: of('dip_check').length,
        bleChecks: of('ble_check').length,
        bleReadings: of('ble').length,
        directionSearches: of('direction').filter((e) => /^DIRECTION SEARCH/.test(e.raw)).length,
        imuFaults: of('imu').filter((e) => /IO ERROR/.test(e.raw)).length,
        returns: of('return').length,
        turns: turns.length,
        colourChecks: colour.length,
        matScans: scans.length,
        wifiScans: of('rssi_raw').length,
      },
      stops, phases, segs, pathStops, scans,
    };
  }

  return { parse, DEPTH_LABEL, COLOUR_CHECK_MS };
});
