// Places a parsed run on the test field: a grid of square floor tiles with a
// known start pose and beacon positions. Runs in the browser and in Node.
//
// Field coordinates are centimetres from the bottom-left corner of the grid:
// x grows to the right (columns A, B, C…), y grows up (rows 1, 2, 3…).
// Bearings are compass-style on the grid: 0 = up (towards the last row),
// 90 = right. The log's headings are relative to where the robot faced at the
// start, so the start bearing is needed to turn them into field bearings.
//
// The log has no position sensor. Distance comes from motor time multiplied by
// a drive speed, and that speed is calibrated from runs that reached AP1.
(function (root, factory) {
  if (typeof module === 'object' && module.exports) module.exports = factory();
  else root.FieldMap = factory();
}(typeof self !== 'undefined' ? self : this, function () {
  'use strict';

  const GRID = { cols: 5, rows: 5, tileCm: 60 };
  const SNAP_CM = 15;
  const DEFAULT_SPEED_CM_S = 20;
  // A positive motor turn drives the left wheel back and the right wheel
  // forward (motion.c), so a positive log heading is anticlockwise from above.
  const DEFAULT_HEADING_SIGN = 1;

  const sizeCm = () => ({ w: GRID.cols * GRID.tileCm, h: GRID.rows * GRID.tileCm });
  const snap = (v) => Math.round(v / SNAP_CM) * SNAP_CM;
  const dist = (a, b) => Math.hypot(a.x - b.x, a.y - b.y);

  // Tile name such as "C4". A point on a tile edge counts as the tile up and to the right of it.
  function tileName(p) {
    if (!p) return '';
    const c = Math.min(GRID.cols - 1, Math.max(0, Math.floor(p.x / GRID.tileCm)));
    const r = Math.min(GRID.rows - 1, Math.max(0, Math.floor(p.y / GRID.tileCm)));
    return String.fromCharCode(65 + c) + (r + 1);
  }

  const complete = (layout) => !!(layout && layout.start && typeof layout.start.bearing === 'number');

  // Log heading (relative, signed) to field bearing.
  const bearingOf = (layout, heading, sign) => layout.start.bearing - sign * heading;

  // Walks the run's drives on the field.
  // opts.speedCmS: centimetres per second of motor time.
  // opts.headingSign: +1 when a positive log heading is anticlockwise.
  // opts.reanchor: snap the path onto a beacon once its payload completed.
  function mapRun(run, layout, opts) {
    if (!complete(layout)) return null;
    const speed = opts.speedCmS;
    const sign = opts.headingSign;
    let pos = { x: layout.start.x, y: layout.start.y };
    let drivenCm = 0;
    let leg = 1;
    const segs = [];
    const stops = [];
    const arrivals = [];
    const trail = [{ x: pos.x, y: pos.y, t: run.t0, leg }];
    run.events.forEach((e) => {
      if (e.type === 'drive') {
        const b = (bearingOf(layout, e.d.heading, sign) * Math.PI) / 180;
        const cm = (e.d.ms / 1000) * speed;
        const next = { x: pos.x + e.d.dir * cm * Math.sin(b), y: pos.y + e.d.dir * cm * Math.cos(b) };
        segs.push({ x0: pos.x, y0: pos.y, x1: next.x, y1: next.y, dir: e.d.dir, leg, t: e.t });
        drivenCm += cm;
        pos = next;
        trail.push({ x: pos.x, y: pos.y, t: e.t, leg });
      } else if (e.type === 'line_point') {
        stops.push({ x: pos.x, y: pos.y, t: e.t, leg, dbm: (e.d.dbm1 + e.d.dbm2) / 2 });
      } else if (e.type === 'xfer') {
        const m = /^AP(\d) PAYLOAD COMPLETE/.exec(e.raw);
        if (!m) return;
        const ap = layout['ap' + m[1]];
        const at = { x: pos.x, y: pos.y };
        const arrival = { ap: +m[1], t: e.t, at, drivenCm, errorCm: ap ? dist(at, ap) : null, anchored: false };
        if (ap && opts.reanchor) {
          pos = { x: ap.x, y: ap.y };
          arrival.anchored = true;
          trail.push({ x: pos.x, y: pos.y, t: e.t, leg, jump: true });
        }
        arrivals.push(arrival);
        leg += 1;
      }
    });
    return { segs, stops, arrivals, trail, end: pos, drivenCm };
  }

  // Speed that best puts the dead-reckoned AP1 arrival on AP1 (least squares
  // along the travelled direction), and the distance still left over.
  function fitRun(run, layout, headingSign) {
    if (!complete(layout) || !layout.ap1) return null;
    const unit = mapRun(run, layout, { speedCmS: 1, headingSign, reanchor: false });
    const arrival = unit && unit.arrivals.find((a) => a.ap === 1);
    if (!arrival) return null;
    const v = { x: arrival.at.x - layout.start.x, y: arrival.at.y - layout.start.y };
    const d = { x: layout.ap1.x - layout.start.x, y: layout.ap1.y - layout.start.y };
    const vv = v.x * v.x + v.y * v.y;
    if (vv === 0) return null;
    const speed = (v.x * d.x + v.y * d.y) / vv;
    if (!(speed > 0)) return { speed: null, errorCm: Math.hypot(d.x, d.y) };
    return { speed, errorCm: Math.hypot(speed * v.x - d.x, speed * v.y - d.y) };
  }

  const median = (xs) => {
    if (!xs.length) return null;
    const s = xs.slice().sort((a, b) => a - b);
    const m = s.length >> 1;
    return s.length % 2 ? s[m] : (s[m - 1] + s[m]) / 2;
  };

  // Calibration across every run with a full layout that reached AP1.
  function calibrate(runs, layoutOf, headingSign) {
    const fits = [];
    runs.forEach((run) => {
      const f = fitRun(run, layoutOf(run), headingSign);
      if (f && f.speed) fits.push({ name: run.name, title: run.title, speed: f.speed, errorCm: f.errorCm });
    });
    return { fits, speed: median(fits.map((f) => f.speed)), errorCm: median(fits.map((f) => f.errorCm)) };
  }

  // Distances along a mapped path to one point, for one leg.
  function closest(mapped, p, leg) {
    let best = null;
    const pts = mapped.trail.filter((q) => q.leg === leg);
    const consider = (q) => { const d = dist(q, p); if (best === null || d < best) best = d; };
    pts.forEach(consider);
    // Also check along each segment, not just its ends.
    mapped.segs.filter((s) => s.leg === leg).forEach((s) => {
      const vx = s.x1 - s.x0; const vy = s.y1 - s.y0;
      const L = vx * vx + vy * vy;
      if (!L) return;
      const k = Math.max(0, Math.min(1, ((p.x - s.x0) * vx + (p.y - s.y0) * vy) / L));
      consider({ x: s.x0 + k * vx, y: s.y0 + k * vy });
    });
    return best;
  }

  // Per-run numbers for spotting gaps across many runs.
  function metrics(run, layout, mapped) {
    const out = { startToAp1Cm: null, ap1ToAp2Cm: null, legs: [] };
    if (!layout) return out;
    if (layout.start && layout.ap1) out.startToAp1Cm = dist(layout.start, layout.ap1);
    if (layout.ap1 && layout.ap2) out.ap1ToAp2Cm = dist(layout.ap1, layout.ap2);
    if (!mapped) return out;
    [1, 2].forEach((leg) => {
      const ap = layout['ap' + leg];
      const from = leg === 1 ? layout.start : layout.ap1;
      const tried = mapped.trail.some((q) => q.leg === leg);
      if (!ap || !from || !tried) return;
      const arrival = mapped.arrivals.find((a) => a.ap === leg);
      const prev = mapped.arrivals.find((a) => a.ap === leg - 1);
      const legStartT = prev ? prev.t : run.t0;
      const legStartCm = prev ? prev.drivenCm : 0;
      const legEnd = mapped.trail.filter((q) => q.leg === leg).pop();
      const straight = dist(from, ap);
      const driven = (arrival ? arrival.drivenCm : mapped.drivenCm) - legStartCm;
      out.legs.push({
        leg, straightCm: straight,
        reached: !!arrival,
        timeMs: arrival && legStartT !== null && arrival.t !== null ? arrival.t - legStartT : null,
        drivenCm: driven,
        efficiency: arrival && driven > 0 ? straight / driven : null,
        closestCm: closest(mapped, ap, leg),
        endCm: arrival ? 0 : dist(legEnd, ap),
      });
    });
    return out;
  }

  // Runs whose beacons sit in the same places can be overlaid on one map.
  const setupKey = (layout) => (layout && layout.ap1 ? 'AP1 ' + tileName(layout.ap1) + ' (' + layout.ap1.x + ', ' + layout.ap1.y + ')' +
    (layout.ap2 ? ' · AP2 ' + tileName(layout.ap2) + ' (' + layout.ap2.x + ', ' + layout.ap2.y + ')' : '') : null);

  return { GRID, SNAP_CM, DEFAULT_SPEED_CM_S, DEFAULT_HEADING_SIGN, sizeCm, snap, dist, tileName, complete, mapRun, fitRun, calibrate, metrics, setupKey };
}));
