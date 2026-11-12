// Checks the parser against every saved log:  node docs/tools/test_logparser.js
'use strict';
const assert = require('assert');
const fs = require('fs');
const path = require('path');
const { parse } = require('../assets/logparser.js');

const dir = path.join(__dirname, '..', 'runs');
const logs = fs.readdirSync(dir).filter((f) => f.endsWith('.txt')).sort();
assert(logs.length > 0, 'no logs found');

const rows = [];
for (const name of logs) {
  const text = fs.readFileSync(path.join(dir, name), 'utf8');
  const run = parse(text, { name });

  assert(run.banner, name + ': firmware banner line not found');
  assert(run.result, name + ': RESULT line not found');
  assert(run.events.length > 20, name + ': too few parsed events');

  // Stamped lines never go backwards, and interpolated lines stay inside the run.
  let last = -Infinity;
  run.events.forEach((e) => {
    if (e.t === null) return;
    assert(e.t >= last, name + ': time went backwards at event ' + e.i);
    last = e.t;
  });

  if (run.timed) {
    const s = run.stats;
    assert(s.wallMs > 0, name + ': empty timeline');
    assert(s.driveMs <= s.wallMs, name + ': more driving than elapsed time');
    assert(s.otherMs >= 0, name + ': negative remainder');
    const parts = s.driveMs + s.turnMs + s.scanMs + s.colourMs;
    assert(parts <= s.wallMs * 1.02, name + ': time use adds up to ' + (parts / s.wallMs * 100).toFixed(0) + '% of the run');
    assert(run.phases.length > 0, name + ': no phases');
    const covered = run.phases.reduce((a, p) => a + (p.t1 - p.t0), 0);
    assert(Math.abs(covered - s.wallMs) < 1, name + ': phases do not cover the run');
  }

  const s = run.stats;
  rows.push([
    name.replace(/^2026-/, '').replace(/\.txt$/, '').slice(0, 38).padEnd(38),
    run.timed ? (s.wallMs / 1000).toFixed(0).padStart(5) + ' s' : '    n/a',
    (s.driveMs / 1000).toFixed(1).padStart(6) + ' s',
    s.driveShare === null ? '   n/a' : (s.driveShare * 100).toFixed(1).padStart(5) + '%',
    String(s.stops).padStart(3),
    s.medianStepWallMs === null ? '   n/a' : (s.medianStepWallMs / 1000).toFixed(1).padStart(5) + ' s',
    run.resultKind.padEnd(4),
  ].join('  '));
}
console.log('run'.padEnd(38) + '   total   moving  share  stops  s/step  result');
rows.forEach((r) => console.log(r));
console.log('\nall ' + logs.length + ' logs parsed');

// Field map: placing AP1 straight ahead of the start on a run that drove
// straight to AP1 must calibrate with no leftover error, and every saved log
// must map to finite positions.
const FieldMap = require('../assets/fieldmap.js');
const straight = parse(fs.readFileSync(path.join(dir, '2026-11-12-ap1-ap2-route-ap1-done-ap2-wifi-lost.txt'), 'utf8'), { name: 'x' });
const layout = { start: { x: 150, y: 30, bearing: 0 }, ap1: { x: 150, y: 270 }, ap2: { x: 270, y: 150 } };
const fit = FieldMap.fitRun(straight, layout, 1);
assert(fit && fit.speed > 0 && fit.errorCm < 1, 'field map: straight AP1 approach should fit exactly');
const onField = FieldMap.mapRun(straight, layout, { speedCmS: fit.speed, headingSign: 1, reanchor: true });
assert.strictEqual(onField.arrivals.length, 1, 'field map: one AP1 arrival');
assert(Math.abs(onField.arrivals[0].drivenCm - 240) < 1, 'field map: AP1 reached after 240 cm');
const legs = FieldMap.metrics(straight, layout, onField).legs;
assert(legs[0].reached && !legs[1].reached, 'field map: leg 1 reached, leg 2 not');
for (const name of logs) {
  const run = parse(fs.readFileSync(path.join(dir, name), 'utf8'), { name });
  const m = FieldMap.mapRun(run, layout, { speedCmS: 20, headingSign: -1, reanchor: true });
  assert(m.trail.every((q) => isFinite(q.x) && isFinite(q.y)), name + ': field map produced a non-finite point');
}
console.log('field map checks passed');
