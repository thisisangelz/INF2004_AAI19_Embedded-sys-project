// Checks the parser against every saved log:  node dashboard/test_logparser.js
'use strict';
const assert = require('assert');
const fs = require('fs');
const path = require('path');
const { parse } = require('./logparser.js');

const dir = path.join(__dirname, '..', 'docs', 'runs');
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
