// Embeds every saved run log in docs/runs into dashboard/runs.js so the
// dashboard opens straight from disk (a page opened from a file cannot read
// other files). Re-run after adding a log:  node dashboard/build_runs.js
'use strict';
const fs = require('fs');
const path = require('path');

const runsDir = path.join(__dirname, '..', 'docs', 'runs');
const files = fs.readdirSync(runsDir);

// Run notes (.md) name their log; use the note's heading as the title.
const titleOf = {};
files.filter((f) => f.endsWith('.md')).forEach((md) => {
  const text = fs.readFileSync(path.join(runsDir, md), 'utf8');
  const heading = (/^#\s+(.+)$/m.exec(text) || [])[1];
  if (!heading) return;
  for (const m of text.matchAll(/\(([^)\s]+\.txt)\)/g)) titleOf[m[1]] = heading.trim();
});

const pretty = (name) => name.replace(/^\d{4}-\d{2}-\d{2}-/, '').replace(/\.txt$/, '').replace(/-/g, ' ');
const logs = files.filter((f) => f.endsWith('.txt')).sort();
const shared = {};
logs.forEach((f) => { if (titleOf[f]) shared[titleOf[f]] = (shared[titleOf[f]] || 0) + 1; });

const runs = logs.map((name) => {
  let title = titleOf[name] || pretty(name);
  if (titleOf[name] && shared[title] > 1) title += ' · ' + pretty(name);
  return { name, title, text: fs.readFileSync(path.join(runsDir, name), 'utf8') };
});

fs.writeFileSync(path.join(__dirname, 'runs.js'), 'window.RUNS = ' + JSON.stringify(runs) + ';\n');
console.log('embedded ' + runs.length + ' logs');
