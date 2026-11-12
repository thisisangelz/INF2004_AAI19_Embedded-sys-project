// Embeds every saved run log in docs/runs into docs/assets/data.js so the
// dashboard opens straight from disk (a page opened from a file cannot read
// other files) and on GitHub Pages. The data is compressed and encrypted with
// a key derived from the dashboard password, so the page carries neither the
// password nor readable run data. Re-run after adding a log:
//   DASHBOARD_PASSWORD=... node docs/tools/build_runs.js
'use strict';
const crypto = require('crypto');
const fs = require('fs');
const path = require('path');
const zlib = require('zlib');

const password = process.env.DASHBOARD_PASSWORD;
if (!password) {
  console.error('Set DASHBOARD_PASSWORD to the dashboard password, for example:\n  DASHBOARD_PASSWORD=... node docs/tools/build_runs.js');
  process.exit(1);
}
// Must match assets/gate.js.
const ITERATIONS = 250000;

const runsDir = path.join(__dirname, '..', 'runs');
const files = fs.readdirSync(runsDir);

// Run notes (.md) name their log; use the note's heading as the title.
const titleOf = {};
const noteOf = {};
files.filter((f) => f.endsWith('.md')).forEach((md) => {
  const text = fs.readFileSync(path.join(runsDir, md), 'utf8');
  const heading = (/^#\s+(.+)$/m.exec(text) || [])[1];
  if (!heading) return;
  for (const m of text.matchAll(/\(([^)\s]+\.txt)\)/g)) { titleOf[m[1]] = heading.trim(); noteOf[m[1]] = md; }
});

const pretty = (name) => name.replace(/^\d{4}-\d{2}-\d{2}-/, '').replace(/\.txt$/, '').replace(/-/g, ' ');
const logs = files.filter((f) => f.endsWith('.txt')).sort();
const shared = {};
logs.forEach((f) => { if (titleOf[f]) shared[titleOf[f]] = (shared[titleOf[f]] || 0) + 1; });

const runs = logs.map((name) => {
  let title = titleOf[name] || pretty(name);
  if (titleOf[name] && shared[title] > 1) title += ' · ' + pretty(name);
  return { name, title, note: noteOf[name] || null, text: fs.readFileSync(path.join(runsDir, name), 'utf8') };
});

// Field positions entered on the dashboard and exported to runs/layouts.json.
const layoutsFile = path.join(runsDir, 'layouts.json');
const layouts = fs.existsSync(layoutsFile) ? JSON.parse(fs.readFileSync(layoutsFile, 'utf8')) : { settings: {}, runs: {} };
const unknown = Object.keys(layouts.runs || {}).filter((name) => !logs.includes(name));
if (unknown.length) console.warn('layouts.json names logs that are not in runs/: ' + unknown.join(', '));

// gzip, then AES-256-GCM with a PBKDF2-SHA256 key. WebCrypto expects the
// 16-byte tag after the ciphertext.
const salt = crypto.randomBytes(16);
const iv = crypto.randomBytes(12);
const key = crypto.pbkdf2Sync(password, salt, ITERATIONS, 32, 'sha256');
const cipher = crypto.createCipheriv('aes-256-gcm', key, iv);
const plain = zlib.gzipSync(JSON.stringify({ runs, layouts }), { level: 9 });
const sealed = Buffer.concat([cipher.update(plain), cipher.final(), cipher.getAuthTag()]);
const data = { v: 1, iterations: ITERATIONS, salt: salt.toString('base64'), iv: iv.toString('base64'), data: sealed.toString('base64') };
fs.writeFileSync(path.join(__dirname, '..', 'assets', 'data.js'), 'window.DASHBOARD_DATA = ' + JSON.stringify(data) + ';\n');
console.log('encrypted ' + runs.length + ' logs (' + Math.round(sealed.length / 1024) + ' KiB), field positions for ' + Object.keys(layouts.runs || {}).length);
