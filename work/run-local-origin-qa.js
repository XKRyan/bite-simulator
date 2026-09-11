'use strict';

// Low-cost regression test for the Rapier local-coordinate representation.
// It serves the real prototype in a real browser; it never edits production.
const fs = require('fs');
const http = require('http');
const path = require('path');
const { spawn } = require('child_process');

const root = path.resolve(__dirname, '..');
const prototypePath = path.join(__dirname, 'material-continuous-prototype-app.js');
const formalPath = path.join(root, 'outputs', 'bite-simulator', 'app.js');
const prototype = fs.readFileSync(prototypePath, 'utf8');
const formalHash = require('crypto').createHash('sha256').update(fs.readFileSync(formalPath)).digest('hex').toUpperCase();
const expectedFormalHash = '89E73B62F395719A5C48589B20922E5D839938B97454B82817E3F459F178E4C0';

function esc(value) { return JSON.stringify(value).replace(/<\/script/gi, '<\\/script'); }
function sourceChecks() {
  const checks = [
    ['origin helper exists', /function rigSceneOriginX\([\s\S]*?sceneOriginX/.test(prototype)],
    ['snapshot stores origin', /function snapshotRapierRig[\s\S]*?sceneOriginX:\s*rigSceneOriginX/.test(prototype)],
    ['restore reads origin', /function restoreRapierRig[\s\S]*?sceneOriginX\s*=\s*Number\.isFinite\(snapshot\.sceneOriginX\)/.test(prototype)],
    ['target sync adds origin', /sim\.target\.pos\s*=\s*rigPointToScene\(targetPosition,\s*physics\)/.test(prototype)],
    ['contact display point converts to scene', /sim\.currentContact\s*=\s*\{[\s\S]*?point:\s*rigPointToScene\(strongestTool\.point,\s*physics\)/.test(prototype)],
  ];
  return checks.map(([name, pass]) => ({ name, pass }));
}

if (process.argv.includes('--static')) {
  const checks = sourceChecks();
  const report = { pass: formalHash === expectedFormalHash && checks.every(x => x.pass), formalHash, expectedFormalHash, checks };
  console.log(JSON.stringify(report, null, 2));
  process.exit(report.pass ? 0 : 1);
}

const originalIndex = fs.readFileSync(path.join(root, 'outputs', 'bite-simulator', 'index.html'), 'utf8');
const reportScript = `
<script>
(async () => {
  const out = document.createElement('pre'); out.id = 'out'; document.body.appendChild(out);
  const report = ${esc({ formalHash, expectedFormalHash, sourceChecks: sourceChecks() })};
  const wait = async (predicate, ms = 30000) => { const t = performance.now(); while (performance.now() - t < ms) { const v = predicate(); if (v) return v; await new Promise(r => setTimeout(r, 25)); } throw new Error('timeout'); };
  const near = (a, b, e = 2e-7) => Math.abs(a - b) <= e;
  try {
    const api = await wait(() => window.BiteSim);
    api.setParams({ paramWeaponEnabled: true, paramForkEnabled: true, paramToothCount: 1, targetSceneX: 380, simulationDuration: 0.02 });
    const initial = api.getState();
    const before = { target: initial.target.pos, fork: initial.fork.origin, weapon: initial.weaponScene };
    api.advance(8);
    const advanced = api.getState();
    const beforeSnapshot = { target: advanced.target.pos, fork: advanced.fork.origin, weapon: advanced.weaponScene };
    const roundTrip = api.snapshotRoundTrip();
    const afterSnapshot = api.getState();
    report.browser = {
      targetInitialX: initial.target.pos.x,
      targetInitialSync: near(initial.target.pos.x, 0.38),
      advancedTargetFinite: Number.isFinite(advanced.target.pos.x) && Number.isFinite(advanced.target.pos.y),
      roundTrip,
      snapshotPosePreserved: ['target', 'fork', 'weapon'].every(key => near(beforeSnapshot[key].x, afterSnapshot[key].x) && near(beforeSnapshot[key].y, afterSnapshot[key].y)),
      publicSceneCoordinatePreserved: near(before.target.x, 0.38),
    };
    report.pass = report.formalHash === report.expectedFormalHash
      && report.sourceChecks.every(x => x.pass)
      && report.browser.targetInitialSync
      && report.browser.advancedTargetFinite
      && report.browser.roundTrip.ok
      && report.browser.browserPosePreserved !== false
      && report.browser.snapshotPosePreserved
      && report.browser.publicSceneCoordinatePreserved;
  } catch (error) { report.pass = false; report.error = String(error && error.stack || error); }
  out.textContent = JSON.stringify(report);
})().catch(error => { const out = document.getElementById('out') || document.body.appendChild(document.createElement('pre')); out.id = 'out'; out.textContent = JSON.stringify({ pass: false, error: String(error && error.stack || error) }); });
</script>`;

// Keep the production HTML/asset graph, replacing only the served work copy.
const page = originalIndex.replace('app.js?v=1.0.2', '../../work/material-continuous-prototype-app.js')
  .replace('</body>', `${reportScript}</body>`);
const edge = ['C:\\Program Files (x86)\\Microsoft\\Edge\\Application\\msedge.exe', 'C:\\Program Files\\Microsoft\\Edge\\Application\\msedge.exe'].find(fs.existsSync);
if (!edge) throw new Error('Microsoft Edge not found');
const mime = { '.html': 'text/html; charset=utf-8', '.js': 'text/javascript; charset=utf-8', '.css': 'text/css; charset=utf-8', '.wasm': 'application/wasm' };
const server = http.createServer((req, res) => {
  if (req.url === '/' || req.url === '/index.html') { res.writeHead(200, { 'Content-Type': 'text/html; charset=utf-8', 'Cache-Control': 'no-store' }); res.end(page); return; }
  const target = path.resolve(root, decodeURIComponent(new URL(req.url, 'http://127.0.0.1').pathname).replace(/^\/+/, ''));
  if (!target.startsWith(root + path.sep)) { res.writeHead(403).end(); return; }
  fs.readFile(target, (error, data) => { if (error) { res.writeHead(404).end(); return; } res.writeHead(200, { 'Content-Type': mime[path.extname(target)] || 'application/octet-stream', 'Cache-Control': 'no-store' }); res.end(data); });
});
const profile = path.join(__dirname, `.edge-local-origin-${process.pid}-${Date.now()}`);
server.listen(0, '127.0.0.1', () => {
  const url = `http://127.0.0.1:${server.address().port}/`;
  const child = spawn(edge, ['--headless=new', '--disable-gpu', '--no-sandbox', '--disable-background-timer-throttling', '--disable-renderer-backgrounding', `--user-data-dir=${profile}`, '--virtual-time-budget=15000', '--dump-dom', url], { windowsHide: true });
  let stdout = ''; let stderr = '';
  child.stdout.on('data', chunk => { stdout += chunk; }); child.stderr.on('data', chunk => { stderr += chunk; });
  child.on('close', code => {
    server.close();
    const match = stdout.match(/<pre id="out">([\s\S]*?)<\/pre>/);
    if (!match) { console.error(JSON.stringify({ pass: false, code, error: 'no report', stderr: stderr.slice(-1000) }, null, 2)); process.exitCode = 2; return; }
    const report = JSON.parse(match[1].replaceAll('&quot;', '"').replaceAll('&amp;', '&').replaceAll('&lt;', '<').replaceAll('&gt;', '>'));
    console.log(JSON.stringify(report, null, 2));
    process.exitCode = report.pass ? 0 : 1;
  });
});
