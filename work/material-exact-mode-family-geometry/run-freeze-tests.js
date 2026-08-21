'use strict';

const assert = require('node:assert/strict');
const childProcess = require('node:child_process');
const crypto = require('node:crypto');
const fs = require('node:fs');
const path = require('node:path');

const workspace = path.resolve(__dirname, '../..');
const suites = [
  ['finiteRotation', 'work/material-finite-rotation-sweep-oracle/run-tests.js'],
  ['modeFamily', 'work/material-kkt-mode-family-geometry/run-tests.js'],
  ['certifiedRootV3', 'work/material-prepared-root-integration/run-v3-tests.js'],
  ['exactArea', 'work/material-exact-triangle-sweep-area/run-tests.js'],
  ['exactUnionRandom', 'work/material-exact-triangle-sweep-area/run-union-random-audit.js'],
  ['exactModeFamily', 'work/material-exact-mode-family-geometry/run-tests.js'],
];
const sources = [
  'work/material-finite-rotation-sweep-oracle/finite-rotation-sweep-oracle.js',
  'work/material-kkt-mode-family-geometry/mode-family-geometry.js',
  'work/material-exact-triangle-sweep-area/exact-triangle-sweep-area.js',
  'work/material-exact-mode-family-geometry/exact-mode-family-geometry.js',
  'work/material-prepared-root-integration/event-kkt-coulomb-v3.js',
];

function sha256(relativePath) {
  return crypto.createHash('sha256').update(fs.readFileSync(path.join(workspace, relativePath)))
    .digest('hex').toUpperCase();
}

const results = {};
for (const [name, relativeRunner] of suites) {
  const execution = childProcess.spawnSync(process.execPath, [path.join(workspace, relativeRunner)], {
    cwd: workspace, encoding: 'utf8', maxBuffer: 16 * 1024 * 1024,
  });
  assert.equal(execution.status, 0, `${name} failed:\n${execution.stdout}\n${execution.stderr}`);
  const parsed = JSON.parse(execution.stdout);
  assert.equal(parsed.pass === true || parsed.ok === true, true, `${name} did not report pass`);
  results[name] = parsed;
}

const report = {
  schema: 'exact-finite-rotation-freeze-tests-v1',
  pass: true,
  sourceHashes: Object.fromEntries(sources.map((entry) => [entry, sha256(entry)])),
  runnerHashes: Object.fromEntries(suites.map(([, entry]) => [entry, sha256(entry)])),
  suites: results,
  scope: 'work-only pure math and prepared-root integration; no Sfree, TOI, path owner, main, or removal authority',
};
const serialized = `${JSON.stringify(report, null, 2)}\n`;
fs.writeFileSync(path.join(__dirname, 'freeze-test-report.json'), serialized);
process.stdout.write(serialized);
