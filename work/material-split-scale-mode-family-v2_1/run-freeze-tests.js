'use strict';

const assert = require('node:assert/strict');
const childProcess = require('node:child_process');
const crypto = require('node:crypto');
const fs = require('node:fs');
const path = require('node:path');

const workspace = path.resolve(__dirname, '../..');
const relative = (entry) => path.join(workspace, entry);
const sha256 = (entry) => crypto.createHash('sha256').update(fs.readFileSync(relative(entry)))
  .digest('hex').toUpperCase();
function run(relativeRunner, args = [], expectedSuccess = true) {
  const execution = childProcess.spawnSync(process.execPath, [relative(relativeRunner), ...args], {
    cwd: workspace, encoding: 'utf8', maxBuffer: 32 * 1024 * 1024,
  });
  if (expectedSuccess) assert.equal(execution.status, 0,
    `${relativeRunner} failed:\n${execution.stdout}\n${execution.stderr}`);
  else assert.notEqual(execution.status, 0, `${relativeRunner} unexpectedly passed`);
  return execution;
}
function parsed(relativeRunner, args = []) {
  const execution = run(relativeRunner, args);
  const value = JSON.parse(execution.stdout);
  assert.equal(value.pass === true || value.ok === true, true,
    `${relativeRunner} did not report PASS`);
  return value;
}

const focused = parsed('work/material-split-scale-mode-family-v2_1/run-split-scale-tests.js');
const realThreeMode = parsed('work/material-split-scale-mode-family-v2_1/run-real-s4d-v2_1-fixture.js');
const randomSubset = parsed('work/material-split-scale-mode-family-v2_1/run-random-audit.js', [
  '--quick', '--width=1e-8',
  '--report=work/material-split-scale-mode-family-v2_1/random-subset-report.json',
]);
assert.equal(realThreeMode.signatureCounts.canonicalGeometrySignature, 1);
assert.equal(realThreeMode.signatureCounts.commonNumericalPolicySignature, 1);
assert.equal(realThreeMode.signatureCounts.compositeGeometrySnapshotSignature, 1);
assert.equal(realThreeMode.signatureCounts.pathSignature, 3);
assert.equal(realThreeMode.scope.trigProviderCertified, false);
assert.equal(realThreeMode.scope.productionWiring, false);

const inheritedPreparedRoot = run(
  'work/material-split-scale-mode-family-v2_1/run-tests.js', [], false,
);
const preparedRootCombinedOutput = `${inheritedPreparedRoot.stdout}\n${inheritedPreparedRoot.stderr}`;
assert.match(preparedRootCombinedOutput,
  /least-root interval proof is unresolved: an interval contains zero but cannot be refined to a bound endpoint sample/);

const frozenSuites = {
  finiteRotation: parsed('work/material-finite-rotation-sweep-oracle/run-tests.js'),
  modeFamily: parsed('work/material-kkt-mode-family-geometry/run-tests.js'),
  exactArea: parsed('work/material-exact-triangle-sweep-area/run-tests.js'),
  exactUnionRandom: parsed('work/material-exact-triangle-sweep-area/run-union-random-audit.js'),
  exactModeFamily: parsed('work/material-exact-mode-family-geometry/run-tests.js'),
  preparedRootV3: parsed('work/material-prepared-root-integration/run-v3-tests.js'),
};

const expectedUnchanged = {
  'outputs/bite-simulator/app.js':
    '3A58BFB55ACA173DB63435F75C7FDF19BB0F6D68D42AEABF4E77FC1A7AB2D344',
  'work/s4c-private-toi-dxf-combined/app-combined-candidate.js':
    '06B6EC1F6E611DDD4122652DC9763D8F75129904D224ACCA3CABB1AF3D71D0AA',
  'work/material-finite-rotation-sweep-oracle/finite-rotation-sweep-oracle.js':
    '9F37F457CD1F5DE6FB76680E136E3FC80900848C79A5E8B3A041D831BCF3D3CF',
  'work/material-kkt-mode-family-geometry/mode-family-geometry.js':
    'EDD725432EB01E0A800647A68A6B67E74B4F2AA27A134805FAC21B2D46519EB8',
  'work/material-exact-triangle-sweep-area/exact-triangle-sweep-area.js':
    'BC128BAFC05397B073584D5FAF4362A7AD3F09CD91EBBFF173886AD84B43A77B',
  'work/material-exact-mode-family-geometry/exact-mode-family-geometry.js':
    '59E0567C0C1545A79A7759BF3379953F9A87159F619C79F8A09BA1256F45A7B0',
  'work/material-prepared-root-integration/event-kkt-coulomb-v3.js':
    '385013DF70B3D40B503C9043148F573DCCB4D6EA01ECD51F040F052F5F52C447',
};
for (const [entry, expected] of Object.entries(expectedUnchanged)) {
  assert.equal(sha256(entry), expected, `${entry} changed`);
}

const sources = [
  'work/material-split-scale-mode-family-v2_1/finite-rotation-sweep-oracle-v2_1.js',
  'work/material-split-scale-mode-family-v2_1/mode-family-geometry-v2_1.js',
  'work/material-split-scale-mode-family-v2_1/exact-mode-family-geometry-v2_1.js',
  'work/material-split-scale-mode-family-v2_1/real-s4d-three-mode-fixture.json',
];
const runners = [
  'work/material-split-scale-mode-family-v2_1/run-split-scale-tests.js',
  'work/material-split-scale-mode-family-v2_1/run-real-s4d-v2_1-fixture.js',
  'work/material-split-scale-mode-family-v2_1/run-random-audit.js',
  'work/material-split-scale-mode-family-v2_1/run-tests.js',
  'work/material-split-scale-mode-family-v2_1/run-freeze-tests.js',
];
const report = {
  schema: 'split-scale-mode-family-v2.1-freeze-test-report-v1',
  pass: true,
  sourceHashes: Object.fromEntries(sources.map((entry) => [entry, sha256(entry)])),
  runnerHashes: Object.fromEntries(runners.map((entry) => [entry, sha256(entry)])),
  focused: { pass: focused.pass, passed: focused.passed, total: focused.total },
  realThreeMode: {
    pass: realThreeMode.pass,
    status: realThreeMode.status,
    commonFinalCoordinateScaleUpperBound:
      realThreeMode.commonFinalBound,
    commonComputedFinalCoordinateScaleUpperBound:
      realThreeMode.commonComputedFinalBound,
    commonTransformOperandScaleUpperBound:
      realThreeMode.commonTransformBound,
    signatureCounts: realThreeMode.signatureCounts,
  },
  randomSubset,
  preparedRootConsumption: {
    pass: false,
    status: 'expected-domain-stop-not-authorized',
    reason:
      'least-root interval proof is unresolved under the nonzero twice-transform-roundoff interval radius floor',
    intervalBudgetWasNotRelaxed: true,
  },
  frozenRegression: Object.fromEntries(Object.entries(frozenSuites).map(([name, value]) => [
    name,
    { pass: value.pass === true || value.ok === true,
      passed: value.passed ?? value.cases ?? value.total ?? null },
  ])),
  unchangedHashes: expectedUnchanged,
  authority: {
    trigProviderCertified: false,
    productionNumericalAuthority: false,
    productionWiring: false,
    preparedRootAuthority: false,
    s4dReconnected: false,
    qWrites: 0,
    geometryCommits: 0,
    removals: 0,
    main: false,
  },
};
const serialized = `${JSON.stringify(report, null, 2)}\n`;
fs.writeFileSync(path.join(__dirname, 'freeze-test-report.json'), serialized);
process.stdout.write(serialized);
