'use strict';

const assert = require('node:assert/strict');
const crypto = require('node:crypto');
const fs = require('node:fs');
const path = require('node:path');
const subject = require('./mode-family-geometry.js');
const coulomb = require('../kkt-coulomb-extension/event-kkt-coulomb.js');

function hashFile(filename) {
  return crypto.createHash('sha256').update(fs.readFileSync(filename)).digest('hex').toUpperCase();
}
function identity(size) {
  return Array.from({ length: size }, (_, row) => Array.from({ length: size }, (_, column) => row === column ? 1 : 0));
}
function embed(row) { return [row[0], 0, 0, row[1], row[2], 0]; }
function kktInput() {
  return {
    qFree: [2, 300, 0, 1, 0.6, 0], Minv: identity(6),
    materialContact: { id: 'weapon-material', point: { x: 0.1, y: 0.2 },
      normalRow: embed([1, 2, 1]) },
    structuralContacts: [
      { id: 'fork-target', role: 'fork-target', point: { x: 1, y: 1 },
        normalRow: embed([0, 1, 0]), tangentRow: embed([1, 0, 0]), mu: 0.5, normalBias: 0 },
      { id: 'target-floor', role: 'target-floor', point: { x: 2, y: 0 },
        normalRow: embed([0, 0, 1]), tangentRow: embed([1, 0, 0]), mu: 0.3, normalBias: 0 },
    ],
    specificCuttingEnergy: 1, width: 1,
  };
}
function geometry(overrides = {}) {
  return {
    h: 2e-4, timeFractions: Array.from({ length: 9 }, (_, index) => index / 8),
    maximumTimeChordError: 5e-6, lengthTolerance: 1e-14,
    // A radial working segment keeps a pure-rotation time cell non-folding.
    workingSegment: { start: [0.004, 0], end: [0.006, 0] },
    featureStartPosition: [-0.001, 0], featureStartAngle: 0,
    materialStartPosition: [0, 0], materialStartAngle: 0,
    remainingGeometry: [[[[-0.01, -0.01], [0.01, -0.01], [0.01, 0.01],
      [-0.01, 0.01], [-0.01, -0.01]]]],
    ...overrides,
  };
}
function create(overrides = {}) {
  const result = subject.createModeFamilyGeometry({
    pDomain: [0, 2], kktInput: kktInput(), geometry: geometry(), ...overrides,
  });
  assert.equal(result.ok, true, result.reason); return result;
}
function privateSolve(p) {
  return coulomb.solvePrescribedImpulse({ ...kktInput(), freshArea: () => 0 }, p);
}

const checks = [];
function check(name, run) { const started = Date.now(); run(); checks.push({ name, pass: true, elapsedMs: Date.now() - started }); }

check('signed module pins and six-DOF order are exact', () => {
  assert.equal(subject.EXPECTED_COULOMB_SHA256,
    hashFile(path.resolve(__dirname, '../kkt-coulomb-extension/event-kkt-coulomb.js')));
  assert.equal(subject.EXPECTED_FINITE_ROTATION_SHA256,
    hashFile(path.resolve(__dirname, '../material-finite-rotation-sweep-oracle/finite-rotation-sweep-oracle.js')));
  const family = create();
  assert.deepEqual(family.descriptor.qOrder, ['rail-pivot-vx', 'weapon-omega', 'fork-omega',
    'target-com-vx', 'target-com-vy', 'target-omega']);
});

check('maximum-dissipation envelope has the four expected exact cells', () => {
  const family = create(); const envelope = family.descriptor.envelope;
  assert.deepEqual(envelope.map((entry) => [entry.start, entry.end, entry.modeKey]), [
    [0, 0.5, 'fork-target:inactive|target-floor:inactive'],
    [0.5, 0.6, 'fork-target:-slide|target-floor:inactive'],
    [0.6, 1.165217391304348, 'fork-target:-slide|target-floor:-slide'],
    [1.165217391304348, 2, 'fork-target:stick|target-floor:-slide'],
  ]);
  assert.equal(new Set(envelope.map((entry) => entry.pathSignature)).size, 4);
});

check('exact geometry binds the same mode and qPost at transitions', () => {
  const family = create();
  for (const p of [0, 0.25, 0.5, 0.55, 0.6, 0.8, 1.165217391304348, 1.5, 2]) {
    const solved = privateSolve(p); assert.equal(solved.ok, true, solved.reason);
    const sample = family.exactSample(p, solved);
    assert.equal(sample.ok, true, sample.reason);
    assert.equal(sample.modeKey, solved.modeKey); assert.deepEqual(sample.qPost, solved.qPost);
    assert.deepEqual(sample.payload.qPost, solved.qPost);
  }
});

check('forged endpoint trials cannot select geometry', () => {
  const family = create(); const solved = JSON.parse(JSON.stringify(privateSolve(0.75)));
  solved.qPost[0] += 1e-15;
  const qForgery = family.exactSample(0.75, solved);
  assert.equal(qForgery.ok, false); assert.match(qForgery.reason, /does not match/);
  const mode = JSON.parse(JSON.stringify(privateSolve(0.75))); mode.modeKey = 'forged-mode';
  const modeForgery = family.exactSample(0.75, mode);
  assert.equal(modeForgery.ok, false); assert.match(modeForgery.reason, /does not match/);
  const point = JSON.parse(JSON.stringify(privateSolve(0.75)));
  point.contactPointBindings[0].point.x += 1e-15;
  const pointForgery = family.exactSample(0.75, point);
  assert.equal(pointForgery.ok, false); assert.match(pointForgery.reason, /does not match/);
});

check('intervals crossing every mode transition enclose dense exact samples', () => {
  const family = create();
  for (const [lo, hi] of [[0.1, 0.49], [0.4, 0.55], [0.45, 0.65], [0.55, 1.0],
    [0.9, 1.3], [0.4, 1.4], [1.1, 1.9]]) {
    const bound = family.boundFreshAreaInterval(lo, hi);
    assert.equal(bound.ok, true, bound.reason);
    for (let index = 0; index <= 100; index += 1) {
      const sample = family.exactSample(lo + (hi - lo) * index / 100);
      assert.equal(sample.ok, true, sample.reason);
      assert.ok(sample.areaLower >= bound.areaLower - 1e-18, `lower miss ${lo}/${hi}/${index}`);
      assert.ok(sample.areaUpper <= bound.areaUpper + 1e-18, `upper miss ${lo}/${hi}/${index}`);
    }
    const crossed = family.descriptor.envelope.filter((entry) => entry.start > lo && entry.start < hi).length;
    assert.equal(bound.proof.pieces.length, crossed + 1);
  }
});

check('transition proof is ordered and binds every inner certificate', () => {
  const family = create(); const bound = family.boundFreshAreaInterval(0.4, 1.4);
  assert.equal(bound.ok, true, bound.reason);
  assert.equal(bound.proof.completeOrderedEnvelope, true);
  assert.deepEqual(bound.proof.pieces.map((entry) => [entry.subLo, entry.subHi]), [
    [0.4, 0.5], [0.5, 0.6], [0.6, 1.165217391304348], [1.165217391304348, 1.4],
  ]);
  assert.deepEqual(bound.proof.transitions.map((entry) => entry.p),
    [0.4, 0.5, 0.6, 1.165217391304348, 1.4]);
  bound.proof.pieces.forEach((entry) => {
    assert.match(entry.innerIntervalDigest, /^[0-9A-F]{64}$/);
    assert.match(entry.innerPathSignature, /^[0-9A-F]{64}$/);
  });
});

check('wrong q dimension and incomplete p domains stop safely', () => {
  const wrong = kktInput(); wrong.qFree = wrong.qFree.slice(0, 5); wrong.Minv = identity(5);
  let result = subject.createModeFamilyGeometry({ pDomain: [0, 2], kktInput: wrong, geometry: geometry() });
  assert.equal(result.ok, false); assert.match(result.reason, /six-DOF/);
  result = subject.createModeFamilyGeometry({ pDomain: [0.1, 2], kktInput: kktInput(), geometry: geometry() });
  assert.equal(result.ok, false); assert.match(result.reason, /pDomain/);
});

check('non-radial pure-rotation fold is not hidden by mode composition', () => {
  const family = create({ geometry: geometry({ workingSegment: { start: [0.005, -0.001], end: [0.005, 0.001] } }) });
  const sample = family.exactSample(1.5);
  assert.equal(sample.ok, false); assert.match(sample.reason, /fold|refinement|unresolved/);
  const bound = family.boundFreshAreaInterval(1.2, 1.8);
  assert.equal(bound.ok, false);
});

check('determinism and non-authority are explicit', () => {
  const left = create(); const right = create();
  assert.deepEqual(left.descriptor, right.descriptor);
  assert.match(left.descriptor.authority, /^none/);
  assert.equal(left.exactSample(0.73).sampleDigest, right.exactSample(0.73).sampleDigest);
  assert.doesNotMatch(fs.readFileSync(path.resolve(__dirname, 'mode-family-geometry.js'), 'utf8'),
    /authorityAuthorized\s*:\s*true|materialRemovalDefined\s*:\s*true/);
});

const report = { schema: 'material-kkt-mode-family-geometry-tests-v1', pass: true,
  passed: checks.length, checks,
  sourceSha256: hashFile(path.resolve(__dirname, 'mode-family-geometry.js')) };
process.stdout.write(`${JSON.stringify(report, null, 2)}\n`);
