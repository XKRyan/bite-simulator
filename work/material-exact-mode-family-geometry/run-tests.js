'use strict';

const assert = require('node:assert/strict');
const crypto = require('node:crypto');
const fs = require('node:fs');
const path = require('node:path');
const subject = require('./exact-mode-family-geometry.js');
const coulombV2 = require('../kkt-coulomb-extension/event-kkt-coulomb.js');
const coulombV3 = require('../material-prepared-root-integration/event-kkt-coulomb-v3.js');

function hashFile(filename) {
  return crypto.createHash('sha256').update(fs.readFileSync(filename)).digest('hex').toUpperCase();
}
function identity(size) {
  return Array.from({ length: size }, (_, row) => Array.from({ length: size }, (_, column) => row === column ? 1 : 0));
}
function kktInput() {
  return {
    qFree: [0.2, 0, 0, 0, 0, 0],
    Minv: identity(6),
    materialContact: { id: 'weapon-material', point: { x: 0, y: 0 },
      normalRow: [1, 0, 0, 0, 0, 0] },
    structuralContacts: [],
    specificCuttingEnergy: 10000,
    width: 1,
  };
}
function remainingGeometry() {
  return [[[[-0.01, -0.01], [0.01, -0.01], [0.01, 0.01], [-0.01, 0.01], [-0.01, -0.01]]]];
}
function remainingTriangles() {
  return [[[-0.01, -0.01], [0.01, -0.01], [0.01, 0.01]],
    [[-0.01, -0.01], [0.01, 0.01], [-0.01, 0.01]]];
}
function config() {
  return {
    remainingTriangles: remainingTriangles(),
    modeFamilyConfig: {
      pDomain: [0, 0.4],
      kktInput: kktInput(),
      geometry: {
        h: 0.001,
        timeFractions: [0, 0.25, 0.5, 0.75, 1],
        maximumTimeChordError: 5e-6,
        lengthTolerance: 0,
        workingSegment: { start: [0, -0.005], end: [0, 0.005] },
        featureStartPosition: [0, 0],
        featureStartAngle: 0,
        materialStartPosition: [0, 0],
        materialStartAngle: 0,
        remainingGeometry: remainingGeometry(),
      },
    },
  };
}
function create() {
  const result = subject.createExactModeFamilyGeometry(config());
  assert.equal(result.ok, true, result.reason); return result;
}
function solve(p) {
  return coulombV2.solvePrescribedImpulse({ ...kktInput(), freshArea: () => 0 }, p);
}
function bindingContext() {
  return { sFreeBinding: 'fixture-sfree', contactFrameSignature: 'fixture-frame',
    qFreeSignature: 'fixture-qfree', inverseMassSignature: 'fixture-minv',
    materialContactSignature: 'fixture-material', structuralContactsSignature: 'fixture-structural',
    h: 0.001, Wact: 0 };
}

const checks = [];
function check(name, run) {
  const started = Date.now(); run(); checks.push({ name, pass: true, elapsedMs: Date.now() - started });
}

check('all source pins and non-authority declarations are exact', () => {
  assert.equal(subject.EXPECTED_MODE_FAMILY_SHA256,
    hashFile(path.resolve(__dirname, '../material-kkt-mode-family-geometry/mode-family-geometry.js')));
  assert.equal(subject.EXPECTED_EXACT_AREA_SHA256,
    hashFile(path.resolve(__dirname, '../material-exact-triangle-sweep-area/exact-triangle-sweep-area.js')));
  const oracle = create(); assert.match(oracle.descriptor.authority, /^none/);
});

check('exact strip samples match the analytic affine area and moments', () => {
  const oracle = create();
  for (const p of [0, 0.025, 0.05, 0.1, 0.15, 0.199, 0.2, 0.225]) {
    const sample = oracle.exactSample(p, solve(p)); assert.equal(sample.ok, true, sample.reason);
    const expected = Math.abs(0.2 - p) * 0.001 * 0.01;
    assert.ok(Math.abs(sample.area - expected) <= Math.max(1e-22, expected * 2e-15), `${p}: ${sample.area}/${expected}`);
    assert.ok(sample.areaLower <= expected && expected <= sample.areaUpper);
    assert.ok(sample.payload.exactMoments.polarSecondMomentCentroid);
  }
});

check('interval bounds contain dense exact samples with shrinking width', () => {
  const oracle = create(); let previousWidth = Infinity;
  for (const [lo, hi] of [[0, 0.19], [0.05, 0.12], [0.07, 0.09], [0.075, 0.08]]) {
    const bound = oracle.boundFreshAreaInterval(lo, hi); assert.equal(bound.ok, true, bound.reason);
    for (let index = 0; index <= 100; index += 1) {
      const sample = oracle.exactSample(lo + (hi - lo) * index / 100);
      assert.equal(sample.ok, true, sample.reason);
      assert.ok(sample.areaLower >= bound.areaLower - Number.MIN_VALUE);
      assert.ok(sample.areaUpper <= bound.areaUpper + Number.MIN_VALUE);
    }
    const width = bound.areaUpper - bound.areaLower;
    if (hi - lo < 0.03) assert.ok(width < previousWidth);
    previousWidth = width;
  }
});

check('v3 isolates and validates the first constitutive root without a fixed polygon guard', () => {
  const oracle = create();
  const prepared = coulombV3.prepareMaterialEvent({
    ...kktInput(),
    freshArea: (p, trial) => {
      const sample = oracle.exactSample(p, trial);
      if (!sample.ok) throw new Error(`exact sample ${p} stopped: ${sample.reason}`);
      return sample;
    },
    boundFreshAreaInterval: (lo, hi) => oracle.boundFreshAreaInterval(lo, hi),
    intervalOracle: oracle.descriptor.intervalOracle,
    trustedGeometryIntervalOracleSha256: subject.moduleSha256,
    trustedCertifiedCoulombSha256: coulombV3.selfSha256,
    freshAreaIntervalSource: { schema: 'exact-mode-family-test-source-v1' },
    bindingContext: bindingContext(),
    options: { maximumCertifiedIntervals: 2048, workRelativeTolerance: 0 },
  });
  assert.equal(prepared.ok, true, prepared.reason);
  const analyticRoot = 0.3 - Math.sqrt(0.05);
  assert.ok(Math.abs(prepared.p - analyticRoot) < 2e-10, `${prepared.p}/${analyticRoot}`);
  assert.ok(Math.abs(prepared.workResidual) <= 3e-12);
  assert.ok(prepared.rootCertificate.prefixExclusionLeaves.length > 0);
  const validation = coulombV3.validatePreparedEvent(prepared, {
    trustedGeometryIntervalOracleSha256: subject.moduleSha256,
    trustedCertifiedCoulombSha256: coulombV3.selfSha256,
  });
  assert.equal(validation.ok, true, validation.reason);
});

check('v3 can refine a finite-rotation full-range prefix into certified topology leaves', () => {
  const rotatingConfig = config(); rotatingConfig.modeFamilyConfig.kktInput.qFree[1] = 10;
  rotatingConfig.modeFamilyConfig.geometry.workingSegment = { start: [0.002, 0], end: [0.008, 0] };
  const oracle = subject.createExactModeFamilyGeometry(rotatingConfig);
  assert.equal(oracle.ok, true, oracle.reason);
  const rotatingInput = kktInput(); rotatingInput.qFree[1] = 10;
  const prepared = coulombV3.prepareMaterialEvent({
    ...rotatingInput,
    freshArea: (p, trial) => {
      const sample = oracle.exactSample(p, trial);
      if (!sample.ok) throw new Error(`rotating exact sample ${p} stopped: ${sample.reason}`);
      return sample;
    },
    boundFreshAreaInterval: (lo, hi) => oracle.boundFreshAreaInterval(lo, hi),
    intervalOracle: oracle.descriptor.intervalOracle,
    trustedGeometryIntervalOracleSha256: subject.moduleSha256,
    trustedCertifiedCoulombSha256: coulombV3.selfSha256,
    freshAreaIntervalSource: { schema: 'exact-rotating-mode-family-test-source-v1' },
    bindingContext: bindingContext(),
    options: { maximumCertifiedIntervals: 4096, workRelativeTolerance: 0 },
  });
  assert.equal(prepared.ok, true, prepared.reason);
  assert.ok(prepared.p > 0 && prepared.p < 0.2);
  assert.ok(prepared.rootCertificate.prefixExclusionLeaves.length > 0);
  const validation = coulombV3.validatePreparedEvent(prepared, {
    trustedGeometryIntervalOracleSha256: subject.moduleSha256,
    trustedCertifiedCoulombSha256: coulombV3.selfSha256,
  });
  assert.equal(validation.ok, true, validation.reason);
});

check('forged endpoint trial and cloned cover-derived payload stop safely', () => {
  const oracle = create(); const forged = JSON.parse(JSON.stringify(solve(0.08)));
  forged.qPost[0] += 1e-15;
  const result = oracle.exactSample(0.08, forged);
  assert.equal(result.ok, false); assert.match(result.reason, /disagrees|does not match/);
});

check('folded time cells stop instead of being filled by their convex hull', () => {
  const folded = config();
  folded.modeFamilyConfig.pDomain = [0, 0.9];
  folded.modeFamilyConfig.kktInput.qFree = [2, 0, 0, 0, 0, 0];
  folded.modeFamilyConfig.kktInput.materialContact.normalRow = [1, -1, 0, 0, 0, 0];
  folded.modeFamilyConfig.geometry.workingSegment = { start: [0, -2], end: [0, 2] };
  folded.remainingTriangles = [[[-3, -3], [3, -3], [3, 3]], [[-3, -3], [3, 3], [-3, 3]]];
  folded.modeFamilyConfig.geometry.remainingGeometry =
    [[[[-3, -3], [3, -3], [3, 3], [-3, 3], [-3, -3]]]];
  const oracle = subject.createExactModeFamilyGeometry(folded);
  assert.equal(oracle.ok, true, oracle.reason);
  assert.equal(oracle.exactSample(0).ok, true);
  for (const p of [0.67, 0.8, 0.9]) {
    const sample = oracle.exactSample(p);
    assert.equal(sample.ok, false, `folded p=${p} was accepted`);
    assert.match(sample.reason, /fold|intersect|declared order/);
  }
  const bound = oracle.boundFreshAreaInterval(0.67, 0.9);
  assert.equal(bound.ok, true, bound.reason);
  assert.ok(bound.proof.pieces.some((piece) =>
    piece.topologyProof.status === 'full-range-topology-unresolved'));
});

check('overlapping triangles and mismatched total area are rejected at construction', () => {
  let bad = config(); bad.remainingTriangles.push(bad.remainingTriangles[0]);
  let result = subject.createExactModeFamilyGeometry(bad);
  assert.equal(result.ok, false); assert.match(result.reason, /overlap/);
  bad = config(); bad.remainingTriangles = [[[-0.01, -0.01], [0, -0.01], [0, 0]]];
  result = subject.createExactModeFamilyGeometry(bad);
  assert.equal(result.ok, false); assert.match(result.reason, /disagrees|does not match/);
});

check('rotating intervals carry a whole-interval topology proof or an explicit full-range enclosure', () => {
  const rotatingConfig = config(); rotatingConfig.modeFamilyConfig.kktInput.qFree[1] = 10;
  const oracle = subject.createExactModeFamilyGeometry(rotatingConfig);
  assert.equal(oracle.ok, true, oracle.reason);
  const sample = oracle.exactSample(0.05); assert.equal(sample.ok, true, sample.reason);
  const interval = oracle.boundFreshAreaInterval(0.04, 0.06);
  assert.equal(interval.ok, true, interval.reason);
  for (const piece of interval.proof.pieces) {
    const status = piece.topologyProof?.status;
    assert.ok(status === 'cellwise-convexity-and-nonadjacent-separation'
      || status === 'full-range-topology-unresolved', status);
    if (status === 'full-range-topology-unresolved') {
      assert.equal(piece.lower, 0); assert.ok(piece.upper >= oracle.exactSample(0.05).areaUpper);
    }
  }
});

check('determinism is byte exact', () => {
  const left = create(); const right = create();
  assert.deepEqual(left.descriptor, right.descriptor);
  assert.deepEqual(left.exactSample(0.073), right.exactSample(0.073));
  assert.deepEqual(left.boundFreshAreaInterval(0.05, 0.1), right.boundFreshAreaInterval(0.05, 0.1));
});

process.stdout.write(`${JSON.stringify({ schema: 'exact-mode-family-geometry-tests-v1',
  pass: true, passed: checks.length, checks,
  sourceSha256: hashFile(path.resolve(__dirname, 'exact-mode-family-geometry.js')) }, null, 2)}\n`);
