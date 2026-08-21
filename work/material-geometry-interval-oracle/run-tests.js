'use strict';

const assert = require('node:assert/strict');
const crypto = require('node:crypto');
const fs = require('node:fs');
const path = require('node:path');
const kernelModule = require('./geometry-interval-oracle.js');

const results = [];
function test(name, run) { results.push({ name, run }); }
function closeRing(points) { return [...points, points[0]]; }
function rectangle(x0, y0, x1, y1) {
  return [closeRing([[x0, y0], [x1, y0], [x1, y1], [x0, y1]])];
}
function hashFile(filename) {
  return crypto.createHash('sha256').update(fs.readFileSync(filename)).digest('hex').toUpperCase();
}
function baseConfig(overrides = {}) {
  return {
    remainingGeometry: [rectangle(-1, -1, 5, 1)],
    workingSegment: { start: [0, -0.4], end: [0, 0.4] },
    displacementAffine: { intercept: [0, 0], slope: [1, 0] },
    angularTravel: { intercept: 0, slope: 0 },
    pDomain: [0, 0.8],
    lengthTolerance: 0,
    ...overrides,
  };
}
function create(overrides = {}) {
  return kernelModule.createTranslationSweepKernel(baseConfig(overrides));
}
function assertContainsDense(kernel, lo, hi, count = 1001) {
  const bound = kernel.boundFreshAreaInterval(lo, hi);
  assert.equal(bound.ok, true, bound.reason);
  for (let index = 0; index < count; index += 1) {
    const p = lo + (hi - lo) * index / (count - 1);
    const sample = kernel.exactSample(p);
    assert.equal(sample.ok, true, sample.reason);
    assert.ok(sample.area >= bound.areaLower,
      `${p}: returned exact area ${sample.area} below ${bound.areaLower}`);
    assert.ok(sample.area <= bound.areaUpper,
      `${p}: returned exact area ${sample.area} above ${bound.areaUpper}`);
    assert.ok(sample.areaLower >= bound.areaLower,
      `${p}: exact lower enclosure ${sample.areaLower} below interval lower ${bound.areaLower}`);
    assert.ok(sample.areaUpper <= bound.areaUpper,
      `${p}: exact upper enclosure ${sample.areaUpper} above interval upper ${bound.areaUpper}`);
  }
  return bound;
}

test('outer polygon exact sweep and dense conservative interval', () => {
  const kernel = create();
  assert.equal(kernel.ok, true, kernel.reason);
  assert.equal(kernel.descriptor.authority, 'none; digests are integrity checks, never authentication');
  const sample = kernel.exactSample(0.4);
  assert.equal(sample.ok, true, sample.reason);
  assert.ok(Math.abs(sample.area - 0.32) <= sample.numericalAreaGuard);
  assert.ok(Math.abs(sample.payload.intersectionArea - sample.payload.differenceArea)
    <= 2 * sample.numericalAreaGuard);
  const bound = assertContainsDense(kernel, 0, 0.8, 1001);
  return { area: sample.area, sampleGuard: sample.numericalAreaGuard, bound };
});

test('holes and disconnected components remain authoritative', () => {
  const outerWithHole = [
    closeRing([[-1, -1], [4, -1], [4, 1], [-1, 1]]),
    closeRing([[1.25, -0.25], [1.25, 0.25], [2.25, 0.25], [2.25, -0.25]]),
  ];
  const island = rectangle(4.5, -0.5, 5, 0.5);
  const kernel = create({ remainingGeometry: [outerWithHole, island] });
  assert.equal(kernel.ok, true, kernel.reason);
  const bound = assertContainsDense(kernel, 0.05, 0.75, 1001);
  const sample = kernel.exactSample(0.5);
  assert.equal(sample.ok, true, sample.reason);
  assert.ok(sample.payload.desiredGeometry.length >= 1);
  return { area: sample.area, bound };
});

test('re-entry uses only current remaining material', () => {
  const firstKernel = create();
  const first = firstKernel.exactSample(0.5);
  assert.equal(first.ok, true, first.reason);
  const replay = create({ remainingGeometry: first.payload.desiredGeometry });
  assert.equal(replay.ok, true, replay.reason);
  const repeated = replay.exactSample(0.5);
  assert.equal(repeated.ok, true, repeated.reason);
  assert.ok(repeated.area <= repeated.numericalAreaGuard,
    `repeat sweep returned ${repeated.area} with guard ${repeated.numericalAreaGuard}`);
  const later = replay.exactSample(0.7);
  assert.equal(later.ok, true, later.reason);
  assert.ok(later.area > later.numericalAreaGuard);
  return { repeatedArea: repeated.area, laterArea: later.area };
});

test('empty geometry is exactly zero', () => {
  const kernel = create({ remainingGeometry: [] });
  assert.equal(kernel.ok, true, kernel.reason);
  const sample = kernel.exactSample(0.5);
  assert.deepEqual(
    { area: sample.area, lo: sample.areaLower, hi: sample.areaUpper, guard: sample.numericalAreaGuard },
    { area: 0, lo: 0, hi: 0, guard: 0 },
  );
  const bound = kernel.boundFreshAreaInterval(0.1, 0.7);
  assert.equal(bound.ok, true, bound.reason);
  assert.equal(bound.areaLower, 0); assert.equal(bound.areaUpper, 0);
  return { sample, bound };
});

test('finite or p-dependent rotation is refused', () => {
  for (const angularTravel of [
    { intercept: 1e-15, slope: 0 },
    { intercept: 0, slope: 1e-15 },
    { intercept: 0.1, slope: -0.2 },
  ]) {
    const result = create({ angularTravel });
    assert.equal(result.ok, false);
    assert.equal(result.status, 'solver-domain-stop');
    assert.match(result.reason, /rotation/);
  }
  return { refused: 3 };
});

test('missing or malformed angular provenance is refused fail-closed', () => {
  const mutants = [
    (config) => { delete config.angularTravel; },
    (config) => { config.angularTravel = null; },
    (config) => { config.angularTravel = {}; },
    (config) => { config.angularTravel = { intercept: 0 }; },
    (config) => { config.angularTravel = { slope: 0 }; },
    (config) => { config.angularTravel = { intercept: Number.NaN, slope: 0 }; },
    (config) => { config.angularTravel = { intercept: 0, slope: Number.NaN }; },
  ];
  mutants.forEach((mutate, index) => {
    const config = baseConfig(); mutate(config);
    const result = kernelModule.createTranslationSweepKernel(config);
    assert.equal(result.ok, false, `angular mutant ${index}`);
    assert.equal(result.status, 'solver-domain-stop');
    assert.match(result.reason, /angular/);
  });
  return { refused: mutants.length };
});

test('zero working segment and unresolved geometry stop', () => {
  const zero = create({ workingSegment: { start: [0, 0], end: [0, 0] } });
  assert.equal(zero.ok, false); assert.match(zero.reason, /segment/);
  const unresolved = create({
    remainingGeometry: [rectangle(0, 0, 1e-7, 1)],
    lengthTolerance: 1e-8,
  });
  assert.equal(unresolved.ok, false);
  assert.match(unresolved.reason, /thin feature|relative-area cap|numerically/);
  return { zeroReason: zero.reason, unresolvedReason: unresolved.reason };
});

test('collinear disjoint boundaries retain a resolved thin ligament', () => {
  const ligament = [[closeRing([
    [0, 0], [8, 0], [8, 1.995], [4, 1.995], [4, 2.005], [8, 2.005], [8, 4], [0, 4],
  ])]];
  const minimum = kernelModule._geometry.minimumGeometryFeature(ligament);
  assert.ok(Math.abs(minimum - 0.01) < 1e-12, `minimum feature was ${minimum}`);
  const kernel = create({ remainingGeometry: ligament });
  assert.equal(kernel.ok, true, kernel.reason);
  return { minimumFeature: minimum, lengthGuard: kernel.descriptor.lengthGuard };
});

test('published interval strictly contains endpoint values and enclosures', () => {
  const kernel = create();
  assert.equal(kernel.ok, true, kernel.reason);
  const lo = 0.173; const hi = 0.611;
  const bound = kernel.boundFreshAreaInterval(lo, hi);
  assert.equal(bound.ok, true, bound.reason);
  for (const p of [lo, (lo + hi) / 2, hi]) {
    const sample = kernel.exactSample(p);
    assert.ok(bound.areaLower <= sample.areaLower);
    assert.ok(bound.areaUpper >= sample.areaUpper);
    assert.ok(bound.areaLower <= sample.area && sample.area <= bound.areaUpper);
  }
  return {
    interval: [bound.areaLower, bound.areaUpper],
    endpointGuards: bound.proof.endpointNumericalAreaGuards,
  };
});

test('seeded random intervals contain every dense exact enclosure', () => {
  const kernel = create({
    remainingGeometry: [[
      closeRing([[-2, -2], [6, -2], [6, 2], [-2, 2]]),
      closeRing([[1, -0.35], [1, 0.35], [2.4, 0.35], [2.4, -0.35]]),
    ], rectangle(6.5, -0.6, 7.3, 0.6)],
    pDomain: [0, 1],
  });
  assert.equal(kernel.ok, true, kernel.reason);
  let seed = 0x5a17c0de; const random = () => {
    seed = (Math.imul(seed, 1664525) + 1013904223) >>> 0;
    return seed / 0x100000000;
  };
  let probes = 0;
  for (let intervalIndex = 0; intervalIndex < 80; intervalIndex += 1) {
    const a = random(); const b = random();
    const lo = Math.min(a, b); const hi = Math.max(a, b);
    if (hi - lo < 1e-6) { intervalIndex -= 1; continue; }
    assertContainsDense(kernel, lo, hi, 65); probes += 65;
  }
  return { intervals: 80, probes, finalSeed: seed };
});

test('large-coordinate caller tolerance cannot create a dimensionally wrong loose area gate', () => {
  const scale = 1e6;
  const safe = create({
    remainingGeometry: [rectangle(-scale, -scale, scale, scale)],
    workingSegment: { start: [0, -0.4 * scale], end: [0, 0.4 * scale] },
    displacementAffine: { intercept: [0, 0], slope: [scale, 0] },
  });
  assert.equal(safe.ok, true, safe.reason);
  const sample = safe.exactSample(0.4);
  assert.equal(sample.ok, true, sample.reason);
  assert.ok(sample.numericalAreaGuard / sample.payload.currentArea
    <= kernelModule.RELATIVE_NUMERIC_AREA_CAP);

  const loose = create({
    remainingGeometry: [rectangle(-scale, -scale, scale, scale)],
    workingSegment: { start: [0, -0.4 * scale], end: [0, 0.4 * scale] },
    displacementAffine: { intercept: [0, 0], slope: [scale, 0] },
    lengthTolerance: 0.1,
  });
  assert.equal(loose.ok, false);
  assert.match(loose.reason, /relative-area cap/);
  return {
    safeGuard: sample.numericalAreaGuard,
    relativeGuard: sample.numericalAreaGuard / sample.payload.currentArea,
    looseReason: loose.reason,
  };
});

test('parallel and oblique translation use the same exact convex sweep identity', () => {
  const parallel = create({ displacementAffine: { intercept: [0, 0], slope: [0, 1] } });
  assert.equal(parallel.ok, true, parallel.reason);
  const parallelSample = parallel.exactSample(0.4);
  assert.equal(parallelSample.ok, true, parallelSample.reason);
  assert.ok(parallelSample.area <= parallelSample.numericalAreaGuard);

  const oblique = create({ displacementAffine: { intercept: [0, 0], slope: [1, 0.25] } });
  assert.equal(oblique.ok, true, oblique.reason);
  const obliqueSample = oblique.exactSample(0.4);
  assert.equal(obliqueSample.ok, true, obliqueSample.reason);
  assert.ok(Math.abs(obliqueSample.payload.analyticSweepArea - 0.32)
    <= obliqueSample.numericalAreaGuard);
  assertContainsDense(oblique, 0.11, 0.73, 503);
  return { parallelArea: parallelSample.area, obliqueArea: obliqueSample.area };
});

test('invalid intervals, nonfinite values, and self-crossing inputs stop safely', () => {
  const kernel = create();
  assert.equal(kernel.ok, true, kernel.reason);
  for (const interval of [[0.5, 0.5], [-0.1, 0.2], [0.1, 0.9]]) {
    const result = kernel.boundFreshAreaInterval(interval[0], interval[1]);
    assert.equal(result.ok, false, `${interval}`);
  }
  assert.equal(kernel.exactSample(Number.NaN).ok, false);
  const selfCrossing = create({
    remainingGeometry: [[closeRing([[0, 0], [2, 2], [0, 2], [2, 0]])]],
  });
  assert.equal(selfCrossing.ok, false);
  return { refused: 5, selfCrossingReason: selfCrossing.reason };
});

test('module exposes no authority or trust-signing API', () => {
  const forbidden = [
    'trustedWorkingWitnessProviderSha256', 'frozenWorkingWitness',
    'sFreeBinding', 'contactFrameSignature', 'materialRowSignature',
    'kinematics', 'solverInput', '_signing',
  ];
  forbidden.forEach((key) => assert.equal(Object.hasOwn(kernelModule, key), false, key));
  const kernel = create();
  assert.equal(kernel.ok, true, kernel.reason);
  assert.equal(kernel.descriptor.authority.startsWith('none'), true);
  return { forbidden, authority: kernel.descriptor.authority };
});

const startedAt = new Date().toISOString();
const observations = []; let failed = null;
for (const entry of results) {
  try {
    observations.push({ name: entry.name, pass: true, observation: entry.run() });
  } catch (error) {
    observations.push({ name: entry.name, pass: false, error: String(error?.stack || error) });
    failed ||= error;
  }
}
const reportPath = path.join(__dirname, 'test-report.json');
const report = {
  stage: 'S4b-1A-pure-translation-sweep-geometry-kernel',
  startedAt,
  completedAt: new Date().toISOString(),
  pass: !failed,
  passed: observations.filter((entry) => entry.pass).length,
  total: observations.length,
  scope: {
    proven: 'pure fixed-orientation segment + affine translation + authoritative MultiPolygon numerical enclosure',
    excluded: [
      'Sfree/path/owner/witness/body provenance', 'finite rotation', 'structural active-mode transition',
      'production removal authority', 'parameter-tooth fracture/removal definition',
    ],
  },
  sourceSha256: hashFile(__filename.replace('run-tests.js', 'geometry-interval-oracle.js')),
  runnerSha256: hashFile(__filename),
  observations,
};
fs.writeFileSync(reportPath, `${JSON.stringify(report, null, 2)}\n`);
process.stdout.write(`${report.pass ? 'PASS' : 'FAIL'} ${report.passed}/${report.total}\n`);
process.stdout.write(`${JSON.stringify({
  sourceSha256: report.sourceSha256,
  runnerSha256: report.runnerSha256,
  reportPath,
}, null, 2)}\n`);
if (failed) process.exitCode = 1;
