'use strict';

const assert = require('node:assert/strict');
const crypto = require('node:crypto');
const fs = require('node:fs');
const path = require('node:path');
const { findEarliestTriangleMaterialWitness } = require('./material-toi-witness');

const ROOT = __dirname;
const results = [];

function approximately(actual, expected, tolerance, label) {
  assert.ok(Math.abs(actual - expected) <= tolerance,
    `${label}: expected ${expected} +/- ${tolerance}, received ${actual}`);
}

function rectangle(minX, minY, maxX, maxY) {
  return [[[[minX, minY], [maxX, minY], [maxX, maxY], [minX, maxY], [minX, minY]]]];
}

function namedTriangle(vertices) {
  return {
    vertices: vertices.map(([id, x, y]) => ({ id, x, y })),
    edges: [
      { id: 'working-face', working: true },
      { id: 'back-edge', working: false },
      { id: 'root-edge', working: false },
    ],
    workingVertexIds: ['tip'],
  };
}

function pose(x, y, angle = 0) {
  return { position: { x, y }, angle };
}

function motion(start, end, angleDelta) {
  const result = { start, end };
  if (angleDelta !== undefined) result.angleDelta = angleDelta;
  return result;
}

function runCase(name, callback) {
  const value = callback();
  results.push({ name, pass: true, ...(value || {}) });
}

const standardOptions = {
  timeTolerance: 2e-8,
  distanceTolerance: 1e-10,
  maxAngularTravelPerCell: Math.PI / 24,
};

runCase('tip-first virgin contact belongs to working face', () => {
  const feature = namedTriangle([
    ['root', 0, 0],
    ['tip', 1, 0],
    ['back', 0.2, 0.6],
  ]);
  const result = findEarliestTriangleMaterialWitness({
    feature,
    material: rectangle(2, -1, 3, 1),
    dt: 1,
    featureMotion: motion(pose(0, 0), pose(2, 0)),
    options: standardOptions,
  });
  assert.equal(result.found, true);
  assert.equal(result.contactClass, 'virgin-contact');
  assert.equal(result.witness.featureVertexId, 'tip');
  assert.equal(result.witness.featureEdgeId, 'working-face');
  assert.equal(result.witness.featureEdgeIndex, 0);
  approximately(result.witness.featureFraction, 1, 1e-12, 'tip owner fraction');
  assert.equal(result.witness.workingFace, true);
  assert.equal(result.removalAllowed, true);
  assert.equal(result.domainAction, 'material-response');
  assert.ok(result.witness.materialOutwardNormal.x < -0.999999);
  assert.ok(result.witness.signedClosingVelocity > 1.999999);
  approximately(result.toi, 0.5, 2e-7, 'tip TOI');
  return { toi: result.toi, role: result.witness.contactRole, edge: result.witness.featureEdgeId };
});

runCase('shared root vertex is owned by root edge and removal is refused', () => {
  const feature = namedTriangle([
    ['root', 0, -1],
    ['tip', 0.8, 0.1],
    ['back', -0.4, 0.5],
  ]);
  const result = findEarliestTriangleMaterialWitness({
    feature,
    material: rectangle(-2, -2, 2, -1),
    dt: 1,
    featureMotion: motion(pose(0, 2), pose(0, -2)),
    options: standardOptions,
  });
  assert.equal(result.found, true);
  assert.equal(result.witness.featureVertexId, 'root');
  assert.equal(result.witness.featureEdgeId, 'root-edge');
  assert.equal(result.witness.featureEdgeIndex, 2);
  approximately(result.witness.featureFraction, 1, 1e-12, 'root owner fraction');
  assert.equal(result.witness.workingFace, false);
  assert.equal(result.removalAllowed, false);
  assert.equal(result.domainAction, 'stop-non-working-boundary');
  assert.match(result.refusalReason, /root/);
  approximately(result.toi, 0.5, 2e-7, 'root TOI');
  return { toi: result.toi, role: result.witness.contactRole, edge: result.witness.featureEdgeId };
});

runCase('back edge interior wins globally and removal is refused', () => {
  const feature = namedTriangle([
    ['root', 0, 0],
    ['tip', 1, -0.6],
    ['back', 1, 0.6],
  ]);
  const result = findEarliestTriangleMaterialWitness({
    feature,
    material: rectangle(2, -0.1, 3, 0.1),
    dt: 1,
    featureMotion: motion(pose(0, 0), pose(2, 0)),
    options: standardOptions,
  });
  assert.equal(result.found, true);
  assert.equal(result.witness.featureVertexId, null);
  assert.equal(result.witness.featureEdgeId, 'back-edge');
  assert.equal(result.witness.featureEdgeIndex, 1);
  assert.ok(result.witness.featureFraction > 0.3 && result.witness.featureFraction < 0.7);
  assert.equal(result.removalAllowed, false);
  assert.equal(result.domainAction, 'stop-non-working-boundary');
  approximately(result.toi, 0.5, 2e-7, 'back-edge TOI');
  return { toi: result.toi, role: result.witness.contactRole, featureFraction: result.witness.featureFraction };
});

runCase('rotational mid-step transient contact is found when fixed samples are clear', () => {
  const feature = namedTriangle([
    ['root', 0, 0],
    ['tip', 1, 0],
    ['back', 0.975, -0.004],
  ]);
  const centerAngle = 7.5 * Math.PI / 180;
  const center = { x: 0.985 * Math.cos(centerAngle), y: 0.985 * Math.sin(centerAngle) };
  const half = 0.0015;
  const material = rectangle(center.x - half, center.y - half, center.x + half, center.y + half);
  const startAngle = -60 * Math.PI / 180;
  const angleDelta = 120 * Math.PI / 180;

  // Prove that a conventional fixed eight-sample sweep, including both
  // endpoints, sees no overlap/contact at any of its samples.
  const fixedSampleStatuses = [];
  for (let index = 0; index <= 8; index += 1) {
    const angle = startAngle + angleDelta * index / 8;
    const fixed = findEarliestTriangleMaterialWitness({
      feature,
      material,
      dt: 1,
      featureMotion: motion(pose(0, 0, angle), pose(0, 0, angle), 0),
      options: { ...standardOptions, maxAngularTravelPerCell: 10 },
    });
    fixedSampleStatuses.push(fixed.status);
    assert.equal(fixed.found, false, `fixed sample ${index}/8 unexpectedly touches`);
  }

  const result = findEarliestTriangleMaterialWitness({
    feature,
    material,
    dt: 1,
    featureMotion: motion(
      pose(0, 0, startAngle),
      pose(0, 0, startAngle + angleDelta),
      angleDelta,
    ),
    options: {
      ...standardOptions,
      // One top-level angular cell deliberately demonstrates that correctness
      // comes from the interval lower bound, not fixed angular samples.
      maxAngularTravelPerCell: 10,
      timeTolerance: 1e-8,
    },
  });
  assert.equal(result.found, true);
  assert.notEqual(result.status, 'indeterminate-node-limit');
  assert.equal(result.certificate.endpointOnly, false);
  assert.equal(result.certificate.angularCells, 1);
  assert.ok(result.toi > 0.5 && result.toi < 0.625);
  assert.ok(result.certificate.searchNodes > 1);

  // Same-TOI pose/twist audit. Endpoint pose is ~52.5 degrees away, so a
  // witness accidentally reconstructed from endpoint body state fails loudly.
  const witness = result.witness;
  let localPoint;
  if (witness.featureVertexIndex !== null) {
    localPoint = feature.vertices[witness.featureVertexIndex];
  } else {
    const a = feature.vertices[witness.featureEdgeIndex];
    const b = feature.vertices[(witness.featureEdgeIndex + 1) % 3];
    localPoint = {
      x: a.x + (b.x - a.x) * witness.featureFraction,
      y: a.y + (b.y - a.y) * witness.featureFraction,
    };
  }
  const toiAngle = startAngle + angleDelta * result.toi;
  const c = Math.cos(toiAngle); const s = Math.sin(toiAngle);
  const expectedPoint = {
    x: c * localPoint.x - s * localPoint.y,
    y: s * localPoint.x + c * localPoint.y,
  };
  approximately(witness.featureWorldPoint.x, expectedPoint.x, 5e-12, 'same-TOI feature point x');
  approximately(witness.featureWorldPoint.y, expectedPoint.y, 5e-12, 'same-TOI feature point y');
  const expectedVelocity = { x: -angleDelta * expectedPoint.y, y: angleDelta * expectedPoint.x };
  const expectedClosing = -(expectedVelocity.x * witness.materialOutwardNormal.x
    + expectedVelocity.y * witness.materialOutwardNormal.y);
  approximately(witness.signedClosingVelocity, expectedClosing, 2e-10, 'same-TOI signed closing');
  const endpointPoint = {
    x: Math.cos(startAngle + angleDelta) * localPoint.x - Math.sin(startAngle + angleDelta) * localPoint.y,
    y: Math.sin(startAngle + angleDelta) * localPoint.x + Math.cos(startAngle + angleDelta) * localPoint.y,
  };
  assert.ok(Math.hypot(
    witness.featureWorldPoint.x - endpointPoint.x,
    witness.featureWorldPoint.y - endpointPoint.y,
  ) > 0.2, 'test must distinguish TOI pose from endpoint pose');
  return {
    toi: result.toi,
    toiBracket: result.toiBracket,
    fixedSampleStatuses,
    searchNodes: result.certificate.searchNodes,
    maximumUnresolvedSpatialError: result.certificate.maximumUnresolvedSpatialError,
  };
});

runCase('hole boundary normal points from solid into hole and contact is re-entry', () => {
  const feature = namedTriangle([
    ['root', -0.2, 0],
    ['tip', 0.2, 0],
    ['back', -0.15, 0.1],
  ]);
  // Both rings are deliberately CCW. Ring role, not supplied winding, must
  // determine which side is material-outward.
  const currentMaterial = [[
    [[-3, -3], [3, -3], [3, 3], [-3, 3], [-3, -3]],
    [[-1, -1], [1, -1], [1, 1], [-1, 1], [-1, -1]],
  ]];
  const originalMaterial = rectangle(-3, -3, 3, 3);
  const result = findEarliestTriangleMaterialWitness({
    feature,
    material: currentMaterial,
    originalMaterial,
    dt: 1,
    featureMotion: motion(pose(0, 0), pose(1.5, 0)),
    options: standardOptions,
  });
  assert.equal(result.found, true);
  assert.equal(result.contactClass, 're-entry');
  assert.equal(result.witness.materialRingIndex, 1);
  assert.equal(result.witness.materialRingId, 'polygon-0:hole-0');
  assert.ok(result.witness.materialOutwardNormal.x < -0.999999);
  assert.ok(Math.abs(result.witness.materialOutwardNormal.y) < 1e-9);
  assert.ok(result.witness.signedClosingVelocity > 1.499999);
  assert.equal(result.witness.featureVertexId, 'tip');
  approximately(result.toi, 0.8 / 1.5, 2e-7, 'hole re-entry TOI');
  return {
    toi: result.toi,
    contactClass: result.contactClass,
    ring: result.witness.materialRingId,
    normal: result.witness.materialOutwardNormal,
  };
});

runCase('initial current-material overlap is a distinct invalid state', () => {
  const feature = namedTriangle([
    ['root', 0, 0],
    ['tip', 0.2, 0],
    ['back', 0, 0.2],
  ]);
  const result = findEarliestTriangleMaterialWitness({
    feature,
    material: rectangle(-1, -1, 1, 1),
    dt: 1,
    featureMotion: motion(pose(0, 0), pose(0.1, 0)),
    options: standardOptions,
  });
  assert.equal(result.found, true);
  assert.equal(result.status, 'invalid-initial-overlap');
  assert.equal(result.contactClass, 'initial-overlap');
  assert.equal(result.removalAllowed, false);
  assert.equal(result.domainAction, 'stop-initial-overlap');
  return { status: result.status, contactClass: result.contactClass };
});

runCase('outer normal is winding-independent', () => {
  const feature = namedTriangle([
    ['root', 0, 0],
    ['tip', 0.2, 0],
    ['back', 0, 0.1],
  ]);
  // Clockwise outer ring; expected left-side outward normal remains (-1, 0).
  const clockwiseOuter = [[[[2, -1], [2, 1], [3, 1], [3, -1], [2, -1]]]];
  const result = findEarliestTriangleMaterialWitness({
    feature,
    material: clockwiseOuter,
    dt: 1,
    featureMotion: motion(pose(0, 0), pose(3, 0)),
    options: standardOptions,
  });
  assert.equal(result.found, true);
  assert.equal(result.witness.materialRingIndex, 0);
  assert.ok(result.witness.materialOutwardNormal.x < -0.999999);
  assert.ok(Math.abs(result.witness.materialOutwardNormal.y) < 1e-9);
  return { normal: result.witness.materialOutwardNormal };
});

runCase('separating old boundary does not mask later compressive non-working/working decision', () => {
  const feature = namedTriangle([
    ['root', 0, 0],
    ['tip', 1, 0],
    ['back', 0.2, 0.5],
  ]);
  const material = [
    [[[-2, -1], [0, -1], [0, 1], [-2, 1], [-2, -1]]],
    [[[2, -1], [3, -1], [3, 1], [2, 1], [2, -1]]],
  ];
  const continued = findEarliestTriangleMaterialWitness({
    feature,
    material,
    dt: 1,
    featureMotion: motion(pose(0, 0), pose(2, 0)),
    options: { ...standardOptions, continueAfterSeparatingBoundary: true },
  });
  assert.equal(continued.found, true);
  assert.equal(continued.status, 'contact');
  assert.equal(continued.witness.featureVertexId, 'tip');
  assert.equal(continued.witness.workingFace, true);
  assert.equal(continued.removalAllowed, true);
  assert.ok(continued.certificate.initialSeparatingBoundary);
  assert.ok(continued.certificate.certifiedSeparatingPrefixSteps >= 1);
  approximately(continued.toi, 0.5, 2e-7, 'later compressive TOI');

  const stopped = findEarliestTriangleMaterialWitness({
    feature,
    material,
    dt: 1,
    featureMotion: motion(pose(0, 0), pose(2, 0)),
    options: { ...standardOptions, continueAfterSeparatingBoundary: false },
  });
  assert.equal(stopped.status, 'separating-boundary');
  assert.equal(stopped.removalAllowed, false);
  assert.equal(stopped.domainAction, 'no-material-response');
  return {
    continuedToi: continued.toi,
    separatingPrefixSteps: continued.certificate.certifiedSeparatingPrefixSteps,
    optOutStatus: stopped.status,
  };
});

runCase('pure replay from frozen snapshot is byte stable and leaves input untouched', () => {
  const request = {
    feature: namedTriangle([
      ['root', 0, 0],
      ['tip', 1, 0],
      ['back', 0.2, 0.5],
    ]),
    material: rectangle(2, -1, 3, 1),
    dt: 1,
    featureMotion: motion(pose(0, 0), pose(2, 0)),
    materialMotion: motion(pose(0, 0), pose(0, 0)),
    history: { hadPriorContact: false },
    options: { ...standardOptions },
  };
  const deepFreeze = (value) => {
    if (value && typeof value === 'object' && !Object.isFrozen(value)) {
      Object.freeze(value);
      Object.values(value).forEach(deepFreeze);
    }
    return value;
  };
  const snapshotBytes = JSON.stringify(request);
  const snapshotHash = crypto.createHash('sha256').update(snapshotBytes).digest('hex').toUpperCase();
  deepFreeze(request);
  const first = findEarliestTriangleMaterialWitness(request);
  assert.equal(JSON.stringify(request), snapshotBytes);

  // Restore solely from the original bytes, freeze again, and replay.
  const restored = deepFreeze(JSON.parse(snapshotBytes));
  const second = findEarliestTriangleMaterialWitness(restored);
  assert.equal(JSON.stringify(restored), snapshotBytes);
  assert.deepEqual(second, first);
  const resultHash = crypto.createHash('sha256').update(JSON.stringify(first)).digest('hex').toUpperCase();
  return { snapshotHash, resultHash, deterministicReplay: true, inputByteStable: true };
});

runCase('three-dt and refinement convergence', () => {
  const feature = namedTriangle([
    ['root', 0, 0],
    ['tip', 1, 0],
    ['back', 0.2, 0.5],
  ]);
  const material = rectangle(2, -1, 3, 1);
  const totalDuration = 1;
  const velocity = 3;
  const solveStepped = (stepDt, timeTolerance) => {
    for (let startTime = 0; startTime < totalDuration - 1e-15; startTime += stepDt) {
      const endTime = Math.min(totalDuration, startTime + stepDt);
      const result = findEarliestTriangleMaterialWitness({
        feature,
        material,
        dt: endTime - startTime,
        featureMotion: motion(
          pose(velocity * startTime, 0),
          pose(velocity * endTime, 0),
        ),
        options: { ...standardOptions, timeTolerance, distanceTolerance: 1e-11 },
      });
      if (result.found) return { absoluteToi: startTime + result.toi, result };
    }
    throw new Error(`no contact for step dt ${stepDt}`);
  };
  const runs = [
    solveStepped(0.5, 2e-7),
    solveStepped(0.25, 5e-8),
    solveStepped(0.125, 1.25e-8),
  ];
  const exact = 1 / 3;
  runs.forEach(({ absoluteToi, result }, index) => {
    const bound = result.certificate.toiBracketWidth + 2e-8;
    assert.ok(Math.abs(absoluteToi - exact) <= Math.max(bound, 3e-7 / (4 ** index)));
  });
  const times = runs.map((candidate) => candidate.absoluteToi);
  assert.ok(Math.max(...times) - Math.min(...times) < 3e-7);
  const widths = runs.map((candidate) => candidate.result.certificate.toiBracketWidth);
  assert.ok(widths[1] <= widths[0] + 1e-15);
  assert.ok(widths[2] <= widths[1] + 1e-15);
  return { stepDts: [0.5, 0.25, 0.125], absoluteTois: times, toiBracketWidths: widths };
});

function sha256(fileName) {
  return crypto.createHash('sha256').update(fs.readFileSync(path.join(ROOT, fileName))).digest('hex').toUpperCase();
}

const report = {
  suite: 'material-toi-witness',
  pass: true,
  testsPassed: results.length,
  testsFailed: 0,
  invariants: {
    endpointOnlySweep: false,
    globalFeatureAndMultiPolygonBoundaryEnumeration: true,
    sameToiPoseAndTwistWitness: true,
    nonWorkingFirstOnsetRefusesRemoval: true,
    deterministicSharedVertexOwnership: true,
    separatingBoundaryDoesNotMaskLaterCompression: true,
    frozenSnapshotReplayIsPure: true,
  },
  results,
  hashes: {
    'material-toi-witness.js': sha256('material-toi-witness.js'),
    'run-tests.js': sha256('run-tests.js'),
    'README.md': sha256('README.md'),
  },
};

const reportPath = path.join(ROOT, 'test-report.json');
fs.writeFileSync(reportPath, `${JSON.stringify(report, null, 2)}\n`, 'utf8');
process.stdout.write(`${JSON.stringify(report, null, 2)}\n`);
