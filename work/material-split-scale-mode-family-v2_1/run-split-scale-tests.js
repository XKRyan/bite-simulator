'use strict';

const assert = require('node:assert/strict');
const crypto = require('node:crypto');
const fs = require('node:fs');
const path = require('node:path');
const finiteRotation = require('./finite-rotation-sweep-oracle-v2_1.js');
const modeFamily = require('./mode-family-geometry-v2_1.js');

function identity(size) {
  return Array.from({ length: size }, (_, row) => Array.from(
    { length: size }, (_, column) => row === column ? 1 : 0,
  ));
}
function embed(row) { return [row[0], 0, 0, row[1], row[2], 0]; }
function clone(value) { return JSON.parse(JSON.stringify(value)); }
function nextDown(value) {
  if (Number.isNaN(value) || value === -Infinity) return value;
  if (value === 0) return -Number.MIN_VALUE;
  const view = new DataView(new ArrayBuffer(8));
  view.setFloat64(0, value, false);
  let bits = view.getBigUint64(0, false);
  bits += value > 0 ? -1n : 1n;
  view.setBigUint64(0, bits, false);
  return view.getFloat64(0, false);
}
function distance(left, right) {
  return Math.hypot(left.x - right.x, left.y - right.y);
}
function remainingGeometry() {
  return [[[[-0.01, -0.01], [0.01, -0.01], [0.01, 0.01],
    [-0.01, 0.01], [-0.01, -0.01]]]];
}
function baseKernelConfig() {
  return {
    pDomain: [0, 1],
    h: 1e-4,
    timeFractions: [0, 0.25, 0.5, 0.75, 1],
    maximumTimeChordError: 5e-6,
    lengthTolerance: 0,
    workingSegment: { start: [0.004, 0], end: [0.006, 0] },
    remainingGeometry: remainingGeometry(),
    featureMotion: {
      startPosition: [-0.001, 0], startAngle: 0,
      linearVelocityAffine: { intercept: [0.2, 0], slope: [0.03, 0] },
      angularVelocityAffine: { intercept: 0, slope: 0 },
    },
    materialMotion: {
      startPosition: [0, 0], startAngle: 0,
      linearVelocityAffine: { intercept: [0, 0], slope: [0, 0] },
      angularVelocityAffine: { intercept: 0, slope: 0 },
    },
  };
}
function measuredKernel(config) {
  const measurement = finiteRotation.measureNumericalScales(config);
  assert.equal(measurement.ok, true, measurement.reason);
  const kernel = finiteRotation.createFiniteRotationSweepKernel({
    ...config,
    finalCoordinateScaleUpperBound: measurement.finalCoordinateScaleUpperBound,
    transformOperandScaleUpperBound: measurement.transformOperandScaleUpperBound,
  });
  return { measurement, kernel };
}
function multiModeKktInput() {
  return {
    qFree: [2, 300, 0, 1, 0.6, 0],
    Minv: identity(6),
    materialContact: {
      id: 'weapon-material', point: { x: 0.1, y: 0.2 }, normalRow: embed([1, 2, 1]),
    },
    structuralContacts: [
      {
        id: 'fork-target', role: 'fork-target', point: { x: 1, y: 1 },
        normalRow: embed([0, 1, 0]), tangentRow: embed([1, 0, 0]),
        mu: 0.5, normalBias: 0,
      },
      {
        id: 'target-floor', role: 'target-floor', point: { x: 2, y: 0 },
        normalRow: embed([0, 0, 1]), tangentRow: embed([1, 0, 0]),
        mu: 0.3, normalBias: 0,
      },
    ],
    specificCuttingEnergy: 1,
    width: 1,
  };
}
function multiModeGeometry() {
  return {
    h: 2e-4,
    timeFractions: Array.from({ length: 9 }, (_, index) => index / 8),
    maximumTimeChordError: 5e-6,
    lengthTolerance: 1e-14,
    workingSegment: { start: [0.004, 0], end: [0.006, 0] },
    featureStartPosition: [-0.001, 0], featureStartAngle: 0,
    materialStartPosition: [0, 0], materialStartAngle: 0,
    remainingGeometry: remainingGeometry(),
  };
}
function createMultiMode(geometry = multiModeGeometry(), extra = {}) {
  return modeFamily.createModeFamilyGeometry({
    pDomain: [0, 2], kktInput: multiModeKktInput(), geometry, ...extra,
  });
}

const checks = [];
function check(name, run) {
  const started = Date.now();
  run();
  checks.push({ name, pass: true, elapsedMs: Date.now() - started });
}

check('both exact measured bounds are required and each nextDown is rejected', () => {
  const config = baseKernelConfig();
  const measurement = finiteRotation.measureNumericalScales(config);
  assert.equal(measurement.ok, true, measurement.reason);
  const accepted = finiteRotation.createFiniteRotationSweepKernel({
    ...config,
    finalCoordinateScaleUpperBound: measurement.finalCoordinateScaleUpperBound,
    transformOperandScaleUpperBound: measurement.transformOperandScaleUpperBound,
  });
  assert.equal(accepted.ok, true, accepted.reason);
  assert.ok(accepted.descriptor.computedFinalCoordinateScaleUpperBound
    > accepted.descriptor.finalCoordinateScaleUpperBound);
  const expectedPolygonGuard = finiteRotation._reference.multiplyUp(
    finiteRotation._reference.multiplyUp(
      finiteRotation._reference.multiplyUp(256, Number.EPSILON),
      accepted.descriptor.computedFinalCoordinateScaleUpperBound,
    ),
    accepted.descriptor.polygonOperationCount,
  );
  assert.equal(accepted.descriptor.polygonLengthGuard, expectedPolygonGuard);
  const lowFinal = finiteRotation.createFiniteRotationSweepKernel({
    ...config,
    finalCoordinateScaleUpperBound: nextDown(measurement.finalCoordinateScaleUpperBound),
    transformOperandScaleUpperBound: measurement.transformOperandScaleUpperBound,
  });
  assert.equal(lowFinal.ok, false);
  assert.match(lowFinal.reason, /finalCoordinateScaleUpperBound under-reports/);
  const lowTransform = finiteRotation.createFiniteRotationSweepKernel({
    ...config,
    finalCoordinateScaleUpperBound: measurement.finalCoordinateScaleUpperBound,
    transformOperandScaleUpperBound: nextDown(measurement.transformOperandScaleUpperBound),
  });
  assert.equal(lowTransform.ok, false);
  assert.match(lowTransform.reason, /transformOperandScaleUpperBound under-reports/);
  const missing = finiteRotation.createFiniteRotationSweepKernel(config);
  assert.equal(missing.ok, false);
  assert.match(missing.reason, /both split numerical scale upper bounds are required/);
});

check('NaN infinities and stale single-bound API are rejected', () => {
  const config = baseKernelConfig();
  const measurement = finiteRotation.measureNumericalScales(config);
  for (const field of ['finalCoordinateScaleUpperBound', 'transformOperandScaleUpperBound']) {
    for (const value of [NaN, Infinity, -Infinity]) {
      const candidate = {
        ...config,
        finalCoordinateScaleUpperBound: measurement.finalCoordinateScaleUpperBound,
        transformOperandScaleUpperBound: measurement.transformOperandScaleUpperBound,
        [field]: value,
      };
      const rejected = finiteRotation.createFiniteRotationSweepKernel(candidate);
      assert.equal(rejected.ok, false); assert.match(rejected.reason, /must be finite/);
    }
  }
  const stale = finiteRotation.createFiniteRotationSweepKernel({
    ...config,
    numericalCoordinateScaleUpperBound: measurement.transformOperandScaleUpperBound,
    finalCoordinateScaleUpperBound: measurement.finalCoordinateScaleUpperBound,
    transformOperandScaleUpperBound: measurement.transformOperandScaleUpperBound,
  });
  assert.equal(stale.ok, false); assert.match(stale.reason, /stale single/);
});

check('interior p rotation component is covered by derivative half-width inflation', () => {
  const config = baseKernelConfig();
  config.remainingGeometry = [];
  config.pDomain = [0, 1]; config.h = 1; config.timeFractions = [0, 1];
  config.workingSegment = { start: [1, 0], end: [1.1, 0] };
  config.featureMotion = {
    startPosition: [0, 0], startAngle: Math.PI / 2 - 0.0025,
    linearVelocityAffine: { intercept: [0, 0], slope: [0, 0] },
    angularVelocityAffine: { intercept: 0, slope: 0.005 },
  };
  const { measurement, kernel } = measuredKernel(config);
  assert.equal(kernel.ok, true, kernel.reason);
  const endpointMaximum = 1.1 * Math.cos(0.0025);
  assert.ok(measurement.finalCoordinateScaleUpperBound >= 1.1);
  assert.ok(measurement.finalCoordinateScaleUpperBound > endpointMaximum);
  const middle = kernel.discretePathSample(0.5);
  assert.equal(middle.ok, true, middle.reason);
  assert.ok(Math.abs(middle.nodes[1].end.y) <= measurement.finalCoordinateScaleUpperBound);
  for (let index = 0; index <= 1000; index += 1) {
    const sample = kernel.discretePathSample(index / 1000);
    assert.equal(sample.ok, true, sample.reason);
    for (const node of sample.nodes) for (const entry of [node.start, node.end]) {
      assert.ok(Math.abs(entry.x) <= measurement.finalCoordinateScaleUpperBound);
      assert.ok(Math.abs(entry.y) <= measurement.finalCoordinateScaleUpperBound);
    }
  }
  assert.ok(measurement.finalEndpointCertificates.some((entry) => entry.interiorInflation > 0));
});

check('common world offset raises only transform conditioning, not squared area scale', () => {
  const original = baseKernelConfig();
  const base = measuredKernel(original); assert.equal(base.kernel.ok, true, base.kernel.reason);
  const shifted = clone(original);
  shifted.featureMotion.startPosition[0] += 1e9;
  shifted.featureMotion.startPosition[1] += 1e9;
  shifted.materialMotion.startPosition[0] += 1e9;
  shifted.materialMotion.startPosition[1] += 1e9;
  const movedMeasurement = finiteRotation.measureNumericalScales(shifted);
  assert.equal(movedMeasurement.ok, true, movedMeasurement.reason);
  assert.ok(movedMeasurement.transformOperandScaleUpperBound > 1e9);
  assert.ok(movedMeasurement.finalCoordinateScaleUpperBound < 2
    * base.measurement.finalCoordinateScaleUpperBound);
  const areaScaleOnly = (scaleValue) => finiteRotation._reference.numericalAreaGuard({
    perimeter: 1, edgeCount: 4, coordinateCount: 10, lengthGuard: 0, scaleValue,
  });
  assert.ok(areaScaleOnly(movedMeasurement.finalCoordinateScaleUpperBound)
    < 4 * areaScaleOnly(base.measurement.finalCoordinateScaleUpperBound));
  const safelyStopped = finiteRotation.createFiniteRotationSweepKernel({
    ...shifted,
    finalCoordinateScaleUpperBound: movedMeasurement.finalCoordinateScaleUpperBound,
    transformOperandScaleUpperBound: movedMeasurement.transformOperandScaleUpperBound,
  });
  assert.equal(safelyStopped.ok, false);
  assert.match(safelyStopped.reason, /relative domain|relative-area cap/);

  const catastrophic = baseKernelConfig();
  catastrophic.remainingGeometry = [];
  catastrophic.h = 1; catastrophic.timeFractions = [0, 1];
  catastrophic.featureMotion.startPosition = [2 ** 52, 0];
  catastrophic.materialMotion.startPosition = [2 ** 52, 0];
  catastrophic.featureMotion.linearVelocityAffine = {
    intercept: [0.25, 0], slope: [0, 0],
  };
  const catastrophicMeasurement = finiteRotation.measureNumericalScales(catastrophic);
  assert.equal(catastrophicMeasurement.ok, true, catastrophicMeasurement.reason);
  const catastrophicTransformGuard = finiteRotation._reference.multiplyUp(
    finiteRotation._reference.gammaForOperationBudget(
      finiteRotation.TRANSFORM_ARITHMETIC_OPERATION_BUDGET,
    ),
    catastrophicMeasurement.transformOperandScaleUpperBound,
  );
  assert.ok(catastrophicTransformGuard > 0.25,
    'conditioning guard must cover common-offset cancellation loss');
});

check('floating p jump is enclosed by derivative radius plus twice transform guard', () => {
  const config = baseKernelConfig();
  config.remainingGeometry = [];
  config.pDomain = [0, Number.EPSILON]; config.h = 1; config.timeFractions = [0, 1];
  config.workingSegment = { start: [0, 0], end: [1e-3, 0] };
  config.featureMotion = {
    startPosition: [1, 0], startAngle: 0,
    linearVelocityAffine: { intercept: [0, 0], slope: [1, 0] },
    angularVelocityAffine: { intercept: 0, slope: 0 },
  };
  const { kernel } = measuredKernel(config); assert.equal(kernel.ok, true, kernel.reason);
  const interval = kernel.boundDiscretePathInterval(0, Number.EPSILON);
  assert.equal(interval.ok, true, interval.reason);
  const middle = kernel.discretePathSample(Number.EPSILON / 2);
  const right = kernel.discretePathSample(Number.EPSILON);
  assert.equal(middle.ok, true); assert.equal(right.ok, true);
  let observed = 0;
  for (let node = 0; node < middle.nodes.length; node += 1) {
    observed = Math.max(observed,
      distance(middle.nodes[node].start, right.nodes[node].start),
      distance(middle.nodes[node].end, right.nodes[node].end));
  }
  assert.ok(observed > 0, 'fixture must exercise a binary64 jump');
  for (const proof of interval.cellProofs) {
    assert.ok(proof.radius >= observed);
    assert.ok(proof.radius >= proof.derivativeRadius + proof.transformRoundoffDiameter
      || proof.radius > proof.derivativeRadius + proof.transformRoundoffDiameter);
    assert.ok(proof.transformRoundoffDiameter >= 2 * kernel.descriptor.transformLengthGuard);
  }
});

check('gamma overflow stops and positive underflow rounds up', () => {
  assert.throws(() => finiteRotation._reference.gammaForOperationBudget(Number.MAX_SAFE_INTEGER),
    /outside the finite domain/);
  assert.equal(finiteRotation._reference.multiplyUp(Number.MIN_VALUE, 0.5), Number.MIN_VALUE);
  assert.ok(finiteRotation._reference.gammaForOperationBudget(
    finiteRotation.TRANSFORM_ARITHMETIC_OPERATION_BUDGET,
  ) > 0);
});

check('mode family derives two independent common maxima and three signature layers', () => {
  const result = createMultiMode(); assert.equal(result.ok, true, result.reason);
  const descriptor = result.descriptor; const envelope = descriptor.envelope;
  assert.equal(new Set(envelope.map((entry) => entry.canonicalGeometrySignature)).size, 1);
  assert.equal(new Set(envelope.map((entry) => entry.commonNumericalPolicySignature)).size, 1);
  assert.equal(new Set(envelope.map((entry) => entry.geometrySnapshotSignature)).size, 1);
  assert.ok(new Set(envelope.map((entry) => entry.pathSignature)).size > 1);
  assert.equal(descriptor.finalCoordinateScaleUpperBound,
    Math.max(...envelope.map((entry) => entry.measuredFinalCoordinateScaleUpperBound)));
  assert.equal(descriptor.transformOperandScaleUpperBound,
    Math.max(...envelope.map((entry) => entry.measuredTransformOperandScaleUpperBound)));
  assert.equal(descriptor.trigProviderCertified, false);
  assert.equal(descriptor.productionNumericalAuthority, false);
});

check('caller cannot inject either split bound at family or geometry scope', () => {
  const attacks = [
    ['geometry', 'finalCoordinateScaleUpperBound'],
    ['geometry', 'transformOperandScaleUpperBoundByMode'],
    ['config', 'perModeFinalCoordinateScaleUpperBounds'],
    ['config', 'perModeTransformOperandScaleUpperBounds'],
    ['geometry', 'numericalCoordinateScaleUpperBound'],
  ];
  for (const [location, field] of attacks) {
    const geometry = multiModeGeometry(); const extra = {};
    (location === 'geometry' ? geometry : extra)[field] = 1;
    const rejected = createMultiMode(geometry, extra);
    assert.equal(rejected.ok, false); assert.match(rejected.reason, /caller-supplied split/);
  }
});

check('path amplification invalidates both stale bounds', () => {
  const original = baseKernelConfig();
  const first = finiteRotation.measureNumericalScales(original);
  const amplified = clone(original);
  amplified.featureMotion.startPosition[0] = 0.04;
  amplified.featureMotion.linearVelocityAffine.intercept[0] = 1e3;
  const second = finiteRotation.measureNumericalScales(amplified);
  assert.equal(first.ok, true); assert.equal(second.ok, true);
  assert.ok(second.finalCoordinateScaleUpperBound > first.finalCoordinateScaleUpperBound);
  assert.ok(second.transformOperandScaleUpperBound > first.transformOperandScaleUpperBound);
  const stale = finiteRotation.createFiniteRotationSweepKernel({
    ...amplified,
    finalCoordinateScaleUpperBound: first.finalCoordinateScaleUpperBound,
    transformOperandScaleUpperBound: first.transformOperandScaleUpperBound,
  });
  assert.equal(stale.ok, false); assert.match(stale.reason, /under-reports/);
});

check('identical inputs and descriptors are deterministic', () => {
  const left = createMultiMode(); const right = createMultiMode();
  assert.equal(left.ok, true, left.reason); assert.equal(right.ok, true, right.reason);
  assert.deepEqual(left.descriptor, right.descriptor);
  assert.equal(left.descriptor.modeFamilySignature, right.descriptor.modeFamilySignature);
});

const report = {
  schema: 'split-scale-mode-family-v2.1-focused-test-report-v1',
  pass: checks.every((entry) => entry.pass),
  passed: checks.length,
  total: checks.length,
  modulePins: {
    finiteRotationV2_1: finiteRotation.moduleSha256,
    modeFamilyV2_1: modeFamily.moduleSha256,
  },
  fixedPolicies: {
    transformArithmeticOperationBudget: finiteRotation.TRANSFORM_ARITHMETIC_OPERATION_BUDGET,
    trigProviderCertified: finiteRotation.TRIG_PROVIDER_CERTIFIED,
    productionNumericalAuthority: false,
  },
  checks,
};
const reportPath = path.join(__dirname, 'split-scale-test-report.json');
fs.writeFileSync(reportPath, `${JSON.stringify(report, null, 2)}\n`);
const reportHash = crypto.createHash('sha256').update(fs.readFileSync(reportPath))
  .digest('hex').toUpperCase();
process.stdout.write(`${JSON.stringify({ ...report, reportSha256: reportHash }, null, 2)}\n`);
