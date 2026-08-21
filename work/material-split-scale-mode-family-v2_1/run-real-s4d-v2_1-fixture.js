'use strict';

const assert = require('node:assert/strict');
const crypto = require('node:crypto');
const fs = require('node:fs');
const path = require('node:path');
const coulomb = require('../kkt-coulomb-extension/event-kkt-coulomb.js');
const finiteRotation = require('./finite-rotation-sweep-oracle-v2_1.js');
const modeFamily = require('./mode-family-geometry-v2_1.js');
const exactModeFamily = require('./exact-mode-family-geometry-v2_1.js');

const fixturePath = path.join(__dirname, 'real-s4d-three-mode-fixture.json');
const reportPath = path.join(__dirname, 'real-s4d-v2_1-direct-report.json');
const EXPECTED_FIXTURE_SHA256 = '028DE291979843D5CE7B885E7A1BF44DAFBEAA18080A8E40A43688CF9DD063F1';
const sha256 = (bytes) => crypto.createHash('sha256').update(bytes).digest('hex').toUpperCase();
const clone = (value) => JSON.parse(JSON.stringify(value));
const fixtureBytes = fs.readFileSync(fixturePath);
assert.equal(sha256(fixtureBytes), EXPECTED_FIXTURE_SHA256);
const fixture = JSON.parse(fixtureBytes);

const solverInput = {
  ...clone(fixture.kktInput),
  freshArea: () => ({ area: 0, payload: null }),
};
const problem = coulomb._reference.buildProblem(solverInput);
const envelope = problem.workSegments(fixture.pDomain[1]);
assert.ok(Array.isArray(envelope?.segments) && envelope.segments.length > 0);
const modeRanges = new Map();
let cursor = fixture.pDomain[0];
const orderedEnvelope = [];
for (const [index, segment] of envelope.segments.entries()) {
  const low = Number(segment.start); const high = Number(segment.end);
  assert.equal(low, cursor); assert.ok(high > low && high <= fixture.pDomain[1]);
  const selected = problem.solveAt(low + (high - low) / 2);
  assert.ok(selected?._mode); assert.equal(selected.modeKey, segment.modeKey);
  for (const fraction of [0.25, 0.5, 0.75]) {
    const probe = problem.solveAt(low + (high - low) * fraction);
    assert.equal(probe._mode, selected._mode); assert.equal(probe.modeKey, selected.modeKey);
  }
  const range = modeRanges.get(selected._mode) || {
    mode: selected._mode, modeKey: selected.modeKey,
    modeOrdinal: selected._mode.ordinal, low, high,
  };
  range.low = Math.min(range.low, low); range.high = Math.max(range.high, high);
  modeRanges.set(selected._mode, range);
  orderedEnvelope.push({ index, start: low, end: high,
    modeKey: selected.modeKey, modeOrdinal: selected._mode.ordinal });
  cursor = high;
}
assert.equal(cursor, fixture.pDomain[1]);

function kernelConfig(range, finalBound, transformBound) {
  const mode = range.mode; const geometry = fixture.geometry;
  const config = {
    pDomain: [range.low, range.high],
    h: geometry.h,
    timeFractions: geometry.timeFractions,
    maximumTimeChordError: geometry.maximumTimeChordError,
    lengthTolerance: geometry.lengthTolerance,
    workingSegment: geometry.workingSegment,
    remainingGeometry: geometry.remainingGeometry,
    featureMotion: {
      startPosition: geometry.featureStartPosition,
      startAngle: geometry.featureStartAngle,
      linearVelocityAffine: {
        intercept: [mode.qIntercept[0], 0], slope: [mode.qSlope[0], 0],
      },
      angularVelocityAffine: { intercept: mode.qIntercept[1], slope: mode.qSlope[1] },
    },
    materialMotion: {
      startPosition: geometry.materialStartPosition,
      startAngle: geometry.materialStartAngle,
      linearVelocityAffine: {
        intercept: [mode.qIntercept[3], mode.qIntercept[4]],
        slope: [mode.qSlope[3], mode.qSlope[4]],
      },
      angularVelocityAffine: { intercept: mode.qIntercept[5], slope: mode.qSlope[5] },
    },
  };
  if (finalBound !== undefined || transformBound !== undefined) {
    assert.notEqual(finalBound, undefined); assert.notEqual(transformBound, undefined);
    config.finalCoordinateScaleUpperBound = finalBound;
    config.transformOperandScaleUpperBound = transformBound;
  }
  return config;
}

const measurements = [...modeRanges.values()].map((range) => {
  const measurement = finiteRotation.measureNumericalScales(kernelConfig(range));
  assert.equal(measurement.ok, true, measurement.reason);
  return { range, measurement };
});
const commonFinalBound = Math.max(...measurements
  .map((entry) => entry.measurement.finalCoordinateScaleUpperBound));
const commonTransformBound = Math.max(...measurements
  .map((entry) => entry.measurement.transformOperandScaleUpperBound));
const kernels = measurements.map(({ range, measurement }) => {
  const result = finiteRotation.createFiniteRotationSweepKernel(
    kernelConfig(range, commonFinalBound, commonTransformBound),
  );
  return {
    modeKey: range.modeKey,
    modeOrdinal: range.modeOrdinal,
    pRange: [range.low, range.high],
    measuredFinalCoordinateScaleUpperBound: measurement.finalCoordinateScaleUpperBound,
    finalCoordinateScalePolicy: measurement.finalCoordinateScalePolicy,
    measuredTransformOperandScaleUpperBound: measurement.transformOperandScaleUpperBound,
    transformOperandScalePolicy: measurement.transformOperandScalePolicy,
    scaleMeasurementSignature: measurement.measurementSignature,
    commonFinalBound,
    commonTransformBound,
    computedFinalCoordinateScaleUpperBound:
      result.ok ? result.descriptor.computedFinalCoordinateScaleUpperBound : null,
    ok: result.ok,
    reason: result.ok ? null : result.reason,
    canonicalGeometrySignature: result.ok ? result.descriptor.canonicalGeometrySignature : null,
    commonNumericalPolicySignature:
      result.ok ? result.descriptor.commonNumericalPolicySignature : null,
    geometrySnapshotSignature: result.ok ? result.descriptor.geometrySnapshotSignature : null,
    pathSignature: result.ok ? result.descriptor.pathSignature : null,
    lengthGuard: result.ok ? result.descriptor.lengthGuard : null,
    polygonLengthGuard: result.ok ? result.descriptor.polygonLengthGuard : null,
    transformLengthGuard: result.ok ? result.descriptor.transformLengthGuard : null,
    accumulatedNumericalLengthFloor:
      result.ok ? result.descriptor.accumulatedNumericalLengthFloor : null,
    geometryAreaGuard: result.ok ? result.descriptor.geometryAreaGuard : null,
    geometryReferenceArea: result.ok ? result.descriptor.geometryReferenceArea : null,
    relativeAreaCapLimit: result.ok ? result.descriptor.relativeAreaCapLimit : null,
    relativeAreaCapMargin: result.ok ? result.descriptor.relativeAreaCapMargin : null,
    trigProviderCertified: result.ok ? result.descriptor.trigProviderCertified : null,
    authority: result.ok ? result.descriptor.authority : null,
    productionWiring: result.ok ? result.descriptor.productionWiring : null,
  };
});
const family = modeFamily.createModeFamilyGeometry({
  pDomain: fixture.pDomain,
  kktInput: fixture.kktInput,
  geometry: fixture.geometry,
});
const exact = exactModeFamily.createExactModeFamilyGeometry({
  remainingTriangles: fixture.remainingTriangles,
  modeFamilyConfig: {
    pDomain: fixture.pDomain,
    kktInput: fixture.kktInput,
    geometry: fixture.geometry,
  },
});
const successfulKernels = kernels.filter((entry) => entry.ok);
const commonComputedFinalBound = successfulKernels.length
  ? successfulKernels[0].computedFinalCoordinateScaleUpperBound : null;
const count = (field) => new Set(successfulKernels.map((entry) => entry[field])).size;
const relativeCapSafeStop = kernels.length === 3 && kernels.every((entry) => (
  entry.ok === false && /geometry normalization guard exceeds the relative-area cap/.test(entry.reason)
));
const report = {
  schema: 'real-s4d-three-mode-split-scale-v2.1-direct-report-v1',
  pass: family.ok === true && exact.ok === true && successfulKernels.length === 3,
  status: relativeCapSafeStop ? 'certified-domain-stop-at-fixed-relative-area-cap'
    : (family.ok && exact.ok ? 'v2.1-real-three-mode-ready' : 'unexpected-failure'),
  fixtureSha256: EXPECTED_FIXTURE_SHA256,
  sourcePins: {
    finiteRotationV2_1: finiteRotation.moduleSha256,
    modeFamilyV2_1: modeFamily.moduleSha256,
    exactModeFamilyV2_1: exactModeFamily.moduleSha256,
    coulombV2: coulomb.moduleSha256,
  },
  orderedEnvelope,
  modeCount: modeRanges.size,
  commonFinalCoordinateScaleUpperBound: commonFinalBound,
  commonComputedFinalCoordinateScaleUpperBound: commonComputedFinalBound,
  commonTransformOperandScaleUpperBound: commonTransformBound,
  boundOwner: 'EDD-v2.1 takes two independent internal maxima over all full-envelope mode measurements',
  kernels,
  signatureCounts: {
    availableSuccessfulKernels: successfulKernels.length,
    canonicalGeometrySignature: count('canonicalGeometrySignature'),
    commonNumericalPolicySignature: count('commonNumericalPolicySignature'),
    compositeGeometrySnapshotSignature: count('geometrySnapshotSignature'),
    pathSignature: count('pathSignature'),
    unavailableBecauseRelativeCapSafeStop: relativeCapSafeStop,
  },
  relativeCapSafeStop,
  modeFamilyResult: family.ok ? { ok: true, descriptor: family.descriptor }
    : { ok: false, status: family.status, reason: family.reason,
      moduleSha256: family.moduleSha256 },
  exactModeFamilyResult: exact.ok ? { ok: true, descriptor: exact.descriptor }
    : { ok: false, status: exact.status, reason: exact.reason,
      moduleSha256: exact.moduleSha256 },
  scope: {
    mathematicalFixtureOnly: true,
    noRapierStep: true,
    noQWrite: true,
    noGeometryCommit: true,
    noRemoval: true,
    main: false,
    trigProviderCertified: false,
    authority: 'none',
    productionWiring: false,
  },
};
fs.writeFileSync(reportPath, `${JSON.stringify(report, null, 2)}\n`, 'utf8');
process.stdout.write(`${JSON.stringify({
  pass: report.pass,
  status: report.status,
  commonFinalBound,
  commonComputedFinalBound,
  commonTransformBound,
  modes: kernels.map((entry) => ({
    modeKey: entry.modeKey,
    measuredFinalCoordinateScaleUpperBound: entry.measuredFinalCoordinateScaleUpperBound,
    measuredTransformOperandScaleUpperBound: entry.measuredTransformOperandScaleUpperBound,
    ok: entry.ok,
    reason: entry.reason,
  })),
  signatureCounts: report.signatureCounts,
  modeFamilyReason: report.modeFamilyResult.reason || null,
  exactModeFamilyReason: report.exactModeFamilyResult.reason || null,
  scope: report.scope,
  reportSha256: sha256(fs.readFileSync(reportPath)),
}, null, 2)}\n`);
if (!report.pass) process.exitCode = 1;
