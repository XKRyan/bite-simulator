'use strict';

const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const subject = require('./exact-mode-family-geometry.js');

function identity(size) {
  return Array.from({ length: size }, (_, row) => Array.from({ length: size }, (_, column) => row === column ? 1 : 0));
}
function rng(seed = 0x71c3a9f5) {
  let state = seed >>> 0;
  return () => { state ^= state << 13; state ^= state >>> 17; state ^= state << 5; return (state >>> 0) / 0x100000000; };
}
const random = rng();
const quick = process.argv.includes('--quick');
const requestedWidthArgument = process.argv.find((entry) => entry.startsWith('--width='));
const requestedReportArgument = process.argv.find((entry) => entry.startsWith('--report='));
const requestedQuickWidth = requestedWidthArgument ? Number(requestedWidthArgument.slice('--width='.length)) : null;
if (requestedWidthArgument && !(requestedQuickWidth > 0)) throw new Error('--width must be positive');
const remainingGeometry = [[[[-0.012, -0.012], [0.012, -0.012], [0.012, 0.012],
  [-0.012, 0.012], [-0.012, -0.012]]]];
const remainingTriangles = [[[-0.012, -0.012], [0.012, -0.012], [0.012, 0.012]],
  [[-0.012, -0.012], [0.012, 0.012], [-0.012, 0.012]]];

function create(omega, verticalOffset) {
  return subject.createExactModeFamilyGeometry({
    remainingTriangles,
    modeFamilyConfig: {
      pDomain: [0, 0.4],
      kktInput: {
        qFree: [0.2, omega, 0, 0, 0, 0], Minv: identity(6),
        materialContact: { id: 'weapon-material', point: { x: 0, y: verticalOffset },
          normalRow: [1, 0, 0, 0, 0, 0] },
        structuralContacts: [], specificCuttingEnergy: 10000, width: 1,
      },
      geometry: {
        h: 0.001,
        timeFractions: Array.from({ length: 17 }, (_, index) => index / 16),
        maximumTimeChordError: 5e-6,
        lengthTolerance: 0,
        // A radial face produces non-folding annular-sector cells under pure
        // rotation; the independent vertical offset still perturbs its world path.
        workingSegment: { start: [0.002, 0], end: [0.008, 0] },
        featureStartPosition: [0, verticalOffset], featureStartAngle: 0,
        materialStartPosition: [0, 0], materialStartAngle: 0,
        remainingGeometry,
      },
    },
  });
}

let configurations = 0; let intervals = 0; let attemptedSamples = 0; let exactSamples = 0;
let topologyCertified = 0; let fullRange = 0; let safeStops = 0;
let maximumRelativeWidth = 0;
const topologyReasons = {};
for (let configuration = 0; configuration < (quick ? 2 : 24); configuration += 1) {
  const omega = (random() * 2 - 1) * 60;
  const verticalOffset = (random() * 2 - 1) * 0.002;
  const oracle = create(omega, verticalOffset);
  if (!oracle.ok) { safeStops += 1; continue; }
  configurations += 1;
  for (let intervalIndex = 0; intervalIndex < (quick ? 2 : 12); intervalIndex += 1) {
    const lo = random() * 0.14;
    const width = quick ? (requestedQuickWidth || (1e-9 + random() * 1e-8)) : (0.001 + random() * 0.03);
    const hi = Math.min(0.18, lo + width);
    if (!(hi > lo)) continue;
    const bound = oracle.boundFreshAreaInterval(lo, hi);
    if (!bound.ok) { safeStops += 1; continue; }
    intervals += 1;
    const statuses = bound.proof.pieces.map((piece) => piece.topologyProof.status);
    if (statuses.every((status) => status === 'cellwise-convexity-and-nonadjacent-separation')) topologyCertified += 1;
    else {
      fullRange += 1;
      for (const piece of bound.proof.pieces) if (piece.topologyProof.status === 'full-range-topology-unresolved') {
        const reason = piece.topologyProof.reason; topologyReasons[reason] = (topologyReasons[reason] || 0) + 1;
      }
    }
    maximumRelativeWidth = Math.max(maximumRelativeWidth,
      (bound.areaUpper - bound.areaLower) / Math.max(bound.areaUpper, 1e-30));
    const sampleCount = quick ? 4 : 24;
    for (let sampleIndex = 0; sampleIndex <= sampleCount; sampleIndex += 1) {
      attemptedSamples += 1;
      const p = lo + (hi - lo) * sampleIndex / sampleCount;
      const sample = oracle.exactSample(p);
      if (!sample.ok) { safeStops += 1; continue; }
      exactSamples += 1;
      assert.ok(sample.areaLower >= bound.areaLower - Number.MIN_VALUE,
        `lower leak cfg=${configuration} interval=${intervalIndex} p=${p}`);
      assert.ok(sample.areaUpper <= bound.areaUpper + Number.MIN_VALUE,
        `upper leak cfg=${configuration} interval=${intervalIndex} p=${p}`);
    }
  }
}
assert.ok(configurations >= (quick ? 1 : 20));
assert.ok(intervals >= (quick ? 1 : 200));
assert.ok(exactSamples >= (quick ? 1 : 4500));
assert.ok(topologyCertified > 0 || fullRange > 0);
assert.equal(safeStops, 0, `valid fixed-seed rotation audit had ${safeStops} unexpected domain stops`);
assert.equal(exactSamples, attemptedSamples, 'every attempted exact sample must be certified');
if (!quick) {
  assert.equal(configurations, 24); assert.equal(intervals, 288);
  assert.equal(exactSamples, 7200);
}

const report = { schema: 'exact-mode-family-rotation-random-audit-v1', pass: true,
  configurations, intervals, attemptedSamples, exactSamples, topologyCertified, fullRange, safeStops,
  maximumRelativeWidth, topologyReasons };
const serialized = `${JSON.stringify(report, null, 2)}\n`;
if (requestedReportArgument) {
  const outputPath = path.resolve(process.cwd(), requestedReportArgument.slice('--report='.length));
  fs.writeFileSync(outputPath, serialized);
}
process.stdout.write(serialized);
