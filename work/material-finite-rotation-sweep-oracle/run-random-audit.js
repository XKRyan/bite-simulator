'use strict';

const assert = require('node:assert/strict');
const crypto = require('node:crypto');
const fs = require('node:fs');
const path = require('node:path');
const subject = require('./finite-rotation-sweep-oracle.js');

let state = 0x6d2b79f5;
function random() { state = (Math.imul(state, 1664525) + 1013904223) >>> 0; return state / 2 ** 32; }
function between(lo, hi) { return lo + (hi - lo) * random(); }
function rectangle(x0, y0, x1, y1) {
  return [[x0, y0], [x1, y0], [x1, y1], [x0, y1], [x0, y0]];
}
function configFor(index) {
  const geometry = index % 3 === 0
    ? [[rectangle(-0.012, -0.01, 0.012, 0.01), rectangle(-0.002, -0.003, 0.002, 0.003)],
      [rectangle(0.014, -0.002, 0.018, 0.002)]]
    : [[rectangle(-0.012, -0.01, 0.012, 0.01)]];
  return {
    pDomain: [0, 1], h: 2e-4,
    timeFractions: Array.from({ length: 9 }, (_, node) => node / 8),
    maximumTimeChordError: 5e-6, lengthTolerance: 1e-14,
    workingSegment: { start: [between(-2e-4, 2e-4), between(-1.5e-3, -7e-4)],
      end: [between(-2e-4, 2e-4), between(7e-4, 1.5e-3)] },
    featureMotion: {
      startPosition: [between(-0.004, -0.0003), between(-0.001, 0.001)],
      startAngle: between(-0.6, 0.6),
      linearVelocityAffine: { intercept: [between(4, 18), between(-2, 2)],
        slope: [between(-3, 3), between(-2, 2)] },
      angularVelocityAffine: { intercept: between(-700, 700), slope: between(-220, 220) },
    },
    materialMotion: {
      startPosition: [between(-3e-4, 3e-4), between(-3e-4, 3e-4)],
      startAngle: between(-0.3, 0.3),
      linearVelocityAffine: { intercept: [between(-1, 1), between(-1, 1)],
        slope: [between(-0.5, 0.5), between(-0.5, 0.5)] },
      angularVelocityAffine: { intercept: between(-350, 350), slope: between(-120, 120) },
    },
    remainingGeometry: geometry,
  };
}

let kernelsReady = 0; let exactSamples = 0; let certifiedIntervals = 0;
let safeStops = 0; let containmentChecks = 0; let maximumRelativeWidth = 0;
const stopReasons = new Map();
for (let configIndex = 0; configIndex < 160; configIndex += 1) {
  const kernel = subject.createFiniteRotationSweepKernel(configFor(configIndex));
  if (!kernel.ok) {
    safeStops += 1; stopReasons.set(kernel.reason, (stopReasons.get(kernel.reason) || 0) + 1); continue;
  }
  kernelsReady += 1;
  for (let intervalIndex = 0; intervalIndex < 10; intervalIndex += 1) {
    const lo = between(0, 0.94); const hi = Math.min(1, lo + between(0.003, 0.055));
    const bound = kernel.boundFreshAreaInterval(lo, hi);
    if (!bound.ok) {
      safeStops += 1; stopReasons.set(bound.reason, (stopReasons.get(bound.reason) || 0) + 1); continue;
    }
    certifiedIntervals += 1;
    maximumRelativeWidth = Math.max(maximumRelativeWidth,
      (bound.areaUpper - bound.areaLower) / Math.max(bound.areaUpper, 1e-30));
    for (let sampleIndex = 0; sampleIndex < 19; sampleIndex += 1) {
      const p = lo + (hi - lo) * random(); const sample = kernel.exactSample(p);
      if (!sample.ok) {
        safeStops += 1; stopReasons.set(sample.reason, (stopReasons.get(sample.reason) || 0) + 1); continue;
      }
      exactSamples += 1; containmentChecks += 1;
      assert.ok(sample.areaLower >= bound.areaLower - 1e-18,
        `lower miss config=${configIndex} interval=${intervalIndex} p=${p}`);
      assert.ok(sample.areaUpper <= bound.areaUpper + 1e-18,
        `upper miss config=${configIndex} interval=${intervalIndex} p=${p}`);
    }
  }
}

assert.ok(kernelsReady >= 120, `too many kernel-level stops: ${kernelsReady}`);
assert.ok(certifiedIntervals >= 1000, `too few certified intervals: ${certifiedIntervals}`);
assert.ok(containmentChecks >= 18000, `too few containment checks: ${containmentChecks}`);

const sourcePath = path.resolve(__dirname, 'finite-rotation-sweep-oracle.js');
const report = {
  schema: 'finite-rotation-sweep-random-audit-v1', pass: true,
  seed: '0x6d2b79f5', configurations: 160, kernelsReady, certifiedIntervals,
  exactSamples, containmentChecks, safeStops, maximumRelativeWidth,
  stopReasons: Object.fromEntries([...stopReasons.entries()].sort()),
  sourceSha256: crypto.createHash('sha256').update(fs.readFileSync(sourcePath)).digest('hex').toUpperCase(),
};
process.stdout.write(`${JSON.stringify(report, null, 2)}\n`);

