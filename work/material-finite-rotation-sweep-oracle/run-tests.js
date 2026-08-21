'use strict';

const assert = require('node:assert/strict');
const crypto = require('node:crypto');
const fs = require('node:fs');
const path = require('node:path');
const subject = require('./finite-rotation-sweep-oracle.js');

function sha256(filename) {
  return crypto.createHash('sha256').update(fs.readFileSync(filename)).digest('hex').toUpperCase();
}
function rectangle(x0, y0, x1, y1) {
  return [[[x0, y0], [x1, y0], [x1, y1], [x0, y1], [x0, y0]]];
}
function baseConfig(overrides = {}) {
  return {
    pDomain: [0, 1],
    h: 2e-4,
    timeFractions: Array.from({ length: 9 }, (_, index) => index / 8),
    maximumTimeChordError: 5e-6,
    lengthTolerance: 1e-14,
    workingSegment: { start: [0, -1e-3], end: [0, 1e-3] },
    featureMotion: {
      startPosition: [-1e-3, 0], startAngle: 0,
      linearVelocityAffine: { intercept: [10, 0], slope: [2, 0] },
      angularVelocityAffine: { intercept: 0, slope: 0 },
    },
    materialMotion: {
      startPosition: [0, 0], startAngle: 0,
      linearVelocityAffine: { intercept: [0, 0], slope: [0, 0] },
      angularVelocityAffine: { intercept: 0, slope: 0 },
    },
    remainingGeometry: [rectangle(-1e-2, -1e-2, 1e-2, 1e-2)],
    ...overrides,
  };
}
function rotationConfig(overrides = {}) {
  return baseConfig({
    featureMotion: {
      startPosition: [-1e-3, 0], startAngle: 0.15,
      linearVelocityAffine: { intercept: [10, 0.2], slope: [2, -0.1] },
      angularVelocityAffine: { intercept: 500, slope: 100 },
    },
    ...overrides,
  });
}
function create(config) {
  const result = subject.createFiniteRotationSweepKernel(config);
  assert.equal(result.ok, true, result.reason);
  return result;
}
function approx(actual, expected, tolerance, label) {
  assert.ok(Math.abs(actual - expected) <= tolerance,
    `${label}: expected ${expected}, got ${actual}, tolerance ${tolerance}`);
}
function pointAt(motion, p, fraction, h) {
  const velocity = {
    x: motion.linearVelocityAffine.intercept[0] + motion.linearVelocityAffine.slope[0] * p,
    y: motion.linearVelocityAffine.intercept[1] + motion.linearVelocityAffine.slope[1] * p,
  };
  return {
    x: motion.startPosition[0] + velocity.x * h * fraction,
    y: motion.startPosition[1] + velocity.y * h * fraction,
  };
}
function angleAt(motion, p, fraction, h) {
  return motion.startAngle
    + (motion.angularVelocityAffine.intercept + motion.angularVelocityAffine.slope * p) * h * fraction;
}
function rotate(value, angle) {
  const c = Math.cos(angle); const s = Math.sin(angle);
  return { x: c * value.x - s * value.y, y: s * value.x + c * value.y };
}
function relativeEndpoint(config, p, fraction, local) {
  const fp = pointAt(config.featureMotion, p, fraction, config.h);
  const mp = pointAt(config.materialMotion, p, fraction, config.h);
  const fa = angleAt(config.featureMotion, p, fraction, config.h);
  const ma = angleAt(config.materialMotion, p, fraction, config.h);
  const worldLocal = rotate({ x: local[0], y: local[1] }, fa);
  return rotate({ x: fp.x + worldLocal.x - mp.x, y: fp.y + worldLocal.y - mp.y }, -ma);
}
function distance(a, b) { return Math.hypot(a.x - b.x, a.y - b.y); }

const checks = [];
function check(name, fn) {
  const started = Date.now();
  fn(); checks.push({ name, pass: true, elapsedMs: Date.now() - started });
}

check('module and polygon pins are live', () => {
  assert.match(subject.moduleSha256, /^[0-9A-F]{64}$/);
  assert.equal(subject.moduleSha256, sha256(path.resolve(__dirname, 'finite-rotation-sweep-oracle.js')));
  assert.equal(subject.EXPECTED_POLYGON_KERNEL_SHA256,
    sha256(path.resolve(__dirname, '../../outputs/bite-simulator/assets/polygon-clipping/polygon-clipping-0.15.7.umd.min.js')));
});

check('pure translation matches analytic swept area', () => {
  const kernel = create(baseConfig({ timeFractions: [0, 0.25, 0.5, 0.75, 1] }));
  for (const p of [0, 0.125, 0.5, 0.875, 1]) {
    const sample = kernel.exactSample(p);
    assert.equal(sample.ok, true, sample.reason);
    const expected = 2e-3 * (2e-4 * (10 + 2 * p));
    approx(sample.area, expected, Math.max(sample.numericalAreaGuard, 2e-15), `translation p=${p}`);
    assert.ok(Object.isFrozen(sample) && Object.isFrozen(sample.payload));
  }
});

check('translation interval encloses dense exact samples and their guards', () => {
  const kernel = create(baseConfig({ timeFractions: [0, 0.25, 0.5, 0.75, 1] }));
  for (let interval = 0; interval < 40; interval += 1) {
    const lo = (interval % 20) / 25;
    const hi = Math.min(1, lo + 0.04 + ((interval * 17) % 13) / 100);
    const bound = kernel.boundFreshAreaInterval(lo, hi);
    assert.equal(bound.ok, true, bound.reason);
    for (let index = 0; index <= 20; index += 1) {
      const p = lo + (hi - lo) * index / 20;
      const sample = kernel.exactSample(p);
      assert.equal(sample.ok, true, sample.reason);
      assert.ok(sample.areaLower >= bound.areaLower - 1e-18, `lower miss at ${p}`);
      assert.ok(sample.areaUpper <= bound.areaUpper + 1e-18, `upper miss at ${p}`);
      assert.ok(sample.numericalAreaGuard <= bound.proof.intervalSampleGuard);
    }
  }
});

check('finite rotation interval encloses randomized interior samples', () => {
  const kernel = create(rotationConfig());
  let state = 0x51f15e;
  const random = () => { state = (1664525 * state + 1013904223) >>> 0; return state / 2 ** 32; };
  for (let interval = 0; interval < 80; interval += 1) {
    const lo = 0.8 * random(); const hi = Math.min(1, lo + 0.015 + 0.18 * random());
    const bound = kernel.boundFreshAreaInterval(lo, hi);
    assert.equal(bound.ok, true, bound.reason);
    for (let index = 0; index < 17; index += 1) {
      const p = lo + (hi - lo) * random();
      const sample = kernel.exactSample(p);
      assert.equal(sample.ok, true, sample.reason);
      assert.ok(sample.areaLower >= bound.areaLower - 1e-18, `rotation lower miss at ${p}`);
      assert.ok(sample.areaUpper <= bound.areaUpper + 1e-18, `rotation upper miss at ${p}`);
    }
  }
});

check('counter-rotation remains enclosed', () => {
  const config = rotationConfig({
    materialMotion: {
      startPosition: [2e-4, -1e-4], startAngle: -0.2,
      linearVelocityAffine: { intercept: [-0.4, 0.3], slope: [0.2, -0.15] },
      angularVelocityAffine: { intercept: -260, slope: 70 },
    },
  });
  const kernel = create(config);
  for (const [lo, hi] of [[0, 0.1], [0.15, 0.45], [0.4, 0.75], [0.7, 1]]) {
    const bound = kernel.boundFreshAreaInterval(lo, hi);
    assert.equal(bound.ok, true, bound.reason);
    for (let index = 0; index <= 50; index += 1) {
      const sample = kernel.exactSample(lo + (hi - lo) * index / 50);
      assert.equal(sample.ok, true, sample.reason);
      assert.ok(sample.areaLower >= bound.areaLower - 1e-18);
      assert.ok(sample.areaUpper <= bound.areaUpper + 1e-18);
    }
  }
});

check('holes disconnected islands and re-entry preserve area identity', () => {
  const geometry = [
    [rectangle(-1e-2, -1e-2, 1e-2, 1e-2)[0], rectangle(-2e-3, -3e-3, 2e-3, 3e-3)[0]],
    rectangle(1.2e-2, -2e-3, 1.6e-2, 2e-3),
  ];
  const kernel = create(rotationConfig({ remainingGeometry: geometry }));
  for (const p of [0, 0.2, 0.5, 0.8, 1]) {
    const sample = kernel.exactSample(p);
    assert.equal(sample.ok, true, sample.reason);
    approx(sample.payload.intersectionArea, sample.payload.differenceArea,
      2 * sample.numericalAreaGuard, `area split p=${p}`);
    assert.ok(sample.payload.desiredGeometry.length >= 1);
  }
});

check('empty current geometry is exactly empty', () => {
  const kernel = create(rotationConfig({ remainingGeometry: [] }));
  const sample = kernel.exactSample(0.43);
  assert.equal(sample.ok, true); assert.equal(sample.area, 0);
  assert.equal(sample.areaLower, 0); assert.equal(sample.areaUpper, 0);
  const bound = kernel.boundFreshAreaInterval(0.2, 0.8);
  assert.equal(bound.ok, true); assert.equal(bound.areaLower, 0); assert.equal(bound.areaUpper, 0);
  assert.match(bound.intervalDigest, /^[0-9A-F]{64}$/);
});

check('recorded time-chord error encloses dense true endpoint deviations', () => {
  const config = rotationConfig({
    materialMotion: {
      startPosition: [2e-4, -1e-4], startAngle: -0.2,
      linearVelocityAffine: { intercept: [-0.4, 0.3], slope: [0.2, -0.15] },
      angularVelocityAffine: { intercept: -260, slope: 70 },
    },
  });
  const kernel = create(config); let maximum = 0;
  for (const p of [0, 0.2, 0.5, 0.8, 1]) {
    for (let cell = 0; cell < config.timeFractions.length - 1; cell += 1) {
      const s0 = config.timeFractions[cell]; const s1 = config.timeFractions[cell + 1];
      for (const local of [config.workingSegment.start, config.workingSegment.end]) {
        const a = relativeEndpoint(config, p, s0, local); const b = relativeEndpoint(config, p, s1, local);
        for (let index = 0; index <= 32; index += 1) {
          const u = index / 32; const actual = relativeEndpoint(config, p, s0 + (s1 - s0) * u, local);
          const chord = { x: a.x + (b.x - a.x) * u, y: a.y + (b.y - a.y) * u };
          maximum = Math.max(maximum, distance(actual, chord));
        }
      }
    }
  }
  assert.ok(maximum <= kernel.descriptor.timeChordError + 1e-15,
    `${maximum} exceeds ${kernel.descriptor.timeChordError}`);
});

check('time refinement shrinks the discrete sweep difference', () => {
  const sampleArea = (count) => create(rotationConfig({
    timeFractions: Array.from({ length: count }, (_, index) => index / (count - 1)),
  })).exactSample(0.63).area;
  const coarse = sampleArea(5); const medium = sampleArea(9); const fine = sampleArea(17);
  assert.ok(Math.abs(medium - fine) < Math.abs(coarse - medium),
    `refinement did not shrink: ${coarse}, ${medium}, ${fine}`);
});

check('folded or under-resolved time cells fail closed', () => {
  const tinyHalfLength = 1e-6;
  const folded = baseConfig({
    h: 1,
    timeFractions: [0, 1],
    workingSegment: { start: [-tinyHalfLength, 0], end: [tinyHalfLength, 0] },
    featureMotion: {
      startPosition: [0, 0], startAngle: 0,
      linearVelocityAffine: { intercept: [0, 0], slope: [0, 0] },
      angularVelocityAffine: { intercept: Math.PI, slope: 0 },
    },
  });
  const kernel = create(folded);
  const sample = kernel.exactSample(0.5);
  assert.equal(sample.ok, false);
  assert.match(sample.reason, /refinement|fold|unresolved/);

  const overRotation = subject.createFiniteRotationSweepKernel({
    ...folded,
    featureMotion: { ...folded.featureMotion,
      angularVelocityAffine: { intercept: 1.01 * Math.PI, slope: 0 } },
  });
  assert.equal(overRotation.ok, false);
  assert.match(overRotation.reason, /more than pi|chord error/);
});

check('malformed domains and insufficient chord budgets stop safely', () => {
  const cases = [
    baseConfig({ timeFractions: [0, 0.5, 0.5, 1] }),
    baseConfig({ timeFractions: [0.1, 1] }),
    baseConfig({ pDomain: [1, 0] }),
    baseConfig({ maximumTimeChordError: 5.1e-6 }),
    baseConfig({ h: 0 }),
    baseConfig({ featureMotion: null }),
    rotationConfig({ timeFractions: [0, 1], maximumTimeChordError: 1e-12 }),
  ];
  cases.forEach((config) => {
    const result = subject.createFiniteRotationSweepKernel(config);
    assert.equal(result.ok, false, JSON.stringify(config));
    assert.equal(result.status, 'solver-domain-stop');
  });
});

check('uniform geometric scaling preserves normalized area', () => {
  const areas = [];
  for (const factor of [1e-3, 1, 1e3]) {
    const scalePoint = (entry) => entry.map((value) => value * factor);
    const config = baseConfig({
      lengthTolerance: 1e-14 * factor,
      workingSegment: { start: scalePoint([0, -1e-3]), end: scalePoint([0, 1e-3]) },
      featureMotion: {
        startPosition: scalePoint([-1e-3, 0]), startAngle: 0,
        linearVelocityAffine: { intercept: scalePoint([10, 0]), slope: scalePoint([2, 0]) },
        angularVelocityAffine: { intercept: 0, slope: 0 },
      },
      materialMotion: {
        startPosition: [0, 0], startAngle: 0,
        linearVelocityAffine: { intercept: [0, 0], slope: [0, 0] },
        angularVelocityAffine: { intercept: 0, slope: 0 },
      },
      remainingGeometry: [rectangle(-1e-2 * factor, -1e-2 * factor,
        1e-2 * factor, 1e-2 * factor)],
    });
    const sample = create(config).exactSample(0.37);
    assert.equal(sample.ok, true, sample.reason);
    areas.push(sample.area / factor ** 2);
  }
  approx(areas[0], areas[1], 5e-15, 'small scale');
  approx(areas[2], areas[1], 5e-15, 'large scale');
});

check('deterministic outputs and non-authority scope remain explicit', () => {
  const kernel = create(rotationConfig());
  const first = kernel.exactSample(0.417); const second = kernel.exactSample(0.417);
  assert.equal(first.sampleDigest, second.sampleDigest);
  assert.deepEqual(first, second);
  assert.match(kernel.descriptor.authority, /^none/);
  assert.doesNotMatch(fs.readFileSync(path.resolve(__dirname, 'finite-rotation-sweep-oracle.js'), 'utf8'),
    /materialRemovalDefined\s*:\s*true|MAIN_TRANSACTION_WIRING_ENABLED\s*=\s*true/);
});

const report = {
  schema: 'finite-rotation-sweep-oracle-tests-v1',
  pass: checks.every((entry) => entry.pass),
  passed: checks.length,
  checks,
  sourceSha256: sha256(path.resolve(__dirname, 'finite-rotation-sweep-oracle.js')),
};
process.stdout.write(`${JSON.stringify(report, null, 2)}\n`);

