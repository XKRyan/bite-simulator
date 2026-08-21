'use strict';

const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const {
  prepareMaterialEvent,
  solvePrescribedImpulse,
  validatePreparedEvent,
  commitPreparedEvent,
  _reference,
} = require('./event-kkt');

const tests = [];
function test(name, execute) { tests.push({ name, execute }); }
function near(actual, expected, tolerance = 1e-9, label = 'value') {
  assert.ok(Math.abs(actual - expected) <= tolerance, `${label}: ${actual} != ${expected}`);
}
function clone(value) { return JSON.parse(JSON.stringify(value)); }

const base = {
  qFree: [1, 1],
  Minv: [[1, 0], [0, 1]],
  structuralRows: [{ id: 'clamp-y', row: [0, 1], bias: 0 }],
  materialRow: [1, 2],
  specificCuttingEnergy: 1,
  width: 1,
  freshArea: () => ({ area: 0.95, payload: null }),
};

test('prescribed material impulse is simultaneous with structural clamping', () => {
  const result = solvePrescribedImpulse(base, 0.75);
  assert.equal(result.ok, true);
  near(result.qPost[0], 0.25);
  near(result.qPost[1], 0);
  assert.deepEqual(result.activeIds, ['clamp-y']);
  near(result.structuralImpulses[0].impulse, 0.5);
  assert.ok(result.structuralGaps[0].value >= -1e-10);

  // A structural solve followed by an unprojected material impulse would be
  // [0.25, -0.5].  The reference state is deliberately not that two-pass state.
  const sequentialWrong = [1 - 0.75, 1 - 2 * 0.75];
  assert.ok(sequentialWrong[1] < 0);
  assert.notDeepEqual(result.qPost, sequentialWrong);
});

test('piecewise material work crosses an active-set switch exactly', () => {
  const result = solvePrescribedImpulse(base, 1);
  assert.equal(result.ok, true);
  near(result.dissipatedWork, 1, 2e-10, 'D(1)');
  assert.equal(result.workSegments.length, 2);
  near(result.workSegments[0].end, 0.5, 2e-10, 'switch lambda');
  assert.deepEqual(result.workSegments[0].activeIds, []);
  assert.deepEqual(result.workSegments[1].activeIds, ['clamp-y']);
});

test('work root binds the same lambda, qPost, active IDs and geometry payload', () => {
  const callbackKeys = [];
  const input = {
    ...base,
    freshArea: (lambda, trial) => {
      callbackKeys.push(Object.keys(trial).sort());
      assert.equal(Object.hasOwn(trial, 'meanVelocity'), false);
      assert.equal(Object.hasOwn(trial, 'qBaseline'), false);
      return { area: 0.95, payload: { lambda, x: trial.qPost[0], activeIds: trial.activeIds } };
    },
  };
  const prepared = prepareMaterialEvent(input);
  assert.equal(prepared.ok, true, prepared.reason);
  const expectedLambda = 1 - Math.sqrt(0.1);
  near(prepared.lambda, expectedLambda, 2e-9, 'root lambda');
  near(prepared.dissipatedWork, 0.95, 2e-9, 'root work');
  near(prepared.materialWork, 0.95, 2e-9, 'material work');
  near(prepared.geometryPayload.lambda, prepared.lambda, 1e-14, 'payload lambda');
  near(prepared.geometryPayload.x, prepared.qPost[0], 1e-14, 'payload qPost');
  assert.deepEqual(prepared.geometryPayload.activeIds, prepared.activeIds);
  assert.deepEqual(prepared.activeIds, ['clamp-y']);
  assert.ok(prepared.activeSetTransitions >= 1);
  assert.ok(callbackKeys.length > 2);
  assert.ok(callbackKeys.every((keys) => !keys.includes('meanVelocity')));
  assert.equal(validatePreparedEvent(prepared).ok, true);
});

test('unaffordable virgin-area slice is refused with a slice-only diagnostic', () => {
  const result = prepareMaterialEvent({
    ...base,
    freshArea: () => ({ area: 1.2, payload: { untouched: true } }),
  });
  assert.equal(result.ok, false);
  assert.equal(result.status, 'unaffordable-slice');
  assert.equal(result.retryableBySliceReduction, true);
  near(result.maximumDissipatableWork, 1, 2e-9);
  near(result.maximumAffordableFreshArea, 1, 2e-9);
  near(result.suggestedMaximumAreaFraction, 1 / 1.2, 2e-9);
  assert.equal(Object.hasOwn(result, 'scaledImpulse'), false);
});

test('non-monotone geometry response is a domain stop, not an adjusted impulse', () => {
  const result = prepareMaterialEvent({
    ...base,
    freshArea: (lambda) => 0.5 + 0.1 * lambda,
  });
  assert.equal(result.ok, false);
  assert.equal(result.status, 'solver-domain-stop');
  assert.match(result.reason, /grew|non-increasing/);
});

test('zero-bias event is energy non-increasing and work closes the energy drop', () => {
  const prepared = prepareMaterialEvent(base);
  assert.equal(prepared.ok, true, prepared.reason);
  assert.equal(prepared.energyAudit.zeroBias, true);
  assert.equal(prepared.energyAudit.nonIncreasingFromFree, true);
  assert.ok(prepared.energyAudit.kineticEnergyPost <= prepared.energyAudit.kineticEnergyAtZero + 1e-10);
  near(prepared.energyAudit.energyDropFromZero, prepared.dissipatedWork, 2e-9, 'energy/work identity');
  near(prepared.energyAudit.materialWorkIdentityResidual, 0, 2e-9, 'energy residual');
});

test('commit receives the prepared state directly and rejects a mixed-p binding', () => {
  const prepared = prepareMaterialEvent(base);
  assert.equal(prepared.ok, true);
  const state = { q: [9, 9], lambda: null, activeIds: [], signature: null, geometry: null };
  const transaction = {
    snapshot: () => clone(state),
    restore: (snapshot) => Object.assign(state, clone(snapshot)),
    applyBoundState: (bound) => {
      state.q = bound.qPost.slice();
      state.lambda = bound.lambda;
      state.activeIds = bound.activeIds.slice();
      state.signature = bound.bindingSignature;
      state.geometry = clone(bound.geometryPayload);
    },
    observe: () => ({ lambda: state.lambda, qPost: state.q, activeIds: state.activeIds, bindingSignature: state.signature }),
  };
  const committed = commitPreparedEvent(prepared, transaction);
  assert.equal(committed.ok, true, committed.reason);
  assert.deepEqual(state.q, prepared.qPost);
  assert.equal(state.lambda, prepared.lambda);

  const mixed = clone(prepared);
  mixed.lambda += 1e-3;
  const before = clone(state);
  const refused = commitPreparedEvent(mixed, transaction);
  assert.equal(refused.ok, false);
  assert.equal(refused.status, 'binding-refused');
  assert.deepEqual(state, before);
});

test('fault after partial commit rolls every byte of serialisable state back', () => {
  const prepared = prepareMaterialEvent(base);
  assert.equal(prepared.ok, true);
  const state = {
    q: [4, 5], lambda: 7, activeIds: ['old'], signature: 'old',
    damage: { version: 3, loops: [[0, 1, 2]] }, history: [{ event: 'old' }],
  };
  const before = _reference.canonical(state);
  const transaction = {
    snapshot: () => clone(state),
    restore: (snapshot) => {
      Object.keys(state).forEach((key) => delete state[key]);
      Object.assign(state, clone(snapshot));
    },
    applyBoundState: (bound) => {
      state.q = bound.qPost.slice();
      state.lambda = bound.lambda;
      state.damage.version += 1;
      state.history.push({ event: 'partial' });
      throw new Error('injected geometry rebuild fault');
    },
  };
  const result = commitPreparedEvent(prepared, transaction);
  assert.equal(result.ok, false);
  assert.equal(result.status, 'commit-failed-rolled-back');
  assert.equal(result.rolledBack, true);
  assert.equal(result.rollbackExact, true);
  assert.equal(_reference.canonical(state), before);
});

test('same input is deterministic including root, segmentation and signature', () => {
  const first = prepareMaterialEvent(base);
  const second = prepareMaterialEvent(base);
  assert.equal(first.ok, true);
  assert.deepEqual(second, first);
  assert.equal(Object.isFrozen(first), true);
  assert.equal(Object.isFrozen(first.qPost), true);
});

test('seeded coupled systems preserve the zero-bias work/energy identity through switches', () => {
  let seed = 0x5eed1234;
  const random = () => {
    seed = (Math.imul(seed, 1664525) + 1013904223) >>> 0;
    return seed / 0x100000000;
  };
  let checked = 0;
  for (let scenario = 0; scenario < 80; scenario += 1) {
    const basis = Array.from({ length: 3 }, () => Array.from({ length: 3 }, () => random() * 1.2 - 0.6));
    const Minv = Array.from({ length: 3 }, (_, row) => Array.from({ length: 3 }, (_, column) => {
      let value = row === column ? 0.4 : 0;
      for (let index = 0; index < 3; index += 1) value += basis[index][row] * basis[index][column];
      return value;
    }));
    const input = {
      qFree: Array.from({ length: 3 }, () => random() * 2 - 0.6),
      Minv,
      structuralRows: [
        { id: 'fork-target', row: [0, 1, 0], bias: 0 },
        { id: 'target-floor', row: [0, 0, 1], bias: 0 },
        { id: 'fork-floor', row: [1, 0.35, 0.15], bias: 0 },
      ],
      materialRow: Array.from({ length: 3 }, () => random() * 1.5 - 0.25),
      specificCuttingEnergy: 1,
      width: 1,
      freshArea: () => 0.1,
    };
    let problem;
    try { problem = _reference.buildProblem(input); } catch { continue; }
    const zero = problem.solveAt(0);
    if (!(zero.materialSpeed > 1e-7)) continue;
    let upper = 1e-5;
    let stopped = null;
    for (let bracket = 0; bracket < 50; bracket += 1) {
      let trial;
      try { trial = problem.solveAt(upper); } catch { break; }
      if (trial.materialSpeed <= 0) { stopped = upper; break; }
      upper *= 2;
    }
    if (!stopped) continue;
    const lambda = stopped * (0.2 + random() * 0.6);
    const trial = problem.solveAt(lambda);
    if (trial.materialSpeed <= 1e-8) continue;
    const integrated = problem.workSegments(lambda);
    const energyDrop = zero.kineticEnergy - trial.kineticEnergy;
    near(integrated.work, energyDrop, 3e-8, `seeded energy identity ${scenario}`);
    integrated.segments.forEach((segment) => {
      assert.ok(segment.end > segment.start);
      assert.ok(segment.work >= -1e-12);
    });
    checked += 1;
  }
  assert.ok(checked >= 20, `only ${checked} seeded systems were checkable`);
});

const report = { suite: 'material-kkt-event', passed: 0, failed: 0, tests: [] };
for (const entry of tests) {
  const started = process.hrtime.bigint();
  try {
    entry.execute();
    report.passed += 1;
    report.tests.push({ name: entry.name, ok: true, durationMs: Number(process.hrtime.bigint() - started) / 1e6 });
  } catch (error) {
    report.failed += 1;
    report.tests.push({ name: entry.name, ok: false, error: error.stack || error.message, durationMs: Number(process.hrtime.bigint() - started) / 1e6 });
  }
}
fs.writeFileSync(path.join(__dirname, 'test-report.json'), `${JSON.stringify(report, null, 2)}\n`);
console.log(JSON.stringify(report, null, 2));
if (report.failed) process.exitCode = 1;
