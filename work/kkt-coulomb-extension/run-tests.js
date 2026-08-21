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
} = require('./event-kkt-coulomb');

const tests = [];
function test(name, execute) { tests.push({ name, execute }); }
function near(actual, expected, tolerance = 1e-9, label = 'value') {
  assert.ok(Math.abs(actual - expected) <= tolerance, `${label}: ${actual} != ${expected}`);
}
function clone(value) { return JSON.parse(JSON.stringify(value)); }
function identity(size) {
  return Array.from({ length: size }, (_, row) => Array.from({ length: size }, (_, column) => row === column ? 1 : 0));
}

function oneContact(qFree, role = 'fork-target') {
  return {
    qFree,
    Minv: identity(2),
    materialContact: { id: 'material-normal', point: { x: 0, y: 0 }, normalRow: [1, 0] },
    structuralContacts: [{
      id: role, role, point: { x: 1.25, y: -0.5 },
      normalRow: [1, 0], tangentRow: [0, 1], mu: 0.5,
    }],
    specificCuttingEnergy: 1,
    width: 1,
    freshArea: () => 0.1,
  };
}

const multiContact = {
  qFree: [2, 1, 0.6],
  Minv: identity(3),
  materialContact: { id: 'weapon-material', point: { x: 0.1, y: 0.2 }, normalRow: [1, 2, 1] },
  structuralContacts: [
    {
      id: 'fork-target', role: 'fork-target', point: { x: 1, y: 1 },
      normalRow: [0, 1, 0], tangentRow: [1, 0, 0], mu: 0.5,
    },
    {
      id: 'target-floor', role: 'target-floor', point: { x: 2, y: 0 },
      normalRow: [0, 0, 1], tangentRow: [1, 0, 0], mu: 0.3,
    },
  ],
  specificCuttingEnergy: 1,
  width: 1,
  freshArea: () => ({ area: 2, payload: null }),
};

test('stick mode uses one common point and remains inside the Coulomb cone', () => {
  const result = solvePrescribedImpulse(oneContact([-1, 0.2]), 0);
  assert.equal(result.ok, true, result.reason);
  assert.equal(result.modeKey, 'fork-target:stick');
  near(result.qPost[0], 0);
  near(result.qPost[1], 0);
  near(result.contactImpulses[0].normalImpulse, 1);
  near(result.contactImpulses[0].tangentImpulse, -0.2);
  assert.deepEqual(result.contactImpulses[0].point, { x: 1.25, y: -0.5 });
  assert.deepEqual(result.contactPointBindings[1].point, result.contactImpulses[0].point);
  near(result.contactImpulses[0].coneResidual, 0);
  near(result.contactImpulses[0].maximumDissipationSignResidual, 0);
  assert.ok(result.kineticEnergy <= 0.5 * (1 + 0.2 ** 2));
  assert.deepEqual(result.materialFriction, { defined: false, coefficient: 0, mode: 'none-explicit' });
});

test('positive slide is the maximum-dissipation edge for negative tangential velocity', () => {
  const result = solvePrescribedImpulse(oneContact([-1, -2], 'target-floor'), 0);
  assert.equal(result.ok, true, result.reason);
  assert.equal(result.modeKey, 'target-floor:+slide');
  near(result.qPost[0], 0);
  near(result.qPost[1], -1.5);
  near(result.contactImpulses[0].normalImpulse, 1);
  near(result.contactImpulses[0].tangentImpulse, 0.5);
  near(Math.abs(result.contactImpulses[0].tangentImpulse), 0.5 * result.contactImpulses[0].normalImpulse);
  assert.ok(result.contactImpulses[0].tangentImpulse * result.contactImpulses[0].tangentVelocityPost <= 0);
});

test('negative slide is the maximum-dissipation edge for positive tangential velocity', () => {
  const result = solvePrescribedImpulse(oneContact([-1, 2], 'fork-floor'), 0);
  assert.equal(result.ok, true, result.reason);
  assert.equal(result.modeKey, 'fork-floor:-slide');
  near(result.qPost[0], 0);
  near(result.qPost[1], 1.5);
  near(result.contactImpulses[0].normalImpulse, 1);
  near(result.contactImpulses[0].tangentImpulse, -0.5);
  assert.ok(result.contactImpulses[0].tangentImpulse * result.contactImpulses[0].tangentVelocityPost <= 0);
});

test('maximum dissipation resolves a stick-slide boundary deterministically', () => {
  const result = solvePrescribedImpulse(oneContact([-1, 0.5]), 0);
  assert.equal(result.ok, true, result.reason);
  assert.ok(result.maximumDissipation.feasibleCandidateCount >= 2);
  const minimum = Math.min(...result.maximumDissipation.candidateKineticEnergies.map((entry) => entry.kineticEnergy));
  near(result.kineticEnergy, minimum, 1e-13, 'minimum post KE');
  const repeated = solvePrescribedImpulse(oneContact([-1, 0.5]), 0);
  assert.deepEqual(repeated, result);
});

test('two contacts switch modes and D(p) is integrated on exact affine segments', () => {
  const result = solvePrescribedImpulse(multiContact, 1);
  assert.equal(result.ok, true, result.reason);
  assert.equal(result.modeKey, 'fork-target:-slide|target-floor:-slide');
  assert.equal(result.modeTransitions, 2);
  assert.equal(result.workSegments.length, 3);
  near(result.workSegments[0].end, 0.5, 2e-11, 'first activation');
  near(result.workSegments[1].end, 0.6, 2e-11, 'second activation');
  near(result.dissipatedWork, 2.031, 2e-11, 'piecewise D(1)');
  near(result.qPost[0], 0.38, 2e-12);
  near(result.qPost[1], 0);
  near(result.qPost[2], 0);
  assert.ok(result.structuralFrictionDissipation >= -1e-12);
  result.contactImpulses.forEach((contact) => {
    assert.ok(Math.abs(contact.tangentImpulse) <= contact.mu * contact.normalImpulse + 1e-12);
    assert.ok(contact.tangentImpulse * contact.tangentVelocityPost <= 1e-12);
  });
});

test('prepared root binds the same p, KKT state, modes, points and geometry payload', () => {
  const callbackTrials = [];
  const input = {
    ...multiContact,
    freshArea: (p, trial) => {
      callbackTrials.push(trial);
      return {
        area: 2,
        payload: {
          p,
          qPost: trial.qPost,
          modeKey: trial.modeKey,
          materialPoint: trial.materialPoint,
          structuralPoints: trial.contactImpulses.map((entry) => entry.point),
        },
      };
    },
  };
  const prepared = prepareMaterialEvent(input);
  assert.equal(prepared.ok, true, prepared.reason);
  assert.equal(prepared.p, prepared.lambda);
  near(prepared.dissipatedWork, 2, 2e-9);
  near(prepared.materialWork, 2, 1e-14);
  near(prepared.geometryPayload.p, prepared.p, 1e-14);
  assert.deepEqual(prepared.geometryPayload.qPost, prepared.qPost);
  assert.equal(prepared.geometryPayload.modeKey, prepared.modeKey);
  assert.deepEqual(prepared.geometryPayload.materialPoint, { x: 0.1, y: 0.2 });
  assert.deepEqual(prepared.geometryPayload.structuralPoints,
    prepared.contactImpulses.map((entry) => entry.point));
  assert.equal(prepared.modeTransitions, 2);
  assert.equal(prepared.energyAudit.nonIncreasingFromFree, true);
  assert.ok(prepared.energyAudit.structuralFrictionDissipation >= 0);
  near(prepared.energyAudit.balanceResidual, 0, 1e-12);
  assert.ok(callbackTrials.length > 2);
  assert.ok(callbackTrials.every((trial) => trial.p === trial.lambda));
  assert.equal(validatePreparedEvent(prepared).ok, true);
});

test('mixed-p or mixed-point commit is refused before mutation', () => {
  const prepared = prepareMaterialEvent(multiContact);
  assert.equal(prepared.ok, true, prepared.reason);
  const state = { untouched: true };
  let snapshots = 0;
  const transaction = {
    snapshot: () => { snapshots += 1; return clone(state); },
    restore: (snapshot) => Object.assign(state, clone(snapshot)),
    applyBoundState: () => { state.untouched = false; },
  };
  const mixedP = clone(prepared);
  mixedP.p += 1e-3;
  const pRefused = commitPreparedEvent(mixedP, transaction);
  assert.equal(pRefused.status, 'binding-refused');
  assert.equal(snapshots, 0);
  assert.deepEqual(state, { untouched: true });

  const mixedPoint = clone(prepared);
  mixedPoint.contactPointBindings[1].point.x += 0.01;
  const pointRefused = commitPreparedEvent(mixedPoint, transaction);
  assert.equal(pointRefused.status, 'binding-refused');
  assert.equal(snapshots, 0);
  assert.deepEqual(state, { untouched: true });
});

test('fault after partial commit rolls every serialisable byte back', () => {
  const prepared = prepareMaterialEvent(multiContact);
  assert.equal(prepared.ok, true, prepared.reason);
  const state = {
    p: 9, q: [4, 5, 6], modeKey: 'old', points: [{ x: 7, y: 8 }],
    damage: { version: 3 }, history: [{ event: 'old' }],
  };
  const before = _reference.canonical(state);
  const transaction = {
    snapshot: () => clone(state),
    restore: (snapshot) => {
      Object.keys(state).forEach((key) => delete state[key]);
      Object.assign(state, clone(snapshot));
    },
    applyBoundState: (bound) => {
      state.p = bound.p;
      state.q = bound.qPost.slice();
      state.modeKey = bound.modeKey;
      state.points = clone(bound.contactPointBindings);
      state.damage.version += 1;
      state.history.push({ event: 'partial' });
      throw new Error('injected collider rebuild fault');
    },
  };
  const result = commitPreparedEvent(prepared, transaction);
  assert.equal(result.ok, false);
  assert.equal(result.status, 'commit-failed-rolled-back');
  assert.equal(result.rolledBack, true);
  assert.equal(result.rollbackExact, true);
  assert.equal(_reference.canonical(state), before);
});

test('incompatible unilateral contacts return an explicit no-feasible-solution domain stop', () => {
  const input = {
    qFree: [-0.5, 0], Minv: identity(2),
    materialContact: { point: { x: 0, y: 0 }, normalRow: [0, 1] },
    structuralContacts: [
      { id: 'left', role: 'fork-target', point: { x: 0, y: 0 }, normalRow: [1, 0], tangentRow: [0, 1], mu: 0 },
      { id: 'right', role: 'target-floor', point: { x: 1, y: 0 }, normalRow: [-1, 0], tangentRow: [0, 1], normalBias: -1, mu: 0 },
    ],
    specificCuttingEnergy: 1, width: 1, freshArea: () => 0.1,
  };
  const result = solvePrescribedImpulse(input, 0);
  assert.equal(result.ok, false);
  assert.equal(result.status, 'solver-domain-stop');
  assert.match(result.reason, /no nonsingular|no feasible/);
});

test('mode candidate upper bound is an explicit domain stop, never truncation', () => {
  const dimension = 7;
  const input = {
    qFree: Array(dimension).fill(1), Minv: identity(dimension),
    materialContact: { point: { x: 0, y: 0 }, normalRow: [1, 0, 0, 0, 0, 0, 0] },
    structuralContacts: Array.from({ length: 7 }, (_, index) => ({
      id: `contact-${index}`, role: index % 3 === 0 ? 'fork-target' : index % 3 === 1 ? 'target-floor' : 'fork-floor',
      point: { x: index, y: 0 },
      normalRow: Array.from({ length: dimension }, (_, axis) => axis === index ? 1 : 0),
      tangentRow: Array.from({ length: dimension }, (_, axis) => axis === (index + 1) % dimension ? 1 : 0),
      mu: 0.2,
    })),
    specificCuttingEnergy: 1, width: 1, freshArea: () => 0.1,
    options: { maximumContacts: 7, maximumModeCandidates: 4096 },
  };
  const result = solvePrescribedImpulse(input, 0);
  assert.equal(result.ok, false);
  assert.equal(result.status, 'solver-domain-stop');
  assert.match(result.reason, /candidate upper bound 16384 exceeds/);
  assert.doesNotMatch(result.reason, /truncat/i);
});

test('material tangential friction is never guessed from structural coefficients', () => {
  const explicitNone = solvePrescribedImpulse(oneContact([-1, 0.2]), 0);
  assert.equal(explicitNone.ok, true);
  assert.deepEqual(explicitNone.materialFriction, { defined: false, coefficient: 0, mode: 'none-explicit' });
  const unsupported = oneContact([-1, 0.2]);
  unsupported.materialContact = { ...unsupported.materialContact, mu: 0.4, tangentRow: [0, 1] };
  const refused = solvePrescribedImpulse(unsupported, 0);
  assert.equal(refused.ok, false);
  assert.equal(refused.status, 'solver-domain-stop');
  assert.match(refused.reason, /material tangential friction is not defined/);
});

test('same input is byte-deterministic including segmentation and binding signature', () => {
  const first = prepareMaterialEvent(multiContact);
  const second = prepareMaterialEvent(multiContact);
  assert.equal(first.ok, true, first.reason);
  assert.deepEqual(second, first);
  assert.equal(Object.isFrozen(first), true);
  assert.equal(Object.isFrozen(first.contactImpulses), true);
  assert.equal(Object.isFrozen(first.contactPointBindings), true);
});

test('seeded coupled systems preserve cone, maximum-dissipation and no-KE-gain invariants', () => {
  let seed = 0x00c0ffee;
  const random = () => {
    seed = (Math.imul(seed, 1664525) + 1013904223) >>> 0;
    return seed / 0x100000000;
  };
  for (let scenario = 0; scenario < 120; scenario += 1) {
    const diagonalX = 0.5 + random() * 1.5;
    const diagonalY = 0.5 + random() * 1.5;
    const coupling = (random() - 0.5) * 0.4 * Math.sqrt(diagonalX * diagonalY);
    const input = {
      qFree: [-0.1 - random() * 2, (random() - 0.5) * 6],
      Minv: [[diagonalX, coupling], [coupling, diagonalY]],
      materialContact: {
        point: { x: 0, y: 0 },
        normalRow: [0.3 + random(), (random() - 0.5) * 0.2],
      },
      structuralContacts: [{
        id: 'fork-target', role: 'fork-target', point: { x: 1, y: 2 },
        normalRow: [1, 0], tangentRow: [0, 1], mu: 0.02 + random(),
      }],
      specificCuttingEnergy: 1, width: 1, freshArea: () => 0.1,
    };
    const p = random() * 0.15;
    const result = solvePrescribedImpulse(input, p);
    assert.equal(result.ok, true, `seeded scenario ${scenario}: ${result.reason}`);
    const problem = _reference.buildProblem(input);
    assert.ok(result.kineticEnergy <= problem.kineticEnergyFree + 3e-9, `KE gain in scenario ${scenario}`);
    const contact = result.contactImpulses[0];
    assert.ok(contact.normalImpulse >= -1e-12);
    assert.ok(contact.coneResidual <= 2e-9);
    assert.ok(contact.maximumDissipationSignResidual <= 2e-9);
    const minimum = Math.min(...result.maximumDissipation.candidateKineticEnergies.map((entry) => entry.kineticEnergy));
    near(result.kineticEnergy, minimum, 1e-12, `maximum dissipation scenario ${scenario}`);
  }
});

const report = { suite: 'kkt-coulomb-extension', passed: 0, failed: 0, tests: [] };
for (const entry of tests) {
  const started = process.hrtime.bigint();
  try {
    entry.execute();
    report.passed += 1;
    report.tests.push({ name: entry.name, ok: true, durationMs: Number(process.hrtime.bigint() - started) / 1e6 });
  } catch (error) {
    report.failed += 1;
    report.tests.push({ name: entry.name, ok: false, error: error.stack || error.message,
      durationMs: Number(process.hrtime.bigint() - started) / 1e6 });
  }
}
fs.writeFileSync(path.join(__dirname, 'test-report.json'), `${JSON.stringify(report, null, 2)}\n`);
console.log(JSON.stringify(report, null, 2));
if (report.failed) process.exitCode = 1;
