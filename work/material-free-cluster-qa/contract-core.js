'use strict';

const assert = require('node:assert/strict');
const crypto = require('node:crypto');
const fs = require('node:fs');
const path = require('node:path');
const coulomb = require('../kkt-coulomb-extension/event-kkt-coulomb.js');
const normalKkt = require('../material-kkt-event/event-kkt.js');
const {
  clone,
  canonical,
  identity,
  dot,
  makeFreshArea,
  makeConfig,
  createWorldAdapter,
  prepareAcceptedTrial,
} = require('./fixture.js');

const SOURCE_LOCKS = Object.freeze([
  {
    id: 's2-candidate',
    file: path.resolve(__dirname, '..', 'material-event-integrator-design', 'app-candidate.js'),
    sha256: '6E8E638407ED1B4F1A3A9F3C97A1E5C191AB6AE8D843CCB1622679E48312C320',
  },
  {
    id: 'signed-coulomb-kkt',
    file: path.resolve(__dirname, '..', 'kkt-coulomb-extension', 'event-kkt-coulomb.js'),
    sha256: '8B3058E5833E96D7C83971B6D0C234E6641D0CB98044012DFFFA5DA2268E1A3E',
  },
  {
    id: 'signed-normal-kkt',
    file: path.resolve(__dirname, '..', 'material-kkt-event', 'event-kkt.js'),
    sha256: '866BE26C0A0DF18D114B1026EEC08C943E6D9F672265F7DB0B49C10C28836719',
  },
]);

const REQUIRED_FAULT_STAGES = Object.freeze([
  'advance:before-mask',
  'advance:after-mask',
  'adapter:during-mask-write',
  'adapter:during-group-restore',
  'adapter:before-world-step',
  'adapter:after-world-step',
  'advance:after-world',
  'advance:after-free-capture',
  'trial:before-solve',
  'trial:after-solve',
  'finish:before-validate',
  'finish:after-validate',
  'finish:before-writeback',
  'adapter:before-q-write',
  'adapter:after-q-write',
  'finish:after-writeback',
  'finish:before-ticket',
  'finish:after-ticket',
  'finish:before-consume',
  'structural:before-solve',
  'structural:after-solve',
  'structural:before-writeback',
  'structural:after-writeback',
  'structural:before-consume',
]);

const TEST_NAMES = Object.freeze({
  locks: 'frozen source locks and signed normal/Coulomb overlap',
  advance: 'advanceFree owns one world step, one force integration and frozen Wact',
  branchStructural: 'one Sfree cannot finish material after structural finish',
  branchMaterial: 'one Sfree cannot finish structural after material finish',
  dry: 'dry lambda trial is deterministic and byte-pure',
  frame: 'contact rows can only come from the frozen Sfree contactFrame',
  minv: 'inverse mass is frozen in Sfree contactFrame and missing Minv is refused',
  accepted: 'finishMaterial preserves accepted same-p rows modes qPost and ticket',
  tamper: 'mixed accepted p rows modes or qPost are refused before mutation',
  h: 'one h binds free Moreau work and geometry payload',
  faults: 'all declared exception stages restore exposed state and solverGroups',
  onset: 'onset restitution is applied once',
  persistent: 'persistent contact takes ownership without restitution replay',
  domain: 'Coulomb candidate upper bound is a pure solver-domain-stop',
  deterministic: 'identical Sfree inputs produce identical trials',
});

function sha256(file) {
  return crypto.createHash('sha256').update(fs.readFileSync(file)).digest('hex').toUpperCase();
}

function near(actual, expected, tolerance = 1e-11, label = 'value') {
  assert.ok(Number.isFinite(actual), `${label} is not finite`);
  assert.ok(Math.abs(actual - expected) <= tolerance, `${label}: ${actual} != ${expected}`);
}

function moduleFaultStages(moduleUnderTest) {
  return moduleUnderTest.faultStages || moduleUnderTest.FAULT_STAGES || [];
}

function sessionConfig(config, adapter) {
  return {
    solvePrescribedImpulse: coulomb.solvePrescribedImpulse,
    prepareMaterialEvent: coulomb.prepareMaterialEvent,
    canonicalExposed: canonical,
    freshArea: config.freshArea,
    maximumContacts: config.maximumContacts,
    maximumModeCandidates: config.maximumModeCandidates,
    maskPlan: ['weapon-target', 'fork-target', 'target-floor', 'fork-floor'],
    maskSolverGroups: clone(config.maskSolverGroups),
    faultInjector: (stage) => adapter.hit(stage),
  };
}

function makeHarness(moduleUnderTest, overrides = {}) {
  assert.equal(typeof moduleUnderTest.createFreeClusterSession, 'function', 'createFreeClusterSession export is required');
  assert.equal(typeof moduleUnderTest.solveClusterFromFree, 'function', 'solveClusterFromFree export is required');
  assert.equal(typeof moduleUnderTest.canonicalExposed, 'function', 'canonicalExposed export is required');
  const config = makeConfig(overrides);
  const q0 = overrides.q0 || [2, 1, 0.6];
  const adapter = createWorldAdapter({
    q0,
    Minv: config.Minv,
    externalForce: overrides.externalForce || Array(q0.length).fill(0),
    actuationForce: overrides.actuationForce || overrides.externalForce || Array(q0.length).fill(0),
    solverGroups: overrides.solverGroups,
    materialContact: config.materialContact,
    structuralContacts: config.structuralContacts,
    specificCuttingEnergy: config.specificCuttingEnergy,
    width: config.width,
  });
  const session = moduleUnderTest.createFreeClusterSession(adapter, sessionConfig(config, adapter));
  return { moduleUnderTest, config, adapter, session };
}

function advance(harness, event = {}) {
  return harness.session.advanceFree({ h: harness.config.h, event });
}

function acceptedDry(harness, token) {
  const prepared = prepareAcceptedTrial(token, harness.config);
  assert.equal(prepared.ok, true, prepared.reason);
  assert.ok(prepared.p > 0, 'fixture must have a positive material event');
  const trial = harness.session.trialMaterial(token, { lambda: prepared.p, dry: true });
  assert.equal(trial.ok, true, trial.reason);
  return trial;
}

function exposed(adapter) {
  const state = typeof adapter.readExposedState === 'function' ? adapter.readExposedState() : adapter.snapshot();
  const trace = typeof adapter.readTrace === 'function' ? adapter.readTrace() : null;
  return canonical({ state, trace });
}

function expectRefused(execute, messagePattern = /consum|bind|frozen|refus|stale|foreign|row/i) {
  try {
    const result = execute();
    assert.ok(result && result.ok === false, 'operation unexpectedly succeeded');
    if (result.reason) assert.match(result.reason, messagePattern);
    return result;
  } catch (error) {
    assert.match(String(error.message || error), messagePattern);
    return error;
  }
}

function frozenWactProbe(token) {
  const expected = token.Wact;
  try { token.Wact = expected + 123; } catch (_) { /* immutable objects may throw in strict mode */ }
  assert.equal(token.Wact, expected, 'observable token.Wact is mutable');
}

function prepareTwoAxisHarness(moduleUnderTest, phase) {
  const h = 0.1;
  const structuralContacts = [{
    id: 'fork-target-restitution', role: 'fork-target', point: { x: 0, y: 0 },
    normalRow: [1, 0], tangentRow: [0, 1], mu: 0,
    gap: 0, phase, restitution: 0.25, preNormalVelocity: -2,
  }];
  return makeHarness(moduleUnderTest, {
    h,
    q0: [-2, 0],
    Minv: identity(2),
    externalForce: [0, 0],
    materialContact: { id: 'weapon-material', point: { x: 0, y: 0 }, normalRow: [0, 1] },
    structuralContacts,
    freshArea: makeFreshArea(h),
  });
}

function domainHarness(moduleUnderTest) {
  const dimension = 7;
  const contacts = Array.from({ length: dimension }, (_, index) => ({
    id: `contact-${index}`,
    role: index % 3 === 0 ? 'fork-target' : index % 3 === 1 ? 'target-floor' : 'fork-floor',
    point: { x: index, y: 0 },
    normalRow: Array.from({ length: dimension }, (_, axis) => axis === index ? 1 : 0),
    tangentRow: Array.from({ length: dimension }, (_, axis) => axis === (index + 1) % dimension ? 1 : 0),
    mu: 0.2,
    gap: 0,
    phase: 'persistent',
    restitution: 0,
    preNormalVelocity: 1,
  }));
  return makeHarness(moduleUnderTest, {
    h: 0.1,
    q0: Array(dimension).fill(1),
    Minv: identity(dimension),
    externalForce: Array(dimension).fill(0),
    materialContact: {
      id: 'weapon-material', point: { x: 0, y: 0 },
      normalRow: Array.from({ length: dimension }, (_, axis) => axis === 0 ? 1 : 0),
    },
    structuralContacts: contacts,
    maximumContacts: 7,
    maximumModeCandidates: 4096,
    freshArea: makeFreshArea(0.1),
  });
}

function buildTests(moduleUnderTest) {
  return [
    {
      name: TEST_NAMES.locks,
      execute() {
        SOURCE_LOCKS.forEach((entry) => assert.equal(sha256(entry.file), entry.sha256, `${entry.id} hash drift`));
        const baseNormal = {
          qFree: [1, 1], Minv: identity(2),
          structuralRows: [{ id: 'clamp-y', row: [0, 1], bias: 0 }],
          materialRow: [1, 2], specificCuttingEnergy: 1, width: 1, freshArea: () => 1,
        };
        const baseCoulomb = {
          qFree: [1, 1], Minv: identity(2),
          materialContact: { id: 'm', point: { x: 0, y: 0 }, normalRow: [1, 2] },
          structuralContacts: [{
            id: 'clamp-y', role: 'fork-target', point: { x: 0, y: 0 },
            normalRow: [0, 1], tangentRow: [1, 0], mu: 0, normalBias: 0,
          }],
          specificCuttingEnergy: 1, width: 1, freshArea: () => 1,
        };
        const normal = normalKkt.solvePrescribedImpulse(baseNormal, 0.75);
        const frictionless = coulomb.solvePrescribedImpulse(baseCoulomb, 0.75);
        assert.equal(normal.ok, true, normal.reason);
        assert.equal(frictionless.ok, true, frictionless.reason);
        assert.deepEqual(frictionless.qPost, normal.qPost);
        near(frictionless.dissipatedWork, normal.dissipatedWork, 1e-13, 'signed KKT overlap work');
      },
    },
    {
      name: TEST_NAMES.advance,
      execute() {
        const force = [0.5, -0.25, 0.1];
        const harness = makeHarness(moduleUnderTest, { externalForce: force, actuationForce: force });
        const beforeGroups = harness.adapter.readSolverGroups();
        const token = advance(harness);
        const state = harness.adapter.getState();
        assert.equal(state.counters.worldSteps, 1);
        assert.equal(state.counters.externalForceIntegrations, 1);
        assert.equal(state.counters.actuationWorkIntegrations, 1);
        assert.equal(token.stepEpoch, 1);
        assert.equal(token.forceEpoch, 1);
        assert.deepEqual(harness.adapter.readSolverGroups(), beforeGroups);
        const expectedQ = [2, 1, 0.6].map((value, index) => value + harness.config.h * force[index]);
        expectedQ.forEach((value, index) => near(token.qFree[index], value, 1e-14, `qFree[${index}]`));
        const impulse = force.map((value) => value * harness.config.h);
        const midpoint = expectedQ.map((value, index) => 0.5 * (value + [2, 1, 0.6][index]));
        near(token.Wact, dot(impulse, midpoint), 1e-14, 'Wact');
        frozenWactProbe(token);
        const countersBeforeDry = clone(state.counters);
        const first = harness.session.trialMaterial(token, { lambda: 0, dry: true });
        const second = harness.session.trialMaterial(token, { lambda: 0, dry: true });
        assert.deepEqual(first, second);
        assert.deepEqual(harness.adapter.getState().counters, countersBeforeDry);
        assert.equal(token.Wact, dot(impulse, midpoint));
      },
    },
    {
      name: TEST_NAMES.branchStructural,
      execute() {
        const harness = makeHarness(moduleUnderTest);
        const token = advance(harness);
        const dry = acceptedDry(harness, token);
        const structural = harness.session.finishNoMaterial(token);
        assert.equal(structural.ok, true, structural.reason);
        const after = exposed(harness.adapter);
        expectRefused(() => harness.session.finishMaterial(token, { acceptedTrial: dry }), /consum|stale|finish/i);
        assert.equal(exposed(harness.adapter), after, 'refused second finish mutated state');
      },
    },
    {
      name: TEST_NAMES.branchMaterial,
      execute() {
        const harness = makeHarness(moduleUnderTest);
        const token = advance(harness);
        const dry = acceptedDry(harness, token);
        const material = harness.session.finishMaterial(token, { acceptedTrial: dry });
        assert.equal(material.ok, true, material.reason);
        const after = exposed(harness.adapter);
        expectRefused(() => harness.session.finishNoMaterial(token), /consum|stale|finish/i);
        assert.equal(exposed(harness.adapter), after, 'refused second finish mutated state');
      },
    },
    {
      name: TEST_NAMES.dry,
      execute() {
        const harness = makeHarness(moduleUnderTest);
        const token = advance(harness);
        const prepared = prepareAcceptedTrial(token, harness.config);
        assert.equal(prepared.ok, true, prepared.reason);
        const before = exposed(harness.adapter);
        const first = harness.session.trialMaterial(token, { lambda: prepared.p, dry: true });
        const middle = exposed(harness.adapter);
        const second = harness.session.trialMaterial(token, { lambda: prepared.p, dry: true });
        assert.equal(middle, before, 'first dry trial changed exposed state');
        assert.equal(exposed(harness.adapter), before, 'second dry trial changed exposed state');
        assert.deepEqual(second, first);
        assert.equal(first.p, first.lambda);
      },
    },
    {
      name: TEST_NAMES.frame,
      execute() {
        const harness = makeHarness(moduleUnderTest);
        const token = advance(harness);
        const before = exposed(harness.adapter);
        expectRefused(() => harness.session.trialMaterial(token, {
          lambda: 0,
          dry: true,
          structuralContacts: clone(harness.config.structuralContacts),
        }), /contactFrame|frozen|row/i);
        assert.equal(exposed(harness.adapter), before);
        expectRefused(() => harness.session.finishNoMaterial(token, {
          structuralContacts: clone(harness.config.structuralContacts),
        }), /contactFrame|frozen|row/i);
        assert.equal(exposed(harness.adapter), before);
        harness.session.abort(token);
      },
    },
    {
      name: TEST_NAMES.accepted,
      execute() {
        const harness = makeHarness(moduleUnderTest);
        const token = advance(harness);
        const dry = acceptedDry(harness, token);
        const geometryBefore = harness.adapter.getState().geometry;
        const result = harness.session.finishMaterial(token, { acceptedTrial: dry });
        assert.equal(result.ok, true, result.reason);
        ['p', 'lambda', 'modeKey', 'acceptedBindingSignature'].forEach((key) => assert.deepEqual(result[key], dry[key], key));
        ['qPost', 'rowBindings', 'contactStates', 'contactPointBindings', 'workSegments', 'hAudit', 'geometryPayload']
          .forEach((key) => assert.deepEqual(result[key], dry[key], key));
        assert.deepEqual(harness.adapter.getState().q, dry.qPost, 'writeback did not use accepted qPost');
        assert.equal(result.Wact, token.Wact, 'finish recomputed or changed frozen Wact');
        assert.deepEqual(harness.adapter.getState().geometry, geometryBefore, 'free-cluster stage committed geometry early');
        const ticket = result.commitTicket;
        assert.ok(ticket, 'commit ticket is required');
        assert.equal(ticket.p, dry.p);
        assert.equal(ticket.h, token.h);
        assert.equal(ticket.contactFrameSignature, token.contactFrameSignature);
        assert.equal(ticket.acceptedBindingSignature, dry.acceptedBindingSignature);
        assert.equal(ticket.Wact, token.Wact);
        assert.deepEqual(ticket.qPost, dry.qPost);
        assert.deepEqual(ticket.rowBindings, dry.rowBindings);
        assert.deepEqual(ticket.geometryPayload, dry.geometryPayload);
      },
    },
    {
      name: TEST_NAMES.tamper,
      execute() {
        const harness = makeHarness(moduleUnderTest);
        const token = advance(harness);
        const dry = acceptedDry(harness, token);
        const mutations = [
          (value) => { value.p += 1e-4; value.lambda = value.p; },
          (value) => { value.rowBindings[0].normalRow[0] += 1e-4; },
          (value) => { value.modeKey += '|tampered'; },
          (value) => { value.qPost[0] += 1e-4; },
        ];
        for (const mutate of mutations) {
          const mixed = clone(dry);
          mutate(mixed);
          const before = exposed(harness.adapter);
          expectRefused(() => harness.session.finishMaterial(token, { acceptedTrial: mixed }), /accept|bind|signature|same-p|mismatch/i);
          assert.equal(exposed(harness.adapter), before, 'mixed accepted state mutated before refusal');
        }
        harness.session.abort(token);
      },
    },
    {
      name: TEST_NAMES.h,
      execute() {
        const h = 0.2;
        const contacts = clone(makeConfig().structuralContacts);
        contacts[0].gap = -0.02;
        const harness = makeHarness(moduleUnderTest, {
          h,
          structuralContacts: contacts,
          freshArea: makeFreshArea(h),
        });
        const token = advance(harness);
        const trial = harness.session.trialMaterial(token, { lambda: 0, dry: true });
        assert.equal(trial.ok, true, trial.reason);
        assert.deepEqual(trial.hAudit, { free: h, moreau: h, work: h, geometry: h });
        near(trial.rowBindings[0].normalBias, Math.max(contacts[0].gap, 0) / h, 1e-14, 'Moreau max(gap,0)/h bias');
        assert.equal(contacts[0].gap < 0, true, 'fixture must exercise negative persistent gap');
        near(trial.rowBindings[0].normalBias, 0, 1e-14, 'negative persistent gap zero bias');
        assert.equal(trial.rowBindings[0].h, h);
        assert.equal(trial.geometryPayload.h, h);
        assert.equal(token.h, h);
      },
    },
    {
      name: TEST_NAMES.minv,
      execute() {
        const harness = makeHarness(moduleUnderTest);
        const token = advance(harness);
        const prepared = prepareAcceptedTrial(token, harness.config);
        assert.equal(prepared.ok, true, prepared.reason);
        const first = harness.session.trialMaterial(token, { lambda: prepared.p, dry: true });
        assert.equal(first.ok, true, first.reason);
        harness.adapter.qaSetMinv([
          [9, 0, 0],
          [0, 7, 0],
          [0, 0, 5],
        ]);
        const mutatedLiveMass = exposed(harness.adapter);
        const second = harness.session.trialMaterial(token, { lambda: prepared.p, dry: true });
        assert.deepEqual(second, first, 'trial read live adapter Minv instead of frozen contactFrame.Minv');
        assert.equal(exposed(harness.adapter), mutatedLiveMass, 'dry trial mutated state after live mass change');
        const missing = makeHarness(moduleUnderTest);
        const missingFrame = {
          materialContact: clone(missing.config.materialContact),
          structuralContacts: clone(missing.config.structuralContacts),
          specificCuttingEnergy: missing.config.specificCuttingEnergy,
          width: missing.config.width,
        };
        const missingRoot = exposed(missing.adapter);
        expectRefused(() => advance(missing, { contactFrame: missingFrame }), /Minv|inverse mass|contactFrame/i);
        assert.equal(exposed(missing.adapter), missingRoot, 'missing frame Minv advance refusal did not restore root exactly');
        harness.session.abort(token);
      },
    },
    {
      name: TEST_NAMES.faults,
      execute() {
        const declared = new Set(moduleFaultStages(moduleUnderTest));
        REQUIRED_FAULT_STAGES.forEach((stage) => assert.ok(declared.has(stage), `missing fault stage ${stage}`));
        for (const stage of REQUIRED_FAULT_STAGES) {
          const harness = makeHarness(moduleUnderTest);
          let token;
          let dry;
          const advanceFault = stage.startsWith('advance:')
            || stage === 'adapter:before-world-step'
            || stage === 'adapter:after-world-step'
            || stage.includes('group')
            || stage.includes('mask');
          if (!advanceFault) {
            token = advance(harness);
          }
          if (stage.startsWith('finish:') || stage === 'adapter:before-q-write' || stage === 'adapter:after-q-write') {
            dry = acceptedDry(harness, token);
          }
          const before = exposed(harness.adapter);
          const groupsBefore = harness.adapter.readSolverGroups();
          harness.adapter.setFault(stage);
          let operation;
          if (advanceFault) {
            operation = () => advance(harness);
          } else if (stage.startsWith('trial:')) {
            operation = () => harness.session.trialMaterial(token, { lambda: 0, dry: true });
          } else if (stage.startsWith('finish:') || stage === 'adapter:before-q-write' || stage === 'adapter:after-q-write') {
            operation = () => harness.session.finishMaterial(token, { acceptedTrial: dry });
          } else {
            operation = () => harness.session.finishNoMaterial(token);
          }
          expectRefused(operation, /fault|inject|fail/i);
          harness.adapter.setFault(null);
          assert.equal(exposed(harness.adapter), before, `${stage} did not roll exposed state back`);
          assert.deepEqual(harness.adapter.readSolverGroups(), groupsBefore, `${stage} leaked solverGroups`);
          if (token) {
            const retry = stage.startsWith('trial:')
              ? harness.session.trialMaterial(token, { lambda: 0, dry: true })
              : stage.startsWith('structural:')
                ? harness.session.finishNoMaterial(token)
                : (stage.startsWith('finish:') || stage === 'adapter:before-q-write' || stage === 'adapter:after-q-write')
                  ? harness.session.finishMaterial(token, { acceptedTrial: dry })
                  : null;
            if (retry) assert.equal(retry.ok, true, `${stage} consumed token on failure`);
          }
        }
      },
    },
    {
      name: TEST_NAMES.onset,
      execute() {
        const harness = prepareTwoAxisHarness(moduleUnderTest, 'onset');
        const token = advance(harness);
        const result = harness.session.finishNoMaterial(token);
        assert.equal(result.ok, true, result.reason);
        near(result.rowBindings[0].normalBias, -0.5, 1e-14, 'onset restitution bias');
        near(result.qPost[0], 0.5, 1e-12, 'onset rebound speed');
        assert.equal(result.rowBindings[0].restitutionApplied, true);
      },
    },
    {
      name: TEST_NAMES.persistent,
      execute() {
        const harness = prepareTwoAxisHarness(moduleUnderTest, 'persistent');
        const token = advance(harness);
        const result = harness.session.finishNoMaterial(token);
        assert.equal(result.ok, true, result.reason);
        near(result.rowBindings[0].normalBias, 0, 1e-14, 'persistent bias');
        near(result.qPost[0], 0, 1e-12, 'persistent non-rebound speed');
        assert.equal(result.rowBindings[0].restitutionApplied, false);
      },
    },
    {
      name: TEST_NAMES.domain,
      execute() {
        const harness = domainHarness(moduleUnderTest);
        const root = exposed(harness.adapter);
        const token = advance(harness);
        const before = exposed(harness.adapter);
        const result = harness.session.trialMaterial(token, { lambda: 0, dry: true });
        assert.equal(result.ok, false, 'candidate overflow was truncated or solved');
        assert.equal(result.status, 'solver-domain-stop');
        assert.match(result.reason, /candidate upper bound 16384 exceeds/i);
        assert.doesNotMatch(result.reason, /truncat/i);
        assert.equal(exposed(harness.adapter), before, 'domain stop mutated Sfree');
        harness.session.abort(token);
        assert.equal(exposed(harness.adapter), root, 'abort after domain stop did not restore root');
      },
    },
    {
      name: TEST_NAMES.deterministic,
      execute() {
        const left = makeHarness(moduleUnderTest);
        const right = makeHarness(moduleUnderTest);
        const tokenLeft = advance(left);
        const tokenRight = advance(right);
        assert.deepEqual(tokenLeft, tokenRight);
        const dryLeft = acceptedDry(left, tokenLeft);
        const dryRight = acceptedDry(right, tokenRight);
        assert.deepEqual(dryLeft, dryRight);
      },
    },
  ];
}

function runConformance(moduleUnderTest, options = {}) {
  const only = options.only ? new Set(options.only) : null;
  const skipSourceLocks = options.skipSourceLocks === true;
  const selected = buildTests(moduleUnderTest).filter((entry) => (
    (!only || only.has(entry.name)) && !(skipSourceLocks && entry.name === TEST_NAMES.locks)
  ));
  const results = [];
  for (const entry of selected) {
    try {
      entry.execute();
      results.push({ name: entry.name, ok: true });
    } catch (error) {
      results.push({ name: entry.name, ok: false, reason: String(error.stack || error) });
    }
  }
  return {
    ok: results.every((entry) => entry.ok),
    passed: results.filter((entry) => entry.ok).length,
    failed: results.filter((entry) => !entry.ok).length,
    total: results.length,
    results,
  };
}

module.exports = {
  SOURCE_LOCKS,
  REQUIRED_FAULT_STAGES,
  TEST_NAMES,
  sha256,
  makeHarness,
  runConformance,
};
