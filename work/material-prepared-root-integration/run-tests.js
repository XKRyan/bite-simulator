'use strict';

const assert = require('node:assert/strict');
const crypto = require('node:crypto');
const fs = require('node:fs');
const path = require('node:path');
const moduleUnderTest = require('./free-cluster-prepared.js');
const certifiedSolver = require('./event-kkt-coulomb-v3.js');
const independentCertificate = require('../prepared-root-formula-audit/certificate-contract.js');
const fixture = require('../material-free-cluster-qa/fixture.js');
const { runConformance } = require('../material-free-cluster-qa/contract-core.js');

function clone(value) {
  return value === undefined ? undefined : JSON.parse(JSON.stringify(value));
}

function canonical(value) {
  return moduleUnderTest.canonicalExposed(value);
}

function sha256(file) {
  return crypto.createHash('sha256').update(fs.readFileSync(file)).digest('hex').toUpperCase();
}

function makeHarness(options = {}) {
  const config = fixture.makeConfig();
  const adapter = fixture.createWorldAdapter({
    q0: [2, 1, 0.6],
    Minv: config.Minv,
    materialContact: config.materialContact,
    structuralContacts: config.structuralContacts,
    specificCuttingEnergy: config.specificCuttingEnergy,
    width: config.width,
  });
  let geometryDomain = { revision: 0, remaining: 'virgin' };
  let mutateFreshArea = false;
  let faultAt = null;
  const callbackDomain = {
    snapshot: () => clone(geometryDomain),
    restore: (snapshot) => { geometryDomain = clone(snapshot); },
    canonical,
  };
  const intervalOracle = {
    schema: 'signed-geometry-fresh-area-interval-v1',
    moduleSha256: 'D6F4E2CF8719AB66B51490B0F2A7195C5B621D73734735C962C3F13D609A25F0',
    geometrySnapshotSignature: `prepared-callback-domain-snapshot-v1-${crypto.createHash('sha256')
      .update(canonical(geometryDomain)).digest('hex').toUpperCase()}`,
    geometryLipschitzSignature: 'qa-constant-area-lipschitz-zero-v1',
  };
  const pureFreshArea = (p, trial) => {
    if (mutateFreshArea) geometryDomain.revision += 1;
    return {
      area: 2,
      payload: {
        schema: 's4a-staged-difference-v1',
        p,
        qPost: clone(trial.qPost),
        modeKey: trial.modeKey,
        remainingRevision: geometryDomain.revision,
      },
    };
  };
  const boundFreshAreaInterval = (lo, hi) => ({
    areaLower: 2,
    areaUpper: 2,
    certificateType: 'exact-constant-area-interval',
    sourceSignature: intervalOracle.moduleSha256,
    moduleSha256: intervalOracle.moduleSha256,
    geometrySnapshotSignature: intervalOracle.geometrySnapshotSignature,
    geometryLipschitzSignature: intervalOracle.geometryLipschitzSignature,
    proof: { kind: 'constant-area', area: 2, interval: [lo, hi] },
  });
  const session = moduleUnderTest.createPreparedFreeClusterSession(adapter, {
    trustedCertifiedCoulombSha256: moduleUnderTest.certifiedCoulombSha256,
    freshArea: pureFreshArea,
    boundFreshAreaInterval,
    intervalOracle,
    trustedGeometryIntervalOracleSha256: intervalOracle.moduleSha256,
    freshAreaIntervalSource: { kind: 'qa-analytic-constant-area' },
    callbackDomain,
    maskSolverGroups: config.maskSolverGroups,
    maximumContacts: config.maximumContacts,
    maximumModeCandidates: config.maximumModeCandidates,
    faultInjector(stage) {
      if (faultAt === stage) {
        if (stage === 'prepared:finish:after-writeback') geometryDomain.revision += 100;
        throw new Error(`injected S4a fault at ${stage}`);
      }
    },
  });
  return {
    config,
    adapter,
    session,
    geometry: () => clone(geometryDomain),
    setMutateFreshArea: (value) => { mutateFreshArea = Boolean(value); },
    setFault: (stage) => { faultAt = stage || null; },
  };
}

function advance(harness) {
  return harness.session.advanceFree({ h: harness.config.h });
}

const tests = [];
function test(name, execute) { tests.push({ name, execute }); }

test('frozen source locks', () => {
  const root = path.resolve(__dirname, '..');
  assert.equal(
    sha256(path.join(root, 'material-free-cluster-integration', 'free-cluster.js')),
    '9BD6EC52CC6BBBF0EB09B75EB5CA53F8BD23BFACDCD4897C515523BC18F9715D',
  );
  assert.equal(
    sha256(path.join(root, 'kkt-coulomb-extension', 'event-kkt-coulomb.js')),
    '8B3058E5833E96D7C83971B6D0C234E6641D0CB98044012DFFFA5DA2268E1A3E',
  );
});

test('signed least root is session-owned and frozen', () => {
  const harness = makeHarness();
  const token = advance(harness);
  const before = canonical(harness.adapter.snapshot());
  const prepared = harness.session.prepareLeastRoot(token);
  assert.equal(prepared.ok, true);
  assert.equal(prepared.status, 'prepared-least-root');
  assert.ok(prepared.p > 0);
  assert.equal(prepared.p, prepared.lambda);
  assert.ok(Object.isFrozen(prepared));
  assert.ok(Object.isFrozen(prepared.geometryPayload));
  assert.equal(prepared.provenance.sFreeBinding, token.sFreeBinding);
  assert.equal(prepared.provenance.contactFrameSignature, token.contactFrameSignature);
  assert.equal(prepared.provenance.signedSolverSha256, moduleUnderTest.signedCoulombSha256);
  assert.equal(prepared.leastRootCertificate.searchDomain[0], 0);
  assert.equal(prepared.leastRootCertificate.acceptedBracket.hiP, prepared.p);
  assert.equal(certifiedSolver.validatePreparedEvent({
    // The session capability is an outer view, so validate its internally
    // signed solver object through the exact certificate fields instead.
    ...prepared,
    ok: true,
    status: 'prepared',
    version: 3,
  }).ok, false, 'outer capability must not impersonate the private signed solver object');
  assert.equal(independentCertificate.validateLeastRootCertificate(prepared, {
    trustedGeometryOracleSha256: prepared.provenance.trustedGeometryIntervalOracleSha256,
  }).ok, true);
  assert.equal(canonical(harness.adapter.snapshot()), before, 'preparation mutated Sfree');
  assert.deepEqual(harness.geometry(), { revision: 0, remaining: 'virgin' });
  const uniqueP = new Set(prepared.preparationAudit.evaluatedFreshSamples.map((entry) => entry.p));
  assert.equal(uniqueP.size, prepared.preparationAudit.evaluatedFreshSamples.length);
});

test('finish reuses exact accepted sample with zero solve and zero freshArea', () => {
  const harness = makeHarness();
  const token = advance(harness);
  const prepared = harness.session.prepareLeastRoot(token);
  const beforeAudit = harness.session.audit();
  const result = harness.session.finishMaterial(token, { preparedRoot: prepared });
  const afterAudit = harness.session.audit();
  assert.equal(afterAudit.signedPrepareCalls, beforeAudit.signedPrepareCalls);
  assert.equal(afterAudit.rawFreshAreaCalls, beforeAudit.rawFreshAreaCalls);
  assert.equal(afterAudit.finishSolveCalls, 0);
  assert.equal(afterAudit.finishFreshAreaCalls, 0);
  assert.equal(afterAudit.physicalQWrites - beforeAudit.physicalQWrites, 1);
  assert.equal(result.p, prepared.p);
  assert.deepEqual(result.qPost, prepared.qPost);
  assert.equal(result.modeKey, prepared.modeKey);
  assert.deepEqual(result.geometryPayload, prepared.geometryPayload);
  assert.deepEqual(result.workSegments, prepared.workSegments);
  assert.equal(result.commitTicket.acceptedSampleSignature, prepared.acceptedSampleSignature);
  assert.deepEqual(result.commitTicket.geometryPayload, prepared.geometryPayload);
  assert.deepEqual(harness.adapter.getState().q, prepared.qPost);
});

test('P1 negative: foreign or cloned prepared root is refused before writeback', () => {
  const left = makeHarness();
  const right = makeHarness();
  const leftToken = advance(left);
  const rightToken = advance(right);
  const leftRoot = left.session.prepareLeastRoot(leftToken);
  const rightRoot = right.session.prepareLeastRoot(rightToken);
  const before = canonical(left.adapter.snapshot());
  assert.throws(
    () => left.session.finishMaterial(leftToken, { preparedRoot: clone(leftRoot) }),
    /foreign|forged|stale/i,
  );
  assert.throws(
    () => left.session.finishMaterial(leftToken, { preparedRoot: rightRoot }),
    /foreign|forged|stale/i,
  );
  assert.equal(canonical(left.adapter.snapshot()), before);
  assert.equal(left.session.audit().physicalQWrites, 0);
});

test('P1 negative: mixed/later caller root cannot substitute for least root', () => {
  const harness = makeHarness();
  const token = advance(harness);
  const prepared = harness.session.prepareLeastRoot(token);
  const later = clone(prepared);
  later.p *= 1.01;
  later.lambda = later.p;
  later.geometryPayload = { forged: true };
  const before = canonical(harness.adapter.snapshot());
  assert.throws(
    () => harness.session.finishMaterial(token, { preparedRoot: later }),
    /foreign|forged|stale/i,
  );
  assert.equal(canonical(harness.adapter.snapshot()), before);
  assert.equal(harness.session.audit().physicalQWrites, 0);
  const verified = harness.session.verifyPreparedRoot(token, prepared);
  assert.equal(verified.ok, true);
});

test('P1 negative: finish has no solver or geometry callback re-entry', () => {
  const harness = makeHarness();
  const token = advance(harness);
  const prepared = harness.session.prepareLeastRoot(token);
  const counts = harness.session.audit();
  harness.session.verifyPreparedRoot(token, prepared);
  const afterVerify = harness.session.audit();
  assert.equal(afterVerify.signedPrepareCalls, counts.signedPrepareCalls);
  assert.equal(afterVerify.rawFreshAreaCalls, counts.rawFreshAreaCalls);
  harness.session.finishMaterial(token, { preparedRoot: prepared });
  const afterFinish = harness.session.audit();
  assert.equal(afterFinish.signedPrepareCalls, counts.signedPrepareCalls);
  assert.equal(afterFinish.rawFreshAreaCalls, counts.rawFreshAreaCalls);
  assert.equal(afterFinish.finishSolveCalls, 0);
  assert.equal(afterFinish.finishFreshAreaCalls, 0);
});

test('callback-visible geometry mutation is restored and token is retryable', () => {
  const harness = makeHarness();
  const token = advance(harness);
  const before = canonical(harness.adapter.snapshot());
  const beforeAudit = harness.session.audit();
  harness.setMutateFreshArea(true);
  assert.throws(() => harness.session.prepareLeastRoot(token), /mutated.*domain/i);
  assert.equal(canonical(harness.adapter.snapshot()), before);
  assert.deepEqual(harness.geometry(), { revision: 0, remaining: 'virgin' });
  assert.deepEqual(harness.session.audit(), beforeAudit, 'failed preparation leaked invocation ledger');
  harness.setMutateFreshArea(false);
  const prepared = harness.session.prepareLeastRoot(token);
  assert.equal(prepared.ok, true);
});

test('partial accepted writeback fault restores body, closure, audit and token', () => {
  const harness = makeHarness();
  const token = advance(harness);
  const prepared = harness.session.prepareLeastRoot(token);
  const beforeAdapter = canonical(harness.adapter.snapshot());
  const beforeGeometry = harness.geometry();
  const beforeAudit = harness.session.audit();
  harness.setFault('prepared:finish:after-writeback');
  assert.throws(
    () => harness.session.finishMaterial(token, { preparedRoot: prepared }),
    /injected S4a fault/i,
  );
  harness.setFault(null);
  assert.equal(canonical(harness.adapter.snapshot()), beforeAdapter);
  assert.deepEqual(harness.geometry(), beforeGeometry);
  assert.deepEqual(harness.session.audit(), beforeAudit);
  const retry = harness.session.finishMaterial(token, { preparedRoot: prepared });
  assert.equal(retry.status, 'prepared-material-finished');
});

test('prepare publication fault removes capability and restores invocation ledger', () => {
  const harness = makeHarness();
  const token = advance(harness);
  const beforeAdapter = canonical(harness.adapter.snapshot());
  const beforeAudit = harness.session.audit();
  harness.setFault('prepared:prepare:after-publish');
  assert.throws(() => harness.session.prepareLeastRoot(token), /injected S4a fault/i);
  harness.setFault(null);
  assert.equal(canonical(harness.adapter.snapshot()), beforeAdapter);
  assert.deepEqual(harness.session.audit(), beforeAudit);
  assert.equal(harness.session.prepareLeastRoot(token).ok, true);
});

test('missing callback transaction domain is refused before root solve', () => {
  const config = fixture.makeConfig();
  const adapter = fixture.createWorldAdapter({
    q0: [2, 1, 0.6], Minv: config.Minv,
    materialContact: config.materialContact,
    structuralContacts: config.structuralContacts,
    specificCuttingEnergy: config.specificCuttingEnergy,
    width: config.width,
  });
  const session = moduleUnderTest.createPreparedFreeClusterSession(adapter, {
    trustedCertifiedCoulombSha256: moduleUnderTest.certifiedCoulombSha256,
    freshArea: config.freshArea,
    maskSolverGroups: config.maskSolverGroups,
  });
  const token = session.advanceFree({ h: config.h });
  const before = canonical(adapter.snapshot());
  assert.throws(() => session.prepareLeastRoot(token), /callbackDomain\.snapshot\/restore/i);
  assert.equal(canonical(adapter.snapshot()), before);
});

test('loaded v3 solver and geometry provider must match exact trusted hashes', () => {
  const config = fixture.makeConfig();
  const adapter = fixture.createWorldAdapter({
    q0: [2, 1, 0.6], Minv: config.Minv,
    materialContact: config.materialContact,
    structuralContacts: config.structuralContacts,
    specificCuttingEnergy: config.specificCuttingEnergy,
    width: config.width,
  });
  assert.throws(
    () => moduleUnderTest.createPreparedFreeClusterSession(adapter, {
      trustedCertifiedCoulombSha256: '0'.repeat(64),
    }),
    /pin the loaded certified v3 solver/i,
  );
  const harness = makeHarness();
  const token = advance(harness);
  const before = canonical(harness.adapter.snapshot());
  const badDomain = { revision: 0, remaining: 'virgin' };
  assert.throws(
    () => harness.session.prepareLeastRoot(token, {
      // Caller cannot replace the session-pinned oracle through options.
      intervalOracle: { moduleSha256: '0'.repeat(64) },
    }),
    /rows.*frozen Sfree/i,
  );
  assert.equal(canonical(harness.adapter.snapshot()), before);
  void badDomain;
});

const results = [];
for (const entry of tests) {
  try {
    entry.execute();
    results.push({ name: entry.name, ok: true });
  } catch (error) {
    results.push({ name: entry.name, ok: false, reason: error.stack || String(error) });
  }
}

const p0 = runConformance(moduleUnderTest);
const sourcePath = path.join(__dirname, 'free-cluster-prepared.js');
const report = {
  schema: 'material-prepared-root-integration-s4a-v1',
  source: {
    path: 'work/material-prepared-root-integration/free-cluster-prepared.js',
    sha256: sha256(sourcePath),
  },
  signedInputs: {
    p0: {
      path: 'work/material-free-cluster-integration/free-cluster.js',
      sha256: sha256(path.resolve(__dirname, '..', 'material-free-cluster-integration', 'free-cluster.js')),
    },
    coulomb: {
      path: 'work/kkt-coulomb-extension/event-kkt-coulomb.js',
      sha256: sha256(path.resolve(__dirname, '..', 'kkt-coulomb-extension', 'event-kkt-coulomb.js')),
    },
    certifiedCoulombV3: {
      path: 'work/material-prepared-root-integration/event-kkt-coulomb-v3.js',
      sha256: sha256(path.resolve(__dirname, 'event-kkt-coulomb-v3.js')),
    },
    analyticGeometryOracleQa: {
      path: 'work/material-prepared-root-integration/geometry-interval-oracle-qa.js',
      sha256: sha256(path.resolve(__dirname, 'geometry-interval-oracle-qa.js')),
    },
  },
  preparedRootTests: results,
  p1NegativeTests: {
    passed: results.filter((entry) => entry.ok && entry.name.startsWith('P1 negative:')).length,
    total: results.filter((entry) => entry.name.startsWith('P1 negative:')).length,
  },
  frozenP0Conformance: {
    passed: p0.passed,
    total: p0.total,
    ok: p0.ok,
    results: p0.results,
  },
  summary: {
    preparedPassed: results.filter((entry) => entry.ok).length,
    preparedTotal: results.length,
    p0: `${p0.passed}/${p0.total}`,
    ok: results.every((entry) => entry.ok) && p0.ok,
  },
};

const reportPath = path.join(__dirname, 'test-report.json');
fs.writeFileSync(reportPath, `${JSON.stringify(report, null, 2)}\n`, 'utf8');
console.log(JSON.stringify({
  ...report.summary,
  p1Negative: `${report.p1NegativeTests.passed}/${report.p1NegativeTests.total}`,
  sourceSha256: report.source.sha256,
  reportSha256: sha256(reportPath),
}, null, 2));
if (!report.summary.ok) process.exitCode = 1;
