'use strict';

const assert = require('node:assert/strict');
const crypto = require('node:crypto');
const fs = require('node:fs');
const path = require('node:path');
const v3 = require('./event-kkt-coulomb-v3.js');
const v2 = require('../kkt-coulomb-extension/event-kkt-coulomb.js');
const fixture = require('../material-free-cluster-qa/fixture.js');
const oracleModule = require('./geometry-interval-oracle-qa.js');
const independent = require('../prepared-root-formula-audit/certificate-contract.js');

function clone(value) { return JSON.parse(JSON.stringify(value)); }
function hashFile(file) {
  return crypto.createHash('sha256').update(fs.readFileSync(file)).digest('hex').toUpperCase();
}

const oracleFile = path.join(__dirname, 'geometry-interval-oracle-qa.js');
const oracleSha256 = hashFile(oracleFile);
const config = fixture.makeConfig();
const bindingContext = Object.freeze({
  sFreeBinding: 'qa-sfree-binding-v1',
  contactFrameSignature: 'qa-contact-frame-signature-v1',
  h: config.h,
  Wact: 0,
  qFreeSignature: 'qa-qfree-signature-v1',
  inverseMassSignature: 'qa-minv-signature-v1',
  materialContactSignature: 'qa-material-row-signature-v1',
  structuralContactsSignature: 'qa-structural-rows-signature-v1',
});

function inputFor(oracle, freshArea, overrides = {}) {
  return {
    qFree: [2, 1, 0.6],
    Minv: config.Minv,
    materialContact: config.materialContact,
    structuralContacts: config.structuralContacts,
    specificCuttingEnergy: 1,
    width: 1,
    freshArea,
    boundFreshAreaInterval: oracle.bound,
    intervalOracle: oracle.intervalOracle,
    trustedGeometryIntervalOracleSha256: oracleSha256,
    trustedCertifiedCoulombSha256: v3.selfSha256,
    freshAreaIntervalSource: { kind: 'independent-analytic-qa-oracle' },
    bindingContext,
    ...overrides,
  };
}

function constantPrepared() {
  const oracle = oracleModule.constantArea(oracleSha256, 'qa-geometry-snapshot-v1', 2);
  return v3.prepareMaterialEvent(inputFor(
    oracle,
    (p, trial) => ({ area: oracle.areaAt(p), payload: { p, qPost: trial.qPost } }),
  ));
}

const tests = [];
function test(name, execute) { tests.push({ name, execute }); }

test('constant analytic oracle produces a signed least-root certificate', () => {
  const prepared = constantPrepared();
  assert.equal(prepared.ok, true, prepared.reason);
  assert.equal(v3.validatePreparedEvent(prepared, {
    trustedGeometryIntervalOracleSha256: oracleSha256,
    trustedCertifiedCoulombSha256: v3.selfSha256,
  }).ok, true);
  assert.equal(independent.validateLeastRootCertificate(prepared, {
    trustedGeometryOracleSha256: oracleSha256,
  }).ok, true);
  assert.equal(prepared.rootCertificate.intervalOracle.moduleSha256, oracleSha256);
  assert.equal(prepared.rootCertificate.bindingContext.sFreeBinding, bindingContext.sFreeBinding);
  assert.equal(prepared.rootCertificate.earlierIntervalTree.traversal, 'left-first');
  assert.deepEqual(
    prepared.rootCertificate.earlierIntervalTree.coverage,
    [0, prepared.rootCertificate.acceptedBracket.loP],
  );
});

test('narrow earlier tangential root is found instead of the later v2 root', () => {
  const hiddenP = 0.7;
  const halfWidth = 0.005;
  const constantInput = {
    qFree: [2, 1, 0.6], Minv: config.Minv,
    materialContact: config.materialContact,
    structuralContacts: config.structuralContacts,
    specificCuttingEnergy: 1, width: 1,
    freshArea: () => 2,
  };
  const hiddenD = v2.solvePrescribedImpulse(constantInput, hiddenP).dissipatedWork;
  const oracle = oracleModule.triangularNotch(
    oracleSha256,
    'qa-hidden-notch-snapshot-v1',
    { center: hiddenP, halfWidth, baseline: 2, minimum: hiddenD },
  );
  const prepared = v3.prepareMaterialEvent(inputFor(
    oracle,
    (p, trial) => ({ area: oracle.areaAt(p), payload: { p, qPost: trial.qPost } }),
    { options: { maximumCertifiedIntervals: 1024 } },
  ));
  assert.equal(prepared.ok, true, prepared.reason);
  assert.equal(v3.validatePreparedEvent(prepared, {
    trustedGeometryIntervalOracleSha256: oracleSha256,
    trustedCertifiedCoulombSha256: v3.selfSha256,
  }).ok, true);
  assert.equal(independent.validateLeastRootCertificate(prepared, {
    trustedGeometryOracleSha256: oracleSha256,
  }).ok, true);
  assert.ok(Math.abs(prepared.p - hiddenP) < 2e-9, `${prepared.p} did not isolate ${hiddenP}`);
  assert.ok(prepared.p < 0.8, `later root was accepted: ${prepared.p}`);
  assert.equal(
    prepared.rootCertificate.prefixExclusionLeaves.at(-1).hi,
    prepared.rootCertificate.acceptedBracket.loP,
  );
});

test('negative stopping-endpoint residual cannot hide an interior tangential root', () => {
  const hiddenP = 0.3;
  const constantInput = {
    qFree: [2, 1, 0.6], Minv: config.Minv,
    materialContact: config.materialContact,
    structuralContacts: config.structuralContacts,
    specificCuttingEnergy: 1, width: 1,
    freshArea: () => 2.4,
  };
  const hiddenD = v2.solvePrescribedImpulse(constantInput, hiddenP).dissipatedWork;
  const oracle = oracleModule.triangularNotch(
    oracleSha256,
    'qa-negative-stopping-hidden-root-v1',
    { center: hiddenP, halfWidth: 0.005, baseline: 2.4, minimum: hiddenD },
  );
  const prepared = v3.prepareMaterialEvent(inputFor(
    oracle,
    (p, trial) => ({ area: oracle.areaAt(p), payload: { p, qPost: trial.qPost } }),
    { options: { maximumCertifiedIntervals: 2048 } },
  ));
  assert.equal(prepared.ok, true, prepared.reason);
  assert.ok(Math.abs(prepared.p - hiddenP) < 2e-9, `${prepared.p} did not isolate ${hiddenP}`);
  assert.equal(v3.validatePreparedEvent(prepared, {
    trustedGeometryIntervalOracleSha256: oracleSha256,
    trustedCertifiedCoulombSha256: v3.selfSha256,
  }).ok, true);
});

test('negative stopping endpoint without an interval-certified root stops safely', () => {
  const oracle = oracleModule.constantArea(oracleSha256, 'qa-no-root-negative-stopping-v1', 10);
  const result = v3.prepareMaterialEvent(inputFor(
    oracle,
    (p, trial) => ({ area: oracle.areaAt(p), payload: { p, qPost: trial.qPost } }),
    { options: { maximumCertifiedIntervals: 128 } },
  ));
  assert.equal(result.ok, false);
  assert.equal(result.status, 'solver-domain-stop');
  assert.notEqual(result.status, 'unaffordable-slice');
  assert.match(result.reason, /least-root interval proof is unresolved/);
});

test('missing interval oracle is a safe domain stop', () => {
  const result = v3.prepareMaterialEvent({
    qFree: [2, 1, 0.6], Minv: config.Minv,
    materialContact: config.materialContact,
    structuralContacts: config.structuralContacts,
    specificCuttingEnergy: 1, width: 1,
    freshArea: () => 2,
    bindingContext,
  });
  assert.equal(result.ok, false);
  assert.equal(result.status, 'invalid-input');
  assert.match(result.reason, /boundFreshAreaInterval/i);
});

test('untrusted provider module hash is refused', () => {
  const oracle = oracleModule.constantArea(oracleSha256, 'qa-geometry-snapshot-v1', 2);
  const result = v3.prepareMaterialEvent(inputFor(
    oracle,
    () => 2,
    { trustedGeometryIntervalOracleSha256: 'F'.repeat(64) },
  ));
  assert.equal(result.ok, false);
  assert.match(result.reason, /pinned trusted geometry provider/i);
});

test('interval bound unable to prove a range stops the domain', () => {
  const oracle = oracleModule.constantArea(oracleSha256, 'qa-geometry-snapshot-v1', 2);
  const result = v3.prepareMaterialEvent(inputFor(
    { ...oracle, bound: () => null },
    () => 2,
  ));
  assert.equal(result.ok, false);
  assert.equal(result.status, 'solver-domain-stop');
  assert.match(result.reason, /interval proof is unresolved/i);
});

const certificateMutants = {
  'delete-bracket-low'(prepared) { delete prepared.rootCertificate.acceptedBracket.lo; },
  'swap-bracket'(prepared) {
    const bracket = prepared.rootCertificate.acceptedBracket;
    [bracket.lo, bracket.hi] = [bracket.hi, bracket.lo];
  },
  'later-root'(prepared) { prepared.p += 1e-3; prepared.lambda = prepared.p; },
  'change-F'(prepared) { prepared.rootCertificate.acceptedBracket.hi.F += 1; },
  'change-D'(prepared) { prepared.rootCertificate.acceptedBracket.hi.D += 1; },
  'change-area'(prepared) { prepared.rootCertificate.acceptedBracket.hi.freshArea += 1; },
  'drop-early-leaf'(prepared) { prepared.rootCertificate.prefixExclusionLeaves.shift(); },
  'forge-leaf-range'(prepared) { prepared.rootCertificate.prefixExclusionLeaves[0].fMax = 1; },
  'change-mode'(prepared) { prepared.rootCertificate.acceptedBracket.hi.modeKey = 'forged-mode'; },
  'change-work-segment'(prepared) { prepared.rootCertificate.acceptedBracket.hi.workSegments[0].work += 1; },
  'change-provider-hash'(prepared) { prepared.rootCertificate.intervalOracle.moduleSha256 = '0'.repeat(64); },
  'self-resigned-fake-provider'(prepared) {
    const fake = 'F'.repeat(64);
    prepared.rootCertificate.intervalOracle.moduleSha256 = fake;
    prepared.rootCertificate.trustedGeometryIntervalOracleSha256 = fake;
    prepared.leastRootCertificate = prepared.rootCertificate;
    const certificateBody = clone(prepared.rootCertificate);
    delete certificateBody.certificateSignature;
    prepared.rootCertificate.certificateSignature = v3._reference.signature(
      'material-v3-least-root-certificate-v1',
      certificateBody,
    );
    prepared.bindingSignature = v3._reference.bindingSignature(
      v3._reference.preparedBindingBody(prepared),
    );
  },
};

test('certificate validator catches all signed-field mutants', () => {
  const baseline = constantPrepared();
  const trust = {
    trustedGeometryIntervalOracleSha256: oracleSha256,
    trustedCertifiedCoulombSha256: v3.selfSha256,
  };
  assert.equal(v3.validatePreparedEvent(baseline, trust).ok, true);
  for (const [name, mutate] of Object.entries(certificateMutants)) {
    const mutant = clone(baseline);
    mutate(mutant);
    const validity = v3.validatePreparedEvent(mutant, trust);
    assert.equal(validity.ok, false, `${name} escaped validation`);
  }
});

const results = tests.map((entry) => {
  try { entry.execute(); return { name: entry.name, ok: true }; }
  catch (error) { return { name: entry.name, ok: false, reason: error.stack || String(error) }; }
});
const report = {
  schema: 'material-coulomb-certified-least-root-v3-qa-v1',
  sources: {
    solver: { path: 'work/material-prepared-root-integration/event-kkt-coulomb-v3.js', sha256: hashFile(path.join(__dirname, 'event-kkt-coulomb-v3.js')) },
    oracle: { path: 'work/material-prepared-root-integration/geometry-interval-oracle-qa.js', sha256: oracleSha256 },
    parent: { path: 'work/kkt-coulomb-extension/event-kkt-coulomb.js', sha256: hashFile(path.resolve(__dirname, '..', 'kkt-coulomb-extension', 'event-kkt-coulomb.js')) },
  },
  tests: results,
  certificateMutants: { caught: Object.keys(certificateMutants).length, total: Object.keys(certificateMutants).length },
  summary: { passed: results.filter((entry) => entry.ok).length, total: results.length, ok: results.every((entry) => entry.ok) },
};
const reportPath = path.join(__dirname, 'v3-test-report.json');
fs.writeFileSync(reportPath, `${JSON.stringify(report, null, 2)}\n`, 'utf8');
console.log(JSON.stringify({ ...report.summary, mutants: `${report.certificateMutants.caught}/${report.certificateMutants.total}`, solverSha256: report.sources.solver.sha256, oracleSha256, reportSha256: hashFile(reportPath) }, null, 2));
if (!report.summary.ok) process.exitCode = 1;
