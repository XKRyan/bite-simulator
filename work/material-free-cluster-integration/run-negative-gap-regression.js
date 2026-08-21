'use strict';

const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const {
  solveClusterFromFree,
  createFreeClusterSession,
  canonicalExposed,
} = require('./free-cluster.js');
const {
  clone,
  createWorldAdapter,
  makeConfig,
  prepareAcceptedTrial,
} = require('../material-free-cluster-qa/fixture.js');

const h = 0.01;
const qFree = [0, 0];
const result = solveClusterFromFree({
  h,
  qFree,
  Minv: [[1, 0], [0, 1]],
  materialContact: {
    id: 'material-normal',
    point: { x: 0, y: 0 },
    normalRow: [0, 1],
  },
  structuralContacts: [{
    id: 'target-floor',
    role: 'target-floor',
    point: { x: 0, y: 0 },
    normalRow: [1, 0],
    tangentRow: [0, 1],
    mu: 0.4,
    gap: -0.002,
    phase: 'persistent',
    restitution: 0.8,
    preNormalVelocity: -3,
  }],
  specificCuttingEnergy: 1,
  width: 1,
  freshArea: () => ({ area: 0.1, payload: null }),
  p: 0,
});

assert.equal(result.ok, true, result.reason);
assert.deepEqual(result.qFree, qFree);
assert.deepEqual(result.qPost, qFree, 'negative persistent gap must not inject separation velocity');
assert.equal(result.rowBindings.length, 1);
assert.equal(result.rowBindings[0].gap, -0.002);
assert.equal(result.rowBindings[0].normalBias, 0);
assert.equal(result.rowBindings[0].restitutionApplied, false);
assert.deepEqual(result.hAudit, { free: h, moreau: h, work: h, geometry: h });

const checks = [{
  name: 'persistent negative gap with stationary qFree stays stationary',
  ok: true,
}];

{
  const baseFresh = (p, trial) => ({
    area: 2,
    payload: { p, qPost: clone(trial.qPost) },
  });
  const config = makeConfig({ freshArea: baseFresh });
  const adapter = createWorldAdapter({
    Minv: config.Minv,
    materialContact: config.materialContact,
    structuralContacts: config.structuralContacts,
    specificCuttingEnergy: config.specificCuttingEnergy,
    width: config.width,
  });
  let liveAdapterMinv = clone(config.Minv);
  Object.defineProperty(adapter, 'Minv', { configurable: true, get: () => clone(liveAdapterMinv) });
  const session = createFreeClusterSession(adapter, {
    ...config,
    maskPlan: ['weapon-target', 'fork-target', 'target-floor', 'fork-floor'],
  });
  const token = session.advanceFree({ h: config.h, event: { freshArea: baseFresh } });
  const prepared = prepareAcceptedTrial(token, config);
  assert.equal(prepared.ok, true, prepared.reason);

  liveAdapterMinv = liveAdapterMinv.map((row, i) => row.map((value, j) => (
    i === j ? value * 17 : value + 3
  )));
  const dry = session.trialMaterial(token, {
    lambda: prepared.p,
    dry: true,
  });
  assert.equal(dry.ok, true, dry.reason);
  assert.deepEqual(dry.qPost, prepared.qPost, 'trial used mutable adapter.Minv instead of frozen contactFrame.Minv');
  session.abort(token);
  checks.push({
    name: 'frozen contactFrame.Minv survives live adapter mutation',
    ok: true,
  });
}

{
  const config = makeConfig();
  const adapter = createWorldAdapter({ Minv: config.Minv });
  const session = createFreeClusterSession(adapter, config);
  const before = canonicalExposed(adapter.snapshot());
  const contactFrame = {
    materialContact: clone(config.materialContact),
    structuralContacts: clone(config.structuralContacts),
    specificCuttingEnergy: config.specificCuttingEnergy,
    width: config.width,
  };
  assert.throws(
    () => session.advanceFree({ h: config.h, event: { contactFrame } }),
    /contactFrame\.Minv/,
  );
  assert.equal(canonicalExposed(adapter.snapshot()), before, 'missing contactFrame.Minv did not rollback root state');
  checks.push({ name: 'missing contactFrame.Minv is refused with exact root rollback', ok: true });
}

const report = {
  schema: 'material-free-cluster-negative-gap-regression-v1',
  passed: checks.length,
  total: checks.length,
  checks,
  negativeGap: {
    gap: result.rowBindings[0].gap,
    normalBias: result.rowBindings[0].normalBias,
    restitutionApplied: result.rowBindings[0].restitutionApplied,
    qFree: result.qFree,
    qPost: result.qPost,
  },
};

fs.writeFileSync(path.join(__dirname, 'negative-gap-report.json'), `${JSON.stringify(report, null, 2)}\n`);
process.stdout.write(`PASS ${checks.length}/${checks.length} integration regressions\n`);
