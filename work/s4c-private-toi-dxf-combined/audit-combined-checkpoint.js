'use strict';

const crypto = require('node:crypto');
const fs = require('node:fs');
const path = require('node:path');

const here = __dirname;
const root = path.resolve(here, '..', '..');
const files = {
  source: path.join(here, 'app-combined-candidate.js'),
  bundle: path.join(here, 'app-combined-bundle.js'),
  manifest: path.join(here, 'bundle-manifest.json'),
  plan: path.join(here, 'build-plan.json'),
  browser: path.join(here, 's4c-browser-report.json'),
  rebuilt: path.join(here, 'rebuild-proof', 'app-combined-bundle.js'),
  production: path.join(root, 'outputs', 'bite-simulator', 'app.js'),
  base: path.join(root, 'work', 'material-real-authority-checkpoint', 'app-s4b1-real-capture-candidate.js'),
};
const expected = Object.freeze({
  source: '06B6EC1F6E611DDD4122652DC9763D8F75129904D224ACCA3CABB1AF3D71D0AA',
  bundle: 'C100E7FBF724448643D3CF6123EEDECF201A923321838E7B27299CFCC809A2A9',
  planDigest: '0250944AA850F97B4A75E46AC567B65E6EF4AABAC27FF542A53E7B087606ED30',
  privateToi: '78F53C3ACA51678BC3BA16336E97BD6F09084AE257F7C3C558450974E28D5166',
  prepared: 'AE46B3E461BFD5BE7641E71FD279BB2BF56F7743BFB3137ECB36215498F691BF',
  base: 'FA9D838EF5ECBE98448423946D253462F8C8ABC5CB792D5B430EB7A10773C82A',
  production: '3A58BFB55ACA173DB63435F75C7FDF19BB0F6D68D42AEABF4E77FC1A7AB2D344',
});
const sha = (filename) => crypto.createHash('sha256').update(fs.readFileSync(filename)).digest('hex').toUpperCase();
const source = fs.readFileSync(files.source, 'utf8');
const bundle = fs.readFileSync(files.bundle, 'utf8');
const production = fs.readFileSync(files.production, 'utf8');
const manifest = JSON.parse(fs.readFileSync(files.manifest, 'utf8'));
const plan = JSON.parse(fs.readFileSync(files.plan, 'utf8'));
const browser = JSON.parse(fs.readFileSync(files.browser, 'utf8'));
const run = browser.run;
const checkpointSource = source.slice(
  source.indexOf('  let s4bAuthorityGeneration = 0;'),
  source.indexOf('  async function initialise()'),
);
const finiteSameToi = browser.witnessCases.filter((entry) => entry.found).every((entry) => (
  entry.sameToi
  && ['featurePointError', 'materialPointError', 'commonPointError',
    'signedClosingError', 'normalMagnitudeError'].every((key) => Number.isFinite(entry.sameToi[key]))
));
const checks = {
  sourceHashPinned: sha(files.source) === expected.source,
  bundleHashPinned: sha(files.bundle) === expected.bundle,
  reproducibleBundle: sha(files.rebuilt) === expected.bundle,
  baseCheckpointUntouched: sha(files.base) === expected.base,
  productionUntouched: sha(files.production) === expected.production,
  planDigestPinned: plan.planDigest === expected.planDigest
    && manifest.planDigest === expected.planDigest,
  privateToiInputPinned: manifest.buildPlan.inputs.find((entry) => entry.id === 'materialToiWitness')?.sha256
    === expected.privateToi,
  preparedInputPinned: manifest.buildPlan.inputs.find((entry) => entry.id === 'preparedFreeCluster')?.sha256
    === expected.prepared,
  allBuildInputsPinned: manifest.buildPlan.inputs.every((entry) => (
    sha(path.resolve(root, ...entry.path.split('/'))) === entry.sha256
  )),
  manifestNamesExactArtifact: manifest.artifact.bundleSha256 === expected.bundle
    && manifest.artifact.candidateActualSha256 === expected.bundle,
  privateModuleActuallyCalled: source.includes("const s4cToiModule = require('../material-toi-witness/material-toi-witness.js');")
    && (source.match(/s4cToiModule\.findEarliestTriangleMaterialWitness\(/g) || []).length === 2,
  noGeometryKernelCall: !/s4bGeometryKernelModule\s*\./.test(source),
  noPreparedKktEntryCall: !/\.prepareLeastRoot\s*\(/.test(checkpointSource)
    && !/\.finishMaterial\s*\(/.test(checkpointSource)
    && !/\.finishNoMaterial\s*\(/.test(checkpointSource),
  candidateMainSwitchClosed: source.includes('const MATERIAL_EVENT_MAIN_TRANSACTION_WIRING_ENABLED = false;')
    && source.includes('const TRACEABLE_CUTTING_CONTINUOUS_TOI_ENABLED = false;'),
  productionSwitchClosed: production.includes('const TRACEABLE_CUTTING_CONTINUOUS_TOI_ENABLED = false;')
    && !production.includes('S4C_PRIVATE_WITNESS_CONTEXT'),
  privateIifeNoModuleLeak: bundle.includes('/* SINGLE_PRIVATE_IIFE_BEGIN */')
    && bundle.includes("const __modules=Object.freeze(")
    && !/globalThis\.BiteMaterialEventWitness/.test(bundle)
    && !/window\.s4cToiModule\s*=/.test(bundle),
  browserPass: browser.pass === true && Object.values(browser.checks).every(Boolean),
  browserSameToiFinite: finiteSameToi,
  realScopeHonest: browser.description.realRapierScenarioScope === 'baseline parameter-tip ingress only'
    && browser.witnessCases.every((entry) => entry.realRapierWorldStep === false),
  soleRealAdvanceExact: run.realRapier === true && run.checkpointPass === true
    && run.worldStepDelta === 1 && run.forceApplicationDelta === 1
    && run.freeCaptureDelta === 1 && run.private78fCallCount === 1
    && run.captureKktSolveDelta === 0 && run.capturePublicationWriteDelta === 0
    && run.qWriteDelta === 0,
  exactTokenAndRoot: run.exactReturnedTokenWeakKeyBound === true
    && run.completeCheckpointSfreeCapture === true
    && run.acceptedRootExact === true
    && run.rootBeforeSha256 === run.rootAfterSha256
    && run.scenarioRootBeforeSha256 === run.scenarioRootAfterSha256,
  spatialCertificates: run.rigidSpatialGate === 8e-5
    && run.witnessSpatialGate === 5e-6
    && run.toiMaximumUnresolvedSpatialError <= 5e-6
    && browser.witnessCases.filter((entry) => entry.found).every((entry) => (
      entry.separation <= 8e-5 && entry.maximumUnresolvedSpatialError <= 5e-6
    )),
  fullAuthorityStillStopped: run.ok === false && run.status === 'domain-stop'
    && run.trajectoryAuthorityClosed === false
    && run.geometryKernelCalled === false && run.kktCalled === false
    && run.publicationWritten === false && run.materialRemovalDefined === false,
  attacksAndFaultsReal: browser.attacks.every((entry) => entry.actualAttackExecuted
    && entry.rejected && entry.acceptedRootExact)
    && browser.faults.every((entry) => entry.actualFaultInjected
      && entry.rejected && entry.acceptedRootExact
      && entry.prepareCallDelta === 0 && entry.kktSolveDelta === 0
      && entry.publicationWriteDelta === 0 && entry.qWriteDelta === 0),
};
const report = {
  schema: 'material-private-toi-param-checkpoint-audit-v1',
  pass: Object.values(checks).every(Boolean),
  checks,
  scope: {
    realRapier: 'one fixed parameter-tip ingress; one world step and one force application',
    fixedWitnessCases: 'real Edge/private 78F over trusted parameter-generator geometry and declared SE(2) motions; not Rapier world steps',
    authority: 'domain-stop before hidden-trajectory authority, 80D, KKT, publication or removal',
  },
  hashes: Object.fromEntries(Object.entries(files).map(([id, filename]) => [id, sha(filename)])),
  pins: expected,
  observables: {
    realRun: {
      toiStatus: run.toiStatus,
      contactRole: run.toiContactRole,
      signedClosingVelocity: run.toiSignedClosingVelocity,
      maximumUnresolvedSpatialError: run.toiMaximumUnresolvedSpatialError,
      acceptedRootExact: run.acceptedRootExact,
    },
    witnessCases: browser.witnessCases.map((entry) => ({
      id: entry.id, found: entry.found, status: entry.status,
      contactRole: entry.contactRole, featureEdgeId: entry.featureEdgeId,
      materialRingId: entry.materialRingId, domainAction: entry.domainAction,
      sameToi: entry.sameToi,
    })),
  },
};
const output = path.join(here, 'combined-checkpoint-audit-report.json');
fs.writeFileSync(output, `${JSON.stringify(report, null, 2)}\n`, 'utf8');
process.stdout.write(`${JSON.stringify({ pass: report.pass, checks, output, sha256: sha(output) }, null, 2)}\n`);
if (!report.pass) process.exitCode = 1;
