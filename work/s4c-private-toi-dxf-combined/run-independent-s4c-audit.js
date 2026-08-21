'use strict';

// Independent audit driver for the combined work-only checkpoint. It never
// writes under the signed S4c directory or outputs/.

const { spawn, spawnSync } = require('node:child_process');
const crypto = require('node:crypto');
const fs = require('node:fs');
const http = require('node:http');
const path = require('node:path');

const auditDir = __dirname;
const root = path.resolve(auditDir, '..', '..');
const candidateDir = auditDir;
const outputDir = path.join(root, 'outputs', 'bite-simulator');
const files = Object.freeze({
  source: path.join(candidateDir, 'app-combined-candidate.js'),
  bundle: path.join(candidateDir, 'app-combined-bundle.js'),
  manifest: path.join(candidateDir, 'bundle-manifest.json'),
  build: path.join(candidateDir, 'build.js'),
  privateToi: path.join(root, 'work', 'material-toi-witness', 'material-toi-witness.js'),
  production: path.join(outputDir, 'app.js'),
  base: path.join(root, 'work', 'material-real-authority-checkpoint', 'app-s4b1-real-capture-candidate.js'),
  harness: path.join(auditDir, 'independent-harness.html'),
  runner: __filename,
});
const expected = Object.freeze({
  source: '06B6EC1F6E611DDD4122652DC9763D8F75129904D224ACCA3CABB1AF3D71D0AA',
  bundle: 'C100E7FBF724448643D3CF6123EEDECF201A923321838E7B27299CFCC809A2A9',
  manifest: '27528C05B74BE1ACC777FF954B3104405586F48E3EC18EA2A1B596C0132853D2',
  plan: '0250944AA850F97B4A75E46AC567B65E6EF4AABAC27FF542A53E7B087606ED30',
  private78f: '78F53C3ACA51678BC3BA16336E97BD6F09084AE257F7C3C558450974E28D5166',
  preparedAe46: 'AE46B3E461BFD5BE7641E71FD279BB2BF56F7743BFB3137ECB36215498F691BF',
  production: '3A58BFB55ACA173DB63435F75C7FDF19BB0F6D68D42AEABF4E77FC1A7AB2D344',
  base: 'FA9D838EF5ECBE98448423946D253462F8C8ABC5CB792D5B430EB7A10773C82A',
});
const reportFile = path.join(auditDir, 'combined-independent-s4c-audit-report.json');
const sha = (filename) => crypto.createHash('sha256').update(fs.readFileSync(filename)).digest('hex').toUpperCase();
const all = (values) => values.every(Boolean);
const finite = (value) => Number.isFinite(value);
const finitePoint = (value) => Boolean(value) && finite(value.x) && finite(value.y);
const close = (left, right, tolerance = 1e-10) => finite(left) && finite(right)
  && Math.abs(left - right) <= tolerance;
const sameJson = (left, right) => JSON.stringify(left) === JSON.stringify(right);
const decode = (text) => text.replaceAll('&quot;', '"').replaceAll('&#39;', "'")
  .replaceAll('&amp;', '&').replaceAll('&lt;', '<').replaceAll('&gt;', '>');

function snapshotFrozen() {
  return Object.freeze({
    source: sha(files.source), bundle: sha(files.bundle), manifest: sha(files.manifest),
    production: sha(files.production), base: sha(files.base), privateToi: sha(files.privateToi),
  });
}

function makeTrustedFeature() {
  // This is the literal result of the externally set frozen parameter preset:
  // one tooth, R=55 mm, working length=14 mm, width=6 mm, phase=0, no mirrors.
  const vertices = [
    Object.freeze({ id: 'root', x: 0.041, y: 0 }),
    Object.freeze({ id: 'tip', x: 0.055, y: 0 }),
    Object.freeze({ id: 'back', x: 0.041, y: -0.006 }),
  ];
  return Object.freeze({
    vertices: Object.freeze(vertices),
    edges: Object.freeze([
      Object.freeze({ id: 'working-face', working: true }),
      Object.freeze({ id: 'back-edge', working: false }),
      Object.freeze({ id: 'root-edge', working: false }),
    ]),
    workingVertexIds: Object.freeze(['tip']),
  });
}

const point = (x, y) => ({ x, y });
const add = (left, right) => point(left.x + right.x, left.y + right.y);
const subtract = (left, right) => point(left.x - right.x, left.y - right.y);
const scale = (value, scalar) => point(value.x * scalar, value.y * scalar);
const dot = (left, right) => left.x * right.x + left.y * right.y;
const length = (value) => Math.hypot(value.x, value.y);
const normalise = (value) => {
  const magnitude = length(value);
  return magnitude > 0 ? scale(value, 1 / magnitude) : point(0, 0);
};
const perpendicular = (value) => point(-value.y, value.x);
const rotate = (value, angle) => point(
  Math.cos(angle) * value.x - Math.sin(angle) * value.y,
  Math.sin(angle) * value.x + Math.cos(angle) * value.y,
);
const lerp = (left, right, fraction) => add(left, scale(subtract(right, left), fraction));
const motion = (position0, position1, angle0 = 0, angle1 = angle0, angleDelta = angle1 - angle0) => ({
  start: { position: position0, angle: angle0 },
  end: { position: position1, angle: angle1 },
  angleDelta,
});
const ringRectangle = (center, tangent, outward, halfTangent, halfNormal) => {
  const t = scale(normalise(tangent), halfTangent);
  const n = scale(normalise(outward), halfNormal);
  return [[[
    [center.x - t.x - n.x, center.y - t.y - n.y],
    [center.x + t.x - n.x, center.y + t.y - n.y],
    [center.x + t.x + n.x, center.y + t.y + n.y],
    [center.x - t.x + n.x, center.y - t.y + n.y],
  ]]];
};

function fixedCaseRequest(id) {
  const feature = makeTrustedFeature();
  const vertices = feature.vertices;
  const centroid = scale(vertices.reduce((sum, vertex) => add(sum, vertex), point(0, 0)), 1 / 3);
  const stillMaterial = motion(point(0, 0), point(0, 0));
  const options = {
    timeTolerance: 1e-8,
    distanceTolerance: 1e-10,
    maxAngularTravelPerCell: Math.PI / 24,
    maxNodes: 500000,
    continueAfterSeparatingBoundary: true,
  };
  if (id === 'tip-first' || id === 'root-first') {
    const vertex = vertices[id === 'tip-first' ? 1 : 0];
    const direction = normalise(subtract(vertex, centroid));
    const tangent = perpendicular(direction);
    const gap = 0.004; const materialHalfNormal = 0.001;
    const center = add(vertex, scale(direction, gap + materialHalfNormal));
    return {
      feature, material: ringRectangle(center, tangent, direction, 0.0002, materialHalfNormal),
      originalMaterial: null, dt: 1,
      featureMotion: motion(point(0, 0), scale(direction, gap * 2)),
      materialMotion: stillMaterial, history: { hadPriorContact: false }, options,
    };
  }
  if (id === 'back-edge-first') {
    const start = vertices[1]; const end = vertices[2];
    const midpoint = scale(add(start, end), 0.5);
    const tangent = normalise(subtract(end, start));
    let direction = perpendicular(tangent);
    if (dot(direction, subtract(midpoint, centroid)) < 0) direction = scale(direction, -1);
    const gap = 0.004; const materialHalfNormal = 0.001;
    const center = add(midpoint, scale(direction, gap + materialHalfNormal));
    return {
      feature,
      material: ringRectangle(center, tangent, direction,
        Math.min(0.001, length(subtract(end, start)) * 0.2), materialHalfNormal),
      originalMaterial: null, dt: 1,
      featureMotion: motion(point(0, 0), scale(direction, gap * 2)),
      materialMotion: stillMaterial, history: { hadPriorContact: false }, options,
    };
  }
  if (id === 'rotational-midstep-enter-exit') {
    const tip = vertices[1]; const half = 0.00005;
    return {
      feature,
      material: [[[
        [tip.x - half, tip.y - half], [tip.x + half, tip.y - half],
        [tip.x + half, tip.y + half], [tip.x - half, tip.y + half],
      ]]],
      originalMaterial: null, dt: 1,
      featureMotion: motion(point(0, 0), point(0, 0), -0.5, 0.5, 1),
      materialMotion: stillMaterial, history: { hadPriorContact: false },
      options: { ...options, maxAngularTravelPerCell: 10 },
    };
  }
  if (id === 'hole-reentry') {
    const startPosition = scale(centroid, -1);
    return {
      feature,
      material: [[
        [[-0.1, -0.1], [0.1, -0.1], [0.1, 0.1], [-0.1, 0.1]],
        [[-0.02, -0.02], [0.02, -0.02], [0.02, 0.02], [-0.02, 0.02]],
      ]],
      originalMaterial: [[[
        [-0.1, -0.1], [0.1, -0.1], [0.1, 0.1], [-0.1, 0.1],
      ]]],
      dt: 1,
      featureMotion: motion(startPosition, add(startPosition, point(0.04, 0))),
      materialMotion: stillMaterial, history: { hadPriorContact: false }, options,
    };
  }
  if (id === 'no-contact') return {
    feature, material: [[[[0.2, -0.01], [0.22, -0.01], [0.22, 0.01], [0.2, 0.01]]]],
    originalMaterial: null, dt: 1,
    featureMotion: motion(point(0, 0), point(0.001, 0)),
    materialMotion: stillMaterial, history: { hadPriorContact: false }, options,
  };
  throw new Error(`unknown fixed case ${id}`);
}

function poseAt(value, time, dt) {
  const fraction = time / dt;
  return {
    position: lerp(value.start.position, value.end.position, fraction),
    angle: value.start.angle + value.angleDelta * fraction,
  };
}

function localWitnessPoint(feature, witness) {
  if (witness.featureVertexIndex !== null) {
    const vertex = feature.vertices[witness.featureVertexIndex];
    return point(vertex.x, vertex.y);
  }
  const start = feature.vertices[witness.featureEdgeIndex];
  const end = feature.vertices[(witness.featureEdgeIndex + 1) % feature.vertices.length];
  return lerp(start, end, witness.featureFraction);
}

function independentlyRecomputeSameToi(request, result) {
  if (!result || !result.witness || !finite(result.toi)) return null;
  const witness = result.witness;
  const featurePose = poseAt(request.featureMotion, result.toi, request.dt);
  const materialPose = poseAt(request.materialMotion, result.toi, request.dt);
  const localFeature = localWitnessPoint(request.feature, witness);
  const featureWorld = add(featurePose.position, rotate(localFeature, featurePose.angle));
  const ring = request.material[witness.materialPolygonIndex][witness.materialRingIndex];
  const materialStart = point(...ring[witness.materialSegmentIndex]);
  const materialEnd = point(...ring[(witness.materialSegmentIndex + 1) % ring.length]);
  const localMaterial = lerp(materialStart, materialEnd, witness.materialFraction);
  const materialWorld = add(materialPose.position, rotate(localMaterial, materialPose.angle));
  const common = scale(add(featureWorld, materialWorld), 0.5);
  const featureOriginVelocity = scale(
    subtract(request.featureMotion.end.position, request.featureMotion.start.position), 1 / request.dt,
  );
  const materialOriginVelocity = scale(
    subtract(request.materialMotion.end.position, request.materialMotion.start.position), 1 / request.dt,
  );
  const featureRadial = rotate(localFeature, featurePose.angle);
  const materialRadial = rotate(localMaterial, materialPose.angle);
  const featureVelocity = add(featureOriginVelocity, point(
    -request.featureMotion.angleDelta / request.dt * featureRadial.y,
    request.featureMotion.angleDelta / request.dt * featureRadial.x,
  ));
  const materialVelocity = add(materialOriginVelocity, point(
    -request.materialMotion.angleDelta / request.dt * materialRadial.y,
    request.materialMotion.angleDelta / request.dt * materialRadial.x,
  ));
  const expectedClosing = -dot(subtract(featureVelocity, materialVelocity), witness.materialOutwardNormal);
  return Object.freeze({
    featurePointError: length(subtract(featureWorld, witness.featureWorldPoint)),
    materialPointError: length(subtract(materialWorld, witness.materialWorldPoint)),
    commonPointError: length(subtract(common, witness.commonWorldPoint)),
    signedClosingError: Math.abs(expectedClosing - witness.signedClosingVelocity),
    normalMagnitudeError: Math.abs(length(witness.materialOutwardNormal) - 1),
    featureWorld, materialWorld, common, expectedClosing,
    witnessCommon: witness.commonWorldPoint, witnessNormal: witness.materialOutwardNormal,
  });
}

function summarizeIndependentWitness(id, module) {
  const request = fixedCaseRequest(id);
  const result = module.findEarliestTriangleMaterialWitness(request);
  const sameToi = independentlyRecomputeSameToi(request, result);
  return Object.freeze({
    id,
    result,
    sameToi,
    sameToiFinite: sameToi === null || all([
      sameToi.featurePointError, sameToi.materialPointError, sameToi.commonPointError,
      sameToi.signedClosingError, sameToi.normalMagnitudeError, sameToi.expectedClosing,
      sameToi.featureWorld.x, sameToi.featureWorld.y, sameToi.materialWorld.x,
      sameToi.materialWorld.y, sameToi.common.x, sameToi.common.y,
      sameToi.witnessCommon.x, sameToi.witnessCommon.y,
      sameToi.witnessNormal.x, sameToi.witnessNormal.y,
    ].map(finite)),
    sameToiExact: sameToi === null || (
      sameToi.featurePointError <= 1e-11 && sameToi.materialPointError <= 1e-11
      && sameToi.commonPointError <= 1e-11 && sameToi.signedClosingError <= 1e-10
      && sameToi.normalMagnitudeError <= 1e-12
    ),
  });
}

function runCleanBuild() {
  const rebuiltDir = fs.mkdtempSync(path.join(auditDir, 'clean-rebuild-'));
  const processResult = spawnSync(process.execPath, [files.build, `--qa-output-dir=${rebuiltDir}`], {
    cwd: root, encoding: 'utf8', windowsHide: true,
  });
  const bundle = path.join(rebuiltDir, 'app-combined-bundle.js');
  const manifest = path.join(rebuiltDir, 'bundle-manifest.json');
  return Object.freeze({
    directory: path.relative(root, rebuiltDir).replaceAll('\\', '/'),
    exitCode: processResult.status,
    stdout: processResult.stdout.trim(),
    stderr: processResult.stderr.trim(),
    bundle,
    manifest,
    bundleSha256: fs.existsSync(bundle) ? sha(bundle) : null,
    manifestSha256: fs.existsSync(manifest) ? sha(manifest) : null,
  });
}

function runBrowserSampler() {
  const edge = [
    'C:\\Program Files (x86)\\Microsoft\\Edge\\Application\\msedge.exe',
    'C:\\Program Files\\Microsoft\\Edge\\Application\\msedge.exe',
  ].find(fs.existsSync);
  if (!edge) return Promise.reject(new Error('Microsoft Edge not found'));
  const profile = path.join(auditDir, `edge-profile-${process.pid}-${Date.now()}`);
  const mime = {
    '.html': 'text/html; charset=utf-8', '.js': 'text/javascript; charset=utf-8',
    '.css': 'text/css; charset=utf-8', '.wasm': 'application/wasm',
    '.svg': 'image/svg+xml', '.png': 'image/png', '.json': 'application/json',
  };
  const resolveRequest = (url) => {
    const pathname = decodeURIComponent(new URL(url, 'http://127.0.0.1').pathname);
    if (pathname === '/' || pathname === '/independent-harness.html') return files.harness;
    const virtual = pathname.match(/^\/candidate\/(.*)$/);
    if (!virtual) return null;
    const relative = virtual[1] || 'index.html';
    if (relative === 'app.js') return files.bundle;
    const requested = path.resolve(outputDir, relative);
    return requested.startsWith(`${outputDir}${path.sep}`) ? requested : null;
  };
  const server = http.createServer((request, response) => {
    const file = resolveRequest(request.url);
    if (!file) return response.writeHead(403).end();
    fs.readFile(file, (error, data) => {
      if (error) return response.writeHead(404).end();
      response.writeHead(200, {
        'Content-Type': mime[path.extname(file)] || 'application/octet-stream',
        'Cache-Control': 'no-store',
      });
      response.end(data);
    });
  });
  return new Promise((resolve, reject) => {
    server.once('error', reject);
    server.listen(0, '127.0.0.1', () => {
      let stdout = ''; let stderr = ''; let settled = false;
      const page = `http://127.0.0.1:${server.address().port}/independent-harness.html`;
      const child = spawn(edge, [
        '--headless=new', '--disable-gpu', '--no-sandbox',
        '--disable-background-timer-throttling', '--disable-renderer-backgrounding',
        `--user-data-dir=${profile}`, '--virtual-time-budget=180000', '--dump-dom', page,
      ], { windowsHide: true });
      const timeout = setTimeout(() => child.kill(), 210000);
      child.stdout.on('data', (chunk) => { stdout += chunk; });
      child.stderr.on('data', (chunk) => { stderr += chunk; });
      child.once('error', (error) => finish(error));
      child.once('close', (exitCode) => {
        const match = stdout.match(/<pre id="out">([\s\S]*?)<\/pre>/);
        let report;
        try {
          report = match ? JSON.parse(decode(match[1])) : {
            schema: 's4c-independent-raw-browser-v1',
            error: 'browser sampler output element missing', exitCode, stderr: stderr.slice(-4000),
          };
        } catch (error) {
          report = { schema: 's4c-independent-raw-browser-v1', error: String(error) };
        }
        finish(null, report);
      });
      function finish(error, value) {
        if (settled) return;
        settled = true;
        clearTimeout(timeout);
        server.close();
        const resolvedProfile = path.resolve(profile);
        if (path.dirname(resolvedProfile) === path.resolve(auditDir)) {
          fs.rmSync(resolvedProfile, { recursive: true, force: true });
        }
        if (error) reject(error); else resolve(value);
      }
    });
  });
}

function publicWitnessMatchesIndependent(publicEntry, independent) {
  const result = independent.result;
  const witness = result.witness;
  if (!publicEntry || !result) return false;
  const fieldsMatch = publicEntry.found === (result.found === true)
    && publicEntry.status === (result.status || null)
    && publicEntry.contactClass === (result.contactClass || null)
    && publicEntry.contactRole === (witness?.contactRole || null)
    && publicEntry.featureVertexId === (witness?.featureVertexId || null)
    && publicEntry.featureEdgeId === (witness?.featureEdgeId || null)
    && publicEntry.materialRingId === (witness?.materialRingId || null)
    && publicEntry.workingFace === (witness?.workingFace ?? null)
    && publicEntry.removalAllowedBy78f === (result.removalAllowed === true)
    && publicEntry.domainAction === (result.domainAction || null);
  const numberFieldsMatch = (!result.found && publicEntry.toi === null)
    || (close(publicEntry.toi, result.toi, 1e-10)
      && close(publicEntry.signedClosingVelocity, witness?.signedClosingVelocity, 1e-9)
      && close(publicEntry.separation, witness?.separation, 1e-10)
      && close(publicEntry.maximumUnresolvedSpatialError,
        result.certificate?.maximumUnresolvedSpatialError, 1e-12));
  const candidateSameToi = publicEntry.sameToi;
  const localSameToi = independent.sameToi;
  const sameToiMatches = (!localSameToi && candidateSameToi === null)
    || Boolean(localSameToi && candidateSameToi && all([
      'featurePointError', 'materialPointError', 'commonPointError',
      'signedClosingError', 'normalMagnitudeError',
    ].map((key) => close(candidateSameToi[key], localSameToi[key], 1e-12))));
  return fieldsMatch && numberFieldsMatch && sameToiMatches;
}

function validate(source, manifest, browser, rebuild, initial, final, independentCases) {
  const checkpointSource = source.slice(
    source.indexOf('  let s4bAuthorityGeneration = 0;'),
    source.indexOf('  async function initialise()'),
  );
  const run = browser.run || {};
  const description = browser.description || {};
  const witnesses = Array.isArray(browser.witnessCases) ? browser.witnessCases : [];
  const attacks = Array.isArray(browser.attacks) ? browser.attacks : [];
  const faults = Array.isArray(browser.faults) ? browser.faults : [];
  const fixedIds = [
    'tip-first', 'root-first', 'back-edge-first',
    'rotational-midstep-enter-exit', 'hole-reentry', 'no-contact',
  ];
  const byId = Object.fromEntries(witnesses.map((entry) => [entry.id, entry]));
  const expectedAttacks = Object.freeze({
    'clone-prepared-token': 'S4B_FOREIGN_OR_CLONED_TOKEN',
    'cross-session-prepared-token': 'S4B_CROSS_SESSION_TOKEN',
    'stale-after-restore-token': 'S4B_STALE_TOKEN',
    'clone-capability': 'S4B_FOREIGN_OR_CLONED_CAPABILITY',
    'cross-session-capability': 'S4B_CROSS_SESSION_TOKEN',
  });
  const expectedFaults = Object.freeze({
    'partial-capture-throw': 'S4B_INJECTED_PARTIAL_CAPTURE',
    'toi-before-call-throw': 'S4C_INJECTED_TOI_BEFORE_CALL',
    'toi-after-call-throw': 'S4C_INJECTED_TOI_AFTER_CALL',
    'abort-partial-mutate-throw': 'S4B_INJECTED_ABORT_RESTORE',
    'outer-restore-partial-mutate-throw': 'S4B_INJECTED_OUTER_RESTORE',
  });
  const expectedBoundFields = [
    'preparedSessionIdentity', 'preparedTokenIdentity', 'sessionGeneration',
    'stepEpoch', 'forceEpoch', 'h', 'sFreeBindingIdentity', 'qFree', 'Wact',
    'contactFrameSignature', 'Minv', 'structuralRows', 'maskedPairOwnerSet',
    'robotBodyIdentity', 'robotHandle', 'weaponBodyIdentity', 'weaponHandle',
    'forkBodyIdentity', 'forkHandle', 'targetBodyIdentity', 'targetHandle',
    'weaponColliderHandles', 'targetColliderHandles', 'forkColliderHandles',
    'remainingGeometryIdentity', 'remainingVersion', 'featureIdentity',
    'materialRowIdentity', 'featureVertices', 'featureEdges', 'workingEdgeIndex',
    'workingOwnership', 'toiModuleIdentity', 'toiTrajectoryIdentity',
    'toiStartState', 'toiEndState', 'toiFeatureMotion', 'toiMaterialMotion',
    'toiWitnessIdentity', 'toiWitnessDigest', 'toiWorkingFaceIdentity',
    'toiResultStatus', 'toiDomainAction', 'toiCertificate', 'commonPoint', 'normal',
  ];
  const found = witnesses.filter((entry) => entry.found === true);
  const manifestInputs = Array.isArray(manifest.buildPlan?.inputs) ? manifest.buildPlan.inputs : [];
  const checks = {
    frozenHashesPinned: initial.source === expected.source && initial.bundle === expected.bundle
      && initial.manifest === expected.manifest && initial.privateToi === expected.private78f,
    frozenInputsUntouchedAfterAudit: sameJson(initial, final),
    productionAndBaselineUntouched: initial.production === expected.production
      && initial.base === expected.base && final.production === initial.production && final.base === initial.base,
    cleanRebuild: rebuild.exitCode === 0 && rebuild.bundleSha256 === expected.bundle
      && fs.existsSync(rebuild.manifest)
      && JSON.parse(fs.readFileSync(rebuild.manifest, 'utf8')).artifact.bundleSha256 === expected.bundle,
    manifestPinnedAndNamesExact: manifest.planDigest === expected.plan
      && manifest.artifact?.bundleSha256 === expected.bundle
      && manifest.artifact?.candidateActualSha256 === expected.bundle
      && manifestInputs.find((entry) => entry.id === 'materialToiWitness')?.sha256 === expected.private78f
      && manifestInputs.find((entry) => entry.id === 'preparedFreeCluster')?.sha256 === expected.preparedAe46
      && manifestInputs.every((entry) => sha(path.resolve(root, ...entry.path.split('/'))) === entry.sha256),
    sourceContractStatic: source.includes("const s4cToiModule = require('../material-toi-witness/material-toi-witness.js');")
      && source.includes("preparedFreeCluster: 'AE46B3E461BFD5BE7641E71FD279BB2BF56F7743BFB3137ECB36215498F691BF'")
      && source.includes('const MATERIAL_EVENT_MAIN_TRANSACTION_WIRING_ENABLED = false;')
      && source.includes('const TRACEABLE_CUTTING_CONTINUOUS_TOI_ENABLED = false;')
      && !/s4bGeometryKernelModule\s*\./.test(checkpointSource)
      && !/\.prepareLeastRoot\s*\(/.test(checkpointSource)
      && !/\.finishMaterial\s*\(/.test(checkpointSource)
      && !/\.finishNoMaterial\s*\(/.test(checkpointSource),
    privateBundleBoundary: fs.readFileSync(files.bundle, 'utf8').includes('/* SINGLE_PRIVATE_IIFE_BEGIN */')
      && !/globalThis\.BiteMaterialEventWitness/.test(fs.readFileSync(files.bundle, 'utf8'))
      && !/window\.s4cToiModule\s*=/.test(fs.readFileSync(files.bundle, 'utf8')),
    samplerActuallyRan: browser.schema === 's4c-independent-raw-browser-v1' && !browser.error
      && browser.executed?.parameterPresetApplied === true && browser.executed?.baselineRunCalled === true
      && browser.executed?.witnessCalls === fixedIds.length
      && browser.executed?.attackCalls === Object.keys(expectedAttacks).length
      && browser.executed?.faultCalls === Object.keys(expectedFaults).length,
    baselineIsOnlyRealRapierTip: Array.isArray(description.runScenarioIds)
      && description.runScenarioIds.length === 1
      && description.runScenarioIds[0] === 'baseline-real-sfree-private-78f-parameter-tip'
      && description.realRapierScenarioScope === 'baseline parameter-tip ingress only'
      && run.realRapier === true && run.realPreparedSession === true && run.realParameterTooth === true
      && run.toiStatus === 'contact' && run.toiContactRole === 'tip'
      && run.toiFeatureEdgeId === 'working-face' && run.toiRemovalAllowedLocally === true
      && run.toiCommonPointExact === true && run.toiNormalExact === true
      && finite(run.toiSignedClosingVelocity) && run.toiSignedClosingVelocity > 0,
    exactSingleAdvanceAndAe46Token: run.capturePoint === 'same-post-world-pre-kkt-sfree'
      && run.worldStepDelta === 1 && run.forceApplicationDelta === 1
      && run.freeCaptureDelta === 1 && run.private78fCallCount === 1
      && run.captureKktSolveDelta === 0 && run.capturePublicationWriteDelta === 0
      && run.qWriteDelta === 0 && run.exactReturnedTokenWeakKeyBound === true
      && run.completeCheckpointSfreeCapture === true && run.private78fModuleIdentityBound === true
      && description.sourcePins?.preparedFreeCluster === expected.preparedAe46
      && description.sourcePins?.materialToiWitness === expected.private78f,
    doubleRootExact: run.acceptedRootExact === true && run.configuredRootExact === true
      && run.scenarioRootExact === true && run.exposedRootExact === true
      && run.physicalRootExact === true && run.damageRootExact === true
      && run.bookkeepingRootExact === true && run.solverGroupsRootExact === true
      && run.mlcpAliasRootExact === true && run.forkResidualRootExact === true
      && typeof run.rootBeforeSha256 === 'string' && run.rootBeforeSha256 === run.rootAfterSha256
      && typeof run.scenarioRootBeforeSha256 === 'string'
      && run.scenarioRootBeforeSha256 === run.scenarioRootAfterSha256,
    materialCommonNormalClosingRecomputedFinite: independentCases.every((entry) => entry.sameToiFinite && entry.sameToiExact)
      && found.every((entry) => entry.sameToi && all([
        'featurePointError', 'materialPointError', 'commonPointError',
        'signedClosingError', 'normalMagnitudeError',
      ].map((key) => finite(entry.sameToi[key])))),
    browserWitnessesMatchIndependent78fRecompute: fixedIds.every((id) =>
      publicWitnessMatchesIndependent(byId[id], independentCases.find((entry) => entry.id === id))),
    spatialBudgetsFinite: run.rigidSpatialGate === 8e-5 && run.witnessSpatialGate === 5e-6
      && finite(run.toiMaximumUnresolvedSpatialError)
      && run.toiMaximumUnresolvedSpatialError <= run.witnessSpatialGate
      && found.every((entry) => finite(entry.separation)
        && finite(entry.maximumUnresolvedSpatialError)
        && entry.separation <= entry.rigid80MicrometreGate
        && entry.maximumUnresolvedSpatialError <= entry.witness5MicrometreGate),
    fixedScopeAndBoundaryCasesHonest: witnesses.length === fixedIds.length
      && fixedIds.every((id) => byId[id]?.realRapierWorldStep === false
        && byId[id]?.execution === 'real-edge-private-78f-fixed-declared-se2-motion')
      && byId['tip-first']?.featureVertexId === 'tip'
      && byId['tip-first']?.featureEdgeId === 'working-face'
      && byId['root-first']?.featureVertexId === 'root'
      && byId['root-first']?.featureEdgeId === 'root-edge'
      && byId['root-first']?.domainAction === 'stop-non-working-boundary'
      && byId['back-edge-first']?.featureEdgeId === 'back-edge'
      && byId['back-edge-first']?.domainAction === 'stop-non-working-boundary'
      && byId['rotational-midstep-enter-exit']?.found === true
      && finite(byId['rotational-midstep-enter-exit']?.toi)
      && byId['rotational-midstep-enter-exit'].toi > 0 && byId['rotational-midstep-enter-exit'].toi < 1
      && byId['hole-reentry']?.found === true
      && byId['hole-reentry']?.contactClass === 're-entry'
      && byId['hole-reentry']?.materialRingId === 'polygon-0:hole-0'
      && byId['no-contact']?.found === false && byId['no-contact']?.status === 'no-contact',
    attacksActuallyExecuted: attacks.length === Object.keys(expectedAttacks).length
      && attacks.every((entry) => entry.actualAttackExecuted === true && entry.rejected === true
        && entry.status === 'domain-stop' && entry.errorCode === expectedAttacks[entry.id]
        && entry.worldStepDelta === 1 && entry.forceApplicationDelta === 1
        && entry.kktSolveDelta === 0 && entry.publicationWriteDelta === 0 && entry.qWriteDelta === 0
        && entry.acceptedRootExact === true && entry.rootBeforeSha256 === entry.rootAfterSha256
        && entry.capabilityLeaked === false && entry.revocationGenerationAdvanced === true),
    faultsActuallyExecutedAndRolledBack: faults.length === Object.keys(expectedFaults).length
      && faults.every((entry) => entry.actualFaultInjected === true && entry.rejected === true
        && entry.status === 'domain-stop' && entry.errorCode === expectedFaults[entry.id]
        && entry.worldStepDelta === 1 && entry.forceApplicationDelta === 1
        && entry.prepareCallDelta === 0 && entry.kktSolveDelta === 0
        && entry.publicationWriteDelta === 0 && entry.qWriteDelta === 0
        && entry.acceptedRootExact === true && entry.rootBeforeSha256 === entry.rootAfterSha256
        && entry.capabilityLeaked === false && entry.revocationGenerationAdvanced === true),
    fullAuthorityRemovalAndMainAllFalse: description.main === false
      && description.authorityAuthorized === false
      && Array.isArray(description.fullAuthorityBoundFields)
      && description.fullAuthorityBoundFields.length === 0
      && run.main === false && run.ok === false && run.status === 'domain-stop'
      && run.trajectoryAuthorityClosed === false && run.geometryKernelCalled === false
      && run.kktCalled === false && run.publicationWritten === false
      && run.materialRemovalDefined === false && run.fracturePathDefined === false,
    privateAndArgumentBoundaries: Array.isArray(browser.forbidden) && browser.forbidden.length === 0
      && browser.lexical?.s4cToiModule?.type === 'undefined'
      && browser.lexical?.s4cToiModule?.referenceError === true
      && browser.lexical?.moduleTable?.type === 'undefined'
      && browser.lexical?.moduleTable?.referenceError === true
      && Object.values(browser.invalidArgumentsRejected || {}).every(Boolean),
    checkpointBoundFieldsExact: sameJson(description.boundFields, expectedBoundFields)
      && sameJson(run.checkpointBoundFields, expectedBoundFields),
  };
  return checks;
}

async function main() {
  const initial = snapshotFrozen();
  const source = fs.readFileSync(files.source, 'utf8');
  const manifest = JSON.parse(fs.readFileSync(files.manifest, 'utf8'));
  const rebuild = runCleanBuild();
  if (rebuild.exitCode !== 0) throw new Error(`clean rebuild failed: ${rebuild.stderr || rebuild.stdout}`);
  // Requiring the pinned public 78F module is only for an independent numerical
  // replay of the six declared SE(2) witness cases; the browser sample above
  // separately proves that the frozen bundle calls its private copy.
  delete require.cache[require.resolve(files.privateToi)];
  const private78f = require(files.privateToi);
  const caseIds = ['tip-first', 'root-first', 'back-edge-first', 'rotational-midstep-enter-exit', 'hole-reentry', 'no-contact'];
  const independentCases = caseIds.map((id) => summarizeIndependentWitness(id, private78f));
  const browser = await runBrowserSampler();
  const final = snapshotFrozen();
  const checks = validate(source, manifest, browser, rebuild, initial, final, independentCases);
  const report = {
    schema: 'material-private-toi-param-checkpoint-independent-audit-v1',
    status: all(Object.values(checks)) ? 'PASS' : 'FAIL',
    pass: all(Object.values(checks)),
    readOnlyTarget: 'signed work/material-private-toi-param-checkpoint and outputs/bite-simulator were read only; combined artifacts are isolated in this work directory.',
    checks,
    expectedFrozenHashes: expected,
    frozenHashesBefore: initial,
    frozenHashesAfter: final,
    cleanRebuild: {
      directory: rebuild.directory,
      exitCode: rebuild.exitCode,
      bundleSha256: rebuild.bundleSha256,
      manifestSha256: rebuild.manifestSha256,
    },
    scope: {
      realRapier: 'only baseline parameter-tip ingress; exactly one world step and one force application',
      fixedSE2: 'root/back/rotation/hole/no-contact are real Edge/private-78F executions over trusted generator geometry and declared SE(2), not Rapier world steps',
      authority: 'FULL authority, material removal and main remain false; the run domain-stops before 80D/KKT/publication/removal',
    },
    rawBrowserExecution: browser,
    independentSameToiRecomputation: independentCases.map((entry) => ({
      id: entry.id,
      found: entry.result.found === true,
      status: entry.result.status || null,
      toi: entry.result.toi ?? null,
      contactRole: entry.result.witness?.contactRole || null,
      featureEdgeId: entry.result.witness?.featureEdgeId || null,
      materialRingId: entry.result.witness?.materialRingId || null,
      signedClosingVelocity: entry.result.witness?.signedClosingVelocity ?? null,
      maximumUnresolvedSpatialError: entry.result.certificate?.maximumUnresolvedSpatialError ?? null,
      sameToi: entry.sameToi,
      sameToiFinite: entry.sameToiFinite,
      sameToiExact: entry.sameToiExact,
    })),
    auditArtifacts: { runnerSha256: sha(files.runner), harnessSha256: sha(files.harness) },
  };
  fs.writeFileSync(reportFile, `${JSON.stringify(report, null, 2)}\n`, 'utf8');
  console.log(JSON.stringify({
    pass: report.pass,
    failedChecks: Object.entries(checks).filter(([, value]) => !value).map(([key]) => key),
    report: path.relative(root, reportFile).replaceAll('\\', '/'),
    reportSha256: sha(reportFile),
  }, null, 2));
  if (!report.pass) process.exitCode = 1;
}

main().catch((error) => {
  console.error(error?.stack || String(error));
  process.exitCode = 1;
});
