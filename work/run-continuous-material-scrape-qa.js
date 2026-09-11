'use strict';

const { spawn } = require('child_process');
const crypto = require('crypto');
const fs = require('fs');
const http = require('http');
const path = require('path');

const root = path.resolve(__dirname, '..');
const argument = name => process.argv.find(value => value.startsWith(`--${name}=`))?.slice(name.length + 3);
const suite = argument('suite') || 'baseline';
const appArgument = argument('app');
const appPath = appArgument ? path.resolve(root, appArgument) : path.join(root, 'outputs', 'bite-simulator', 'app.js');
const publicAppPath = path.join(root, 'outputs', 'bite-simulator', 'app.js');
const pagePath = path.join(root, 'work', 'continuous-material-scrape-qa.html');
const frozenBytes = fs.readFileSync(appPath);
const testedHash = crypto.createHash('sha256').update(frozenBytes).digest('hex').toUpperCase();
const parallel = Math.max(1, Math.min(2, Number(argument('parallel') || 1)));
const horizonOverride = argument('horizon') == null ? null : Math.max(.05, Number(argument('horizon')));
const angleOverride = argument('angle') == null ? 90 : Number(argument('angle'));
const single = argument('single') == null ? null : Number(argument('single'));
const maxRefinementOverride = argument('max-refinement') == null
  ? null
  : Math.max(0, Math.min(11, Number(argument('max-refinement'))));
const outputArgument = argument('output');
const edge = [
  'C:\\Program Files (x86)\\Microsoft\\Edge\\Application\\msedge.exe',
  'C:\\Program Files\\Microsoft\\Edge\\Application\\msedge.exe',
].find(candidate => fs.existsSync(candidate));
if (!edge) throw new Error('Microsoft Edge not found');

const product = (...axes) => axes.reduce((rows, axis) => rows.flatMap(row => axis.map(value => [...row, value])), [[]]);
const makeCase = (caseSuite, weapon, fork, teeth, ratio, angle, horizon, fixedDtMs = .5, extra = {}) => ({
  suite: caseSuite, weapon, fork, teeth, ratio, angle, horizon, fixedDtMs, ...extra,
});
const geometryPairs = product(['param', 'cad'], ['param', 'cad']);

function casesFor(requested) {
  if (requested === 'prototype' || requested === 'single-tooth') return [
    makeCase('prototype', 'param', 'param', 1, .5, angleOverride, horizonOverride || .4, .5, { variant: 'normal' }),
    makeCase('prototype', 'param', 'param', 1, .5, angleOverride, horizonOverride || .4, .5, { variant: 'rollback', failureInjection: 1 }),
  ];
  if (requested === 'baseline') return geometryPairs.map(([weapon, fork]) => makeCase('baseline', weapon, fork, 20, .5, 90, horizonOverride || .75));
  if (requested === 'matrix') return product(['param', 'cad'], ['param', 'cad'], [1, 10, 20], [.5, 2.1], [0, 90, 180, 270])
    .map(([weapon, fork, teeth, ratio, angle]) => makeCase('matrix', weapon, fork, teeth, ratio, angle, horizonOverride || 1));
  if (requested === 'history') return product(['param', 'cad'], ['param', 'cad'], [10, 20], [.5, 2.1])
    .map(([weapon, fork, teeth, ratio]) => makeCase('history', weapon, fork, teeth, ratio, 90, horizonOverride || 1.25));
  if (requested === 'convergence') return geometryPairs.flatMap(([weapon, fork]) => [.5, .25, .125]
    .map(fixedDtMs => makeCase('convergence', weapon, fork, 20, .5, 90, horizonOverride || .75, fixedDtMs)));
  if (requested === 'rollback') return geometryPairs.map(([weapon, fork]) => makeCase('rollback', weapon, fork, 20, .5, 90, horizonOverride || .75, .5, { failureInjection: 1 }));
  if (requested === 'duration') return geometryPairs.flatMap(([weapon, fork]) => [1.25, 4]
    .map(horizon => makeCase('duration', weapon, fork, 20, .5, 90, horizonOverride || horizon)));
  if (requested === 'all') return ['matrix', 'history', 'convergence', 'rollback', 'duration'].flatMap(casesFor);
  throw new Error(`Unknown suite: ${requested}`);
}

let cases = casesFor(suite);
if (Number.isInteger(single)) cases = single >= 0 && single < cases.length ? [cases[single]] : [];
if (!cases.length) throw new Error('No QA cases selected');

const sourceMeta = { testedHash, testedPath: path.relative(root, appPath).replaceAll('\\', '/'), instrumentationVersion: 1 };
const instrumentation = String.raw`
  window.__ContinuousMaterialFailureInjection = { enabled: false, fired: 0, last: null };
  const __continuousMaterialBody = (body, name) => {
    if (!body) return null;
    const position = body.translation(); const velocity = body.linvel();
    const localCom = body.localCom?.() || { x: 0, y: 0 }; const worldCom = body.worldCom?.() || position;
    return {
      name, position: { x: position.x, y: position.y }, rotation: body.rotation(),
      velocity: { x: velocity.x, y: velocity.y }, omega: body.angvel(),
      mass: body.mass?.(), inertia: body.principalInertia?.(),
      localCom: { x: localCom.x, y: localCom.y }, worldCom: { x: worldCom.x, y: worldCom.y },
    };
  };
  const __continuousMaterialMomentum = (bodies) => bodies.filter(Boolean).reduce((sum, body) => {
    const px = body.mass * body.velocity.x; const py = body.mass * body.velocity.y;
    sum[0] += px; sum[1] += py;
    sum[2] += body.inertia * body.omega + body.worldCom.x * py - body.worldCom.y * px;
    return sum;
  }, [0, 0, 0]);
  const __continuousMaterialGroups = (physics) => [
    physics.targetCollider, ...(physics.targetMaterialColliders || []),
    ...(physics.weaponColliders || []), ...(physics.forkColliders || []),
    physics.floorCollider, physics.forkGroundCollider,
  ].filter(Boolean).map((collider) => ({ handle: collider.handle, collisionGroups: collider.collisionGroups?.(), solverGroups: collider.solverGroups?.() }));
  window.__ContinuousMaterialAudit = {
    sourceMeta: () => ({ ...${JSON.stringify(sourceMeta)}, servedFixedDt: FIXED_DT }),
    setFailureInjection(enabled) {
      window.__ContinuousMaterialFailureInjection.enabled = Boolean(enabled);
      return { ...window.__ContinuousMaterialFailureInjection };
    },
    failureInjectionStatus: () => ({ ...window.__ContinuousMaterialFailureInjection }),
    capture(includeWorld = false) {
      const sim = state.sim; const physics = sim?.physics;
      const bodies = physics ? [
        __continuousMaterialBody(physics.robotBody, 'robot'),
        __continuousMaterialBody(physics.weaponBody, 'weapon'),
        __continuousMaterialBody(physics.forkBody, 'fork'),
        __continuousMaterialBody(physics.targetBody, 'target'),
      ] : [];
      const energy = physics ? rigEnergySnapshot() : null;
      const snapshot = includeWorld && physics ? snapshotRapierRig(physics) : null;
      const damage = cloneMaterialDamage(sim?.materialDamage);
      const materialSubstepContext = physics?.materialSubstepContext;
      const materialTransients = physics ? {
        tickStage: clonePlaybackPlain(physics.materialTickStage || null),
        eventState: clonePlaybackPlain(physics.materialEventState || null),
        substepContext: materialSubstepContext ? {
          hasWorldSnapshot: Boolean(materialSubstepContext.worldSnapshot),
          hasBookkeepingSnapshot: Boolean(materialSubstepContext.bookkeepingSnapshot),
          before: clonePlaybackPlain(materialSubstepContext.before || null),
          intervalDt: materialSubstepContext.intervalDt,
          tickOffset: materialSubstepContext.tickOffset,
          tickScale: materialSubstepContext.tickScale,
        } : null,
        substepToi: clonePlaybackPlain(physics.materialSubstepToi
          ? { ...physics.materialSubstepToi, samples: undefined }
          : null),
        deferMaterialRebuild: Boolean(physics.deferMaterialRebuild),
      } : null;
      const bodyVector = bodies.flatMap((body) => body ? [
        body.position.x, body.position.y, body.rotation,
        body.velocity.x, body.velocity.y, body.omega,
        body.mass, body.inertia, body.localCom.x, body.localCom.y,
      ] : []);
      return {
        time: sim?.time ?? null, fixedDt: FIXED_DT,
        materialCuttingRequested: Boolean(physics?.materialCuttingRequested),
        materialCuttingEnabled: Boolean(physics?.materialCuttingEnabled),
        materialResponseEnabled: Boolean(physics?.materialResponseEnabled),
        effectiveWeaponZOverlap: effectiveToolWidthZ('weapon'),
        materialDensity: targetMaterialProperties().density,
        initialTargetMaterialMass: effectiveTargetMass(),
        materialToiAudit: clonePlaybackPlain(physics?.materialToiAudit || window.__MaterialToiAudit || null),
        damage, materialTransients,
        materialStats: { ...(sim?.materialStats || {}) },
        bodies, bodyVector, momentum: __continuousMaterialMomentum(bodies),
        mechanicalEnergy: energy ? rigMechanicalEnergy(energy) : null,
        targetBody: bodies.find((body) => body?.name === 'target') || null,
        colliderSolverGroups: physics ? __continuousMaterialGroups(physics) : [],
        bookkeeping: sim ? {
          driveSlip: sim.driveSlip, forkContact: sim.forkContact, forkEngaged: sim.forkEngaged,
          targetPushedByFork: sim.targetPushedByFork, targetLaunched: sim.targetLaunched,
          target: clonePlaybackPlain(sim.target), currentContact: clonePlaybackPlain(sim.currentContact),
          lastImpact: clonePlaybackPlain(sim.lastImpact), hitCount: sim.hitCount, bodyImpactCount: sim.bodyImpactCount,
          lastImpactTime: sim.lastImpactTime, lastBodyImpactTime: sim.lastBodyImpactTime,
          activeContacts: [...(sim.rigActiveContacts || [])].sort(),
          weaponEpisode: cloneRigEpisode(sim.rigWeaponEpisode),
          trail: clonePlaybackPlain(sim.trail), materialDamage: damage,
          materialStats: { ...(sim.materialStats || {}) }, materialMaxIntrusion: sim.materialMaxIntrusion,
          eventCount: activeEventEntries().length,
          mlcpV2ClusterActive: Boolean(physics?.mlcpV2ClusterActive),
          mlcpV2HandoffTick: Number.isFinite(physics?.mlcpV2HandoffTick) ? physics.mlcpV2HandoffTick : null,
          mlcpV2Audit: clonePlaybackPlain(physics?.mlcpV2Audit || null),
        } : null,
        worldBytes: snapshot?.bytes ? Array.from(snapshot.bytes) : null,
      };
    },
  };
`;

function instrumentSource(fixedDtMs) {
  let source = frozenBytes.toString('utf8');
  const fixedDtSeconds = Number(fixedDtMs) / 1000;
  if (!(fixedDtSeconds > 0)) throw new Error(`Invalid fixedDtMs ${fixedDtMs}`);
  const dtNeedle = '  const FIXED_DT = 0.0005;';
  if (!source.includes(dtNeedle)) throw new Error('FIXED_DT instrumentation needle not found');
  source = source.replace(dtNeedle, `  const FIXED_DT = ${fixedDtSeconds};`);
  if (Number.isFinite(maxRefinementOverride)) {
    const refinementNeedle = '  const RIG_MAX_REFINEMENT = 10;';
    if (!source.includes(refinementNeedle)) throw new Error('RIG_MAX_REFINEMENT instrumentation needle not found');
    source = source.replace(refinementNeedle, `  const RIG_MAX_REFINEMENT = ${maxRefinementOverride};`);
  }

  const cutNeedle = '    damage.cuts.push({ x: localPoint.x, y: localPoint.y, depth: maxAdvance, time: sim.time });';
  if (!source.includes(cutNeedle)) throw new Error('material cut event instrumentation needle not found');
  source = source.replace(cutNeedle, `    damage.cuts.push({
      x: localPoint.x, y: localPoint.y, depth: maxAdvance, time: sim.time,
      version: damage.version, sequence: damage.version, fixedTick: Math.round(sim.time / FIXED_DT),
      toothOrder: contact.toothOrder ?? null, sourceIndex: contact.index ?? null,
      freshArea, removedArea, removedVolume,
      toiAudit: clonePlaybackPlain(sweep.toiAudit || estimate.toiAudit || null),
      momentumLedger: clonePlaybackPlain(estimate.momentumLedger || null),
      energyLedger: clonePlaybackPlain(estimate.energyLedger || null),
    });
    if (window.__ContinuousMaterialFailureInjection?.enabled) {
      window.__ContinuousMaterialFailureInjection.fired += 1;
      window.__ContinuousMaterialFailureInjection.last = {
        time: sim.time, version: damage.version, toothOrder: contact.toothOrder ?? null,
        freshArea, removedArea, removedVolume,
        sequence: damage.version, fixedTick: Math.round(sim.time / FIXED_DT),
        toiAudit: clonePlaybackPlain(sweep.toiAudit || estimate.toiAudit || sim.physics?.materialToiAudit || window.__MaterialToiAudit || null),
        momentumLedger: clonePlaybackPlain(estimate.momentumLedger || null),
        energyLedger: clonePlaybackPlain(estimate.energyLedger || null),
      };
      return -1;
    }`);
  const apiNeedle = '    window.BiteSim = {';
  if (!source.includes(apiNeedle)) throw new Error('public API instrumentation needle not found');
  source = source.replace(apiNeedle, `${instrumentation}\n${apiNeedle}`);
  source = source
    .replace('  function updateReadouts() {', '  function updateReadouts() { return;')
    .replace('  function drawScene() {', '  function drawScene() { return;');
  return Buffer.from(source);
}

const instrumentedByDt = new Map();
const bytesForDt = fixedDtMs => {
  const key = String(fixedDtMs);
  if (!instrumentedByDt.has(key)) instrumentedByDt.set(key, instrumentSource(fixedDtMs));
  return instrumentedByDt.get(key);
};

const mime = {
  '.html': 'text/html; charset=utf-8', '.js': 'text/javascript; charset=utf-8',
  '.css': 'text/css; charset=utf-8', '.wasm': 'application/wasm', '.json': 'application/json; charset=utf-8',
};
const server = http.createServer((request, response) => {
  const requestUrl = new URL(request.url, 'http://127.0.0.1');
  const relative = decodeURIComponent(requestUrl.pathname).replace(/^\/+/, '');
  const target = path.resolve(root, relative || path.relative(root, pagePath));
  if (!target.startsWith(root + path.sep)) return response.writeHead(403).end();
  if (target === publicAppPath) {
    let fixedDtMs = .5;
    try { fixedDtMs = Number(new URL(request.headers.referer).searchParams.get('fixedDtMs') || .5); } catch (_) { /* default */ }
    const bytes = bytesForDt(fixedDtMs);
    response.writeHead(200, {
      'Content-Type': mime['.js'], 'Cache-Control': 'no-store',
      'X-Tested-App-SHA256': testedHash, 'X-QA-Fixed-Dt-Ms': String(fixedDtMs),
    });
    return response.end(bytes);
  }
  fs.readFile(target, (error, data) => {
    if (error) return response.writeHead(404).end();
    response.writeHead(200, { 'Content-Type': mime[path.extname(target)] || 'application/octet-stream', 'Cache-Control': 'no-store' });
    response.end(data);
  });
});

const decodeHtml = text => text
  .replaceAll('&quot;', '"').replaceAll('&#39;', "'").replaceAll('&amp;', '&')
  .replaceAll('&lt;', '<').replaceAll('&gt;', '>');

function runOne(testCase, index, port) {
  const params = new URLSearchParams(Object.entries(testCase).map(([key, value]) => [key, String(value)]));
  const url = `http://127.0.0.1:${port}/work/continuous-material-scrape-qa.html?${params}`;
  const profile = path.resolve(root, 'work', `.edge-continuous-material-${process.pid}-${index}-${Date.now()}`);
  if (!profile.startsWith(path.resolve(root, 'work') + path.sep)) throw new Error('Unsafe QA profile path');
  return new Promise(resolve => {
    const child = spawn(edge, [
      '--headless=new', '--disable-gpu', '--no-sandbox', '--disable-background-timer-throttling',
      '--disable-renderer-backgrounding', `--user-data-dir=${profile}`, '--virtual-time-budget=1200000', '--dump-dom', url,
    ], { windowsHide: true });
    let stdout = ''; let stderr = '';
    child.stdout.on('data', chunk => { stdout += chunk; });
    child.stderr.on('data', chunk => { stderr += chunk; });
    child.on('close', code => {
      const match = stdout.match(/<pre id="out"[^>]*>([\s\S]*?)<\/pre>/);
      let result;
      if (!match || match[1] === 'waiting') result = { ...testCase, pass: false, code, error: 'no report', stderr: stderr.slice(-1600), tail: stdout.slice(-1200) };
      else {
        const decoded = decodeHtml(match[1]);
        try { result = { ...testCase, ...JSON.parse(decoded) }; }
        catch (error) { result = { ...testCase, pass: false, code, error: String(error), decoded: decoded.slice(0, 2000), stderr: stderr.slice(-1000) }; }
      }
      let profileRemoved = false;
      try { if (fs.existsSync(profile)) fs.rmSync(profile, { recursive: true, force: true }); profileRemoved = !fs.existsSync(profile); }
      catch (_) { profileRemoved = false; }
      resolve({ ...result, profileRemoved });
    });
  });
}

async function runPool(port) {
  const results = new Array(cases.length); let cursor = 0;
  async function worker() {
    while (cursor < cases.length) {
      const index = cursor; cursor += 1;
      results[index] = await runOne(cases[index], index, port);
    }
  }
  await Promise.all(Array.from({ length: Math.min(parallel, cases.length) }, worker));
  return results;
}

const relativeDifference = (a, b, floor = 1e-15) => Math.abs(a - b) / Math.max(Math.abs(a), Math.abs(b), floor);
function convergenceChecks(results) {
  const groups = new Map();
  for (const result of results.filter(entry => entry.suite === 'convergence')) {
    const key = [result.weapon, result.fork, result.teeth, result.ratio, result.angle, result.horizon].join('|');
    if (!groups.has(key)) groups.set(key, []);
    groups.get(key).push(result);
  }
  return [...groups.entries()].map(([key, entries]) => {
    const byDt = new Map(entries.map(entry => [Number(entry.fixedDtMs), entry]));
    const coarse = byDt.get(.5); const medium = byDt.get(.25); const fine = byDt.get(.125);
    const fields = ['removedArea', 'removedVolume', 'removedMass'];
    const scalar = Object.fromEntries(fields.map(field => {
      const a = Number(coarse?.material?.[field]); const b = Number(medium?.material?.[field]); const c = Number(fine?.material?.[field]);
      const first = Math.abs(a - b); const second = Math.abs(b - c);
      const tolerance = field === 'removedArea' ? Number(fine?.material?.areaTolerance || 1e-12)
        : field === 'removedVolume' ? Number(fine?.material?.volumeTolerance || 1e-15)
          : Number(fine?.material?.massTolerance || 1e-10);
      return [field, { coarse: a, medium: b, fine: c, first, second, tolerance,
        contraction: Number.isFinite(first) && Number.isFinite(second) && second <= .75 * first + tolerance,
        fineRelative: relativeDifference(b, c, tolerance),
        fineWithinOnePercent: relativeDifference(b, c, tolerance) <= .01 }];
    }));
    const orders = entries.map(entry => (entry.acceptedEvents || []).map(event => event.toothOrder));
    const orderStable = orders.length === 3 && orders.every(order => JSON.stringify(order) === JSON.stringify(orders[0]));
    const pass = Boolean(coarse && medium && fine) && entries.every(entry => entry.pass)
      && orderStable && Object.values(scalar).every(check => check.contraction && check.fineWithinOnePercent);
    return { key, pass, orderStable, scalar };
  });
}

function durationChecks(results) {
  const groups = new Map();
  for (const result of results.filter(entry => entry.suite === 'duration')) {
    const key = [result.weapon, result.fork, result.teeth, result.ratio, result.angle].join('|');
    if (!groups.has(key)) groups.set(key, []);
    groups.get(key).push(result);
  }
  return [...groups.entries()].map(([key, entries]) => {
    const short = entries.find(entry => Math.abs(entry.horizon - 1.25) < 1e-9);
    const long = entries.find(entry => Math.abs(entry.horizon - 4) < 1e-9);
    const checkpoint = long?.checkpoint;
    const scalarEqual = Boolean(short && checkpoint)
      && ['removedArea', 'removedVolume', 'removedMass', 'work'].every(field => {
        const left = Number(short.finalState?.material?.[field]); const right = Number(checkpoint.state?.material?.[field]);
        return Math.abs(left - right) <= Math.max(field === 'removedArea' ? 1e-12 : 1e-10, Math.max(Math.abs(left), Math.abs(right)) * 1e-8);
      });
    const bodyEqual = Boolean(short?.checkpoint?.audit?.bodyVector && checkpoint?.audit?.bodyVector)
      && short.checkpoint.audit.bodyVector.length === checkpoint.audit.bodyVector.length
      && short.checkpoint.audit.bodyVector.every((value, index) => Object.is(value, checkpoint.audit.bodyVector[index]));
    return { key, pass: Boolean(short?.pass && long?.pass && scalarEqual && bodyEqual), scalarEqual, bodyEqual };
  });
}

server.listen(0, '127.0.0.1', async () => {
  const started = Date.now();
  try {
    const results = await runPool(server.address().port);
    const endBytes = fs.readFileSync(appPath);
    const endHash = crypto.createHash('sha256').update(endBytes).digest('hex').toUpperCase();
    const convergence = convergenceChecks(results);
    const duration = durationChecks(results);
    const baselineExpectedFailureObserved = suite === 'baseline'
      && results.every(result => result.expectedBaselineFailureObserved === true);
    const acceptancePass = results.every(result => result.pass)
      && convergence.every(group => group.pass) && duration.every(group => group.pass)
      && testedHash === endHash;
    const aggregate = {
      acceptancePass,
      baselineExpectedFailureObserved,
      suite, testedHash, endHash, productionChangedDuringAudit: testedHash !== endHash,
      testedPath: path.relative(root, appPath).replaceAll('\\', '/'), sourceFrozenAtStart: true,
      fixedDtSourceMutationOnlyInServedWorkCopy: true,
      elapsedWallMs: Date.now() - started,
      counts: { total: results.length, passed: results.filter(result => result.pass).length, failed: results.filter(result => !result.pass).length },
      convergence, duration,
      summary: results.map(result => ({
        pass: result.pass, expectedBaselineFailureObserved: result.expectedBaselineFailureObserved,
        suite: result.suite, variant: result.variant, weapon: result.weapon, fork: result.fork, teeth: result.teeth,
        ratio: result.ratio, angle: result.angle, horizon: result.horizon, fixedDtMs: result.fixedDtMs,
        time: result.runtime?.time, failureDomain: result.runtime?.failureDomain,
        runtimeContactMode: result.runtime?.runtimeContactMode,
        removedArea: result.material?.removedArea, acceptedEventCount: result.material?.acceptedEventCount,
        distinctToothOrders: result.material?.distinctToothOrders, gates: result.gates,
        error: result.error, profileRemoved: result.profileRemoved,
      })),
      results,
    };
    const defaultOutput = path.join(root, 'work', `continuous-material-scrape-report-${suite}${Number.isInteger(single) ? `-single${single}` : ''}.json`);
    const outputPath = outputArgument ? path.resolve(root, outputArgument) : defaultOutput;
    if (!outputPath.startsWith(path.resolve(root, 'work') + path.sep)) throw new Error('QA report must stay under work/');
    fs.writeFileSync(outputPath, `${JSON.stringify(aggregate, null, 2)}\n`);
    console.log(JSON.stringify({ ...aggregate, results: undefined, report: path.relative(root, outputPath).replaceAll('\\', '/') }, null, 2));
    if (suite === 'baseline') {
      if (!baselineExpectedFailureObserved || testedHash !== endHash) process.exitCode = 1;
    } else if (!acceptancePass) process.exitCode = 1;
  } catch (error) {
    console.error(error.stack || error); process.exitCode = 2;
  } finally {
    server.close();
  }
});
