import assert from 'node:assert/strict';
import { spawnSync } from 'node:child_process';
import { createHash } from 'node:crypto';
import { readFile } from 'node:fs/promises';
import { fileURLToPath } from 'node:url';
import test from 'node:test';
import {
  DxfValidationError,
  prepareSolidTriangulation,
  validateSingleOuterSolid,
} from '../friend-dxf-port/src/dxf-strict.mjs';

const EXPECTED = Object.freeze({
  combinedSource: '06B6EC1F6E611DDD4122652DC9763D8F75129904D224ACCA3CABB1AF3D71D0AA',
  combinedBundle: 'C100E7FBF724448643D3CF6123EEDECF201A923321838E7B27299CFCC809A2A9',
  combinedManifest: '27528C05B74BE1ACC777FF954B3104405586F48E3EC18EA2A1B596C0132853D2',
  combinedPlanFile: '3E2C3D91950E87408419A814CD93A41F202A88FC40F061514807ED14D865DB73',
  s4c: '81953B2929DA15C4DBCCB821C81CBB15C8164C8BCFEB6812F2C1217CDBE56A1C',
  dxf: 'AA04E8839B7C79E2E41E74C36B16135ECB289C96953C6899E6DC8BB7524E5638',
  s3: '2381261205DCF64E82439480DDF937DFCE7AD10872FF2509334900E0B63B89D7',
  strictPort: 'BE5DF7D7A2C4B5CF735073E4A653C55FD928F5E112F8F162B2E25F22327C1F1B',
  private78f: '78F53C3ACA51678BC3BA16336E97BD6F09084AE257F7C3C558450974E28D5166',
  production: '3A58BFB55ACA173DB63435F75C7FDF19BB0F6D68D42AEABF4E77FC1A7AB2D344',
});
const repoUrl = new URL('../../', import.meta.url);
const repo = fileURLToPath(repoUrl);
const combinedUrl = new URL('work/s4c-private-toi-dxf-combined/', repoUrl);
const paths = Object.freeze({
  combinedSource: new URL('app-combined-candidate.js', combinedUrl),
  combinedBundle: new URL('app-combined-bundle.js', combinedUrl),
  combinedManifest: new URL('bundle-manifest.json', combinedUrl),
  combinedPlanFile: new URL('build-plan.json', combinedUrl),
  s4c: new URL('work/material-private-toi-param-checkpoint/app-s4c-private-toi-param-candidate.js', repoUrl),
  dxf: new URL('work/friend-dxf-app-integration-fix/app-candidate.js', repoUrl),
  s3: new URL('work/material-event-integrator-design/app-s3-free-cluster-candidate.js', repoUrl),
  strictPort: new URL('work/friend-dxf-port/src/dxf-strict.mjs', repoUrl),
  private78f: new URL('work/material-toi-witness/material-toi-witness.js', repoUrl),
  production: new URL('outputs/bite-simulator/app.js', repoUrl),
});
const sha = (bytes) => createHash('sha256').update(bytes).digest('hex').toUpperCase();
const p = (x, y) => ({ x, y });
const path = (points) => ({ type: 'polyline', closed: true, points });
const square = (x0, y0, x1, y1) => [p(x0, y0), p(x1, y0), p(x1, y1), p(x0, y1)];
const solidFrom = (...rings) => validateSingleOuterSolid(rings.map(path)).solid;
const expectCode = (fn, code) => assert.throws(fn, (error) => error instanceof DxfValidationError && error.code === code);

function changedLines(left, right) {
  const result = spawnSync('git', ['diff', '--no-index', '-U0', '--', fileURLToPath(left), fileURLToPath(right)], {
    cwd: repo, encoding: 'utf8', windowsHide: true,
  });
  assert.ok(result.status === 0 || result.status === 1, result.stderr || `git diff exited ${result.status}`);
  return result.stdout.split(/\r?\n/).filter((line) => (
    (/^[+-]/.test(line)) && !/^(---|\+\+\+)/.test(line)
  ));
}

test('combined source, bundle, manifest, parents, pins, baseline, and production are locked', async () => {
  const bytes = Object.fromEntries(await Promise.all(Object.entries(paths).map(async ([key, url]) => [key, await readFile(url)])));
  for (const [key, expected] of Object.entries(EXPECTED)) assert.equal(sha(bytes[key]), expected, key);
  const plan = JSON.parse(bytes.combinedPlanFile.toString('utf8'));
  const manifest = JSON.parse(bytes.combinedManifest.toString('utf8'));
  assert.equal(plan.planDigest, '0250944AA850F97B4A75E46AC567B65E6EF4AABAC27FF542A53E7B087606ED30');
  assert.equal(manifest.planDigest, plan.planDigest);
  assert.equal(manifest.artifact.bundleSha256, EXPECTED.combinedBundle);
  assert.equal(manifest.buildPlan.inputs.find((entry) => entry.id === 'app238')?.sha256, EXPECTED.combinedSource);
  assert.equal(manifest.buildPlan.inputs.find((entry) => entry.id === 'materialToiWitness')?.sha256, EXPECTED.private78f);
});

test('source-level union is exactly both parent deltas with S4c ownership preserved', () => {
  const dxfDelta = changedLines(paths.s3, paths.dxf);
  const appliedDxfDelta = changedLines(paths.s4c, paths.combinedSource);
  const s4cDelta = changedLines(paths.s3, paths.s4c);
  const preservedS4cDelta = changedLines(paths.dxf, paths.combinedSource);
  assert.equal(dxfDelta.length, 1045);
  assert.deepEqual(appliedDxfDelta, dxfDelta, 'combined contains a modified or additional DXF change line');
  assert.equal(s4cDelta.length, 1368);
  assert.deepEqual(preservedS4cDelta, s4cDelta, 'combined altered an S4c-owned change line');
});

test('top-level conflict resolution keeps S4c private ownership before the exact strict port', async () => {
  const source = (await readFile(paths.combinedSource)).toString('utf8');
  const s4cStart = source.indexOf('  /* GEOMETRY_PATH_AUTHORITY_REAL_APP_BROWSER_QA');
  const strictStart = source.indexOf('  // STRICT_DXF_FROZEN_PORT_BEGIN');
  const witnessStart = source.indexOf('  // MATERIAL_EVENT_WITNESS_PORT_BEGIN');
  assert.ok(s4cStart > 0 && strictStart > s4cStart && witnessStart > strictStart);
  assert.equal((source.match(/STRICT_DXF_FROZEN_PORT_BEGIN/g) || []).length, 1);
  assert.equal((source.match(/S4C_PRIVATE_WITNESS_CONTEXT/g) || []).length > 0, true);
  assert.equal((source.match(/<<<<<<<|>>>>>>>|\|\|\|\|\|\|\|/g) || []).length, 0);

  const port = (await readFile(paths.strictPort)).toString('utf8');
  const begin = '  const BiteStrictDxfPort = (() => {';
  const end = '    return Object.freeze({';
  const start = source.indexOf(begin, strictStart);
  const finish = source.indexOf(end, start);
  assert.ok(start >= 0 && finish > start);
  const embedded = source.slice(start + begin.length, finish)
    .replace(/^\r?\n/, '')
    .split(/\r?\n/)
    .map((line) => line.startsWith('    ') ? line.slice(4) : line)
    .join('\n')
    .trimEnd();
  assert.equal(embedded, port.replace(/^export\s+/gm, '').trimEnd());
});

test('automatic mass properties, preset/import inertia gates, flatMap fix, and domain stops remain closed', async () => {
  const source = (await readFile(paths.combinedSource)).toString('utf8');
  assert.match(source, /delete incoming\.weaponInertia;/);
  assert.match(source, /name !== 'weaponInertia'/);
  assert.match(source, /inertiaField\.readOnly = true;/);
  assert.match(source, /weaponInertiaDerivedFromMassAndGeometry: true/);
  assert.match(source, /automaticWeaponMassProperties\(\)/);
  assert.equal((source.match(/flatMap\(samplePathForCollision\)/g) || []).length, 0);
  assert.equal((source.match(/flatMap\(\(path\) => samplePathForCollision\(path\)\)/g) || []).length, 5);
  assert.match(source, /const MATERIAL_EVENT_MAIN_TRANSACTION_WIRING_ENABLED = false;/);
  assert.match(source, /const TRACEABLE_CUTTING_CONTINUOUS_TOI_ENABLED = false;/);
  assert.doesNotMatch(source, /const (?:MATERIAL_EVENT_MAIN_TRANSACTION_WIRING_ENABLED|TRACEABLE_CUTTING_CONTINUOUS_TOI_ENABLED) = true;/);
  assert.match(source, /const s4cToiModule = require\('\.\.\/material-toi-witness\/material-toi-witness\.js'\);/);
  assert.equal((source.match(/s4cToiModule\.findEarliestTriangleMaterialWitness\(/g) || []).length, 2);
  assert.doesNotMatch(source, /strictDxfQaTrace|QA_TRACE_FORK_IMPORT/);
});

test('combined bundle retains private module and strict parser boundaries', async () => {
  const bundle = (await readFile(paths.combinedBundle)).toString('utf8');
  assert.match(bundle, /\/\* SINGLE_PRIVATE_IIFE_BEGIN \*\//);
  assert.match(bundle, /const __modules=Object\.freeze\(/);
  assert.doesNotMatch(bundle, /globalThis\.BiteMaterialEventWitness/);
  assert.doesNotMatch(bundle, /window\.s4cToiModule\s*=/);
  assert.doesNotMatch(bundle, /(?:window|globalThis)\s*(?:\.|\[)\s*['"]?(?:BiteStrictDxfPort|DxfValidationError|prepareSolidTriangulation|parseStrictPlanarDxf)/);
});

test('both independent equal-area wrong-coverage attacks remain rejected', () => {
  const simple = solidFrom(square(0, 0, 4, 4));
  expectCode(
    () => prepareSolidTriangulation(simple, () => [0, 1, 2, 0, 1, 3], { geometryFloor: 1e-6 }),
    'DXF_TRIANGLE_OVERLAP',
  );
  const hole = solidFrom(square(0, 0, 4, 3), square(1, 1, 3, 2));
  const wrong = [
    0, 1, 4, 1, 5, 4,
    4, 5, 6, 4, 6, 7,
    2, 3, 6, 3, 7, 6,
    3, 0, 7, 0, 4, 7,
  ];
  expectCode(
    () => prepareSolidTriangulation(hole, () => wrong, { geometryFloor: 0.1 }),
    'DXF_TRIANGLE_HOLE_COVERAGE',
  );
});
