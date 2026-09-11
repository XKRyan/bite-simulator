'use strict';

// Short, dependency-free QA for the work-only engagement phase API.
const fs = require('fs');
const path = require('path');
const vm = require('vm');
let assertionCount = 0;
const assert = new Proxy(require('assert'), {
  apply(target, receiver, args) { assertionCount += 1; return Reflect.apply(target, receiver, args); },
  get(target, key) {
    const value = target[key];
    return typeof value === 'function'
      ? (...args) => { assertionCount += 1; return Reflect.apply(value, target, args); }
      : value;
  },
});

const root = path.resolve(__dirname, '..');
const appPath = path.join(__dirname, 'material-continuous-prototype-app.js');
const clipPath = path.join(root, 'outputs', 'bite-simulator', 'assets', 'polygon-clipping', 'polygon-clipping-0.15.7.umd.min.js');
const source = fs.readFileSync(appPath, 'utf8');
const clone = (value) => value == null ? value : JSON.parse(JSON.stringify(value));
const context = {
  console, Math, structuredClone, polygonClipping: {},
  MATERIAL_TOI_AREA_TOLERANCE: 1e-14,
  cloneMaterialGeometry: clone,
  clonePlaybackPlain: clone,
  materialGeometryArea: (geometry) => (geometry || []).reduce((sum, polygon) => {
    const ring = polygon[0] || []; let area = 0;
    for (let i = 0; i + 1 < ring.length; i += 1) area += ring[i][0] * ring[i + 1][1] - ring[i + 1][0] * ring[i][1];
    return sum + Math.abs(area) / 2;
  }, 0),
};
vm.createContext(context);
vm.runInContext(fs.readFileSync(clipPath, 'utf8'), context, { filename: clipPath });

function extract(name) {
  const marker = new RegExp(`(?:function|const|let|var)\\s+${name}\\b`);
  const match = marker.exec(source);
  if (!match) throw new Error(`missing ${name}`);
  const start = source.indexOf('function', match.index);
  let depth = 0; let quote = null; let escaped = false;
  for (let i = start; i < source.length; i += 1) {
    const ch = source[i];
    if (quote) { if (escaped) escaped = false; else if (ch === '\\') escaped = true; else if (ch === quote) quote = null; continue; }
    if (ch === '"' || ch === "'" || ch === '`') { quote = ch; continue; }
    if (ch === '{') depth += 1;
    if (ch === '}' && --depth === 0) return source.slice(start, i + 1);
  }
  throw new Error(`unterminated ${name}`);
}

const previewCode = extract('previewMaterialEngagement');
const commitCode = extract('commitMaterialEngagement');
vm.runInContext(`globalThis.previewMaterialEngagement = ${previewCode}; globalThis.commitMaterialEngagement = ${commitCode};`, context);
const preview = context.previewMaterialEngagement;
const commit = context.commitMaterialEngagement;

const square = [[0, 0], [1, 0], [1, 1], [0, 1], [0, 0]];
const sweep = (a, b, paths = [1, 1, 1, 1, 1]) => ({ sweptGeometry: [[[a, 0], [b, 0], [b, 1], [a, 1], [a, 0]]], vertexPathLengths: paths, startLoop: 0, endLoop: 0 });
const run = (featureId, sw, previous = null) => preview(previous, featureId, sw, [[square]], 0.01, 0, 1, true);
const close = (a, b, eps = 1e-9) => assert(Math.abs(a - b) <= eps, `${a} != ${b}`);

const one = run('f', sweep(0, 1, [1, 1, 1, 1, 1]));
const first = run('f', sweep(0, .5, [.5, .5, .5, .5, .5]));
const multi = run('f', sweep(.5, 1, [.5, .5, .5, .5, .5]), first);
close(one.engagedArea, multi.engagedArea); close(one.pathLength, multi.pathLength); close(one.thresholdResidual, multi.thresholdResidual);
close(run('f', sweep(0, 1, [1, 1, 1, 1, 1]), one).engagedArea, 1);
const maxed = run('f', sweep(0, .5, [2, 0, 2, 0, 2]));
const maxed2 = run('f', sweep(.5, 1, [0, 2, 0, 2, 0]), maxed);
close(maxed2.pathLength, 2);
assert(one.active && ['ploughing', 'chip'].includes(one.phase));
for (const key of ['entryGeometry', 'engagedGeometry', 'vertexArcLengths', 'pathLength', 'engagedArea', 'thresholdResidual', 'chipLatchedAt', 'thresholdCrossing']) assert(Object.prototype.hasOwnProperty.call(one, key), key);
assert(multi.pathLength >= one.pathLength, 'vertex path length must accumulate across slices');
assert.strictEqual(one.phase, 'ploughing', 'preview must not latch chip');
const immutableSweep = sweep(.5, 1);
const immutableEntry = [[square]];
const before = JSON.stringify({ previous: first, sweep: immutableSweep, entry: immutableEntry });
preview(first, 'f', immutableSweep, immutableEntry, .01, 1, 2, true);
assert.strictEqual(JSON.stringify({ previous: first, sweep: immutableSweep, entry: immutableEntry }), before);
const reset = run('other', sweep(0, .5, [.5, .5, .5, .5, .5]), one);
close(reset.pathLength, .5); close(reset.engagedArea, .5);
assert.strictEqual(reset.vertexArcLengths.length, one.vertexArcLengths.length);
const state = { materialEventState: { phase: 'ploughing', featureId: 'f' } };
const beforeRefusal = JSON.stringify(state);
assert.strictEqual(commit(state, { engagementCandidate: one, phaseBoundaryResolved: false }).ok, false);
assert.strictEqual(JSON.stringify(state), beforeRefusal);
const legal = { ...one, thresholdCrossing: true, phase: 'ploughing', lastTime: 1 };
commit(state, { engagementCandidate: legal, phaseBoundaryResolved: true });
assert.strictEqual(state.materialEventState.phase, 'chip');
const beforeChip = clone(state.materialEventState);
const downgrade = commit(state, { engagementCandidate: {
  ...one, phase: 'ploughing', thresholdCrossing: false,
}, phaseBoundaryResolved: true });
assert.strictEqual(downgrade.ok, false, 'locked chip must reject ploughing downgrade');
assert.strictEqual(JSON.stringify(state.materialEventState), JSON.stringify(beforeChip));
for (const malformed of [
  { ...one, active: false },
  { ...one, phase: 'unknown' },
  { ...one, featureId: null },
]) {
  assert.strictEqual(commit(state, { engagementCandidate: malformed, phaseBoundaryResolved: true }).ok, false,
    'malformed candidate must be rejected');
}
const continuedChip = preview(state.materialEventState, 'f', sweep(0, 1), [[square]], .01, 1, 2, true);
assert.strictEqual(commit(state, { engagementCandidate: continuedChip }).ok, true);
assert.strictEqual(state.materialEventState.phase, 'chip');
const latched = preview(state.materialEventState, 'f', sweep(0, 1, [1000, 1000, 1000, 1000, 1000]), [[square]], .01, 2, 3, true);
assert(latched.thresholdResidual < 0);
assert.strictEqual(latched.phase, 'chip');
assert.strictEqual(latched.chipLatchedAt, 1);
// Exercise the production time-bracket resolver with a deterministic trajectory.
// Both free candidates and forced-response wrappers must return the last replay.
const rootStart = source.indexOf('    const resolvePhaseCandidate = (evaluateAt, fraction) => {');
const rootEnd = source.indexOf('    const evaluate = fraction =>', rootStart);
assert(rootStart >= 0 && rootEnd > rootStart);
vm.runInContext(`(() => {
  const rootFraction = 0, MATERIAL_TOI_MAX_RECURSION = 28;
  const MATERIAL_TOI_FRACTION_TOLERANCE = 1 / 2 ** 24, FIXED_DT = .0005;
  const context = { intervalDt: .0005, tickOffset: 0 }, state = { sim: { time: 1 } };
  const number = value => Number(value) || 0;
  ${source.slice(rootStart, rootEnd)}
  globalThis.resolveTestPhase = resolvePhaseCandidate;
})()`, context);
for (const forced of [false, true]) {
  let lastFraction = null;
  const resolved = context.resolveTestPhase(fraction => {
    lastFraction = fraction;
    const candidate = { ok: true, fraction, estimate: { engagementCandidate: {
      thresholdCrossing: fraction > .375, thresholdResidual: fraction - .375,
    } } };
    return forced ? { ok: true, candidate } : candidate;
  }, 1);
  const candidate = resolved.candidate || resolved;
  assert(resolved.ok && candidate.estimate.phaseBoundaryResolved);
  assert(candidate.fraction > .375 && candidate.fraction - .375 <= 1 / 2 ** 24);
  assert.strictEqual(lastFraction, candidate.fraction);
  assert(candidate.estimate.phaseBoundary.bracketDt <= .0005 / 2 ** 24);
}
console.log(JSON.stringify({ ok: true, assertions: assertionCount }));
