'use strict';

// Independent QA for the work-only spatial plastic-process ledger.
// The production source is read and its functions are extracted verbatim;
// this file never edits or serves the application.
const fs = require('fs');
const path = require('path');
const vm = require('vm');
const assert = require('assert');

const root = path.resolve(__dirname, '..');
const sourcePath = path.join(__dirname, 'material-continuous-prototype-app.js');
const clippingPath = path.join(root, 'outputs', 'bite-simulator', 'assets', 'polygon-clipping', 'polygon-clipping-0.15.7.umd.min.js');
const source = fs.readFileSync(sourcePath, 'utf8');
let assertions = 0;
const check = (condition, message) => { assertions += 1; assert(condition, message); };

function extract(name) {
  const marker = new RegExp(`function\\s+${name}\\b`);
  const match = marker.exec(source);
  if (!match) throw new Error(`missing ${name}`);
  const start = match.index;
  let depth = 0; let quote = null; let escaped = false;
  for (let index = start; index < source.length; index += 1) {
    const ch = source[index];
    if (quote) {
      if (escaped) escaped = false;
      else if (ch === '\\') escaped = true;
      else if (ch === quote) quote = null;
      continue;
    }
    if (ch === '"' || ch === "'" || ch === '`') { quote = ch; continue; }
    if (ch === '{') depth += 1;
    if (ch === '}' && --depth === 0) return source.slice(start, index + 1);
  }
  throw new Error(`unterminated ${name}`);
}

const context = {
  console,
  Math,
  Number,
  MATERIAL_TOI_AREA_TOLERANCE: 1e-14,
  MATERIAL_SWEEP_GEOMETRY_TOLERANCE: 0.00002,
  MATERIAL_ATTEMPT_GEOMETRY_QUANTUM: 1e-10,
  cloneMaterialGeometry: geometry => (geometry || []).map(polygon => polygon.map(ring => ring.map(([x, y]) => [x, y]))),
  number: (value, fallback = 0) => Number.isFinite(Number(value)) ? Number(value) : fallback,
  clamp: (value, min, max) => Math.max(min, Math.min(max, value)),
  point: (x, y) => ({ x, y }),
  add: (a, b) => ({ x: a.x + b.x, y: a.y + b.y }),
  subtract: (a, b) => ({ x: a.x - b.x, y: a.y - b.y }),
  scalePoint: (a, scale) => ({ x: a.x * scale, y: a.y * scale }),
  dot: (a, b) => a.x * b.x + a.y * b.y,
  length: a => Math.hypot(a.x, a.y),
};
vm.createContext(context);
vm.runInContext(fs.readFileSync(clippingPath, 'utf8'), context, { filename: clippingPath });
const extracted = [
  'materialGeometryMoments',
  'materialGeometryArea',
  'pointSegmentDistance',
  'simplifyMaterialGeometryForAttempt',
  'coalescedMaterialUnionForAttempt',
  'materialProcessCellTotals',
  'materialProcessEntryCells',
  'previewMaterialProcessLedger',
  'materialProcessEntryFromCells',
  'materialGeometryBoundaryMeasure',
  'resolvableMaterialProcessGeometry',
  'canonicaliseMaterialProcessLedgerForGeometry',
].map(name => `${extract(name)}\nglobalThis.${name} = ${name};`).join('\n');
vm.runInContext(extracted, context, { filename: sourcePath });

const area = context.materialGeometryArea;
const union = (...geometries) => context.polygonClipping.union(...geometries.filter(Boolean));
const difference = (left, right) => context.polygonClipping.difference(left, right);
const square = (x0, y0, x1, y1) => [[[[x0, y0], [x1, y0], [x1, y1], [x0, y1], [x0, y0]]]];
const close = (left, right, tolerance, message) => check(
  Math.abs(left - right) <= tolerance,
  `${message}: ${left} vs ${right} (tol ${tolerance})`,
);

function overlapArea(left, right) {
  return area(context.polygonClipping.intersection(left, right));
}

function assertLedger(entry, label) {
  const cells = entry?.plasticCells || [];
  const accounted = entry?.accountedGeometry || entry?.geometry || [];
  const cellUnion = cells.length ? union(...cells.map(cell => cell.geometry)) : [];
  const accountedArea = area(accounted);
  const unionArea = area(cellUnion);
  const scale = Math.max(1, accountedArea, unionArea);
  close(unionArea, accountedArea, scale * 1e-9, `${label}: union(cell) area == accountedGeometry area`);
  let required = 0; let paid = 0;
  cells.forEach((cell, index) => {
    const cellArea = area(cell.geometry);
    const requiredPerArea = Math.max(0, Number(cell.requiredPerArea) || 0);
    const paidPerArea = Math.max(0, Number(cell.paidPerArea) || 0);
    required += cellArea * requiredPerArea;
    paid += cellArea * paidPerArea;
    check(paidPerArea <= requiredPerArea + 1e-15, `${label}: cell ${index} paid density exceeds required density`);
    for (let other = 0; other < index; other += 1) {
      const overlap = overlapArea(cell.geometry, cells[other].geometry);
      check(overlap <= 1e-18, `${label}: cells ${other}/${index} overlap by ${overlap}`);
    }
  });
  close(required, Number(entry.plasticRequired) || 0, Math.max(1e-18, Math.abs(required) * 1e-9), `${label}: required total`);
  close(paid, Number(entry.plasticPaid) || 0, Math.max(1e-18, Math.abs(paid) * 1e-9), `${label}: paid total`);
  check(paid <= required + Math.max(1e-18, Math.abs(required) * 1e-9), `${label}: paid total exceeds required total`);
  return { cellCount: cells.length, accountedArea, unionArea, required, paid };
}

// A normal disjoint ledger produced by the real constructor.
const first = context.materialProcessEntryFromCells(
  null,
  [
    { geometry: square(0, 0, 1, 1), requiredPerArea: 3, paidPerArea: 1 },
    { geometry: square(2, 0, 3, 1), requiredPerArea: 5, paidPerArea: 2 },
  ],
  [],
);
const normal = assertLedger(first, 'normal disjoint ledger');
check(normal.cellCount === 2, 'normal ledger keeps distinct constitutive densities');

// Exercise repeated preview/split/consume operations from the production path.
let entry = context.materialProcessEntryFromCells(
  null,
  [{ geometry: square(0, 0, 4, 1), requiredPerArea: 2, paidPerArea: 0 }],
  [],
);
for (const [x0, x1] of [[0, 1.2], [0.8, 2.1], [1.9, 3.2], [3.0, 4.0]]) {
  const active = square(x0, 0, x1, 1);
  const preview = context.previewMaterialProcessLedger(entry, active, 2);
  check(preview.ok && !preview.empty, `slice ${x0}-${x1}: preview is valid`);
  entry = context.materialProcessEntryFromCells(
    entry,
    preview.outsideCells,
    [],
    preview.insideRequired,
    0,
    preview.remainingAccountedGeometry,
  );
  assertLedger(entry, `repeated slice ${x0}-${x1}`);
}

// The QA must detect the exact class of bad state that would invalidate a
// convergence claim: overlapping cells or totals not matching their geometry.
let rejected = false;
try {
  assertLedger(context.materialProcessEntryFromCells(
    null,
    [
      { geometry: square(0, 0, 1, 1), requiredPerArea: 3, paidPerArea: 0 },
      { geometry: square(.5, 0, 1.5, 1), requiredPerArea: 3, paidPerArea: 0 },
    ],
    [],
  ), 'overlap detector');
} catch (_) { rejected = true; }
check(rejected, 'overlap detector must reject overlapping process cells');

// A later cut must remove stale process obligations and their paid balance from
// the live zone without refunding already-dissipated work.
const clippedSource = context.materialProcessEntryFromCells(
  { featureId: 'tooth:0', toothOrder: 0 },
  [{ geometry: square(0, 0, 2, 1), requiredPerArea: 4, paidPerArea: 1.5 }],
  square(0, 0, 2, 1),
);
const clipped = context.canonicaliseMaterialProcessLedgerForGeometry(
  { incubationByFeature: [clippedSource] },
  square(1, 0, 3, 1),
);
check(clipped.ok, 'surviving-material clip succeeds');
check(clipped.entries.length === 1, 'partially surviving feature remains');
const clippedTotals = assertLedger(clipped.entries[0], 'partially surviving feature');
close(clippedTotals.accountedArea, 1, 1e-9, 'only surviving process area remains');
close(clipped.incubationWork, 1.5, 1e-9, 'surviving paid work remains spatially bound');
close(clipped.newlyDiscardedPaid, 1.5, 1e-9, 'removed paid work becomes dissipation, not future credit');

const removed = context.canonicaliseMaterialProcessLedgerForGeometry(
  { incubationByFeature: [clippedSource] },
  square(3, 0, 4, 1),
);
check(removed.ok, 'fully removed process clip succeeds');
check(removed.entries.length === 0, 'fully removed process obligation is dropped');
check(removed.incubationGeometry.length === 0, 'fully removed process geometry is empty');
close(removed.newlyDiscardedPaid, 3, 1e-9, 'all paid work remains historical dissipation');

const unresolvedStrip = square(0, 0, .001, .000001);
const resolvedBlock = square(0, 0, .0001, .0001);
check(context.resolvableMaterialProcessGeometry(unresolvedStrip).geometry.length === 0,
  'process strip thinner than swept-boundary uncertainty is not persisted');
check(context.resolvableMaterialProcessGeometry(resolvedBlock).geometry.length === 1,
  'process block larger than its swept-boundary uncertainty is retained');

console.log(JSON.stringify({ ok: true, assertions, normalCellCount: normal.cellCount, finalCellCount: entry.plasticCells.length }));
