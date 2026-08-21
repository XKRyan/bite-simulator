'use strict';

const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const subject = require('./exact-triangle-sweep-area.js');

const GRID = 16;
const CASES = 256;
function abs(value) { return value < 0n ? -value : value; }
function gcd(left, right) {
  let a = abs(left); let b = abs(right);
  while (b) { const next = a % b; a = b; b = next; }
  return a || 1n;
}
function rat(n, d = 1n) {
  let numerator = BigInt(n); let denominator = BigInt(d);
  if (denominator < 0n) { numerator = -numerator; denominator = -denominator; }
  const divisor = gcd(numerator, denominator);
  return { n: numerator / divisor, d: denominator / divisor };
}
function add(left, right) { return rat(left.n * right.d + right.n * left.d, left.d * right.d); }
function subtract(left, right) { return rat(left.n * right.d - right.n * left.d, left.d * right.d); }
function multiply(left, right) { return rat(left.n * right.n, left.d * right.d); }
function divide(left, right) { return rat(left.n * right.d, left.d * right.n); }
function key(value) { return `${value.n}/${value.d}`; }
function rng(seed = 0x96f341ad) {
  let state = seed >>> 0;
  return () => { state ^= state << 13; state ^= state >>> 17; state ^= state << 5;
    return (state >>> 0) / 0x100000000; };
}

const random = rng();
const triangles = [[[0, 0], [1, 0], [1, 1]], [[0, 0], [1, 1], [0, 1]]];
let maximumUnionPieces = 0; let totalRectangles = 0;
for (let caseIndex = 0; caseIndex < CASES; caseIndex += 1) {
  const rectangleCount = 1 + Math.floor(random() * 8); const rectangles = [];
  const covered = Array.from({ length: GRID }, () => Array(GRID).fill(false));
  for (let rectangleIndex = 0; rectangleIndex < rectangleCount; rectangleIndex += 1) {
    const x0 = Math.floor(random() * GRID); const y0 = Math.floor(random() * GRID);
    const x1 = x0 + 1 + Math.floor(random() * (GRID - x0));
    const y1 = y0 + 1 + Math.floor(random() * (GRID - y0));
    rectangles.push([[x0 / GRID, y0 / GRID], [x1 / GRID, y0 / GRID],
      [x1 / GRID, y1 / GRID], [x0 / GRID, y1 / GRID]]);
    for (let x = x0; x < x1; x += 1) for (let y = y0; y < y1; y += 1) covered[x][y] = true;
  }
  const result = subject.computeExactFreshArea({ triangles, cells: rectangles });
  assert.equal(result.ok, true, `case ${caseIndex}: ${result.reason}`);
  maximumUnionPieces = Math.max(maximumUnionPieces, result.unionPieceCount);
  totalRectangles += rectangleCount;
  let cellCount = 0n; let firstXNumerator = 0n; let firstYNumerator = 0n;
  let polarNumerator = 0n;
  for (let x = 0; x < GRID; x += 1) for (let y = 0; y < GRID; y += 1) if (covered[x][y]) {
    cellCount += 1n; firstXNumerator += BigInt(2 * x + 1); firstYNumerator += BigInt(2 * y + 1);
    polarNumerator += BigInt((x + 1) ** 3 - x ** 3 + (y + 1) ** 3 - y ** 3);
  }
  const area = rat(cellCount, BigInt(GRID ** 2));
  const firstX = rat(firstXNumerator, BigInt(2 * GRID ** 3));
  const firstY = rat(firstYNumerator, BigInt(2 * GRID ** 3));
  const polarOrigin = rat(polarNumerator, BigInt(3 * GRID ** 4));
  const centroidX = divide(firstX, area); const centroidY = divide(firstY, area);
  const polarCentroid = subtract(polarOrigin, multiply(area,
    add(multiply(centroidX, centroidX), multiply(centroidY, centroidY))));
  assert.equal(`${result.exactAreaNumerator}/${result.exactAreaDenominator}`, key(area), `area case ${caseIndex}`);
  assert.equal(result.exactMoments.firstMomentX, key(firstX), `first x case ${caseIndex}`);
  assert.equal(result.exactMoments.firstMomentY, key(firstY), `first y case ${caseIndex}`);
  assert.equal(result.exactMoments.polarSecondMomentOrigin, key(polarOrigin), `polar origin case ${caseIndex}`);
  assert.equal(result.exactMoments.polarSecondMomentCentroid, key(polarCentroid), `polar centroid case ${caseIndex}`);
}

const report = { schema: 'exact-sweep-cell-union-random-audit-v1', pass: true,
  cases: CASES, gridCellsPerCase: GRID ** 2, totalRectangles, maximumUnionPieces };
const serialized = `${JSON.stringify(report, null, 2)}\n`;
fs.writeFileSync(path.join(__dirname, 'union-random-audit-report.json'), serialized);
process.stdout.write(serialized);
