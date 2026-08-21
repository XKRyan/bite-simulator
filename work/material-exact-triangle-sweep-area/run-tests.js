'use strict';

const assert = require('node:assert/strict');
const subject = require('./exact-triangle-sweep-area.js');

const checks = [];
function check(name, run) {
  const started = Date.now(); run(); checks.push({ name, pass: true, elapsedMs: Date.now() - started });
}

function squareTriangles(size = 1) {
  return [
    [[0, 0], [size, 0], [size, size]],
    [[0, 0], [size, size], [0, size]],
  ];
}

check('dyadic rectangle intersection is exact', () => {
  const result = subject.computeExactFreshArea({
    triangles: squareTriangles(),
    cells: [[[0.25, -1], [0.75, -1], [0.75, 2], [0.25, 2]]],
  });
  assert.equal(result.ok, true, result.reason);
  assert.equal(result.exactAreaNumerator, '1');
  assert.equal(result.exactAreaDenominator, '2');
  assert.equal(result.area, 0.5);
  assert.deepEqual(result.centroid, { x: 0.5, y: 0.5 });
  assert.equal(result.exactMoments.firstMomentX, '1/4');
  assert.equal(result.exactMoments.firstMomentY, '1/4');
  assert.equal(result.exactMoments.polarSecondMomentOrigin, '29/96');
  assert.equal(result.exactMoments.polarSecondMomentCentroid, '5/96');
  assert.ok(result.areaLower <= 0.5 && result.areaUpper >= 0.5);
});

check('hole-like triangulation and disjoint cells do not double count', () => {
  const triangles = [
    [[0, 0], [3, 0], [3, 1]], [[0, 0], [3, 1], [0, 1]],
    [[0, 2], [3, 2], [3, 3]], [[0, 2], [3, 3], [0, 3]],
    [[0, 1], [1, 1], [1, 2]], [[0, 1], [1, 2], [0, 2]],
    [[2, 1], [3, 1], [3, 2]], [[2, 1], [3, 2], [2, 2]],
  ];
  const result = subject.computeExactFreshArea({
    triangles,
    cells: [[[-1, 0.5], [1.5, 0.5], [1.5, 1.5], [-1, 1.5]],
      [[-1, 1.5], [1.5, 1.5], [1.5, 2.5], [-1, 2.5]]],
  });
  assert.equal(result.ok, true, result.reason);
  assert.equal(result.area, 2.5);
});

check('non-dyadic decimal input is treated as its exact binary value', () => {
  const result = subject.computeExactFreshArea({
    triangles: squareTriangles(0.1),
    cells: [[[0.03, -1], [0.07, -1], [0.07, 1], [0.03, 1]]],
  });
  assert.equal(result.ok, true, result.reason);
  assert.ok(result.area > 0.00399999999999999 && result.area < 0.00400000000000001);
  assert.ok(result.areaLower <= result.area && result.area <= result.areaUpper);
});

check('one-ulp outward conversion encloses exact thirds', () => {
  const value = subject._reference.rat(1n, 3n);
  const bounds = subject._reference.toNumberBounds(value);
  const lower = subject._reference.fromNumber(bounds.lower);
  const upper = subject._reference.fromNumber(bounds.upper);
  assert.ok(subject._reference.compare(lower, value) <= 0);
  assert.ok(subject._reference.compare(upper, value) >= 0);
});

check('huge exact numerators and denominators convert without Infinity over Infinity', () => {
  const value = subject._reference.rat((1n << 5000n) + (1n << 4946n), 1n << 5003n);
  const bounds = subject._reference.toNumberBounds(value);
  assert.equal(bounds.nearest, 0.125 + 2 ** -57);
  const lower = subject._reference.fromNumber(bounds.lower);
  const upper = subject._reference.fromNumber(bounds.upper);
  assert.ok(subject._reference.compare(lower, value) <= 0);
  assert.ok(subject._reference.compare(upper, value) >= 0);
});

check('overlapping material triangles stop instead of double counting', () => {
  const result = subject.computeExactFreshArea({
    triangles: [[[0, 0], [1, 0], [0, 1]], [[0, 0], [1, 0], [0, 1]]],
    cells: [[[-1, -1], [2, -1], [2, 2], [-1, 2]]],
  });
  assert.equal(result.ok, false); assert.match(result.reason, /overlap/);
});

check('overlapping sweep cells are unioned exactly instead of double counted', () => {
  const result = subject.computeExactFreshArea({
    triangles: squareTriangles(),
    cells: [[[0, 0], [0.75, 0], [0.75, 1], [0, 1]],
      [[0.5, 0], [1, 0], [1, 1], [0.5, 1]]],
  });
  assert.equal(result.ok, true, result.reason);
  assert.equal(result.area, 1); assert.equal(result.exactAreaNumerator, '1');
  assert.equal(result.exactAreaDenominator, '1'); assert.ok(result.unionPieceCount >= 2);
});

check('triple overlaps and non-convex sweep unions retain exact area and moments', () => {
  let result = subject.computeExactFreshArea({
    triangles: squareTriangles(),
    cells: [[[0, 0], [0.75, 0], [0.75, 1], [0, 1]],
      [[0.25, 0], [1, 0], [1, 0.75], [0.25, 0.75]],
      [[0.25, 0.25], [1, 0.25], [1, 1], [0.25, 1]]],
  });
  assert.equal(result.ok, true, result.reason); assert.equal(result.area, 1);
  result = subject.computeExactFreshArea({
    triangles: squareTriangles(),
    cells: [[[0, 0], [0.5, 0], [0.5, 1], [0, 1]],
      [[0, 0], [1, 0], [1, 0.5], [0, 0.5]]],
  });
  assert.equal(result.ok, true, result.reason);
  assert.equal(result.exactAreaNumerator, '3'); assert.equal(result.exactAreaDenominator, '4');
  assert.deepEqual(result.centroid, { x: 5 / 12, y: 5 / 12 });
});

check('shared boundaries are legal and contribute zero overlap area', () => {
  const result = subject.computeExactFreshArea({
    triangles: squareTriangles(),
    cells: [[[0, 0], [0.5, 0], [0.5, 1], [0, 1]],
      [[0.5, 0], [1, 0], [1, 1], [0.5, 1]]],
  });
  assert.equal(result.ok, true, result.reason); assert.equal(result.area, 1);
});

check('collinear and duplicate polygon inputs fail closed', () => {
  let result = subject.computeExactFreshArea({
    triangles: [[[0, 0], [1, 0], [2, 0]]], cells: [[[0, 0], [1, 0], [1, 1], [0, 1]]],
  });
  assert.equal(result.ok, false); assert.match(result.reason, /degenerate/);
  result = subject.computeExactFreshArea({
    triangles: squareTriangles(), cells: [[[0, 0], [1, 0], [1, 0], [0, 1]]],
  });
  assert.equal(result.ok, false); assert.match(result.reason, /strict convex|duplicate|degenerate/);
});

check('bow-tie and concave declared cell orders cannot be convex-hull filled', () => {
  let result = subject.computeExactFreshArea({
    triangles: squareTriangles(),
    cells: [[[0, 0], [1, 1], [1, 0], [0, 1]]],
  });
  assert.equal(result.ok, false); assert.match(result.reason, /fold|intersect|declared order/);
  result = subject.computeExactFreshArea({
    triangles: squareTriangles(),
    cells: [[[0, 0], [1, 0], [0.5, 0.25], [1, 1], [0, 1]]],
  });
  assert.equal(result.ok, false); assert.match(result.reason, /concave|declared order|strict convex/);
  result = subject.computeExactFreshArea({
    triangles: [[[-20, -20], [20, -20], [20, 20]], [[-20, -20], [20, 20], [-20, 20]]],
    cells: [[[-5, 9], [-8, -9], [10, 1], [9, 6], [3, 10], [-8, 2], [5, -10], [9, -5]]],
  });
  assert.equal(result.ok, false); assert.match(result.reason, /intersecting|overlapping non-adjacent/);
});

check('determinism is byte exact and authority stays false', () => {
  const input = { triangles: squareTriangles(), cells: [[[0.2, -1], [0.8, -1], [0.8, 2], [0.2, 2]]] };
  const left = subject.computeExactFreshArea(input); const right = subject.computeExactFreshArea(input);
  assert.deepEqual(left, right); assert.match(left.authority, /^none/);
  assert.equal(Object.prototype.hasOwnProperty.call(left, 'authorityAuthorized'), false);
});

check('prepared triangle cover is identity-bound and reusable', () => {
  const cover = subject.prepareTriangleCover(squareTriangles());
  assert.equal(cover.ok, true, cover.reason); assert.equal(cover.exactArea, '1/1');
  const input = { preparedTriangleCover: cover,
    cells: [[[0.1, -1], [0.9, -1], [0.9, 2], [0.1, 2]]] };
  assert.equal(subject.computeExactFreshArea(input).area, 0.8);
  assert.equal(subject.computeExactFreshArea(input).area, 0.8);
  const clone = JSON.parse(JSON.stringify(cover));
  const rejected = subject.computeExactFreshArea({ preparedTriangleCover: clone, cells: input.cells });
  assert.equal(rejected.ok, false); assert.match(rejected.reason, /foreign|cloned|stale/);
});

process.stdout.write(`${JSON.stringify({ schema: 'exact-triangle-sweep-area-tests-v1',
  pass: true, passed: checks.length, checks }, null, 2)}\n`);
