import assert from 'node:assert/strict';
import test from 'node:test';

import { DXF_FIXTURES, DXF_FIXTURES_BY_ID } from './dxf-fixtures.mjs';

function parsePairs(text) {
  const lines = text.replace(/^\uFEFF/, '').split(/\r?\n/);
  if (lines.at(-1) === '') lines.pop();
  assert.equal(lines.length % 2, 0, 'DXF must contain complete group-code/value pairs');
  return Array.from({ length: lines.length / 2 }, (_, index) => ({
    code: Number.parseInt(lines[index * 2].trim(), 10),
    value: lines[index * 2 + 1].trim(),
  }));
}

function entityChunks(text) {
  const source = parsePairs(text);
  const entities = [];
  let section = null;
  let index = 0;
  while (index < source.length) {
    const pair = source[index];
    if (pair.code === 0 && pair.value === 'SECTION') {
      section = source[index + 1]?.code === 2 ? source[index + 1].value : null;
      index += 2;
      continue;
    }
    if (pair.code === 0 && pair.value === 'ENDSEC') {
      section = null;
      index += 1;
      continue;
    }
    if (section === 'ENTITIES' && pair.code === 0) {
      const chunk = { type: pair.value, pairs: [] };
      index += 1;
      while (index < source.length && source[index].code !== 0) chunk.pairs.push(source[index++]);
      entities.push(chunk);
      continue;
    }
    index += 1;
  }
  return entities;
}

function valuesFor(entity, code) {
  return entity.pairs.filter((pair) => pair.code === code).map((pair) => Number(pair.value));
}

function firstValue(entity, code) {
  return valuesFor(entity, code)[0];
}

function entityCounts(entities) {
  return Object.fromEntries([...new Set(entities.map((entity) => entity.type))]
    .map((type) => [type, entities.filter((entity) => entity.type === type).length]));
}

function polylineVertices(entity) {
  const vertices = [];
  let current = null;
  entity.pairs.forEach((pair) => {
    if (pair.code === 10) {
      current = [Number(pair.value), NaN];
      vertices.push(current);
    } else if (pair.code === 20 && current && !Number.isFinite(current[1])) {
      current[1] = Number(pair.value);
    }
  });
  assert(vertices.every(([x, y]) => Number.isFinite(x) && Number.isFinite(y)));
  return vertices;
}

function signedArea(vertices) {
  return vertices.reduce((sum, point, index) => {
    const next = vertices[(index + 1) % vertices.length];
    return sum + point[0] * next[1] - point[1] * next[0];
  }, 0) / 2;
}

function pointInside(point, vertices) {
  let inside = false;
  for (let index = 0, previous = vertices.length - 1; index < vertices.length; previous = index, index += 1) {
    const a = vertices[index];
    const b = vertices[previous];
    if ((a[1] > point[1]) !== (b[1] > point[1])
      && point[0] < (b[0] - a[0]) * (point[1] - a[1]) / (b[1] - a[1]) + a[0]) inside = !inside;
  }
  return inside;
}

function orientation(a, b, c) {
  return Math.sign((b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0]));
}

function properIntersection(a, b, c, d) {
  return orientation(a, b, c) * orientation(a, b, d) < 0
    && orientation(c, d, a) * orientation(c, d, b) < 0;
}

function fixture(id) {
  const result = DXF_FIXTURES_BY_ID[id];
  assert(result, `Missing fixture: ${id}`);
  return result;
}

test('fixture catalogue is unique, ASCII DXF, and has complete expectations', () => {
  const expectedIds = [
    'planar-closed-triangle',
    'line-3d',
    'open-gap-0.2mm',
    'nonzero-elevation',
    'nonzero-thickness',
    'nonzero-width',
    'nonzero-bulge',
    'tilted-normal',
    'negative-z-normal',
    'malformed-extrusion-number',
    'malformed-bulge-number',
    'zero-length-line',
    'outer-with-hole',
    'two-independent-outers',
    'self-intersecting',
  ];
  assert.deepEqual(DXF_FIXTURES.map(({ id }) => id), expectedIds);
  assert.equal(new Set(expectedIds).size, expectedIds.length);

  DXF_FIXTURES.forEach((item) => {
    assert(item.description.length > 0);
    assert([...item.text].every((character) => character.charCodeAt(0) <= 0x7f), `${item.id} must remain ASCII`);
    assert.equal(typeof item.expected.accepted, 'boolean');
    if (!item.expected.accepted) assert.match(item.expected.errorCode, /^DXF_[A-Z0-9_]+$/);

    const parsed = parsePairs(item.text);
    assert(parsed.some((pair, index) => pair.code === 9 && pair.value === '$INSUNITS'
      && parsed[index + 1]?.code === 70 && parsed[index + 1]?.value === '4'));
    assert.deepEqual(entityCounts(entityChunks(item.text)), item.expected.entityCounts);
    assert.deepEqual(parsed.slice(-2), [{ code: 0, value: 'ENDSEC' }, { code: 0, value: 'EOF' }]);
  });
});

test('planar and 3D LINE fixtures isolate the Z-plane decision', () => {
  const planarEntities = entityChunks(fixture('planar-closed-triangle').text);
  const elevatedEntities = entityChunks(fixture('line-3d').text);

  assert(planarEntities.flatMap((entity) => [...valuesFor(entity, 30), ...valuesFor(entity, 31)])
    .every((value) => value === 0));
  assert(elevatedEntities.flatMap((entity) => [...valuesFor(entity, 30), ...valuesFor(entity, 31)])
    .every((value) => value === 2));

  const xy = (entity) => [firstValue(entity, 10), firstValue(entity, 20), firstValue(entity, 11), firstValue(entity, 21)];
  assert.deepEqual(elevatedEntities.map(xy), planarEntities.map(xy), '3D fixture must retain the same closed XY projection');
});

test('open contour fixture has a literal 0.2 mm endpoint gap', () => {
  const lines = entityChunks(fixture('open-gap-0.2mm').text);
  const firstStart = [firstValue(lines[0], 10), firstValue(lines[0], 20)];
  const finalEnd = [firstValue(lines.at(-1), 11), firstValue(lines.at(-1), 21)];
  const gap = Math.hypot(finalEnd[0] - firstStart[0], finalEnd[1] - firstStart[1]);
  assert.equal(gap, fixture('open-gap-0.2mm').expected.topology.gapMm);
  assert(gap < 0.35, 'fixture must reproduce the former 0.35 mm silent-close path');
});

test('elevation, thickness, width, and bulge fixtures carry their declared non-zero group', () => {
  const cases = [
    ['nonzero-elevation', 38, 1.5],
    ['nonzero-thickness', 39, 0.5],
    ['nonzero-width', 43, 0.8],
    ['nonzero-bulge', 42, 0.25],
  ];
  cases.forEach(([id, code, expectedValue]) => {
    const actual = entityChunks(fixture(id).text).flatMap((entity) => valuesFor(entity, code));
    assert.deepEqual(actual, [expectedValue], `${id} group ${code}`);
    assert.deepEqual(fixture(id).expected.offendingGroupCodes, [code]);
    assert.deepEqual(fixture(id).expected.offendingValues, [expectedValue]);
  });
});

test('extrusion fixtures encode tilted and negative-Z normals exactly', () => {
  ['tilted-normal', 'negative-z-normal'].forEach((id) => {
    const [entity] = entityChunks(fixture(id).text);
    const normal = [firstValue(entity, 210), firstValue(entity, 220), firstValue(entity, 230)];
    assert.deepEqual(normal, fixture(id).expected.extrusionNormal);
    assert.notDeepEqual(normal, [0, 0, 1]);
    assert(Math.abs(Math.hypot(...normal) - 1) < 1e-12, `${id} normal must be unit length`);
  });
});

test('outer-with-hole fixture has one contained hole and the declared net area', () => {
  const polylines = entityChunks(fixture('outer-with-hole').text);
  assert(polylines.every((entity) => (firstValue(entity, 70) & 1) === 1));
  const [outer, hole] = polylines.map(polylineVertices);
  const outerArea = Math.abs(signedArea(outer));
  const holeArea = Math.abs(signedArea(hole));
  const expected = fixture('outer-with-hole').expected.topology;

  assert.equal(outerArea, expected.outerAreaMm2);
  assert.equal(holeArea, expected.holeAreaMm2);
  assert.equal(outerArea - holeArea, expected.netAreaMm2);
  assert(pointInside(hole[0], outer));
  assert.equal(Math.sign(signedArea(outer)), Math.sign(signedArea(hole)), 'fixture intentionally uses equal winding');
});

test('two-independent-outers fixture contains two disjoint top-level rings', () => {
  const rings = entityChunks(fixture('two-independent-outers').text).map(polylineVertices);
  assert.equal(rings.length, 2);
  assert(!pointInside(rings[0][0], rings[1]));
  assert(!pointInside(rings[1][0], rings[0]));
  const bounds = rings.map((ring) => ({
    minX: Math.min(...ring.map(([x]) => x)),
    maxX: Math.max(...ring.map(([x]) => x)),
  }));
  assert(bounds[0].maxX < bounds[1].minX);
});

test('self-intersecting fixture is a bow-tie with one proper crossing', () => {
  const [polyline] = entityChunks(fixture('self-intersecting').text);
  const vertices = polylineVertices(polyline);
  const intersections = [];
  for (let left = 0; left < vertices.length; left += 1) {
    const leftNext = (left + 1) % vertices.length;
    for (let right = left + 1; right < vertices.length; right += 1) {
      const rightNext = (right + 1) % vertices.length;
      if (left === right || leftNext === right || rightNext === left) continue;
      if (properIntersection(vertices[left], vertices[leftNext], vertices[right], vertices[rightNext])) {
        intersections.push([left, right]);
      }
    }
  }
  assert.deepEqual(intersections, [[0, 2]]);
  assert.equal(intersections.length, fixture('self-intersecting').expected.topology.selfIntersections);
});
