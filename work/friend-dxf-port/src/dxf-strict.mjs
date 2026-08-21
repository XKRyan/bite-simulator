/* Work-only, dependency-free DXF geometry adapter. Coordinates stay in source units. */

export const PENETRATION_GATE_METRES = 0.00008;
export const DEFAULT_ENDPOINT_EPSILON = 1e-9;
export const DEFAULT_AREA_EPSILON = 1e-12;

export class DxfValidationError extends Error {
  constructor(code, message) { super(message); this.name = 'DxfValidationError'; this.code = code; }
}

export class GeometryContractError extends Error {
  constructor(code, message, result = null) { super(message); this.name = 'GeometryContractError'; this.code = code; this.result = result; }
}

const reject = (code, message) => { throw new DxfValidationError(code, message); };
const finite = (value) => Number.isFinite(Number(value));
const number = (value, fallback = NaN) => finite(value) ? Number(value) : fallback;
const point = (x, y) => ({ x, y });
const sub = (a, b) => point(a.x - b.x, a.y - b.y);
const add = (a, b) => point(a.x + b.x, a.y + b.y);
const scale = (p, amount) => point(p.x * amount, p.y * amount);
const cross = (a, b) => a.x * b.y - a.y * b.x;
const dot = (a, b) => a.x * b.x + a.y * b.y;
const length = (p) => Math.hypot(p.x, p.y);
const distance = (a, b) => length(sub(a, b));
const clamp = (value, low, high) => Math.max(low, Math.min(high, value));

export function pairsFromDxf(text) {
  if (typeof text !== 'string') reject('DXF_INVALID_TEXT', 'DXF must be ASCII text.');
  const lines = text.replace(/^\uFEFF/, '').split(/\r?\n/);
  if (lines.at(-1) === '') lines.pop();
  if (lines.length % 2) reject('DXF_INCOMPLETE_PAIR', 'DXF has an incomplete group-code/value pair.');
  return Array.from({ length: lines.length / 2 }, (_, index) => {
    const raw = lines[index * 2].trim();
    if (!/^[+-]?\d+$/.test(raw)) reject('DXF_INVALID_GROUP_CODE', `Invalid DXF group code at line ${index * 2 + 1}.`);
    return { code: Number(raw), value: lines[index * 2 + 1].trim() };
  });
}

export function entityChunks(text) {
  const pairs = pairsFromDxf(text); const chunks = []; let section = null;
  for (let index = 0; index < pairs.length;) {
    const pair = pairs[index]; const upper = pair.value.toUpperCase();
    if (pair.code === 0 && upper === 'SECTION') { section = pairs[index + 1]?.code === 2 ? pairs[index + 1].value.toUpperCase() : null; index += 2; continue; }
    if (pair.code === 0 && upper === 'ENDSEC') { section = null; index += 1; continue; }
    if (section === 'ENTITIES' && pair.code === 0) {
      const values = []; const type = upper; index += 1;
      while (index < pairs.length && pairs[index].code !== 0) values.push(pairs[index++]);
      chunks.push({ type, pairs: values }); continue;
    }
    index += 1;
  }
  return chunks;
}

const first = (pairs, code, fallback = NaN) => {
  const found = pairs.find((pair) => pair.code === code);
  return found ? number(found.value) : fallback;
};
const label = (entity, index) => `${entity.type} #${index + 1}`;
const nonzero = (entity, index, codes, name, errorCode) => {
  const values = entity.pairs.filter((pair) => codes.includes(pair.code));
  const malformed = values.find((pair) => !finite(pair.value));
  if (malformed) reject('DXF_INVALID_NUMBER', `${label(entity, index)} ${name} group ${malformed.code} is not finite.`);
  const invalid = values.find((pair) => Math.abs(Number(pair.value)) > 1e-12);
  if (invalid) reject(errorCode, `${label(entity, index)} ${name} group ${invalid.code} must be zero in world XY.`);
};
const validateExtrusion = (entity, index) => {
  if (!entity.pairs.some((pair) => [210, 220, 230].includes(pair.code))) return;
  const normal = [first(entity.pairs, 210, 0), first(entity.pairs, 220, 0), first(entity.pairs, 230, 1)];
  if (!normal.every(Number.isFinite)) reject('DXF_INVALID_NUMBER', `${label(entity, index)} extrusion normal is not finite.`);
  if (Math.abs(normal[0]) > 1e-12 || Math.abs(normal[1]) > 1e-12 || Math.abs(normal[2] - 1) > 1e-12) reject('DXF_UNSUPPORTED_EXTRUSION', `${label(entity, index)} requires OCS/WCS conversion; it is rejected without projection.`);
};
const validatePlanar = (entity, index) => {
  validateExtrusion(entity, index);
  if (entity.type === 'LINE') nonzero(entity, index, [30, 31], 'Z coordinate', 'DXF_NON_PLANAR_Z');
  if (entity.type === 'ARC' || entity.type === 'CIRCLE') nonzero(entity, index, [30], 'centre Z coordinate', 'DXF_NON_PLANAR_Z');
  if (entity.type === 'LWPOLYLINE') nonzero(entity, index, [38], 'elevation', 'DXF_NONZERO_ELEVATION');
  if (entity.type === 'POLYLINE' || entity.type === 'VERTEX') nonzero(entity, index, [30], 'Z/elevation', 'DXF_NON_PLANAR_Z');
  nonzero(entity, index, [39], 'thickness', 'DXF_NONZERO_THICKNESS');
  if (['LWPOLYLINE', 'POLYLINE', 'VERTEX'].includes(entity.type)) {
    nonzero(entity, index, [40, 41, 43], 'polyline width', 'DXF_NONZERO_WIDTH');
    const bulges = entity.pairs.filter((pair) => pair.code === 42);
    if (bulges.some((pair) => !finite(pair.value))) reject('DXF_INVALID_NUMBER', `${label(entity, index)} bulge is not finite.`);
    if (bulges.some((pair) => Math.abs(Number(pair.value)) > 1e-12)) reject('DXF_UNSUPPORTED_BULGE', `${label(entity, index)} has non-zero bulge; it will not be silently flattened.`);
  }
};
const requireFinite = (entity, index, values, description) => {
  if (!values.every(Number.isFinite)) reject('DXF_INVALID_NUMBER', `${label(entity, index)} lacks finite ${description}.`);
};
const xyVertices = (pairs) => {
  const output = []; let current = null;
  for (const entry of pairs) {
    if (entry.code === 10) { if (current) output.push(current); current = point(number(entry.value), NaN); }
    else if (entry.code === 20 && current && !Number.isFinite(current.y)) { current.y = number(entry.value); output.push(current); current = null; }
  }
  if (current) output.push(current); return output;
};
const rejectedTypes = new Set(['SPLINE', 'ELLIPSE', 'INSERT', 'MINSERT', 'MLINE', 'HATCH', 'SOLID', '3DFACE', 'TRACE', 'REGION', 'BODY', 'ACAD_PROXY_ENTITY', '3DSOLID', 'MESH', 'SURFACE', 'PLANESURFACE', 'REVOLVEDSURFACE', 'EXTRUDEDSURFACE', 'LOFTEDSURFACE', 'SWEPTSURFACE', 'HELIX']);
const annotations = new Set(['TEXT', 'MTEXT', 'DIMENSION', 'LEADER', 'MLEADER', 'ATTRIB', 'ATTDEF']);

export function parseStrictPlanarDxf(text) {
  if (!/\bSECTION\b/i.test(text) || !/\bENTITIES\b/i.test(text)) reject('DXF_MISSING_ENTITIES', 'ASCII DXF ENTITIES section is required.');
  const chunks = entityChunks(text); const paths = []; const unsupported = new Set(); const rejected = new Set();
  for (let index = 0; index < chunks.length; index += 1) {
    const entity = chunks[index];
    if (rejectedTypes.has(entity.type)) { rejected.add(entity.type); continue; }
    if (entity.type === 'LINE') {
      validatePlanar(entity, index); const start = point(first(entity.pairs, 10), first(entity.pairs, 20)); const end = point(first(entity.pairs, 11), first(entity.pairs, 21));
      requireFinite(entity, index, [start.x, start.y, end.x, end.y], 'LINE XY endpoints');
      if (distance(start, end) <= DEFAULT_AREA_EPSILON) reject('DXF_ZERO_LENGTH', `${label(entity, index)} is zero length.`);
      paths.push({ type: 'line', start, end }); continue;
    }
    if (entity.type === 'LWPOLYLINE') {
      validatePlanar(entity, index); const flags = first(entity.pairs, 70, 0); const declared = first(entity.pairs, 90, entity.pairs.filter((pair) => pair.code === 10).length);
      const expected = entity.pairs.filter((pair) => pair.code === 10).length;
      if (!Number.isInteger(flags) || (flags & ~(1 | 128))) reject('DXF_UNSUPPORTED_FLAGS', `${label(entity, index)} has unsupported flags.`);
      if (!Number.isInteger(declared) || declared !== expected) reject('DXF_INVALID_VERTEX_COUNT', `${label(entity, index)} vertex count does not match group 10 values.`);
      const points = xyVertices(entity.pairs); requireFinite(entity, index, points.flatMap((p) => [p.x, p.y]), 'polyline XY vertices');
      if (points.length !== expected || points.length < 2) reject('DXF_TOO_FEW_VERTICES', `${label(entity, index)} has incomplete or too few vertices.`);
      paths.push({ type: 'polyline', points, closed: Boolean(flags & 1) }); continue;
    }
    if (entity.type === 'POLYLINE') {
      validatePlanar(entity, index); const flags = first(entity.pairs, 70, 0);
      if (!Number.isInteger(flags) || (flags & ~(1 | 128))) reject('DXF_UNSUPPORTED_FLAGS', `${label(entity, index)} has unsupported flags.`);
      const points = []; let cursor = index + 1;
      while (chunks[cursor]?.type === 'VERTEX') { const vertex = chunks[cursor]; validatePlanar(vertex, cursor); if (first(vertex.pairs, 70, 0) !== 0) reject('DXF_UNSUPPORTED_FLAGS', `${label(vertex, cursor)} has unsupported flags.`); const p = point(first(vertex.pairs, 10), first(vertex.pairs, 20)); requireFinite(vertex, cursor, [p.x, p.y], 'VERTEX XY'); points.push(p); cursor += 1; }
      if (chunks[cursor]?.type !== 'SEQEND') reject('DXF_MISSING_SEQEND', `${label(entity, index)} lacks SEQEND.`);
      if (points.length < 2) reject('DXF_TOO_FEW_VERTICES', `${label(entity, index)} has too few vertices.`);
      paths.push({ type: 'polyline', points, closed: Boolean(flags & 1) }); index = cursor; continue;
    }
    if (entity.type === 'CIRCLE' || entity.type === 'ARC') {
      validatePlanar(entity, index); const center = point(first(entity.pairs, 10), first(entity.pairs, 20)); const radius = first(entity.pairs, 40);
      const angles = entity.type === 'ARC' ? [first(entity.pairs, 50), first(entity.pairs, 51)] : [];
      requireFinite(entity, index, [center.x, center.y, radius, ...angles], `${entity.type} centre, radius, and angles`);
      if (!(radius > 0)) reject('DXF_INVALID_RADIUS', `${label(entity, index)} radius must be positive.`);
      paths.push(entity.type === 'CIRCLE' ? { type: 'circle', center, radius } : { type: 'arc', center, radius, startAngle: angles[0] * Math.PI / 180, endAngle: angles[1] * Math.PI / 180 }); continue;
    }
    if (annotations.has(entity.type)) unsupported.add(entity.type);
    else if (!['SEQEND', 'EOF'].includes(entity.type)) rejected.add(entity.type === 'VERTEX' ? 'isolated VERTEX' : entity.type);
  }
  if (rejected.size) reject('DXF_UNSUPPORTED_GEOMETRY', `Unsupported geometry cannot be losslessly parsed: ${[...rejected].join(', ')}.`);
  if (!paths.length) reject('DXF_NO_SUPPORTED_GEOMETRY', 'No supported contour geometry exists.');
  return { paths, entityCount: chunks.length, ignoredAnnotations: [...unsupported] };
}

export function polygonArea(ring) { return ring.reduce((sum, a, index) => { const b = ring[(index + 1) % ring.length]; return sum + a.x * b.y - a.y * b.x; }, 0) / 2; }
export function pointInsideSimpleLoop(sample, ring) {
  let inside = false;
  for (let index = 0, previous = ring.length - 1; index < ring.length; previous = index, index += 1) { const a = ring[index]; const b = ring[previous]; if ((a.y > sample.y) !== (b.y > sample.y) && sample.x < (b.x - a.x) * (sample.y - a.y) / (b.y - a.y) + a.x) inside = !inside; }
  return inside;
}
export function pointSegmentDistance(sample, start, end) { const edge = sub(end, start); const denominator = dot(edge, edge); return distance(sample, add(start, scale(edge, denominator > 0 ? clamp(dot(sub(sample, start), edge) / denominator, 0, 1) : 0))); }
const segmentsTouch = (a, b, c, d, epsilon) => {
  const orient = (left, right, sample) => cross(sub(right, left), sub(sample, left)); const abC = orient(a, b, c); const abD = orient(a, b, d); const cdA = orient(c, d, a); const cdB = orient(c, d, b);
  if (abC * abD < 0 && cdA * cdB < 0) return true;
  return pointSegmentDistance(c, a, b) <= epsilon || pointSegmentDistance(d, a, b) <= epsilon || pointSegmentDistance(a, c, d) <= epsilon || pointSegmentDistance(b, c, d) <= epsilon;
};
export function ringSelfIntersects(ring, epsilon = DEFAULT_ENDPOINT_EPSILON) { for (let left = 0; left < ring.length; left += 1) for (let right = left + 1; right < ring.length; right += 1) { const nextLeft = (left + 1) % ring.length; const nextRight = (right + 1) % ring.length; if (left === right || nextLeft === right || nextRight === left || (left === 0 && nextRight === 0)) continue; if (segmentsTouch(ring[left], ring[nextLeft], ring[right], ring[nextRight], epsilon)) return true; } return false; }
export function ringsTouchOrIntersect(left, right, epsilon = DEFAULT_ENDPOINT_EPSILON) { return left.some((a, i) => right.some((c, j) => segmentsTouch(a, left[(i + 1) % left.length], c, right[(j + 1) % right.length], epsilon))); }
export function samplePath(path, maxChord = 0.25) {
  if (path.type === 'line') return [path.start, path.end];
  if (path.type === 'polyline') { const out = [...path.points]; return path.closed ? [...out, out[0]] : out; }
  const start = path.type === 'arc' ? path.startAngle : 0; let end = path.type === 'arc' ? path.endAngle : Math.PI * 2; while (end < start) end += Math.PI * 2; const count = Math.max(16, Math.ceil(path.radius * (end - start) / maxChord));
  return Array.from({ length: count + 1 }, (_, index) => point(path.center.x + Math.cos(start + (end - start) * index / count) * path.radius, path.center.y + Math.sin(start + (end - start) * index / count) * path.radius));
}
export function buildClosedTopology(paths, { endpointEpsilon = DEFAULT_ENDPOINT_EPSILON, maxChord = 0.25 } = {}) {
  const loops = []; const pieces = []; const openChains = []; const ambiguousJunctions = []; const degenerateEntities = [];
  paths.forEach((path, pathIndex) => { const sampled = samplePath(path, maxChord); if (sampled.length < 2) { degenerateEntities.push({ pathIndex }); return; } if (path.type === 'circle' || path.closed || distance(sampled[0], sampled.at(-1)) <= endpointEpsilon) { const ring = distance(sampled[0], sampled.at(-1)) <= endpointEpsilon ? sampled.slice(0, -1) : sampled; if (ring.length >= 3) loops.push(ring); else degenerateEntities.push({ pathIndex }); } else pieces.push({ points: sampled, pathIndex, used: false }); });
  for (const seed of pieces) { if (seed.used) continue; seed.used = true; const chain = [...seed.points]; const used = [seed.pathIndex]; let closed = false; for (let guard = 0; guard <= pieces.length + 1; guard += 1) { const end = chain.at(-1); if (distance(end, chain[0]) <= endpointEpsilon && chain.length > 3) { loops.push(chain.slice(0, -1)); closed = true; break; } const candidates = []; for (const candidate of pieces) { if (candidate.used) continue; if (distance(end, candidate.points[0]) <= endpointEpsilon) candidates.push({ candidate, reverse: false }); if (distance(end, candidate.points.at(-1)) <= endpointEpsilon) candidates.push({ candidate, reverse: true }); } if (candidates.length !== 1) { if (candidates.length > 1) ambiguousJunctions.push({ pathIndexes: [...used], candidateCount: candidates.length }); break; } const { candidate, reverse } = candidates[0]; candidate.used = true; used.push(candidate.pathIndex); chain.push(...(reverse ? [...candidate.points].reverse() : candidate.points).slice(1)); } if (!closed) openChains.push({ pathIndexes: used, start: chain[0], end: chain.at(-1), nearestGap: distance(chain[0], chain.at(-1)) }); }
  return { loops, openChains, ambiguousJunctions, degenerateEntities };
}
export function validateSingleOuterSolid(paths, options = {}) {
  const topology = buildClosedTopology(paths, options); const failures = []; const fail = (code, message) => failures.push({ code, message });
  if (topology.degenerateEntities.length) fail('DXF_DEGENERATE_ENTITY', 'Degenerate contour entity.');
  if (topology.ambiguousJunctions.length) fail('DXF_AMBIGUOUS_TOPOLOGY', 'Ambiguous chain junction.');
  if (topology.openChains.length) fail('DXF_OPEN_CONTOUR', 'Open contour; gaps are never bridged.');
  topology.loops.filter((ring) => ring.length >= 3).forEach((ring) => { if (ringSelfIntersects(ring, options.endpointEpsilon)) fail('DXF_SELF_INTERSECTION', 'Self-intersecting ring.'); });
  const rings = topology.loops.filter((ring) => ring.length >= 3 && Math.abs(polygonArea(ring)) > DEFAULT_AREA_EPSILON).sort((a, b) => Math.abs(polygonArea(b)) - Math.abs(polygonArea(a)));
  if (!rings.length) fail('DXF_NO_CLOSED_CONTOUR', 'No positive-area closed contour.');
  const outer = rings[0] || null; const holes = [];
  if (outer) for (const ring of rings.slice(1)) { if (ringsTouchOrIntersect(outer, ring, options.endpointEpsilon)) fail('DXF_INTERSECTING_RINGS', 'Outer and inner rings touch or intersect.'); else if (pointInsideSimpleLoop(ring[0], outer)) holes.push(ring); else fail('DXF_MULTIPLE_OUTERS', 'Only one independent outer ring is supported.'); }
  for (let left = 0; left < holes.length; left += 1) for (let right = left + 1; right < holes.length; right += 1) { if (ringsTouchOrIntersect(holes[left], holes[right], options.endpointEpsilon)) fail('DXF_INTERSECTING_RINGS', 'Hole rings touch or intersect.'); else if (pointInsideSimpleLoop(holes[left][0], holes[right]) || pointInsideSimpleLoop(holes[right][0], holes[left])) fail('DXF_NESTED_HOLES', 'Nested holes/islands are not supported.'); }
  if (failures.length) reject(failures[0].code, failures.map((failure) => failure.message).join(' '));
  const netArea = Math.abs(polygonArea(outer)) - holes.reduce((sum, ring) => sum + Math.abs(polygonArea(ring)), 0);
  if (!(netArea > DEFAULT_AREA_EPSILON)) reject('DXF_NON_POSITIVE_SOLID', 'Outer area minus holes must remain positive.');
  return { solid: { outer, holes, rings: [outer, ...holes], netArea }, topology };
}

const edgeKey = (a, b) => a < b ? `${a}:${b}` : `${b}:${a}`;
const samePoint = (left, right, epsilon) => distance(left, right) <= epsilon;
const onSegmentInterior = (sample, start, end, epsilon) => pointSegmentDistance(sample, start, end) <= epsilon && !samePoint(sample, start, epsilon) && !samePoint(sample, end, epsilon);
const hasProperSegmentCrossing = (a, b, c, d) => {
  const abC = cross(sub(b, a), sub(c, a)); const abD = cross(sub(b, a), sub(d, a));
  const cdA = cross(sub(d, c), sub(a, c)); const cdB = cross(sub(d, c), sub(b, c));
  return abC * abD < 0 && cdA * cdB < 0;
};
const segmentsHaveForbiddenIntersection = (a, b, c, d, epsilon) => hasProperSegmentCrossing(a, b, c, d)
  || onSegmentInterior(a, c, d, epsilon) || onSegmentInterior(b, c, d, epsilon)
  || onSegmentInterior(c, a, b, epsilon) || onSegmentInterior(d, a, b, epsilon);
const segmentDistance = (a, b, c, d, epsilon) => segmentsTouch(a, b, c, d, epsilon)
  ? 0 : Math.min(pointSegmentDistance(a, c, d), pointSegmentDistance(b, c, d), pointSegmentDistance(c, a, b), pointSegmentDistance(d, a, b));
const pointOnRing = (sample, ring, epsilon) => ring.some((start, index) => pointSegmentDistance(sample, start, ring[(index + 1) % ring.length]) <= epsilon);
const pointInCanonicalSolid = (sample, solid, epsilon, { strict = false } = {}) => {
  const onBoundary = solid.rings.some((ring) => pointOnRing(sample, ring, epsilon));
  if (onBoundary) return !strict;
  return pointInsideSimpleLoop(sample, solid.outer) && !solid.holes.some((hole) => pointInsideSimpleLoop(sample, hole));
};
const triangleCentroid = (triangle) => point((triangle[0].x + triangle[1].x + triangle[2].x) / 3, (triangle[0].y + triangle[1].y + triangle[2].y) / 3);
const pointStrictlyInTriangle = (sample, triangle, epsilon) => {
  const values = [cross(sub(triangle[1], triangle[0]), sub(sample, triangle[0])), cross(sub(triangle[2], triangle[1]), sub(sample, triangle[1])), cross(sub(triangle[0], triangle[2]), sub(sample, triangle[2]))];
  const positive = values.every((value) => value > epsilon); const negative = values.every((value) => value < -epsilon);
  return positive || negative;
};
const trianglesOverlap = (left, right, epsilon) => {
  const leftIndexes = [...left.indexes].sort((a, b) => a - b); const rightIndexes = [...right.indexes].sort((a, b) => a - b);
  if (leftIndexes.every((value, index) => value === rightIndexes[index])) return true;
  for (let leftEdge = 0; leftEdge < 3; leftEdge += 1) for (let rightEdge = 0; rightEdge < 3; rightEdge += 1) {
    const a = left.points[leftEdge]; const b = left.points[(leftEdge + 1) % 3]; const c = right.points[rightEdge]; const d = right.points[(rightEdge + 1) % 3];
    if (hasProperSegmentCrossing(a, b, c, d)) return true;
    const sameEdge = (samePoint(a, c, epsilon) && samePoint(b, d, epsilon)) || (samePoint(a, d, epsilon) && samePoint(b, c, epsilon));
    if (!sameEdge && (onSegmentInterior(a, c, d, epsilon) || onSegmentInterior(b, c, d, epsilon) || onSegmentInterior(c, a, b, epsilon) || onSegmentInterior(d, a, b, epsilon))) return true;
  }
  return left.points.some((sample) => pointStrictlyInTriangle(sample, right.points, epsilon)) || right.points.some((sample) => pointStrictlyInTriangle(sample, left.points, epsilon));
};
const numericAreaTolerance = (area) => Math.max(DEFAULT_AREA_EPSILON, Math.abs(area) * Number.EPSILON * 128);

export function assertApprovedGeometryFloor(solid, geometryFloor, { epsilon = DEFAULT_ENDPOINT_EPSILON } = {}) {
  if (!Number.isFinite(geometryFloor) || geometryFloor <= 0) reject('DXF_UNCALIBRATED_GEOMETRY_FLOOR', 'A positive, unit-matched approved geometry floor is required before triangulation.');
  const rings = solid?.rings;
  if (!Array.isArray(rings) || !rings.length) reject('DXF_INVALID_TOPOLOGY', 'Feature-floor check requires canonical solid rings.');
  const edges = rings.flatMap((ring, ringIndex) => ring.map((start, edgeIndex) => ({ ringIndex, edgeIndex, start, end: ring[(edgeIndex + 1) % ring.length] })));
  let shortestEdge = Infinity; let nearestNonAdjacentBoundary = Infinity;
  edges.forEach((edge) => { shortestEdge = Math.min(shortestEdge, distance(edge.start, edge.end)); });
  for (let left = 0; left < edges.length; left += 1) for (let right = left + 1; right < edges.length; right += 1) {
    const a = edges[left]; const b = edges[right];
    const sameRing = a.ringIndex === b.ringIndex; const ringLength = rings[a.ringIndex].length;
    const adjacent = sameRing && (Math.abs(a.edgeIndex - b.edgeIndex) === 1 || Math.abs(a.edgeIndex - b.edgeIndex) === ringLength - 1);
    if (!adjacent) nearestNonAdjacentBoundary = Math.min(nearestNonAdjacentBoundary, segmentDistance(a.start, a.end, b.start, b.end, epsilon));
  }
  const minimumFeature = Math.min(shortestEdge, nearestNonAdjacentBoundary);
  if (!Number.isFinite(minimumFeature) || minimumFeature < geometryFloor) reject('DXF_FEATURE_BELOW_GEOMETRY_FLOOR', `Minimum canonical feature ${minimumFeature} is below approved geometry floor ${geometryFloor}.`);
  return { geometryFloor, shortestEdge, nearestNonAdjacentBoundary, minimumFeature };
}

export function verifySolidTriangulation({ solid, points, indices, boundaryEdgeKeys, epsilon = DEFAULT_ENDPOINT_EPSILON, areaTolerance = numericAreaTolerance(solid.netArea) }) {
  if (!Number.isFinite(areaTolerance) || areaTolerance < 0) reject('DXF_INVALID_TRIANGULATION_TOLERANCE', 'Triangulation area tolerance must be finite and non-negative.');
  const canonicalArea = Math.abs(polygonArea(solid.outer)) - solid.holes.reduce((sum, hole) => sum + Math.abs(polygonArea(hole)), 0);
  if (!(canonicalArea > DEFAULT_AREA_EPSILON) || Math.abs(canonicalArea - solid.netArea) > numericAreaTolerance(canonicalArea)) reject('DXF_INVALID_SOLID_AREA', 'Canonical rings and supplied net area disagree.');
  const edgeUses = new Map(); const triangles = []; let triangulatedArea = 0;
  for (let offset = 0; offset < indices.length; offset += 3) {
    const triangleIndexes = [indices[offset], indices[offset + 1], indices[offset + 2]];
    if (new Set(triangleIndexes).size !== 3) reject('DXF_DEGENERATE_TRIANGLE', `Triangle ${offset / 3} repeats a vertex.`);
    const trianglePoints = triangleIndexes.map((index) => points[index]); const area = Math.abs(polygonArea(trianglePoints));
    if (!(area > numericAreaTolerance(canonicalArea))) reject('DXF_DEGENERATE_TRIANGLE', `Triangle ${offset / 3} has no auditable area.`);
    for (const hole of solid.holes) if (hole.some((vertex) => pointStrictlyInTriangle(vertex, trianglePoints, epsilon))) reject('DXF_TRIANGLE_HOLE_COVERAGE', `Triangle ${offset / 3} covers a hole interior.`);
    const centroid = triangleCentroid(trianglePoints);
    if (solid.holes.some((hole) => pointInsideSimpleLoop(centroid, hole) && !pointOnRing(centroid, hole, epsilon))) reject('DXF_TRIANGLE_HOLE_COVERAGE', `Triangle ${offset / 3} occupies a hole interior.`);
    if (!pointInCanonicalSolid(centroid, solid, epsilon, { strict: true })) reject('DXF_TRIANGLE_OUTSIDE_SOLID', `Triangle ${offset / 3} centroid is outside canonical solid.`);
    for (let edgeIndex = 0; edgeIndex < 3; edgeIndex += 1) {
      const startIndex = triangleIndexes[edgeIndex]; const endIndex = triangleIndexes[(edgeIndex + 1) % 3]; const start = trianglePoints[edgeIndex]; const end = trianglePoints[(edgeIndex + 1) % 3]; const key = edgeKey(startIndex, endIndex);
      if (!pointInCanonicalSolid(point((start.x + end.x) / 2, (start.y + end.y) / 2), solid, epsilon)) reject('DXF_TRIANGLE_OUTSIDE_SOLID', `Triangle ${offset / 3} has an edge outside canonical solid.`);
      for (const ring of solid.rings) for (let boundaryIndex = 0; boundaryIndex < ring.length; boundaryIndex += 1) {
        const boundaryStart = ring[boundaryIndex]; const boundaryEnd = ring[(boundaryIndex + 1) % ring.length];
        const isOwnBoundary = boundaryEdgeKeys.has(key) && ((samePoint(start, boundaryStart, epsilon) && samePoint(end, boundaryEnd, epsilon)) || (samePoint(start, boundaryEnd, epsilon) && samePoint(end, boundaryStart, epsilon)));
        if (!isOwnBoundary && segmentsHaveForbiddenIntersection(start, end, boundaryStart, boundaryEnd, epsilon)) reject('DXF_TRIANGLE_CROSSES_BOUNDARY', `Triangle ${offset / 3} crosses a canonical boundary.`);
      }
      const uses = edgeUses.get(key) || []; uses.push(offset / 3); edgeUses.set(key, uses);
    }
    triangles.push({ indexes: triangleIndexes, points: trianglePoints }); triangulatedArea += area;
  }
  for (let left = 0; left < triangles.length; left += 1) for (let right = left + 1; right < triangles.length; right += 1) if (trianglesOverlap(triangles[left], triangles[right], epsilon)) reject('DXF_TRIANGLE_OVERLAP', `Triangles ${left} and ${right} overlap or form a non-manifold T-junction.`);
  for (const boundaryKey of boundaryEdgeKeys) { const uses = edgeUses.get(boundaryKey) || []; if (uses.length === 0) reject('DXF_TRIANGULATION_BOUNDARY_GAP', `Canonical boundary edge ${boundaryKey} is not represented by a triangle.`); if (uses.length !== 1) reject('DXF_TRIANGULATION_BOUNDARY_DUPLICATE', `Canonical boundary edge ${boundaryKey} is represented ${uses.length} times.`); }
  for (const [key, uses] of edgeUses) if (!boundaryEdgeKeys.has(key) && uses.length !== 2) reject('DXF_TRIANGULATION_INTERNAL_GAP', `Internal mesh edge ${key} is used ${uses.length} times instead of twice.`);
  if (Math.abs(triangulatedArea - canonicalArea) > areaTolerance) reject('DXF_TRIANGULATION_AREA_MISMATCH', `Triangle area ${triangulatedArea} differs from canonical solid area ${canonicalArea}.`);
  return { triangulatedArea, canonicalArea, areaTolerance, triangleCount: triangles.length, proof: 'contained + non-overlap + exact boundary/internal edge multiplicity + area equality' };
}

export function prepareSolidTriangulation(solid, triangulate, { geometryFloor, epsilon = DEFAULT_ENDPOINT_EPSILON, areaTolerance } = {}) {
  if (typeof triangulate !== 'function') throw new TypeError('A local Earcut-compatible triangulate function is required.');
  const rings = solid?.rings; if (!Array.isArray(rings) || !rings[0] || rings.some((ring) => ring.length < 3)) reject('DXF_INVALID_TOPOLOGY', 'Triangulation requires one outer ring and rings of at least three points.');
  const featureReport = assertApprovedGeometryFloor(solid, geometryFloor, { epsilon });
  const points = rings.flat(); const starts = []; let offset = 0; for (const ring of rings) { starts.push(offset); offset += ring.length; }
  const holes = starts.slice(1); const flat = points.flatMap((p) => [p.x, p.y]); const indices = [...triangulate(flat, holes, 2)];
  if (!indices.length || indices.length % 3) reject('DXF_TRIANGULATION_FAILED', 'Triangulator returned no complete triangles.');
  if (indices.some((index) => !Number.isInteger(index) || index < 0 || index >= points.length)) reject('DXF_TRIANGULATION_FAILED', 'Triangulator returned an invalid vertex index.');
  const boundaryEdgeKeys = new Set(); rings.forEach((ring, ringIndex) => ring.forEach((_, index) => boundaryEdgeKeys.add(edgeKey(starts[ringIndex] + index, starts[ringIndex] + (index + 1) % ring.length))));
  const quality = verifySolidTriangulation({ solid, points, indices, boundaryEdgeKeys, epsilon, ...(areaTolerance === undefined ? {} : { areaTolerance }) });
  return { rings, points, indices, holeIndexes: holes, boundaryEdgeKeys, triangulatedArea: quality.triangulatedArea, netArea: quality.canonicalArea, featureReport, quality };
}
export function classifyTriangleEdge(prepared, triangleIndex, edgeIndex) { const start = prepared.indices[triangleIndex * 3 + edgeIndex]; const end = prepared.indices[triangleIndex * 3 + (edgeIndex + 1) % 3]; const key = edgeKey(start, end); return { vertexIndexes: [start, end], boundary: prepared.boundaryEdgeKeys.has(key), internalTriangleEdge: !prepared.boundaryEdgeKeys.has(key) }; }

export function assertNoInitialOverlap(intersectionArea, { kernelAreaTolerance = 0 } = {}) { if (!Number.isFinite(intersectionArea) || intersectionArea < 0) reject('DXF_INVALID_OVERLAP_AREA', 'Initial-overlap adapter must return a finite non-negative area.'); if (!Number.isFinite(kernelAreaTolerance) || kernelAreaTolerance < 0) reject('DXF_INVALID_OVERLAP_TOLERANCE', 'Kernel overlap tolerance must be externally supplied as a finite non-negative area.'); if (intersectionArea > kernelAreaTolerance) reject('DXF_INITIAL_SOLID_OVERLAP', `Initial solid overlap ${intersectionArea} exceeds externally bound kernel tolerance ${kernelAreaTolerance}.`); return { intersectionArea, kernelAreaTolerance }; }
const rotate = (p, angle) => point(Math.cos(angle) * p.x - Math.sin(angle) * p.y, Math.sin(angle) * p.x + Math.cos(angle) * p.y);
const transformed = (rings, pose, targetPose) => rings.map((ring) => ring.map((p) => rotate(sub(add(pose.position, rotate(p, pose.angle)), targetPose.position), -targetPose.angle)));
export function exactSolidRectanglePenetration(localRings, movingPose, targetPose, halfExtents) {
  if (!localRings?.[0]?.length) return 0; const rings = transformed(localRings, movingPose, targetPose); const halfX = halfExtents.x; const halfY = halfExtents.y; let deepest = 0;
  for (const ring of rings) for (let index = 0; index < ring.length; index += 1) { const a = ring[index]; const b = ring[(index + 1) % ring.length]; const delta = sub(b, a); let enter = 0; let leave = 1; const clip = (origin, direction, low, high) => { if (Math.abs(direction) <= 1e-16) return origin >= low && origin <= high; let firstHit = (low - origin) / direction; let lastHit = (high - origin) / direction; if (firstHit > lastHit) [firstHit, lastHit] = [lastHit, firstHit]; enter = Math.max(enter, firstHit); leave = Math.min(leave, lastHit); return enter <= leave; }; if (!clip(a.x, delta.x, -halfX, halfX) || !clip(a.y, delta.y, -halfY, halfY)) continue; const candidates = [clamp(enter, 0, 1), clamp(leave, 0, 1)]; const faces = [[halfX - a.x, -delta.x], [halfX + a.x, delta.x], [halfY - a.y, -delta.y], [halfY + a.y, delta.y]]; for (let left = 0; left < faces.length; left += 1) for (let right = left + 1; right < faces.length; right += 1) { const denominator = faces[left][1] - faces[right][1]; if (Math.abs(denominator) > 1e-16) { const fraction = (faces[right][0] - faces[left][0]) / denominator; if (fraction >= enter && fraction <= leave) candidates.push(fraction); } } for (const fraction of candidates) deepest = Math.max(deepest, Math.min(...faces.map(([constant, slope]) => constant + slope * fraction))); }
  for (const corner of [point(-halfX, -halfY), point(halfX, -halfY), point(halfX, halfY), point(-halfX, halfY)]) if (pointInsideSimpleLoop(corner, rings[0]) && !rings.slice(1).some((hole) => pointInsideSimpleLoop(corner, hole))) { const nearest = Math.min(...rings.flatMap((ring) => ring.map((p, index) => pointSegmentDistance(corner, p, ring[(index + 1) % ring.length])))); deepest = Math.max(deepest, nearest); }
  return Math.max(0, deepest);
}
const isPlainOptionsObject = (value) => {
  if (value === null || typeof value !== 'object' || Array.isArray(value)) return false;
  const prototype = Object.getPrototypeOf(value);
  return prototype === Object.prototype || prototype === null;
};
const ownDataOption = (value, key) => {
  const descriptor = Object.getOwnPropertyDescriptor(value, key);
  if (!descriptor) return { present: false, value: undefined };
  if (!Object.hasOwn(descriptor, 'value')) return { error: `${key} must be a data property, not an accessor.` };
  return { present: true, value: descriptor.value };
};
const matcherOptions = (value) => {
  if (typeof value === 'number') return { tolerance: value, failClosed: true };
  if (!isPlainOptionsObject(value)) return { error: 'Options must be a number tolerance or a plain object.' };
  const failClosedOption = ownDataOption(value, 'failClosed'); const toleranceOption = ownDataOption(value, 'tolerance');
  if (failClosedOption.error || toleranceOption.error) return { error: failClosedOption.error || toleranceOption.error };
  if (failClosedOption.present && failClosedOption.value !== undefined && typeof failClosedOption.value !== 'boolean') return { error: 'failClosed must be boolean when supplied.' };
  // Soft return is an opt-out, never a truthy/falsy coercion. Inherited fields
  // are deliberately ignored, so a prototype cannot disable the hard gate.
  return { tolerance: toleranceOption.present ? toleranceOption.value : DEFAULT_ENDPOINT_EPSILON, failClosed: !(failClosedOption.present && failClosedOption.value === false) };
};
const contactFailure = (code, message, detail, failClosed) => { const result = { ok: false, code, message, detail, geometricToSolver: new Map(), unmatchedGeometricContacts: detail?.unmatchedGeometricContacts || [], unmatchedSolverContacts: detail?.unmatchedSolverContacts || [] }; if (failClosed !== false) throw new GeometryContractError(code, message, result); return result; };
export function matchGeometricContactsToSolver(rawGeometry, rawDistances, solverPoints, options = {}) {
  const normalized = matcherOptions(options);
  if (normalized.error) return contactFailure('GEOMETRY_SOLVER_CONTACT_INVALID_OPTIONS', normalized.error, { receivedType: options === null ? 'null' : typeof options }, true);
  const { tolerance, failClosed } = normalized;
  if (!Number.isFinite(tolerance) || tolerance < 0) return contactFailure('GEOMETRY_SOLVER_CONTACT_INVALID', 'Contact matching tolerance must be finite and non-negative.', {}, failClosed);
  if (!Array.isArray(rawGeometry) || !Array.isArray(rawDistances) || !Array.isArray(solverPoints) || rawGeometry.length !== rawDistances.length) return contactFailure('GEOMETRY_SOLVER_CONTACT_INVALID', 'Geometric contacts, distances, and solver contacts must have consistent arrays.', {}, failClosed);
  const invalidGeometry = rawGeometry.findIndex((entry, index) => !Number.isFinite(rawDistances[index]) || !Number.isFinite(entry?.midpoint?.x) || !Number.isFinite(entry?.midpoint?.y));
  const invalidSolver = solverPoints.findIndex((entry) => !Number.isFinite(entry?.distance) || !Number.isFinite(entry?.point?.x) || !Number.isFinite(entry?.point?.y));
  if (invalidGeometry >= 0 || invalidSolver >= 0) return contactFailure('GEOMETRY_SOLVER_CONTACT_INVALID', 'Contact mapping received a non-finite witness.', { invalidGeometry, invalidSolver }, failClosed);
  const byGeometry = rawGeometry.map(() => []); const bySolver = solverPoints.map(() => []);
  rawGeometry.forEach((raw, geometricIndex) => solverPoints.forEach((solver, solverIndex) => { const distanceError = Math.abs(rawDistances[geometricIndex] - solver.distance); const pointError = distance(raw.midpoint, solver.point); if (distanceError <= tolerance && pointError <= tolerance) { const candidate = { geometricIndex, solverIndex, score: distanceError + pointError }; byGeometry[geometricIndex].push(candidate); bySolver[solverIndex].push(candidate); } }));
  const unmatchedGeometricContacts = byGeometry.map((entries, index) => entries.length === 0 ? index : null).filter(Number.isInteger); const unmatchedSolverContacts = bySolver.map((entries, index) => entries.length === 0 ? index : null).filter(Number.isInteger);
  if (unmatchedGeometricContacts.length || unmatchedSolverContacts.length || rawGeometry.length !== solverPoints.length) return contactFailure('GEOMETRY_SOLVER_CONTACT_UNMATCHED', 'Every geometric and solver contact must have exactly one witness match.', { unmatchedGeometricContacts, unmatchedSolverContacts, geometricCount: rawGeometry.length, solverCount: solverPoints.length }, failClosed);
  const ambiguousGeometricContacts = byGeometry.map((entries, index) => entries.length > 1 ? index : null).filter(Number.isInteger); const ambiguousSolverContacts = bySolver.map((entries, index) => entries.length > 1 ? index : null).filter(Number.isInteger);
  if (ambiguousGeometricContacts.length || ambiguousSolverContacts.length) return contactFailure('GEOMETRY_SOLVER_CONTACT_AMBIGUOUS', 'Duplicate or ambiguous geometric/solver contact witnesses are rejected.', { ambiguousGeometricContacts, ambiguousSolverContacts }, failClosed);
  const geometricToSolver = new Map(byGeometry.map((entries, geometricIndex) => [geometricIndex, entries[0].solverIndex]));
  return { ok: true, geometricToSolver, unmatchedGeometricContacts: [], unmatchedSolverContacts: [], tolerance };
}
