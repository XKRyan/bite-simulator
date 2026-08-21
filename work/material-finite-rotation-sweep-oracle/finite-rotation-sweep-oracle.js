'use strict';

// Pure numerical geometry only. This module authenticates neither a Rapier
// state nor a working-face owner. Its input must later be produced by the same
// private Sfree/prepared-root composition that owns the material transaction.

const crypto = require('node:crypto');
const fs = require('node:fs');
const path = require('node:path');

const EXPECTED_POLYGON_KERNEL_SHA256 = '7F8619E84A86CE8DD400C9B50430E9C8CF8F26027B19EEB479F109F5C1A9D688';
const POLYGON_KERNEL_PATH = path.resolve(
  __dirname,
  '../../outputs/bite-simulator/assets/polygon-clipping/polygon-clipping-0.15.7.umd.min.js',
);
const RELATIVE_NUMERIC_AREA_CAP = 1e-8;
const MAXIMUM_SPATIAL_MODEL_ERROR = 5e-6;

function sha256(value) { return crypto.createHash('sha256').update(value).digest('hex').toUpperCase(); }
function selfSha256() { return sha256(fs.readFileSync(__filename)); }
function fileSha256(filename) { return sha256(fs.readFileSync(filename)); }
function finite(value, label) {
  if (!Number.isFinite(value)) throw new TypeError(`${label} must be finite`);
  return Number(value);
}
function point(raw, label) {
  if (Array.isArray(raw) && raw.length === 2) {
    return { x: finite(raw[0], `${label}[0]`), y: finite(raw[1], `${label}[1]`) };
  }
  if (!raw || typeof raw !== 'object') throw new TypeError(`${label} is required`);
  return { x: finite(raw.x, `${label}.x`), y: finite(raw.y, `${label}.y`) };
}
function clone(value) { return JSON.parse(JSON.stringify(value)); }
function deepFreeze(value) {
  if (value && typeof value === 'object' && !Object.isFrozen(value)) {
    Object.freeze(value); Object.values(value).forEach(deepFreeze);
  }
  return value;
}
function canonicalValue(value) {
  if (value === null || typeof value === 'string' || typeof value === 'boolean') return value;
  if (typeof value === 'number') {
    if (!Number.isFinite(value)) throw new TypeError('canonical values must be finite');
    return Object.is(value, -0) ? 0 : value;
  }
  if (Array.isArray(value)) return value.map(canonicalValue);
  if (value && typeof value === 'object') {
    const result = {};
    Object.keys(value).sort().forEach((key) => { result[key] = canonicalValue(value[key]); });
    return result;
  }
  throw new TypeError(`unsupported canonical value ${typeof value}`);
}
function canonical(value) { return JSON.stringify(canonicalValue(value)); }
function digest(label, value) { return sha256(`${label}\n${canonical(value)}`); }

function add(a, b) { return { x: a.x + b.x, y: a.y + b.y }; }
function subtract(a, b) { return { x: a.x - b.x, y: a.y - b.y }; }
function scale(a, amount) { return { x: a.x * amount, y: a.y * amount }; }
function norm(a) { return Math.hypot(a.x, a.y); }
function rotate(a, angle) {
  const c = Math.cos(angle); const s = Math.sin(angle);
  return { x: c * a.x - s * a.y, y: s * a.x + c * a.y };
}
function cross(origin, left, right) {
  return (left.x - origin.x) * (right.y - origin.y)
    - (left.y - origin.y) * (right.x - origin.x);
}
function sameCoordinate(a, b) { return a[0] === b[0] && a[1] === b[1]; }

function ringSignedArea(ring) {
  let twice = 0;
  for (let index = 0; index < ring.length - 1; index += 1) {
    twice += ring[index][0] * ring[index + 1][1] - ring[index + 1][0] * ring[index][1];
  }
  return twice / 2;
}
function ringPerimeter(ring) {
  let result = 0;
  for (let index = 0; index < ring.length - 1; index += 1) {
    result += Math.hypot(ring[index + 1][0] - ring[index][0], ring[index + 1][1] - ring[index][1]);
  }
  return result;
}
function geometryMetrics(geometry) {
  let area = 0; let perimeter = 0; let edgeCount = 0; let coordinateCount = 0;
  (geometry || []).forEach((polygon) => polygon.forEach((ring, ringIndex) => {
    const magnitude = Math.abs(ringSignedArea(ring));
    area += ringIndex === 0 ? magnitude : -magnitude;
    perimeter += ringPerimeter(ring); edgeCount += ring.length - 1; coordinateCount += 2 * ring.length;
  }));
  return { area, perimeter, edgeCount, coordinateCount };
}
function normalizeMultiPolygon(raw) {
  if (!Array.isArray(raw)) throw new TypeError('remainingGeometry must be a MultiPolygon');
  return raw.map((polygon, polygonIndex) => {
    if (!Array.isArray(polygon) || !polygon.length) throw new TypeError(`polygon ${polygonIndex} has no outer ring`);
    return polygon.map((ring, ringIndex) => {
      if (!Array.isArray(ring) || ring.length < 4) throw new TypeError(`ring ${polygonIndex}/${ringIndex} is too short`);
      const result = ring.map((entry, pointIndex) => {
        const value = point(entry, `remainingGeometry[${polygonIndex}][${ringIndex}][${pointIndex}]`);
        return [value.x, value.y];
      });
      if (!sameCoordinate(result[0], result.at(-1))) throw new TypeError(`ring ${polygonIndex}/${ringIndex} is open`);
      if (!(Math.abs(ringSignedArea(result)) > 0)) throw new TypeError(`ring ${polygonIndex}/${ringIndex} has zero area`);
      for (let index = 0; index < result.length - 1; index += 1) {
        if (sameCoordinate(result[index], result[index + 1])) throw new TypeError('remaining geometry has a zero edge');
      }
      return result;
    });
  });
}

function convexHullPoints(points) {
  const unique = [...new Map(points.map((entry) => [`${entry.x},${entry.y}`, entry])).values()]
    .sort((a, b) => a.x - b.x || a.y - b.y);
  if (unique.length <= 2) return unique;
  const lower = [];
  unique.forEach((entry) => {
    while (lower.length >= 2 && cross(lower.at(-2), lower.at(-1), entry) <= 0) lower.pop();
    lower.push(entry);
  });
  const upper = [];
  for (let index = unique.length - 1; index >= 0; index -= 1) {
    const entry = unique[index];
    while (upper.length >= 2 && cross(upper.at(-2), upper.at(-1), entry) <= 0) upper.pop();
    upper.push(entry);
  }
  lower.pop(); upper.pop(); return lower.concat(upper);
}
function generalizedHullPerimeter(points) {
  const hull = convexHullPoints(points);
  if (hull.length < 2) return 0;
  if (hull.length === 2) return 2 * norm(subtract(hull[1], hull[0]));
  let perimeter = 0;
  hull.forEach((entry, index) => { perimeter += norm(subtract(hull[(index + 1) % hull.length], entry)); });
  return perimeter;
}
function hullGeometry(points) {
  const hull = convexHullPoints(points);
  if (hull.length < 3) return [];
  const ring = hull.map((entry) => [entry.x, entry.y]); ring.push(ring[0].slice());
  return [[ring]];
}

function pointSegmentDistance(value, start, end) {
  const edge = subtract(end, start); const offset = subtract(value, start);
  const denominator = edge.x ** 2 + edge.y ** 2;
  const t = denominator > 0 ? Math.max(0, Math.min(1, (offset.x * edge.x + offset.y * edge.y) / denominator)) : 0;
  return norm(subtract(value, add(start, scale(edge, t))));
}
function segmentDistance(a, b, c, d) {
  const onSegment = (p, q, r) => q.x >= Math.min(p.x, r.x) && q.x <= Math.max(p.x, r.x)
    && q.y >= Math.min(p.y, r.y) && q.y <= Math.max(p.y, r.y);
  const o1 = cross(a, b, c); const o2 = cross(a, b, d); const o3 = cross(c, d, a); const o4 = cross(c, d, b);
  const proper = o1 !== 0 && o2 !== 0 && o3 !== 0 && o4 !== 0
    && Math.sign(o1) !== Math.sign(o2) && Math.sign(o3) !== Math.sign(o4);
  const touching = (o1 === 0 && onSegment(a, c, b)) || (o2 === 0 && onSegment(a, d, b))
    || (o3 === 0 && onSegment(c, a, d)) || (o4 === 0 && onSegment(c, b, d));
  if (proper || touching) return 0;
  return Math.min(pointSegmentDistance(a, c, d), pointSegmentDistance(b, c, d),
    pointSegmentDistance(c, a, b), pointSegmentDistance(d, a, b));
}
function minimumGeometryFeature(geometry) {
  const edges = [];
  geometry.forEach((polygon, polygonIndex) => polygon.forEach((ring, ringIndex) => {
    const count = ring.length - 1;
    for (let index = 0; index < count; index += 1) {
      edges.push({ polygonIndex, ringIndex, index, count,
        a: { x: ring[index][0], y: ring[index][1] },
        b: { x: ring[index + 1][0], y: ring[index + 1][1] } });
    }
  }));
  let minimum = Infinity;
  edges.forEach((edge) => { minimum = Math.min(minimum, norm(subtract(edge.b, edge.a))); });
  for (let left = 0; left < edges.length; left += 1) for (let right = left + 1; right < edges.length; right += 1) {
    const a = edges[left]; const b = edges[right];
    const sameRing = a.polygonIndex === b.polygonIndex && a.ringIndex === b.ringIndex;
    const adjacent = sameRing && (Math.abs(a.index - b.index) === 1 || Math.abs(a.index - b.index) === a.count - 1);
    if (!adjacent) minimum = Math.min(minimum, segmentDistance(a.a, a.b, b.a, b.b));
  }
  return minimum;
}
function numericalAreaGuard({ perimeter, edgeCount, coordinateCount, lengthGuard, scaleValue }) {
  return 32 * (perimeter * lengthGuard + Math.PI * Math.max(1, edgeCount) * lengthGuard ** 2)
    + 256 * Number.EPSILON * scaleValue ** 2 * Math.max(1, coordinateCount + 4 * edgeCount);
}
function enforceRelativeGuard(guard, referenceArea, label) {
  if (!(guard >= 0 && Number.isFinite(guard))) throw new Error(`${label} is not finite`);
  if (referenceArea > 0 && guard > RELATIVE_NUMERIC_AREA_CAP * referenceArea) {
    throw new Error(`${label} exceeds the relative-area cap`);
  }
}
function domainStop(reason, extra = {}) {
  return deepFreeze({ ok: false, status: 'solver-domain-stop', reason, ...extra });
}

function parseAffineVector(raw, label) {
  return { intercept: point(raw?.intercept, `${label}.intercept`), slope: point(raw?.slope, `${label}.slope`) };
}
function parseAffineScalar(raw, label) {
  return { intercept: finite(raw?.intercept, `${label}.intercept`), slope: finite(raw?.slope, `${label}.slope`) };
}
function affineVectorAt(value, p) { return add(value.intercept, scale(value.slope, p)); }
function affineScalarAt(value, p) { return value.intercept + value.slope * p; }
function parseMotion(raw, label) {
  return {
    startPosition: point(raw?.startPosition, `${label}.startPosition`),
    startAngle: finite(raw?.startAngle, `${label}.startAngle`),
    linearVelocity: parseAffineVector(raw?.linearVelocityAffine, `${label}.linearVelocityAffine`),
    angularVelocity: parseAffineScalar(raw?.angularVelocityAffine, `${label}.angularVelocityAffine`),
  };
}

function createFiniteRotationSweepKernel(config) {
  try {
    if (!config || typeof config !== 'object') throw new TypeError('config is required');
    if (fileSha256(POLYGON_KERNEL_PATH) !== EXPECTED_POLYGON_KERNEL_SHA256) throw new Error('polygon kernel hash mismatch');
    const polygonKernel = require(POLYGON_KERNEL_PATH);
    const pDomain = Array.isArray(config.pDomain) && config.pDomain.length === 2
      ? config.pDomain.map((entry, index) => finite(entry, `pDomain[${index}]`)) : null;
    if (!pDomain || !(pDomain[0] >= 0 && pDomain[1] > pDomain[0])) throw new Error('invalid pDomain');
    const h = finite(config.h, 'h'); if (!(h > 0)) throw new Error('h must be positive');
    const timeFractions = Array.isArray(config.timeFractions)
      ? config.timeFractions.map((entry, index) => finite(entry, `timeFractions[${index}]`)) : null;
    if (!timeFractions || timeFractions.length < 2 || timeFractions.length > 4097
      || timeFractions[0] !== 0 || timeFractions.at(-1) !== 1
      || timeFractions.some((entry, index) => entry < 0 || entry > 1 || (index && entry <= timeFractions[index - 1]))) {
      throw new Error('timeFractions must be a strict partition from 0 to 1');
    }
    const maximumTimeChordError = finite(config.maximumTimeChordError, 'maximumTimeChordError');
    if (!(maximumTimeChordError > 0 && maximumTimeChordError <= MAXIMUM_SPATIAL_MODEL_ERROR)) {
      throw new Error('maximumTimeChordError exceeds the fixed 5 micrometre domain');
    }
    const segment = { start: point(config.workingSegment?.start, 'workingSegment.start'),
      end: point(config.workingSegment?.end, 'workingSegment.end') };
    const feature = parseMotion(config.featureMotion, 'featureMotion');
    const material = parseMotion(config.materialMotion, 'materialMotion');
    const geometryInput = normalizeMultiPolygon(config.remainingGeometry);
    const geometryInputMetrics = geometryMetrics(geometryInput);
    const featureRadius = Math.max(norm(segment.start), norm(segment.end));
    const faceLength = norm(subtract(segment.end, segment.start));

    const positionAt = (motion, p, fraction) => add(motion.startPosition,
      scale(affineVectorAt(motion.linearVelocity, p), h * fraction));
    const angleAt = (motion, p, fraction) => motion.startAngle
      + affineScalarAt(motion.angularVelocity, p) * h * fraction;
    const relativeEndpointAt = (p, fraction, localPoint) => {
      const featurePosition = positionAt(feature, p, fraction);
      const materialPosition = positionAt(material, p, fraction);
      const worldPoint = add(featurePosition, rotate(localPoint, angleAt(feature, p, fraction)));
      return rotate(subtract(worldPoint, materialPosition), -angleAt(material, p, fraction));
    };
    const nodeEndpointsAt = (p) => timeFractions.map((fraction) => ({
      start: relativeEndpointAt(p, fraction, segment.start),
      end: relativeEndpointAt(p, fraction, segment.end),
    }));
    const cellPoints = (nodes, index) => [nodes[index].start, nodes[index].end,
      nodes[index + 1].start, nodes[index + 1].end];
    const cellGeometriesAt = (p) => {
      const nodes = nodeEndpointsAt(p); const cells = [];
      for (let index = 0; index < nodes.length - 1; index += 1) cells.push(hullGeometry(cellPoints(nodes, index)));
      return { nodes, cells };
    };

    const allDomainPoints = pDomain.flatMap((p) => nodeEndpointsAt(p).flatMap((node) => [node.start, node.end]));
    let coordinateScale = Number.MIN_VALUE;
    geometryInput.forEach((polygon) => polygon.forEach((ring) => ring.forEach((entry) => {
      coordinateScale = Math.max(coordinateScale, Math.abs(entry[0]), Math.abs(entry[1]));
    })));
    allDomainPoints.forEach((entry) => { coordinateScale = Math.max(coordinateScale, Math.abs(entry.x), Math.abs(entry.y)); });
    // A transformed coordinate uses a fixed number of arithmetic operations.
    // The number of time cells belongs in the boolean-output roundoff budget,
    // not in the perturbation assigned to every individual coordinate.
    const operationCount = Math.max(64, geometryInputMetrics.coordinateCount
      + 8 * geometryInputMetrics.edgeCount + 64);
    const requestedLengthGuard = finite(config.lengthTolerance ?? 0, 'lengthTolerance');
    if (requestedLengthGuard < 0) throw new Error('lengthTolerance cannot be negative');
    const lengthGuard = Math.max(requestedLengthGuard, 256 * Number.EPSILON * coordinateScale * operationCount);
    if (!(lengthGuard <= 1e-6 * coordinateScale)) throw new Error('lengthTolerance is outside the relative domain');
    if (!(faceLength >= 128 * lengthGuard)) throw new Error('working segment is unresolved');

    const canonicalGeometry = geometryInput.length ? polygonKernel.union(geometryInput) : [];
    const canonicalMetrics = geometryMetrics(canonicalGeometry);
    const xorGeometry = geometryInput.length ? polygonKernel.xor(geometryInput, canonicalGeometry) : [];
    const xorMetrics = geometryMetrics(xorGeometry);
    const geometryAreaGuard = geometryInput.length ? numericalAreaGuard({
      perimeter: geometryInputMetrics.perimeter + canonicalMetrics.perimeter + xorMetrics.perimeter,
      edgeCount: geometryInputMetrics.edgeCount + canonicalMetrics.edgeCount + xorMetrics.edgeCount,
      coordinateCount: geometryInputMetrics.coordinateCount + canonicalMetrics.coordinateCount + xorMetrics.coordinateCount,
      lengthGuard, scaleValue: coordinateScale,
    }) : 0;
    const geometryReferenceArea = Math.max(geometryInputMetrics.area, canonicalMetrics.area,
      coordinateScale ** 2 * Number.EPSILON);
    enforceRelativeGuard(geometryAreaGuard, geometryReferenceArea, 'geometry normalization guard');
    if (xorMetrics.area > geometryAreaGuard
      || Math.abs(geometryInputMetrics.area - canonicalMetrics.area) > geometryAreaGuard) {
      throw new Error('remaining geometry is unstable under union/xor');
    }
    const minimumFeature = minimumGeometryFeature(canonicalGeometry);
    if (Number.isFinite(minimumFeature) && minimumFeature < 128 * lengthGuard) {
      throw new Error('remaining geometry has an unresolved feature');
    }

    function endpointPDerivativeBound(fraction, local, pLo, pHi) {
      const time = h * fraction;
      const dSlope = scale(subtract(feature.linearVelocity.slope, material.linearVelocity.slope), time);
      const thetaMaterialSlope = time * material.angularVelocity.slope;
      const thetaRelativeSlope = time * (feature.angularVelocity.slope - material.angularVelocity.slope);
      const maxD = Math.max(...[pLo, pHi].map((p) => norm(subtract(
        positionAt(feature, p, fraction), positionAt(material, p, fraction)))));
      return norm(dSlope) + Math.abs(thetaMaterialSlope) * maxD
        + Math.abs(thetaRelativeSlope) * norm(local);
    }

    function proveNonFoldingCellsAt(nodes) {
      const proofs = [];
      for (let index = 0; index < timeFractions.length - 1; index += 1) {
        const ordered = [nodes[index].start, nodes[index].end,
          nodes[index + 1].end, nodes[index + 1].start];
        const hullAtSample = hullGeometry(ordered);
        const hullAreaAtSample = geometryMetrics(hullAtSample).area;
        const maximumNodeAdvance = Math.max(
          norm(subtract(ordered[3], ordered[0])), norm(subtract(ordered[2], ordered[1])));
        const cellResolutionGuard = 128 * lengthGuard
          * Math.max(faceLength, maximumNodeAdvance, 128 * lengthGuard);
        if (hullAreaAtSample <= cellResolutionGuard) {
          if (hullAtSample.length) throw new Error(`time cell ${index} sweep area is unresolved`);
          if (maximumNodeAdvance > 128 * lengthGuard) {
            throw new Error(`time cell ${index} moving collinear chord requires refinement`);
          }
          proofs.push({ index, status: 'stationary-zero-cell',
            hullAreaAtSample, maximumNodeAdvance, cellResolutionGuard });
          continue;
        }
        const cornerProofs = []; let orientationSign = 0;
        for (let corner = 0; corner < ordered.length; corner += 1) {
          const previous = ordered[(corner + ordered.length - 1) % ordered.length];
          const current = ordered[corner]; const next = ordered[(corner + 1) % ordered.length];
          const determinant = cross(current, previous, next);
          if (!(Math.abs(determinant) > cellResolutionGuard)) {
            throw new Error(`time cell ${index} convexity is unresolved at the sample`);
          }
          const sign = Math.sign(determinant);
          if (orientationSign && sign !== orientationSign) throw new Error(`time cell ${index} folds at the sample`);
          orientationSign = sign; cornerProofs.push({ corner, determinant });
        }
        proofs.push({ index, status: 'strict-convex-at-sample', hullAreaAtSample,
          maximumNodeAdvance, cellResolutionGuard, orientationSign, cornerProofs });
      }
      return proofs;
    }

    // Bound the error between each exact rigid target-local endpoint path and
    // the endpoint chord used by the declared polygonal numerical sweep.
    let timeChordError = 0;
    const pEnds = pDomain.slice();
    for (let index = 0; index < timeFractions.length - 1; index += 1) {
      const s0 = timeFractions[index]; const s1 = timeFractions[index + 1]; const duration = h * (s1 - s0);
      const maxWm = Math.max(...pEnds.map((p) => Math.abs(affineScalarAt(material.angularVelocity, p))));
      const maxWrel = Math.max(...pEnds.map((p) => Math.abs(
        affineScalarAt(feature.angularVelocity, p) - affineScalarAt(material.angularVelocity, p))));
      const maxU = Math.max(...pEnds.map((p) => norm(subtract(
        affineVectorAt(feature.linearVelocity, p), affineVectorAt(material.linearVelocity, p)))));
      const maxD = Math.max(...pEnds.flatMap((p) => [s0, s1].map((s) => norm(subtract(
        positionAt(feature, p, s), positionAt(material, p, s))))));
      const maxBodyAngle = Math.max(maxWm, maxWrel,
        ...pEnds.map((p) => Math.abs(affineScalarAt(feature.angularVelocity, p))));
      if (maxBodyAngle * duration > Math.PI) throw new Error('a time cell rotates by more than pi');
      for (const local of [segment.start, segment.end]) {
        const secondDerivativeBound = maxWm ** 2 * maxD + 2 * maxWm * maxU
          + maxWrel ** 2 * norm(local);
        timeChordError = Math.max(timeChordError, secondDerivativeBound * duration ** 2 / 8);
      }
    }
    if (!(timeChordError <= maximumTimeChordError)) throw new Error('time chord error exceeds the spatial budget');

    const geometrySnapshotSignature = digest('finite-rotation-current-geometry-v1', {
      geometry: canonicalGeometry, metrics: canonicalMetrics, geometryAreaGuard, lengthGuard,
    });
    const pathDescriptor = { pDomain, h, timeFractions, segment, feature, material,
      maximumTimeChordError, timeChordError };
    const pathSignature = digest('finite-rotation-discrete-path-v1', pathDescriptor);
    const numericReferenceArea = Math.max(canonicalMetrics.area, faceLength ** 2,
      coordinateScale ** 2 * Number.EPSILON);

    // Geometry-neutral access to the declared discrete path.  This deliberately
    // performs no polygon boolean and makes no area-resolution decision.  It is
    // used by an independently exact triangle/cell backend near zero velocity,
    // where the older floating boolean guard may conservatively stop.
    function discretePathSample(pValue) {
      try {
        const p = finite(pValue, 'p');
        if (p < pDomain[0] || p > pDomain[1]) throw new Error('p is outside the path domain');
        const nodes = nodeEndpointsAt(p);
        const result = {
          ok: true,
          status: 'declared-discrete-path-sample',
          p,
          nodes,
          timeFractions: timeFractions.slice(),
          pathSignature,
          geometrySnapshotSignature,
          timeChordError,
          authority: 'none-pure-mathematical-input',
        };
        result.pathSampleDigest = digest('finite-rotation-discrete-path-sample-v1', result);
        return deepFreeze(result);
      } catch (error) { return domainStop(String(error?.message || error)); }
    }

    function boundDiscretePathInterval(loValue, hiValue) {
      try {
        const pLo = finite(loValue, 'pLo'); const pHi = finite(hiValue, 'pHi');
        if (!(pHi > pLo) || pLo < pDomain[0] || pHi > pDomain[1]) throw new Error('invalid p interval');
        const pMid = pLo + (pHi - pLo) / 2;
        const middleNodes = nodeEndpointsAt(pMid);
        const halfWidth = (pHi - pLo) / 2;
        let geometricAreaVariation = 0; let sweepPerimeterUpper = 0; const cellProofs = [];
        for (let index = 0; index < timeFractions.length - 1; index += 1) {
          const derivativeBound = Math.max(
            endpointPDerivativeBound(timeFractions[index], segment.start, pLo, pHi),
            endpointPDerivativeBound(timeFractions[index], segment.end, pLo, pHi),
            endpointPDerivativeBound(timeFractions[index + 1], segment.start, pLo, pHi),
            endpointPDerivativeBound(timeFractions[index + 1], segment.end, pLo, pHi),
          );
          const radius = derivativeBound * halfWidth;
          const midpointPerimeter = generalizedHullPerimeter(cellPoints(middleNodes, index));
          const perimeterUpper = midpointPerimeter + 2 * Math.PI * radius;
          const variation = 2 * perimeterUpper * radius + 2 * Math.PI * radius ** 2;
          geometricAreaVariation += variation; sweepPerimeterUpper += perimeterUpper;
          cellProofs.push({ index, derivativeBound, radius, midpointPerimeter, perimeterUpper, variation });
        }
        const result = {
          ok: true,
          status: 'declared-discrete-path-interval-bound',
          pLo,
          pHi,
          halfWidth,
          geometricAreaVariation,
          sweepPerimeterUpper,
          cellProofs,
          pathSignature,
          geometrySnapshotSignature,
          timeChordError,
          authority: 'none-pure-mathematical-input',
        };
        result.pathIntervalDigest = digest('finite-rotation-discrete-path-interval-v1', result);
        return deepFreeze(result);
      } catch (error) { return domainStop(String(error?.message || error)); }
    }

    function exactSample(pValue) {
      try {
        const p = finite(pValue, 'p');
        if (p < pDomain[0] || p > pDomain[1]) throw new Error('p is outside the path domain');
        const { nodes, cells } = cellGeometriesAt(p);
        const nonFoldingCellProofs = proveNonFoldingCellsAt(nodes);
        const nonemptyCells = cells.filter((entry) => entry.length);
        const sweepGeometry = nonemptyCells.length ? polygonKernel.union(...nonemptyCells) : [];
        const sweepMetrics = geometryMetrics(sweepGeometry);
        if (sweepMetrics.area > 0) {
          const preliminary = numericalAreaGuard({
            perimeter: canonicalMetrics.perimeter + sweepMetrics.perimeter,
            edgeCount: canonicalMetrics.edgeCount + sweepMetrics.edgeCount,
            coordinateCount: canonicalMetrics.coordinateCount + sweepMetrics.coordinateCount,
            lengthGuard, scaleValue: coordinateScale,
          });
          if (sweepMetrics.area <= 4 * preliminary) throw new Error('positive sweep is below area resolution');
        }
        const freshGeometry = canonicalGeometry.length && sweepGeometry.length
          ? polygonKernel.intersection(canonicalGeometry, sweepGeometry) : [];
        const desiredGeometry = canonicalGeometry.length && sweepGeometry.length
          ? polygonKernel.difference(canonicalGeometry, sweepGeometry) : clone(canonicalGeometry);
        const freshMetrics = geometryMetrics(freshGeometry); const desiredMetrics = geometryMetrics(desiredGeometry);
        const sampleGuard = canonicalGeometry.length ? numericalAreaGuard({
          perimeter: canonicalMetrics.perimeter + sweepMetrics.perimeter
            + freshMetrics.perimeter + desiredMetrics.perimeter,
          edgeCount: canonicalMetrics.edgeCount + sweepMetrics.edgeCount
            + freshMetrics.edgeCount + desiredMetrics.edgeCount,
          coordinateCount: canonicalMetrics.coordinateCount + sweepMetrics.coordinateCount
            + freshMetrics.coordinateCount + desiredMetrics.coordinateCount,
          lengthGuard, scaleValue: coordinateScale,
        }) : 0;
        enforceRelativeGuard(sampleGuard, numericReferenceArea, 'sample area guard');
        const differenceArea = canonicalMetrics.area - desiredMetrics.area;
        if (freshMetrics.area < -sampleGuard || differenceArea < -sampleGuard
          || Math.abs(freshMetrics.area - differenceArea) > 2 * sampleGuard) {
          throw new Error('intersection/difference identity exceeds its enclosure');
        }
        const area = Math.max(0, freshMetrics.area);
        const payload = {
          schema: 'finite-rotation-polygonal-sweep-payload-v1', p, nodes,
          cellGeometries: cells, sweepGeometry, freshGeometry, desiredGeometry,
          currentArea: canonicalMetrics.area, remainingAfterArea: desiredMetrics.area,
          intersectionArea: freshMetrics.area, differenceArea,
          numericalAreaGuard: sampleGuard, timeChordError,
          nonFoldingCellProofs,
          geometrySnapshotSignature, pathSignature,
        };
        payload.payloadDigest = digest('finite-rotation-polygonal-sweep-payload-v1', payload);
        const sample = {
          ok: true, status: area > sampleGuard ? 'exact-discrete-sample' : 'zero-within-enclosure',
          p, area, freshArea: area, areaLower: Math.max(0, area - sampleGuard),
          areaUpper: Math.min(canonicalMetrics.area + geometryAreaGuard, area + sampleGuard),
          numericalAreaGuard: sampleGuard, payload, authority: 'none-pure-mathematical-input',
        };
        sample.sampleDigest = digest('finite-rotation-discrete-sample-v1', sample);
        return deepFreeze(sample);
      } catch (error) { return domainStop(String(error?.message || error)); }
    }

    function boundFreshAreaInterval(loValue, hiValue) {
      try {
        const pLo = finite(loValue, 'pLo'); const pHi = finite(hiValue, 'pHi');
        if (!(pHi > pLo) || pLo < pDomain[0] || pHi > pDomain[1]) throw new Error('invalid p interval');
        if (!canonicalGeometry.length) {
          const result = { ok: true, status: 'certified-empty-interval',
            certificateType: 'finite-rotation-exact-empty-geometry-v1',
            pLo, pHi, areaLower: 0, areaUpper: 0, minArea: 0, maxArea: 0,
            geometrySnapshotSignature, geometryLipschitzSignature: pathSignature,
            sourceSignature: pathSignature,
            pathSignature, proof: { schema: 'empty-current-geometry-proof-v1', currentArea: 0 },
            moduleSha256: selfSha256(), authority: 'none-pure-mathematical-input' };
          result.intervalDigest = digest('finite-rotation-area-interval-v1', result);
          return deepFreeze(result);
        }
        const pMid = pLo + (pHi - pLo) / 2;
        const left = exactSample(pLo); const middle = exactSample(pMid); const right = exactSample(pHi);
        if (!left.ok || !middle.ok || !right.ok) throw new Error('lo/mid/hi sample is outside the numerical domain');
        const halfWidth = (pHi - pLo) / 2;
        const middleNodes = middle.payload.nodes;
        let geometricAreaVariation = 0; let sweepPerimeterUpper = 0; const cellProofs = [];
        for (let index = 0; index < timeFractions.length - 1; index += 1) {
          const derivativeBound = Math.max(
            endpointPDerivativeBound(timeFractions[index], segment.start, pLo, pHi),
            endpointPDerivativeBound(timeFractions[index], segment.end, pLo, pHi),
            endpointPDerivativeBound(timeFractions[index + 1], segment.start, pLo, pHi),
            endpointPDerivativeBound(timeFractions[index + 1], segment.end, pLo, pHi),
          );
          const radius = derivativeBound * halfWidth;
          const midpointPerimeter = generalizedHullPerimeter(cellPoints(middleNodes, index));
          const perimeterUpper = midpointPerimeter + 2 * Math.PI * radius;
          const variation = 2 * perimeterUpper * radius + 2 * Math.PI * radius ** 2;
          geometricAreaVariation += variation; sweepPerimeterUpper += perimeterUpper;
          cellProofs.push({ index, derivativeBound, radius, midpointPerimeter, perimeterUpper, variation });
        }
        // Every result boundary is made from the original G edges and the raw
        // cell edges.  A planar arrangement of n segments has at most n^2
        // fragments; using that bound for sweep/intersection/difference makes
        // the roundoff enclosure valid for every p, including topology changes
        // between lo/mid/hi.
        const rawCellEdgeBudget = 4 * (timeFractions.length - 1);
        const combinedRawEdgeBudget = canonicalMetrics.edgeCount + rawCellEdgeBudget;
        const sweepEdgeBudget = rawCellEdgeBudget ** 2;
        const resultEdgeBudget = combinedRawEdgeBudget ** 2;
        const totalEdgeBudget = canonicalMetrics.edgeCount + sweepEdgeBudget + 2 * resultEdgeBudget;
        const intervalSampleGuard = numericalAreaGuard({
          perimeter: 3 * (canonicalMetrics.perimeter + sweepPerimeterUpper),
          edgeCount: totalEdgeBudget,
          coordinateCount: 4 * totalEdgeBudget,
          lengthGuard,
          scaleValue: coordinateScale,
        });
        enforceRelativeGuard(intervalSampleGuard, numericReferenceArea, 'interval boolean area guard');
        if (left.numericalAreaGuard > intervalSampleGuard
          || middle.numericalAreaGuard > intervalSampleGuard
          || right.numericalAreaGuard > intervalSampleGuard) {
          throw new Error('interval boolean guard does not enclose sampled guards');
        }
        let areaLower = Math.max(0, middle.areaLower - geometricAreaVariation - intervalSampleGuard);
        let areaUpper = Math.min(canonicalMetrics.area + geometryAreaGuard,
          middle.areaUpper + geometricAreaVariation + intervalSampleGuard);
        areaLower = Math.min(areaLower, left.areaLower, right.areaLower, left.area, right.area);
        areaUpper = Math.max(areaUpper, left.areaUpper, right.areaUpper, left.area, right.area);
        if (!(left.area >= areaLower && left.area <= areaUpper
          && middle.area >= areaLower && middle.area <= areaUpper
          && right.area >= areaLower && right.area <= areaUpper)) throw new Error('interval misses an exact sample');
        const proof = { schema: 'finite-rotation-cellwise-convex-enclosure-v1',
          cellProofs, halfWidth, geometricAreaVariation,
          intervalSampleGuard, rawCellEdgeBudget, totalEdgeBudget,
          midpointSampleDigest: middle.sampleDigest,
          endpointSampleDigests: [left.sampleDigest, right.sampleDigest],
          timeChordError, numericalModel: 'union of endpoint-chord convex cells' };
        const result = {
          ok: true, status: 'pure-certified-interval',
          certificateType: 'finite-rotation-cellwise-interval-steiner-plus-numeric-v1',
          pLo, pHi, areaLower, areaUpper, minArea: areaLower, maxArea: areaUpper,
          moduleSha256: selfSha256(), geometrySnapshotSignature,
          geometryLipschitzSignature: pathSignature, sourceSignature: pathSignature,
          pathSignature, proof,
          authority: 'none-pure-mathematical-input',
        };
        result.intervalDigest = digest('finite-rotation-area-interval-v1', result);
        return deepFreeze(result);
      } catch (error) { return domainStop(String(error?.message || error)); }
    }

    const descriptor = deepFreeze({
      schema: 'finite-rotation-polygonal-sweep-kernel-v1', moduleSha256: selfSha256(),
      polygonKernelSha256: EXPECTED_POLYGON_KERNEL_SHA256,
      intervalOracle: { schema: 'signed-geometry-fresh-area-interval-v1', moduleSha256: selfSha256(),
        geometrySnapshotSignature, geometryLipschitzSignature: pathSignature },
      geometrySnapshotSignature, pathSignature, pDomain, h, timeFractions, segment,
      motionFamily: { feature, material },
      timeChordError, maximumTimeChordError, lengthGuard, geometryAreaGuard,
      relativeNumericAreaCap: RELATIVE_NUMERIC_AREA_CAP,
      supportedDomain: 'affine post-impact velocities; unwrapped constant-rate body drift; polygonal endpoint-chord sweep',
      authority: 'none; integrity only',
    });
    return deepFreeze({ ok: true, status: 'pure-kernel-ready', descriptor,
      discretePathSample, boundDiscretePathInterval,
      exactSample, freshArea: exactSample, boundFreshAreaInterval });
  } catch (error) { return domainStop(String(error?.message || error), { moduleSha256: selfSha256() }); }
}

module.exports = {
  createFiniteRotationSweepKernel,
  EXPECTED_POLYGON_KERNEL_SHA256,
  MAXIMUM_SPATIAL_MODEL_ERROR,
  moduleSha256: selfSha256(),
  _reference: { generalizedHullPerimeter, numericalAreaGuard },
};
