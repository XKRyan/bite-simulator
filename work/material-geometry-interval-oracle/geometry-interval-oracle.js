'use strict';

// S4b-1A: pure geometry/numerics only.
//
// This module deliberately has no concept of a Rapier world, Sfree token,
// feature ownership, contact witness, body handle, or trusted path producer.
// It accepts an already-authorised mathematical path descriptor and proves an
// area enclosure only for a fixed-orientation segment translated by an affine
// displacement.  The B-layer authority wrapper must construct that descriptor
// from an opaque live-session capability; callers must never treat a digest
// emitted here as authentication.

const crypto = require('node:crypto');
const fs = require('node:fs');
const path = require('node:path');

const EXPECTED_POLYGON_KERNEL_SHA256 = '7F8619E84A86CE8DD400C9B50430E9C8CF8F26027B19EEB479F109F5C1A9D688';
const POLYGON_KERNEL_PATH = path.resolve(
  __dirname,
  '../../outputs/bite-simulator/assets/polygon-clipping/polygon-clipping-0.15.7.umd.min.js',
);
const RELATIVE_NUMERIC_AREA_CAP = 1e-8;

function sha256Bytes(bytes) {
  return crypto.createHash('sha256').update(bytes).digest('hex').toUpperCase();
}
function selfSha256() { return sha256Bytes(fs.readFileSync(__filename)); }
function fileSha256(filename) { return sha256Bytes(fs.readFileSync(filename)); }
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
function vector2(raw, label) { return point(raw, label); }
function clone(value) { return JSON.parse(JSON.stringify(value)); }
function deepFreeze(value) {
  if (value && typeof value === 'object' && !Object.isFrozen(value)) {
    Object.freeze(value);
    Object.values(value).forEach(deepFreeze);
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
  if (typeof value === 'object') {
    const result = {};
    Object.keys(value).sort().forEach((key) => { result[key] = canonicalValue(value[key]); });
    return result;
  }
  throw new TypeError(`unsupported canonical value ${typeof value}`);
}
function canonical(value) { return JSON.stringify(canonicalValue(value)); }
function integrityDigest(label, value) { return sha256Bytes(`${label}\n${canonical(value)}`); }

function add(left, right) { return { x: left.x + right.x, y: left.y + right.y }; }
function subtract(left, right) { return { x: left.x - right.x, y: left.y - right.y }; }
function scale(value, scalar) { return { x: value.x * scalar, y: value.y * scalar }; }
function norm(value) { return Math.hypot(value.x, value.y); }
function crossVector(left, right) { return left.x * right.y - left.y * right.x; }
function cross(origin, left, right) {
  return crossVector(subtract(left, origin), subtract(right, origin));
}

function ringSignedArea(ring) {
  let twiceArea = 0;
  for (let index = 0; index < ring.length - 1; index += 1) {
    twiceArea += ring[index][0] * ring[index + 1][1]
      - ring[index + 1][0] * ring[index][1];
  }
  return twiceArea / 2;
}
function ringPerimeter(ring) {
  let perimeter = 0;
  for (let index = 0; index < ring.length - 1; index += 1) {
    perimeter += Math.hypot(
      ring[index + 1][0] - ring[index][0],
      ring[index + 1][1] - ring[index][1],
    );
  }
  return perimeter;
}
function geometryMetrics(geometry) {
  let area = 0; let perimeter = 0; let edgeCount = 0; let ringCount = 0; let coordinateCount = 0;
  (geometry || []).forEach((polygon) => {
    polygon.forEach((ring, ringIndex) => {
      const magnitude = Math.abs(ringSignedArea(ring));
      area += ringIndex === 0 ? magnitude : -magnitude;
      perimeter += ringPerimeter(ring);
      edgeCount += ring.length - 1;
      ringCount += 1;
      coordinateCount += 2 * ring.length;
    });
  });
  return { area, perimeter, edgeCount, ringCount, coordinateCount };
}
function sameCoordinate(left, right) { return left[0] === right[0] && left[1] === right[1]; }
function normalizeMultiPolygon(raw) {
  if (!Array.isArray(raw)) throw new TypeError('remainingGeometry must be a MultiPolygon');
  return raw.map((polygon, polygonIndex) => {
    if (!Array.isArray(polygon) || !polygon.length) {
      throw new TypeError(`remainingGeometry polygon ${polygonIndex} has no outer ring`);
    }
    return polygon.map((ring, ringIndex) => {
      if (!Array.isArray(ring) || ring.length < 4) {
        throw new TypeError(`remainingGeometry ring ${polygonIndex}/${ringIndex} is too short`);
      }
      const result = ring.map((entry, pointIndex) => {
        const value = point(entry, `remainingGeometry[${polygonIndex}][${ringIndex}][${pointIndex}]`);
        return [value.x, value.y];
      });
      if (!sameCoordinate(result[0], result.at(-1))) {
        throw new TypeError(`remainingGeometry ring ${polygonIndex}/${ringIndex} is not closed`);
      }
      if (!(Math.abs(ringSignedArea(result)) > 0)) {
        throw new TypeError(`remainingGeometry ring ${polygonIndex}/${ringIndex} has zero signed area`);
      }
      for (let index = 0; index < result.length - 1; index += 1) {
        if (sameCoordinate(result[index], result[index + 1])) {
          throw new TypeError(`remainingGeometry ring ${polygonIndex}/${ringIndex} has a zero-length edge`);
        }
      }
      return result;
    });
  });
}
function geometryCoordinateScale(geometry, additionalPoints) {
  let coordinateScale = Number.MIN_VALUE;
  geometry.forEach((polygon) => polygon.forEach((ring) => ring.forEach((entry) => {
    coordinateScale = Math.max(coordinateScale, Math.abs(entry[0]), Math.abs(entry[1]));
  })));
  additionalPoints.forEach((entry) => {
    coordinateScale = Math.max(coordinateScale, Math.abs(entry.x), Math.abs(entry.y));
  });
  return coordinateScale;
}

function convexHull(points) {
  const unique = [...new Map(points.map((entry) => [`${entry.x},${entry.y}`, entry])).values()]
    .sort((left, right) => left.x - right.x || left.y - right.y);
  if (unique.length < 3) return [];
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
  lower.pop(); upper.pop();
  return lower.concat(upper);
}
function hullGeometry(points) {
  const hull = convexHull(points);
  if (hull.length < 3) return [];
  const ring = hull.map((entry) => [entry.x, entry.y]);
  ring.push(ring[0].slice());
  return [[ring]];
}

function pointSegmentDistance(value, start, end) {
  const segment = subtract(end, start); const offset = subtract(value, start);
  const denominator = segment.x ** 2 + segment.y ** 2;
  const fraction = denominator > 0
    ? Math.max(0, Math.min(1, (offset.x * segment.x + offset.y * segment.y) / denominator)) : 0;
  return norm(subtract(value, add(start, scale(segment, fraction))));
}
function segmentDistance(a, b, c, d) {
  const orientation = (p, q, r) => cross(p, q, r);
  const onSegment = (p, q, r) => q.x >= Math.min(p.x, r.x) && q.x <= Math.max(p.x, r.x)
    && q.y >= Math.min(p.y, r.y) && q.y <= Math.max(p.y, r.y);
  const o1 = orientation(a, b, c); const o2 = orientation(a, b, d);
  const o3 = orientation(c, d, a); const o4 = orientation(c, d, b);
  const properCrossing = o1 !== 0 && o2 !== 0 && o3 !== 0 && o4 !== 0
    && Math.sign(o1) !== Math.sign(o2) && Math.sign(o3) !== Math.sign(o4);
  const touching = (o1 === 0 && onSegment(a, c, b)) || (o2 === 0 && onSegment(a, d, b))
    || (o3 === 0 && onSegment(c, a, d)) || (o4 === 0 && onSegment(c, b, d));
  if (properCrossing || touching) return 0;
  return Math.min(
    pointSegmentDistance(a, c, d), pointSegmentDistance(b, c, d),
    pointSegmentDistance(c, a, b), pointSegmentDistance(d, a, b),
  );
}
function minimumGeometryFeature(geometry) {
  const edges = [];
  geometry.forEach((polygon, polygonIndex) => polygon.forEach((ring, ringIndex) => {
    const edgeCount = ring.length - 1;
    for (let edgeIndex = 0; edgeIndex < edgeCount; edgeIndex += 1) {
      edges.push({
        polygonIndex, ringIndex, edgeIndex, edgeCount,
        a: { x: ring[edgeIndex][0], y: ring[edgeIndex][1] },
        b: { x: ring[edgeIndex + 1][0], y: ring[edgeIndex + 1][1] },
      });
    }
  }));
  let minimum = Infinity;
  edges.forEach((edge) => { minimum = Math.min(minimum, norm(subtract(edge.b, edge.a))); });
  for (let leftIndex = 0; leftIndex < edges.length; leftIndex += 1) {
    const left = edges[leftIndex];
    for (let rightIndex = leftIndex + 1; rightIndex < edges.length; rightIndex += 1) {
      const right = edges[rightIndex];
      const sameRing = left.polygonIndex === right.polygonIndex && left.ringIndex === right.ringIndex;
      const adjacent = sameRing && (Math.abs(left.edgeIndex - right.edgeIndex) === 1
        || Math.abs(left.edgeIndex - right.edgeIndex) === left.edgeCount - 1);
      if (!adjacent) minimum = Math.min(minimum, segmentDistance(left.a, left.b, right.a, right.b));
    }
  }
  return minimum;
}
function domainStop(reason, extra = {}) {
  return deepFreeze({ ok: false, status: 'solver-domain-stop', reason, ...extra });
}

// A boundary tube plus shoelace roundoff enclosure.  The topology guard below
// requires every resolved feature to be far wider than lengthGuard, so each
// computed boundary remains in the same ambient-isotopy class.  For E edges,
// moving every boundary by at most lengthGuard changes area by no more than
// P*lengthGuard + E*pi*lengthGuard^2; factors 32 and 256 cover the two boolean
// operands/results and floating orientation/intersection/shoelace operations.
function numericalAreaGuard({ perimeter, edgeCount, coordinateCount, lengthGuard, scaleValue }) {
  const tube = 32 * (
    perimeter * lengthGuard + Math.PI * Math.max(1, edgeCount) * lengthGuard ** 2
  );
  const roundoff = 256 * Number.EPSILON * scaleValue ** 2
    * Math.max(1, coordinateCount + 4 * edgeCount);
  return tube + roundoff;
}
function enforceRelativeGuard(guard, referenceArea, label) {
  if (!(guard >= 0 && Number.isFinite(guard))) throw new Error(`${label} is not finite`);
  if (!(referenceArea > 0)) return;
  if (guard > RELATIVE_NUMERIC_AREA_CAP * referenceArea) {
    throw new Error(`${label} exceeds the scale-independent relative-area cap`);
  }
}

function createTranslationSweepKernel(config) {
  try {
    if (!config || typeof config !== 'object') throw new TypeError('kernel config is required');
    if (fileSha256(POLYGON_KERNEL_PATH) !== EXPECTED_POLYGON_KERNEL_SHA256) {
      throw new Error('local polygon-clipping kernel bytes do not match the pinned dependency');
    }
    const polygonKernel = require(POLYGON_KERNEL_PATH);
    const pDomain = Array.isArray(config.pDomain) && config.pDomain.length === 2
      ? config.pDomain.map((entry, index) => finite(entry, `pDomain[${index}]`)) : null;
    if (!pDomain || !(pDomain[0] >= 0 && pDomain[1] > pDomain[0])) {
      throw new Error('pDomain must be [nonnegative, larger]');
    }
    const angular = config.angularTravel;
    if (!angular || typeof angular !== 'object') {
      throw new Error('angularTravel must be explicitly supplied for the pure translation kernel');
    }
    const angularIntercept = finite(angular.intercept, 'angularTravel.intercept');
    const angularSlope = finite(angular.slope, 'angularTravel.slope');
    if (angularIntercept !== 0 || angularSlope !== 0) {
      throw new Error('pure translation kernel refuses finite or p-dependent rotation');
    }
    const segment = {
      start: point(config.workingSegment?.start, 'workingSegment.start'),
      end: point(config.workingSegment?.end, 'workingSegment.end'),
    };
    const displacement = {
      intercept: vector2(config.displacementAffine?.intercept, 'displacementAffine.intercept'),
      slope: vector2(config.displacementAffine?.slope, 'displacementAffine.slope'),
    };
    const displacementAt = (pValue) => add(displacement.intercept, scale(displacement.slope, pValue));
    const endPoints = pDomain.flatMap((pValue) => {
      const value = displacementAt(pValue);
      return [add(segment.start, value), add(segment.end, value)];
    });
    const inputGeometry = normalizeMultiPolygon(config.remainingGeometry);
    const inputMetrics = geometryMetrics(inputGeometry);
    const coordinateScale = geometryCoordinateScale(
      inputGeometry,
      [segment.start, segment.end, ...endPoints],
    );
    const operationCount = Math.max(32, inputMetrics.coordinateCount + 8 * inputMetrics.edgeCount + 32);
    const derivedLengthGuard = 256 * Number.EPSILON * coordinateScale * operationCount;
    const requestedLengthGuard = finite(config.lengthTolerance ?? 0, 'lengthTolerance');
    if (!(requestedLengthGuard >= 0)) throw new Error('lengthTolerance cannot be negative');
    const lengthGuard = Math.max(derivedLengthGuard, requestedLengthGuard);
    if (!(lengthGuard <= 1e-6 * coordinateScale)) {
      throw new Error('lengthTolerance is outside the certified relative-length domain');
    }
    const faceVector = subtract(segment.end, segment.start);
    const faceLength = norm(faceVector);
    if (!(faceLength >= 128 * lengthGuard)) throw new Error('working segment is numerically unresolved');

    const canonicalGeometry = inputGeometry.length ? polygonKernel.union(inputGeometry) : [];
    const canonicalMetrics = geometryMetrics(canonicalGeometry);
    const normalizationXor = inputGeometry.length
      ? polygonKernel.xor(inputGeometry, canonicalGeometry) : [];
    const xorMetrics = geometryMetrics(normalizationXor);
    const normalizationBoundary = {
      perimeter: inputMetrics.perimeter + canonicalMetrics.perimeter + xorMetrics.perimeter,
      edgeCount: inputMetrics.edgeCount + canonicalMetrics.edgeCount + xorMetrics.edgeCount,
      coordinateCount: inputMetrics.coordinateCount + canonicalMetrics.coordinateCount
        + xorMetrics.coordinateCount,
      lengthGuard,
      scaleValue: coordinateScale,
    };
    const geometryAreaGuard = inputGeometry.length
      ? numericalAreaGuard(normalizationBoundary) : 0;
    const geometryReferenceArea = Math.max(
      inputMetrics.area,
      canonicalMetrics.area,
      coordinateScale ** 2 * Number.EPSILON,
    );
    enforceRelativeGuard(geometryAreaGuard, geometryReferenceArea, 'geometry normalization area guard');
    if (xorMetrics.area > geometryAreaGuard
      || Math.abs(inputMetrics.area - canonicalMetrics.area) > geometryAreaGuard) {
      throw new Error('remaining MultiPolygon is not stable under canonical union/xor');
    }
    const minimumResolvedFeature = minimumGeometryFeature(canonicalGeometry);
    if (Number.isFinite(minimumResolvedFeature) && minimumResolvedFeature < 128 * lengthGuard) {
      throw new Error('remaining geometry has an unresolved thin feature or boundary crossing');
    }

    const maxDisplacement = Math.max(
      norm(displacementAt(pDomain[0])),
      norm(displacementAt(pDomain[1])),
    );
    const sweepAreaScale = faceLength * Math.max(maxDisplacement, lengthGuard);
    const numericReferenceArea = Math.max(
      canonicalMetrics.area,
      sweepAreaScale,
      coordinateScale ** 2 * Number.EPSILON,
    );
    const geometrySnapshotDigest = integrityDigest('pure-current-multipolygon-v1', {
      geometry: canonicalGeometry,
      metrics: canonicalMetrics,
      lengthGuard,
      geometryAreaGuard,
    });
    const pathDigest = integrityDigest('pure-affine-translation-path-v1', {
      pDomain, segment, displacement, angularTravel: { intercept: 0, slope: 0 },
    });

    function exactSample(pValue) {
      try {
        const p = finite(pValue, 'p');
        if (p < pDomain[0] || p > pDomain[1]) throw new Error('p is outside the frozen path domain');
        if (!canonicalGeometry.length) {
          const empty = {
            ok: true, status: 'zero-fresh-area', p, area: 0, freshArea: 0,
            areaLower: 0, areaUpper: 0, numericalAreaGuard: 0,
            payload: {
              schema: 'pure-translation-sweep-payload-v1', p,
              displacement: displacementAt(p), startSegment: clone(segment), endSegment: clone(segment),
              sweepGeometry: [], freshGeometry: [], desiredGeometry: [],
              currentArea: 0, remainingAfterArea: 0, intersectionArea: 0, differenceArea: 0,
              geometrySnapshotDigest, pathDigest,
            },
          };
          empty.sampleDigest = integrityDigest('pure-geometry-sample-v1', empty);
          return deepFreeze(empty);
        }
        const translation = displacementAt(p);
        const endSegment = {
          start: add(segment.start, translation), end: add(segment.end, translation),
        };
        const analyticSweepArea = Math.abs(crossVector(faceVector, translation));
        const sweepGeometry = hullGeometry([
          segment.start, segment.end, endSegment.start, endSegment.end,
        ]);
        const sweepMetrics = geometryMetrics(sweepGeometry);
        const preliminaryGuard = numericalAreaGuard({
          perimeter: canonicalMetrics.perimeter + sweepMetrics.perimeter,
          edgeCount: canonicalMetrics.edgeCount + sweepMetrics.edgeCount,
          coordinateCount: canonicalMetrics.coordinateCount + sweepMetrics.coordinateCount,
          lengthGuard,
          scaleValue: coordinateScale,
        });
        enforceRelativeGuard(preliminaryGuard, numericReferenceArea, 'sweep construction area guard');
        if (analyticSweepArea > 0 && analyticSweepArea <= 4 * preliminaryGuard) {
          throw new Error('positive sweep is below the certified numerical area resolution');
        }
        if (Math.abs(sweepMetrics.area - analyticSweepArea) > preliminaryGuard) {
          throw new Error('convex sweep disagrees with its analytic segment cross-product area');
        }
        const freshGeometry = sweepGeometry.length
          ? polygonKernel.intersection(canonicalGeometry, sweepGeometry) : [];
        const desiredGeometry = sweepGeometry.length
          ? polygonKernel.difference(canonicalGeometry, sweepGeometry) : clone(canonicalGeometry);
        const freshMetrics = geometryMetrics(freshGeometry);
        const desiredMetrics = geometryMetrics(desiredGeometry);
        const sampleGuard = numericalAreaGuard({
          perimeter: canonicalMetrics.perimeter + sweepMetrics.perimeter
            + freshMetrics.perimeter + desiredMetrics.perimeter,
          edgeCount: canonicalMetrics.edgeCount + sweepMetrics.edgeCount
            + freshMetrics.edgeCount + desiredMetrics.edgeCount,
          coordinateCount: canonicalMetrics.coordinateCount + sweepMetrics.coordinateCount
            + freshMetrics.coordinateCount + desiredMetrics.coordinateCount,
          lengthGuard,
          scaleValue: coordinateScale,
        });
        enforceRelativeGuard(sampleGuard, numericReferenceArea, 'sample boolean area guard');
        const differenceArea = canonicalMetrics.area - desiredMetrics.area;
        if (freshMetrics.area < -sampleGuard || differenceArea < -sampleGuard
          || Math.abs(freshMetrics.area - differenceArea) > 2 * sampleGuard) {
          throw new Error('intersection/difference/remaining area consistency exceeds the numerical enclosure');
        }
        const area = Math.max(0, freshMetrics.area);
        const areaLower = Math.max(0, area - sampleGuard);
        const areaUpper = Math.min(
          canonicalMetrics.area + geometryAreaGuard,
          area + sampleGuard,
        );
        const payload = {
          schema: 'pure-translation-sweep-payload-v1', p,
          displacement: translation,
          startSegment: clone(segment), endSegment,
          sweepGeometry, freshGeometry, desiredGeometry,
          currentArea: canonicalMetrics.area,
          remainingAfterArea: desiredMetrics.area,
          intersectionArea: freshMetrics.area,
          differenceArea,
          analyticSweepArea,
          numericalAreaGuard: sampleGuard,
          geometrySnapshotDigest, pathDigest,
        };
        payload.payloadDigest = integrityDigest('pure-translation-sweep-payload-v1', payload);
        const sample = {
          ok: true,
          status: area > sampleGuard ? 'exact-sample-with-numerical-enclosure' : 'zero-within-numerical-enclosure',
          p, area, freshArea: area, areaLower, areaUpper,
          numericalAreaGuard: sampleGuard,
          payload,
          authority: 'none-pure-mathematical-input',
        };
        sample.sampleDigest = integrityDigest('pure-geometry-sample-v1', sample);
        return deepFreeze(sample);
      } catch (error) { return domainStop(String(error?.message || error)); }
    }

    function boundFreshAreaInterval(loValue, hiValue) {
      try {
        const pLo = finite(loValue, 'pLo'); const pHi = finite(hiValue, 'pHi');
        if (!(pHi > pLo) || pLo < pDomain[0] || pHi > pDomain[1]) {
          throw new Error('interval is outside the positive-width path domain');
        }
        if (!canonicalGeometry.length) {
          return deepFreeze({
            ok: true, status: 'certified-empty-interval', pLo, pHi,
            areaLower: 0, areaUpper: 0, minArea: 0, maxArea: 0,
            geometrySnapshotDigest, pathDigest, authority: 'none-pure-mathematical-input',
          });
        }
        const pMid = pLo + (pHi - pLo) / 2;
        const left = exactSample(pLo); const middle = exactSample(pMid); const right = exactSample(pHi);
        if (!left.ok || !middle.ok || !right.ok) {
          throw new Error('an exact endpoint or midpoint is outside the numerical domain');
        }
        const halfWidth = (pHi - pLo) / 2;
        const hausdorffRadius = norm(displacement.slope) * halfWidth;
        const intervalMaxDisplacement = Math.max(
          norm(displacementAt(pLo)), norm(displacementAt(pHi)),
        );
        // S(p)=workingSegment+[0,d(p)] is convex.  With
        // delta=|dSlope|*|p-pMid| and Pmax=2*(faceLength+max|d|), the two
        // Steiner inclusions give |S(p) triangle S(mid)| <=
        // 2*Pmax*delta+2*pi*delta^2.  Intersection with fixed G contracts
        // symmetric difference.  The midpoint's numerical enclosure is then
        // added once; endpoint numerical enclosures are explicitly unioned so
        // the published interval contains every returned endpoint without an
        // unreported tolerance escape hatch.
        const perimeterUpper = 2 * (faceLength + intervalMaxDisplacement);
        const geometricAreaVariation = 2 * perimeterUpper * hausdorffRadius
          + 2 * Math.PI * hausdorffRadius ** 2;
        let areaLower = Math.max(0, middle.areaLower - geometricAreaVariation);
        let areaUpper = Math.min(
          canonicalMetrics.area + geometryAreaGuard,
          middle.areaUpper + geometricAreaVariation,
        );
        areaLower = Math.min(areaLower, left.areaLower, right.areaLower, left.area, right.area);
        areaUpper = Math.max(areaUpper, left.areaUpper, right.areaUpper, left.area, right.area);
        if (!(left.area >= areaLower && left.area <= areaUpper
          && middle.area >= areaLower && middle.area <= areaUpper
          && right.area >= areaLower && right.area <= areaUpper)) {
          throw new Error('published interval does not contain its returned exact samples');
        }
        const proof = {
          schema: 'pure-convex-translation-steiner-enclosure-v1',
          midpointSampleDigest: middle.sampleDigest,
          endpointSampleDigests: [left.sampleDigest, right.sampleDigest],
          faceLength, halfWidth, hausdorffRadius, perimeterUpper,
          geometricAreaVariation,
          midpointNumericalAreaGuard: middle.numericalAreaGuard,
          endpointNumericalAreaGuards: [left.numericalAreaGuard, right.numericalAreaGuard],
          geometryAreaGuard,
          setIdentity: 'S(p)=workingSegment Minkowski [0,d(p)]',
          contraction: '(G intersection A) triangle (G intersection B) subset A triangle B',
        };
        const result = {
          ok: true,
          status: 'pure-certified-interval',
          certificateType: 'pure-convex-translation-steiner-plus-numeric-enclosure-v1',
          pLo, pHi, areaLower, areaUpper, minArea: areaLower, maxArea: areaUpper,
          geometrySnapshotDigest, pathDigest, proof,
          authority: 'none-pure-mathematical-input',
        };
        result.intervalDigest = integrityDigest('pure-geometry-interval-v1', result);
        return deepFreeze(result);
      } catch (error) { return domainStop(String(error?.message || error)); }
    }

    const descriptor = deepFreeze({
      schema: 'pure-translation-sweep-kernel-v1',
      moduleSha256: selfSha256(),
      polygonKernelSha256: EXPECTED_POLYGON_KERNEL_SHA256,
      geometrySnapshotDigest,
      pathDigest,
      pDomain,
      segment,
      displacement,
      lengthGuard,
      geometryAreaGuard,
      relativeNumericAreaCap: RELATIVE_NUMERIC_AREA_CAP,
      supportedDomain: 'fixed-orientation line segment; affine translation; authoritative MultiPolygon supplied by upper layer',
      authority: 'none; digests are integrity checks, never authentication',
    });
    return deepFreeze({
      ok: true, status: 'pure-kernel-ready', descriptor,
      exactSample, freshArea: exactSample, boundFreshAreaInterval,
    });
  } catch (error) {
    return domainStop(String(error?.message || error), { moduleSha256: selfSha256() });
  }
}

module.exports = {
  contractVersion: 1,
  createTranslationSweepKernel,
  // Compatibility alias is deliberately math-only; no authority semantics.
  createGeometryIntervalOracle: createTranslationSweepKernel,
  selfSha256,
  EXPECTED_POLYGON_KERNEL_SHA256,
  RELATIVE_NUMERIC_AREA_CAP,
  _geometry: {
    geometryMetrics, normalizeMultiPolygon, convexHull, hullGeometry,
    minimumGeometryFeature, numericalAreaGuard,
  },
  _integrity: { canonical, digest: integrityDigest },
};
