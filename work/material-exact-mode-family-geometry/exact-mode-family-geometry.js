'use strict';

// Work-only composition of the signed Coulomb mode family, the finite-rotation
// discrete path, and exact dyadic triangle/cell area accumulation.
// No Sfree/path/owner authority is created here.

const crypto = require('node:crypto');
const fs = require('node:fs');
const path = require('node:path');
const modeFamilyModule = require('../material-kkt-mode-family-geometry/mode-family-geometry.js');
const exactAreaModule = require('../material-exact-triangle-sweep-area/exact-triangle-sweep-area.js');

const MODE_FAMILY_PATH = path.resolve(__dirname, '../material-kkt-mode-family-geometry/mode-family-geometry.js');
const EXACT_AREA_PATH = path.resolve(__dirname, '../material-exact-triangle-sweep-area/exact-triangle-sweep-area.js');
const EXPECTED_MODE_FAMILY_SHA256 = 'EDD725432EB01E0A800647A68A6B67E74B4F2AA27A134805FAC21B2D46519EB8';
const EXPECTED_EXACT_AREA_SHA256 = 'BC128BAFC05397B073584D5FAF4362A7AD3F09CD91EBBFF173886AD84B43A77B';

function sha256(value) { return crypto.createHash('sha256').update(value).digest('hex').toUpperCase(); }
function fileSha256(filename) { return sha256(fs.readFileSync(filename)); }
function selfSha256() { return fileSha256(__filename); }
function clone(value) { return value === undefined ? undefined : JSON.parse(JSON.stringify(value)); }
function finite(value, label) {
  if (!Number.isFinite(value)) throw new TypeError(`${label} must be finite`);
  return Number(value);
}
function canonicalValue(value) {
  if (value === null || typeof value === 'string' || typeof value === 'boolean') return value;
  if (typeof value === 'number') {
    if (!Number.isFinite(value)) throw new TypeError('canonical values must be finite');
    return Object.is(value, -0) ? 0 : value;
  }
  if (Array.isArray(value)) return value.map(canonicalValue);
  if (value && typeof value === 'object') {
    const output = {}; Object.keys(value).sort().forEach((key) => { output[key] = canonicalValue(value[key]); });
    return output;
  }
  throw new TypeError(`unsupported canonical type ${typeof value}`);
}
function digest(label, value) { return sha256(`${label}\n${JSON.stringify(canonicalValue(value))}`); }
function deepFreeze(value) {
  if (value && typeof value === 'object' && !Object.isFrozen(value)) {
    Object.freeze(value); Object.values(value).forEach(deepFreeze);
  }
  return value;
}
function domainStop(reason, extra = {}) {
  return deepFreeze({ ok: false, status: 'solver-domain-stop', reason: String(reason), ...extra });
}
function nextUp(value) { return exactAreaModule._reference.nextUp(value); }
function outwardNonnegative(value) {
  if (!(value >= 0 && Number.isFinite(value))) throw new Error('geometric variation is invalid');
  return nextUp(nextUp(value * (1 + 512 * Number.EPSILON) + Number.MIN_VALUE));
}
function rationalToNearest(text) {
  const [numerator, denominator] = String(text).split('/').map(BigInt);
  return Number(numerator) / Number(denominator);
}

function rationalBounds(text) {
  const [numerator, denominator] = String(text).split('/').map(BigInt);
  return exactAreaModule._reference.toNumberBounds(exactAreaModule._reference.rat(numerator, denominator));
}
function pointDistance(left, right) { return Math.hypot(left[0] - right[0], left[1] - right[1]); }
function determinant(a, b, c) {
  return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0]);
}
function pointSegmentDistance(point, start, end) {
  const dx = end[0] - start[0]; const dy = end[1] - start[1]; const lengthSquared = dx * dx + dy * dy;
  if (lengthSquared === 0) return pointDistance(point, start);
  const t = Math.max(0, Math.min(1, ((point[0] - start[0]) * dx + (point[1] - start[1]) * dy) / lengthSquared));
  return Math.hypot(point[0] - (start[0] + t * dx), point[1] - (start[1] + t * dy));
}
function pointOnSegmentNear(point, start, end, coordinateGuard, determinantGuard) {
  return Math.abs(determinant(start, end, point)) <= determinantGuard
    && point[0] >= Math.min(start[0], end[0]) - coordinateGuard
    && point[0] <= Math.max(start[0], end[0]) + coordinateGuard
    && point[1] >= Math.min(start[1], end[1]) - coordinateGuard
    && point[1] <= Math.max(start[1], end[1]) + coordinateGuard;
}
function segmentDistance(a, b, c, d, coordinateGuard, determinantGuard) {
  const o1 = determinant(a, b, c); const o2 = determinant(a, b, d);
  const o3 = determinant(c, d, a); const o4 = determinant(c, d, b);
  const crossing = ((o1 > determinantGuard && o2 < -determinantGuard)
    || (o1 < -determinantGuard && o2 > determinantGuard))
    && ((o3 > determinantGuard && o4 < -determinantGuard)
      || (o3 < -determinantGuard && o4 > determinantGuard));
  const touching = pointOnSegmentNear(c, a, b, coordinateGuard, determinantGuard)
    || pointOnSegmentNear(d, a, b, coordinateGuard, determinantGuard)
    || pointOnSegmentNear(a, c, d, coordinateGuard, determinantGuard)
    || pointOnSegmentNear(b, c, d, coordinateGuard, determinantGuard);
  if (crossing || touching) return 0;
  return Math.min(pointSegmentDistance(a, c, d), pointSegmentDistance(b, c, d),
    pointSegmentDistance(c, a, b), pointSegmentDistance(d, a, b));
}
function polygonDistance(left, right, coordinateGuard) {
  let minimum = Infinity;
  const determinantGuard = coordinateGuard ** 2 + 4 * coordinateGuard
    * Math.max(...left.flatMap((point) => point.map(Math.abs)), ...right.flatMap((point) => point.map(Math.abs)), 1);
  for (let i = 0; i < left.length; i += 1) for (let j = 0; j < right.length; j += 1) {
    minimum = Math.min(minimum, segmentDistance(left[i], left[(i + 1) % left.length],
      right[j], right[(j + 1) % right.length], coordinateGuard, determinantGuard));
  }
  return Math.max(0, minimum - coordinateGuard);
}

function rawCellsFromPath(pathSample) {
  const nodes = pathSample?.inner?.nodes;
  if (!Array.isArray(nodes) || nodes.length < 2) throw new Error('finite path nodes are missing');
  const rawCells = [];
  for (let index = 0; index < nodes.length - 1; index += 1) rawCells.push([
    [nodes[index].start.x, nodes[index].start.y],
    [nodes[index].end.x, nodes[index].end.y],
    [nodes[index + 1].end.x, nodes[index + 1].end.y],
    [nodes[index + 1].start.x, nodes[index + 1].start.y],
  ]);
  return rawCells;
}

function cellsFromPathSample(pathSample) {
  const cells = [];
  for (const [index, raw] of rawCellsFromPath(pathSample).entries()) {
    const canonical = exactAreaModule._reference.canonicalOrderedConvexNumbers(raw, `timeCell[${index}]`);
    if (!canonical.length) continue;
    cells.push(canonical);
  }
  return cells;
}

function createExactModeFamilyGeometry(config) {
  try {
    if (!config || typeof config !== 'object') throw new TypeError('config object is required');
    if (fileSha256(MODE_FAMILY_PATH) !== EXPECTED_MODE_FAMILY_SHA256
      || modeFamilyModule.moduleSha256 !== EXPECTED_MODE_FAMILY_SHA256) {
      throw new Error('mode-family source hash mismatch');
    }
    if (fileSha256(EXACT_AREA_PATH) !== EXPECTED_EXACT_AREA_SHA256) {
      throw new Error('exact area source hash mismatch');
    }
    const base = modeFamilyModule.createModeFamilyGeometry(config.modeFamilyConfig);
    if (!base.ok) throw new Error(`base mode family stopped: ${base.reason}`);
    const cover = exactAreaModule.prepareTriangleCover(config.remainingTriangles);
    if (!cover.ok) throw new Error(`triangle cover stopped: ${cover.reason}`);
    const coverArea = rationalToNearest(cover.exactArea); const coverAreaEnclosure = rationalBounds(cover.exactArea);
    const exactSampleCache = new Map();
    const keyOf = (p) => Number(p).toPrecision(17);

    function exactSample(pValue, externalTrial) {
      try {
        const p = finite(pValue, 'p');
        const cacheKey = externalTrial ? null : keyOf(p);
        if (cacheKey && exactSampleCache.has(cacheKey)) return exactSampleCache.get(cacheKey);
        const pathSample = base.discretePathSample(p, externalTrial);
        if (!pathSample.ok) throw new Error(pathSample.reason);
        const cells = cellsFromPathSample(pathSample);
        const exact = exactAreaModule.computeExactFreshArea({ preparedTriangleCover: cover, cells });
        if (!exact.ok) throw new Error(`exact triangle/cell area stopped: ${exact.reason}`);
        if (exact.areaLower < -Number.MIN_VALUE || exact.areaUpper > coverAreaEnclosure.upper) {
          throw new Error('exact fresh area exceeds the prepared cover');
        }
        const baseSample = base.exactSample(p, externalTrial);
        if (baseSample.ok && (exact.areaUpper < baseSample.areaLower || exact.areaLower > baseSample.areaUpper)) {
          throw new Error('exact triangle/cell area disagrees with the independent polygon enclosure');
        }
        const payload = {
          schema: 'exact-mode-family-triangle-sweep-payload-v1',
          p,
          pathSample: clone(pathSample),
          pathSampleDigest: pathSample.pathSampleDigest,
          independentPolygonCrossCheck: baseSample.ok ? {
            payload: clone(baseSample.payload), sampleDigest: baseSample.sampleDigest,
          } : { status: 'independent-polygon-domain-stop', reason: baseSample.reason },
          exactArea: {
            numerator: exact.exactAreaNumerator,
            denominator: exact.exactAreaDenominator,
            lower: exact.areaLower,
            upper: exact.areaUpper,
          },
          centroid: clone(exact.centroid),
          exactMoments: clone(exact.exactMoments),
          momentBounds: clone(exact.momentBounds),
          exactResultSignature: exact.resultSignature,
        };
        payload.payloadDigest = digest('exact-mode-family-triangle-sweep-payload-v1', payload);
        const result = {
          ok: true,
          status: 'exact-dyadic-mode-family-sample',
          p,
          area: exact.area,
          freshArea: exact.area,
          areaLower: exact.areaLower,
          areaUpper: exact.areaUpper,
          payload,
          modeKey: pathSample.modeKey,
          qPost: clone(pathSample.qPost),
          authority: 'none-pure-mathematical-composition',
        };
        result.sampleDigest = digest('exact-mode-family-triangle-sweep-sample-v1', result);
        const frozen = deepFreeze(result);
        if (cacheKey) exactSampleCache.set(cacheKey, frozen);
        return frozen;
      } catch (error) { return domainStop(error?.message || error); }
    }

    const first = exactSample(config.modeFamilyConfig?.pDomain?.[0] ?? 0);
    if (!first.ok) throw new Error(`first exact sample stopped: ${first.reason}`);
    const baseFirst = base.exactSample(first.p);
    const baseCurrentArea = finite(baseFirst.payload?.inner?.currentArea, 'base current area');
    const baseGuard = finite(baseFirst.payload?.inner?.numericalAreaGuard, 'base numerical area guard');
    if (Math.abs(baseCurrentArea - coverArea) > 2 * baseGuard) {
      throw new Error('triangle-cover area does not match the frozen remaining geometry');
    }

    const geometrySnapshotSignature = digest('exact-mode-family-triangle-cover-v1', {
      baseGeometrySnapshotSignature: base.descriptor.geometrySnapshotSignature,
      cover,
    });
    const geometryLipschitzSignature = digest('exact-mode-family-geometric-variation-v1', {
      baseModeFamilySignature: base.descriptor.modeFamilySignature,
      exactAreaSha256: EXPECTED_EXACT_AREA_SHA256,
      roundingPolicy: 'exact-rational-samples-plus-outward-geometric-variation-v1',
    });

    function provePieceTopology(basePiece) {
      const envelopeDescriptor = base.descriptor.envelope?.[basePiece.envelopeIndex];
      const featureAngular = envelopeDescriptor?.featureAngularVelocityAffine;
      const materialAngular = envelopeDescriptor?.materialAngularVelocityAffine;
      if (!featureAngular || !materialAngular) return { ok: false, reason: 'angular motion family is missing' };
      if (featureAngular.intercept === 0 && featureAngular.slope === 0
        && materialAngular.intercept === 0 && materialAngular.slope === 0) {
        return { ok: true, status: 'zero-angular-translated-strip-partition' };
      }
      const subLo = basePiece.subLo; const subHi = basePiece.subHi;
      const midpoint = subLo + (subHi - subLo) / 2;
      const path = base.discretePathSample(midpoint);
      if (!path.ok) return { ok: false, reason: path.reason };
      const rawCells = rawCellsFromPath(path); const cellBounds = basePiece.inner?.cellProofs;
      if (!Array.isArray(cellBounds) || cellBounds.length !== rawCells.length) {
        return { ok: false, reason: 'cell displacement bounds are missing' };
      }
      const angularPInvariant = featureAngular.slope === 0 && materialAngular.slope === 0;
      let rawCellsLo = null; let rawCellsHi = null;
      if (angularPInvariant) {
        const pathLo = base.discretePathSample(subLo); const pathHi = base.discretePathSample(subHi);
        if (!pathLo.ok || !pathHi.ok) return { ok: false, reason: 'angular-invariant endpoint path sample stopped' };
        rawCellsLo = rawCellsFromPath(pathLo); rawCellsHi = rawCellsFromPath(pathHi);
      }
      const coordinateScale = Math.max(1, ...rawCells.flat(2).map(Math.abs));
      const coordinateGuard = 4096 * Number.EPSILON * coordinateScale;
      const determinantGuard = 8192 * Number.EPSILON * coordinateScale ** 2;
      const cells = []; const convexity = []; let commonSign = 0;
      for (let index = 0; index < rawCells.length; index += 1) {
        const raw = rawCells[index]; const radius = outwardNonnegative(finite(cellBounds[index].radius, 'cell radius'));
        let hull;
        try {
          hull = exactAreaModule._reference.canonicalOrderedConvexNumbers(raw, `intervalCell[${index}]`);
        } catch (error) {
          return { ok: false, reason: `cell ${index} ordered topology stopped: ${error.message}` };
        }
        if (hull.length < 3) return { ok: false, reason: `cell ${index} is degenerate at the interval midpoint` };
        const cornerProofs = []; let cellSign = 0;
        for (let corner = 0; corner < raw.length; corner += 1) {
          const a = raw[corner]; const b = raw[(corner + 1) % raw.length]; const c = raw[(corner + 2) % raw.length];
          const value = determinant(a, b, c);
          let variation; let sign;
          if (angularPInvariant) {
            const loRaw = rawCellsLo[index]; const hiRaw = rawCellsHi[index];
            const f0 = determinant(loRaw[corner], loRaw[(corner + 1) % loRaw.length],
              loRaw[(corner + 2) % loRaw.length]);
            const f1 = determinant(hiRaw[corner], hiRaw[(corner + 1) % hiRaw.length],
              hiRaw[(corner + 2) % hiRaw.length]);
            const coefficientA = 2 * (f1 + f0 - 2 * value);
            const coefficientB = f1 - f0 - coefficientA;
            const candidates = [f0, f1];
            if (coefficientA !== 0) {
              const extremum = -coefficientB / (2 * coefficientA);
              if (extremum > 0 && extremum < 1) {
                candidates.push(coefficientA * extremum ** 2 + coefficientB * extremum + f0);
              }
            }
            const minimum = Math.min(...candidates); const maximum = Math.max(...candidates);
            variation = outwardNonnegative(determinantGuard);
            if (minimum > variation) sign = 1;
            else if (maximum < -variation) sign = -1;
            else return { ok: false, reason: `cell ${index} quadratic convexity reaches zero inside the p interval` };
            cornerProofs.push({ corner, determinant: value, variation, sign,
              method: 'p-invariant-angular-quadratic-determinant', f0, fMid: value, f1,
              coefficientA, coefficientB, minimum, maximum });
          } else {
            const u = pointDistance(a, b); const v = pointDistance(a, c);
            variation = outwardNonnegative(2 * radius * (u + v) + 4 * radius ** 2 + determinantGuard);
            if (!(Math.abs(value) > variation)) {
              return { ok: false, reason: `cell ${index} convexity can reach zero inside the p interval` };
            }
            sign = Math.sign(value);
            cornerProofs.push({ corner, determinant: value, variation, sign,
              method: 'endpoint-displacement-determinant-bound' });
          }
          if (cellSign && sign !== cellSign) return { ok: false, reason: `cell ${index} folds at midpoint` };
          cellSign = sign;
        }
        if (commonSign && cellSign !== commonSign) return { ok: false, reason: 'adjacent cells have inconsistent orientation' };
        commonSign = cellSign; cells.push({ hull, radius });
        convexity.push({ index, radius, sign: cellSign, cornerProofs });
      }
      const separation = [];
      for (let left = 0; left < cells.length; left += 1) for (let right = left + 2; right < cells.length; right += 1) {
        const midpointDistance = polygonDistance(cells[left].hull, cells[right].hull, coordinateGuard);
        const required = outwardNonnegative(cells[left].radius + cells[right].radius + coordinateGuard);
        if (!(midpointDistance > required)) {
          return { ok: false, reason: `non-adjacent cells ${left}/${right} may overlap inside the p interval` };
        }
        separation.push({ left, right, midpointDistance, required });
      }
      return { ok: true, status: 'cellwise-convexity-and-nonadjacent-separation',
        midpoint, coordinateGuard, commonSign, convexity, separation };
    }

    function boundFreshAreaInterval(loValue, hiValue) {
      try {
        const lo = finite(loValue, 'lo'); const hi = finite(hiValue, 'hi');
        if (!(hi > lo)) throw new Error('interval must have positive width');
        const baseBound = base.boundDiscretePathInterval(lo, hi);
        if (!baseBound.ok) throw new Error(`base path interval stopped: ${baseBound.reason}`);
        const pieces = [];
        for (const basePiece of baseBound.pieces || []) {
          const subLo = finite(basePiece.subLo, 'piece subLo'); const subHi = finite(basePiece.subHi, 'piece subHi');
          const topology = provePieceTopology(basePiece);
          const variation = outwardNonnegative(finite(basePiece.inner?.geometricAreaVariation,
            'piece geometricAreaVariation'));
          if (!topology.ok) {
            pieces.push({ subLo, subHi, modeKey: basePiece.modeKey, variation,
              lower: 0, upper: coverAreaEnclosure.upper,
              topologyProof: { status: 'full-range-topology-unresolved', reason: topology.reason },
              basePathIntervalDigest: basePiece.inner?.pathIntervalDigest,
              baseInnerPathSignature: basePiece.inner?.pathSignature });
            continue;
          }
          const mid = subLo + (subHi - subLo) / 2;
          const left = exactSample(subLo); const middle = exactSample(mid); const right = exactSample(subHi);
          if (!left.ok || !middle.ok || !right.ok) throw new Error('an exact interval sample stopped');
          const lower = Math.max(0, Math.min(left.areaLower, right.areaLower,
            middle.areaLower - variation));
          const upper = Math.min(coverAreaEnclosure.upper, Math.max(left.areaUpper, right.areaUpper,
            middle.areaUpper + variation));
          if (!(left.area >= lower && left.area <= upper && middle.area >= lower && middle.area <= upper
            && right.area >= lower && right.area <= upper)) throw new Error('exact interval misses a check sample');
          pieces.push({ subLo, subHi, modeKey: basePiece.modeKey, variation, lower, upper,
            topologyProof: topology,
            leftSampleDigest: left.sampleDigest, middleSampleDigest: middle.sampleDigest,
            rightSampleDigest: right.sampleDigest,
            basePathIntervalDigest: basePiece.inner?.pathIntervalDigest,
            baseInnerPathSignature: basePiece.inner?.pathSignature });
        }
        if (!pieces.length || pieces[0].subLo !== lo || pieces.at(-1).subHi !== hi
          || pieces.some((entry, index) => index && pieces[index - 1].subHi !== entry.subLo)) {
          throw new Error('exact interval pieces do not form a complete ordered cover');
        }
        const areaLower = Math.max(0, Math.min(...pieces.map((entry) => entry.lower)));
        const areaUpper = Math.min(coverAreaEnclosure.upper, Math.max(...pieces.map((entry) => entry.upper)));
        const proof = {
          schema: 'exact-mode-family-triangle-sweep-interval-proof-v1',
          pieces,
          completeOrderedEnvelope: true,
          exactAreaSha256: EXPECTED_EXACT_AREA_SHA256,
          baseModeFamilySha256: EXPECTED_MODE_FAMILY_SHA256,
          basePathIntervalDigest: baseBound.pathIntervalDigest,
          coverExactArea: cover.exactArea,
          roundingPolicy: 'dyadic-rational-exact-samples; outward-inflated-analytic-variation',
          topologyProofDomain: 'zero-angular translated strip or cellwise convexity plus nonadjacent separation; unresolved gives [0,currentArea]',
        };
        const result = {
          ok: true,
          status: 'exact-sample-mode-family-interval',
          certificateType: 'exact-dyadic-sample-plus-analytic-mode-interval-v1',
          pLo: lo,
          pHi: hi,
          areaLower,
          areaUpper,
          minArea: areaLower,
          maxArea: areaUpper,
          moduleSha256: selfSha256(),
          sourceSignature: geometryLipschitzSignature,
          geometrySnapshotSignature,
          geometryLipschitzSignature,
          proof,
          authority: 'none-pure-mathematical-composition',
        };
        result.intervalDigest = digest('exact-mode-family-triangle-sweep-interval-v1', result);
        return deepFreeze(result);
      } catch (error) { return domainStop(error?.message || error); }
    }

    const descriptor = deepFreeze({
      schema: 'exact-mode-family-triangle-sweep-oracle-v1',
      moduleSha256: selfSha256(),
      modeFamilySha256: EXPECTED_MODE_FAMILY_SHA256,
      exactAreaSha256: EXPECTED_EXACT_AREA_SHA256,
      pDomain: clone(config.modeFamilyConfig?.pDomain),
      geometrySnapshotSignature,
      geometryLipschitzSignature,
      intervalOracle: {
        schema: 'signed-geometry-fresh-area-interval-v1',
        moduleSha256: selfSha256(),
        geometrySnapshotSignature,
        geometryLipschitzSignature,
      },
      cover: clone(cover),
      baseDescriptor: clone(base.descriptor),
      authority: 'none; Sfree, TOI, path, owner and cover completeness are not authenticated',
    });
    return deepFreeze({ ok: true, status: 'exact-mode-family-mathematical-oracle-ready', descriptor,
      exactSample, freshArea: exactSample, boundFreshAreaInterval });
  } catch (error) {
    return domainStop(error?.message || error, { moduleSha256: selfSha256() });
  }
}

module.exports = Object.freeze({ createExactModeFamilyGeometry,
  EXPECTED_MODE_FAMILY_SHA256, EXPECTED_EXACT_AREA_SHA256, moduleSha256: selfSha256() });
