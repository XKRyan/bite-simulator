'use strict';

// Exact, work-only planar area kernel.
//
// Every finite JavaScript Number is first decoded as its exact dyadic rational.
// Convex clipping and shoelace accumulation then use reduced BigInt fractions.
// This module deliberately does not authenticate triangulation, motion, TOI,
// working-face ownership, or a Rapier state.  Those remain upstream duties.

const crypto = require('node:crypto');
const fs = require('node:fs');

const SELF_PATH = __filename;
const numberView = new DataView(new ArrayBuffer(8));
const preparedTriangleCovers = new WeakMap();

function hashFile(filename) {
  return crypto.createHash('sha256').update(fs.readFileSync(filename)).digest('hex').toUpperCase();
}

function domainStop(reason, extra = {}) {
  return Object.freeze({ ok: false, status: 'geometry-domain-stop', reason: String(reason), ...extra });
}

function absBig(value) { return value < 0n ? -value : value; }

function gcd(left, right) {
  let a = absBig(left); let b = absBig(right);
  while (b !== 0n) { const next = a % b; a = b; b = next; }
  return a || 1n;
}

function rat(numerator, denominator = 1n) {
  if (denominator === 0n) throw new Error('zero rational denominator');
  let n = BigInt(numerator); let d = BigInt(denominator);
  if (d < 0n) { n = -n; d = -d; }
  if (n === 0n) return { n: 0n, d: 1n };
  const divisor = gcd(n, d);
  return { n: n / divisor, d: d / divisor };
}

function fromNumber(value, label = 'number') {
  if (!Number.isFinite(value)) throw new Error(`${label} must be finite`);
  if (value === 0) return rat(0n);
  numberView.setFloat64(0, value, false);
  const high = numberView.getUint32(0, false);
  const low = numberView.getUint32(4, false);
  const negative = (high >>> 31) !== 0;
  const exponentBits = (high >>> 20) & 0x7ff;
  const fraction = (BigInt(high & 0xfffff) << 32n) | BigInt(low);
  let mantissa; let exponent;
  if (exponentBits === 0) {
    mantissa = fraction;
    exponent = -1074;
  } else {
    mantissa = (1n << 52n) | fraction;
    exponent = exponentBits - 1023 - 52;
  }
  let n = negative ? -mantissa : mantissa;
  if (exponent >= 0) n <<= BigInt(exponent);
  else return rat(n, 1n << BigInt(-exponent));
  return rat(n);
}

function add(left, right) { return rat(left.n * right.d + right.n * left.d, left.d * right.d); }
function subtract(left, right) { return rat(left.n * right.d - right.n * left.d, left.d * right.d); }
function multiply(left, right) { return rat(left.n * right.n, left.d * right.d); }
function divide(left, right) {
  if (right.n === 0n) throw new Error('division by zero');
  return rat(left.n * right.d, left.d * right.n);
}
function negate(value) { return { n: -value.n, d: value.d }; }
function compare(left, right) {
  const delta = left.n * right.d - right.n * left.d;
  return delta < 0n ? -1 : delta > 0n ? 1 : 0;
}
function equal(left, right) { return left.n === right.n && left.d === right.d; }
function rationalKey(value) { return `${value.n}/${value.d}`; }

function point(raw, label = 'point') {
  const x = Array.isArray(raw) ? raw[0] : raw?.x;
  const y = Array.isArray(raw) ? raw[1] : raw?.y;
  return { x: fromNumber(x, `${label}.x`), y: fromNumber(y, `${label}.y`) };
}

function pointKey(value) { return `${rationalKey(value.x)},${rationalKey(value.y)}`; }
function pointEqual(left, right) { return equal(left.x, right.x) && equal(left.y, right.y); }
function pointSubtract(left, right) { return { x: subtract(left.x, right.x), y: subtract(left.y, right.y) }; }
function pointAdd(left, right) { return { x: add(left.x, right.x), y: add(left.y, right.y) }; }
function pointScale(value, scalar) { return { x: multiply(value.x, scalar), y: multiply(value.y, scalar) }; }
function crossVectors(left, right) { return subtract(multiply(left.x, right.y), multiply(left.y, right.x)); }
function orient(a, b, c) { return crossVectors(pointSubtract(b, a), pointSubtract(c, a)); }
function betweenInclusive(value, left, right) {
  const minimum = compare(left, right) <= 0 ? left : right;
  const maximum = compare(left, right) <= 0 ? right : left;
  return compare(value, minimum) >= 0 && compare(value, maximum) <= 0;
}
function pointOnSegmentExact(candidate, start, end) {
  return orient(start, end, candidate).n === 0n
    && betweenInclusive(candidate.x, start.x, end.x)
    && betweenInclusive(candidate.y, start.y, end.y);
}
function segmentsIntersectExact(a, b, c, d) {
  const o1 = orient(a, b, c); const o2 = orient(a, b, d);
  const o3 = orient(c, d, a); const o4 = orient(c, d, b);
  const s1 = o1.n < 0n ? -1 : o1.n > 0n ? 1 : 0;
  const s2 = o2.n < 0n ? -1 : o2.n > 0n ? 1 : 0;
  const s3 = o3.n < 0n ? -1 : o3.n > 0n ? 1 : 0;
  const s4 = o4.n < 0n ? -1 : o4.n > 0n ? 1 : 0;
  if (s1 * s2 < 0 && s3 * s4 < 0) return true;
  return (s1 === 0 && pointOnSegmentExact(c, a, b))
    || (s2 === 0 && pointOnSegmentExact(d, a, b))
    || (s3 === 0 && pointOnSegmentExact(a, c, d))
    || (s4 === 0 && pointOnSegmentExact(b, c, d));
}

function cleanPolygon(points) {
  const cleaned = [];
  for (const entry of points) {
    if (!cleaned.length || !pointEqual(cleaned[cleaned.length - 1], entry)) cleaned.push(entry);
  }
  if (cleaned.length > 1 && pointEqual(cleaned[0], cleaned[cleaned.length - 1])) cleaned.pop();
  return cleaned;
}

function signedDoubleArea(polygon) {
  let value = rat(0n);
  for (let index = 0; index < polygon.length; index += 1) {
    const current = polygon[index]; const next = polygon[(index + 1) % polygon.length];
    value = add(value, subtract(multiply(current.x, next.y), multiply(current.y, next.x)));
  }
  return value;
}

function area(polygon) {
  const twice = signedDoubleArea(polygon);
  return divide(twice.n < 0n ? negate(twice) : twice, rat(2n));
}

function polygonMoments(rawPolygon) {
  let polygon = rawPolygon;
  if (signedDoubleArea(polygon).n < 0n) polygon = [...polygon].reverse();
  let twiceArea = rat(0n); let firstX6 = rat(0n); let firstY6 = rat(0n);
  let secondX12 = rat(0n); let secondY12 = rat(0n);
  for (let index = 0; index < polygon.length; index += 1) {
    const current = polygon[index]; const next = polygon[(index + 1) % polygon.length];
    const wedge = subtract(multiply(current.x, next.y), multiply(current.y, next.x));
    twiceArea = add(twiceArea, wedge);
    firstX6 = add(firstX6, multiply(add(current.x, next.x), wedge));
    firstY6 = add(firstY6, multiply(add(current.y, next.y), wedge));
    secondX12 = add(secondX12, multiply(add(add(multiply(current.x, current.x),
      multiply(current.x, next.x)), multiply(next.x, next.x)), wedge));
    secondY12 = add(secondY12, multiply(add(add(multiply(current.y, current.y),
      multiply(current.y, next.y)), multiply(next.y, next.y)), wedge));
  }
  const areaValue = divide(twiceArea, rat(2n));
  if (areaValue.n <= 0n) throw new Error('polygon moment area must be positive');
  return {
    area: areaValue,
    firstX: divide(firstX6, rat(6n)),
    firstY: divide(firstY6, rat(6n)),
    secondX: divide(secondX12, rat(12n)),
    secondY: divide(secondY12, rat(12n)),
  };
}

function convexHull(rawPoints, label = 'polygon') {
  const byKey = new Map();
  rawPoints.forEach((entry, index) => {
    const converted = point(entry, `${label}[${index}]`);
    byKey.set(pointKey(converted), converted);
  });
  const values = [...byKey.values()].sort((left, right) => compare(left.x, right.x) || compare(left.y, right.y));
  if (values.length < 3) return [];
  const half = (input) => {
    const output = [];
    for (const candidate of input) {
      while (output.length >= 2 && compare(orient(output[output.length - 2], output[output.length - 1], candidate), rat(0n)) <= 0) {
        output.pop();
      }
      output.push(candidate);
    }
    return output;
  };
  const lower = half(values); const upper = half([...values].reverse());
  const hull = cleanPolygon(lower.slice(0, -1).concat(upper.slice(0, -1)));
  return hull.length >= 3 && compare(area(hull), rat(0n)) > 0 ? hull : [];
}

function assertOrderedConvexSupport(ordered, hull, label) {
  if (new Set(ordered.map(pointKey)).size !== ordered.length) {
    throw new Error(`${label} contains repeated vertices`);
  }
  for (let left = 0; left < ordered.length; left += 1) {
    for (let right = left + 1; right < ordered.length; right += 1) {
      if ((left + 1) % ordered.length === right || (right + 1) % ordered.length === left) continue;
      if (segmentsIntersectExact(ordered[left], ordered[(left + 1) % ordered.length],
        ordered[right], ordered[(right + 1) % ordered.length])) {
        throw new Error(`${label} has intersecting or overlapping non-adjacent edges`);
      }
    }
  }
  const orderedTwiceArea = signedDoubleArea(ordered);
  if (orderedTwiceArea.n === 0n) throw new Error(`${label} folds or self-intersects`);
  const sign = orderedTwiceArea.n > 0n ? 1 : -1;
  for (let index = 0; index < ordered.length; index += 1) {
    const turn = orient(ordered[index], ordered[(index + 1) % ordered.length],
      ordered[(index + 2) % ordered.length]);
    if (turn.n !== 0n && (turn.n > 0n ? 1 : -1) !== sign) {
      throw new Error(`${label} is concave, folded, or self-intersecting in its declared order`);
    }
  }
  const absoluteOrderedTwice = orderedTwiceArea.n < 0n ? negate(orderedTwiceArea) : orderedTwiceArea;
  const hullTwice = signedDoubleArea(hull);
  if (compare(absoluteOrderedTwice, hullTwice) !== 0) {
    throw new Error(`${label} declared order does not cover exactly its convex hull`);
  }
}

function canonicalConvexHullNumbers(rawPoints, label = 'cell') {
  const hull = convexHull(rawPoints, label);
  if (!hull.length) return [];
  return hull.map((entry) => [toNumberBounds(entry.x).nearest, toNumberBounds(entry.y).nearest]);
}

function canonicalOrderedConvexNumbers(rawPoints, label = 'cell') {
  if (!Array.isArray(rawPoints)) throw new Error(`${label} must be an array`);
  const ordered = cleanPolygon(rawPoints.map((entry, index) => point(entry, `${label}[${index}]`)));
  const hull = convexHull(rawPoints, label);
  if (!hull.length) return [];
  assertOrderedConvexSupport(ordered, hull, label);
  return hull.map((entry) => [toNumberBounds(entry.x).nearest, toNumberBounds(entry.y).nearest]);
}

function lineIntersection(segmentStart, segmentEnd, clipStart, clipEnd) {
  const motion = pointSubtract(segmentEnd, segmentStart);
  const clip = pointSubtract(clipEnd, clipStart);
  const denominator = crossVectors(motion, clip);
  if (denominator.n === 0n) throw new Error('uncertified parallel clipping transition');
  const t = divide(crossVectors(pointSubtract(clipStart, segmentStart), clip), denominator);
  return pointAdd(segmentStart, pointScale(motion, t));
}

function convexIntersection(subjectRaw, clipRaw) {
  let subject = cleanPolygon(subjectRaw);
  let clip = cleanPolygon(clipRaw);
  if (subject.length < 3 || clip.length < 3) return [];
  if (signedDoubleArea(subject).n < 0n) subject = [...subject].reverse();
  if (signedDoubleArea(clip).n < 0n) clip = [...clip].reverse();
  let output = subject;
  for (let edgeIndex = 0; edgeIndex < clip.length && output.length; edgeIndex += 1) {
    const a = clip[edgeIndex]; const b = clip[(edgeIndex + 1) % clip.length];
    const input = output; output = [];
    let start = input[input.length - 1];
    let startInside = orient(a, b, start).n >= 0n;
    for (const end of input) {
      const endInside = orient(a, b, end).n >= 0n;
      if (endInside) {
        if (!startInside) output.push(lineIntersection(start, end, a, b));
        output.push(end);
      } else if (startInside) output.push(lineIntersection(start, end, a, b));
      start = end; startInside = endInside;
    }
    output = cleanPolygon(output);
  }
  if (output.length < 3 || area(output).n === 0n) return [];
  return output;
}

function clipConvexHalfPlane(subjectRaw, clipStart, clipEnd, keepLeft) {
  const subject = cleanPolygon(subjectRaw);
  if (subject.length < 3) return [];
  const accepted = (value) => keepLeft ? value.n >= 0n : value.n <= 0n;
  let output = [];
  let start = subject[subject.length - 1];
  let startAccepted = accepted(orient(clipStart, clipEnd, start));
  for (const end of subject) {
    const endAccepted = accepted(orient(clipStart, clipEnd, end));
    if (endAccepted) {
      if (!startAccepted) output.push(lineIntersection(start, end, clipStart, clipEnd));
      output.push(end);
    } else if (startAccepted) output.push(lineIntersection(start, end, clipStart, clipEnd));
    start = end; startAccepted = endAccepted;
  }
  output = cleanPolygon(output);
  if (output.length < 3 || area(output).n === 0n) return [];
  return output;
}

// Decompose A \\ B into convex, interior-disjoint pieces.  At edge i of the
// CCW convex clip polygon, the emitted piece is the part that satisfied all
// earlier inside half-planes and first violates edge i.  The pieces therefore
// cannot overlap in positive area; the final remainder is exactly A intersect B.
function subtractConvex(subjectRaw, clipRaw) {
  let subject = cleanPolygon(subjectRaw); let clip = cleanPolygon(clipRaw);
  if (subject.length < 3) return [];
  if (clip.length < 3) return [subject];
  if (signedDoubleArea(subject).n < 0n) subject = [...subject].reverse();
  if (signedDoubleArea(clip).n < 0n) clip = [...clip].reverse();
  let remainder = subject; const output = [];
  for (let edgeIndex = 0; edgeIndex < clip.length && remainder.length; edgeIndex += 1) {
    const a = clip[edgeIndex]; const b = clip[(edgeIndex + 1) % clip.length];
    const outside = clipConvexHalfPlane(remainder, a, b, false);
    if (outside.length) output.push(outside);
    remainder = clipConvexHalfPlane(remainder, a, b, true);
  }
  return output;
}

function nextUp(value) {
  if (Number.isNaN(value) || value === Infinity) return value;
  if (value === 0) return Number.MIN_VALUE;
  numberView.setFloat64(0, value, false);
  let bits = numberView.getBigUint64(0, false);
  bits = value > 0 ? bits + 1n : bits - 1n;
  numberView.setBigUint64(0, bits, false);
  return numberView.getFloat64(0, false);
}

function nextDown(value) {
  if (Number.isNaN(value) || value === -Infinity) return value;
  if (value === 0) return -Number.MIN_VALUE;
  numberView.setFloat64(0, value, false);
  let bits = numberView.getBigUint64(0, false);
  bits = value > 0 ? bits - 1n : bits + 1n;
  numberView.setBigUint64(0, bits, false);
  return numberView.getFloat64(0, false);
}

function roundedQuotient(numerator, denominator) {
  let quotient = numerator / denominator; const remainder = numerator % denominator;
  const doubled = remainder * 2n;
  if (doubled > denominator || (doubled === denominator && (quotient & 1n) === 1n)) quotient += 1n;
  return quotient;
}

function nearestNumberFromRational(value) {
  if (value.n === 0n) return 0;
  const negative = value.n < 0n; const n = absBig(value.n); const d = value.d;
  const nBits = n.toString(2).length; const dBits = d.toString(2).length;
  let exponent = nBits - dBits;
  if (exponent >= 0 ? n < (d << BigInt(exponent)) : (n << BigInt(-exponent)) < d) exponent -= 1;
  if (exponent > 1023) throw new Error('rational is outside finite Number range');
  let bits;
  if (exponent >= -1022) {
    const scaleBits = 52 - exponent;
    const scaledNumerator = scaleBits >= 0 ? n << BigInt(scaleBits) : n;
    const scaledDenominator = scaleBits >= 0 ? d : d << BigInt(-scaleBits);
    let significand = roundedQuotient(scaledNumerator, scaledDenominator);
    if (significand === (1n << 53n)) { significand >>= 1n; exponent += 1; }
    if (exponent > 1023) throw new Error('rational rounds outside finite Number range');
    const exponentBits = BigInt(exponent + 1023);
    const fractionBits = significand - (1n << 52n);
    bits = (exponentBits << 52n) | fractionBits;
  } else {
    const significand = roundedQuotient(n << 1074n, d);
    if (significand === 0n) return negative ? -0 : 0;
    if (significand > (1n << 52n)) throw new Error('subnormal rounding overflow');
    bits = significand;
  }
  if (negative) bits |= 1n << 63n;
  numberView.setBigUint64(0, bits, false);
  return numberView.getFloat64(0, false);
}

function toNumberBounds(value) {
  const nearest = nearestNumberFromRational(value);
  if (!Number.isFinite(nearest)) throw new Error('rational is outside finite Number range');
  const represented = fromNumber(nearest, 'represented rational');
  const relation = compare(represented, value);
  return {
    nearest,
    lower: relation <= 0 ? nearest : nextDown(nearest),
    upper: relation >= 0 ? nearest : nextUp(nearest),
  };
}

function convertConvex(raw, label) {
  if (!Array.isArray(raw)) throw new Error(`${label} must be an array`);
  const converted = raw.map((entry, index) => point(entry, `${label}[${index}]`));
  if (converted.length > 1 && pointEqual(converted[0], converted[converted.length - 1])) converted.pop();
  for (let index = 1; index < converted.length; index += 1) {
    if (pointEqual(converted[index - 1], converted[index])) throw new Error(`${label} contains duplicate vertices`);
  }
  if (new Set(converted.map(pointKey)).size !== converted.length) {
    throw new Error(`${label} contains repeated vertices`);
  }
  const hull = convexHull(raw, label);
  if (hull.length < 3) throw new Error(`${label} is degenerate`);
  if (hull.length !== converted.length) {
    throw new Error(`${label} is not a strict convex polygon without duplicate/collinear vertices`);
  }
  assertOrderedConvexSupport(converted, hull, label);
  return hull;
}

function polygonBounds(polygon) {
  let minX = polygon[0].x; let maxX = polygon[0].x;
  let minY = polygon[0].y; let maxY = polygon[0].y;
  for (let index = 1; index < polygon.length; index += 1) {
    const entry = polygon[index];
    if (compare(entry.x, minX) < 0) minX = entry.x;
    if (compare(entry.x, maxX) > 0) maxX = entry.x;
    if (compare(entry.y, minY) < 0) minY = entry.y;
    if (compare(entry.y, maxY) > 0) maxY = entry.y;
  }
  return { minX, maxX, minY, maxY };
}
function boundsInteriorSeparated(left, right) {
  return compare(left.maxX, right.minX) < 0 || compare(right.maxX, left.minX) < 0
    || compare(left.maxY, right.minY) < 0 || compare(right.maxY, left.minY) < 0;
}
function convexInteriorSeparated(left, right) {
  const separatedBy = (source, other) => {
    for (let edge = 0; edge < source.length; edge += 1) {
      const a = source[edge]; const b = source[(edge + 1) % source.length];
      let maximum = null;
      for (const candidate of other) {
        const value = orient(a, b, candidate);
        if (maximum === null || compare(value, maximum) > 0) maximum = value;
      }
      if (maximum !== null && maximum.n <= 0n) return true;
    }
    return false;
  };
  return separatedBy(left, right) || separatedBy(right, left);
}

function validateDisjoint(polygons, label) {
  const bounds = polygons.map(polygonBounds);
  for (let left = 0; left < polygons.length; left += 1) {
    for (let right = left + 1; right < polygons.length; right += 1) {
      if (boundsInteriorSeparated(bounds[left], bounds[right])
        || convexInteriorSeparated(polygons[left], polygons[right])) continue;
      const overlap = convexIntersection(polygons[left], polygons[right]);
      if (overlap.length && area(overlap).n !== 0n) {
        throw new Error(`${label}[${left}] and ${label}[${right}] overlap in positive area`);
      }
    }
  }
}

function disjointConvexUnion(polygons, maximumPieces = 4096) {
  const union = [];
  for (let sourceCellIndex = 0; sourceCellIndex < polygons.length; sourceCellIndex += 1) {
    let candidates = [polygons[sourceCellIndex]];
    for (const existing of union) {
      const next = [];
      const existingBounds = polygonBounds(existing.polygon);
      for (const candidate of candidates) {
        if (boundsInteriorSeparated(polygonBounds(candidate), existingBounds)
          || convexInteriorSeparated(candidate, existing.polygon)) {
          next.push(candidate); continue;
        }
        const overlap = convexIntersection(candidate, existing.polygon);
        if (!overlap.length || area(overlap).n === 0n) next.push(candidate);
        else next.push(...subtractConvex(candidate, existing.polygon));
        if (union.length + next.length > maximumPieces) {
          throw new Error(`sweep-cell exact union exceeds ${maximumPieces} convex pieces`);
        }
      }
      candidates = next;
      if (!candidates.length) break;
    }
    for (const polygon of candidates) union.push({ polygon, sourceCellIndex });
    if (union.length > maximumPieces) {
      throw new Error(`sweep-cell exact union exceeds ${maximumPieces} convex pieces`);
    }
  }
  validateDisjoint(union.map((entry) => entry.polygon), 'unionPieces');
  return union;
}

function prepareTriangleCover(rawTriangles) {
  try {
    if (!Array.isArray(rawTriangles)) throw new Error('triangles array is required');
    const triangles = rawTriangles.map((entry, index) => {
      if (!Array.isArray(entry) || cleanPolygon(entry.map((raw, i) => point(raw, `triangles[${index}][${i}]`))).length !== 3) {
        throw new Error(`triangles[${index}] must contain exactly three unique vertices`);
      }
      return convertConvex(entry, `triangles[${index}]`);
    });
    validateDisjoint(triangles, 'triangles');
    let totalArea = rat(0n); let totalFirstX = rat(0n); let totalFirstY = rat(0n);
    let totalSecondX = rat(0n); let totalSecondY = rat(0n);
    for (const triangle of triangles) {
      const moments = polygonMoments(triangle);
      totalArea = add(totalArea, moments.area); totalFirstX = add(totalFirstX, moments.firstX);
      totalFirstY = add(totalFirstY, moments.firstY); totalSecondX = add(totalSecondX, moments.secondX);
      totalSecondY = add(totalSecondY, moments.secondY);
    }
    const publicCover = Object.freeze({
      ok: true,
      status: 'prepared-exact-triangle-cover',
      triangleCount: triangles.length,
      exactArea: rationalKey(totalArea),
      exactFirstMomentX: rationalKey(totalFirstX),
      exactFirstMomentY: rationalKey(totalFirstY),
      exactPolarSecondMomentOrigin: rationalKey(add(totalSecondX, totalSecondY)),
      authority: 'none-pure-mathematical-input',
      moduleSha256: hashFile(SELF_PATH),
    });
    preparedTriangleCovers.set(publicCover, { triangles, totalArea, totalFirstX, totalFirstY,
      totalSecondX, totalSecondY });
    return publicCover;
  } catch (error) {
    return domainStop(error?.message || error);
  }
}

function computeExactFreshArea(input) {
  try {
    if (!input || typeof input !== 'object') throw new Error('input object is required');
    if (!Array.isArray(input.cells)) throw new Error('cells array is required');
    let coverRecord;
    if (input.preparedTriangleCover) {
      coverRecord = preparedTriangleCovers.get(input.preparedTriangleCover);
      if (!coverRecord) throw new Error('preparedTriangleCover is foreign, cloned, or stale');
    } else {
      const prepared = prepareTriangleCover(input.triangles);
      if (!prepared.ok) throw new Error(prepared.reason);
      coverRecord = preparedTriangleCovers.get(prepared);
    }
    const triangles = coverRecord.triangles;
    const cells = input.cells.map((entry, index) => convertConvex(entry, `cells[${index}]`));
    const unionPieces = disjointConvexUnion(cells);
    const triangleBounds = triangles.map(polygonBounds);
    const unionPieceBounds = unionPieces.map((entry) => polygonBounds(entry.polygon));
    let exactArea = rat(0n); let exactFirstX = rat(0n); let exactFirstY = rat(0n);
    let exactSecondX = rat(0n); let exactSecondY = rat(0n); const pieces = [];
    for (let triangleIndex = 0; triangleIndex < triangles.length; triangleIndex += 1) {
      for (let unionPieceIndex = 0; unionPieceIndex < unionPieces.length; unionPieceIndex += 1) {
        const unionPiece = unionPieces[unionPieceIndex];
        if (boundsInteriorSeparated(triangleBounds[triangleIndex], unionPieceBounds[unionPieceIndex])
          || convexInteriorSeparated(triangles[triangleIndex], unionPiece.polygon)) continue;
        const intersection = convexIntersection(triangles[triangleIndex], unionPiece.polygon);
        if (!intersection.length) continue;
        const pieceMoments = polygonMoments(intersection); const pieceArea = pieceMoments.area;
        if (pieceArea.n === 0n) continue;
        exactArea = add(exactArea, pieceArea);
        exactFirstX = add(exactFirstX, pieceMoments.firstX);
        exactFirstY = add(exactFirstY, pieceMoments.firstY);
        exactSecondX = add(exactSecondX, pieceMoments.secondX);
        exactSecondY = add(exactSecondY, pieceMoments.secondY);
        pieces.push({ triangleIndex, unionPieceIndex, sourceCellIndex: unionPiece.sourceCellIndex,
          exactDoubleArea: rationalKey(multiply(pieceArea, rat(2n))) });
      }
    }
    const bounds = toNumberBounds(exactArea);
    const hasArea = exactArea.n > 0n;
    const exactCentroidX = hasArea ? divide(exactFirstX, exactArea) : null;
    const exactCentroidY = hasArea ? divide(exactFirstY, exactArea) : null;
    const exactPolarOrigin = add(exactSecondX, exactSecondY);
    const exactPolarCentroid = hasArea ? subtract(exactPolarOrigin,
      multiply(exactArea, add(multiply(exactCentroidX, exactCentroidX),
        multiply(exactCentroidY, exactCentroidY)))) : rat(0n);
    if (exactPolarCentroid.n < 0n) throw new Error('exact centroidal polar moment is negative');
    const firstXBounds = toNumberBounds(exactFirstX); const firstYBounds = toNumberBounds(exactFirstY);
    const polarOriginBounds = toNumberBounds(exactPolarOrigin);
    const polarCentroidBounds = toNumberBounds(exactPolarCentroid);
    const result = {
      ok: true,
      status: 'exact-dyadic-rational-area',
      area: bounds.nearest,
      areaLower: bounds.lower,
      areaUpper: bounds.upper,
      exactAreaNumerator: exactArea.n.toString(),
      exactAreaDenominator: exactArea.d.toString(),
      firstMomentX: firstXBounds.nearest,
      firstMomentY: firstYBounds.nearest,
      polarSecondMomentOrigin: polarOriginBounds.nearest,
      polarSecondMomentCentroid: polarCentroidBounds.nearest,
      centroid: hasArea ? {
        x: toNumberBounds(exactCentroidX).nearest,
        y: toNumberBounds(exactCentroidY).nearest,
      } : null,
      exactMoments: {
        firstMomentX: rationalKey(exactFirstX),
        firstMomentY: rationalKey(exactFirstY),
        polarSecondMomentOrigin: rationalKey(exactPolarOrigin),
        polarSecondMomentCentroid: rationalKey(exactPolarCentroid),
      },
      momentBounds: {
        firstMomentX: [firstXBounds.lower, firstXBounds.upper],
        firstMomentY: [firstYBounds.lower, firstYBounds.upper],
        polarSecondMomentOrigin: [polarOriginBounds.lower, polarOriginBounds.upper],
        polarSecondMomentCentroid: [polarCentroidBounds.lower, polarCentroidBounds.upper],
      },
      triangleCount: triangles.length,
      cellCount: cells.length,
      unionPieceCount: unionPieces.length,
      positivePieceCount: pieces.length,
      pieces,
      authority: 'none-pure-mathematical-input',
      moduleSha256: hashFile(SELF_PATH),
    };
    result.resultSignature = crypto.createHash('sha256').update(JSON.stringify(result)).digest('hex').toUpperCase();
    return Object.freeze(result);
  } catch (error) {
    return domainStop(error?.message || error);
  }
}

module.exports = Object.freeze({
  prepareTriangleCover,
  computeExactFreshArea,
  _reference: Object.freeze({ rat, fromNumber, add, subtract, multiply, divide, compare,
    convexHull, canonicalConvexHullNumbers, canonicalOrderedConvexNumbers,
    convexIntersection, area, polygonMoments,
    polygonBounds, boundsInteriorSeparated, convexInteriorSeparated,
    nearestNumberFromRational, toNumberBounds, nextUp, nextDown }),
});
