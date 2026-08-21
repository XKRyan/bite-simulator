'use strict';

// Independent, dependency-free continuous witness helper for one triangular
// cutting feature against a polygon-clipping/GeoJSON-style MultiPolygon.
//
// The search is not an endpoint sweep.  It uses a Lipschitz bound on the
// relative speed of every possible feature/material point and recursively
// certifies time intervals empty.  Any unresolved leaf is returned with an
// explicit time/spatial error certificate.

const TWO_PI = Math.PI * 2;

function finite(value, fallback = 0) {
  const result = Number(value);
  return Number.isFinite(result) ? result : fallback;
}

function point(value, fallback = { x: 0, y: 0 }) {
  if (Array.isArray(value)) return { x: finite(value[0]), y: finite(value[1]) };
  if (value && typeof value === 'object') return { x: finite(value.x), y: finite(value.y) };
  return { x: fallback.x, y: fallback.y };
}

function add(a, b) { return { x: a.x + b.x, y: a.y + b.y }; }
function subtract(a, b) { return { x: a.x - b.x, y: a.y - b.y }; }
function scale(a, value) { return { x: a.x * value, y: a.y * value }; }
function dot(a, b) { return a.x * b.x + a.y * b.y; }
function cross(a, b) { return a.x * b.y - a.y * b.x; }
function lengthSquared(a) { return dot(a, a); }
function length(a) { return Math.hypot(a.x, a.y); }
function clamp(value, lower, upper) { return Math.max(lower, Math.min(upper, value)); }
function lerp(a, b, t) { return add(a, scale(subtract(b, a), t)); }

function rotate(a, angle) {
  const c = Math.cos(angle); const s = Math.sin(angle);
  return { x: c * a.x - s * a.y, y: s * a.x + c * a.y };
}

function normalize(a) {
  const magnitude = length(a);
  return magnitude > 0 ? scale(a, 1 / magnitude) : { x: 0, y: 0 };
}

function shortestAngleDelta(start, end) {
  let delta = end - start;
  while (delta > Math.PI) delta -= TWO_PI;
  while (delta <= -Math.PI) delta += TWO_PI;
  return delta;
}

function transformPoint(localPoint, pose) {
  return add(pose.position, rotate(localPoint, pose.angle));
}

function inverseTransformPoint(worldPoint, pose) {
  return rotate(subtract(worldPoint, pose.position), -pose.angle);
}

function normalizePose(value) {
  const source = value || {};
  return {
    position: source.position ? point(source.position) : point(source),
    angle: finite(source.angle),
  };
}

function normalizeMotion(value, dt) {
  const source = value || {};
  const start = normalizePose(source.start || source.pose || source);
  const end = normalizePose(source.end || source.pose || source.start || source);
  const angleDelta = Number.isFinite(Number(source.angleDelta))
    ? Number(source.angleDelta)
    : shortestAngleDelta(start.angle, end.angle);
  return {
    start,
    end,
    angleDelta,
    translationVelocity: scale(subtract(end.position, start.position), 1 / dt),
    angularVelocity: angleDelta / dt,
  };
}

function poseAt(motion, time, dt) {
  const fraction = clamp(time / dt, 0, 1);
  return {
    position: lerp(motion.start.position, motion.end.position, fraction),
    angle: motion.start.angle + motion.angleDelta * fraction,
  };
}

function rigidPointVelocity(motion, pose, localPoint) {
  const radial = rotate(localPoint, pose.angle);
  return add(motion.translationVelocity, {
    x: -motion.angularVelocity * radial.y,
    y: motion.angularVelocity * radial.x,
  });
}

function signedRingArea(ring) {
  let twiceArea = 0;
  for (let index = 0; index < ring.length; index += 1) {
    const a = ring[index]; const b = ring[(index + 1) % ring.length];
    twiceArea += cross(a, b);
  }
  return twiceArea / 2;
}

function normalizeRing(rawRing) {
  const ring = (rawRing || []).map((candidate) => point(candidate));
  if (ring.length > 1 && length(subtract(ring[0], ring[ring.length - 1])) <= 1e-14) ring.pop();
  if (ring.length < 3) throw new Error('Each material ring must have at least three distinct points.');
  return ring;
}

function normalizeMultiPolygon(rawGeometry) {
  if (!Array.isArray(rawGeometry)) throw new Error('material must be a MultiPolygon array.');
  return rawGeometry.map((rawPolygon, polygonIndex) => {
    if (!Array.isArray(rawPolygon) || rawPolygon.length < 1) {
      throw new Error(`material polygon ${polygonIndex} has no outer ring.`);
    }
    return rawPolygon.map((ring) => normalizeRing(ring));
  });
}

function materialBoundarySegments(geometry) {
  const result = [];
  let globalSegmentIndex = 0;
  geometry.forEach((polygon, polygonIndex) => polygon.forEach((ring, ringIndex) => {
    const area = signedRingArea(ring);
    if (Math.abs(area) <= 1e-20) throw new Error(`material polygon ${polygonIndex} ring ${ringIndex} is degenerate.`);
    for (let segmentIndex = 0; segmentIndex < ring.length; segmentIndex += 1) {
      const start = ring[segmentIndex]; const end = ring[(segmentIndex + 1) % ring.length];
      const tangent = normalize(subtract(end, start));
      if (lengthSquared(tangent) === 0) continue;
      const leftNormal = { x: -tangent.y, y: tangent.x };
      const ringInteriorNormal = area > 0 ? leftNormal : scale(leftNormal, -1);
      // Ring 0 contains material on its interior side. Hole rings contain
      // material on their exterior side, so their material-outward normal
      // points into the geometric interior of the hole.
      const outwardLocal = ringIndex === 0 ? scale(ringInteriorNormal, -1) : ringInteriorNormal;
      const ringId = ringIndex === 0
        ? `polygon-${polygonIndex}:outer`
        : `polygon-${polygonIndex}:hole-${ringIndex - 1}`;
      result.push({
        polygonIndex,
        ringIndex,
        ringId,
        segmentIndex,
        globalSegmentIndex,
        segmentId: `${ringId}:segment-${segmentIndex}`,
        start,
        end,
        outwardLocal,
      });
      globalSegmentIndex += 1;
    }
  }));
  return result;
}

function normalizeFeature(rawFeature) {
  const source = rawFeature || {};
  if (!Array.isArray(source.vertices) || source.vertices.length !== 3) {
    throw new Error('feature.vertices must contain exactly three triangle vertices.');
  }
  const vertices = source.vertices.map((candidate, index) => ({
    ...point(candidate),
    id: String(candidate?.id || source.vertexIds?.[index] || `vertex-${index}`),
  }));
  const inferredNamedTriangle = vertices.map((candidate) => candidate.id).join('|') === 'root|tip|back';
  const defaultEdgeIds = inferredNamedTriangle
    ? ['working-face', 'back-edge', 'root-edge']
    : ['edge-0', 'edge-1', 'edge-2'];
  const edges = vertices.map((_, index) => {
    const rawEdge = source.edges?.[index];
    const edgeId = String(rawEdge?.id || source.edgeIds?.[index] || defaultEdgeIds[index]);
    return {
      index,
      id: edgeId,
      working: Boolean(rawEdge?.working || source.workingEdgeIds?.includes(edgeId)),
    };
  });
  if (!edges.some((candidate) => candidate.working) && inferredNamedTriangle) edges[0].working = true;
  const workingVertexIds = new Set(
    source.workingVertexIds
      || (vertices.some((candidate) => candidate.id === 'tip') ? ['tip'] : []),
  );
  const vertexOwnerEdgeIndexes = vertices.map((_, vertexIndex) => {
    const explicit = source.vertexOwnerEdgeIndexes?.[vertexIndex];
    return Number.isInteger(explicit) ? explicit : (vertexIndex + vertices.length - 1) % vertices.length;
  });
  vertexOwnerEdgeIndexes.forEach((owner, index) => {
    if (owner < 0 || owner >= edges.length) throw new Error(`feature vertex ${index} has an invalid owner edge.`);
  });
  return { vertices, edges, workingVertexIds, vertexOwnerEdgeIndexes };
}

function pointSegmentClosest(query, start, end) {
  const delta = subtract(end, start);
  const denominator = lengthSquared(delta);
  const fraction = denominator > 0 ? clamp(dot(subtract(query, start), delta) / denominator, 0, 1) : 0;
  const closest = lerp(start, end, fraction);
  return { fraction, point: closest, distanceSquared: lengthSquared(subtract(query, closest)) };
}

function segmentIntersection(a0, a1, b0, b1, epsilon = 1e-13) {
  const r = subtract(a1, a0); const s = subtract(b1, b0);
  const denominator = cross(r, s); const delta = subtract(b0, a0);
  if (Math.abs(denominator) > epsilon) {
    const firstFraction = cross(delta, s) / denominator;
    const secondFraction = cross(delta, r) / denominator;
    if (firstFraction >= -epsilon && firstFraction <= 1 + epsilon
      && secondFraction >= -epsilon && secondFraction <= 1 + epsilon) {
      const fa = clamp(firstFraction, 0, 1); const fb = clamp(secondFraction, 0, 1);
      return {
        intersects: true,
        proper: fa > epsilon && fa < 1 - epsilon && fb > epsilon && fb < 1 - epsilon,
        firstFraction: fa,
        secondFraction: fb,
        point: lerp(a0, a1, fa),
      };
    }
    return { intersects: false, proper: false };
  }
  if (Math.abs(cross(delta, r)) > epsilon * Math.max(1, length(r))) {
    return { intersects: false, proper: false };
  }
  // Collinear overlap: choose the lexicographically earliest feature fraction,
  // then material fraction. This is deterministic and gives a common point.
  const candidates = [];
  const a0OnB = pointSegmentClosest(a0, b0, b1);
  if (a0OnB.distanceSquared <= epsilon ** 2) candidates.push({ firstFraction: 0, secondFraction: a0OnB.fraction, point: a0 });
  const a1OnB = pointSegmentClosest(a1, b0, b1);
  if (a1OnB.distanceSquared <= epsilon ** 2) candidates.push({ firstFraction: 1, secondFraction: a1OnB.fraction, point: a1 });
  const b0OnA = pointSegmentClosest(b0, a0, a1);
  if (b0OnA.distanceSquared <= epsilon ** 2) candidates.push({ firstFraction: b0OnA.fraction, secondFraction: 0, point: b0 });
  const b1OnA = pointSegmentClosest(b1, a0, a1);
  if (b1OnA.distanceSquared <= epsilon ** 2) candidates.push({ firstFraction: b1OnA.fraction, secondFraction: 1, point: b1 });
  candidates.sort((left, right) => left.firstFraction - right.firstFraction || left.secondFraction - right.secondFraction);
  if (!candidates.length) return { intersects: false, proper: false };
  return { intersects: true, proper: false, ...candidates[0] };
}

function closestSegmentPair(a0, a1, b0, b1) {
  const intersection = segmentIntersection(a0, a1, b0, b1);
  if (intersection.intersects) {
    return {
      distance: 0,
      firstFraction: intersection.firstFraction,
      secondFraction: intersection.secondFraction,
      firstPoint: intersection.point,
      secondPoint: intersection.point,
      properIntersection: intersection.proper,
    };
  }
  const candidates = [];
  const a0OnB = pointSegmentClosest(a0, b0, b1);
  candidates.push({ distanceSquared: a0OnB.distanceSquared, firstFraction: 0, secondFraction: a0OnB.fraction, firstPoint: a0, secondPoint: a0OnB.point });
  const a1OnB = pointSegmentClosest(a1, b0, b1);
  candidates.push({ distanceSquared: a1OnB.distanceSquared, firstFraction: 1, secondFraction: a1OnB.fraction, firstPoint: a1, secondPoint: a1OnB.point });
  const b0OnA = pointSegmentClosest(b0, a0, a1);
  candidates.push({ distanceSquared: b0OnA.distanceSquared, firstFraction: b0OnA.fraction, secondFraction: 0, firstPoint: b0OnA.point, secondPoint: b0 });
  const b1OnA = pointSegmentClosest(b1, a0, a1);
  candidates.push({ distanceSquared: b1OnA.distanceSquared, firstFraction: b1OnA.fraction, secondFraction: 1, firstPoint: b1OnA.point, secondPoint: b1 });
  candidates.sort((left, right) => left.distanceSquared - right.distanceSquared
    || left.firstFraction - right.firstFraction
    || left.secondFraction - right.secondFraction);
  const best = candidates[0];
  return { ...best, distance: Math.sqrt(Math.max(0, best.distanceSquared)), properIntersection: false };
}

function pointOnRingBoundary(query, ring, tolerance = 1e-12) {
  for (let index = 0; index < ring.length; index += 1) {
    if (pointSegmentClosest(query, ring[index], ring[(index + 1) % ring.length]).distanceSquared <= tolerance ** 2) return true;
  }
  return false;
}

function pointInsideRing(query, ring) {
  let inside = false;
  for (let index = 0, previous = ring.length - 1; index < ring.length; previous = index, index += 1) {
    const a = ring[index]; const b = ring[previous];
    const crossesRay = (a.y > query.y) !== (b.y > query.y);
    if (!crossesRay) continue;
    const x = (b.x - a.x) * (query.y - a.y) / (b.y - a.y) + a.x;
    if (query.x < x) inside = !inside;
  }
  return inside;
}

function pointMaterialRelation(query, geometry, tolerance = 1e-12) {
  for (const polygon of geometry) {
    for (const ring of polygon) if (pointOnRingBoundary(query, ring, tolerance)) return 0;
  }
  for (const polygon of geometry) {
    if (!pointInsideRing(query, polygon[0])) continue;
    let inHole = false;
    for (let ringIndex = 1; ringIndex < polygon.length; ringIndex += 1) {
      if (pointInsideRing(query, polygon[ringIndex])) { inHole = true; break; }
    }
    if (!inHole) return 1;
  }
  return -1;
}

function pointInsideTriangle(query, triangle, tolerance = 1e-13) {
  const signs = [];
  for (let index = 0; index < 3; index += 1) {
    signs.push(cross(subtract(triangle[(index + 1) % 3], triangle[index]), subtract(query, triangle[index])));
  }
  const hasPositive = signs.some((value) => value > tolerance);
  const hasNegative = signs.some((value) => value < -tolerance);
  if (hasPositive && hasNegative) return -1;
  if (signs.some((value) => Math.abs(value) <= tolerance)) return 0;
  return 1;
}

function triangleIntersectsMaterialInterior(triangle, geometry, boundarySegments, tolerance = 1e-12) {
  const centroid = scale(triangle.reduce((sum, candidate) => add(sum, candidate), { x: 0, y: 0 }), 1 / 3);
  if (pointMaterialRelation(centroid, geometry, tolerance) === 1) return true;
  if (triangle.some((candidate) => pointMaterialRelation(candidate, geometry, tolerance) === 1)) return true;

  for (const polygon of geometry) {
    for (const candidate of polygon[0]) {
      if (pointInsideTriangle(candidate, triangle, tolerance) === 1) return true;
    }
  }

  for (let edgeIndex = 0; edgeIndex < 3; edgeIndex += 1) {
    const start = triangle[edgeIndex]; const end = triangle[(edgeIndex + 1) % 3];
    for (const segment of boundarySegments) {
      if (segmentIntersection(start, end, segment.start, segment.end, tolerance).proper) return true;
    }
  }
  return false;
}

function canonicalFeatureLocation(feature, edgeIndex, edgeFraction, vertexDistanceTolerance) {
  let vertexIndex = null;
  const edgeLength = length(subtract(
    feature.vertices[(edgeIndex + 1) % feature.vertices.length],
    feature.vertices[edgeIndex],
  ));
  if (edgeFraction * edgeLength <= vertexDistanceTolerance) vertexIndex = edgeIndex;
  else if ((1 - edgeFraction) * edgeLength <= vertexDistanceTolerance) vertexIndex = (edgeIndex + 1) % feature.vertices.length;
  if (vertexIndex === null) {
    return {
      edgeIndex,
      edgeFraction,
      vertexIndex: null,
      localPoint: lerp(feature.vertices[edgeIndex], feature.vertices[(edgeIndex + 1) % 3], edgeFraction),
    };
  }
  const ownerEdgeIndex = feature.vertexOwnerEdgeIndexes[vertexIndex];
  const ownerStart = ownerEdgeIndex;
  const ownerEnd = (ownerEdgeIndex + 1) % 3;
  let ownerFraction;
  if (ownerStart === vertexIndex) ownerFraction = 0;
  else if (ownerEnd === vertexIndex) ownerFraction = 1;
  else throw new Error(`feature vertex owner edge ${ownerEdgeIndex} is not incident to vertex ${vertexIndex}.`);
  return {
    edgeIndex: ownerEdgeIndex,
    edgeFraction: ownerFraction,
    vertexIndex,
    localPoint: point(feature.vertices[vertexIndex]),
  };
}

function featureMaximumRadius(feature) {
  return Math.max(...feature.vertices.map((candidate) => length(candidate)));
}

function materialMaximumRadius(geometry) {
  let maximum = 0;
  geometry.forEach((polygon) => polygon.forEach((ring) => ring.forEach((candidate) => {
    maximum = Math.max(maximum, length(candidate));
  })));
  return maximum;
}

function makeCertificateBase(feature, materialSegments, featureMotion, materialMotion, dt, options) {
  const featureRadius = featureMaximumRadius(feature);
  const materialRadius = materialMaximumRadius(options.geometry);
  const relativeTranslationSpeed = length(subtract(
    featureMotion.translationVelocity,
    materialMotion.translationVelocity,
  ));
  const relativeSurfaceSpeedBound = relativeTranslationSpeed
    + Math.abs(featureMotion.angularVelocity) * featureRadius
    + Math.abs(materialMotion.angularVelocity) * materialRadius;
  const totalAngularTravel = Math.abs(featureMotion.angleDelta) + Math.abs(materialMotion.angleDelta);
  const angularCells = Math.max(1, Math.ceil(totalAngularTravel / options.maxAngularTravelPerCell));
  const featureCellAngle = Math.abs(featureMotion.angleDelta) / angularCells;
  const materialCellAngle = Math.abs(materialMotion.angleDelta) / angularCells;
  return {
    method: 'adaptive-lipschitz-interval-certificate',
    endpointOnly: false,
    exactRigidPoseEvaluation: true,
    dt,
    featureAngularTravel: Math.abs(featureMotion.angleDelta),
    materialAngularTravel: Math.abs(materialMotion.angleDelta),
    angularCells,
    maxAngularTravelPerCell: totalAngularTravel / angularCells,
    endpointChordSagittaNotReliedUpon: featureRadius * (1 - Math.cos(featureCellAngle / 2))
      + materialRadius * (1 - Math.cos(materialCellAngle / 2)),
    relativeTranslationSpeed,
    relativeSurfaceSpeedBound,
    featureRadius,
    materialRadius,
    timeTolerance: options.timeTolerance,
    distanceTolerance: options.distanceTolerance,
    globallyEnumeratedFeatureEdges: feature.edges.length,
    globallyEnumeratedMaterialSegments: materialSegments.length,
    evaluatedStates: 0,
    searchNodes: 0,
    certifiedEmptyIntervals: 0,
    unresolvedLeaves: 0,
    maximumNodes: options.maxNodes,
  };
}

function publicWitness(candidate, feature) {
  if (!candidate) return null;
  const edge = feature.edges[candidate.featureEdgeIndex];
  const vertex = candidate.featureVertexIndex === null
    ? null
    : feature.vertices[candidate.featureVertexIndex];
  const workingFace = vertex
    ? feature.workingVertexIds.has(vertex.id)
    : edge.working;
  return {
    featureEdgeId: edge.id,
    featureEdgeIndex: edge.index,
    featureFraction: candidate.featureFraction,
    featureVertexId: vertex?.id || null,
    featureVertexIndex: candidate.featureVertexIndex,
    featureRegion: vertex
      ? { kind: 'vertex', id: vertex.id, index: candidate.featureVertexIndex }
      : { kind: 'edge', id: edge.id, index: edge.index },
    contactRole: vertex?.id || edge.id,
    materialPolygonIndex: candidate.materialSegment.polygonIndex,
    materialRingId: candidate.materialSegment.ringId,
    materialRingIndex: candidate.materialSegment.ringIndex,
    materialSegmentId: candidate.materialSegment.segmentId,
    materialSegmentIndex: candidate.materialSegment.segmentIndex,
    materialGlobalSegmentIndex: candidate.materialSegment.globalSegmentIndex,
    materialFraction: candidate.materialFraction,
    featureWorldPoint: candidate.featureWorldPoint,
    materialWorldPoint: candidate.materialWorldPoint,
    commonWorldPoint: scale(add(candidate.featureWorldPoint, candidate.materialWorldPoint), 0.5),
    separation: candidate.distance,
    materialOutwardNormal: candidate.outwardWorld,
    signedClosingVelocity: candidate.signedClosingVelocity,
    workingFace,
  };
}

function findEarliestTriangleMaterialWitness(input) {
  const request = input || {};
  const dt = finite(request.dt);
  if (!(dt > 0)) throw new Error('dt must be positive.');
  const options = {
    timeTolerance: Math.max(1e-12, finite(request.options?.timeTolerance, Math.max(1e-10, dt * 1e-8))),
    distanceTolerance: Math.max(1e-14, finite(request.options?.distanceTolerance, 1e-10)),
    closingVelocityTolerance: Math.max(0, finite(request.options?.closingVelocityTolerance, 1e-10)),
    // Vertex identity is decided in physical length units, not by an arbitrary
    // fraction of differently sized feature edges.
    vertexDistanceTolerance: Math.max(
      1e-14,
      finite(request.options?.vertexDistanceTolerance, Math.max(1e-12, finite(request.options?.distanceTolerance, 1e-10) * 8)),
    ),
    distanceTieTolerance: Math.max(1e-14, finite(request.options?.distanceTieTolerance, 1e-10)),
    maxAngularTravelPerCell: Math.max(1e-4, finite(request.options?.maxAngularTravelPerCell, Math.PI / 36)),
    maxNodes: Math.max(100, Math.trunc(finite(request.options?.maxNodes, 500000))),
    continueAfterSeparatingBoundary: request.options?.continueAfterSeparatingBoundary !== false,
    geometry: normalizeMultiPolygon(request.material || []),
  };
  const originalGeometry = request.originalMaterial
    ? normalizeMultiPolygon(request.originalMaterial)
    : null;
  const feature = normalizeFeature(request.feature);
  const materialSegments = materialBoundarySegments(options.geometry);
  const originalSegments = originalGeometry ? materialBoundarySegments(originalGeometry) : null;
  const featureMotion = normalizeMotion(request.featureMotion, dt);
  const materialMotion = normalizeMotion(request.materialMotion || {
    start: { position: { x: 0, y: 0 }, angle: 0 },
    end: { position: { x: 0, y: 0 }, angle: 0 },
  }, dt);
  const certificate = makeCertificateBase(
    feature,
    materialSegments,
    featureMotion,
    materialMotion,
    dt,
    options,
  );

  if (!materialSegments.length) {
    return {
      found: false,
      contactClass: null,
      witness: null,
      reason: 'current material MultiPolygon has no boundary segments',
      certificate,
    };
  }

  const stateCache = new Map();
  const evaluate = (time) => {
    const boundedTime = clamp(time, 0, dt);
    const key = boundedTime.toPrecision(17);
    if (stateCache.has(key)) return stateCache.get(key);
    certificate.evaluatedStates += 1;
    const featurePose = poseAt(featureMotion, boundedTime, dt);
    const materialPose = poseAt(materialMotion, boundedTime, dt);
    const featureWorld = feature.vertices.map((candidate) => transformPoint(candidate, featurePose));
    const featureInMaterial = featureWorld.map((candidate) => inverseTransformPoint(candidate, materialPose));
    const strictOverlap = triangleIntersectsMaterialInterior(
      featureInMaterial,
      options.geometry,
      materialSegments,
      options.distanceTolerance,
    );
    const pairs = [];
    for (let edgeIndex = 0; edgeIndex < 3; edgeIndex += 1) {
      const edgeStart = featureWorld[edgeIndex]; const edgeEnd = featureWorld[(edgeIndex + 1) % 3];
      for (const materialSegment of materialSegments) {
        const materialStart = transformPoint(materialSegment.start, materialPose);
        const materialEnd = transformPoint(materialSegment.end, materialPose);
        const closest = closestSegmentPair(edgeStart, edgeEnd, materialStart, materialEnd);
        const location = canonicalFeatureLocation(
          feature,
          edgeIndex,
          closest.firstFraction,
          options.vertexDistanceTolerance,
        );
        const materialLocalPoint = lerp(materialSegment.start, materialSegment.end, closest.secondFraction);
        const outwardWorld = rotate(materialSegment.outwardLocal, materialPose.angle);
        const featureVelocity = rigidPointVelocity(featureMotion, featurePose, location.localPoint);
        const materialVelocity = rigidPointVelocity(materialMotion, materialPose, materialLocalPoint);
        const signedClosingVelocity = -dot(subtract(featureVelocity, materialVelocity), outwardWorld);
        let maximumPairClosingVelocity = -Infinity;
        for (const featureLocalEndpoint of [
          feature.vertices[edgeIndex],
          feature.vertices[(edgeIndex + 1) % 3],
        ]) {
          const endpointFeatureVelocity = rigidPointVelocity(featureMotion, featurePose, featureLocalEndpoint);
          for (const materialLocalEndpoint of [materialSegment.start, materialSegment.end]) {
            const endpointMaterialVelocity = rigidPointVelocity(materialMotion, materialPose, materialLocalEndpoint);
            maximumPairClosingVelocity = Math.max(
              maximumPairClosingVelocity,
              -dot(subtract(endpointFeatureVelocity, endpointMaterialVelocity), outwardWorld),
            );
          }
        }
        pairs.push({
          distance: closest.distance,
          featureEdgeIndex: location.edgeIndex,
          featureFraction: location.edgeFraction,
          featureVertexIndex: location.vertexIndex,
          materialSegment,
          materialFraction: closest.secondFraction,
          featureWorldPoint: transformPoint(location.localPoint, featurePose),
          materialWorldPoint: transformPoint(materialLocalPoint, materialPose),
          outwardWorld,
          signedClosingVelocity,
          maximumPairClosingVelocity,
          properIntersection: closest.properIntersection,
        });
      }
    }
    const comparePairs = (left, right) => {
      if (Math.abs(left.distance - right.distance) > options.distanceTieTolerance) return left.distance - right.distance;
      if (Math.abs(left.signedClosingVelocity - right.signedClosingVelocity) > options.closingVelocityTolerance) {
        return right.signedClosingVelocity - left.signedClosingVelocity;
      }
      if (left.featureEdgeIndex !== right.featureEdgeIndex) return left.featureEdgeIndex - right.featureEdgeIndex;
      if (left.materialSegment.polygonIndex !== right.materialSegment.polygonIndex) {
        return left.materialSegment.polygonIndex - right.materialSegment.polygonIndex;
      }
      if (left.materialSegment.ringIndex !== right.materialSegment.ringIndex) {
        return left.materialSegment.ringIndex - right.materialSegment.ringIndex;
      }
      if (left.materialSegment.segmentIndex !== right.materialSegment.segmentIndex) {
        return left.materialSegment.segmentIndex - right.materialSegment.segmentIndex;
      }
      if (left.featureFraction !== right.featureFraction) return left.featureFraction - right.featureFraction;
      return left.materialFraction - right.materialFraction;
    };
    pairs.sort(comparePairs);
    const nearest = pairs[0] || null;
    const touchingPairs = pairs
      .filter((candidate) => candidate.distance <= options.distanceTolerance)
      .sort(comparePairs);
    const compressivePairs = touchingPairs.filter(
      (candidate) => candidate.signedClosingVelocity > options.closingVelocityTolerance,
    );
    const primary = compressivePairs[0] || touchingPairs[0] || nearest;
    const state = {
      time: boundedTime,
      strictOverlap,
      distance: nearest?.distance ?? Infinity,
      geometricContact: strictOverlap || touchingPairs.length > 0,
      compressiveContact: strictOverlap || compressivePairs.length > 0,
      contact: strictOverlap || compressivePairs.length > 0,
      primary,
      nearest,
      pairs,
      touchingPairs,
      compressivePairs,
      featureInMaterial,
    };
    stateCache.set(key, state);
    return state;
  };

  const startState = evaluate(0);
  const originalStartOverlap = originalGeometry
    ? triangleIntersectsMaterialInterior(
      startState.featureInMaterial,
      originalGeometry,
      originalSegments,
      options.distanceTolerance,
    )
    : false;
  const reentry = Boolean(request.history?.hadPriorContact)
    || (originalStartOverlap && !startState.strictOverlap);

  const finish = (state, bracket, conservativeOnly = false) => {
    const witness = publicWitness(state.primary, feature);
    const contactClass = reentry ? 're-entry' : 'virgin-contact';
    const compressive = Boolean(witness && witness.signedClosingVelocity > options.closingVelocityTolerance);
    const actualCertifiedContact = Boolean(
      state.strictOverlap || (witness && witness.separation <= options.distanceTolerance),
    );
    const effectiveConservativeOnly = conservativeOnly || !actualCertifiedContact;
    const removalAllowed = Boolean(witness?.workingFace && compressive && !effectiveConservativeOnly);
    let domainAction = 'material-response';
    let refusalReason = null;
    if (!witness?.workingFace) {
      domainAction = 'stop-non-working-boundary';
      refusalReason = `first onset belongs to non-working feature region ${witness?.contactRole || 'unknown'}`;
    } else if (!compressive) {
      domainAction = 'stop-non-compressive-boundary';
      refusalReason = 'first onset does not have positive signed closing velocity';
    } else if (effectiveConservativeOnly) {
      domainAction = 'refine-before-material-response';
      refusalReason = 'contact is enclosed only by the conservative error bracket';
    }
    const bracketWidth = Math.max(0, bracket[1] - bracket[0]);
    return {
      found: true,
      status: effectiveConservativeOnly ? 'conservative-contact-bracket' : 'contact',
      contactClass,
      toi: state.time,
      fraction: state.time / dt,
      toiBracket: bracket,
      witness,
      removalAllowed,
      domainAction,
      refusalReason,
      certificate: {
        ...certificate,
        toiBracketWidth: bracketWidth,
        // The full bracket width is intentionally used. It remains valid when
        // the reported sample is at either endpoint, unlike a midpoint /2 term.
        maximumUnresolvedSpatialError: options.distanceTolerance
          + certificate.relativeSurfaceSpeedBound * bracketWidth,
      },
    };
  };

  if (startState.strictOverlap) {
    return {
      found: true,
      status: 'invalid-initial-overlap',
      contactClass: 'initial-overlap',
      toi: 0,
      fraction: 0,
      toiBracket: [0, 0],
      witness: publicWitness(startState.primary, feature),
      removalAllowed: false,
      domainAction: 'stop-initial-overlap',
      refusalReason: 'the interval begins with positive-area feature/material overlap',
      certificate: { ...certificate, toiBracketWidth: 0, maximumUnresolvedSpatialError: 0 },
    };
  }
  if (startState.compressiveContact) return finish(startState, [0, 0]);

  let searchFloor = 0;
  if (startState.geometricContact && !startState.compressiveContact) {
    certificate.initialSeparatingBoundary = true;
    if (!options.continueAfterSeparatingBoundary) {
      return {
        found: true,
        status: 'separating-boundary',
        contactClass: 'separating-boundary',
        toi: 0,
        fraction: 0,
        toiBracket: [0, 0],
        witness: publicWitness(startState.primary, feature),
        removalAllowed: false,
        domainAction: 'no-material-response',
        refusalReason: 'initial boundary touch is separating or non-compressive',
        certificate: { ...certificate, toiBracketWidth: 0, maximumUnresolvedSpatialError: 0 },
      };
    }

    // A separating old boundary must not hide a later compressive onset.  Skip
    // only a prefix that is certified non-compressive for every segment pair.
    // At a touching pair we bound the change of closing velocity; at every
    // other pair the distance Lipschitz bound prevents contact.
    const accelerationBound = featureMotion.angularVelocity ** 2 * certificate.featureRadius
      + materialMotion.angularVelocity ** 2 * certificate.materialRadius;
    const closingRateBound = accelerationBound
      + certificate.relativeSurfaceSpeedBound * Math.abs(materialMotion.angularVelocity);
    certificate.relativeClosingRateBound = closingRateBound;
    certificate.certifiedSeparatingPrefixSteps = 0;
    let separatingState = startState;
    for (let iteration = 0; iteration < 128
      && separatingState.geometricContact
      && !separatingState.compressiveContact
      && searchFloor < dt; iteration += 1) {
      let safeAdvance = dt - searchFloor;
      for (const pair of separatingState.pairs) {
        if (pair.distance <= options.distanceTolerance) {
          if (closingRateBound > 0) {
            safeAdvance = Math.min(
              safeAdvance,
              Math.max(0, (options.closingVelocityTolerance - pair.maximumPairClosingVelocity) / closingRateBound) * 0.5,
            );
          } else if (pair.maximumPairClosingVelocity > options.closingVelocityTolerance) {
            safeAdvance = 0;
          }
        } else {
          safeAdvance = Math.min(
            safeAdvance,
            Math.max(0, (pair.distance - options.distanceTolerance)
              / certificate.relativeSurfaceSpeedBound) * 0.5,
          );
        }
      }
      if (!(safeAdvance > options.timeTolerance)) {
        return finish(separatingState, [searchFloor, Math.min(dt, searchFloor + options.timeTolerance)], true);
      }
      searchFloor += safeAdvance;
      certificate.certifiedSeparatingPrefixSteps += 1;
      separatingState = evaluate(searchFloor);
    }
    if (separatingState.compressiveContact) {
      return finish(separatingState, [Math.max(0, searchFloor - options.timeTolerance), searchFloor], true);
    }
    if (separatingState.geometricContact) {
      return finish(separatingState, [searchFloor, Math.min(dt, searchFloor + options.timeTolerance)], true);
    }
    if (searchFloor >= dt) {
      return {
        found: false,
        status: 'no-contact',
        contactClass: null,
        witness: null,
        reason: 'the initial separating boundary was certified non-compressive through the interval',
        certificate,
      };
    }
  }

  if (!(certificate.relativeSurfaceSpeedBound > 0)) {
    return {
      found: false,
      status: 'no-contact',
      contactClass: null,
      witness: null,
      reason: 'zero relative surface-speed bound and no initial contact',
      certificate,
    };
  }

  let nodeLimitReached = false;
  const searchInterval = (startTime, endTime, stateStart, stateEnd) => {
    certificate.searchNodes += 1;
    if (certificate.searchNodes > options.maxNodes) {
      nodeLimitReached = true;
      return null;
    }
    if (stateStart.contact) return { lower: startTime, upper: startTime, states: [stateStart], conservativeOnly: false };
    const middleTime = (startTime + endTime) / 2;
    const stateMiddle = evaluate(middleTime);
    const halfWidth = (endTime - startTime) / 2;
    const distanceLowerBound = stateMiddle.distance
      - certificate.relativeSurfaceSpeedBound * halfWidth;
    if (!stateMiddle.strictOverlap && distanceLowerBound > options.distanceTolerance) {
      certificate.certifiedEmptyIntervals += 1;
      return null;
    }
    if (endTime - startTime <= options.timeTolerance) {
      certificate.unresolvedLeaves += 1;
      const states = [stateStart, stateMiddle, stateEnd].sort((left, right) => left.time - right.time);
      return {
        lower: startTime,
        upper: endTime,
        states,
        conservativeOnly: !states.some((candidate) => candidate.contact),
      };
    }
    const left = searchInterval(startTime, middleTime, stateStart, stateMiddle);
    if (left) return left;
    return searchInterval(middleTime, endTime, stateMiddle, stateEnd);
  };

  let leaf = null;
  for (let cell = 0; cell < certificate.angularCells && !leaf; cell += 1) {
    const startTime = Math.max(searchFloor, dt * cell / certificate.angularCells);
    const endTime = dt * (cell + 1) / certificate.angularCells;
    if (endTime <= searchFloor) continue;
    leaf = searchInterval(startTime, endTime, evaluate(startTime), evaluate(endTime));
    if (nodeLimitReached) break;
  }

  if (nodeLimitReached) {
    return {
      found: false,
      status: 'indeterminate-node-limit',
      contactClass: null,
      witness: null,
      reason: `adaptive search exceeded ${options.maxNodes} nodes`,
      certificate,
    };
  }
  if (!leaf) {
    return {
      found: false,
      status: 'no-contact',
      contactClass: null,
      witness: null,
      reason: 'every time interval was conservatively certified clear',
      certificate,
    };
  }

  // Seek an actually touching/overlapping sample inside the conservative leaf.
  // This refines ordinary crossings while retaining a certified tangent leaf.
  let sampledStates = [...leaf.states];
  for (let sampleIndex = 1; sampleIndex < 16 && !sampledStates.some((candidate) => candidate.contact); sampleIndex += 1) {
    sampledStates.push(evaluate(leaf.lower + (leaf.upper - leaf.lower) * sampleIndex / 16));
  }
  sampledStates.sort((left, right) => left.time - right.time);
  const firstContactIndex = sampledStates.findIndex((candidate) => candidate.contact);
  if (firstContactIndex >= 0) {
    let upperState = sampledStates[firstContactIndex];
    let sampleLowerState = firstContactIndex > 0 ? sampledStates[firstContactIndex - 1] : evaluate(leaf.lower);
    const witnessRefinementTimeTolerance = Math.min(
      options.timeTolerance,
      Math.max(1e-14, options.distanceTolerance
        / Math.max(1e-30, certificate.relativeSurfaceSpeedBound) * 0.25),
    );
    if (!sampleLowerState.contact) {
      for (let iteration = 0; iteration < 100
        && upperState.time - sampleLowerState.time > witnessRefinementTimeTolerance; iteration += 1) {
        const middleState = evaluate((sampleLowerState.time + upperState.time) / 2);
        if (middleState.contact) upperState = middleState; else sampleLowerState = middleState;
      }
    }
    // The ordinary boolean samples above may tighten the known-contact upper
    // bound, but they cannot certify absence of a briefer earlier touch. Keep
    // the recursively certified leaf lower bound.
    return finish(upperState, [leaf.lower, upperState.time], false);
  }

  sampledStates.sort((left, right) => left.distance - right.distance || left.time - right.time);
  const nearestState = sampledStates[0];
  return finish(nearestState, [leaf.lower, leaf.upper], true);
}

module.exports = {
  findEarliestTriangleMaterialWitness,
  _geometry: {
    closestSegmentPair,
    materialBoundarySegments,
    normalizeMultiPolygon,
    pointMaterialRelation,
    signedRingArea,
  },
};
