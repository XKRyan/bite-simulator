'use strict';

// Independent analytic geometry oracle used only by the S4a work tests. The
// runner pins the SHA-256 of this file. Production must substitute the frozen
// sweep/difference oracle and its own pinned module hash.

function descriptor(moduleSha256, geometrySnapshotSignature, lipschitzSignature) {
  return Object.freeze({
    schema: 'signed-geometry-fresh-area-interval-v1',
    moduleSha256,
    geometrySnapshotSignature,
    geometryLipschitzSignature: lipschitzSignature,
  });
}

function constantArea(moduleSha256, geometrySnapshotSignature, area) {
  const intervalOracle = descriptor(
    moduleSha256,
    geometrySnapshotSignature,
    `analytic-constant-area-L0-A${Number(area).toPrecision(17)}`,
  );
  return {
    intervalOracle,
    areaAt() { return area; },
    bound(lo, hi) {
      return {
        areaLower: area,
        areaUpper: area,
        certificateType: 'exact-analytic-constant-area-interval',
        sourceSignature: moduleSha256,
        moduleSha256,
        geometrySnapshotSignature,
        geometryLipschitzSignature: intervalOracle.geometryLipschitzSignature,
        proof: { kind: 'constant-area', area, interval: [lo, hi] },
      };
    },
  };
}

function triangularNotch(moduleSha256, geometrySnapshotSignature, parameters) {
  const center = Number(parameters.center);
  const halfWidth = Number(parameters.halfWidth);
  const baseline = Number(parameters.baseline);
  const minimum = Number(parameters.minimum);
  if (![center, halfWidth, baseline, minimum].every(Number.isFinite)
    || !(halfWidth > 0) || !(baseline >= minimum) || minimum < 0) {
    throw new TypeError('invalid analytic triangular-notch parameters');
  }
  const slope = (baseline - minimum) / halfWidth;
  const intervalOracle = descriptor(
    moduleSha256,
    geometrySnapshotSignature,
    `analytic-triangular-notch-L${slope.toPrecision(17)}`,
  );
  function areaAt(p) {
    const distance = Math.abs(p - center);
    return distance >= halfWidth
      ? baseline
      : minimum + slope * distance;
  }
  function bound(lo, hi) {
    if (!(Number.isFinite(lo) && Number.isFinite(hi) && hi > lo)) {
      throw new TypeError('invalid triangular-notch interval');
    }
    const critical = [lo, hi, center - halfWidth, center, center + halfWidth]
      .filter((p) => p >= lo && p <= hi);
    const areas = critical.map(areaAt);
    return {
      areaLower: Math.min(...areas),
      areaUpper: Math.max(...areas),
      certificateType: 'exact-analytic-piecewise-linear-area-interval',
      sourceSignature: moduleSha256,
      moduleSha256,
      geometrySnapshotSignature,
      geometryLipschitzSignature: intervalOracle.geometryLipschitzSignature,
      proof: {
        kind: 'piecewise-linear-critical-points',
        center,
        halfWidth,
        baseline,
        minimum,
        slope,
        critical,
      },
    };
  }
  return { intervalOracle, areaAt, bound };
}

module.exports = { descriptor, constantArea, triangularNotch };

