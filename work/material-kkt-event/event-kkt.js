'use strict';

// Work-only reference implementation for one material event.
//
// Sign convention:
//   * structural rows A_i admit A_i q + bias_i >= 0;
//   * their impulses are +A_i^T gamma_i, gamma_i >= 0;
//   * j*q > 0 is cutting motion;
//   * the resisting material impulse is -j^T lambda, lambda >= 0.
//
// The prescribed material impulse and every structural impulse are solved in
// one KKT state.  No sequential projection and no mean endpoint velocity are
// part of this API.

const DEFAULT_OPTIONS = Object.freeze({
  symmetryTolerance: 1e-11,
  pivotTolerance: 1e-12,
  feasibilityTolerance: 2e-10,
  complementarityTolerance: 2e-9,
  activeImpulseTolerance: 1e-10,
  velocityTolerance: 1e-11,
  lambdaAbsoluteTolerance: 1e-13,
  lambdaRelativeTolerance: 2e-11,
  workAbsoluteTolerance: 2e-12,
  workRelativeTolerance: 2e-10,
  areaAbsoluteTolerance: 1e-15,
  areaRelativeTolerance: 2e-10,
  maximumStructuralRows: 12,
  maximumBracketIterations: 80,
  maximumRootIterations: 96,
});

function fail(message) {
  throw new Error(message);
}

function finite(value, label) {
  if (!Number.isFinite(value)) fail(`${label} must be finite`);
  return Number(value);
}

function vector(value, length, label) {
  if (!Array.isArray(value) || value.length !== length) fail(`${label} must contain ${length} numbers`);
  return value.map((entry, index) => finite(entry, `${label}[${index}]`));
}

function dot(left, right) {
  let sum = 0;
  for (let index = 0; index < left.length; index += 1) sum += left[index] * right[index];
  return sum;
}

function addScaled(target, source, scale) {
  for (let index = 0; index < target.length; index += 1) target[index] += source[index] * scale;
  return target;
}

function multiply(matrix, column) {
  return matrix.map((row) => dot(row, column));
}

function transposeMultiply(rows, column, size) {
  const result = Array(size).fill(0);
  rows.forEach((row, rowIndex) => addScaled(result, row, column[rowIndex]));
  return result;
}

function quadratic(matrix, column) {
  return dot(column, multiply(matrix, column));
}

function clonePlain(value, seen = new Map()) {
  if (value === null || typeof value !== 'object') {
    if (typeof value === 'number') finite(value, 'plain payload number');
    if (typeof value === 'function' || typeof value === 'symbol' || typeof value === 'bigint' || typeof value === 'undefined') {
      fail('geometry payload must be finite JSON-like data');
    }
    return value;
  }
  if (seen.has(value)) fail('geometry payload must not contain cycles');
  seen.set(value, true);
  if (Array.isArray(value)) {
    const result = value.map((entry) => clonePlain(entry, seen));
    seen.delete(value);
    return result;
  }
  const result = {};
  Object.keys(value).sort().forEach((key) => {
    const entry = value[key];
    if (typeof entry === 'undefined') fail('geometry payload must not contain undefined');
    result[key] = clonePlain(entry, seen);
  });
  seen.delete(value);
  return result;
}

function deepFreeze(value) {
  if (value && typeof value === 'object' && !Object.isFrozen(value)) {
    Object.freeze(value);
    Object.keys(value).forEach((key) => deepFreeze(value[key]));
  }
  return value;
}

function canonical(value) {
  if (value === null) return 'null';
  if (typeof value === 'number') {
    finite(value, 'canonical number');
    if (Object.is(value, -0)) return '0';
    return Number(value).toString();
  }
  if (typeof value === 'boolean') return value ? 'true' : 'false';
  if (typeof value === 'string') return JSON.stringify(value);
  if (Array.isArray(value)) return `[${value.map(canonical).join(',')}]`;
  if (typeof value === 'object') {
    return `{${Object.keys(value).sort().map((key) => `${JSON.stringify(key)}:${canonical(value[key])}`).join(',')}}`;
  }
  fail('only finite JSON-like values can be canonicalised');
}

function fnv1a(text) {
  let hash = 0x811c9dc5;
  for (let index = 0; index < text.length; index += 1) {
    hash ^= text.charCodeAt(index);
    hash = Math.imul(hash, 0x01000193) >>> 0;
  }
  return hash.toString(16).padStart(8, '0');
}

function solveDense(matrix, rhs, pivotTolerance) {
  const count = rhs.length;
  if (!count) return [];
  const scale = Math.max(1, ...matrix.flat().map(Math.abs));
  const augmented = matrix.map((row, index) => [...row, rhs[index]]);
  for (let column = 0; column < count; column += 1) {
    let pivot = column;
    for (let row = column + 1; row < count; row += 1) {
      if (Math.abs(augmented[row][column]) > Math.abs(augmented[pivot][column])) pivot = row;
    }
    if (Math.abs(augmented[pivot][column]) <= pivotTolerance * scale) return null;
    [augmented[column], augmented[pivot]] = [augmented[pivot], augmented[column]];
    const divisor = augmented[column][column];
    for (let index = column; index <= count; index += 1) augmented[column][index] /= divisor;
    for (let row = 0; row < count; row += 1) {
      if (row === column) continue;
      const factor = augmented[row][column];
      if (factor === 0) continue;
      for (let index = column; index <= count; index += 1) augmented[row][index] -= factor * augmented[column][index];
    }
  }
  return augmented.map((row) => row[count]);
}

function invert(matrix, pivotTolerance) {
  const count = matrix.length;
  const columns = Array.from({ length: count }, (_, column) => {
    const rhs = Array(count).fill(0);
    rhs[column] = 1;
    return solveDense(matrix, rhs, pivotTolerance);
  });
  if (columns.some((entry) => !entry)) fail('inverse mass matrix is singular');
  return Array.from({ length: count }, (_, row) => columns.map((column) => column[row]));
}

function assertSpd(matrix, options) {
  const count = matrix.length;
  if (!count || matrix.some((row) => !Array.isArray(row) || row.length !== count)) fail('Minv must be a non-empty square matrix');
  const clean = matrix.map((row, rowIndex) => row.map((entry, columnIndex) => finite(entry, `Minv[${rowIndex}][${columnIndex}]`)));
  const scale = Math.max(1, ...clean.flat().map(Math.abs));
  for (let row = 0; row < count; row += 1) {
    for (let column = row + 1; column < count; column += 1) {
      if (Math.abs(clean[row][column] - clean[column][row]) > options.symmetryTolerance * scale) fail('Minv must be symmetric');
    }
  }
  const lower = Array.from({ length: count }, () => Array(count).fill(0));
  for (let row = 0; row < count; row += 1) {
    for (let column = 0; column <= row; column += 1) {
      let value = clean[row][column];
      for (let k = 0; k < column; k += 1) value -= lower[row][k] * lower[column][k];
      if (row === column) {
        if (!(value > options.pivotTolerance * scale)) fail('Minv must be positive definite');
        lower[row][column] = Math.sqrt(value);
      } else {
        lower[row][column] = value / lower[column][column];
      }
    }
  }
  return clean;
}

function intersectNonnegativeAffine(interval, intercept, slope, options) {
  const parallel = options.pivotTolerance * Math.max(1, Math.abs(intercept));
  if (Math.abs(slope) <= parallel) {
    if (intercept < -options.feasibilityTolerance) interval.valid = false;
    return;
  }
  const crossing = -intercept / slope;
  if (slope > 0) interval.low = Math.max(interval.low, crossing);
  else interval.high = Math.min(interval.high, crossing);
  if (interval.high < interval.low - options.feasibilityTolerance) interval.valid = false;
}

function normaliseInputs(input) {
  const options = { ...DEFAULT_OPTIONS, ...(input.options || {}) };
  const minv = assertSpd(input.Minv, options);
  const dimension = minv.length;
  const qFree = vector(input.qFree, dimension, 'qFree');
  const materialRow = vector(input.materialRow, dimension, 'materialRow');
  if (!(dot(materialRow, materialRow) > options.pivotTolerance ** 2)) fail('materialRow must be non-zero');
  const rows = (input.structuralRows || []).map((entry, index) => ({
    id: String(entry.id ?? `structural-${index}`),
    row: vector(entry.row, dimension, `structuralRows[${index}].row`),
    bias: finite(entry.bias ?? 0, `structuralRows[${index}].bias`),
  }));
  if (rows.length > options.maximumStructuralRows) fail(`frictionless reference solver accepts at most ${options.maximumStructuralRows} structural rows`);
  if (new Set(rows.map((entry) => entry.id)).size !== rows.length) fail('structural row IDs must be unique');
  const specificCuttingEnergy = finite(input.specificCuttingEnergy, 'specificCuttingEnergy');
  const width = finite(input.width, 'width');
  if (!(specificCuttingEnergy > 0) || !(width > 0)) fail('specificCuttingEnergy and width must be positive');
  if (typeof input.freshArea !== 'function') fail('freshArea(lambda, trial) callback is required');
  return { options, minv, dimension, qFree, materialRow, rows, specificCuttingEnergy, width, freshArea: input.freshArea };
}

function buildProblem(input) {
  const data = normaliseInputs(input);
  const { options, minv, dimension, qFree, materialRow, rows } = data;
  const mass = invert(minv, options.pivotTolerance);
  const materialVelocitySlope = multiply(minv, materialRow);
  const structuralMatrix = rows.map((left) => rows.map((right) => dot(left.row, multiply(minv, right.row))));
  const freeConstraintIntercept = rows.map((entry) => dot(entry.row, qFree) + entry.bias);
  const freeConstraintSlopeMagnitude = rows.map((entry) => dot(entry.row, materialVelocitySlope));
  const modes = [];
  const modeCount = 2 ** rows.length;

  for (let mask = 0; mask < modeCount; mask += 1) {
    const active = Array.from({ length: rows.length }, (_, index) => index).filter((index) => mask & (2 ** index));
    const activeMatrix = active.map((row) => active.map((column) => structuralMatrix[row][column]));
    const gammaInterceptActive = solveDense(activeMatrix, active.map((index) => -freeConstraintIntercept[index]), options.pivotTolerance);
    const gammaSlopeActive = solveDense(activeMatrix, active.map((index) => freeConstraintSlopeMagnitude[index]), options.pivotTolerance);
    if (!gammaInterceptActive || !gammaSlopeActive) continue;
    const gammaIntercept = Array(rows.length).fill(0);
    const gammaSlope = Array(rows.length).fill(0);
    active.forEach((rowIndex, activeIndex) => {
      gammaIntercept[rowIndex] = gammaInterceptActive[activeIndex];
      gammaSlope[rowIndex] = gammaSlopeActive[activeIndex];
    });
    const qIntercept = qFree.slice();
    const qSlope = materialVelocitySlope.map((value) => -value);
    addScaled(qIntercept, multiply(minv, transposeMultiply(rows.map((entry) => entry.row), gammaIntercept, dimension)), 1);
    addScaled(qSlope, multiply(minv, transposeMultiply(rows.map((entry) => entry.row), gammaSlope, dimension)), 1);
    const constraintIntercept = rows.map((entry) => dot(entry.row, qIntercept) + entry.bias);
    const constraintSlope = rows.map((entry) => dot(entry.row, qSlope));
    const interval = { low: 0, high: Infinity, valid: true };
    active.forEach((index) => intersectNonnegativeAffine(interval, gammaIntercept[index], gammaSlope[index], options));
    rows.forEach((_, index) => {
      if (!active.includes(index)) intersectNonnegativeAffine(interval, constraintIntercept[index], constraintSlope[index], options);
    });
    if (!interval.valid || interval.high < Math.max(0, interval.low) - options.feasibilityTolerance) continue;
    interval.low = Math.max(0, interval.low);
    const declaredIds = active.map((index) => rows[index].id).sort();
    modes.push({
      mask, active, declaredIds, gammaIntercept, gammaSlope, qIntercept, qSlope,
      constraintIntercept, constraintSlope, interval,
      speedIntercept: dot(materialRow, qIntercept),
      speedSlope: dot(materialRow, qSlope),
    });
  }
  if (!modes.length) fail('no nonsingular structural active-set mode exists');

  function solveAt(lambda) {
    lambda = finite(lambda, 'lambda');
    if (lambda < 0) fail('lambda must be non-negative');
    const boundTolerance = options.feasibilityTolerance * Math.max(1, lambda);
    const candidates = [];
    modes.forEach((mode) => {
      if (lambda < mode.interval.low - boundTolerance || lambda > mode.interval.high + boundTolerance) return;
      const gamma = mode.gammaIntercept.map((entry, index) => entry + mode.gammaSlope[index] * lambda);
      const qPost = mode.qIntercept.map((entry, index) => entry + mode.qSlope[index] * lambda);
      const constraints = mode.constraintIntercept.map((entry, index) => entry + mode.constraintSlope[index] * lambda);
      const gammaViolation = Math.max(0, ...mode.active.map((index) => -gamma[index]));
      const constraintViolation = Math.max(0, ...constraints.map((entry) => -entry));
      const complementarityViolation = Math.max(0, ...mode.active.map((index) => Math.abs(constraints[index])));
      if (gammaViolation > options.feasibilityTolerance
        || constraintViolation > options.feasibilityTolerance
        || complementarityViolation > options.complementarityTolerance) return;
      const activeIds = mode.active
        .filter((index) => gamma[index] > options.activeImpulseTolerance)
        .map((index) => rows[index].id)
        .sort();
      const impulseNorm = Math.hypot(...gamma);
      candidates.push({ mode, gamma, qPost, constraints, activeIds, impulseNorm });
    });
    if (!candidates.length) fail(`no structural KKT mode is feasible at lambda=${lambda}`);
    candidates.sort((left, right) => {
      const leftKey = left.activeIds.join('\u0000');
      const rightKey = right.activeIds.join('\u0000');
      if (leftKey !== rightKey) return leftKey.localeCompare(rightKey);
      if (Math.abs(left.impulseNorm - right.impulseNorm) > options.feasibilityTolerance) return left.impulseNorm - right.impulseNorm;
      return left.mode.mask - right.mode.mask;
    });
    const selected = candidates[0];
    for (const candidate of candidates.slice(1)) {
      const disagreement = Math.max(...selected.qPost.map((entry, index) => Math.abs(entry - candidate.qPost[index])));
      if (disagreement > options.complementarityTolerance * 10) {
        fail(`ambiguous structural KKT state at lambda=${lambda}; deduplicate dependent rows`);
      }
    }
    const structuralImpulses = rows.map((entry, index) => ({ id: entry.id, impulse: Math.max(0, selected.gamma[index]) }));
    const materialSpeed = dot(materialRow, selected.qPost);
    return {
      lambda,
      qPost: selected.qPost,
      activeIds: selected.activeIds,
      structuralImpulses,
      structuralGaps: rows.map((entry, index) => ({ id: entry.id, value: selected.constraints[index] })),
      materialSpeed,
      kineticEnergy: 0.5 * quadratic(mass, selected.qPost),
      _mode: selected.mode,
    };
  }

  function workSegments(lambda) {
    lambda = finite(lambda, 'work lambda');
    if (lambda < 0) fail('work lambda must be non-negative');
    if (lambda === 0) return { work: 0, segments: [], transitions: 0 };
    const rawBreakpoints = [0, lambda];
    modes.forEach((mode) => {
      if (mode.interval.low > 0 && mode.interval.low < lambda) rawBreakpoints.push(mode.interval.low);
      if (Number.isFinite(mode.interval.high) && mode.interval.high > 0 && mode.interval.high < lambda) rawBreakpoints.push(mode.interval.high);
    });
    rawBreakpoints.sort((left, right) => left - right);
    const breakpoints = [];
    rawBreakpoints.forEach((entry) => {
      const previous = breakpoints.at(-1);
      const tolerance = options.lambdaAbsoluteTolerance
        + options.lambdaRelativeTolerance * Math.max(1, Math.abs(entry), Math.abs(previous ?? entry));
      if (previous == null || Math.abs(entry - previous) > tolerance) breakpoints.push(entry);
      else if (entry === lambda) breakpoints[breakpoints.length - 1] = lambda;
    });
    if (breakpoints.at(-1) !== lambda) breakpoints.push(lambda);
    const segments = [];
    let work = 0;
    for (let index = 0; index < breakpoints.length - 1; index += 1) {
      const start = breakpoints[index];
      const end = breakpoints[index + 1];
      if (!(end > start)) continue;
      const middle = start + (end - start) / 2;
      const trial = solveAt(middle);
      const speedIntercept = trial._mode.speedIntercept;
      const speedSlope = trial._mode.speedSlope;
      let positiveWork = 0;
      const speedStart = speedIntercept + speedSlope * start;
      const speedEnd = speedIntercept + speedSlope * end;
      if (speedStart > 0 && speedEnd > 0) {
        positiveWork = (speedStart + speedEnd) * (end - start) / 2;
      } else if (speedStart > 0 || speedEnd > 0) {
        const crossing = Math.max(start, Math.min(end, -speedIntercept / speedSlope));
        if (speedStart > 0) positiveWork = speedStart * (crossing - start) / 2;
        else positiveWork = speedEnd * (end - crossing) / 2;
      }
      work += positiveWork;
      const activeIds = trial.activeIds;
      const previous = segments.at(-1);
      if (previous && canonical(previous.activeIds) === canonical(activeIds)
        && Math.abs(previous.speedSlope - speedSlope) <= options.velocityTolerance) {
        previous.end = end;
        previous.speedEnd = speedEnd;
        previous.work += positiveWork;
      } else {
        segments.push({ start, end, activeIds, speedStart, speedEnd, speedSlope, work: positiveWork });
      }
    }
    return { work, segments, transitions: Math.max(0, segments.length - 1) };
  }

  return { ...data, mass, modes, solveAt, workSegments, materialCompliance: dot(materialRow, materialVelocitySlope) };
}

function normaliseFreshResult(raw) {
  const object = typeof raw === 'number' ? { area: raw, payload: null } : raw;
  if (!object || typeof object !== 'object') fail('freshArea must return a number or {area, payload}');
  const area = finite(object.area, 'fresh area');
  if (area < 0) fail('fresh area must be non-negative');
  return { area, payload: clonePlain(object.payload ?? null) };
}

function publicTrial(trial) {
  return deepFreeze({
    lambda: trial.lambda,
    qPost: trial.qPost.slice(),
    activeIds: trial.activeIds.slice(),
    structuralImpulses: trial.structuralImpulses.map((entry) => ({ ...entry })),
    materialSpeed: trial.materialSpeed,
  });
}

function bindingBody(prepared) {
  return {
    version: prepared.version,
    lambda: prepared.lambda,
    qPost: prepared.qPost,
    activeIds: prepared.activeIds,
    structuralImpulses: prepared.structuralImpulses,
    freshArea: prepared.freshArea,
    geometryPayload: prepared.geometryPayload,
    dissipatedWork: prepared.dissipatedWork,
    materialWork: prepared.materialWork,
  };
}

function bindingSignature(body) {
  return `material-kkt-v1-${fnv1a(canonical(body))}`;
}

function validatePreparedEvent(prepared) {
  if (!prepared || prepared.status !== 'prepared' || prepared.version !== 1) return { ok: false, reason: 'not a prepared material event' };
  try {
    const expected = bindingSignature(bindingBody(prepared));
    if (expected !== prepared.bindingSignature) return { ok: false, reason: 'prepared event binding mismatch' };
    if (!Array.isArray(prepared.qPost) || !prepared.qPost.every(Number.isFinite)) return { ok: false, reason: 'prepared qPost is invalid' };
    if (!(prepared.lambda >= 0) || !(prepared.freshArea > 0)) return { ok: false, reason: 'prepared event has no positive bound event' };
    return { ok: true };
  } catch (error) {
    return { ok: false, reason: error.message };
  }
}

function prepareMaterialEvent(input) {
  let problem;
  try {
    problem = buildProblem(input);
  } catch (error) {
    return { ok: false, status: 'invalid-input', reason: error.message };
  }
  const { options } = problem;
  let zero;
  try {
    zero = problem.solveAt(0);
  } catch (error) {
    return { ok: false, status: 'solver-domain-stop', reason: error.message };
  }
  if (!(zero.materialSpeed > options.velocityTolerance)) {
    return { ok: false, status: 'non-compressive-event', reason: 'material row has no positive cutting speed after the simultaneous structural solve' };
  }
  let upper = Math.max(1e-12, zero.materialSpeed / Math.max(problem.materialCompliance, options.pivotTolerance) / 8);
  let upperTrial = null;
  try {
    for (let iteration = 0; iteration < options.maximumBracketIterations; iteration += 1) {
      upperTrial = problem.solveAt(upper);
      if (upperTrial.materialSpeed <= options.velocityTolerance) break;
      upper *= 2;
    }
  } catch (error) {
    return { ok: false, status: 'solver-domain-stop', reason: error.message };
  }
  if (!upperTrial || upperTrial.materialSpeed > options.velocityTolerance) {
    return { ok: false, status: 'solver-domain-stop', reason: 'no finite material stopping impulse could be bracketed' };
  }
  let stopLow = 0;
  let stopHigh = upper;
  try {
    for (let iteration = 0; iteration < options.maximumRootIterations; iteration += 1) {
      const middle = stopLow + (stopHigh - stopLow) / 2;
      const trial = problem.solveAt(middle);
      if (trial.materialSpeed > options.velocityTolerance) stopLow = middle;
      else stopHigh = middle;
      const tolerance = options.lambdaAbsoluteTolerance
        + options.lambdaRelativeTolerance * Math.max(1, stopHigh);
      if (stopHigh - stopLow <= tolerance) break;
    }
  } catch (error) {
    return { ok: false, status: 'solver-domain-stop', reason: error.message };
  }
  const stoppingLambda = stopHigh;

  const freshCache = new Map();
  function evaluateFresh(trial) {
    const key = Number(trial.lambda).toString();
    if (freshCache.has(key)) return freshCache.get(key);
    const value = normaliseFreshResult(problem.freshArea(trial.lambda, publicTrial(trial)));
    freshCache.set(key, value);
    return value;
  }
  function evaluate(lambda) {
    const trial = problem.solveAt(lambda);
    const work = problem.workSegments(lambda);
    const fresh = evaluateFresh(trial);
    const materialWork = problem.specificCuttingEnergy * problem.width * fresh.area;
    return { lambda, trial, work, fresh, materialWork, residual: work.work - materialWork };
  }

  let lowEval;
  let highEval;
  try {
    lowEval = evaluate(0);
    highEval = evaluate(stoppingLambda);
  } catch (error) {
    return { ok: false, status: 'solver-domain-stop', reason: error.message };
  }
  const areaTolerance = options.areaAbsoluteTolerance
    + options.areaRelativeTolerance * Math.max(1, lowEval.fresh.area, highEval.fresh.area);
  if (!(lowEval.fresh.area > areaTolerance)) {
    return { ok: false, status: 'zero-fresh-area', reason: 'this event slice contains no positive virgin area at zero resisting impulse' };
  }
  if (highEval.fresh.area > lowEval.fresh.area + areaTolerance) {
    return { ok: false, status: 'solver-domain-stop', reason: 'freshArea grew with resisting impulse; refine the causal event slice' };
  }
  const highWorkTolerance = options.workAbsoluteTolerance
    + options.workRelativeTolerance * Math.max(1, highEval.materialWork, highEval.work.work);
  if (highEval.residual < -highWorkTolerance) {
    const maximumArea = highEval.work.work / (problem.specificCuttingEnergy * problem.width);
    const suggestedAreaFraction = highEval.fresh.area > 0 ? Math.max(0, Math.min(1, maximumArea / highEval.fresh.area)) : 0;
    return deepFreeze({
      ok: false,
      status: 'unaffordable-slice',
      reason: 'the stopping impulse cannot pay for this virgin-area slice',
      retryableBySliceReduction: true,
      stoppingLambda,
      maximumDissipatableWork: highEval.work.work,
      requiredMaterialWork: highEval.materialWork,
      freshAreaAtZero: lowEval.fresh.area,
      freshAreaAtStopping: highEval.fresh.area,
      maximumAffordableFreshArea: maximumArea,
      suggestedMaximumAreaFraction: suggestedAreaFraction,
      workSegments: highEval.work.segments,
    });
  }

  let accepted = highEval;
  try {
    for (let iteration = 0; iteration < options.maximumRootIterations; iteration += 1) {
      const middle = lowEval.lambda + (highEval.lambda - lowEval.lambda) / 2;
      const middleEval = evaluate(middle);
      const localAreaTolerance = options.areaAbsoluteTolerance
        + options.areaRelativeTolerance * Math.max(1, lowEval.fresh.area, middleEval.fresh.area, highEval.fresh.area);
      if (middleEval.fresh.area > lowEval.fresh.area + localAreaTolerance
        || middleEval.fresh.area < highEval.fresh.area - localAreaTolerance) {
        return { ok: false, status: 'solver-domain-stop', reason: 'freshArea is not non-increasing over the resisting-impulse root bracket' };
      }
      if (middleEval.residual >= 0) highEval = middleEval;
      else lowEval = middleEval;
      accepted = highEval;
      const lambdaTolerance = options.lambdaAbsoluteTolerance
        + options.lambdaRelativeTolerance * Math.max(1, accepted.lambda);
      const workTolerance = options.workAbsoluteTolerance
        + options.workRelativeTolerance * Math.max(1, accepted.materialWork, accepted.work.work);
      if (highEval.lambda - lowEval.lambda <= lambdaTolerance && Math.abs(accepted.residual) <= workTolerance) break;
    }
  } catch (error) {
    return { ok: false, status: 'solver-domain-stop', reason: error.message };
  }
  const finalWorkTolerance = options.workAbsoluteTolerance
    + options.workRelativeTolerance * Math.max(1, accepted.materialWork, accepted.work.work);
  if (Math.abs(accepted.residual) > finalWorkTolerance) {
    return { ok: false, status: 'solver-domain-stop', reason: 'material work root did not converge; refine the event slice' };
  }
  let repeatedFresh;
  try {
    repeatedFresh = normaliseFreshResult(problem.freshArea(accepted.lambda, publicTrial(accepted.trial)));
  } catch (error) {
    return { ok: false, status: 'solver-domain-stop', reason: error.message };
  }
  if (canonical(repeatedFresh) !== canonical(accepted.fresh)) {
    return { ok: false, status: 'solver-domain-stop', reason: 'freshArea callback is not deterministic at the accepted impulse' };
  }

  const kineticEnergyFree = 0.5 * quadratic(problem.mass, problem.qFree);
  const kineticEnergyAtZero = zero.kineticEnergy;
  const kineticEnergyPost = accepted.trial.kineticEnergy;
  const zeroBias = problem.rows.every((entry) => entry.bias === 0);
  const energyDropFromZero = kineticEnergyAtZero - kineticEnergyPost;
  const energyIdentityResidual = zeroBias ? energyDropFromZero - accepted.work.work : null;
  const prepared = {
    ok: true,
    status: 'prepared',
    version: 1,
    lambda: accepted.lambda,
    qPost: accepted.trial.qPost.slice(),
    activeIds: accepted.trial.activeIds.slice(),
    structuralImpulses: accepted.trial.structuralImpulses.map((entry) => ({ ...entry })),
    freshArea: accepted.fresh.area,
    geometryPayload: accepted.fresh.payload,
    dissipatedWork: accepted.work.work,
    materialWork: accepted.materialWork,
    workResidual: accepted.residual,
    stoppingLambda,
    materialSpeedPost: accepted.trial.materialSpeed,
    workSegments: accepted.work.segments.map((entry) => ({ ...entry, activeIds: entry.activeIds.slice() })),
    activeSetTransitions: accepted.work.transitions,
    energyAudit: {
      kineticEnergyFree,
      kineticEnergyAtZero,
      kineticEnergyPost,
      nonIncreasingFromFree: kineticEnergyPost <= kineticEnergyFree + finalWorkTolerance,
      zeroBias,
      energyDropFromZero,
      materialWorkIdentityResidual: energyIdentityResidual,
    },
    model: 'frictionless-structural-unilateral-plus-one-material-row',
  };
  prepared.bindingSignature = bindingSignature(bindingBody(prepared));
  return deepFreeze(prepared);
}

function compareObservedBinding(observed, prepared, tolerance = 2e-11) {
  if (!observed || typeof observed !== 'object') return 'transaction observe() returned no binding state';
  if (observed.bindingSignature !== prepared.bindingSignature) return 'committed binding signature differs from the prepared event';
  if (Math.abs(Number(observed.lambda) - prepared.lambda) > tolerance * Math.max(1, prepared.lambda)) return 'committed lambda differs from the prepared root';
  if (!Array.isArray(observed.qPost) || observed.qPost.length !== prepared.qPost.length
    || observed.qPost.some((entry, index) => Math.abs(entry - prepared.qPost[index]) > tolerance * Math.max(1, Math.abs(prepared.qPost[index])))) {
    return 'committed qPost differs from the prepared KKT state';
  }
  if (canonical(observed.activeIds || []) !== canonical(prepared.activeIds)) return 'committed active IDs differ from the prepared KKT state';
  return null;
}

function commitPreparedEvent(prepared, transaction) {
  const validity = validatePreparedEvent(prepared);
  if (!validity.ok) return { ok: false, status: 'binding-refused', reason: validity.reason, rolledBack: false };
  if (!transaction || typeof transaction.snapshot !== 'function' || typeof transaction.restore !== 'function'
    || typeof transaction.applyBoundState !== 'function') {
    return { ok: false, status: 'invalid-transaction', reason: 'snapshot, restore and applyBoundState are required', rolledBack: false };
  }
  let snapshot;
  let beforeBytes;
  try {
    snapshot = clonePlain(transaction.snapshot());
    beforeBytes = canonical(snapshot);
  } catch (error) {
    return { ok: false, status: 'snapshot-failed', reason: error.message, rolledBack: false };
  }
  try {
    const boundState = deepFreeze(clonePlain({
      lambda: prepared.lambda,
      qPost: prepared.qPost,
      activeIds: prepared.activeIds,
      structuralImpulses: prepared.structuralImpulses,
      freshArea: prepared.freshArea,
      geometryPayload: prepared.geometryPayload,
      dissipatedWork: prepared.dissipatedWork,
      materialWork: prepared.materialWork,
      bindingSignature: prepared.bindingSignature,
    }));
    transaction.applyBoundState(boundState);
    if (typeof transaction.observe === 'function') {
      const mismatch = compareObservedBinding(transaction.observe(), prepared);
      if (mismatch) fail(mismatch);
    }
    return { ok: true, status: 'committed', bindingSignature: prepared.bindingSignature, rolledBack: false };
  } catch (error) {
    let rolledBack = false;
    let rollbackExact = false;
    let rollbackReason = null;
    try {
      transaction.restore(clonePlain(snapshot));
      rolledBack = true;
      rollbackExact = canonical(clonePlain(transaction.snapshot())) === beforeBytes;
      if (!rollbackExact) rollbackReason = 'transaction restore was not byte-stable in canonical state';
    } catch (rollbackError) {
      rollbackReason = rollbackError.message;
    }
    return {
      ok: false,
      status: rollbackExact ? 'commit-failed-rolled-back' : 'rollback-mismatch',
      reason: error.message,
      rolledBack,
      rollbackExact,
      rollbackReason,
    };
  }
}

function solvePrescribedImpulse(input, lambda) {
  try {
    const problem = buildProblem(input);
    const trial = problem.solveAt(lambda);
    const work = problem.workSegments(lambda);
    return deepFreeze({
      ok: true,
      lambda: trial.lambda,
      qPost: trial.qPost.slice(),
      activeIds: trial.activeIds.slice(),
      structuralImpulses: trial.structuralImpulses.map((entry) => ({ ...entry })),
      structuralGaps: trial.structuralGaps.map((entry) => ({ ...entry })),
      materialSpeed: trial.materialSpeed,
      dissipatedWork: work.work,
      workSegments: work.segments.map((entry) => ({ ...entry, activeIds: entry.activeIds.slice() })),
    });
  } catch (error) {
    return { ok: false, reason: error.message };
  }
}

module.exports = {
  DEFAULT_OPTIONS,
  prepareMaterialEvent,
  solvePrescribedImpulse,
  validatePreparedEvent,
  commitPreparedEvent,
  _reference: { canonical, buildProblem },
};

