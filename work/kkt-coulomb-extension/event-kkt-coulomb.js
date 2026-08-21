'use strict';

// Work-only reference solver for one material event coupled to structural
// unilateral contacts with 2-D Coulomb friction.
//
// Sign convention:
//   normalRow*q + normalBias >= 0
//   normal impulse = +normalRow^T*pn, pn >= 0
//   tangent impulse = +tangentRow^T*pt, |pt| <= mu*pn
//   material speed = materialRow*q > 0 while cutting
//   material impulse = -materialRow^T*p, p >= 0
//
// Every structural contact owns exactly one public point. Its normal and
// tangent rows are inseparable fields of that point record. The material row
// has no tangential law in this prototype: omission is recorded as an explicit
// zero-friction choice rather than inheriting a Rapier/default coefficient.

const DEFAULT_OPTIONS = Object.freeze({
  symmetryTolerance: 1e-11,
  pivotTolerance: 1e-12,
  feasibilityTolerance: 2e-10,
  complementarityTolerance: 2e-9,
  coneTolerance: 2e-9,
  velocityTolerance: 1e-11,
  activeImpulseTolerance: 1e-10,
  energyTolerance: 3e-10,
  lambdaAbsoluteTolerance: 1e-13,
  lambdaRelativeTolerance: 2e-11,
  workAbsoluteTolerance: 2e-12,
  workRelativeTolerance: 2e-10,
  areaAbsoluteTolerance: 1e-15,
  areaRelativeTolerance: 2e-10,
  maximumContacts: 6,
  maximumModeCandidates: 4096,
  maximumEnvelopePairs: 2000000,
  maximumBracketIterations: 80,
  maximumRootIterations: 96,
});

const ALLOWED_STRUCTURAL_ROLES = new Set(['fork-target', 'target-floor', 'fork-floor']);
const MODE = Object.freeze({ INACTIVE: 0, STICK: 1, POSITIVE_SLIDE: 2, NEGATIVE_SLIDE: 3 });
const MODE_NAMES = Object.freeze(['inactive', 'stick', '+slide', '-slide']);

class InvalidInputError extends Error {}
class SolverDomainError extends Error {}

function invalid(message) { throw new InvalidInputError(message); }
function domain(message) { throw new SolverDomainError(message); }
function finite(value, label) {
  if (!Number.isFinite(value)) invalid(`${label} must be finite`);
  return Number(value);
}
function vector(value, length, label) {
  if (!Array.isArray(value) || value.length !== length) invalid(`${label} must contain ${length} numbers`);
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
function multiply(matrix, column) { return matrix.map((row) => dot(row, column)); }
function quadratic(matrix, column) { return dot(column, multiply(matrix, column)); }
function affine(intercept, slope, parameter) { return intercept + slope * parameter; }
function affineVector(intercept, slope, parameter) {
  return intercept.map((entry, index) => entry + slope[index] * parameter);
}

function clonePlain(value, seen = new Map()) {
  if (value === null || typeof value !== 'object') {
    if (typeof value === 'number') finite(value, 'plain payload number');
    if (typeof value === 'function' || typeof value === 'symbol' || typeof value === 'bigint' || typeof value === 'undefined') {
      invalid('geometry payload must be finite JSON-like data');
    }
    return value;
  }
  if (seen.has(value)) invalid('geometry payload must not contain cycles');
  seen.set(value, true);
  if (Array.isArray(value)) {
    const result = value.map((entry) => clonePlain(entry, seen));
    seen.delete(value);
    return result;
  }
  const result = {};
  Object.keys(value).sort().forEach((key) => {
    if (typeof value[key] === 'undefined') invalid('geometry payload must not contain undefined');
    result[key] = clonePlain(value[key], seen);
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
    return Object.is(value, -0) ? '0' : Number(value).toString();
  }
  if (typeof value === 'boolean') return value ? 'true' : 'false';
  if (typeof value === 'string') return JSON.stringify(value);
  if (Array.isArray(value)) return `[${value.map(canonical).join(',')}]`;
  if (typeof value === 'object') {
    return `{${Object.keys(value).sort().map((key) => `${JSON.stringify(key)}:${canonical(value[key])}`).join(',')}}`;
  }
  invalid('only finite JSON-like values can be canonicalised');
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
  if (columns.some((entry) => !entry)) invalid('inverse mass matrix is singular');
  return Array.from({ length: count }, (_, row) => columns.map((column) => column[row]));
}

function assertSpd(matrix, options) {
  const count = matrix?.length || 0;
  if (!count || matrix.some((row) => !Array.isArray(row) || row.length !== count)) invalid('Minv must be a non-empty square matrix');
  const clean = matrix.map((row, rowIndex) => row.map((entry, columnIndex) => finite(entry, `Minv[${rowIndex}][${columnIndex}]`)));
  const scale = Math.max(1, ...clean.flat().map(Math.abs));
  for (let row = 0; row < count; row += 1) {
    for (let column = row + 1; column < count; column += 1) {
      if (Math.abs(clean[row][column] - clean[column][row]) > options.symmetryTolerance * scale) invalid('Minv must be symmetric');
    }
  }
  const lower = Array.from({ length: count }, () => Array(count).fill(0));
  for (let row = 0; row < count; row += 1) {
    for (let column = 0; column <= row; column += 1) {
      let value = clean[row][column];
      for (let inner = 0; inner < column; inner += 1) value -= lower[row][inner] * lower[column][inner];
      if (row === column) {
        if (!(value > options.pivotTolerance * scale)) invalid('Minv must be positive definite');
        lower[row][column] = Math.sqrt(value);
      } else lower[row][column] = value / lower[column][column];
    }
  }
  return clean;
}

function normalisePoint(raw, label) {
  if (!raw || typeof raw !== 'object') invalid(`${label} is required`);
  return { x: finite(raw.x, `${label}.x`), y: finite(raw.y, `${label}.y`) };
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

function classifyError(error) {
  return error instanceof InvalidInputError ? 'invalid-input' : 'solver-domain-stop';
}

function normaliseInputs(input) {
  const options = { ...DEFAULT_OPTIONS, ...(input.options || {}) };
  const minv = assertSpd(input.Minv, options);
  const dimension = minv.length;
  const qFree = vector(input.qFree, dimension, 'qFree');
  const material = input.materialContact;
  if (!material || typeof material !== 'object') invalid('materialContact is required');
  const materialId = String(material.id ?? 'material-normal');
  const materialPoint = normalisePoint(material.point, 'materialContact.point');
  const materialRow = vector(material.normalRow, dimension, 'materialContact.normalRow');
  if (!(dot(materialRow, materialRow) > options.pivotTolerance ** 2)) invalid('materialContact.normalRow must be non-zero');
  if (Object.hasOwn(material, 'tangentRow') || Object.hasOwn(material, 'mu') || Object.hasOwn(material, 'friction')) {
    domain('material tangential friction is not defined by this prototype; omit tangentRow/mu/friction to request explicit zero material friction');
  }
  const materialFriction = Object.freeze({ defined: false, coefficient: 0, mode: 'none-explicit' });

  const contacts = (input.structuralContacts || []).map((entry, index) => {
    const role = String(entry.role ?? '');
    if (!ALLOWED_STRUCTURAL_ROLES.has(role)) invalid(`structuralContacts[${index}].role must be fork-target, target-floor, or fork-floor`);
    if (!Object.hasOwn(entry, 'mu')) invalid(`structuralContacts[${index}].mu must be supplied explicitly`);
    const mu = finite(entry.mu, `structuralContacts[${index}].mu`);
    if (mu < 0) invalid(`structuralContacts[${index}].mu must be non-negative`);
    const normalRow = vector(entry.normalRow, dimension, `structuralContacts[${index}].normalRow`);
    const tangentRow = vector(entry.tangentRow, dimension, `structuralContacts[${index}].tangentRow`);
    if (!(dot(normalRow, normalRow) > options.pivotTolerance ** 2)) invalid(`structuralContacts[${index}].normalRow must be non-zero`);
    if (!(dot(tangentRow, tangentRow) > options.pivotTolerance ** 2)) invalid(`structuralContacts[${index}].tangentRow must be non-zero`);
    return {
      id: String(entry.id ?? `${role}:${index}`), role,
      point: normalisePoint(entry.point, `structuralContacts[${index}].point`),
      normalRow, tangentRow, mu,
      normalBias: finite(entry.normalBias ?? 0, `structuralContacts[${index}].normalBias`),
    };
  });
  if (new Set(contacts.map((entry) => entry.id)).size !== contacts.length) invalid('structural contact IDs must be unique');
  if (contacts.length > options.maximumContacts) {
    domain(`contact count ${contacts.length} exceeds explicit maximumContacts=${options.maximumContacts}`);
  }
  const theoreticalCandidateCount = 4 ** contacts.length;
  if (!Number.isSafeInteger(theoreticalCandidateCount) || theoreticalCandidateCount > options.maximumModeCandidates) {
    domain(`Coulomb mode candidate upper bound ${theoreticalCandidateCount} exceeds maximumModeCandidates=${options.maximumModeCandidates}`);
  }
  const specificCuttingEnergy = finite(input.specificCuttingEnergy, 'specificCuttingEnergy');
  const width = finite(input.width, 'width');
  if (!(specificCuttingEnergy > 0) || !(width > 0)) invalid('specificCuttingEnergy and width must be positive');
  if (typeof input.freshArea !== 'function') invalid('freshArea(p, trial) callback is required');
  const pointBindings = [
    { id: materialId, role: 'material-normal', point: materialPoint, normalRow: materialRow, friction: materialFriction },
    ...contacts.map((entry) => ({
      id: entry.id, role: entry.role, point: entry.point,
      normalRow: entry.normalRow, tangentRow: entry.tangentRow, mu: entry.mu,
    })),
  ];
  return {
    options, minv, dimension, qFree, materialId, materialPoint, materialRow, materialFriction,
    contacts, theoreticalCandidateCount, specificCuttingEnergy, width, freshArea: input.freshArea,
    pointBindings,
  };
}

function kineticCoefficients(mass, qIntercept, qSlope) {
  return {
    constant: 0.5 * quadratic(mass, qIntercept),
    linear: dot(qIntercept, multiply(mass, qSlope)),
    quadratic: 0.5 * quadratic(mass, qSlope),
  };
}
function polynomial(coefficients, parameter) {
  return coefficients.constant + coefficients.linear * parameter + coefficients.quadratic * parameter * parameter;
}
function differenceCoefficients(left, right) {
  return {
    constant: left.constant - right.constant,
    linear: left.linear - right.linear,
    quadratic: left.quadratic - right.quadratic,
  };
}
function rootsInInterval(coefficients, low, high, tolerance) {
  const { constant, linear, quadratic: square } = coefficients;
  const scale = Math.max(1, Math.abs(constant), Math.abs(linear), Math.abs(square));
  const roots = [];
  if (Math.abs(square) <= tolerance * scale) {
    if (Math.abs(linear) > tolerance * scale) roots.push(-constant / linear);
  } else {
    const discriminant = linear * linear - 4 * square * constant;
    const discriminantTolerance = tolerance * scale * scale;
    if (discriminant >= -discriminantTolerance) {
      const rootDiscriminant = Math.sqrt(Math.max(0, discriminant));
      const q = -0.5 * (linear + (linear >= 0 ? rootDiscriminant : -rootDiscriminant));
      if (q === 0) roots.push(-linear / (2 * square));
      else {
        roots.push(q / square);
        if (rootDiscriminant > discriminantTolerance) roots.push(constant / q);
      }
    }
  }
  return roots.filter((root) => Number.isFinite(root) && root > low && root < high);
}

function buildProblem(input) {
  const data = normaliseInputs(input);
  const { options, minv, dimension, qFree, materialRow, contacts } = data;
  const mass = invert(minv, options.pivotTolerance);
  const materialVelocitySlope = multiply(minv, materialRow);
  const qBaseIntercept = qFree.slice();
  const qBaseSlope = materialVelocitySlope.map((value) => -value);
  const kineticEnergyFree = 0.5 * quadratic(mass, qFree);
  const modes = [];

  for (let ordinal = 0; ordinal < data.theoreticalCandidateCount; ordinal += 1) {
    let digits = ordinal;
    const states = contacts.map(() => {
      const state = digits % 4;
      digits = Math.floor(digits / 4);
      return state;
    });
    const unknownRows = [];
    const equations = [];
    const equationBiases = [];
    const normalUnknown = new Map();
    const tangentUnknown = new Map();
    contacts.forEach((contact, index) => {
      const state = states[index];
      if (state === MODE.INACTIVE) return;
      const slideSign = state === MODE.POSITIVE_SLIDE ? 1 : state === MODE.NEGATIVE_SLIDE ? -1 : 0;
      const effectiveNormal = contact.normalRow.map((value, axis) => value + slideSign * contact.mu * contact.tangentRow[axis]);
      normalUnknown.set(index, unknownRows.length);
      unknownRows.push(effectiveNormal);
      equations.push(contact.normalRow);
      equationBiases.push(contact.normalBias);
    });
    contacts.forEach((contact, index) => {
      if (states[index] !== MODE.STICK) return;
      tangentUnknown.set(index, unknownRows.length);
      unknownRows.push(contact.tangentRow);
      equations.push(contact.tangentRow);
      equationBiases.push(0);
    });
    const system = equations.map((equation) => unknownRows.map((unknown) => dot(equation, multiply(minv, unknown))));
    const rhsIntercept = equations.map((equation, index) => -dot(equation, qBaseIntercept) - equationBiases[index]);
    const rhsSlope = equations.map((equation) => -dot(equation, qBaseSlope));
    const impulseIntercept = solveDense(system, rhsIntercept, options.pivotTolerance);
    const impulseSlope = solveDense(system, rhsSlope, options.pivotTolerance);
    if (!impulseIntercept || !impulseSlope) continue;

    const qIntercept = qBaseIntercept.slice();
    const qSlope = qBaseSlope.slice();
    unknownRows.forEach((row, index) => {
      const response = multiply(minv, row);
      addScaled(qIntercept, response, impulseIntercept[index]);
      addScaled(qSlope, response, impulseSlope[index]);
    });
    const pnIntercept = Array(contacts.length).fill(0);
    const pnSlope = Array(contacts.length).fill(0);
    const ptIntercept = Array(contacts.length).fill(0);
    const ptSlope = Array(contacts.length).fill(0);
    contacts.forEach((contact, index) => {
      const state = states[index];
      if (state === MODE.INACTIVE) return;
      const normalIndex = normalUnknown.get(index);
      pnIntercept[index] = impulseIntercept[normalIndex];
      pnSlope[index] = impulseSlope[normalIndex];
      if (state === MODE.STICK) {
        const tangentIndex = tangentUnknown.get(index);
        ptIntercept[index] = impulseIntercept[tangentIndex];
        ptSlope[index] = impulseSlope[tangentIndex];
      } else {
        const sign = state === MODE.POSITIVE_SLIDE ? 1 : -1;
        ptIntercept[index] = sign * contact.mu * pnIntercept[index];
        ptSlope[index] = sign * contact.mu * pnSlope[index];
      }
    });

    const normalIntercept = contacts.map((contact) => dot(contact.normalRow, qIntercept) + contact.normalBias);
    const normalSlope = contacts.map((contact) => dot(contact.normalRow, qSlope));
    const tangentIntercept = contacts.map((contact) => dot(contact.tangentRow, qIntercept));
    const tangentSlope = contacts.map((contact) => dot(contact.tangentRow, qSlope));
    const interval = { low: 0, high: Infinity, valid: true };
    contacts.forEach((contact, index) => {
      const state = states[index];
      if (state === MODE.INACTIVE) {
        intersectNonnegativeAffine(interval, normalIntercept[index], normalSlope[index], options);
        return;
      }
      intersectNonnegativeAffine(interval, pnIntercept[index], pnSlope[index], options);
      if (state === MODE.STICK) {
        intersectNonnegativeAffine(interval,
          contact.mu * pnIntercept[index] - ptIntercept[index],
          contact.mu * pnSlope[index] - ptSlope[index], options);
        intersectNonnegativeAffine(interval,
          contact.mu * pnIntercept[index] + ptIntercept[index],
          contact.mu * pnSlope[index] + ptSlope[index], options);
      } else {
        const sign = state === MODE.POSITIVE_SLIDE ? 1 : -1;
        intersectNonnegativeAffine(interval, -sign * tangentIntercept[index], -sign * tangentSlope[index], options);
      }
    });
    if (!interval.valid || interval.high < Math.max(0, interval.low) - options.feasibilityTolerance) continue;
    interval.low = Math.max(0, interval.low);
    const kinetic = kineticCoefficients(mass, qIntercept, qSlope);
    const impulseNorm = { constant: 0, linear: 0, quadratic: 0 };
    for (let index = 0; index < contacts.length; index += 1) {
      impulseNorm.constant += pnIntercept[index] ** 2 + ptIntercept[index] ** 2;
      impulseNorm.linear += 2 * (pnIntercept[index] * pnSlope[index] + ptIntercept[index] * ptSlope[index]);
      impulseNorm.quadratic += pnSlope[index] ** 2 + ptSlope[index] ** 2;
    }
    const modeKey = states.map((state, index) => `${contacts[index].id}:${MODE_NAMES[state]}`).join('|') || 'no-structural-contact';
    modes.push({
      ordinal, states, modeKey, interval,
      qIntercept, qSlope, pnIntercept, pnSlope, ptIntercept, ptSlope,
      normalIntercept, normalSlope, tangentIntercept, tangentSlope,
      kinetic, impulseNorm,
      materialSpeedIntercept: dot(materialRow, qIntercept),
      materialSpeedSlope: dot(materialRow, qSlope),
    });
  }
  if (!modes.length) domain('no nonsingular Coulomb KKT mode exists');

  function evaluateMode(mode, p) {
    const qPost = affineVector(mode.qIntercept, mode.qSlope, p);
    const pn = mode.pnIntercept.map((value, index) => affine(value, mode.pnSlope[index], p));
    const pt = mode.ptIntercept.map((value, index) => affine(value, mode.ptSlope[index], p));
    const normalVelocity = mode.normalIntercept.map((value, index) => affine(value, mode.normalSlope[index], p));
    const tangentVelocity = mode.tangentIntercept.map((value, index) => affine(value, mode.tangentSlope[index], p));
    const kineticEnergy = polynomial(mode.kinetic, p);
    const impulseNormSquared = Math.max(0, polynomial(mode.impulseNorm, p));
    return { mode, p, qPost, pn, pt, normalVelocity, tangentVelocity, kineticEnergy, impulseNormSquared };
  }

  function isFeasible(candidate, p) {
    const scale = Math.max(1, Math.abs(p), Math.sqrt(candidate.impulseNormSquared));
    const feasibility = options.feasibilityTolerance * scale;
    for (let index = 0; index < contacts.length; index += 1) {
      const state = candidate.mode.states[index];
      if (candidate.normalVelocity[index] < -feasibility) return false;
      if (state === MODE.INACTIVE) {
        if (Math.abs(candidate.pn[index]) > feasibility || Math.abs(candidate.pt[index]) > feasibility) return false;
        continue;
      }
      if (candidate.pn[index] < -feasibility) return false;
      if (Math.abs(candidate.normalVelocity[index]) > options.complementarityTolerance * scale) return false;
      if (Math.abs(candidate.pt[index]) > contacts[index].mu * Math.max(0, candidate.pn[index]) + options.coneTolerance * scale) return false;
      if (state === MODE.STICK) {
        if (Math.abs(candidate.tangentVelocity[index]) > options.velocityTolerance * scale) return false;
      } else {
        const sign = state === MODE.POSITIVE_SLIDE ? 1 : -1;
        if (sign * candidate.tangentVelocity[index] > options.velocityTolerance * scale) return false;
        if (Math.abs(Math.abs(candidate.pt[index]) - contacts[index].mu * Math.max(0, candidate.pn[index])) > options.coneTolerance * scale) return false;
      }
    }
    return true;
  }

  function solveAt(parameter) {
    const p = finite(parameter, 'p');
    if (p < 0) invalid('p must be non-negative');
    const boundTolerance = options.feasibilityTolerance * Math.max(1, p);
    const candidates = [];
    modes.forEach((mode) => {
      if (p < mode.interval.low - boundTolerance || p > mode.interval.high + boundTolerance) return;
      const candidate = evaluateMode(mode, p);
      if (isFeasible(candidate, p)) candidates.push(candidate);
    });
    if (!candidates.length) domain(`no feasible Coulomb KKT mode at p=${p}`);
    candidates.sort((left, right) => {
      if (left.kineticEnergy !== right.kineticEnergy) return left.kineticEnergy - right.kineticEnergy;
      if (left.impulseNormSquared !== right.impulseNormSquared) return left.impulseNormSquared - right.impulseNormSquared;
      return left.mode.modeKey < right.mode.modeKey ? -1 : left.mode.modeKey > right.mode.modeKey ? 1 : 0;
    });
    const selected = candidates[0];
    const energyTolerance = options.energyTolerance * Math.max(1, kineticEnergyFree, selected.kineticEnergy);
    if (selected.kineticEnergy > kineticEnergyFree + energyTolerance) {
      domain(`all feasible Coulomb modes increase kinetic energy at p=${p}`);
    }
    const contactImpulses = contacts.map((contact, index) => ({
      id: contact.id, role: contact.role, point: { ...contact.point }, mode: MODE_NAMES[selected.mode.states[index]],
      normalImpulse: Math.max(0, selected.pn[index]), tangentImpulse: selected.pt[index], mu: contact.mu,
      normalVelocityPost: selected.normalVelocity[index], tangentVelocityPost: selected.tangentVelocity[index],
      coneResidual: Math.max(0, Math.abs(selected.pt[index]) - contact.mu * Math.max(0, selected.pn[index])),
      maximumDissipationSignResidual: selected.mode.states[index] === MODE.STICK ? Math.abs(selected.tangentVelocity[index])
        : Math.max(0, (selected.mode.states[index] === MODE.POSITIVE_SLIDE ? 1 : -1) * selected.tangentVelocity[index]),
    }));
    const activeIds = contactImpulses.filter((entry) => entry.normalImpulse > options.activeImpulseTolerance).map((entry) => entry.id).sort();
    return {
      p, lambda: p, qPost: selected.qPost,
      modeKey: selected.mode.modeKey,
      contactStates: contactImpulses.map((entry) => ({ id: entry.id, mode: entry.mode })),
      contactImpulses, activeIds,
      materialSpeed: dot(materialRow, selected.qPost),
      kineticEnergy: selected.kineticEnergy,
      kineticEnergyFree,
      maximumDissipation: {
        selectedMinimumKineticEnergy: selected.kineticEnergy,
        feasibleCandidateCount: candidates.length,
        candidateKineticEnergies: candidates.map((entry) => ({ modeKey: entry.mode.modeKey, kineticEnergy: entry.kineticEnergy })),
      },
      _mode: selected.mode,
    };
  }

  function addBreakpoint(list, value, upper) {
    if (Number.isFinite(value) && value > 0 && value < upper) list.push(value);
  }

  function workSegments(parameter) {
    const p = finite(parameter, 'work p');
    if (p < 0) invalid('work p must be non-negative');
    if (p === 0) {
      const zero = solveAt(0);
      return { work: 0, segments: [], transitions: 0, kineticEnergyAtZero: zero.kineticEnergy,
        kineticEnergyPost: zero.kineticEnergy, structuralFrictionDissipation: 0 };
    }
    const breakpoints = [0, p];
    const relevantModes = modes.filter((mode) => mode.interval.high >= 0 && mode.interval.low <= p);
    relevantModes.forEach((mode) => {
      addBreakpoint(breakpoints, mode.interval.low, p);
      addBreakpoint(breakpoints, mode.interval.high, p);
      rootsInInterval(differenceCoefficients(mode.kinetic, { constant: kineticEnergyFree, linear: 0, quadratic: 0 }),
        Math.max(0, mode.interval.low), Math.min(p, mode.interval.high), options.pivotTolerance)
        .forEach((root) => addBreakpoint(breakpoints, root, p));
    });
    const pairCount = relevantModes.length * (relevantModes.length - 1) / 2;
    if (pairCount > options.maximumEnvelopePairs) {
      domain(`energy-envelope pair upper bound ${pairCount} exceeds maximumEnvelopePairs=${options.maximumEnvelopePairs}`);
    }
    for (let leftIndex = 0; leftIndex < relevantModes.length; leftIndex += 1) {
      const left = relevantModes[leftIndex];
      for (let rightIndex = leftIndex + 1; rightIndex < relevantModes.length; rightIndex += 1) {
        const right = relevantModes[rightIndex];
        const low = Math.max(0, left.interval.low, right.interval.low);
        const high = Math.min(p, left.interval.high, right.interval.high);
        if (!(high > low)) continue;
        const energyDifference = differenceCoefficients(left.kinetic, right.kinetic);
        const energyScale = Math.max(1, Math.abs(energyDifference.constant), Math.abs(energyDifference.linear), Math.abs(energyDifference.quadratic));
        if (Math.max(Math.abs(energyDifference.constant), Math.abs(energyDifference.linear), Math.abs(energyDifference.quadratic)) <= options.pivotTolerance * energyScale) {
          rootsInInterval(differenceCoefficients(left.impulseNorm, right.impulseNorm), low, high, options.pivotTolerance)
            .forEach((root) => addBreakpoint(breakpoints, root, p));
        } else {
          rootsInInterval(energyDifference, low, high, options.pivotTolerance)
            .forEach((root) => addBreakpoint(breakpoints, root, p));
        }
      }
    }
    breakpoints.sort((left, right) => left - right);
    const unique = [];
    breakpoints.forEach((entry) => {
      const previous = unique.at(-1);
      const tolerance = options.lambdaAbsoluteTolerance
        + options.lambdaRelativeTolerance * Math.max(1, Math.abs(entry), Math.abs(previous ?? entry));
      if (previous == null || Math.abs(entry - previous) > tolerance) unique.push(entry);
      else if (entry === p) unique[unique.length - 1] = p;
    });
    if (unique[0] !== 0) unique.unshift(0);
    if (unique.at(-1) !== p) unique.push(p);

    const segments = [];
    let work = 0;
    for (let index = 0; index < unique.length - 1; index += 1) {
      const start = unique[index];
      const end = unique[index + 1];
      if (!(end > start)) continue;
      const middle = start + (end - start) / 2;
      const trial = solveAt(middle);
      const mode = trial._mode;
      for (const fraction of [0.25, 0.75]) {
        const probe = solveAt(start + (end - start) * fraction);
        if (probe.modeKey !== trial.modeKey) domain('maximum-dissipation envelope changed inside an unsplit work interval');
      }
      const speedStart = affine(mode.materialSpeedIntercept, mode.materialSpeedSlope, start);
      const speedEnd = affine(mode.materialSpeedIntercept, mode.materialSpeedSlope, end);
      let positiveWork = 0;
      if (speedStart > 0 && speedEnd > 0) positiveWork = (speedStart + speedEnd) * (end - start) / 2;
      else if (speedStart > 0 || speedEnd > 0) {
        const crossing = Math.max(start, Math.min(end, -mode.materialSpeedIntercept / mode.materialSpeedSlope));
        positiveWork = speedStart > 0 ? speedStart * (crossing - start) / 2 : speedEnd * (end - crossing) / 2;
      }
      const energyStart = polynomial(mode.kinetic, start);
      const energyEnd = polynomial(mode.kinetic, end);
      const entry = {
        start, end, modeKey: mode.modeKey,
        contactStates: mode.states.map((state, contactIndex) => ({ id: contacts[contactIndex].id, mode: MODE_NAMES[state] })),
        speedStart, speedEnd, speedSlope: mode.materialSpeedSlope,
        work: positiveWork, kineticEnergyStart: energyStart, kineticEnergyEnd: energyEnd,
        structuralFrictionDissipation: energyStart - energyEnd - positiveWork,
      };
      work += positiveWork;
      const previous = segments.at(-1);
      if (previous && previous.modeKey === entry.modeKey
        && Math.abs(previous.speedSlope - entry.speedSlope) <= options.velocityTolerance) {
        previous.end = entry.end;
        previous.speedEnd = entry.speedEnd;
        previous.work += entry.work;
        previous.kineticEnergyEnd = entry.kineticEnergyEnd;
        previous.structuralFrictionDissipation += entry.structuralFrictionDissipation;
      } else segments.push(entry);
    }
    const zero = solveAt(0);
    const end = solveAt(p);
    return {
      work, segments, transitions: Math.max(0, segments.length - 1),
      kineticEnergyAtZero: zero.kineticEnergy,
      kineticEnergyPost: end.kineticEnergy,
      structuralFrictionDissipation: zero.kineticEnergy - end.kineticEnergy - work,
    };
  }

  return {
    ...data, mass, modes, solveAt, workSegments, kineticEnergyFree,
    materialCompliance: dot(materialRow, materialVelocitySlope),
  };
}

function normaliseFreshResult(raw) {
  const object = typeof raw === 'number' ? { area: raw, payload: null } : raw;
  if (!object || typeof object !== 'object') invalid('freshArea must return a number or {area, payload}');
  const area = finite(object.area, 'fresh area');
  if (area < 0) invalid('fresh area must be non-negative');
  return { area, payload: clonePlain(object.payload ?? null) };
}

function publicTrial(trial, problem) {
  return deepFreeze({
    p: trial.p, lambda: trial.lambda, qPost: trial.qPost.slice(), modeKey: trial.modeKey,
    activeIds: trial.activeIds.slice(),
    contactStates: trial.contactStates.map((entry) => ({ ...entry })),
    contactImpulses: trial.contactImpulses.map((entry) => ({ ...entry, point: { ...entry.point } })),
    materialPoint: { ...problem.materialPoint }, materialFriction: { ...problem.materialFriction },
  });
}

function bindingBody(prepared) {
  return {
    version: prepared.version, p: prepared.p, lambda: prepared.lambda,
    qPost: prepared.qPost, modeKey: prepared.modeKey,
    activeIds: prepared.activeIds, contactStates: prepared.contactStates,
    contactImpulses: prepared.contactImpulses, contactPointBindings: prepared.contactPointBindings,
    materialFriction: prepared.materialFriction,
    freshArea: prepared.freshArea, geometryPayload: prepared.geometryPayload,
    dissipatedWork: prepared.dissipatedWork, materialWork: prepared.materialWork,
  };
}
function bindingSignature(body) { return `material-coulomb-kkt-v2-${fnv1a(canonical(body))}`; }

function validatePreparedEvent(prepared) {
  if (!prepared || prepared.status !== 'prepared' || prepared.version !== 2) return { ok: false, reason: 'not a prepared Coulomb material event' };
  try {
    if (prepared.p !== prepared.lambda) return { ok: false, reason: 'prepared same-p binding is broken' };
    if (bindingSignature(bindingBody(prepared)) !== prepared.bindingSignature) return { ok: false, reason: 'prepared event binding mismatch' };
    if (!Array.isArray(prepared.qPost) || !prepared.qPost.every(Number.isFinite)) return { ok: false, reason: 'prepared qPost is invalid' };
    if (!(prepared.p >= 0) || !(prepared.freshArea > 0)) return { ok: false, reason: 'prepared event has no positive bound event' };
    return { ok: true };
  } catch (error) { return { ok: false, reason: error.message }; }
}

function prepareMaterialEvent(input) {
  let problem;
  try { problem = buildProblem(input); }
  catch (error) { return { ok: false, status: classifyError(error), reason: error.message }; }
  const { options } = problem;
  let zero;
  try { zero = problem.solveAt(0); }
  catch (error) { return { ok: false, status: classifyError(error), reason: error.message }; }
  if (!(zero.materialSpeed > options.velocityTolerance)) {
    return { ok: false, status: 'non-compressive-event', reason: 'material normal has no positive cutting speed after the simultaneous Coulomb KKT solve' };
  }
  let upper = Math.max(1e-12, zero.materialSpeed / Math.max(problem.materialCompliance, options.pivotTolerance) / 8);
  let upperTrial = null;
  try {
    for (let iteration = 0; iteration < options.maximumBracketIterations; iteration += 1) {
      upperTrial = problem.solveAt(upper);
      if (upperTrial.materialSpeed <= options.velocityTolerance) break;
      upper *= 2;
    }
  } catch (error) { return { ok: false, status: classifyError(error), reason: error.message }; }
  if (!upperTrial || upperTrial.materialSpeed > options.velocityTolerance) {
    return { ok: false, status: 'solver-domain-stop', reason: 'no finite material stopping impulse could be bracketed inside the Coulomb domain' };
  }
  let stopLow = 0;
  let stopHigh = upper;
  try {
    for (let iteration = 0; iteration < options.maximumRootIterations; iteration += 1) {
      const middle = stopLow + (stopHigh - stopLow) / 2;
      const trial = problem.solveAt(middle);
      if (trial.materialSpeed > options.velocityTolerance) stopLow = middle;
      else stopHigh = middle;
      const tolerance = options.lambdaAbsoluteTolerance + options.lambdaRelativeTolerance * Math.max(1, stopHigh);
      if (stopHigh - stopLow <= tolerance) break;
    }
  } catch (error) { return { ok: false, status: classifyError(error), reason: error.message }; }
  const stoppingP = stopHigh;

  const freshCache = new Map();
  function evaluateFresh(trial) {
    const key = Number(trial.p).toString();
    if (freshCache.has(key)) return freshCache.get(key);
    const result = normaliseFreshResult(problem.freshArea(trial.p, publicTrial(trial, problem)));
    freshCache.set(key, result);
    return result;
  }
  function evaluate(p) {
    const trial = problem.solveAt(p);
    const dissipation = problem.workSegments(p);
    const fresh = evaluateFresh(trial);
    const materialWork = problem.specificCuttingEnergy * problem.width * fresh.area;
    return { p, trial, dissipation, fresh, materialWork, residual: dissipation.work - materialWork };
  }

  let lowEval;
  let highEval;
  try { lowEval = evaluate(0); highEval = evaluate(stoppingP); }
  catch (error) { return { ok: false, status: classifyError(error), reason: error.message }; }
  const areaTolerance = options.areaAbsoluteTolerance
    + options.areaRelativeTolerance * Math.max(1, lowEval.fresh.area, highEval.fresh.area);
  if (!(lowEval.fresh.area > areaTolerance)) return { ok: false, status: 'zero-fresh-area', reason: 'event slice contains no positive virgin area' };
  if (highEval.fresh.area > lowEval.fresh.area + areaTolerance) {
    return { ok: false, status: 'solver-domain-stop', reason: 'freshArea grew with resisting impulse; refine the causal event slice' };
  }
  const highWorkTolerance = options.workAbsoluteTolerance
    + options.workRelativeTolerance * Math.max(1, highEval.materialWork, highEval.dissipation.work);
  if (highEval.residual < -highWorkTolerance) {
    const maximumArea = highEval.dissipation.work / (problem.specificCuttingEnergy * problem.width);
    return deepFreeze({
      ok: false, status: 'unaffordable-slice', reason: 'the stopping impulse cannot pay for this virgin-area slice',
      retryableBySliceReduction: true, stoppingP,
      maximumDissipatableWork: highEval.dissipation.work, requiredMaterialWork: highEval.materialWork,
      maximumAffordableFreshArea: maximumArea,
      suggestedMaximumAreaFraction: highEval.fresh.area > 0 ? Math.max(0, Math.min(1, maximumArea / highEval.fresh.area)) : 0,
      workSegments: highEval.dissipation.segments,
    });
  }

  let accepted = highEval;
  try {
    for (let iteration = 0; iteration < options.maximumRootIterations; iteration += 1) {
      const middle = lowEval.p + (highEval.p - lowEval.p) / 2;
      const middleEval = evaluate(middle);
      const localAreaTolerance = options.areaAbsoluteTolerance
        + options.areaRelativeTolerance * Math.max(1, lowEval.fresh.area, middleEval.fresh.area, highEval.fresh.area);
      if (middleEval.fresh.area > lowEval.fresh.area + localAreaTolerance
        || middleEval.fresh.area < highEval.fresh.area - localAreaTolerance) {
        return { ok: false, status: 'solver-domain-stop', reason: 'freshArea is not non-increasing over the same-p constitutive bracket' };
      }
      if (middleEval.residual >= 0) highEval = middleEval;
      else lowEval = middleEval;
      accepted = highEval;
      const pTolerance = options.lambdaAbsoluteTolerance + options.lambdaRelativeTolerance * Math.max(1, accepted.p);
      const workTolerance = options.workAbsoluteTolerance
        + options.workRelativeTolerance * Math.max(1, accepted.materialWork, accepted.dissipation.work);
      if (highEval.p - lowEval.p <= pTolerance && Math.abs(accepted.residual) <= workTolerance) break;
    }
  } catch (error) { return { ok: false, status: classifyError(error), reason: error.message }; }
  const finalWorkTolerance = options.workAbsoluteTolerance
    + options.workRelativeTolerance * Math.max(1, accepted.materialWork, accepted.dissipation.work);
  if (Math.abs(accepted.residual) > finalWorkTolerance) {
    return { ok: false, status: 'solver-domain-stop', reason: 'same-p material work root did not converge; refine the event slice' };
  }
  let repeatedFresh;
  try { repeatedFresh = normaliseFreshResult(problem.freshArea(accepted.p, publicTrial(accepted.trial, problem))); }
  catch (error) { return { ok: false, status: classifyError(error), reason: error.message }; }
  if (canonical(repeatedFresh) !== canonical(accepted.fresh)) {
    return { ok: false, status: 'solver-domain-stop', reason: 'freshArea callback is not deterministic at the accepted p' };
  }
  const structuralDissipation = accepted.dissipation.structuralFrictionDissipation;
  if (structuralDissipation < -finalWorkTolerance) {
    return { ok: false, status: 'solver-domain-stop', reason: 'the selected Coulomb loading path would return structural friction work' };
  }
  const prepared = {
    ok: true, status: 'prepared', version: 2,
    p: accepted.p, lambda: accepted.p,
    qPost: accepted.trial.qPost.slice(), modeKey: accepted.trial.modeKey,
    activeIds: accepted.trial.activeIds.slice(),
    contactStates: accepted.trial.contactStates.map((entry) => ({ ...entry })),
    contactImpulses: accepted.trial.contactImpulses.map((entry) => ({ ...entry, point: { ...entry.point } })),
    contactPointBindings: clonePlain(problem.pointBindings),
    materialFriction: { ...problem.materialFriction },
    freshArea: accepted.fresh.area, geometryPayload: accepted.fresh.payload,
    dissipatedWork: accepted.dissipation.work, materialWork: accepted.materialWork,
    workResidual: accepted.residual, stoppingP,
    materialSpeedPost: accepted.trial.materialSpeed,
    workSegments: accepted.dissipation.segments.map((entry) => clonePlain(entry)),
    modeTransitions: accepted.dissipation.transitions,
    maximumDissipation: clonePlain(accepted.trial.maximumDissipation),
    energyAudit: {
      kineticEnergyFree: problem.kineticEnergyFree,
      kineticEnergyAtZero: accepted.dissipation.kineticEnergyAtZero,
      kineticEnergyPost: accepted.trial.kineticEnergy,
      nonIncreasingFromFree: accepted.trial.kineticEnergy <= problem.kineticEnergyFree + finalWorkTolerance,
      materialDissipation: accepted.dissipation.work,
      structuralFrictionDissipation: Math.max(0, structuralDissipation),
      balanceResidual: accepted.dissipation.kineticEnergyAtZero - accepted.trial.kineticEnergy
        - accepted.dissipation.work - structuralDissipation,
    },
    candidateAudit: {
      theoreticalUpperBound: problem.theoreticalCandidateCount,
      nonsingularFeasibleIntervals: problem.modes.length,
      maximumModeCandidates: options.maximumModeCandidates,
    },
    model: 'same-kkt-coulomb-stick-slide-plus-one-frictionless-material-normal',
  };
  prepared.bindingSignature = bindingSignature(bindingBody(prepared));
  return deepFreeze(prepared);
}

function compareObservedBinding(observed, prepared, tolerance = 2e-11) {
  if (!observed || typeof observed !== 'object') return 'transaction observe() returned no binding state';
  if (observed.bindingSignature !== prepared.bindingSignature) return 'committed binding signature differs from prepared event';
  if (Math.abs(Number(observed.p) - prepared.p) > tolerance * Math.max(1, prepared.p)) return 'committed p differs from prepared root';
  if (Math.abs(Number(observed.lambda) - prepared.lambda) > tolerance * Math.max(1, prepared.lambda)) return 'committed lambda differs from prepared p';
  if (!Array.isArray(observed.qPost) || observed.qPost.length !== prepared.qPost.length
    || observed.qPost.some((entry, index) => Math.abs(entry - prepared.qPost[index]) > tolerance * Math.max(1, Math.abs(prepared.qPost[index])))) {
    return 'committed qPost differs from prepared KKT state';
  }
  if (observed.modeKey !== prepared.modeKey) return 'committed mode differs from prepared maximum-dissipation mode';
  if (canonical(observed.contactPointBindings || []) !== canonical(prepared.contactPointBindings)) return 'committed common-point binding differs from prepared state';
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
  try { snapshot = clonePlain(transaction.snapshot()); beforeBytes = canonical(snapshot); }
  catch (error) { return { ok: false, status: 'snapshot-failed', reason: error.message, rolledBack: false }; }
  try {
    const boundState = deepFreeze(clonePlain({
      p: prepared.p, lambda: prepared.lambda, qPost: prepared.qPost,
      modeKey: prepared.modeKey, activeIds: prepared.activeIds,
      contactStates: prepared.contactStates, contactImpulses: prepared.contactImpulses,
      contactPointBindings: prepared.contactPointBindings, materialFriction: prepared.materialFriction,
      freshArea: prepared.freshArea, geometryPayload: prepared.geometryPayload,
      dissipatedWork: prepared.dissipatedWork, materialWork: prepared.materialWork,
      bindingSignature: prepared.bindingSignature,
    }));
    transaction.applyBoundState(boundState);
    if (typeof transaction.observe === 'function') {
      const mismatch = compareObservedBinding(transaction.observe(), prepared);
      if (mismatch) domain(mismatch);
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
    } catch (rollbackError) { rollbackReason = rollbackError.message; }
    return {
      ok: false, status: rollbackExact ? 'commit-failed-rolled-back' : 'rollback-mismatch',
      reason: error.message, rolledBack, rollbackExact, rollbackReason,
    };
  }
}

function solvePrescribedImpulse(input, parameter) {
  try {
    const problem = buildProblem(input);
    const trial = problem.solveAt(parameter);
    const dissipation = problem.workSegments(parameter);
    return deepFreeze({
      ok: true, status: 'solved', p: trial.p, lambda: trial.lambda,
      qPost: trial.qPost.slice(), modeKey: trial.modeKey,
      activeIds: trial.activeIds.slice(), contactStates: trial.contactStates.map((entry) => ({ ...entry })),
      contactImpulses: trial.contactImpulses.map((entry) => ({ ...entry, point: { ...entry.point } })),
      contactPointBindings: clonePlain(problem.pointBindings), materialFriction: { ...problem.materialFriction },
      materialSpeed: trial.materialSpeed, kineticEnergy: trial.kineticEnergy,
      dissipatedWork: dissipation.work,
      workSegments: dissipation.segments.map((entry) => clonePlain(entry)),
      modeTransitions: dissipation.transitions,
      structuralFrictionDissipation: dissipation.structuralFrictionDissipation,
      maximumDissipation: clonePlain(trial.maximumDissipation),
      candidateAudit: {
        theoreticalUpperBound: problem.theoreticalCandidateCount,
        nonsingularFeasibleIntervals: problem.modes.length,
        maximumModeCandidates: problem.options.maximumModeCandidates,
      },
    });
  } catch (error) {
    return { ok: false, status: classifyError(error), reason: error.message };
  }
}

module.exports = {
  DEFAULT_OPTIONS, MODE,
  prepareMaterialEvent, solvePrescribedImpulse, validatePreparedEvent, commitPreparedEvent,
  _reference: { canonical, buildProblem, bindingBody, bindingSignature },
};
