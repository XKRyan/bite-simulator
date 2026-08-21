'use strict';

const path = require('node:path');

const COULOMB_PATH = path.resolve(__dirname, '..', 'kkt-coulomb-extension', 'event-kkt-coulomb.js');
const coulomb = require(COULOMB_PATH);

function clone(value) {
  return value === undefined ? undefined : JSON.parse(JSON.stringify(value));
}

function canonical(value) {
  if (value === null || typeof value !== 'object') {
    if (typeof value === 'number' && Object.is(value, -0)) return '0';
    return JSON.stringify(value);
  }
  if (Array.isArray(value)) return `[${value.map(canonical).join(',')}]`;
  return `{${Object.keys(value).sort().map((key) => `${JSON.stringify(key)}:${canonical(value[key])}`).join(',')}}`;
}

function deepFreeze(value) {
  if (!value || typeof value !== 'object' || Object.isFrozen(value)) return value;
  Object.freeze(value);
  Object.values(value).forEach(deepFreeze);
  return value;
}

function identity(size) {
  return Array.from({ length: size }, (_, row) => (
    Array.from({ length: size }, (_, column) => row === column ? 1 : 0)
  ));
}

function dot(left, right) {
  return left.reduce((sum, value, index) => sum + value * right[index], 0);
}

function multiply(matrix, vector) {
  return matrix.map((row) => dot(row, vector));
}

function add(left, right) {
  return left.map((value, index) => value + right[index]);
}

function scale(vector, scalar) {
  return vector.map((value) => value * scalar);
}

function sameLengthVector(value, size, label) {
  if (!Array.isArray(value) || value.length !== size || !value.every(Number.isFinite)) {
    throw new TypeError(`${label} must be a finite vector of length ${size}`);
  }
  return value.slice();
}

function normalizeStructuralContacts(contacts, h, options = {}) {
  if (!(Number.isFinite(h) && h > 0)) throw new TypeError('h must be finite and positive');
  return (contacts || []).map((source, index) => {
    const phase = source.phase || 'persistent';
    if (phase !== 'onset' && phase !== 'persistent') {
      throw new TypeError(`structuralContacts[${index}].phase must be onset or persistent`);
    }
    const gap = Number(source.gap ?? 0);
    const restitution = Number(source.restitution ?? 0);
    const preNormalVelocity = Number(source.preNormalVelocity ?? 0);
    if (![gap, restitution, preNormalVelocity].every(Number.isFinite)) {
      throw new TypeError(`structuralContacts[${index}] has non-finite Moreau/restitution data`);
    }
    if (restitution < 0 || restitution > 1) {
      throw new TypeError(`structuralContacts[${index}].restitution must be in [0,1]`);
    }
    const applyRestitution = phase === 'onset' || options.mutant === 'persistent-restitution';
    const restitutionBias = applyRestitution ? restitution * Math.min(0, preNormalVelocity) : 0;
    // Existing overlap is not projected out at velocity level. Only positive
    // gap contributes endpoint closure budget; negative gap stays telemetry.
    const normalBias = Math.max(gap, 0) / h + restitutionBias;
    return {
      id: String(source.id ?? `${source.role || 'contact'}-${index}`),
      role: source.role,
      point: clone(source.point),
      normalRow: clone(source.normalRow),
      tangentRow: clone(source.tangentRow),
      mu: source.mu,
      normalBias,
      qaBinding: {
        h,
        gap,
        phase,
        restitution,
        preNormalVelocity,
        restitutionApplied: applyRestitution && restitution > 0 && preNormalVelocity < 0,
      },
    };
  });
}

function makeFreshArea(h) {
  return (p, trial) => ({
    area: 2,
    payload: {
      schema: 'material-free-cluster-qa-geometry-v1',
      h,
      p,
      qPost: clone(trial.qPost),
      modeKey: trial.modeKey,
      materialPoint: clone(trial.materialPoint),
      structuralPoints: (trial.contactImpulses || []).map((entry) => clone(entry.point)),
    },
  });
}

function defaultContacts(dimension = 3) {
  if (dimension !== 3) throw new Error('defaultContacts is defined for the three-axis QA fixture');
  return [
    {
      id: 'fork-target', role: 'fork-target', point: { x: 1, y: 1 },
      normalRow: [0, 1, 0], tangentRow: [1, 0, 0], mu: 0.5,
      gap: 0, phase: 'persistent', restitution: 0, preNormalVelocity: 1,
    },
    {
      id: 'target-floor', role: 'target-floor', point: { x: 2, y: 0 },
      normalRow: [0, 0, 1], tangentRow: [1, 0, 0], mu: 0.3,
      gap: 0, phase: 'persistent', restitution: 0, preNormalVelocity: 0.6,
    },
  ];
}

function makeConfig(overrides = {}) {
  const q0 = overrides.q0 || [2, 1, 0.6];
  const dimension = q0.length;
  const h = overrides.h ?? 0.1;
  const structuralContacts = overrides.structuralContacts || defaultContacts(dimension);
  const materialContact = overrides.materialContact || {
    id: 'weapon-material', point: { x: 0.1, y: 0.2 }, normalRow: [1, 2, 1],
  };
  return {
    h,
    Minv: overrides.Minv || identity(dimension),
    materialContact,
    structuralContacts,
    specificCuttingEnergy: overrides.specificCuttingEnergy ?? 1,
    width: overrides.width ?? 1,
    freshArea: overrides.freshArea || makeFreshArea(h),
    maximumContacts: overrides.maximumContacts ?? 6,
    maximumModeCandidates: overrides.maximumModeCandidates ?? 4096,
    maskSolverGroups: overrides.maskSolverGroups || {
      target: 0, weapon: 0, fork: 0, targetFloor: 0, forkFloor: 0,
    },
    qaMutant: overrides.qaMutant || null,
  };
}

function createWorldAdapter(options = {}) {
  const q0 = (options.q0 || [2, 1, 0.6]).slice();
  const dimension = q0.length;
  let Minv = clone(options.Minv || identity(dimension));
  const externalForce = sameLengthVector(options.externalForce || Array(dimension).fill(0), dimension, 'externalForce');
  const actuationForce = sameLengthVector(options.actuationForce || externalForce, dimension, 'actuationForce');
  const contactFrameTemplate = clone(options.contactFrame || {
    Minv,
    materialContact: options.materialContact || (dimension === 3
      ? { id: 'weapon-material', point: { x: 0.1, y: 0.2 }, normalRow: [1, 2, 1] }
      : { id: 'weapon-material', point: { x: 0, y: 0 }, normalRow: Array.from({ length: dimension }, (_, i) => i === dimension - 1 ? 1 : 0) }),
    structuralContacts: options.structuralContacts || (dimension === 3 ? defaultContacts(3) : []),
    specificCuttingEnergy: options.specificCuttingEnergy ?? 1,
    width: options.width ?? 1,
    geometrySource: 'same-sfree-contact-frame',
  });
  let faultAt = null;
  let state = {
    q: q0,
    time: 0,
    geometry: { revision: 0, payload: null },
    solverGroups: clone(options.solverGroups || {
      target: 0x0001001e,
      weapon: 0x00020001,
      fork: 0x00040019,
      targetFloor: 0x00080001,
      forkFloor: 0x00100004,
    }),
    counters: {
      worldSteps: 0,
      externalForceIntegrations: 0,
      actuationWorkIntegrations: 0,
      qWrites: 0,
      geometryWrites: 0,
      solverGroupWrites: 0,
      dryProbeWrites: 0,
    },
    ledger: { actuationWork: 0 },
    cache: { epoch: 0, lastProbe: null },
    trace: { stepEpoch: 0, forceEpoch: 0, lastH: null },
  };

  function hit(stage) {
    if (faultAt === stage) throw new Error(`injected fault at ${stage}`);
  }

  const adapter = {
    contractVersion: 1,
    snapshot() { return clone(state); },
    restore(snapshot) { state = clone(snapshot); },
    canonicalSnapshot() { return canonical(state); },
    readSolverGroups() { return clone(state.solverGroups); },
    writeSolverGroups(groups) {
      if (faultAt === 'adapter:during-mask-write') {
        const first = Object.keys(groups || {})[0];
        if (first !== undefined) state.solverGroups[first] = groups[first];
        state.counters.solverGroupWrites += 1;
        throw new Error('injected fault during solverGroups mask write');
      }
      state.solverGroups = clone(groups);
      state.counters.solverGroupWrites += 1;
    },
    advanceFree({ h, event = {}, maskedPairs = [] }) {
      hit('adapter:before-world-step');
      if (!(Number.isFinite(h) && h > 0)) throw new TypeError('h must be finite and positive');
      const force = sameLengthVector(event.externalForce || externalForce, dimension, 'event.externalForce');
      const drive = sameLengthVector(event.actuationForce || actuationForce, dimension, 'event.actuationForce');
      const qStart = state.q.slice();
      const generalizedImpulse = scale(force, h);
      const qFree = add(qStart, multiply(Minv, generalizedImpulse));
      const actuationImpulse = scale(drive, h);
      const midpointVelocity = qStart.map((value, index) => 0.5 * (value + qFree[index]));
      const Wact = dot(actuationImpulse, midpointVelocity);
      state.q = qFree.slice();
      state.time += h;
      state.counters.worldSteps += 1;
      state.counters.externalForceIntegrations += 1;
      state.counters.actuationWorkIntegrations += 1;
      state.ledger.actuationWork += Wact;
      state.cache.epoch += 1;
      state.trace.stepEpoch += 1;
      state.trace.forceEpoch += 1;
      state.trace.lastH = h;
      hit('adapter:after-world-step');
      const contactFrame = clone(event.contactFrame || contactFrameTemplate);
      contactFrame.h = h;
      contactFrame.maskedPairs = clone(maskedPairs);
      return deepFreeze({
        qStart,
        qFree: qFree.slice(),
        Wact,
        h,
        contactFrame,
        stepEpoch: state.trace.stepEpoch,
        forceEpoch: state.trace.forceEpoch,
      });
    },
    writeQPost(qPost) {
      hit('adapter:before-q-write');
      state.q = sameLengthVector(qPost, dimension, 'qPost');
      state.counters.qWrites += 1;
      state.cache.epoch += 1;
      hit('adapter:after-q-write');
    },
    applyGeometry(payload) {
      hit('adapter:before-geometry-write');
      state.geometry = { revision: state.geometry.revision + 1, payload: clone(payload) };
      state.counters.geometryWrites += 1;
      state.cache.epoch += 1;
      hit('adapter:after-geometry-write');
    },
    qaMutateDryProbe(value) {
      state.counters.dryProbeWrites += 1;
      state.cache.lastProbe = clone(value);
    },
    qaSetMinv(value) { Minv = clone(value); },
    setFault(stage) { faultAt = stage || null; },
    hit,
    getState() { return clone(state); },
    readExposedState() { return clone(state); },
    readTrace() { return clone(state.trace); },
    restoreSolverGroups(groups) {
      if (faultAt === 'adapter:during-group-restore') {
        const first = Object.keys(groups || {})[0];
        if (first !== undefined) state.solverGroups[first] = groups[first];
        state.counters.solverGroupWrites += 1;
        throw new Error('injected fault during solverGroups restore');
      }
      state.solverGroups = clone(groups);
      state.counters.solverGroupWrites += 1;
    },
    get Minv() { return clone(Minv); },
  };
  return adapter;
}

function kktInputFromFree(token, config, structuralContacts = config.structuralContacts, options = {}) {
  const frame = token.contactFrame || {};
  const frameContacts = frame.structuralContacts || structuralContacts;
  const rows = normalizeStructuralContacts(frameContacts, token.h, options);
  return {
    qFree: clone(token.qFree),
    Minv: clone(frame.Minv || config.Minv),
    materialContact: clone(frame.materialContact || config.materialContact),
    structuralContacts: rows.map(({ qaBinding, ...contact }) => contact),
    specificCuttingEnergy: frame.specificCuttingEnergy ?? config.specificCuttingEnergy,
    width: frame.width ?? config.width,
    freshArea: config.freshArea,
    options: {
      maximumContacts: config.maximumContacts,
      maximumModeCandidates: config.maximumModeCandidates,
    },
  };
}

function prepareAcceptedTrial(token, config, structuralContacts = config.structuralContacts) {
  return coulomb.prepareMaterialEvent(kktInputFromFree(token, config, structuralContacts));
}

module.exports = {
  COULOMB_PATH,
  clone,
  canonical,
  deepFreeze,
  identity,
  dot,
  multiply,
  normalizeStructuralContacts,
  makeFreshArea,
  defaultContacts,
  makeConfig,
  createWorldAdapter,
  kktInputFromFree,
  prepareAcceptedTrial,
};
