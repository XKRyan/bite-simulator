'use strict';

// Transaction boundary between one actual free Rapier advance and one
// simultaneous structural/material Coulomb solve.  This module deliberately
// does not own geometry publication; finishMaterial returns a bound commit
// ticket for the later root transaction.

const signedCoulomb = require('../kkt-coulomb-extension/event-kkt-coulomb.js');

const FAULT_STAGES = Object.freeze([
  'advance:before-mask',
  'advance:after-mask',
  'adapter:during-mask-write',
  'adapter:during-group-restore',
  'adapter:before-world-step',
  'adapter:after-world-step',
  'advance:after-world',
  'advance:after-free-capture',
  'trial:before-solve',
  'trial:after-solve',
  'finish:before-validate',
  'finish:after-validate',
  'finish:before-writeback',
  'adapter:before-q-write',
  'adapter:after-q-write',
  'finish:after-writeback',
  'finish:before-ticket',
  'finish:after-ticket',
  'finish:before-consume',
  'structural:before-solve',
  'structural:after-solve',
  'structural:before-writeback',
  'structural:after-writeback',
  'structural:before-consume',
]);

function clone(value) {
  return value === undefined ? undefined : JSON.parse(JSON.stringify(value));
}

function canonicalExposed(value) {
  if (value === null || typeof value !== 'object') {
    if (typeof value === 'number' && Object.is(value, -0)) return '0';
    return JSON.stringify(value);
  }
  if (Array.isArray(value)) return `[${value.map(canonicalExposed).join(',')}]`;
  return `{${Object.keys(value).sort().map((key) => (
    `${JSON.stringify(key)}:${canonicalExposed(value[key])}`
  )).join(',')}}`;
}

function deepFreeze(value) {
  if (!value || typeof value !== 'object' || Object.isFrozen(value)) return value;
  Object.freeze(value);
  Object.values(value).forEach(deepFreeze);
  return value;
}

function fnv1a(text) {
  let hash = 0x811c9dc5;
  for (let index = 0; index < text.length; index += 1) {
    hash ^= text.charCodeAt(index);
    hash = Math.imul(hash, 0x01000193) >>> 0;
  }
  return hash.toString(16).padStart(8, '0');
}

function signature(prefix, body) {
  return `${prefix}-${fnv1a(canonicalExposed(body))}`;
}

function freezeContactFrame(contactFrame, qFree) {
  if (!contactFrame || typeof contactFrame !== 'object') {
    throw new TypeError('contactFrame is required');
  }
  if (!Array.isArray(qFree) || !qFree.length || !qFree.every(Number.isFinite)) {
    throw new TypeError('qFree must be a non-empty finite vector');
  }
  const Minv = contactFrame.Minv;
  if (!Array.isArray(Minv) || Minv.length !== qFree.length
    || !Minv.every((row) => Array.isArray(row) && row.length === qFree.length
      && row.every(Number.isFinite))) {
    throw new TypeError('contactFrame.Minv must be a finite square matrix matching qFree');
  }
  return deepFreeze(clone(contactFrame));
}

function normalizeStructuralContacts(contacts, h) {
  if (!(Number.isFinite(h) && h > 0)) throw new TypeError('h must be finite and positive');
  return (contacts || []).map((source, index) => {
    const phase = source.phase || 'persistent';
    if (phase !== 'onset' && phase !== 'persistent') {
      throw new TypeError(`structuralContacts[${index}].phase must be onset or persistent`);
    }
    const gap = Number(source.gap ?? 0);
    const restitution = Number(source.restitution ?? 0);
    const preNormalVelocity = Number(source.preNormalVelocity ?? 0);
    const mu = Number(source.mu);
    if (![gap, restitution, preNormalVelocity, mu].every(Number.isFinite)) {
      throw new TypeError(`structuralContacts[${index}] has non-finite contact data`);
    }
    if (restitution < 0 || restitution > 1) {
      throw new TypeError(`structuralContacts[${index}].restitution must be in [0,1]`);
    }
    if (mu < 0) throw new TypeError(`structuralContacts[${index}].mu must be non-negative`);
    const restitutionApplied = phase === 'onset' && restitution > 0 && preNormalVelocity < 0;
    const normalBias = Math.max(gap, 0) / h + (phase === 'onset' ? restitution * Math.min(0, preNormalVelocity) : 0);
    return {
      id: String(source.id ?? `${source.role || 'contact'}-${index}`),
      role: source.role,
      point: clone(source.point),
      normalRow: clone(source.normalRow),
      tangentRow: clone(source.tangentRow),
      mu,
      normalBias,
      qaBinding: {
        h,
        gap,
        phase,
        restitution,
        preNormalVelocity,
        restitutionApplied,
      },
    };
  });
}

function bindingBody(trial) {
  return {
    h: trial.h,
    p: trial.p,
    lambda: trial.lambda,
    qFree: trial.qFree,
    qPost: trial.qPost,
    modeKey: trial.modeKey,
    contactStates: trial.contactStates,
    contactPointBindings: trial.contactPointBindings,
    rowBindings: trial.rowBindings,
    workSegments: trial.workSegments,
    dissipatedWork: trial.dissipatedWork,
    geometryPayload: trial.geometryPayload,
    hAudit: trial.hAudit,
  };
}

function refused(error) {
  return deepFreeze({
    ok: false,
    status: 'contract-refused',
    reason: error instanceof Error ? error.message : String(error),
  });
}

function solveClusterFromFreeWith(input, solvePrescribedImpulse) {
  try {
    const h = Number(input.h);
    if (!(Number.isFinite(h) && h > 0)) throw new TypeError('h must be finite and positive');
    const rows = normalizeStructuralContacts(input.structuralContacts || [], h);
    const solverInput = {
      qFree: clone(input.qFree),
      Minv: clone(input.Minv),
      materialContact: clone(input.materialContact),
      structuralContacts: rows.map(({ qaBinding, ...contact }) => contact),
      specificCuttingEnergy: input.specificCuttingEnergy,
      width: input.width,
      freshArea: input.freshArea,
      options: {
        maximumContacts: input.maximumContacts ?? 6,
        maximumModeCandidates: input.maximumModeCandidates ?? 4096,
      },
    };
    const solved = solvePrescribedImpulse(solverInput, input.p ?? input.lambda ?? 0);
    if (!solved.ok) return deepFreeze(clone(solved));

    let geometryPayload = null;
    if (typeof input.freshArea === 'function') {
      const fresh = input.freshArea(solved.p, {
        p: solved.p,
        lambda: solved.lambda,
        qPost: clone(solved.qPost),
        modeKey: solved.modeKey,
        contactImpulses: clone(solved.contactImpulses),
        materialPoint: clone(input.materialContact.point),
      });
      geometryPayload = clone(typeof fresh === 'number' ? null : fresh?.payload ?? null);
    }
    const result = {
      ...clone(solved),
      h,
      qFree: clone(input.qFree),
      rowBindings: rows.map((entry) => ({
        id: entry.id,
        role: entry.role,
        point: clone(entry.point),
        normalRow: clone(entry.normalRow),
        tangentRow: clone(entry.tangentRow),
        mu: entry.mu,
        normalBias: entry.normalBias,
        ...clone(entry.qaBinding),
      })),
      geometryPayload,
      hAudit: { free: h, moreau: h, work: h, geometry: h },
    };
    result.acceptedBindingSignature = signature('free-cluster-accepted-v1', bindingBody(result));
    return deepFreeze(result);
  } catch (error) {
    return refused(error);
  }
}

function solveClusterFromFree(input) {
  return solveClusterFromFreeWith(input, signedCoulomb.solvePrescribedImpulse);
}

function createFreeClusterSession(adapter, rawConfig = {}) {
  if (!adapter || typeof adapter.snapshot !== 'function' || typeof adapter.restore !== 'function') {
    throw new TypeError('adapter.snapshot/restore are required');
  }
  if (typeof adapter.advanceFree !== 'function' || typeof adapter.writeQPost !== 'function') {
    throw new TypeError('adapter.advanceFree/writeQPost are required');
  }
  const config = { ...rawConfig };
  const solvePrescribedImpulse = config.solvePrescribedImpulse || signedCoulomb.solvePrescribedImpulse;
  const active = new Map();
  let nextToken = 1;
  let maskEpoch = 0;

  function hit(stage) {
    if (typeof config.faultInjector === 'function') config.faultInjector(stage);
    else if (typeof adapter.hit === 'function') adapter.hit(stage);
  }

  function readGroups() {
    return typeof adapter.readSolverGroups === 'function' ? clone(adapter.readSolverGroups()) : null;
  }

  function writeMask(groups) {
    if (groups !== null && typeof adapter.writeSolverGroups === 'function') {
      adapter.writeSolverGroups(clone(groups));
    }
  }

  function restoreGroups(groups) {
    if (groups === null) return;
    if (typeof adapter.restoreSolverGroups === 'function') adapter.restoreSolverGroups(clone(groups));
    else if (typeof adapter.writeSolverGroups === 'function') adapter.writeSolverGroups(clone(groups));
    else throw new Error('adapter cannot restore solverGroups');
  }

  function assertGroups(expected, label) {
    if (expected !== null && canonicalExposed(readGroups()) !== canonicalExposed(expected)) {
      throw new Error(`${label} leaked solverGroups`);
    }
  }

  function recordFor(token) {
    if (!token || !Number.isInteger(token.id)) throw new Error('invalid Sfree token');
    const record = active.get(token.id);
    if (!record || record.publicToken !== token) throw new Error('foreign or stale Sfree token');
    if (record.consumed) throw new Error(`Sfree token ${token.id} already consumed by ${record.finishedBy}`);
    if (token.sFreeBinding !== record.binding) throw new Error('Sfree binding mismatch');
    if (token.h !== record.h || token.Wact !== record.Wact
      || canonicalExposed(token.qFree) !== canonicalExposed(record.qFree)) {
      throw new Error('Sfree h/qFree/Wact binding mismatch');
    }
    if (token.contactFrameSignature !== record.contactFrameSignature
      || token.stepEpoch !== record.stepEpoch || token.forceEpoch !== record.forceEpoch) {
      throw new Error('Sfree contact-frame/step/force epoch binding mismatch');
    }
    return record;
  }

  function solverInput(record, p) {
    const frame = record.contactFrame;
    return {
      h: record.h,
      qFree: clone(record.qFree),
      Minv: clone(frame.Minv),
      materialContact: clone(frame.materialContact),
      structuralContacts: clone(frame.structuralContacts || []),
      specificCuttingEnergy: frame.specificCuttingEnergy,
      width: frame.width,
      freshArea: record.freshArea,
      maximumContacts: config.maximumContacts,
      maximumModeCandidates: config.maximumModeCandidates,
      p,
    };
  }

  function advanceFree({ h, event = {} }) {
    const stepH = Number(h);
    if (!(Number.isFinite(stepH) && stepH > 0)) throw new TypeError('h must be finite and positive');
    const root = adapter.snapshot();
    const traceBefore = typeof adapter.readTrace === 'function' ? adapter.readTrace() : null;
    const originalGroups = readGroups();
    let free = null;
    let contactFrame = null;
    let failure = null;
    try {
      hit('advance:before-mask');
      writeMask(config.maskSolverGroups ?? null);
      hit('advance:after-mask');
      free = adapter.advanceFree({ h: stepH, event, maskedPairs: clone(config.maskPlan || []) });
      hit('advance:after-world');
      if (!free || !Array.isArray(free.qFree) || !Number.isFinite(free.Wact)
        || !free.contactFrame || !Number.isInteger(free.stepEpoch) || !Number.isInteger(free.forceEpoch)) {
        throw new Error('adapter.advanceFree must return qFree/Wact/contactFrame/stepEpoch/forceEpoch');
      }
      contactFrame = freezeContactFrame(free.contactFrame, free.qFree);
      if (traceBefore) {
        if (free.stepEpoch !== traceBefore.stepEpoch + 1) throw new Error('advanceFree did not own exactly one world step');
        if (free.forceEpoch !== traceBefore.forceEpoch + 1) throw new Error('advanceFree did not integrate external force exactly once');
      }
      hit('advance:after-free-capture');
    } catch (error) {
      failure = error;
    }
    try {
      restoreGroups(originalGroups);
    } catch (error) {
      failure = failure || error;
    }
    if (failure) {
      adapter.restore(root);
      throw failure;
    }
    try {
      assertGroups(originalGroups, 'advanceFree');
    } catch (error) {
      adapter.restore(root);
      throw error;
    }

    maskEpoch += 1;
    const tokenBody = {
      version: 1,
      id: nextToken,
      h: stepH,
      qFree: clone(free.qFree),
      Wact: free.Wact,
      maskEpoch,
      contactFrameSignature: signature('contact-frame-v1', contactFrame),
      stepEpoch: free.stepEpoch,
      forceEpoch: free.forceEpoch,
    };
    const binding = signature('sfree-v1', tokenBody);
    const publicToken = deepFreeze({ ...tokenBody, sFreeBinding: binding });
    active.set(nextToken, {
      publicToken,
      binding,
      h: stepH,
      qFree: clone(free.qFree),
      Wact: free.Wact,
      contactFrame,
      freshArea: typeof event.freshArea === 'function' ? event.freshArea : config.freshArea,
      contactFrameSignature: tokenBody.contactFrameSignature,
      stepEpoch: free.stepEpoch,
      forceEpoch: free.forceEpoch,
      root,
      groups: originalGroups,
      consumed: false,
      finishedBy: null,
    });
    nextToken += 1;
    return publicToken;
  }

  function trialMaterial(token, options = {}) {
    const record = recordFor(token);
    if (options.dry !== true) throw new Error('trialMaterial requires dry:true');
    if (options.materialContact !== undefined || options.structuralContacts !== undefined) {
      throw new Error('trialMaterial rows must come only from the frozen Sfree contactFrame');
    }
    if (options.p !== undefined && options.lambda !== undefined && options.p !== options.lambda) {
      throw new Error('trial p/lambda mismatch');
    }
    const p = options.p ?? options.lambda;
    if (!(Number.isFinite(p) && p >= 0)) throw new TypeError('trial lambda must be finite and non-negative');
    const before = adapter.snapshot();
    try {
      hit('trial:before-solve');
      const result = solveClusterFromFreeWith(solverInput(record, p), solvePrescribedImpulse);
      hit('trial:after-solve');
      if (canonicalExposed(adapter.snapshot()) !== canonicalExposed(before)) {
        throw new Error('dry trial mutated adapter state');
      }
      return result;
    } catch (error) {
      adapter.restore(before);
      throw error;
    }
  }

  function validateAccepted(record, acceptedTrial) {
    if (!acceptedTrial || acceptedTrial.ok !== true) {
      throw new Error('acceptedTrial must be a successful dry trial');
    }
    if (acceptedTrial.acceptedBindingSignature
      !== signature('free-cluster-accepted-v1', bindingBody(acceptedTrial))) {
      throw new Error('accepted trial signature is invalid');
    }
    const repeated = solveClusterFromFreeWith(
      solverInput(record, acceptedTrial.p),
      solvePrescribedImpulse,
    );
    if (!repeated.ok) throw new Error(`accepted trial no longer solves: ${repeated.reason}`);
    if (canonicalExposed(bindingBody(repeated)) !== canonicalExposed(bindingBody(acceptedTrial))) {
      throw new Error('accepted same-p/rows/modes/qPost binding mismatch');
    }
    return repeated;
  }

  function finishMaterial(token, { acceptedTrial } = {}) {
    const record = recordFor(token);
    const before = adapter.snapshot();
    try {
      hit('finish:before-validate');
      const result = validateAccepted(record, acceptedTrial);
      hit('finish:after-validate');
      hit('finish:before-writeback');
      adapter.writeQPost(clone(result.qPost));
      hit('finish:after-writeback');
      hit('finish:before-ticket');
      const commitTicket = deepFreeze({
        version: 1,
        tokenId: token.id,
        sFreeBinding: token.sFreeBinding,
        contactFrameSignature: token.contactFrameSignature,
        acceptedBindingSignature: result.acceptedBindingSignature,
        Wact: record.Wact,
        h: result.h,
        p: result.p,
        qPost: clone(result.qPost),
        modeKey: result.modeKey,
        rowBindings: clone(result.rowBindings),
        geometryPayload: clone(result.geometryPayload),
      });
      hit('finish:after-ticket');
      hit('finish:before-consume');
      record.consumed = true;
      record.finishedBy = 'material';
      assertGroups(record.groups, 'finishMaterial');
      return deepFreeze({
        ...clone(result),
        status: 'material-finished',
        Wact: record.Wact,
        finishedBy: 'material',
        commitTicket,
      });
    } catch (error) {
      record.consumed = false;
      record.finishedBy = null;
      adapter.restore(before);
      throw error;
    }
  }

  function finishNoMaterial(token, options = {}) {
    const record = recordFor(token);
    if (options.structuralContacts !== undefined || options.materialContact !== undefined) {
      throw new Error('finishNoMaterial rows must come only from the frozen Sfree contactFrame');
    }
    const before = adapter.snapshot();
    try {
      hit('structural:before-solve');
      const result = solveClusterFromFreeWith(solverInput(record, 0), solvePrescribedImpulse);
      if (!result.ok) return result;
      hit('structural:after-solve');
      hit('structural:before-writeback');
      adapter.writeQPost(clone(result.qPost));
      hit('structural:after-writeback');
      hit('structural:before-consume');
      record.consumed = true;
      record.finishedBy = 'structural';
      assertGroups(record.groups, 'finishNoMaterial');
      return deepFreeze({
        ...clone(result),
        status: 'structural-finished',
        Wact: record.Wact,
        finishedBy: 'structural',
      });
    } catch (error) {
      record.consumed = false;
      record.finishedBy = null;
      adapter.restore(before);
      throw error;
    }
  }

  function abort(token) {
    const record = recordFor(token);
    adapter.restore(record.root);
    record.consumed = true;
    record.finishedBy = 'aborted';
    assertGroups(record.groups, 'abort');
    return deepFreeze({ ok: true, status: 'aborted', tokenId: token.id });
  }

  return {
    advanceFree,
    finishNoMaterial,
    trialMaterial,
    finishMaterial,
    abort,
    snapshot: () => adapter.snapshot(),
    canonicalExposed,
  };
}

module.exports = {
  contractVersion: 1,
  faultStages: FAULT_STAGES,
  FAULT_STAGES,
  createFreeClusterSession,
  solveClusterFromFree,
  canonicalExposed,
};
