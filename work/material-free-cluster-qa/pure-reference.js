'use strict';

const coulomb = require('../kkt-coulomb-extension/event-kkt-coulomb.js');
const {
  clone,
  canonical,
  deepFreeze,
  normalizeStructuralContacts,
} = require('./fixture.js');

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

const MUTANTS = Object.freeze([
  'double-world-step',
  'wact-mutable',
  'wact-recomputed',
  'dual-finish',
  'dry-mutation',
  'mixed-accepted',
  'split-h',
  'groups-leak',
  'persistent-restitution',
  'truncate-domain',
]);

function fnv1a(text) {
  let hash = 0x811c9dc5;
  for (let index = 0; index < text.length; index += 1) {
    hash ^= text.charCodeAt(index);
    hash = Math.imul(hash, 0x01000193) >>> 0;
  }
  return hash.toString(16).padStart(8, '0');
}

function signature(prefix, body) {
  return `${prefix}-${fnv1a(canonical(body))}`;
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

function classifyFailure(error) {
  return {
    ok: false,
    status: 'contract-refused',
    reason: error instanceof Error ? error.message : String(error),
  };
}

function validateFrozenFrame(frame, qFree) {
  const minv = frame?.Minv;
  if (!Array.isArray(minv) || minv.length !== qFree.length
    || !minv.every((row) => Array.isArray(row) && row.length === qFree.length
      && row.every(Number.isFinite))) {
    throw new TypeError('contactFrame.Minv must be a finite square matrix matching qFree');
  }
  return deepFreeze(clone(frame));
}

function solveClusterFromFree(input, internal = {}) {
  try {
    const h = Number(input.h);
    if (!(Number.isFinite(h) && h > 0)) throw new TypeError('h must be finite and positive');
    let rows = normalizeStructuralContacts(input.structuralContacts || [], h, {
      mutant: internal.mutant,
    });
    const options = {
      maximumContacts: input.maximumContacts ?? 6,
      maximumModeCandidates: input.maximumModeCandidates ?? 4096,
    };
    const makeInput = (selectedRows) => ({
      qFree: clone(input.qFree),
      Minv: clone(input.Minv),
      materialContact: clone(input.materialContact),
      structuralContacts: selectedRows.map(({ qaBinding, ...contact }) => contact),
      specificCuttingEnergy: input.specificCuttingEnergy,
      width: input.width,
      freshArea: input.freshArea,
      options,
    });
    let solved = coulomb.solvePrescribedImpulse(makeInput(rows), input.p ?? input.lambda ?? 0);
    if (!solved.ok && solved.status === 'solver-domain-stop' && internal.mutant === 'truncate-domain') {
      const candidateLimitedCount = Math.floor(Math.log(options.maximumModeCandidates) / Math.log(4));
      rows = rows.slice(0, Math.min(options.maximumContacts, candidateLimitedCount));
      solved = coulomb.solvePrescribedImpulse(makeInput(rows), input.p ?? input.lambda ?? 0);
    }
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
    const hAudit = {
      free: h,
      moreau: h,
      work: h,
      geometry: internal.mutant === 'split-h' ? 2 * h : h,
    };
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
      hAudit,
    };
    result.acceptedBindingSignature = signature('free-cluster-accepted-v1', bindingBody(result));
    return deepFreeze(result);
  } catch (error) {
    return deepFreeze(classifyFailure(error));
  }
}

function createReferenceModule({ mutant = null } = {}) {
  if (mutant !== null && !MUTANTS.includes(mutant)) throw new Error(`unknown QA mutant ${mutant}`);

  function createFreeClusterSession(adapter, rawConfig = {}) {
    if (!adapter || typeof adapter.snapshot !== 'function' || typeof adapter.restore !== 'function') {
      throw new TypeError('adapter.snapshot/restore are required');
    }
    const config = { ...rawConfig };
    const active = new Map();
    let nextToken = 1;
    let maskEpoch = 0;

    function hit(stage) {
      if (typeof config.faultInjector === 'function') config.faultInjector(stage);
      else if (typeof adapter.hit === 'function') adapter.hit(stage);
    }

    function groups() {
      return typeof adapter.readSolverGroups === 'function' ? adapter.readSolverGroups() : null;
    }

    function restoreGroups(saved) {
      if (saved !== null && typeof adapter.restoreSolverGroups === 'function') {
        adapter.restoreSolverGroups(saved);
      } else if (saved !== null && typeof adapter.writeSolverGroups === 'function') {
        adapter.writeSolverGroups(saved);
      }
    }

    function recordFor(token, allowConsumed = false) {
      if (!token || !Number.isInteger(token.id)) throw new Error('invalid Sfree token');
      const record = active.get(token.id);
      if (!record || record.publicToken !== token) throw new Error('foreign or stale Sfree token');
      if (record.consumed && !allowConsumed && mutant !== 'dual-finish') {
        throw new Error(`Sfree token ${token.id} already consumed by ${record.finishedBy}`);
      }
      if (token.sFreeBinding !== record.binding) throw new Error('Sfree binding mismatch');
      if (token.h !== record.h || canonical(token.qFree) !== canonical(record.qFree) || token.Wact !== record.Wact) {
        throw new Error('Sfree h/qFree/Wact binding mismatch');
      }
      if (token.contactFrameSignature !== record.contactFrameSignature
        || token.stepEpoch !== record.stepEpoch || token.forceEpoch !== record.forceEpoch) {
        throw new Error('Sfree contact-frame/step/force epoch binding mismatch');
      }
      return record;
    }

    function solverInput(record, parameter) {
      const frame = record.contactFrame;
      if (!Array.isArray(frame.Minv)) {
        throw new Error('frozen Sfree contactFrame.Minv is required');
      }
      return {
        h: record.h,
        qFree: clone(record.qFree),
        Minv: clone(frame.Minv),
        materialContact: clone(frame.materialContact),
        structuralContacts: clone(frame.structuralContacts || []),
        specificCuttingEnergy: frame.specificCuttingEnergy,
        width: frame.width,
        freshArea: config.freshArea,
        maximumContacts: config.maximumContacts,
        maximumModeCandidates: config.maximumModeCandidates,
        p: parameter,
      };
    }

    function assertGroups(saved, label) {
      if (saved !== null && canonical(groups()) !== canonical(saved)) {
        throw new Error(`${label} leaked solverGroups`);
      }
    }

    function advanceFree({ h, event = {} }) {
      const stepH = Number(h);
      if (!(Number.isFinite(stepH) && stepH > 0)) throw new TypeError('h must be finite and positive');
      const root = adapter.snapshot();
      const traceBefore = typeof adapter.readTrace === 'function' ? adapter.readTrace() : null;
      const originalGroups = groups();
      let free = null;
      let failure = null;
      try {
        hit('advance:before-mask');
        if (originalGroups !== null) adapter.writeSolverGroups(clone(config.maskSolverGroups));
        hit('advance:after-mask');
        free = adapter.advanceFree({ h: stepH, event, maskedPairs: clone(config.maskPlan || []) });
        if (mutant === 'double-world-step') {
          free = adapter.advanceFree({ h: stepH, event, maskedPairs: clone(config.maskPlan || []) });
        }
        hit('advance:after-world');
        if (!free || !Array.isArray(free.qFree) || !Number.isFinite(free.Wact)
          || !free.contactFrame || !Number.isInteger(free.stepEpoch) || !Number.isInteger(free.forceEpoch)) {
          throw new Error('adapter.advanceFree must return qFree/Wact/contactFrame/stepEpoch/forceEpoch');
        }
        free = { ...free, contactFrame: validateFrozenFrame(free.contactFrame, free.qFree) };
        if (traceBefore) {
          if (free.stepEpoch !== traceBefore.stepEpoch + 1) throw new Error('advanceFree did not own exactly one world step');
          if (free.forceEpoch !== traceBefore.forceEpoch + 1) throw new Error('advanceFree did not integrate external force exactly once');
        }
        hit('advance:after-free-capture');
      } catch (error) {
        failure = error;
      }
      try {
        if (mutant !== 'groups-leak') restoreGroups(originalGroups);
      } catch (error) {
        failure = failure || error;
      }
      if (failure) {
        adapter.restore(root);
        if (mutant === 'groups-leak' && originalGroups !== null) {
          adapter.writeSolverGroups(clone(config.maskSolverGroups));
        }
        throw failure;
      }
      if (mutant !== 'groups-leak') assertGroups(originalGroups, 'advanceFree');
      maskEpoch += 1;
      const tokenBody = {
        version: 1,
        id: nextToken,
        h: stepH,
        qFree: clone(free.qFree),
        Wact: free.Wact,
        maskEpoch,
        contactFrameSignature: signature('contact-frame-v1', free.contactFrame),
        stepEpoch: free.stepEpoch,
        forceEpoch: free.forceEpoch,
      };
      const binding = signature('sfree-v1', tokenBody);
      const publicToken = { ...tokenBody, sFreeBinding: binding };
      if (mutant !== 'wact-mutable') deepFreeze(publicToken);
      const record = {
        id: nextToken,
        publicToken,
        binding,
        h: stepH,
        qFree: clone(free.qFree),
        Wact: free.Wact,
        contactFrame: clone(free.contactFrame),
        contactFrameSignature: tokenBody.contactFrameSignature,
        stepEpoch: free.stepEpoch,
        forceEpoch: free.forceEpoch,
        root,
        freeSnapshot: adapter.snapshot(),
        groups: originalGroups,
        consumed: false,
        finishedBy: null,
      };
      active.set(nextToken, record);
      nextToken += 1;
      return publicToken;
    }

    function trialMaterial(token, { lambda, p, materialContact, structuralContacts, dry } = {}) {
      const record = recordFor(token);
      if (dry !== true) throw new Error('trialMaterial requires dry:true');
      if (materialContact !== undefined || structuralContacts !== undefined) {
        throw new Error('trialMaterial rows must come only from the frozen Sfree contactFrame');
      }
      if (p !== undefined && lambda !== undefined && p !== lambda) throw new Error('trial p/lambda mismatch');
      const parameter = p ?? lambda;
      if (!(Number.isFinite(parameter) && parameter >= 0)) throw new TypeError('trial lambda must be finite and non-negative');
      const before = adapter.snapshot();
      let failure = null;
      let result;
      try {
        hit('trial:before-solve');
        if (mutant === 'dry-mutation' && typeof adapter.qaMutateDryProbe === 'function') {
          adapter.qaMutateDryProbe({ token: token.id, parameter });
        }
        result = solveClusterFromFree(solverInput(record, parameter), { mutant });
        hit('trial:after-solve');
        if (canonical(adapter.snapshot()) !== canonical(before)) {
          throw new Error('dry trial mutated adapter state');
        }
      } catch (error) {
        failure = error;
      }
      if (failure) {
        adapter.restore(before);
        throw failure;
      }
      return result;
    }

    function validateAccepted(record, acceptedTrial) {
      if (!acceptedTrial || acceptedTrial.ok !== true) throw new Error('acceptedTrial must be a successful dry trial');
      const repeated = solveClusterFromFree(
        solverInput(record, acceptedTrial.p),
        { mutant },
      );
      if (!repeated.ok) throw new Error(`accepted trial no longer solves: ${repeated.reason}`);
      if (acceptedTrial.acceptedBindingSignature !== signature('free-cluster-accepted-v1', bindingBody(acceptedTrial))) {
        throw new Error('accepted trial signature is invalid');
      }
      if (canonical(bindingBody(repeated)) !== canonical(bindingBody(acceptedTrial))) {
        throw new Error('accepted same-p/rows/modes/qPost binding mismatch');
      }
      return repeated;
    }

    function finishMaterial(token, { acceptedTrial } = {}) {
      const record = recordFor(token);
      const before = adapter.snapshot();
      let result;
      try {
        hit('finish:before-validate');
        result = validateAccepted(record, acceptedTrial);
        hit('finish:after-validate');
        hit('finish:before-writeback');
        const qPost = clone(result.qPost);
        if (mutant === 'mixed-accepted') qPost[0] += 1e-3;
        adapter.writeQPost(qPost);
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
        const committed = {
          ...clone(result),
          status: 'material-finished',
          Wact: record.Wact,
          finishedBy: 'material',
          commitTicket,
        };
        if (mutant === 'mixed-accepted') {
          committed.p += 1e-3;
          committed.qPost = qPost;
        }
        if (mutant === 'wact-recomputed') committed.Wact = record.Wact + 1;
        return deepFreeze(committed);
      } catch (error) {
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
        const result = solveClusterFromFree(solverInput(record, 0), { mutant });
        if (!result.ok) return result;
        hit('structural:after-solve');
        hit('structural:before-writeback');
        adapter.writeQPost(result.qPost);
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
      canonicalExposed: canonical,
    };
  }

  return {
    contractVersion: 1,
    faultStages: FAULT_STAGES,
    createFreeClusterSession,
    solveClusterFromFree: (input) => solveClusterFromFree(input, { mutant }),
    canonicalExposed: canonical,
  };
}

const reference = createReferenceModule();

module.exports = {
  ...reference,
  FAULT_STAGES,
  MUTANTS,
  createReferenceModule,
  _qa: { bindingBody, signature, classifyFailure },
};
