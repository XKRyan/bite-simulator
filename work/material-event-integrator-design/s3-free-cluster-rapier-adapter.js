'use strict';

// Work-only S3 adapter.  It owns the *ordering* between one Rapier free
// advance and the frozen free-cluster token, but deliberately knows nothing
// about material removal or the production iterator.
(function expose(factory) {
  const api = factory();
  if (typeof module === 'object' && module.exports) module.exports = api;
  if (typeof globalThis === 'object') globalThis.BiteS3FreeClusterRapierAdapter = api;
}(() => {
  function clone(value) {
    if (value === undefined) return undefined;
    if (value instanceof Uint8Array) return new Uint8Array(value);
    if (Array.isArray(value)) return value.map(clone);
    if (value && typeof value === 'object') {
      const copy = {};
      Object.keys(value).forEach((key) => { copy[key] = clone(value[key]); });
      return copy;
    }
    return value;
  }

  function deepFreeze(value) {
    if (!value || typeof value !== 'object' || Object.isFrozen(value)) return value;
    Object.freeze(value);
    Object.keys(value).forEach((key) => deepFreeze(value[key]));
    return value;
  }

  function canonical(value) {
    if (value === null || typeof value !== 'object') {
      if (typeof value === 'number' && Object.is(value, -0)) return '0';
      return JSON.stringify(value);
    }
    if (ArrayBuffer.isView(value)) return `[${Array.from(value).join(',')}]`;
    if (Array.isArray(value)) return `[${value.map(canonical).join(',')}]`;
    return `{${Object.keys(value).sort().map((key) => (
      `${JSON.stringify(key)}:${canonical(value[key])}`
    )).join(',')}}`;
  }

  function firstDifference(left, right, path = '$') {
    if (Object.is(left, right)) return null;
    if (canonical(left) === canonical(right)) return null;
    if (left === null || right === null || typeof left !== 'object' || typeof right !== 'object') {
      return `${path}: ${JSON.stringify(left)} != ${JSON.stringify(right)}`;
    }
    const leftArray = Array.isArray(left); const rightArray = Array.isArray(right);
    if (leftArray !== rightArray) return `${path}: array/object type mismatch`;
    const leftKeys = Object.keys(left).sort(); const rightKeys = Object.keys(right).sort();
    if (canonical(leftKeys) !== canonical(rightKeys)) {
      return `${path}: keys ${canonical(leftKeys)} != ${canonical(rightKeys)}`;
    }
    for (const key of leftKeys) {
      const difference = firstDifference(left[key], right[key], `${path}.${key}`);
      if (difference) return difference;
    }
    return `${path}: non-canonical value mismatch`;
  }

  function requireFunction(bindings, name) {
    if (typeof bindings?.[name] !== 'function') throw new TypeError(`S3 binding ${name} is required`);
  }

  function finiteVector(value, label) {
    if (!Array.isArray(value) || !value.length || !value.every(Number.isFinite)) {
      throw new TypeError(`${label} must be a non-empty finite vector`);
    }
    return value;
  }

  function finiteSquare(value, size, label) {
    if (!Array.isArray(value) || value.length !== size || !value.every((row) => (
      Array.isArray(row) && row.length === size && row.every(Number.isFinite)
    ))) throw new TypeError(`${label} must be a finite ${size}x${size} matrix`);
    return value;
  }

  function createRapierFreeClusterAdapter(bindings) {
    [
      'snapshotReplay', 'restoreReplay', 'readSolverGroups', 'writeSolverGroups',
      'resetExternalForces', 'applyExternalForces', 'setIntegrationDt', 'worldStep',
      'captureActuationWork', 'captureFreeState', 'writeReducedVelocity',
    ].forEach((name) => requireFunction(bindings, name));

    let adapterState = {
      stepEpoch: 0,
      forceEpoch: 0,
      qWriteCallEpoch: 0,
      qWriteEpoch: 0,
      lastWritePhysicallyMutated: null,
      lastAdvance: null,
    };

    function snapshot() {
      return {
        replay: bindings.snapshotReplay(),
        adapterState: clone(adapterState),
        solverGroups: clone(bindings.readSolverGroups()),
        exposed: typeof bindings.captureExposed === 'function' ? clone(bindings.captureExposed()) : null,
      };
    }

    function restore(root) {
      if (!root || !root.replay || !root.adapterState) throw new TypeError('invalid S3 replay snapshot');
      const restored = bindings.restoreReplay(root.replay);
      if (restored === false) throw new Error('S3 Rapier replay restore failed');
      adapterState = clone(root.adapterState);
      // World restore normally owns collider groups.  Write them explicitly as
      // well so a partial group mutation cannot survive an exception path.
      bindings.writeSolverGroups(clone(root.solverGroups));
      if (typeof bindings.captureExposed === 'function') {
        const actual = bindings.captureExposed();
        if (canonical(actual) !== canonical(root.exposed)) {
          throw new Error(`S3 replay restore leaked exposed world/body/bookkeeping state; ${firstDifference(root.exposed, actual)}`);
        }
      }
    }

    function advanceFree({ h, event = {}, maskedPairs = [] } = {}) {
      const stepH = Number(h);
      if (!(Number.isFinite(stepH) && stepH > 0)) throw new TypeError('S3 h must be finite and positive');
      const localRoot = snapshot();
      try {
        bindings.resetExternalForces({ h: stepH, event });
        adapterState.forceEpoch += 1;
        const actuation = bindings.applyExternalForces({ h: stepH, event });
        bindings.setIntegrationDt(stepH);
        bindings.worldStep({ h: stepH, event });
        adapterState.stepEpoch += 1;

        // This order is a contract: Wact and the complete free state are
        // captured immediately after the sole world step and before any KKT or
        // velocity writeback is callable.
        const Wact = Number(bindings.captureActuationWork({ h: stepH, event, actuation }));
        if (!Number.isFinite(Wact)) throw new Error('S3 Wact must be finite');
        const free = bindings.captureFreeState({ h: stepH, event, maskedPairs: clone(maskedPairs) });
        const qFree = finiteVector(free?.qFree, 'S3 qFree').slice();
        const Minv = finiteSquare(free?.Minv, qFree.length, 'S3 contactFrame.Minv');
        const contactFrame = {
          ...clone(free.contactFrame || {}),
          Minv: clone(Minv),
          source: 'same-post-world-pre-kkt-sfree',
          h: stepH,
          maskedPairs: clone(maskedPairs),
          sfreeExposed: typeof bindings.captureExposed === 'function'
            ? clone(bindings.captureExposed()) : null,
        };
        adapterState.lastAdvance = {
          h: stepH,
          Wact,
          qFree: qFree.slice(),
          maskedPairs: clone(maskedPairs),
          stepEpoch: adapterState.stepEpoch,
          forceEpoch: adapterState.forceEpoch,
        };
        return deepFreeze({
          qFree,
          Wact,
          contactFrame,
          stepEpoch: adapterState.stepEpoch,
          forceEpoch: adapterState.forceEpoch,
        });
      } catch (error) {
        restore(localRoot);
        throw error;
      }
    }

    const adapter = {
      contractVersion: 1,
      snapshot,
      restore,
      canonicalSnapshot: () => canonical(snapshot()),
      readSolverGroups: () => clone(bindings.readSolverGroups()),
      writeSolverGroups: (groups) => bindings.writeSolverGroups(clone(groups)),
      restoreSolverGroups: (groups) => bindings.writeSolverGroups(clone(groups)),
      advanceFree,
      writeQPost(qPost) {
        finiteVector(qPost, 'S3 qPost');
        adapterState.qWriteCallEpoch += 1;
        const physicallyWritten = bindings.writeReducedVelocity(qPost.slice());
        adapterState.lastWritePhysicallyMutated = physicallyWritten !== false;
        if (physicallyWritten !== false) adapterState.qWriteEpoch += 1;
      },
      readTrace: () => ({
        stepEpoch: adapterState.stepEpoch,
        forceEpoch: adapterState.forceEpoch,
        qWriteCallEpoch: adapterState.qWriteCallEpoch,
        qWriteEpoch: adapterState.qWriteEpoch,
      }),
      readAudit: () => clone(adapterState),
      maskSolverGroups: () => clone(bindings.maskSolverGroups?.() ?? null),
      canonicalExposed: canonical,
    };
    return adapter;
  }

  return Object.freeze({
    contractVersion: 1,
    createRapierFreeClusterAdapter,
    canonicalExposed: canonical,
  });
}));
