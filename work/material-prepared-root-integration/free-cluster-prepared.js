'use strict';

// S4a, work-only: one Sfree state owns one signed least-root preparation.
// The frozen 9BD module remains the public P0 implementation; this module
// adds a separate capability path whose finish cannot re-solve or re-enter
// geometry.  It intentionally does not publish geometry.

const crypto = require('node:crypto');
const frozenP0 = require('../material-free-cluster-integration/free-cluster.js');
const signedCoulombV2 = require('../kkt-coulomb-extension/event-kkt-coulomb.js');
const certifiedCoulomb = require('./event-kkt-coulomb-v3.js');

const PREPARED_FAULT_STAGES = Object.freeze([
  'prepared:advance:before-mask',
  'prepared:advance:after-mask',
  'prepared:advance:after-world',
  'prepared:advance:after-free-capture',
  'prepared:prepare:before-solve',
  'prepared:prepare:after-solve',
  'prepared:prepare:before-publish',
  'prepared:prepare:after-publish',
  'prepared:verify:before',
  'prepared:verify:after',
  'prepared:finish:before-verify',
  'prepared:finish:after-verify',
  'prepared:finish:before-writeback',
  'prepared:finish:after-writeback',
  'prepared:finish:before-ticket',
  'prepared:finish:after-ticket',
  'prepared:finish:before-consume',
  'prepared:structural:before-solve',
  'prepared:structural:after-solve',
  'prepared:structural:before-writeback',
  'prepared:structural:after-writeback',
  'prepared:structural:before-consume',
]);

const SIGNED_COULOMB_SHA256 = '8B3058E5833E96D7C83971B6D0C234E6641D0CB98044012DFFFA5DA2268E1A3E';
const CERTIFIED_COULOMB_SOURCE = 'work/material-prepared-root-integration/event-kkt-coulomb-v3.js';
const CERTIFIED_COULOMB_SHA256 = certifiedCoulomb.selfSha256;

function clone(value) {
  return value === undefined ? undefined : JSON.parse(JSON.stringify(value));
}

function deepFreeze(value) {
  if (!value || typeof value !== 'object' || Object.isFrozen(value)) return value;
  Object.freeze(value);
  Object.values(value).forEach(deepFreeze);
  return value;
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

function sha256(value) {
  return crypto.createHash('sha256').update(canonicalExposed(value)).digest('hex').toUpperCase();
}

function signature(prefix, value) {
  return `${prefix}-${sha256(value)}`;
}

function finiteVector(value, label) {
  if (!Array.isArray(value) || !value.length || !value.every(Number.isFinite)) {
    throw new TypeError(`${label} must be a non-empty finite vector`);
  }
  return value;
}

function freezeContactFrame(contactFrame, qFree) {
  if (!contactFrame || typeof contactFrame !== 'object') {
    throw new TypeError('contactFrame is required');
  }
  finiteVector(qFree, 'qFree');
  if (!Array.isArray(contactFrame.Minv) || contactFrame.Minv.length !== qFree.length
    || !contactFrame.Minv.every((row) => Array.isArray(row) && row.length === qFree.length
      && row.every(Number.isFinite))) {
    throw new TypeError('contactFrame.Minv must be a finite square matrix matching qFree');
  }
  if (!contactFrame.materialContact || !Array.isArray(contactFrame.materialContact.normalRow)) {
    throw new TypeError('contactFrame.materialContact is required');
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
    return {
      id: String(source.id ?? `${source.role || 'contact'}-${index}`),
      role: source.role,
      point: clone(source.point),
      normalRow: clone(source.normalRow),
      tangentRow: clone(source.tangentRow),
      mu,
      // Negative persistent penetration is telemetry, not a separating kick.
      normalBias: Math.max(gap, 0) / h
        + (phase === 'onset' ? restitution * Math.min(0, preNormalVelocity) : 0),
      qaBinding: {
        h,
        gap,
        phase,
        restitution,
        preNormalVelocity,
        restitutionApplied: phase === 'onset' && restitution > 0 && preNormalVelocity < 0,
      },
    };
  });
}

function normalizeCallbackDomain(rawDomain) {
  if (!rawDomain || typeof rawDomain.snapshot !== 'function'
    || typeof rawDomain.restore !== 'function') {
    throw new TypeError('callbackDomain.snapshot/restore are required for prepared material roots');
  }
  return {
    snapshot: rawDomain.snapshot.bind(rawDomain),
    restore: rawDomain.restore.bind(rawDomain),
    canonical: typeof rawDomain.canonical === 'function'
      ? rawDomain.canonical.bind(rawDomain)
      : canonicalExposed,
  };
}

function callbackDomainFor(record, config) {
  return normalizeCallbackDomain(record.callbackDomain || config.callbackDomain);
}

function createPreparedFreeClusterSession(adapter, rawConfig = {}) {
  if (!adapter || typeof adapter.snapshot !== 'function' || typeof adapter.restore !== 'function') {
    throw new TypeError('adapter.snapshot/restore are required');
  }
  if (typeof adapter.advanceFree !== 'function' || typeof adapter.writeQPost !== 'function') {
    throw new TypeError('adapter.advanceFree/writeQPost are required');
  }
  const config = { ...rawConfig };
  if (String(config.trustedCertifiedCoulombSha256 || '') !== CERTIFIED_COULOMB_SHA256) {
    throw new TypeError('trustedCertifiedCoulombSha256 must pin the loaded certified v3 solver');
  }
  const prepareMaterialEvent = certifiedCoulomb.prepareMaterialEvent;
  const solvePrescribedImpulse = signedCoulombV2.solvePrescribedImpulse;
  const active = new Map();
  const preparedOwners = new WeakMap();
  let nextToken = 1;
  let maskEpoch = 0;
  const audit = {
    advanceCalls: 0,
    prepareCalls: 0,
    signedPrepareCalls: 0,
    rawFreshAreaCalls: 0,
    freshAreaCacheHits: 0,
    verifyCalls: 0,
    finishCalls: 0,
    finishSolveCalls: 0,
    finishFreshAreaCalls: 0,
    physicalQWrites: 0,
    structuralSolveCalls: 0,
  };

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
    if (!token || !Number.isInteger(token.id)) throw new Error('invalid prepared Sfree token');
    const record = active.get(token.id);
    if (!record || record.publicToken !== token) throw new Error('foreign or stale prepared Sfree token');
    if (record.consumed) throw new Error(`prepared Sfree token ${token.id} already consumed by ${record.finishedBy}`);
    if (token.sFreeBinding !== record.binding
      || token.contactFrameSignature !== record.contactFrameSignature
      || token.h !== record.h || token.Wact !== record.Wact
      || token.stepEpoch !== record.stepEpoch || token.forceEpoch !== record.forceEpoch
      || canonicalExposed(token.qFree) !== canonicalExposed(record.qFree)) {
      throw new Error('prepared Sfree token binding mismatch');
    }
    return record;
  }

  function captureRoot(record, includeDomain = true) {
    const domain = includeDomain ? callbackDomainFor(record, config) : null;
    const callbackSnapshot = domain ? clone(domain.snapshot()) : undefined;
    return {
      adapter: adapter.snapshot(),
      groups: readGroups(),
      callbackDomain: callbackSnapshot,
      callbackCanonical: domain ? domain.canonical(clone(callbackSnapshot)) : undefined,
      audit: clone(audit),
      consumed: record.consumed,
      finishedBy: record.finishedBy,
    };
  }

  function rollbackRoot(root, record, label, includeDomain = true) {
    const failures = [];
    try { adapter.restore(root.adapter); } catch (error) { failures.push(`adapter: ${error.message}`); }
    if (includeDomain) {
      try { callbackDomainFor(record, config).restore(clone(root.callbackDomain)); }
      catch (error) { failures.push(`callbackDomain: ${error.message}`); }
    }
    Object.keys(audit).forEach((key) => { audit[key] = root.audit[key]; });
    record.consumed = root.consumed;
    record.finishedBy = root.finishedBy;
    try { assertGroups(root.groups, `${label} rollback`); }
    catch (error) { failures.push(error.message); }
    if (failures.length) throw new Error(`${label} rollback mismatch: ${failures.join('; ')}`);
  }

  function solverInput(record, freshArea) {
    const frame = record.contactFrame;
    const rows = normalizeStructuralContacts(frame.structuralContacts || [], record.h);
    return {
      qFree: clone(record.qFree),
      Minv: clone(frame.Minv),
      materialContact: clone(frame.materialContact),
      structuralContacts: rows.map(({ qaBinding, ...contact }) => contact),
      specificCuttingEnergy: frame.specificCuttingEnergy,
      width: frame.width,
      freshArea,
      boundFreshAreaInterval: record.boundFreshAreaInterval,
      intervalOracle: clone(record.intervalOracle),
      trustedGeometryIntervalOracleSha256: record.trustedGeometryIntervalOracleSha256,
      trustedCertifiedCoulombSha256: config.trustedCertifiedCoulombSha256,
      freshAreaIntervalSource: clone(record.freshAreaIntervalSource),
      bindingContext: {
        sFreeBinding: record.binding,
        contactFrameSignature: record.contactFrameSignature,
        h: record.h,
        Wact: record.Wact,
        qFreeSignature: signature('prepared-qfree-v1', record.qFree),
        inverseMassSignature: signature('prepared-minv-v1', frame.Minv),
        materialContactSignature: signature('prepared-material-row-v1', frame.materialContact),
        structuralContactsSignature: signature('prepared-structural-rows-v1', rows),
      },
      options: {
        maximumContacts: config.maximumContacts ?? 6,
        maximumModeCandidates: config.maximumModeCandidates ?? 4096,
      },
    };
  }

  function advanceFree({ h, event = {} }) {
    const stepH = Number(h);
    if (!(Number.isFinite(stepH) && stepH > 0)) throw new TypeError('h must be finite and positive');
    const root = adapter.snapshot();
    const originalGroups = readGroups();
    const traceBefore = typeof adapter.readTrace === 'function' ? adapter.readTrace() : null;
    let free;
    let contactFrame;
    let failure = null;
    try {
      hit('prepared:advance:before-mask');
      writeMask(config.maskSolverGroups ?? null);
      hit('prepared:advance:after-mask');
      free = adapter.advanceFree({ h: stepH, event, maskedPairs: clone(config.maskPlan || []) });
      hit('prepared:advance:after-world');
      if (!free || !Array.isArray(free.qFree) || !Number.isFinite(free.Wact)
        || !free.contactFrame || !Number.isInteger(free.stepEpoch)
        || !Number.isInteger(free.forceEpoch)) {
        throw new Error('adapter.advanceFree must return qFree/Wact/contactFrame/stepEpoch/forceEpoch');
      }
      contactFrame = freezeContactFrame(free.contactFrame, free.qFree);
      if (traceBefore) {
        if (free.stepEpoch !== traceBefore.stepEpoch + 1) {
          throw new Error('advanceFree did not own exactly one world step');
        }
        if (free.forceEpoch !== traceBefore.forceEpoch + 1) {
          throw new Error('advanceFree did not integrate external force exactly once');
        }
      }
      hit('prepared:advance:after-free-capture');
    } catch (error) { failure = error; }
    try { restoreGroups(originalGroups); } catch (error) { failure = failure || error; }
    if (failure) {
      adapter.restore(root);
      throw failure;
    }
    try { assertGroups(originalGroups, 'prepared advanceFree'); }
    catch (error) { adapter.restore(root); throw error; }

    maskEpoch += 1;
    audit.advanceCalls += 1;
    const tokenBody = {
      version: 1,
      id: nextToken,
      h: stepH,
      qFree: clone(free.qFree),
      Wact: free.Wact,
      maskEpoch,
      contactFrameSignature: signature('prepared-contact-frame-v1', contactFrame),
      stepEpoch: free.stepEpoch,
      forceEpoch: free.forceEpoch,
    };
    const binding = signature('prepared-sfree-v1', tokenBody);
    const publicToken = deepFreeze({ ...tokenBody, sFreeBinding: binding });
    active.set(nextToken, {
      publicToken,
      binding,
      h: stepH,
      qFree: clone(free.qFree),
      Wact: free.Wact,
      contactFrame,
      contactFrameSignature: tokenBody.contactFrameSignature,
      stepEpoch: free.stepEpoch,
      forceEpoch: free.forceEpoch,
      root,
      groups: originalGroups,
      freshArea: typeof event.freshArea === 'function' ? event.freshArea : config.freshArea,
      // The interval oracle implementation is application-pinned session
      // configuration. Event data may identify a geometry snapshot but may
      // not replace the trusted bound implementation.
      boundFreshAreaInterval: config.boundFreshAreaInterval,
      intervalOracle: clone(event.intervalOracle || config.intervalOracle || null),
      trustedGeometryIntervalOracleSha256: String(
        config.trustedGeometryIntervalOracleSha256 || '',
      ),
      freshAreaIntervalSource: clone(
        event.freshAreaIntervalSource || config.freshAreaIntervalSource || null,
      ),
      callbackDomain: event.callbackDomain || event.geometryTransaction || config.callbackDomain,
      consumed: false,
      finishedBy: null,
      preparedRoots: new Set(),
    });
    nextToken += 1;
    return publicToken;
  }

  function prepareLeastRoot(token, options = {}) {
    const record = recordFor(token);
    if (Object.keys(options).some((key) => !['label'].includes(key))) {
      throw new Error('prepared root rows, material data and callbacks come only from frozen Sfree');
    }
    if (typeof record.freshArea !== 'function') throw new TypeError('freshArea callback is required');
    const domain = callbackDomainFor(record, config);
    const outer = captureRoot(record, true);
    const adapterBefore = canonicalExposed(outer.adapter);
    const callbackBefore = outer.callbackCanonical;
    const callbackSnapshotSignature = signature(
      'prepared-callback-domain-snapshot-v1',
      outer.callbackDomain,
    );
    const rawCache = new Map();
    const evaluatedSamples = [];
    let published = null;
    let callbackFailure = null;
    audit.prepareCalls += 1;

    if (record.intervalOracle?.geometrySnapshotSignature !== callbackSnapshotSignature) {
      rollbackRoot(outer, record, 'prepareLeastRoot geometry snapshot preflight', true);
      throw new Error('interval oracle geometry snapshot signature does not bind the callback domain root');
    }

    function transactionalFreshArea(p, trial) {
      const key = Object.is(p, -0) ? '0' : Number(p).toPrecision(17);
      const trialSignature = signature('prepared-fresh-trial-v1', trial);
      if (rawCache.has(key)) {
        const cached = rawCache.get(key);
        if (cached.trialSignature !== trialSignature) {
          callbackFailure = new Error('same-p freshArea trial binding changed');
          throw callbackFailure;
        }
        audit.freshAreaCacheHits += 1;
        return clone(cached.raw);
      }
      const callbackState = clone(domain.snapshot());
      const callbackStateCanonical = domain.canonical(clone(callbackState));
      audit.rawFreshAreaCalls += 1;
      let raw;
      try { raw = clone(record.freshArea(p, deepFreeze(clone(trial)))); }
      catch (error) {
        try { domain.restore(clone(callbackState)); } catch (_) { /* outer rollback is authoritative */ }
        callbackFailure = error;
        throw error;
      }
      const afterCanonical = domain.canonical(clone(domain.snapshot()));
      if (afterCanonical !== callbackStateCanonical) {
        domain.restore(clone(callbackState));
        callbackFailure = new Error('freshArea mutated its registered callback/geometry domain');
        throw callbackFailure;
      }
      const entry = deepFreeze({
        p,
        trialSignature,
        raw: clone(raw),
        sampleSignature: signature('prepared-fresh-sample-v1', { p, trial, raw }),
      });
      rawCache.set(key, entry);
      evaluatedSamples.push({
        p,
        trialSignature,
        freshSampleSignature: entry.sampleSignature,
      });
      return clone(raw);
    }

    try {
      hit('prepared:prepare:before-solve');
      audit.signedPrepareCalls += 1;
      const solved = prepareMaterialEvent(solverInput(record, transactionalFreshArea));
      if (callbackFailure) throw callbackFailure;
      hit('prepared:prepare:after-solve');
      if (canonicalExposed(adapter.snapshot()) !== adapterBefore) {
        throw new Error('prepared root solve mutated adapter state');
      }
      if (domain.canonical(clone(domain.snapshot())) !== callbackBefore) {
        throw new Error('prepared root solve leaked callback/geometry state');
      }
      assertGroups(record.groups, 'prepareLeastRoot');
      if (!solved || solved.ok !== true) {
        return deepFreeze({
          ...clone(solved || { ok: false, status: 'solver-domain-stop', reason: 'prepareMaterialEvent returned no result' }),
          prepared: false,
          sFreeBinding: record.binding,
          contactFrameSignature: record.contactFrameSignature,
        });
      }
      const signedValidity = certifiedCoulomb.validatePreparedEvent(solved, {
        trustedGeometryIntervalOracleSha256: record.trustedGeometryIntervalOracleSha256,
        trustedCertifiedCoulombSha256: config.trustedCertifiedCoulombSha256,
      });
      if (!signedValidity.ok) throw new Error(`signed prepared root invalid: ${signedValidity.reason}`);
      const provenance = {
        version: 1,
        sFreeBinding: record.binding,
        contactFrameSignature: record.contactFrameSignature,
        h: record.h,
        Wact: record.Wact,
        qFreeSignature: signature('prepared-qfree-v1', record.qFree),
        inverseMassSignature: signature('prepared-minv-v1', record.contactFrame.Minv),
        materialContactSignature: signature('prepared-material-row-v1', record.contactFrame.materialContact),
        structuralContactsSignature: signature(
          'prepared-structural-rows-v1',
          normalizeStructuralContacts(record.contactFrame.structuralContacts || [], record.h),
        ),
        specificCuttingEnergy: record.contactFrame.specificCuttingEnergy,
        width: record.contactFrame.width,
        signedSolverSha256: SIGNED_COULOMB_SHA256,
        certifiedSolverSource: CERTIFIED_COULOMB_SOURCE,
        certifiedSolverSha256: CERTIFIED_COULOMB_SHA256,
        trustedGeometryIntervalOracleSha256: record.trustedGeometryIntervalOracleSha256,
        signedPreparedBindingSignature: solved.bindingSignature,
      };
      provenance.provenanceSignature = signature('prepared-root-provenance-v1', provenance);
      const publicRoot = deepFreeze({
        ok: true,
        status: 'prepared-least-root',
        version: 1,
        provenance,
        p: solved.p,
        lambda: solved.lambda,
        qPost: clone(solved.qPost),
        modeKey: solved.modeKey,
        activeIds: clone(solved.activeIds),
        contactStates: clone(solved.contactStates),
        contactImpulses: clone(solved.contactImpulses),
        contactPointBindings: clone(solved.contactPointBindings),
        freshArea: solved.freshArea,
        geometryPayload: clone(solved.geometryPayload),
        dissipatedWork: solved.dissipatedWork,
        materialWork: solved.materialWork,
        workResidual: solved.workResidual,
        workSegments: clone(solved.workSegments),
        acceptedSampleSignature: solved.acceptedSampleSignature,
        rootCertificate: clone(solved.rootCertificate),
        leastRootCertificate: clone(solved.leastRootCertificate),
        preparationAudit: {
          algorithm: 'certified-coulomb-prepareMaterialEvent-v3-left-first-interval-root',
          evaluatedFreshSamples: clone(evaluatedSamples),
          signedPreparedBindingSignature: solved.bindingSignature,
        },
      });
      hit('prepared:prepare:before-publish');
      const owner = {
        token,
        record,
        publicCanonical: canonicalExposed(publicRoot),
        signedPrepared: solved,
        acceptedSampleSignature: publicRoot.acceptedSampleSignature,
        provenanceSignature: publicRoot.provenance.provenanceSignature,
      };
      preparedOwners.set(publicRoot, owner);
      record.preparedRoots.add(publicRoot);
      published = publicRoot;
      hit('prepared:prepare:after-publish');
      return publicRoot;
    } catch (error) {
      if (published) {
        preparedOwners.delete(published);
        record.preparedRoots.delete(published);
      }
      try { rollbackRoot(outer, record, 'prepareLeastRoot', true); }
      catch (rollbackError) { throw new Error(`${error.message}; ${rollbackError.message}`); }
      throw error;
    }
  }

  function verifyPreparedRootInternal(token, preparedRoot, countAudit) {
    const record = recordFor(token);
    if (countAudit) audit.verifyCalls += 1;
    const owner = preparedOwners.get(preparedRoot);
    if (!owner || owner.token !== token || owner.record !== record
      || !record.preparedRoots.has(preparedRoot)) {
      throw new Error('foreign, forged or stale prepared root');
    }
    if (canonicalExposed(preparedRoot) !== owner.publicCanonical) {
      throw new Error('prepared root payload changed after preparation');
    }
    if (preparedRoot.provenance.sFreeBinding !== record.binding
      || preparedRoot.provenance.contactFrameSignature !== record.contactFrameSignature
      || preparedRoot.provenance.provenanceSignature !== owner.provenanceSignature
      || preparedRoot.acceptedSampleSignature !== owner.acceptedSampleSignature) {
      throw new Error('prepared root provenance binding mismatch');
    }
    const validity = certifiedCoulomb.validatePreparedEvent(owner.signedPrepared, {
      trustedGeometryIntervalOracleSha256: record.trustedGeometryIntervalOracleSha256,
      trustedCertifiedCoulombSha256: config.trustedCertifiedCoulombSha256,
    });
    if (!validity.ok) throw new Error(`signed prepared root is no longer valid: ${validity.reason}`);
    return { record, owner };
  }

  function verifyPreparedRoot(token, preparedRoot) {
    hit('prepared:verify:before');
    const { owner } = verifyPreparedRootInternal(token, preparedRoot, true);
    hit('prepared:verify:after');
    return deepFreeze({
      ok: true,
      status: 'prepared-root-verified',
      sFreeBinding: token.sFreeBinding,
      provenanceSignature: owner.provenanceSignature,
      acceptedSampleSignature: owner.acceptedSampleSignature,
    });
  }

  function finishMaterial(token, options = {}) {
    if (Object.keys(options).some((key) => key !== 'preparedRoot')) {
      throw new Error('finishMaterial accepts only {preparedRoot}');
    }
    const preparedRoot = options.preparedRoot;
    const record = recordFor(token);
    const outer = captureRoot(record, true);
    audit.finishCalls += 1;
    try {
      hit('prepared:finish:before-verify');
      const { owner } = verifyPreparedRootInternal(token, preparedRoot, false);
      hit('prepared:finish:after-verify');
      // Critical S4a property: no call to prepareMaterialEvent,
      // solvePrescribedImpulse or freshArea occurs below this line.
      hit('prepared:finish:before-writeback');
      adapter.writeQPost(clone(owner.signedPrepared.qPost));
      audit.physicalQWrites += 1;
      hit('prepared:finish:after-writeback');
      hit('prepared:finish:before-ticket');
      const commitTicket = deepFreeze({
        version: 1,
        schema: 'prepared-material-commit-ticket-v1',
        tokenId: token.id,
        sFreeBinding: record.binding,
        contactFrameSignature: record.contactFrameSignature,
        provenanceSignature: owner.provenanceSignature,
        acceptedSampleSignature: owner.acceptedSampleSignature,
        signedPreparedBindingSignature: owner.signedPrepared.bindingSignature,
        Wact: record.Wact,
        h: record.h,
        p: owner.signedPrepared.p,
        lambda: owner.signedPrepared.lambda,
        qPost: clone(owner.signedPrepared.qPost),
        modeKey: owner.signedPrepared.modeKey,
        activeIds: clone(owner.signedPrepared.activeIds),
        contactStates: clone(owner.signedPrepared.contactStates),
        contactImpulses: clone(owner.signedPrepared.contactImpulses),
        contactPointBindings: clone(owner.signedPrepared.contactPointBindings),
        freshArea: owner.signedPrepared.freshArea,
        geometryPayload: clone(owner.signedPrepared.geometryPayload),
        dissipatedWork: owner.signedPrepared.dissipatedWork,
        materialWork: owner.signedPrepared.materialWork,
        workResidual: owner.signedPrepared.workResidual,
        workSegments: clone(owner.signedPrepared.workSegments),
      });
      hit('prepared:finish:after-ticket');
      hit('prepared:finish:before-consume');
      record.consumed = true;
      record.finishedBy = 'prepared-material';
      assertGroups(record.groups, 'prepared finishMaterial');
      return deepFreeze({
        ...clone(owner.signedPrepared),
        status: 'prepared-material-finished',
        finishedBy: 'prepared-material',
        Wact: record.Wact,
        provenanceSignature: owner.provenanceSignature,
        acceptedSampleSignature: owner.acceptedSampleSignature,
        commitTicket,
      });
    } catch (error) {
      try { rollbackRoot(outer, record, 'finishMaterial', true); }
      catch (rollbackError) { throw new Error(`${error.message}; ${rollbackError.message}`); }
      throw error;
    }
  }

  function finishNoMaterial(token, options = {}) {
    const record = recordFor(token);
    if (options.structuralContacts !== undefined || options.materialContact !== undefined) {
      throw new Error('finishNoMaterial rows must come only from the frozen Sfree contactFrame');
    }
    const outer = captureRoot(record, false);
    try {
      hit('prepared:structural:before-solve');
      audit.structuralSolveCalls += 1;
      const frame = record.contactFrame;
      const rows = normalizeStructuralContacts(frame.structuralContacts || [], record.h);
      const result = solvePrescribedImpulse({
        qFree: clone(record.qFree),
        Minv: clone(frame.Minv),
        materialContact: clone(frame.materialContact),
        structuralContacts: rows.map(({ qaBinding, ...contact }) => contact),
        specificCuttingEnergy: frame.specificCuttingEnergy,
        width: frame.width,
        freshArea: () => ({ area: 0, payload: null }),
        options: {
          maximumContacts: config.maximumContacts ?? 6,
          maximumModeCandidates: config.maximumModeCandidates ?? 4096,
        },
      }, 0);
      if (!result.ok) return deepFreeze(clone(result));
      hit('prepared:structural:after-solve');
      hit('prepared:structural:before-writeback');
      adapter.writeQPost(clone(result.qPost));
      audit.physicalQWrites += 1;
      hit('prepared:structural:after-writeback');
      hit('prepared:structural:before-consume');
      record.consumed = true;
      record.finishedBy = 'structural';
      assertGroups(record.groups, 'prepared finishNoMaterial');
      return deepFreeze({ ...clone(result), status: 'structural-finished', Wact: record.Wact });
    } catch (error) {
      try { rollbackRoot(outer, record, 'finishNoMaterial', false); }
      catch (rollbackError) { throw new Error(`${error.message}; ${rollbackError.message}`); }
      throw error;
    }
  }

  function abort(token) {
    const record = recordFor(token);
    adapter.restore(record.root);
    record.consumed = true;
    record.finishedBy = 'aborted';
    assertGroups(record.groups, 'prepared abort');
    return deepFreeze({ ok: true, status: 'aborted', tokenId: token.id });
  }

  return {
    advanceFree,
    prepareLeastRoot,
    verifyPreparedRoot,
    finishMaterial,
    finishNoMaterial,
    abort,
    snapshot: () => adapter.snapshot(),
    audit: () => clone(audit),
    canonicalExposed,
  };
}

module.exports = {
  contractVersion: 2,
  preparedContractVersion: 1,
  signedCoulombSha256: SIGNED_COULOMB_SHA256,
  certifiedCoulombSource: CERTIFIED_COULOMB_SOURCE,
  certifiedCoulombSha256: CERTIFIED_COULOMB_SHA256,
  preparedFaultStages: PREPARED_FAULT_STAGES,
  PREPARED_FAULT_STAGES,
  createPreparedFreeClusterSession,
  createFreeClusterPreparedSession: createPreparedFreeClusterSession,
  canonicalExposed,
  // Preserve the exact signed 9BD P0 surface for its existing conformance.
  createFreeClusterSession: frozenP0.createFreeClusterSession,
  solveClusterFromFree: frozenP0.solveClusterFromFree,
  faultStages: frozenP0.faultStages,
  FAULT_STAGES: frozenP0.FAULT_STAGES,
};
