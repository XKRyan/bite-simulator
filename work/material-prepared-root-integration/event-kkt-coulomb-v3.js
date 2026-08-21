'use strict';

// Work-only signed v3 wrapper over the frozen 8B Coulomb KKT solver.
// Unlike v2, this solver will not claim a least constitutive root from point
// samples alone.  Every interval traversed by the left-first root search must
// carry a geometry-owned conservative area bound. Geometry need not be
// monotone; missing or unverifiable interval evidence is a domain stop.

const crypto = require('node:crypto');
const fs = require('node:fs');
const coulombV2 = require('../kkt-coulomb-extension/event-kkt-coulomb.js');

const PARENT_SOLVER_SHA256 = '8B3058E5833E96D7C83971B6D0C234E6641D0CB98044012DFFFA5DA2268E1A3E';
const SELF_SHA256 = crypto.createHash('sha256')
  .update(fs.readFileSync(__filename))
  .digest('hex').toUpperCase();

const DEFAULT_OPTIONS = Object.freeze({
  ...coulombV2.DEFAULT_OPTIONS,
  maximumCertifiedIntervals: 512,
});
const preparedEventOwners = new WeakMap();

function clone(value) {
  return value === undefined ? undefined : JSON.parse(JSON.stringify(value));
}

function deepFreeze(value) {
  if (!value || typeof value !== 'object' || Object.isFrozen(value)) return value;
  Object.freeze(value);
  Object.values(value).forEach(deepFreeze);
  return value;
}

function canonical(value) {
  if (value === null || typeof value !== 'object') {
    if (typeof value === 'number' && Object.is(value, -0)) return '0';
    return JSON.stringify(value);
  }
  if (Array.isArray(value)) return `[${value.map(canonical).join(',')}]`;
  return `{${Object.keys(value).sort().map((key) => (
    `${JSON.stringify(key)}:${canonical(value[key])}`
  )).join(',')}}`;
}

function hash(value) {
  return crypto.createHash('sha256').update(canonical(value)).digest('hex').toUpperCase();
}

function contractSignature(prefix, value) {
  return `${prefix}-${crypto.createHash('sha256').update(canonical(value)).digest('hex')}`;
}

function signature(prefix, value) {
  return `${prefix}-${hash(value)}`;
}

function finite(value, label) {
  const number = Number(value);
  if (!Number.isFinite(number)) throw new TypeError(`${label} must be finite`);
  return number;
}

function optionsFor(input) {
  return { ...DEFAULT_OPTIONS, ...(input.options || {}) };
}

function effectiveTolerance(absolute, relative, ...values) {
  return absolute + relative * Math.max(1, ...values.map((value) => Math.abs(value)));
}

function normaliseFresh(raw) {
  const value = typeof raw === 'number' ? { area: raw, payload: null } : raw;
  if (!value || typeof value !== 'object') {
    throw new TypeError('freshArea must return a number or {area,payload}');
  }
  const area = finite(value.area, 'fresh area');
  if (area < 0) throw new TypeError('fresh area must be non-negative');
  return { area, payload: clone(value.payload ?? null) };
}

function validateWorkSegments(segments, p, D, tolerance) {
  if (!Array.isArray(segments)) throw new Error('workSegments are required');
  if (p === 0 && segments.length === 0) return;
  let cursor = 0;
  let sum = 0;
  for (const [index, segment] of segments.entries()) {
    if (segment.start !== cursor || !(segment.end >= segment.start)
      || segment.end > p || !Number.isFinite(segment.work) || segment.work < 0
      || !String(segment.modeKey || '')) {
      throw new Error(`work segment ${index} does not form a contiguous [0,p] partition`);
    }
    cursor = segment.end;
    sum += segment.work;
  }
  if (cursor !== p || Math.abs(sum - D) > tolerance) {
    throw new Error('work segments do not exactly bind D(p) over [0,p]');
  }
}

function sampleBindingBody(sample) {
  return {
    p: sample.p,
    D: sample.D,
    freshArea: sample.freshArea,
    materialWork: sample.materialWork,
    F: sample.F,
    qPost: sample.qPost,
    modeKey: sample.modeKey,
    activeIds: sample.activeIds,
    contactStates: sample.contactStates,
    contactImpulses: sample.contactImpulses,
    contactPointBindings: sample.contactPointBindings,
    workSegments: sample.workSegments,
    geometryPayload: sample.geometryPayload,
    endpointTrialSignature: sample.endpointTrialSignature,
    workSegmentsSignature: sample.workSegmentsSignature,
    freshSampleSignature: sample.freshSampleSignature,
  };
}

function intervalBindingBody(interval) {
  return {
    loP: interval.loP,
    hiP: interval.hiP,
    loSampleSignature: interval.loSampleSignature,
    hiSampleSignature: interval.hiSampleSignature,
    areaLower: interval.areaLower,
    areaUpper: interval.areaUpper,
    DLower: interval.DLower,
    DUpper: interval.DUpper,
    FLower: interval.FLower,
    FUpper: interval.FUpper,
    certificateType: interval.certificateType,
    sourceSignature: interval.sourceSignature,
    geometrySnapshotSignature: interval.geometrySnapshotSignature,
    geometryLipschitzSignature: interval.geometryLipschitzSignature,
    areaBoundSignature: interval.areaBoundSignature,
    workBoundSignature: interval.workBoundSignature,
    proof: interval.proof,
  };
}

function preparedBindingBody(prepared) {
  return {
    version: prepared.version,
    parentSolverSha256: prepared.parentSolverSha256,
    p: prepared.p,
    lambda: prepared.lambda,
    qPost: prepared.qPost,
    modeKey: prepared.modeKey,
    activeIds: prepared.activeIds,
    contactStates: prepared.contactStates,
    contactImpulses: prepared.contactImpulses,
    contactPointBindings: prepared.contactPointBindings,
    materialFriction: prepared.materialFriction,
    freshArea: prepared.freshArea,
    geometryPayload: prepared.geometryPayload,
    dissipatedWork: prepared.dissipatedWork,
    materialWork: prepared.materialWork,
    workResidual: prepared.workResidual,
    workSegments: prepared.workSegments,
    acceptedSampleSignature: prepared.acceptedSampleSignature,
    rootCertificate: prepared.rootCertificate,
    leastRootCertificate: prepared.leastRootCertificate,
  };
}

function sameBindingContext(left, right) {
  return canonical(left) === canonical(right);
}

function bindingSignature(body) {
  return `material-coulomb-kkt-v3-${hash(body)}`;
}

function domainStop(reason, details = {}) {
  return deepFreeze({ ok: false, status: 'solver-domain-stop', reason, ...clone(details) });
}

function invalidResult(error) {
  return deepFreeze({
    ok: false,
    status: error instanceof TypeError ? 'invalid-input' : 'solver-domain-stop',
    reason: error instanceof Error ? error.message : String(error),
  });
}

function buildSolverInput(input) {
  if (!input || typeof input !== 'object') throw new TypeError('solver input is required');
  if (typeof input.freshArea !== 'function') throw new TypeError('freshArea callback is required');
  const boundInterval = input.boundFreshAreaInterval || input.freshArea.boundInterval;
  if (typeof boundInterval !== 'function') {
    throw new TypeError('a conservative boundFreshAreaInterval callback is required');
  }
  const intervalOracle = clone(input.intervalOracle || input.freshArea.intervalOracle);
  if (!intervalOracle || intervalOracle.schema !== 'signed-geometry-fresh-area-interval-v1'
    || !String(intervalOracle.moduleSha256 || '')
    || !String(intervalOracle.geometrySnapshotSignature || '')
    || !String(intervalOracle.geometryLipschitzSignature || '')) {
    throw new TypeError('signed geometry interval oracle provenance is required');
  }
  const trustedGeometryIntervalOracleSha256 = String(
    input.trustedGeometryIntervalOracleSha256 || '',
  );
  if (!trustedGeometryIntervalOracleSha256
    || intervalOracle.moduleSha256 !== trustedGeometryIntervalOracleSha256) {
    throw new TypeError('interval oracle module hash is not the pinned trusted geometry provider');
  }
  const trustedCertifiedCoulombSha256 = String(input.trustedCertifiedCoulombSha256 || '');
  if (trustedCertifiedCoulombSha256 !== SELF_SHA256) {
    throw new TypeError('trustedCertifiedCoulombSha256 does not match the loaded v3 solver');
  }
  const specificCuttingEnergy = finite(input.specificCuttingEnergy, 'specificCuttingEnergy');
  const width = finite(input.width, 'width');
  if (!(specificCuttingEnergy > 0) || !(width > 0)) {
    throw new TypeError('specificCuttingEnergy and width must be positive');
  }
  const options = optionsFor(input);
  const bindingContext = clone(input.bindingContext);
  if (!bindingContext || !String(bindingContext.sFreeBinding || '')
    || !String(bindingContext.contactFrameSignature || '')
    || !String(bindingContext.qFreeSignature || '')
    || !String(bindingContext.inverseMassSignature || '')
    || !String(bindingContext.materialContactSignature || '')
    || !String(bindingContext.structuralContactsSignature || '')
    || !(Number.isFinite(bindingContext.h) && bindingContext.h > 0)
    || !Number.isFinite(bindingContext.Wact)) {
    throw new TypeError('complete frozen Sfree/frame bindingContext is required');
  }
  return {
    qFree: clone(input.qFree),
    Minv: clone(input.Minv),
    materialContact: clone(input.materialContact),
    structuralContacts: clone(input.structuralContacts || []),
    specificCuttingEnergy,
    width,
    freshArea: input.freshArea,
    boundInterval,
    intervalOracle,
    trustedGeometryIntervalOracleSha256,
    trustedCertifiedCoulombSha256,
    bindingContext,
    intervalSource: clone(input.freshAreaIntervalSource || null),
    options,
  };
}

function prepareMaterialEvent(input) {
  let problem;
  try { problem = buildSolverInput(input); }
  catch (error) { return invalidResult(error); }

  const { options } = problem;
  const sampleCache = new Map();
  const intervalCache = new Map();
  const visitedTree = [];
  const prefixLeaves = [];
  let exactFreshAreaCalls = 0;
  let intervalBoundCalls = 0;

  const keyOf = (p) => (Object.is(p, -0) ? '0' : Number(p).toPrecision(17));

  function publicTrial(solved) {
    return deepFreeze(clone({
      p: solved.p,
      lambda: solved.lambda,
      qPost: solved.qPost,
      modeKey: solved.modeKey,
      activeIds: solved.activeIds,
      contactStates: solved.contactStates,
      contactImpulses: solved.contactImpulses,
      contactPointBindings: solved.contactPointBindings,
      materialPoint: solved.contactPointBindings?.material?.point
        || problem.materialContact.point,
    }));
  }

  function solveAt(p) {
    const solved = coulombV2.solvePrescribedImpulse({
      qFree: problem.qFree,
      Minv: problem.Minv,
      materialContact: problem.materialContact,
      structuralContacts: problem.structuralContacts,
      specificCuttingEnergy: problem.specificCuttingEnergy,
      width: problem.width,
      // v2 solvePrescribedImpulse validates but never evaluates this callback.
      freshArea: () => ({ area: 0, payload: null }),
      options: problem.options,
    }, p);
    if (!solved.ok) throw new Error(`signed Coulomb solve failed at p=${p}: ${solved.reason}`);
    return solved;
  }

  function evaluate(p) {
    const key = keyOf(p);
    if (sampleCache.has(key)) return sampleCache.get(key);
    const solved = solveAt(p);
    exactFreshAreaCalls += 1;
    const fresh = normaliseFresh(problem.freshArea(p, publicTrial(solved)));
    const materialWork = problem.specificCuttingEnergy * problem.width * fresh.area;
    const D = solved.dissipatedWork;
    const F = D - materialWork;
    const tolerance = effectiveTolerance(
      options.workAbsoluteTolerance,
      options.workRelativeTolerance,
      D,
      materialWork,
    );
    validateWorkSegments(solved.workSegments, p, D, tolerance);
    const workSegments = clone(solved.workSegments);
    const endpointTrialSignature = signature('material-v3-endpoint-trial-v1', {
      p,
      qPost: solved.qPost,
      modeKey: solved.modeKey,
      activeIds: solved.activeIds,
      contactStates: solved.contactStates,
      contactImpulses: solved.contactImpulses,
      contactPointBindings: solved.contactPointBindings,
    });
    const freshSampleSignature = signature('material-v3-fresh-sample-v1', {
      p,
      area: fresh.area,
      payload: fresh.payload,
      endpointTrialSignature,
    });
    const sample = {
      p,
      D,
      freshArea: fresh.area,
      materialWork,
      F,
      qPost: clone(solved.qPost),
      modeKey: solved.modeKey,
      activeIds: clone(solved.activeIds),
      contactStates: clone(solved.contactStates),
      contactImpulses: clone(solved.contactImpulses),
      contactPointBindings: clone(solved.contactPointBindings),
      materialFriction: clone(solved.materialFriction),
      materialSpeed: solved.materialSpeed,
      kineticEnergy: solved.kineticEnergy,
      structuralFrictionDissipation: solved.structuralFrictionDissipation,
      maximumDissipation: clone(solved.maximumDissipation),
      workSegments,
      geometryPayload: clone(fresh.payload),
      endpointTrialSignature,
      workSegmentsSignature: signature('material-v3-work-segments-v1', workSegments),
      freshSampleSignature,
    };
    sample.sampleSignature = signature('material-v3-sample-v1', sampleBindingBody(sample));
    const frozen = deepFreeze(sample);
    sampleCache.set(key, frozen);
    return frozen;
  }

  function certifyInterval(left, right) {
    if (!(right.p > left.p)) throw new Error('certified interval must have positive width');
    const key = `${keyOf(left.p)}:${keyOf(right.p)}`;
    if (intervalCache.has(key)) return intervalCache.get(key);
    intervalBoundCalls += 1;
    const context = deepFreeze({
      version: 1,
      pLo: left.p,
      pHi: right.p,
      left: clone({
        p: left.p,
        area: left.freshArea,
        D: left.D,
        F: left.F,
        modeKey: left.modeKey,
        sampleSignature: left.sampleSignature,
      }),
      right: clone({
        p: right.p,
        area: right.freshArea,
        D: right.D,
        F: right.F,
        modeKey: right.modeKey,
        sampleSignature: right.sampleSignature,
      }),
      specificCuttingEnergy: problem.specificCuttingEnergy,
      width: problem.width,
      source: clone(problem.intervalSource),
    });
    const raw = problem.boundInterval(left.p, right.p, context);
    if (!raw || typeof raw !== 'object') {
      throw new Error('boundFreshAreaInterval returned no certificate');
    }
    if (raw.moduleSha256 !== problem.intervalOracle.moduleSha256) {
      throw new Error('interval oracle module hash mismatch');
    }
    const areaLower = finite(raw.areaLower ?? raw.minArea, 'interval areaLower');
    const areaUpper = finite(raw.areaUpper ?? raw.maxArea, 'interval areaUpper');
    if (areaLower < 0 || areaUpper < areaLower) {
      throw new Error('interval area bounds are invalid');
    }
    const certificateType = String(raw.certificateType || '');
    const sourceSignature = String(raw.sourceSignature || '');
    if (!/(analytic|interval|lipschitz|exact)/i.test(certificateType)
      || !sourceSignature || !raw.proof || typeof raw.proof !== 'object') {
      throw new Error('interval requires sourced conservative geometry bounds');
    }
    const areaTolerance = effectiveTolerance(
      options.areaAbsoluteTolerance,
      options.areaRelativeTolerance,
      left.freshArea,
      right.freshArea,
      areaLower,
      areaUpper,
    );
    if (areaLower > Math.min(left.freshArea, right.freshArea) + areaTolerance
      || areaUpper < Math.max(left.freshArea, right.freshArea) - areaTolerance) {
      throw new Error('interval area bounds do not contain their exact endpoints');
    }
    const DLower = left.D;
    const DUpper = right.D;
    const workTolerance = effectiveTolerance(
      options.workAbsoluteTolerance,
      options.workRelativeTolerance,
      left.D,
      right.D,
    );
    if (DUpper < DLower - workTolerance) throw new Error('D(p) decreased over a certified interval');
    const FLower = DLower - problem.specificCuttingEnergy * problem.width * areaUpper;
    const FUpper = DUpper - problem.specificCuttingEnergy * problem.width * areaLower;
    const geometrySnapshotSignature = String(raw.geometrySnapshotSignature || '');
    const geometryLipschitzSignature = String(raw.geometryLipschitzSignature || '');
    if (geometrySnapshotSignature !== problem.intervalOracle.geometrySnapshotSignature
      || geometryLipschitzSignature !== problem.intervalOracle.geometryLipschitzSignature) {
      throw new Error('interval geometry snapshot/Lipschitz provenance mismatch');
    }
    const areaBoundSignature = signature('geometry-fresh-area-bound-v1', {
      loP: left.p,
      hiP: right.p,
      areaLower,
      areaUpper,
      certificateType,
      sourceSignature,
      geometrySnapshotSignature,
      geometryLipschitzSignature,
      proof: raw.proof,
    });
    if (raw.areaBoundSignature && raw.areaBoundSignature !== areaBoundSignature) {
      throw new Error('geometry area-bound signature mismatch');
    }
    const workBoundSignature = signature('signed-coulomb-work-bound-v1', {
      loP: left.p,
      hiP: right.p,
      DLower,
      DUpper,
      loWorkSegmentsSignature: left.workSegmentsSignature,
      hiWorkSegmentsSignature: right.workSegmentsSignature,
      parentSolverSha256: PARENT_SOLVER_SHA256,
      bindingContext: clone(problem.bindingContext),
    });
    const interval = {
      loP: left.p,
      hiP: right.p,
      loSampleSignature: left.sampleSignature,
      hiSampleSignature: right.sampleSignature,
      areaLower,
      areaUpper,
      DLower,
      DUpper,
      FLower,
      FUpper,
      certificateType,
      sourceSignature,
      geometrySnapshotSignature,
      geometryLipschitzSignature,
      areaBoundSignature,
      workBoundSignature,
      proof: clone(raw.proof),
    };
    interval.intervalSignature = signature('material-v3-interval-v1', intervalBindingBody(interval));
    const frozen = deepFreeze(interval);
    intervalCache.set(key, frozen);
    return frozen;
  }

  let zero;
  let stoppingP;
  try {
    zero = evaluate(0);
    if (!(zero.materialSpeed > options.velocityTolerance)) {
      return domainStop('material normal has no positive cutting speed after the simultaneous Coulomb KKT solve');
    }
    const zeroTolerance = effectiveTolerance(
      options.workAbsoluteTolerance,
      options.workRelativeTolerance,
      zero.D,
      zero.materialWork,
    );
    if (!(zero.F < -zeroTolerance) || !(zero.freshArea > 0)) {
      return domainStop('least positive material root requires strictly negative F(0) and positive virgin area');
    }
    let upper = 1e-12;
    let upperSample = null;
    for (let iteration = 0; iteration < options.maximumBracketIterations; iteration += 1) {
      upperSample = evaluate(upper);
      if (upperSample.materialSpeed <= options.velocityTolerance) break;
      upper *= 2;
    }
    if (!upperSample || upperSample.materialSpeed > options.velocityTolerance) {
      return domainStop('no finite material stopping impulse could be bracketed inside the Coulomb domain');
    }
    let stopLow = 0;
    let stopHigh = upper;
    for (let iteration = 0; iteration < options.maximumRootIterations; iteration += 1) {
      const middle = stopLow + (stopHigh - stopLow) / 2;
      const trial = evaluate(middle);
      if (trial.materialSpeed > options.velocityTolerance) stopLow = middle;
      else stopHigh = middle;
      const tolerance = effectiveTolerance(
        options.lambdaAbsoluteTolerance,
        options.lambdaRelativeTolerance,
        stopHigh,
      );
      if (stopHigh - stopLow <= tolerance) break;
    }
    stoppingP = stopHigh;
  } catch (error) { return invalidResult(error); }

  let acceptedBracket = null;
  let unresolvedReason = null;
  let visitedIntervals = 0;

  function search(left, right, depth) {
    if (visitedIntervals >= options.maximumCertifiedIntervals) {
      unresolvedReason = 'certified interval budget exhausted before the least root was isolated';
      return 'unresolved';
    }
    let interval;
    try { interval = certifyInterval(left, right); }
    catch (error) {
      unresolvedReason = error.message;
      return 'unresolved';
    }
    visitedIntervals += 1;
    const residualTolerance = effectiveTolerance(
      options.workAbsoluteTolerance,
      options.workRelativeTolerance,
      left.D,
      right.D,
      left.materialWork,
      right.materialWork,
    );
    const impulseTolerance = effectiveTolerance(
      options.lambdaAbsoluteTolerance,
      options.lambdaRelativeTolerance,
      right.p,
    );
    const node = {
      visit: visitedTree.length,
      depth,
      loP: left.p,
      hiP: right.p,
      loSampleSignature: left.sampleSignature,
      hiSampleSignature: right.sampleSignature,
      intervalSignature: interval.intervalSignature,
      FLower: interval.FLower,
      FUpper: interval.FUpper,
      status: 'open',
    };
    visitedTree.push(node);
    if (interval.FUpper < -residualTolerance) {
      node.status = 'root-excluded-negative';
      const leaf = {
        lo: interval.loP,
        hi: interval.hiP,
        fMin: interval.FLower,
        fMax: interval.FUpper,
        FLower: interval.FLower,
        FUpper: interval.FUpper,
        boundSource: 'signed-geometry-interval-oracle-v1',
        geometrySnapshotSignature: interval.geometrySnapshotSignature,
        geometryLipschitzSignature: interval.geometryLipschitzSignature,
        areaBoundSignature: interval.areaBoundSignature,
        workBoundSignature: interval.workBoundSignature,
        intervalSignature: interval.intervalSignature,
        certificateType: interval.certificateType,
        sourceSignature: interval.sourceSignature,
        bindingContext: clone(problem.bindingContext),
        rootExcluded: true,
        exclusion: 'fMax < -residualTolerance',
      };
      leaf.boundSignature = contractSignature('least-root-interval-leaf-v1', leaf);
      prefixLeaves.push(leaf);
      return 'excluded';
    }
    if (right.p - left.p <= impulseTolerance) {
      if (left.F <= residualTolerance && right.F >= -residualTolerance
        && Math.abs(right.F) <= residualTolerance) {
        node.status = 'accepted-first-bracket';
        acceptedBracket = { left, right, interval, residualTolerance, impulseTolerance };
        return 'accepted';
      }
      // Width may be made smaller than the declared p tolerance when a
      // tangential root needs a tighter residual. Only machine precision or
      // the interval budget may stop this left-first refinement.
      const middleP = left.p + (right.p - left.p) / 2;
      if (!(middleP > left.p && middleP < right.p)) {
        node.status = 'unresolved-machine-precision';
        unresolvedReason = 'an interval contains zero but cannot be refined to a bound endpoint sample';
        return 'unresolved';
      }
    }
    let middle;
    try { middle = evaluate(left.p + (right.p - left.p) / 2); }
    catch (error) {
      node.status = 'unresolved-sample';
      unresolvedReason = error.message;
      return 'unresolved';
    }
    node.status = 'split-left-first';
    node.middleP = middle.p;
    node.middleSampleSignature = middle.sampleSignature;
    const leftStatus = search(left, middle, depth + 1);
    if (leftStatus !== 'excluded') return leftStatus;
    return search(middle, right, depth + 1);
  }

  try {
    const stopping = evaluate(stoppingP);
    const stoppingTolerance = effectiveTolerance(
      options.workAbsoluteTolerance,
      options.workRelativeTolerance,
      stopping.D,
      stopping.materialWork,
    );
    // A negative stopping-endpoint residual does not prove root absence when
    // the trusted geometry oracle permits non-monotone fresh area. A narrow
    // interior tangency can reach F=0 and return negative before stopping.
    // Only the conservative left-first interval tree may exclude or accept a
    // root; endpoint affordability remains diagnostic data.
    const searchStatus = search(zero, stopping, 0);
    if (searchStatus !== 'accepted' || !acceptedBracket) {
      return domainStop(
        `least-root interval proof is unresolved: ${unresolvedReason || 'no certified first bracket'}`,
        { stoppingP, visitedIntervals, stoppingEndpointDiagnostic: {
          F: stopping.F,
          tolerance: stoppingTolerance,
          maximumDissipatableWork: stopping.D,
          requiredMaterialWork: stopping.materialWork,
        } },
      );
    }

    const orderedSamples = [...sampleCache.values()].sort((left, right) => left.p - right.p);
    const monotonicAudit = [];
    for (let index = 0; index < orderedSamples.length - 1; index += 1) {
      const left = orderedSamples[index];
      const right = orderedSamples[index + 1];
      const interval = certifyInterval(left, right);
      const residualTolerance = effectiveTolerance(
        options.workAbsoluteTolerance,
        options.workRelativeTolerance,
        left.D,
        right.D,
        left.materialWork,
        right.materialWork,
      );
      const audit = {
        loP: left.p,
        hiP: right.p,
        deltaD: right.D - left.D,
        deltaFreshArea: right.freshArea - left.freshArea,
        deltaMaterialWork: right.materialWork - left.materialWork,
        deltaF: right.F - left.F,
        tolerance: residualTolerance,
        intervalSignature: interval.intervalSignature,
        DNonDecreasing: right.D + residualTolerance >= left.D,
        areaNonIncreasing: right.freshArea <= left.freshArea
          + effectiveTolerance(options.areaAbsoluteTolerance, options.areaRelativeTolerance, left.freshArea, right.freshArea),
        requiredWorkNonIncreasing: right.materialWork <= left.materialWork + residualTolerance,
        FNonDecreasing: right.F + residualTolerance >= left.F,
      };
      // These deltas are diagnostics only. Root absence is established by
      // conservative interval ranges, never by point-sample monotonicity.
      if (!audit.DNonDecreasing) return domainStop('signed material dissipation D(p) decreased');
      monotonicAudit.push(audit);
    }

    const accepted = acceptedBracket.right;
    const loIndex = orderedSamples.findIndex((sample) => sample.sampleSignature === acceptedBracket.left.sampleSignature);
    const hiIndex = orderedSamples.findIndex((sample) => sample.sampleSignature === accepted.sampleSignature);
    if (loIndex < 0 || hiIndex !== loIndex + 1) {
      return domainStop('accepted least-root bracket is not formed by consecutive bound samples');
    }
    let prefixCursor = 0;
    for (const leaf of prefixLeaves) {
      if (leaf.lo !== prefixCursor || leaf.hi > acceptedBracket.left.p
        || leaf.fMax >= -acceptedBracket.residualTolerance) {
        return domainStop('root-excluded prefix leaves do not form an ordered conservative cover');
      }
      prefixCursor = leaf.hi;
    }
    if (prefixCursor !== acceptedBracket.left.p) {
      return domainStop('root-excluded prefix has a gap before the accepted bracket');
    }

    const certificateSamples = orderedSamples.map((sample, index) => {
      const entry = clone(sample);
      const adjacent = monotonicAudit[index];
      if (adjacent) {
        const interval = intervalCache.get(`${keyOf(sample.p)}:${keyOf(orderedSamples[index + 1].p)}`);
        entry.intervalToNext = {
          intervalSignature: interval.intervalSignature,
          certificateType: interval.certificateType,
          sourceSignature: interval.sourceSignature,
          rootExcluded: interval.hiP <= acceptedBracket.left.p
            && interval.FUpper < -acceptedBracket.residualTolerance,
          FLower: interval.FLower,
          FUpper: interval.FUpper,
        };
      }
      return entry;
    });
    const rootCertificate = {
      version: 1,
      schema: 'material-coulomb-least-root-certificate-v3',
      parentSolverSha256: PARENT_SOLVER_SHA256,
      bindingContext: clone(problem.bindingContext),
      specificCuttingEnergy: problem.specificCuttingEnergy,
      width: problem.width,
      searchDomain: [0, stoppingP],
      stoppingP,
      evaluatedSamples: certificateSamples,
      acceptedBracket: {
        loIndex,
        hiIndex,
        loP: acceptedBracket.left.p,
        hiP: accepted.p,
        loF: acceptedBracket.left.F,
        hiF: accepted.F,
        loSampleSignature: acceptedBracket.left.sampleSignature,
        hiSampleSignature: accepted.sampleSignature,
        intervalSignature: acceptedBracket.interval.intervalSignature,
        lo: clone(acceptedBracket.left),
        hi: clone(accepted),
      },
      residualTolerance: {
        absolute: acceptedBracket.residualTolerance,
        baseAbsolute: options.workAbsoluteTolerance,
        relative: options.workRelativeTolerance,
        effective: acceptedBracket.residualTolerance,
      },
      impulseTolerance: {
        absolute: acceptedBracket.impulseTolerance,
        baseAbsolute: options.lambdaAbsoluteTolerance,
        relative: options.lambdaRelativeTolerance,
        effective: acceptedBracket.impulseTolerance,
      },
      prefixExclusionLeaves: clone(prefixLeaves),
      earlierIntervalTree: {
        traversal: 'left-first',
        coverage: [0, acceptedBracket.left.p],
        exclusionLeaves: clone(prefixLeaves),
        visitedNodes: clone(visitedTree),
      },
      visitedIntervalTree: clone(visitedTree),
      monotonicAudit,
      exactFreshAreaCalls,
      intervalBoundCalls,
      intervalSource: clone(problem.intervalSource),
      intervalOracle: clone(problem.intervalOracle),
      trustedGeometryIntervalOracleSha256: problem.trustedGeometryIntervalOracleSha256,
      trustedCertifiedCoulombSha256: problem.trustedCertifiedCoulombSha256,
      acceptedSampleSignature: accepted.sampleSignature,
      acceptedSample: clone(accepted),
    };
    rootCertificate.certificateSignature = signature(
      'material-v3-least-root-certificate-v1',
      rootCertificate,
    );

    const prepared = {
      ok: true,
      status: 'prepared',
      version: 3,
      parentSolverSha256: PARENT_SOLVER_SHA256,
      p: accepted.p,
      lambda: accepted.p,
      qPost: clone(accepted.qPost),
      modeKey: accepted.modeKey,
      activeIds: clone(accepted.activeIds),
      contactStates: clone(accepted.contactStates),
      contactImpulses: clone(accepted.contactImpulses),
      contactPointBindings: clone(accepted.contactPointBindings),
      materialFriction: clone(accepted.materialFriction),
      freshArea: accepted.freshArea,
      geometryPayload: clone(accepted.geometryPayload),
      dissipatedWork: accepted.D,
      materialWork: accepted.materialWork,
      workResidual: accepted.F,
      stoppingP,
      materialSpeedPost: accepted.materialSpeed,
      workSegments: clone(accepted.workSegments),
      acceptedSampleSignature: accepted.sampleSignature,
      rootCertificate,
      leastRootCertificate: rootCertificate,
      model: 'certified-left-first-least-root-over-signed-v2-coulomb-active-mode-work',
    };
    prepared.bindingSignature = bindingSignature(preparedBindingBody(prepared));
    const frozenPrepared = deepFreeze(prepared);
    preparedEventOwners.set(frozenPrepared, Object.freeze({
      trustedGeometryIntervalOracleSha256: problem.trustedGeometryIntervalOracleSha256,
      trustedCertifiedCoulombSha256: problem.trustedCertifiedCoulombSha256,
      certificateIdentity: frozenPrepared.rootCertificate,
    }));
    return frozenPrepared;
  } catch (error) { return invalidResult(error); }
}

function validateSample(sample, Uc, width, residualTolerance) {
  const required = ['p', 'D', 'freshArea', 'materialWork', 'F'];
  if (!sample || !required.every((key) => Number.isFinite(sample[key]))) {
    return 'certificate sample lacks finite p/D/A/W/F';
  }
  if (Math.abs(sample.materialWork - Uc * width * sample.freshArea) > residualTolerance) {
    return 'certificate material-work identity failed';
  }
  if (Math.abs(sample.F - (sample.D - sample.materialWork)) > residualTolerance) {
    return 'certificate residual identity failed';
  }
  try { validateWorkSegments(sample.workSegments, sample.p, sample.D, residualTolerance); }
  catch (error) { return error.message; }
  if (sample.workSegmentsSignature !== signature('material-v3-work-segments-v1', sample.workSegments)) {
    return 'certificate work-segment signature mismatch';
  }
  if (sample.freshSampleSignature !== signature('material-v3-fresh-sample-v1', {
    p: sample.p,
    area: sample.freshArea,
    payload: sample.geometryPayload,
    endpointTrialSignature: sample.endpointTrialSignature,
  })) return 'certificate fresh-sample signature mismatch';
  if (sample.sampleSignature !== signature('material-v3-sample-v1', sampleBindingBody(sample))) {
    return 'certificate sample signature mismatch';
  }
  return null;
}

function validatePreparedEvent(prepared, trust = {}) {
  try {
    const owner = preparedEventOwners.get(prepared);
    if (!owner || owner.certificateIdentity !== prepared?.rootCertificate) {
      return { ok: false, reason: 'prepared event is foreign, cloned, stale, or not owned by this solver instance' };
    }
    if (!prepared || prepared.ok !== true || prepared.status !== 'prepared' || prepared.version !== 3) {
      return { ok: false, reason: 'not a prepared certified Coulomb material event v3' };
    }
    if (prepared.parentSolverSha256 !== PARENT_SOLVER_SHA256 || prepared.p !== prepared.lambda) {
      return { ok: false, reason: 'prepared parent solver or same-p binding is broken' };
    }
    if (bindingSignature(preparedBindingBody(prepared)) !== prepared.bindingSignature) {
      return { ok: false, reason: 'prepared v3 event binding mismatch' };
    }
    const certificate = prepared.rootCertificate;
    if (!certificate || certificate.version !== 1
      || certificate.schema !== 'material-coulomb-least-root-certificate-v3') {
      return { ok: false, reason: 'least-root certificate v3 is required' };
    }
    const expectedGeometrySha256 = String(trust.trustedGeometryIntervalOracleSha256 || '');
    const expectedCertifiedSha256 = String(trust.trustedCertifiedCoulombSha256 || '');
    if (!/^[A-F0-9]{64}$/i.test(expectedGeometrySha256)
      || expectedCertifiedSha256 !== SELF_SHA256) {
      return { ok: false, reason: 'external geometry-oracle and certified-solver SHA-256 pins are required' };
    }
    if (owner.trustedGeometryIntervalOracleSha256 !== expectedGeometrySha256
      || owner.trustedCertifiedCoulombSha256 !== expectedCertifiedSha256) {
      return { ok: false, reason: 'prepared event owner record does not match the external trust pins' };
    }
    if (!String(certificate.trustedGeometryIntervalOracleSha256 || '')
      || certificate.intervalOracle?.moduleSha256
        !== certificate.trustedGeometryIntervalOracleSha256
      || certificate.trustedGeometryIntervalOracleSha256 !== expectedGeometrySha256) {
      return { ok: false, reason: 'least-root certificate geometry provider hash is not pinned' };
    }
    if (certificate.trustedCertifiedCoulombSha256 !== expectedCertifiedSha256) {
      return { ok: false, reason: 'least-root certificate certified solver hash is not externally pinned' };
    }
    const bindingContext = certificate.bindingContext;
    if (!bindingContext || !String(bindingContext.sFreeBinding || '')
      || !String(bindingContext.contactFrameSignature || '')
      || !String(bindingContext.qFreeSignature || '')
      || !String(bindingContext.inverseMassSignature || '')
      || !String(bindingContext.materialContactSignature || '')
      || !String(bindingContext.structuralContactsSignature || '')
      || !(Number.isFinite(bindingContext.h) && bindingContext.h > 0)
      || !Number.isFinite(bindingContext.Wact)) {
      return { ok: false, reason: 'least-root certificate Sfree/frame binding is incomplete' };
    }
    for (const leaf of certificate.prefixExclusionLeaves || []) {
      if (!sameBindingContext(leaf.bindingContext, bindingContext)) {
        return { ok: false, reason: 'prefix leaf Sfree/frame binding mismatch' };
      }
    }
    if (canonical(prepared.leastRootCertificate) !== canonical(certificate)) {
      return { ok: false, reason: 'rootCertificate/leastRootCertificate binding mismatch' };
    }
    const certificateClone = clone(certificate);
    delete certificateClone.certificateSignature;
    if (certificate.certificateSignature !== signature(
      'material-v3-least-root-certificate-v1',
      certificateClone,
    )) return { ok: false, reason: 'least-root certificate signature mismatch' };
    const Uc = certificate.specificCuttingEnergy;
    const width = certificate.width;
    if (!(Number.isFinite(Uc) && Uc > 0 && Number.isFinite(width) && width > 0)) {
      return { ok: false, reason: 'certificate Uc and width must be positive' };
    }
    const residualTolerance = Number(certificate.residualTolerance?.effective);
    const impulseTolerance = Number(certificate.impulseTolerance?.effective);
    if (!(Number.isFinite(residualTolerance) && residualTolerance >= 0
      && Number.isFinite(impulseTolerance) && impulseTolerance >= 0)) {
      return { ok: false, reason: 'certificate effective tolerances are invalid' };
    }
    const samples = certificate.evaluatedSamples;
    if (!Array.isArray(samples) || samples.length < 2) {
      return { ok: false, reason: 'ordered evaluated root samples are required' };
    }
    for (let index = 0; index < samples.length; index += 1) {
      if (index && !(samples[index].p > samples[index - 1].p)) {
        return { ok: false, reason: 'evaluated root samples are not strictly ordered and unique' };
      }
      const failure = validateSample(samples[index], Uc, width, residualTolerance);
      if (failure) return { ok: false, reason: failure };
      if (index) {
        const left = samples[index - 1];
        const right = samples[index];
        if (right.D + residualTolerance < left.D) {
          return { ok: false, reason: 'evaluated samples violate signed D(p) order' };
        }
      }
    }
    const bracket = certificate.acceptedBracket;
    if (!bracket || !Number.isInteger(bracket.loIndex) || !Number.isInteger(bracket.hiIndex)
      || bracket.hiIndex !== bracket.loIndex + 1) {
      return { ok: false, reason: 'accepted bracket must bind consecutive samples' };
    }
    const lo = samples[bracket.loIndex];
    const hi = samples[bracket.hiIndex];
    if (!lo || !hi || bracket.loP !== lo.p || bracket.hiP !== hi.p
      || bracket.loF !== lo.F || bracket.hiF !== hi.F
      || bracket.loSampleSignature !== lo.sampleSignature
      || bracket.hiSampleSignature !== hi.sampleSignature) {
      return { ok: false, reason: 'accepted bracket endpoint binding mismatch' };
    }
    if (bracket.lo.sampleSignature !== lo.sampleSignature
      || bracket.hi.sampleSignature !== hi.sampleSignature
      || canonical(sampleBindingBody(bracket.lo)) !== canonical(sampleBindingBody(lo))
      || canonical(sampleBindingBody(bracket.hi)) !== canonical(sampleBindingBody(hi))) {
      return { ok: false, reason: 'accepted bracket full endpoint samples mismatch' };
    }
    if (!(lo.F <= residualTolerance && hi.F >= -residualTolerance)
      || hi.p - lo.p > impulseTolerance || Math.abs(hi.F) > residualTolerance) {
      return { ok: false, reason: 'accepted bracket sign, width or residual is invalid' };
    }
    if (prepared.p !== hi.p || prepared.acceptedSampleSignature !== hi.sampleSignature
      || certificate.acceptedSampleSignature !== hi.sampleSignature
      || prepared.modeKey !== hi.modeKey || canonical(prepared.qPost) !== canonical(hi.qPost)
      || prepared.dissipatedWork !== hi.D || prepared.freshArea !== hi.freshArea
      || prepared.materialWork !== hi.materialWork || prepared.workResidual !== hi.F
      || canonical(prepared.geometryPayload) !== canonical(hi.geometryPayload)
      || canonical(prepared.workSegments) !== canonical(hi.workSegments)) {
      return { ok: false, reason: 'prepared endpoint is not the accepted high sample' };
    }
    if (hi.p > 0 && hi.workSegments.at(-1)?.modeKey !== hi.modeKey) {
      return { ok: false, reason: 'accepted endpoint mode/work signature mismatch' };
    }
    const leaves = certificate.prefixExclusionLeaves;
    if (!Array.isArray(leaves)) return { ok: false, reason: 'prefix exclusion leaves are required' };
    let cursor = 0;
    for (const leaf of leaves) {
      const leafBody = clone(leaf);
      delete leafBody.boundSignature;
      if (leaf.lo !== cursor || !(leaf.hi > leaf.lo)
        || leaf.rootExcluded !== true || leaf.FUpper >= -residualTolerance
        || !String(leaf.certificateType || '').match(/analytic|interval|lipschitz|exact/i)
        || !String(leaf.sourceSignature || '')
        || leaf.fMax >= -residualTolerance
        || leaf.boundSource !== 'signed-geometry-interval-oracle-v1'
        || leaf.geometrySnapshotSignature !== certificate.intervalOracle.geometrySnapshotSignature
        || leaf.geometryLipschitzSignature !== certificate.intervalOracle.geometryLipschitzSignature
        || leaf.boundSignature !== contractSignature('least-root-interval-leaf-v1', leafBody)) {
        return { ok: false, reason: 'prefix leaf does not conservatively exclude a root' };
      }
      cursor = leaf.hi;
    }
    if (cursor !== lo.p) return { ok: false, reason: 'prefix exclusion cover has a gap or skips an earlier interval' };
    if (!Array.isArray(certificate.monotonicAudit)
      || certificate.monotonicAudit.length !== samples.length - 1
      || certificate.monotonicAudit.some((entry) => !entry.DNonDecreasing)) {
      return { ok: false, reason: 'complete ordered dissipation audit is missing' };
    }
    const tree = certificate.earlierIntervalTree;
    if (!tree || tree.traversal !== 'left-first'
      || canonical(tree.coverage) !== canonical([0, lo.p])
      || canonical(tree.exclusionLeaves) !== canonical(leaves)) {
      return { ok: false, reason: 'left-first interval tree binding mismatch' };
    }
    if (certificate.acceptedSample.sampleSignature !== hi.sampleSignature
      || canonical(sampleBindingBody(certificate.acceptedSample)) !== canonical(sampleBindingBody(hi))) {
      return { ok: false, reason: 'certificate accepted sample differs from bracket high sample' };
    }
    return { ok: true };
  } catch (error) {
    return { ok: false, reason: error.message };
  }
}

module.exports = {
  version: 3,
  selfSha256: SELF_SHA256,
  parentSolverSha256: PARENT_SOLVER_SHA256,
  DEFAULT_OPTIONS,
  prepareMaterialEvent,
  validatePreparedEvent,
  _reference: {
    canonical,
    signature,
    sampleBindingBody,
    intervalBindingBody,
    preparedBindingBody,
    bindingSignature,
  },
};
