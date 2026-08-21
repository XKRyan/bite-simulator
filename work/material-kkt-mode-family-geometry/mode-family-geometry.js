'use strict';

// Work-only mathematical composition.  It pins and uses the signed Coulomb
// mode construction, but it authenticates neither Sfree nor geometry ownership.

const crypto = require('node:crypto');
const fs = require('node:fs');
const path = require('node:path');
const coulomb = require('../kkt-coulomb-extension/event-kkt-coulomb.js');
const finiteRotation = require('../material-finite-rotation-sweep-oracle/finite-rotation-sweep-oracle.js');

const COULOMB_PATH = path.resolve(__dirname, '../kkt-coulomb-extension/event-kkt-coulomb.js');
const FINITE_ROTATION_PATH = path.resolve(__dirname,
  '../material-finite-rotation-sweep-oracle/finite-rotation-sweep-oracle.js');
const EXPECTED_COULOMB_SHA256 = '8B3058E5833E96D7C83971B6D0C234E6641D0CB98044012DFFFA5DA2268E1A3E';
const EXPECTED_FINITE_ROTATION_SHA256 = '9F37F457CD1F5DE6FB76680E136E3FC80900848C79A5E8B3A041D831BCF3D3CF';

function sha256(value) { return crypto.createHash('sha256').update(value).digest('hex').toUpperCase(); }
function fileSha256(filename) { return sha256(fs.readFileSync(filename)); }
function selfSha256() { return fileSha256(__filename); }
function finite(value, label) {
  if (!Number.isFinite(value)) throw new TypeError(`${label} must be finite`);
  return Number(value);
}
function clone(value) { return value === undefined ? undefined : JSON.parse(JSON.stringify(value)); }
function deepFreeze(value) {
  if (value && (typeof value === 'object' || typeof value === 'function') && !Object.isFrozen(value)) {
    Object.freeze(value); Object.values(value).forEach(deepFreeze);
  }
  return value;
}
function canonicalValue(value) {
  if (value === null || typeof value === 'string' || typeof value === 'boolean') return value;
  if (typeof value === 'number') {
    if (!Number.isFinite(value)) throw new TypeError('canonical values must be finite');
    return Object.is(value, -0) ? 0 : value;
  }
  if (Array.isArray(value)) return value.map(canonicalValue);
  if (value && typeof value === 'object') {
    const result = {}; Object.keys(value).sort().forEach((key) => { result[key] = canonicalValue(value[key]); });
    return result;
  }
  throw new TypeError(`unsupported canonical value ${typeof value}`);
}
function digest(label, value) { return sha256(`${label}\n${JSON.stringify(canonicalValue(value))}`); }
function canonical(value) { return JSON.stringify(canonicalValue(value)); }
function domainStop(reason, extra = {}) {
  return deepFreeze({ ok: false, status: 'solver-domain-stop', reason, ...extra });
}
function exactArray(left, right) {
  return Array.isArray(left) && Array.isArray(right) && left.length === right.length
    && left.every((entry, index) => entry === right[index]);
}
function exactStrings(left, right) {
  return Array.isArray(left) && Array.isArray(right) && left.length === right.length
    && left.every((entry, index) => String(entry) === String(right[index]));
}

function createModeFamilyGeometry(config) {
  try {
    if (!config || typeof config !== 'object') throw new TypeError('config is required');
    if (fileSha256(COULOMB_PATH) !== EXPECTED_COULOMB_SHA256) throw new Error('Coulomb solver hash mismatch');
    if (fileSha256(FINITE_ROTATION_PATH) !== EXPECTED_FINITE_ROTATION_SHA256
      || finiteRotation.moduleSha256 !== EXPECTED_FINITE_ROTATION_SHA256) {
      throw new Error('finite-rotation kernel hash mismatch');
    }
    const pDomain = Array.isArray(config.pDomain) && config.pDomain.length === 2
      ? config.pDomain.map((entry, index) => finite(entry, `pDomain[${index}]`)) : null;
    if (!pDomain || pDomain[0] !== 0 || !(pDomain[1] > 0)) {
      throw new TypeError('pDomain must be [0, positiveMaximum]');
    }
    const kktInput = clone(config.kktInput);
    if (!kktInput || !Array.isArray(kktInput.qFree) || kktInput.qFree.length !== 6) {
      throw new TypeError('the frozen production six-DOF qFree order is required');
    }
    kktInput.qFree.forEach((entry, index) => finite(entry, `qFree[${index}]`));
    // buildProblem does not call freshArea, but the signed input contract
    // requires the callback to exist.
    kktInput.freshArea = () => ({ area: 0, payload: null });
    const problem = coulomb._reference.buildProblem(kktInput);
    const workEnvelope = problem.workSegments(pDomain[1]);
    if (!workEnvelope || !Array.isArray(workEnvelope.segments) || !workEnvelope.segments.length) {
      throw new Error('Coulomb maximum-dissipation envelope is empty');
    }
    const envelopeSegments = [];
    const modeRanges = new Map();
    let cursor = 0;
    for (const [index, raw] of workEnvelope.segments.entries()) {
      const start = finite(raw.start, `envelope[${index}].start`);
      const end = finite(raw.end, `envelope[${index}].end`);
      if (start !== cursor || !(end > start) || end > pDomain[1]) {
        throw new Error('Coulomb envelope does not form an exact ordered partition');
      }
      const midpoint = start + (end - start) / 2;
      const selected = problem.solveAt(midpoint);
      if (!selected?._mode || selected.modeKey !== raw.modeKey) {
        throw new Error('Coulomb envelope mode identity is not recoverable');
      }
      for (const fraction of [0.25, 0.5, 0.75]) {
        const probe = problem.solveAt(start + (end - start) * fraction);
        if (probe._mode !== selected._mode || probe.modeKey !== raw.modeKey) {
          throw new Error('Coulomb envelope switches inside a declared mode cell');
        }
      }
      const entry = { index, start, end, modeKey: raw.modeKey,
        modeOrdinal: selected._mode.ordinal, mode: selected._mode };
      envelopeSegments.push(entry); cursor = end;
      const range = modeRanges.get(selected._mode) || { low: start, high: end };
      range.low = Math.min(range.low, start); range.high = Math.max(range.high, end);
      modeRanges.set(selected._mode, range);
    }
    if (cursor !== pDomain[1]) throw new Error('Coulomb envelope does not cover the full pDomain');

    const geometry = config.geometry;
    if (!geometry || typeof geometry !== 'object') throw new TypeError('geometry is required');
    const featureStartPosition = clone(geometry.featureStartPosition);
    const featureStartAngle = finite(geometry.featureStartAngle, 'geometry.featureStartAngle');
    const materialStartPosition = clone(geometry.materialStartPosition);
    const materialStartAngle = finite(geometry.materialStartAngle, 'geometry.materialStartAngle');
    const kernelCache = new Map();

    function kernelForMode(mode) {
      if (kernelCache.has(mode)) return kernelCache.get(mode);
      const range = modeRanges.get(mode);
      if (!range || !(range.high > range.low)) throw new Error('selected mode has no certified envelope range');
      if (!Array.isArray(mode.qIntercept) || mode.qIntercept.length !== 6
        || !Array.isArray(mode.qSlope) || mode.qSlope.length !== 6) {
        throw new Error('selected mode has no six-DOF affine qPost family');
      }
      const kernel = finiteRotation.createFiniteRotationSweepKernel({
        pDomain: [range.low, range.high],
        h: geometry.h,
        timeFractions: geometry.timeFractions,
        maximumTimeChordError: geometry.maximumTimeChordError,
        lengthTolerance: geometry.lengthTolerance,
        workingSegment: geometry.workingSegment,
        remainingGeometry: geometry.remainingGeometry,
        featureMotion: {
          startPosition: featureStartPosition,
          startAngle: featureStartAngle,
          linearVelocityAffine: {
            intercept: [mode.qIntercept[0], 0], slope: [mode.qSlope[0], 0],
          },
          angularVelocityAffine: { intercept: mode.qIntercept[1], slope: mode.qSlope[1] },
        },
        materialMotion: {
          startPosition: materialStartPosition,
          startAngle: materialStartAngle,
          linearVelocityAffine: {
            intercept: [mode.qIntercept[3], mode.qIntercept[4]],
            slope: [mode.qSlope[3], mode.qSlope[4]],
          },
          angularVelocityAffine: { intercept: mode.qIntercept[5], slope: mode.qSlope[5] },
        },
      });
      if (!kernel.ok) throw new Error(`finite-rotation mode ${mode.modeKey} stopped: ${kernel.reason}`);
      kernelCache.set(mode, kernel); return kernel;
    }

    // Construct all selected kernels now so a later exact/interval call cannot
    // discover an unbounded mode after partial root-search work.
    for (const mode of modeRanges.keys()) kernelForMode(mode);
    const selectedKernelDescriptors = envelopeSegments.map((entry) => {
      const descriptor = kernelForMode(entry.mode).descriptor;
      return { index: entry.index, start: entry.start, end: entry.end,
        modeKey: entry.modeKey, modeOrdinal: entry.modeOrdinal,
        geometrySnapshotSignature: descriptor.geometrySnapshotSignature,
        pathSignature: descriptor.pathSignature,
        featureAngularVelocityAffine: clone(descriptor.motionFamily?.feature?.angularVelocity),
        materialAngularVelocityAffine: clone(descriptor.motionFamily?.material?.angularVelocity),
        timeChordError: descriptor.timeChordError };
    });
    const geometrySignatures = [...new Set(selectedKernelDescriptors
      .map((entry) => entry.geometrySnapshotSignature))];
    if (geometrySignatures.length !== 1) throw new Error('mode kernels do not share one geometry snapshot');
    const geometrySnapshotSignature = geometrySignatures[0];
    const modeFamilyBody = {
      pDomain,
      qFree: kktInput.qFree,
      Minv: kktInput.Minv,
      materialContact: kktInput.materialContact,
      structuralContacts: kktInput.structuralContacts || [],
      selectedKernelDescriptors,
      geometrySnapshotSignature,
      parentCoulombSha256: EXPECTED_COULOMB_SHA256,
      finiteRotationSha256: EXPECTED_FINITE_ROTATION_SHA256,
    };
    const modeFamilySignature = digest('material-kkt-mode-family-geometry-v1', modeFamilyBody);

    function solveExact(pValue) {
      const p = finite(pValue, 'p');
      if (p < pDomain[0] || p > pDomain[1]) throw new Error('p is outside the mode-family domain');
      const solved = problem.solveAt(p);
      if (!solved?._mode) throw new Error('exact Coulomb solve did not retain its private mode identity');
      return solved;
    }
    function validateExternalTrial(solved, trial) {
      if (!trial) return;
      if (trial.p !== solved.p || trial.lambda !== solved.lambda
        || trial.modeKey !== solved.modeKey || !exactArray(trial.qPost, solved.qPost)
        || !exactStrings(trial.activeIds, solved.activeIds)
        || canonical(trial.contactStates) !== canonical(solved.contactStates)
        || canonical(trial.contactImpulses) !== canonical(solved.contactImpulses)
        || canonical(trial.contactPointBindings) !== canonical(problem.pointBindings)) {
        throw new Error('external endpoint trial does not match the private Coulomb solve exactly');
      }
    }
    function discretePathSample(pValue, externalTrial) {
      try {
        const solved = solveExact(pValue); validateExternalTrial(solved, externalTrial);
        const kernel = kernelForMode(solved._mode);
        if (typeof kernel.discretePathSample !== 'function') throw new Error('finite kernel has no discrete path port');
        const inner = kernel.discretePathSample(solved.p);
        if (!inner.ok) throw new Error(inner.reason);
        const expectedQ = solved._mode.qIntercept.map((entry, index) => entry
          + solved._mode.qSlope[index] * solved.p);
        if (!exactArray(expectedQ, solved.qPost)) throw new Error('selected affine mode does not reproduce qPost exactly');
        const result = {
          ok: true,
          status: 'mode-bound-discrete-path-sample',
          p: solved.p,
          qPost: solved.qPost.slice(),
          modeKey: solved.modeKey,
          modeOrdinal: solved._mode.ordinal,
          activeIds: solved.activeIds.slice(),
          inner: clone(inner),
          geometrySnapshotSignature,
          modeFamilySignature,
          authority: 'none-pure-mathematical-composition',
        };
        result.pathSampleDigest = digest('material-kkt-mode-family-path-sample-v1', result);
        return deepFreeze(result);
      } catch (error) { return domainStop(String(error?.message || error)); }
    }
    function boundDiscretePathInterval(loValue, hiValue) {
      try {
        const lo = finite(loValue, 'lo'); const hi = finite(hiValue, 'hi');
        if (!(hi > lo) || lo < pDomain[0] || hi > pDomain[1]) throw new Error('invalid p interval');
        const pieces = [];
        for (const entry of envelopeSegments) {
          const subLo = Math.max(lo, entry.start); const subHi = Math.min(hi, entry.end);
          if (!(subHi > subLo)) continue;
          const kernel = kernelForMode(entry.mode);
          if (typeof kernel.boundDiscretePathInterval !== 'function') throw new Error('finite kernel has no path interval port');
          const inner = kernel.boundDiscretePathInterval(subLo, subHi);
          if (!inner.ok) throw new Error(`mode path interval ${entry.index} stopped: ${inner.reason}`);
          pieces.push({ envelopeIndex: entry.index, modeKey: entry.modeKey,
            modeOrdinal: entry.modeOrdinal, subLo, subHi, inner: clone(inner) });
        }
        if (!pieces.length || pieces[0].subLo !== lo || pieces.at(-1).subHi !== hi
          || pieces.some((entry, index) => index && pieces[index - 1].subHi !== entry.subLo)) {
          throw new Error('mode path interval pieces do not form a complete ordered cover');
        }
        const result = {
          ok: true,
          status: 'mode-family-discrete-path-interval-bound',
          pLo: lo,
          pHi: hi,
          pieces,
          geometrySnapshotSignature,
          modeFamilySignature,
          authority: 'none-pure-mathematical-composition',
        };
        result.pathIntervalDigest = digest('material-kkt-mode-family-path-interval-v1', result);
        return deepFreeze(result);
      } catch (error) { return domainStop(String(error?.message || error)); }
    }
    function exactSample(pValue, externalTrial) {
      try {
        const solved = solveExact(pValue); validateExternalTrial(solved, externalTrial);
        const kernel = kernelForMode(solved._mode); const inner = kernel.exactSample(solved.p);
        if (!inner.ok) throw new Error(inner.reason);
        const expectedQ = solved._mode.qIntercept.map((entry, index) => entry
          + solved._mode.qSlope[index] * solved.p);
        if (!exactArray(expectedQ, solved.qPost)) throw new Error('selected affine mode does not reproduce qPost exactly');
        const payload = {
          schema: 'material-kkt-mode-family-geometry-payload-v1',
          p: solved.p, qPost: solved.qPost.slice(), modeKey: solved.modeKey,
          modeOrdinal: solved._mode.ordinal, activeIds: solved.activeIds.slice(),
          inner: clone(inner.payload), innerSampleDigest: inner.sampleDigest,
          geometrySnapshotSignature, modeFamilySignature,
        };
        payload.payloadDigest = digest('material-kkt-mode-family-geometry-payload-v1', payload);
        const result = { ok: true, status: 'exact-mode-family-sample', p: solved.p,
          area: inner.area, freshArea: inner.area, areaLower: inner.areaLower,
          areaUpper: inner.areaUpper, numericalAreaGuard: inner.numericalAreaGuard,
          payload, modeKey: solved.modeKey, qPost: solved.qPost.slice(),
          authority: 'none-pure-mathematical-composition' };
        result.sampleDigest = digest('material-kkt-mode-family-sample-v1', result);
        return deepFreeze(result);
      } catch (error) { return domainStop(String(error?.message || error)); }
    }

    function boundFreshAreaInterval(loValue, hiValue) {
      try {
        const lo = finite(loValue, 'lo'); const hi = finite(hiValue, 'hi');
        if (!(hi > lo) || lo < pDomain[0] || hi > pDomain[1]) throw new Error('invalid p interval');
        const pieces = [];
        for (const entry of envelopeSegments) {
          const subLo = Math.max(lo, entry.start); const subHi = Math.min(hi, entry.end);
          if (!(subHi > subLo)) continue;
          const inner = kernelForMode(entry.mode).boundFreshAreaInterval(subLo, subHi);
          if (!inner.ok) throw new Error(`mode interval ${entry.index} stopped: ${inner.reason}`);
          pieces.push({ envelopeIndex: entry.index, modeKey: entry.modeKey,
            modeOrdinal: entry.modeOrdinal, subLo, subHi, inner: clone(inner) });
        }
        if (!pieces.length || pieces[0].subLo !== lo || pieces.at(-1).subHi !== hi) {
          throw new Error('mode interval pieces do not cover the requested interval');
        }
        for (let index = 1; index < pieces.length; index += 1) {
          if (pieces[index - 1].subHi !== pieces[index].subLo) {
            throw new Error('mode interval pieces contain a gap or overlap');
          }
        }
        const transitionPs = [...new Set([lo, hi, ...pieces.slice(0, -1).map((entry) => entry.subHi)])]
          .sort((left, right) => left - right);
        const transitionSamples = transitionPs.map((p) => exactSample(p));
        if (transitionSamples.some((entry) => !entry.ok)) throw new Error('a transition sample is unresolved');
        let areaLower = Math.min(...pieces.map((entry) => entry.inner.areaLower),
          ...transitionSamples.map((entry) => entry.areaLower));
        let areaUpper = Math.max(...pieces.map((entry) => entry.inner.areaUpper),
          ...transitionSamples.map((entry) => entry.areaUpper));
        areaLower = Math.max(0, areaLower);
        const proof = {
          schema: 'material-kkt-mode-family-interval-proof-v1',
          pieces: pieces.map((entry) => ({ envelopeIndex: entry.envelopeIndex,
            modeKey: entry.modeKey, modeOrdinal: entry.modeOrdinal,
            subLo: entry.subLo, subHi: entry.subHi,
            innerIntervalDigest: entry.inner.intervalDigest,
            innerPathSignature: entry.inner.pathSignature,
            innerProof: entry.inner.proof })),
          transitions: transitionSamples.map((entry) => ({ p: entry.p, modeKey: entry.modeKey,
            sampleDigest: entry.sampleDigest, areaLower: entry.areaLower, areaUpper: entry.areaUpper })),
          completeOrderedEnvelope: true,
          parentCoulombSha256: EXPECTED_COULOMB_SHA256,
          finiteRotationSha256: EXPECTED_FINITE_ROTATION_SHA256,
        };
        const result = {
          ok: true, status: 'pure-certified-mode-family-interval',
          certificateType: 'material-kkt-mode-family-exact-envelope-interval-v1',
          pLo: lo, pHi: hi, areaLower, areaUpper, minArea: areaLower, maxArea: areaUpper,
          moduleSha256: selfSha256(), sourceSignature: modeFamilySignature,
          geometrySnapshotSignature, geometryLipschitzSignature: modeFamilySignature,
          modeFamilySignature, proof, authority: 'none-pure-mathematical-composition',
        };
        result.intervalDigest = digest('material-kkt-mode-family-area-interval-v1', result);
        return deepFreeze(result);
      } catch (error) { return domainStop(String(error?.message || error)); }
    }

    const descriptor = deepFreeze({
      schema: 'material-kkt-mode-family-geometry-v1', moduleSha256: selfSha256(),
      parentCoulombSha256: EXPECTED_COULOMB_SHA256,
      finiteRotationSha256: EXPECTED_FINITE_ROTATION_SHA256,
      pDomain, geometrySnapshotSignature, modeFamilySignature,
      intervalOracle: { schema: 'signed-geometry-fresh-area-interval-v1',
        moduleSha256: selfSha256(), geometrySnapshotSignature,
        geometryLipschitzSignature: modeFamilySignature },
      envelope: selectedKernelDescriptors,
      qOrder: ['rail-pivot-vx', 'weapon-omega', 'fork-omega',
        'target-com-vx', 'target-com-vy', 'target-omega'],
      authority: 'none; real Sfree, path, owner and geometry authority are not supplied',
    });
    return deepFreeze({ ok: true, status: 'pure-mode-family-ready', descriptor,
      discretePathSample, boundDiscretePathInterval,
      exactSample, freshArea: exactSample, boundFreshAreaInterval });
  } catch (error) {
    return domainStop(String(error?.message || error), { moduleSha256: selfSha256() });
  }
}

module.exports = {
  createModeFamilyGeometry,
  EXPECTED_COULOMB_SHA256,
  EXPECTED_FINITE_ROTATION_SHA256,
  moduleSha256: selfSha256(),
};
