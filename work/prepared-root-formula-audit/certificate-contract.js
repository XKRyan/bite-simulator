'use strict';

const crypto = require('crypto');
function fail(reason) { return { ok: false, reason }; }
const close = (a, b, tolerance) => Math.abs(a - b) <= tolerance;
function canonical(value) {
  if (value === null || typeof value !== 'object') return JSON.stringify(value);
  if (Array.isArray(value)) return `[${value.map(canonical).join(',')}]`;
  return `{${Object.keys(value).sort().map((key) => `${JSON.stringify(key)}:${canonical(value[key])}`).join(',')}}`;
}
const sign = (prefix, value) => `${prefix}-${crypto.createHash('sha256').update(canonical(value)).digest('hex')}`;

function validateSample(sample, Uc, width, residualTolerance, label) {
  if (!sample || !['p', 'D', 'freshArea', 'materialWork', 'F'].every((field) => Number.isFinite(sample[field]))) {
    return fail(`${label} lacks p/D/A/W/F`);
  }
  if (!close(sample.materialWork, Uc * width * sample.freshArea, residualTolerance)) return fail(`${label} material-work identity failed`);
  if (!close(sample.F, sample.D - sample.materialWork, residualTolerance)) return fail(`${label} residual identity failed`);
  if (!String(sample.modeKey || '') || !String(sample.workSegmentsSignature || '') || !String(sample.sampleSignature || '')) {
    return fail(`${label} binding signatures incomplete`);
  }
  if (!Array.isArray(sample.workSegments)) return fail(`${label} work segments required`);
  let cursor = 0; let work = 0;
  for (const segment of sample.workSegments) {
    if (segment.start !== cursor || !(segment.end >= segment.start) || segment.end > sample.p
      || !Number.isFinite(segment.work) || segment.work < 0 || !String(segment.modeKey || '')) {
      return fail(`${label} work segments do not cover [0,p] contiguously`);
    }
    cursor = segment.end; work += segment.work;
  }
  if (cursor !== sample.p || !close(work, sample.D, residualTolerance)) return fail(`${label} segmented D mismatch`);
  if (sample.p > 0 && sample.workSegments.at(-1)?.modeKey !== sample.modeKey) return fail(`${label} endpoint mode mismatch`);
  return { ok: true };
}

function leafBody(leaf) {
  const { boundSignature, ...body } = leaf;
  return body;
}

function validateLeastRootCertificate(prepared, trust = {}) {
  const certificate = prepared?.leastRootCertificate;
  if (!certificate || typeof certificate !== 'object') return fail('leastRootCertificate required');
  const Uc = Number(certificate.specificCuttingEnergy); const width = Number(certificate.width);
  const residualTolerance = Number(certificate.residualTolerance?.absolute ?? certificate.residualTolerance);
  const impulseTolerance = Number(certificate.impulseTolerance?.absolute ?? certificate.impulseTolerance);
  if (!(Uc > 0 && width > 0 && Number.isFinite(residualTolerance) && residualTolerance >= 0
    && Number.isFinite(impulseTolerance) && impulseTolerance >= 0)) return fail('positive Uc/width and finite tolerances required');
  const oracle = certificate.intervalOracle;
  const expectedOracleSha256 = String(trust.trustedGeometryOracleSha256 || '');
  if (!expectedOracleSha256) return fail('trusted geometry oracle SHA-256 must be pinned by config');
  if (!oracle || oracle.schema !== 'signed-geometry-fresh-area-interval-v1'
    || oracle.moduleSha256 !== expectedOracleSha256 || !String(oracle.geometrySnapshotSignature || '')
    || !String(oracle.geometryLipschitzSignature || '')) return fail('signed geometry interval oracle provenance required');

  const bracket = certificate.acceptedBracket;
  if (!bracket || !bracket.lo || !bracket.hi) return fail('accepted bracket endpoint samples required');
  for (const [name, endpoint] of [['lo', bracket.lo], ['hi', bracket.hi]]) {
    const result = validateSample(endpoint, Uc, width, residualTolerance, `bracket ${name}`);
    if (!result.ok) return result;
  }
  if (!(bracket.lo.p < bracket.hi.p) || !(bracket.lo.F <= residualTolerance && bracket.hi.F >= -residualTolerance)) {
    return fail('accepted bracket has no oriented sign crossing');
  }
  if (!(prepared.p >= bracket.lo.p && prepared.p <= bracket.hi.p)) return fail('accepted p outside bracket');
  if (bracket.hi.p - bracket.lo.p > impulseTolerance && Math.abs(prepared.workResidual) > residualTolerance) {
    return fail('accepted bracket/root residual did not converge');
  }

  const tree = certificate.earlierIntervalTree;
  if (!tree || tree.traversal !== 'left-first' || !Array.isArray(tree.exclusionLeaves)) {
    return fail('left-first earlier interval tree required');
  }
  if (!Array.isArray(tree.coverage) || tree.coverage[0] !== 0 || tree.coverage[1] !== bracket.lo.p) {
    return fail('earlier tree coverage must be [0, bracket.lo.p]');
  }
  let cursor = 0;
  for (const [index, leaf] of tree.exclusionLeaves.entries()) {
    if (leaf.lo !== cursor || !(leaf.hi > leaf.lo) || leaf.hi > bracket.lo.p) return fail('earlier interval leaves have a gap/overlap');
    if (!(Number.isFinite(leaf.fMin) && Number.isFinite(leaf.fMax) && leaf.fMin <= leaf.fMax)) return fail(`leaf ${index} F range invalid`);
    if (!(leaf.fMax < -residualTolerance || leaf.fMin > residualTolerance)) return fail(`leaf ${index} does not conservatively exclude zero`);
    if (leaf.boundSource !== 'signed-geometry-interval-oracle-v1'
      || leaf.geometrySnapshotSignature !== oracle.geometrySnapshotSignature
      || leaf.geometryLipschitzSignature !== oracle.geometryLipschitzSignature
      || !String(leaf.areaBoundSignature || '') || !String(leaf.workBoundSignature || '')) {
      return fail(`leaf ${index} bound provenance mismatch`);
    }
    if (leaf.boundSignature !== sign('least-root-interval-leaf-v1', leafBody(leaf))) return fail(`leaf ${index} bound signature mismatch`);
    cursor = leaf.hi;
  }
  if (cursor !== bracket.lo.p) return fail('earlier interval tree does not cover through bracket low endpoint');

  const accepted = certificate.acceptedSample;
  const acceptedValidity = validateSample(accepted, Uc, width, residualTolerance, 'accepted sample');
  if (!acceptedValidity.ok) return acceptedValidity;
  if (accepted.sampleSignature !== prepared.acceptedSampleSignature || accepted.p !== prepared.p
    || accepted.modeKey !== prepared.modeKey || !close(accepted.D, prepared.dissipatedWork, residualTolerance)
    || !close(accepted.freshArea, prepared.freshArea, residualTolerance)
    || !close(accepted.F, prepared.workResidual, residualTolerance)) return fail('accepted fields are not one bound sample');
  return { ok: true };
}

module.exports = { validateLeastRootCertificate, sign, leafBody };
