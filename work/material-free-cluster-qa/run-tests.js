'use strict';

const crypto = require('node:crypto');
const fs = require('node:fs');
const path = require('node:path');
const {
  SOURCE_LOCKS,
  TEST_NAMES,
  sha256,
  runConformance,
} = require('./contract-core.js');
const reference = require('./pure-reference.js');

const MUTANT_EXPECTATIONS = Object.freeze({
  'double-world-step': TEST_NAMES.advance,
  'wact-mutable': TEST_NAMES.advance,
  'wact-recomputed': TEST_NAMES.accepted,
  'dual-finish': TEST_NAMES.branchStructural,
  'dry-mutation': TEST_NAMES.dry,
  'mixed-accepted': TEST_NAMES.accepted,
  'split-h': TEST_NAMES.h,
  'groups-leak': TEST_NAMES.advance,
  'persistent-restitution': TEST_NAMES.persistent,
  'truncate-domain': TEST_NAMES.domain,
});

function firstLine(reason) {
  return String(reason || '').split(/\r?\n/, 1)[0];
}

function resolveModuleArgument(argv) {
  const flag = argv.indexOf('--module');
  const raw = flag >= 0 ? argv[flag + 1] : argv.find((entry) => !entry.startsWith('-'));
  return raw ? path.resolve(process.cwd(), raw) : null;
}

function loadModule(file) {
  const loaded = require(file);
  return loaded && loaded.default && !loaded.createFreeClusterSession ? loaded.default : loaded;
}

const modulePath = resolveModuleArgument(process.argv.slice(2));
const referenceResult = runConformance(reference);
const negativeResults = Object.entries(MUTANT_EXPECTATIONS).map(([mutant, expectedTest]) => {
  const result = runConformance(reference.createReferenceModule({ mutant }), {
    only: [expectedTest],
    skipSourceLocks: true,
  });
  const failure = result.results.find((entry) => !entry.ok);
  return {
    mutant,
    expectedTest,
    caught: !result.ok && result.failed === 1,
    evidence: firstLine(failure?.reason),
  };
});

let integration = {
  status: 'awaiting-module',
  expectedPath: 'work/material-free-cluster-integration/free-cluster.js',
};
if (modulePath) {
  if (!fs.existsSync(modulePath)) {
    integration = { status: 'module-not-found', path: modulePath };
  } else {
    try {
      const moduleUnderTest = loadModule(modulePath);
      const result = runConformance(moduleUnderTest);
      integration = {
        status: result.ok ? 'passed' : 'failed',
        path: modulePath,
        sha256: sha256(modulePath),
        conformance: result,
      };
    } catch (error) {
      integration = {
        status: 'load-failed',
        path: modulePath,
        sha256: sha256(modulePath),
        reason: firstLine(error.stack || error),
      };
    }
  }
}

const report = {
  schema: 'material-free-cluster-contract-qa-v1',
  frozenSources: SOURCE_LOCKS.map((entry) => ({
    id: entry.id,
    path: path.relative(path.resolve(__dirname, '..', '..'), entry.file).replaceAll('\\', '/'),
    expectedSha256: entry.sha256,
    actualSha256: fs.existsSync(entry.file) ? sha256(entry.file) : null,
  })),
  reference: referenceResult,
  negativeMutationTests: negativeResults,
  integration,
  summary: {
    referencePassed: referenceResult.ok,
    referenceTests: `${referenceResult.passed}/${referenceResult.total}`,
    mutantsCaught: `${negativeResults.filter((entry) => entry.caught).length}/${negativeResults.length}`,
    integrationStatus: integration.status,
  },
};

const reportPath = path.join(__dirname, 'test-report.json');
fs.writeFileSync(reportPath, `${JSON.stringify(report, null, 2)}\n`, 'utf8');
const reportHash = crypto.createHash('sha256').update(fs.readFileSync(reportPath)).digest('hex').toUpperCase();

console.log(JSON.stringify({ ...report.summary, report: reportPath, reportSha256: reportHash }, null, 2));

const integrationOk = !modulePath || integration.status === 'passed';
if (!referenceResult.ok || negativeResults.some((entry) => !entry.caught) || !integrationOk) {
  process.exitCode = 1;
}
