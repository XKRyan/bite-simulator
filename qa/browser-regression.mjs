#!/usr/bin/env node

import assert from 'node:assert/strict';
import { spawn } from 'node:child_process';
import {
  existsSync,
  mkdtempSync,
  readFileSync,
  rmSync,
} from 'node:fs';
import { tmpdir } from 'node:os';
import path from 'node:path';
import { fileURLToPath, pathToFileURL } from 'node:url';
import { DXF_FIXTURES } from './dxf-fixtures.mjs';

const QA_TIMEOUT_MS = 30_000;
const LONG_SIMULATION_TIMEOUT_MS = 120_000;
const REQUIRED_NODE_MAJOR = 24;
const scriptDir = path.dirname(fileURLToPath(import.meta.url));
const repoRoot = path.resolve(scriptDir, '..');
const indexPath = path.join(repoRoot, 'index.html');

function parseArguments(argv) {
  const options = { browser: process.env.BITE_QA_BROWSER || null };
  for (let index = 0; index < argv.length; index += 1) {
    const arg = argv[index];
    if (arg === '--browser') options.browser = argv[++index];
    else if (arg.startsWith('--browser=')) options.browser = arg.slice('--browser='.length);
    else if (arg === '--help' || arg === '-h') options.help = true;
    else throw new Error(`Unknown argument: ${arg}`);
  }
  return options;
}

function usage() {
  return [
    'Usage: node qa/browser-regression.mjs [--browser <path>]',
    '',
    'Environment:',
    '  BITE_QA_BROWSER   Explicit Edge/Chrome executable path.',
  ].join('\n');
}

function browserCandidates(explicit) {
  const candidates = [];
  if (explicit) candidates.push(explicit);
  if (process.platform === 'win32') {
    const programFiles = process.env.ProgramFiles || 'C:\\Program Files';
    const programFilesX86 = process.env['ProgramFiles(x86)'] || 'C:\\Program Files (x86)';
    const localAppData = process.env.LOCALAPPDATA || '';
    candidates.push(
      path.join(programFilesX86, 'Microsoft', 'Edge', 'Application', 'msedge.exe'),
      path.join(programFiles, 'Microsoft', 'Edge', 'Application', 'msedge.exe'),
      path.join(programFiles, 'Google', 'Chrome', 'Application', 'chrome.exe'),
      path.join(localAppData, 'Google', 'Chrome', 'Application', 'chrome.exe'),
    );
  } else if (process.platform === 'darwin') {
    candidates.push(
      '/Applications/Microsoft Edge.app/Contents/MacOS/Microsoft Edge',
      '/Applications/Google Chrome.app/Contents/MacOS/Google Chrome',
      '/Applications/Google Chrome Canary.app/Contents/MacOS/Google Chrome Canary',
    );
  } else {
    candidates.push(
      '/usr/bin/microsoft-edge',
      '/usr/bin/microsoft-edge-stable',
      '/usr/bin/google-chrome',
      '/usr/bin/google-chrome-stable',
      '/usr/bin/chromium',
      '/usr/bin/chromium-browser',
    );
  }
  return [...new Set(candidates.filter(Boolean))];
}

function findBrowser(explicit) {
  const candidates = browserCandidates(explicit);
  const browser = candidates.find((candidate) => existsSync(candidate));
  if (!browser) {
    throw new Error(`Edge/Chrome was not found. Checked:\n${candidates.map((entry) => `  ${entry}`).join('\n')}\nUse --browser or BITE_QA_BROWSER.`);
  }
  return browser;
}

const delay = (milliseconds) => new Promise((resolve) => setTimeout(resolve, milliseconds));

async function waitFor(predicate, description, timeoutMs = QA_TIMEOUT_MS) {
  const deadline = Date.now() + timeoutMs;
  let lastError = null;
  while (Date.now() < deadline) {
    try {
      const value = await predicate();
      if (value) return value;
    } catch (error) {
      lastError = error;
    }
    await delay(50);
  }
  const suffix = lastError ? ` Last error: ${lastError.message}` : '';
  throw new Error(`Timed out waiting for ${description}.${suffix}`);
}

function websocketText(data) {
  if (typeof data === 'string') return data;
  if (data instanceof ArrayBuffer) return Buffer.from(data).toString('utf8');
  if (ArrayBuffer.isView(data)) return Buffer.from(data.buffer, data.byteOffset, data.byteLength).toString('utf8');
  return String(data);
}

class CdpConnection {
  constructor(url, label = 'CDP') {
    this.url = url;
    this.label = label;
    this.nextId = 1;
    this.pending = new Map();
    this.listeners = new Map();
    this.socket = null;
  }

  async open() {
    this.socket = new WebSocket(this.url);
    this.socket.addEventListener('message', (event) => this.#onMessage(websocketText(event.data)));
    this.socket.addEventListener('close', () => this.#rejectPending(new Error(`${this.label} connection closed`)));
    await new Promise((resolve, reject) => {
      const timer = setTimeout(() => reject(new Error(`Timed out opening ${this.label}`)), QA_TIMEOUT_MS);
      this.socket.addEventListener('open', () => {
        clearTimeout(timer);
        resolve();
      }, { once: true });
      this.socket.addEventListener('error', () => {
        clearTimeout(timer);
        reject(new Error(`Failed to open ${this.label}: ${this.url}`));
      }, { once: true });
    });
    return this;
  }

  #onMessage(text) {
    const message = JSON.parse(text);
    if (message.id) {
      const request = this.pending.get(message.id);
      if (!request) return;
      this.pending.delete(message.id);
      clearTimeout(request.timer);
      if (message.error) request.reject(new Error(`${request.method}: ${message.error.message}`));
      else request.resolve(message.result);
      return;
    }
    const callbacks = this.listeners.get(message.method) || [];
    callbacks.forEach((callback) => callback(message.params || {}));
  }

  #rejectPending(error) {
    for (const request of this.pending.values()) {
      clearTimeout(request.timer);
      request.reject(error);
    }
    this.pending.clear();
  }

  on(method, callback) {
    const callbacks = this.listeners.get(method) || [];
    callbacks.push(callback);
    this.listeners.set(method, callbacks);
  }

  send(method, params = {}, timeoutMs = QA_TIMEOUT_MS) {
    if (!this.socket || this.socket.readyState !== WebSocket.OPEN) {
      return Promise.reject(new Error(`${this.label} is not open`));
    }
    const id = this.nextId++;
    return new Promise((resolve, reject) => {
      const timer = setTimeout(() => {
        this.pending.delete(id);
        reject(new Error(`${method} timed out after ${timeoutMs} ms`));
      }, timeoutMs);
      this.pending.set(id, { resolve, reject, timer, method });
      this.socket.send(JSON.stringify({ id, method, params }));
    });
  }

  close() {
    if (this.socket && this.socket.readyState < WebSocket.CLOSING) this.socket.close();
  }
}

class BrowserPage {
  constructor(connection, targetId, browserConnection) {
    this.connection = connection;
    this.targetId = targetId;
    this.browserConnection = browserConnection;
    this.exceptions = [];
    this.consoleErrors = [];
    connection.on('Runtime.exceptionThrown', ({ exceptionDetails }) => {
      this.exceptions.push(exceptionDetails?.exception?.description || exceptionDetails?.text || 'Unknown page exception');
    });
    connection.on('Runtime.consoleAPICalled', ({ type, args = [] }) => {
      if (type === 'error' || type === 'assert') {
        this.consoleErrors.push(args.map((entry) => entry.value ?? entry.description ?? '').join(' '));
      }
    });
  }

  async initialise() {
    await Promise.all([
      this.connection.send('Page.enable'),
      this.connection.send('Runtime.enable'),
      this.connection.send('Log.enable'),
    ]);
    await waitFor(
      () => this.evaluate(() => Boolean(window.BiteSim?.getState && window.BiteSim?.advance)),
      'window.BiteSim',
    );
    return this;
  }

  async evaluate(fn, argument, timeoutMs = QA_TIMEOUT_MS) {
    const expression = argument === undefined
      ? `(${fn.toString()})()`
      : `(${fn.toString()})(${JSON.stringify(argument)})`;
    const response = await this.connection.send('Runtime.evaluate', {
      expression,
      awaitPromise: true,
      returnByValue: true,
      userGesture: true,
    }, timeoutMs);
    if (response.exceptionDetails) {
      const description = response.exceptionDetails.exception?.description || response.exceptionDetails.text;
      throw new Error(description || 'Runtime.evaluate failed');
    }
    if (response.result?.subtype === 'error') {
      throw new Error(response.result.description || 'Runtime.evaluate returned an error');
    }
    return response.result?.value;
  }

  async close() {
    try {
      await this.browserConnection.send('Target.closeTarget', { targetId: this.targetId });
    } finally {
      this.connection.close();
    }
  }
}

class BrowserHarness {
  constructor(executable) {
    this.executable = executable;
    this.profileDir = null;
    this.process = null;
    this.port = null;
    this.connection = null;
    this.stderr = '';
  }

  async start() {
    this.profileDir = mkdtempSync(path.join(tmpdir(), 'bite-simulator-qa-'));
    const args = [
      '--headless=new',
      '--remote-debugging-port=0',
      `--user-data-dir=${this.profileDir}`,
      '--allow-file-access-from-files',
      '--disable-background-networking',
      '--disable-component-update',
      '--disable-default-apps',
      '--disable-extensions',
      '--disable-sync',
      '--metrics-recording-only',
      '--no-first-run',
      '--no-default-browser-check',
      'about:blank',
    ];
    this.process = spawn(this.executable, args, { stdio: ['ignore', 'ignore', 'pipe'], windowsHide: true });
    this.process.stderr.setEncoding('utf8');
    this.process.stderr.on('data', (chunk) => {
      this.stderr = `${this.stderr}${chunk}`.slice(-8_000);
    });
    const activePortPath = path.join(this.profileDir, 'DevToolsActivePort');
    const activePort = await waitFor(() => {
      if (this.process.exitCode !== null) {
        throw new Error(`Browser exited with code ${this.process.exitCode}. ${this.stderr}`);
      }
      if (!existsSync(activePortPath)) return null;
      const [port, websocketPath] = readFileSync(activePortPath, 'utf8').trim().split(/\r?\n/);
      return port && websocketPath ? { port: Number(port), websocketPath } : null;
    }, 'the DevTools endpoint');
    this.port = activePort.port;
    this.connection = await new CdpConnection(
      `ws://127.0.0.1:${this.port}${activePort.websocketPath}`,
      'browser CDP',
    ).open();
    return this;
  }

  async page(url) {
    const { targetId } = await this.connection.send('Target.createTarget', { url });
    const target = await waitFor(async () => {
      const response = await fetch(`http://127.0.0.1:${this.port}/json/list`);
      if (!response.ok) throw new Error(`DevTools target list returned ${response.status}`);
      const targets = await response.json();
      return targets.find((entry) => entry.id === targetId && entry.webSocketDebuggerUrl) || null;
    }, `page target ${targetId}`);
    const connection = await new CdpConnection(target.webSocketDebuggerUrl, `page ${targetId}`).open();
    return new BrowserPage(connection, targetId, this.connection).initialise();
  }

  async stop() {
    if (this.connection) {
      try {
        await this.connection.send('Browser.close', {}, 5_000);
      } catch {
        // The browser may close the socket before acknowledging Browser.close.
      }
      this.connection.close();
    }
    if (this.process && this.process.exitCode === null) this.process.kill();
    if (this.profileDir) {
      const resolvedProfile = path.resolve(this.profileDir);
      const safePrefix = path.resolve(tmpdir(), 'bite-simulator-qa-');
      if (resolvedProfile.startsWith(safePrefix)) {
        for (let attempt = 0; attempt < 20 && existsSync(resolvedProfile); attempt += 1) {
          try {
            rmSync(resolvedProfile, { recursive: true, force: true });
          } catch {
            await delay(100);
          }
        }
      }
    }
  }
}

function dxfFromEdges(edges, zCoordinates = null) {
  const records = ['0', 'SECTION', '2', 'ENTITIES'];
  edges.forEach(([start, end], index) => {
    records.push(
      '0', 'LINE',
      '8', 'QA',
      '10', String(start[0]),
      '20', String(start[1]),
    );
    if (zCoordinates) records.push('30', String(zCoordinates[index][0]));
    records.push(
      '11', String(end[0]),
      '21', String(end[1]),
    );
    if (zCoordinates) records.push('31', String(zCoordinates[index][1]));
  });
  records.push('0', 'ENDSEC', '0', 'EOF');
  return records.join('\n');
}

const closedSquareEdges = [
  [[-30, -30], [30, -30]],
  [[30, -30], [30, 30]],
  [[30, 30], [-30, 30]],
  [[-30, 30], [-30, -30]],
];

const openSquareEdges = [
  [[-30, -30], [30, -30]],
  [[30, -30], [30, 30]],
  [[30, 30], [-30, 30]],
  [[-30, 30], [-30, -29.8]],
];

const validDxf = dxfFromEdges(closedSquareEdges);
const threeDimensionalDxf = dxfFromEdges(closedSquareEdges, [
  [0, 0],
  [0, 1],
  [1, 0],
  [0, 0],
]);
const openGapDxf = dxfFromEdges(openSquareEdges);

function approxEqual(actual, expected, tolerance, message) {
  assert.ok(Number.isFinite(actual), `${message}: ${actual} is not finite`);
  assert.ok(Math.abs(actual - expected) <= tolerance, `${message}: expected ${expected} +/- ${tolerance}, got ${actual}`);
}

function pageUrl(query) {
  const url = pathToFileURL(indexPath);
  url.search = query;
  return url.href;
}

async function main() {
  const nodeMajor = Number(process.versions.node.split('.')[0]);
  assert.ok(nodeMajor >= REQUIRED_NODE_MAJOR, `Node ${REQUIRED_NODE_MAJOR}+ is required; found ${process.version}`);
  assert.ok(existsSync(indexPath), `Missing application entry point: ${indexPath}`);

  const options = parseArguments(process.argv.slice(2));
  if (options.help) {
    console.log(usage());
    return;
  }

  const executable = findBrowser(options.browser);
  const harness = new BrowserHarness(executable);
  const results = [];
  let page = null;
  let compatPage = null;

  async function test(name, callback) {
    const started = performance.now();
    try {
      await callback();
      results.push({ name, ok: true, duration: performance.now() - started });
      console.log(`ok ${results.length} - ${name}`);
    } catch (error) {
      results.push({ name, ok: false, duration: performance.now() - started, error });
      console.log(`not ok ${results.length} - ${name}`);
      console.log(`  ${String(error.stack || error).replace(/\n/g, '\n  ')}`);
    }
  }

  async function withFreshPage(callback) {
    const isolatedPage = await harness.page(pageUrl('?qa'));
    try {
      return await callback(isolatedPage);
    } finally {
      await isolatedPage.close().catch(() => {});
    }
  }

  try {
    await harness.start();
    console.log(`# Browser: ${executable}`);
    console.log(`# URL: ${pageUrl('?qa')}`);
    page = await harness.page(pageUrl('?qa'));

    await test('starts from file:// with the Rapier SIMD backend', async () => {
      const value = await page.evaluate(() => ({
        loadState: window.BiteRapierLoadState,
        state: window.BiteSim.getState(),
        status: {
          className: document.querySelector('#appStatus')?.className,
          text: document.querySelector('#appStatus span')?.textContent,
        },
      }));
      assert.equal(value.loadState.backend, 'simd');
      assert.equal(value.loadState.simd, true);
      assert.equal(value.loadState.fallback, false);
      assert.equal(value.loadState.selectedAsset, 'rapier2d-simd-compat-0.20.0.global.js');
      assert.equal(value.state.physics.backend, 'simd');
      assert.equal(value.state.physics.backendVersion, '0.20.0');
      assert.match(value.status.className, /status-pill/);
      assert.ok(value.status.text?.trim(), 'application status must not be blank');
    });

    await test('exposes a valid default 1.36 kg CAD/CAD state', async () => {
      const value = await page.evaluate(() => ({
        state: window.BiteSim.getState(),
        angleInput: document.querySelector('[data-param="weaponInitialAngle"]').value,
        frameAngle: window.BiteSim.__qaCurrentFrame().angle,
      }));
      const { state } = value;
      assert.equal(state.physics.mode, 'rapier-rig');
      assert.equal(state.physics.massBudget.robot, 1.36);
      assert.ok(state.physics.massBudget.weapon > 0);
      assert.ok(state.physics.massBudget.fork > 0);
      assert.ok(state.physics.massBudget.chassis > 0);
      assert.equal(state.activeGeometry.weapon.source, 'cad');
      assert.equal(state.activeGeometry.fork.source, 'cad');
      assert.ok(state.activeGeometry.weapon.colliderCount > 0);
      assert.ok(state.activeGeometry.fork.colliderCount > 0);
      assert.equal(state.cad.holeCount, 12);
      approxEqual(state.cad.triangulatedArea, state.cad.netArea, 1e-12, 'weapon triangles must preserve hole area');
      assert.equal(state.activeGeometry.fork.holeCount, 3);
      assert.equal(state.activeGeometry.fork.massPropertySource, 'cad-uniform-lamina-with-holes');
      assert.equal(state.activeGeometry.fork.massProperties.mass, state.physics.massBudget.fork);
      assert.ok(Number.isFinite(state.activeGeometry.fork.massProperties.com.x));
      assert.ok(Number.isFinite(state.activeGeometry.fork.massProperties.com.y));
      assert.ok(state.activeGeometry.fork.massProperties.inertia > 0);
      assert.ok(state.activeGeometry.fork.massProperties.area > 0);
      assert.equal(state.toothHitCount, 0);
      assert.equal(state.bodyImpactCount, 0);
      assert.equal(state.time, 0);
      assert.equal(value.angleInput, '75');
      approxEqual(value.frameAngle, 75 * Math.PI / 180, 1e-12, 'default initial angle');
      approxEqual(state.target.positionMm.x, 380, 1e-12, 'default target X');
    });

    await test('BiteSim.setParams and BiteSim.advance commit deterministic fixed steps', async () => {
      const value = await page.evaluate(() => {
        window.BiteSim.reset();
        const params = window.BiteSim.setParams({ contactModel: 'rigid', simulationDuration: 0.02 });
        const advanced = window.BiteSim.advance(2);
        return { params, advanced, state: window.BiteSim.getState() };
      });
      assert.equal(value.params.contactModel, 'rigid');
      assert.equal(value.params.simulationDuration, 0.02);
      approxEqual(value.advanced.time, 0.001, 1e-12, 'two fixed steps');
      approxEqual(value.state.time, value.advanced.time, 1e-12, 'public state time');
      assert.equal(value.state.physics.mode, 'rapier-rig');
      assert.equal(value.advanced.solverDomainStopped, false);
    });

    await test('builds all CAD/parametric combinations and 1/10/20-tooth rigs', async () => {
      await withFreshPage(async (geometryPage) => {
        const records = await geometryPage.evaluate(() => {
          const scenarios = [
            { id: 'cad-cad', params: { paramWeaponEnabled: false, paramForkEnabled: false } },
            { id: 'param-cad', params: { paramWeaponEnabled: true, paramForkEnabled: false, paramToothCount: 1 } },
            { id: 'cad-param', params: { paramWeaponEnabled: false, paramForkEnabled: true } },
            { id: 'param-param', params: { paramWeaponEnabled: true, paramForkEnabled: true, paramToothCount: 1 } },
            { id: 'teeth-10', params: { paramWeaponEnabled: true, paramForkEnabled: false, paramToothCount: 10 } },
            { id: 'teeth-20', params: { paramWeaponEnabled: true, paramForkEnabled: false, paramToothCount: 20 } },
          ];
          return scenarios.map((scenario) => {
            document.querySelector('#restoreDefaults').click();
            window.BiteSim.setParams({ contactModel: 'rigid', simulationDuration: 0.8, ...scenario.params });
            let state = window.BiteSim.getState(); let steps = 0;
            if (!scenario.id.startsWith('teeth-')) {
              while (steps < 1600
                && state.toothHitCount === 0
                && state.bodyImpactCount === 0
                && !state.physics.solverDomainStopped
                && !state.physics.modelDomainStopped) {
                window.BiteSim.advance(1); steps += 1; state = window.BiteSim.getState();
              }
            }
            return {
              id: scenario.id,
              mode: state.physics.mode,
              creationError: state.physics.creationError,
              weaponSource: state.activeGeometry.weapon.source,
              forkSource: state.activeGeometry.fork.source,
              toothCount: state.activeGeometry.weapon.toothCount,
              weaponColliders: state.activeGeometry.weapon.colliderCount,
              forkColliders: state.activeGeometry.fork.colliderCount,
              steps,
              contacts: state.toothHitCount + state.bodyImpactCount,
              maxAcceptedPenetration: state.physics.maxAcceptedPenetration,
              massBudget: state.physics.massBudget,
            };
          });
        }, undefined, LONG_SIMULATION_TIMEOUT_MS);
        for (const record of records) {
          assert.equal(record.mode, 'rapier-rig', `${record.id} mode`);
          assert.equal(record.creationError, null, `${record.id} creation error`);
          assert.ok(record.weaponColliders > 0, `${record.id} weapon colliders`);
          assert.ok(record.forkColliders > 0, `${record.id} fork colliders`);
          const massSum = record.massBudget.weapon + record.massBudget.fork + record.massBudget.chassis;
          approxEqual(massSum, record.massBudget.robot, 1e-12, `${record.id} mass budget`);
          if (!record.id.startsWith('teeth-')) {
            assert.ok(record.contacts > 0, `${record.id} did not reach a load-bearing weapon contact in ${record.steps} steps`);
            for (const [role, penetration] of Object.entries(record.maxAcceptedPenetration)) {
              assert.ok(penetration <= 0.00008 + 1e-9, `${record.id} ${role} penetration gate`);
            }
          }
        }
        assert.equal(records.find((entry) => entry.id === 'teeth-10').toothCount, 10);
        assert.equal(records.find((entry) => entry.id === 'teeth-20').toothCount, 20);
      });
    });

    await test('builds deterministic gear-ratio and initial-angle variants', async () => {
      await withFreshPage(async (matrixPage) => {
        const records = await matrixPage.evaluate(() => {
          const ratios = [0.59, 1, 2.1, 4];
          const angles = [15, 75, 210];
          return ratios.flatMap((weaponGearRatio) => angles.map((weaponInitialAngle) => {
            window.BiteSim.setParams({ weaponGearRatio, weaponInitialAngle, contactModel: 'rigid' });
            const state = window.BiteSim.getState();
            const metrics = window.BiteSim.getMetrics();
            return { weaponGearRatio, weaponInitialAngle, mode: state.physics.mode, rpm: metrics.weaponRpm };
          }));
        });
        for (const record of records) {
          assert.equal(record.mode, 'rapier-rig', `${record.weaponGearRatio}:1 at ${record.weaponInitialAngle} degrees`);
          approxEqual(record.rpm, 1750 * 14.8 / record.weaponGearRatio, 1e-8, 'gear ratio RPM');
        }
      });
    });

    await test('parametric rake sign is invariant under single-axis mirrors', async () => {
      await withFreshPage(async (rakePage) => {
        const records = await rakePage.evaluate(() => [-25, 25].flatMap((phase) => [
          { weaponMirrorX: false, weaponMirrorY: false },
          { weaponMirrorX: true, weaponMirrorY: false },
          { weaponMirrorX: false, weaponMirrorY: true },
        ].map((mirrors) => {
          window.BiteSim.setParams({
            paramWeaponEnabled: true,
            paramToothPhaseDeg: phase,
            ...mirrors,
          });
          const weapon = window.BiteSim.getState().activeGeometry.weapon;
          return { phase, ...mirrors, input: weapon.rakeInputDeg, effective: weapon.rakeEffectiveDeg };
        })));
        for (const record of records) {
          assert.equal(record.input, record.phase);
          assert.equal(record.effective, record.phase);
        }
      });
    });

    await test('Rapier snapshot round-trip restores bodies, joints, and collider metadata', async () => {
      const value = await page.evaluate(() => {
        document.querySelector('#restoreDefaults').click();
        const before = window.BiteSim.getState();
        const snapshot = window.BiteSim.snapshotRoundTrip();
        const after = window.BiteSim.getState();
        return { before, snapshot, after };
      });
      assert.equal(value.snapshot.ok, true);
      assert.ok(value.snapshot.weaponColliderCount > 0);
      assert.ok(value.snapshot.forkColliderCount > 0);
      assert.equal(value.snapshot.weaponHandlesMapped, true);
      assert.equal(value.snapshot.forkHandlesMapped, true);
      assert.equal(value.snapshot.forkBodyRestored, true);
      assert.equal(value.snapshot.forkJointRestored, true);
      assert.equal(value.snapshot.forkGroundRestored, true);
      assert.equal(value.after.target.grounded, value.before.target.grounded);
      approxEqual(value.after.target.pos.x, value.before.target.pos.x, 1e-7, 'target X after restore');
      approxEqual(value.after.target.pos.y, value.before.target.pos.y, 1e-7, 'target Y after restore');
      approxEqual(value.after.target.vel.x, value.before.target.vel.x, 1e-7, 'target X velocity after restore');
      approxEqual(value.after.target.vel.y, value.before.target.vel.y, 1e-7, 'target Y velocity after restore');
      approxEqual(value.after.target.angle, value.before.target.angle, 1e-7, 'target angle after restore');
      approxEqual(value.after.target.omega, value.before.target.omega, 1e-7, 'target angular velocity after restore');
      assert.equal(value.after.fork.grounded, value.before.fork.grounded);
      approxEqual(value.after.fork.origin.x, value.before.fork.origin.x, 1e-7, 'fork X after restore');
      approxEqual(value.after.fork.origin.y, value.before.fork.origin.y, 1e-7, 'fork Y after restore');
      approxEqual(value.after.fork.velocity.x, value.before.fork.velocity.x, 1e-7, 'fork X velocity after restore');
      approxEqual(value.after.fork.velocity.y, value.before.fork.velocity.y, 1e-7, 'fork Y velocity after restore');
      approxEqual(value.after.fork.angle, value.before.fork.angle, 1e-7, 'fork angle after restore');
      approxEqual(value.after.fork.omega, value.before.fork.omega, 1e-7, 'fork angular velocity after restore');
      approxEqual(value.after.weaponOmega, value.before.weaponOmega, 1e-12, 'weapon angular velocity after restore');
    });

    await test('invalid parameter transactions and advanced-panel toggles preserve physics state', async () => {
      await withFreshPage(async (transactionPage) => {
        const value = await transactionPage.evaluate(() => {
          window.BiteSim.advance(2);
          const before = window.BiteSim.getState();
          let error = null;
          try {
            window.BiteSim.setParams({ weaponMass: 0.000001, weaponInertia: 1e9 });
          } catch (caught) {
            error = String(caught?.message || caught);
          }
          const afterRejected = window.BiteSim.getState();
          document.querySelector('#toggleAdvanced').click();
          const afterToggle = window.BiteSim.getState();
          return { before, afterRejected, afterToggle, error };
        });
        assert.ok(value.error, 'invalid mass/inertia transaction must throw');
        assert.deepEqual(value.afterRejected, value.before);
        assert.deepEqual(value.afterToggle, value.before);
      });
    });

    await test('every non-1.36 kg preset requests explicit weapon and fork masses', async () => {
      const records = await page.evaluate(() => {
        const presetIds = ['150g', '220g', '454g', '5lb', '13.6kg', '110kg'];
        return presetIds.map((presetId) => {
          document.querySelector('#restoreDefaults').click();
          window.BiteSim.setParams({ weaponMass: 0.12, forkMass: 0.02 });
          const preset = document.querySelector('#robotPreset');
          preset.value = presetId;
          preset.dispatchEvent(new Event('change', { bubbles: true }));
          document.querySelector('#applyRobotPreset').click();
          const state = window.BiteSim.getState();
          return {
            presetId,
            mode: state.physics.mode,
            creationError: state.physics.creationError,
            massBudget: state.physics.massBudget,
            weaponMassInput: document.querySelector('[data-param="weaponMass"]').value,
            forkMassInput: document.querySelector('[data-param="forkMass"]').value,
            playDisabled: document.querySelector('#playPause').disabled,
            stepDisabled: document.querySelector('#stepSimulation').disabled,
            seekDisabled: document.querySelector('#timelineSeek').disabled,
          };
        });
      });
      for (const record of records) {
        assert.equal(record.mode, 'needs-input', `${record.presetId} must enter needs-input`);
        assert.equal(record.weaponMassInput, '', `${record.presetId} must not inherit weapon mass`);
        assert.equal(record.forkMassInput, '', `${record.presetId} must not inherit fork mass`);
        assert.equal(record.massBudget.weapon, null, `${record.presetId} weapon mass budget must be absent`);
        assert.equal(record.massBudget.fork, null, `${record.presetId} fork mass budget must be absent`);
        assert.equal(record.playDisabled, true, `${record.presetId} play must be disabled`);
        assert.equal(record.stepDisabled, true, `${record.presetId} step must be disabled`);
        assert.equal(record.seekDisabled, true, `${record.presetId} seek must be disabled`);
      }
    });

    await test('a needs-input preset becomes solvable only after both masses are supplied', async () => {
      await withFreshPage(async (massPage) => {
        const value = await massPage.evaluate(() => {
          const preset = document.querySelector('#robotPreset');
          preset.value = '150g';
          preset.dispatchEvent(new Event('change', { bubbles: true }));
          document.querySelector('#applyRobotPreset').click();
          const missing = window.BiteSim.getState();
          const commitMass = (name, value) => {
            const input = document.querySelector(`[data-param="${name}"]`);
            input.value = String(value);
            input.dispatchEvent(new Event('input', { bubbles: true }));
            input.dispatchEvent(new Event('change', { bubbles: true }));
            return { value: input.value, valid: input.checkValidity() };
          };
          const weaponInput = commitMass('weaponMass', 0.02);
          const oneMassMode = window.BiteSim.getState().physics.mode;
          const oneMassSolverStopped = window.BiteSim.getState().physics.solverDomainStopped;
          const forkInput = commitMass('forkMass', 0.01);
          const complete = window.BiteSim.getState();
          return {
            missingMode: missing.physics.mode,
            completeMode: complete.physics.mode,
            completeBudget: complete.physics.massBudget,
            playDisabled: document.querySelector('#playPause').disabled,
            weaponInput,
            forkInput,
            oneMassMode,
            oneMassSolverStopped,
          };
        });
        assert.equal(value.missingMode, 'needs-input');
        assert.equal(value.oneMassMode, 'needs-input');
        assert.equal(value.oneMassSolverStopped, false);
        assert.equal(value.completeMode, 'rapier-rig');
        approxEqual(value.completeBudget.chassis, 0.12, 1e-12, '150 g chassis mass');
        assert.deepEqual(value.weaponInput, { value: '0.02', valid: true });
        assert.deepEqual(value.forkInput, { value: '0.01', valid: true });
        assert.equal(value.playDisabled, false);
      });
    });

    await test('imports a supported closed planar ASCII DXF', async () => {
      await withFreshPage(async (dxfPage) => {
        const value = await dxfPage.evaluate(({ dxf }) => {
          window.BiteSim.importDxfText('weapon', dxf, 'qa-valid.dxf');
          const needsInput = window.BiteSim.getState();
          document.querySelector('#applyRobotPreset').click();
          const afterPresetReapply = window.BiteSim.getState();
          window.BiteSim.setParams({ weaponMass: 0.5 });
          const solved = window.BiteSim.getState();
          let unitError = null;
          try {
            window.BiteSim.setParams({ dxfUnit: 'in' });
          } catch (caught) {
            unitError = String(caught?.message || caught);
          }
          return {
            summary: document.querySelector('#fileSummary').textContent,
            needsInput,
            afterPresetReapply,
            state: solved,
            afterUnitReject: window.BiteSim.getState(),
            unitError,
            dxfUnitInput: document.querySelector('[data-param="dxfUnit"]').value,
          };
        }, { dxf: validDxf });
        assert.match(value.summary, /qa-valid\.dxf/);
        assert.equal(value.needsInput.physics.mode, 'needs-input');
        assert.equal(value.needsInput.physics.massBudget.weapon, null);
        assert.equal(value.afterPresetReapply.physics.mode, 'needs-input');
        assert.equal(value.afterPresetReapply.physics.massBudget.weapon, null);
        approxEqual(value.afterPresetReapply.weaponScene.y, 0.05775, 1e-12, 'preset reapply must preserve the valid installation height');
        assert.ok(value.afterPresetReapply.physics.minForkFloorClearance >= -1e-9);
        assert.ok(value.unitError, 'invalid unit rescale must throw');
        assert.equal(value.dxfUnitInput, 'mm');
        assert.deepEqual(value.afterUnitReject, value.state, 'failed unit rescale must preserve the drawing and simulation');
        assert.ok(value.state.cad.sourceOutlinePoints >= 4);
        assert.ok(value.state.activeGeometry.weapon.colliderCount > 0);
        assert.deepEqual(dxfPage.exceptions, []);
        assert.deepEqual(dxfPage.consoleErrors, []);
      });
    });

    await test('all DXF fixtures follow the real importer contract', async () => {
      await withFreshPage(async (dxfPage) => {
        const records = await dxfPage.evaluate((fixtures) => fixtures.map((fixture) => {
          document.querySelector('#restoreDefaults').click();
          const before = document.querySelector('#fileSummary').textContent;
          let error = null; let errorCode = null; let state = null;
          try {
            window.BiteSim.importDxfText('weapon', fixture.text, `${fixture.id}.dxf`);
            if (fixture.expected.accepted) window.BiteSim.setParams({ weaponMass: 0.5 });
            state = window.BiteSim.getState();
          } catch (caught) {
            error = String(caught?.message || caught);
            errorCode = caught?.code || null;
          }
          return {
            id: fixture.id,
            accepted: !error,
            errorCode,
            preserved: document.querySelector('#fileSummary').textContent === before,
            cad: state?.cad || null,
          };
        }), DXF_FIXTURES);
        for (const record of records) {
          const fixture = DXF_FIXTURES.find((candidate) => candidate.id === record.id);
          assert.equal(record.accepted, fixture.expected.accepted, `${record.id} acceptance`);
          if (!fixture.expected.accepted) {
            assert.equal(record.errorCode, fixture.expected.errorCode, `${record.id} error code`);
            assert.equal(record.preserved, true, `${record.id} must preserve the previous drawing`);
          }
          if (record.id === 'outer-with-hole') {
            assert.equal(record.cad.holeCount, 1);
            approxEqual(record.cad.netArea * 1e6, 1000, 1e-6, 'hole fixture net area in mm2');
            approxEqual(record.cad.triangulatedArea, record.cad.netArea, 1e-12, 'hole fixture triangulated area');
          }
        }
        assert.deepEqual(dxfPage.exceptions, []);
        assert.deepEqual(dxfPage.consoleErrors, []);
      });
    });

    await test('a target wholly inside a DXF hole has no solid overlap or contact', async () => {
      await withFreshPage(async (holePage) => {
        const fixture = DXF_FIXTURES.find((candidate) => candidate.id === 'outer-with-hole');
        const value = await holePage.evaluate(({ dxf }) => {
          window.BiteSim.importDxfText('weapon', dxf, 'qa-hole-clearance.dxf');
          const needsInput = window.BiteSim.getState();
          window.BiteSim.setParams({
            contactModel: 'rigid',
            weaponMirrorX: false,
            weaponInitialAngle: 0,
            targetLength: 5,
            targetThickness: 5,
            targetSceneX: 20,
            targetClearance: 70.25,
            simulationDuration: 0.002,
            weaponMass: 0.2,
          });
          const initial = window.BiteSim.getState();
          window.BiteSim.advance(1);
          return { needsInput, initial, advanced: window.BiteSim.getState() };
        }, { dxf: fixture.text });
        assert.equal(value.needsInput.physics.mode, 'needs-input');
        assert.equal(value.initial.physics.mode, 'rapier-rig');
        assert.equal(value.initial.physics.creationError, null);
        assert.equal(value.initial.cad.holeCount, 1);
        assert.equal(value.advanced.toothHitCount, 0);
        assert.equal(value.advanced.bodyImpactCount, 0);
        assert.equal(value.advanced.physics.maxAcceptedPenetration.weapon, 0);
      });
    });

    await test('contact geometry-to-solver matching is deterministic and one-to-one', async () => {
      const value = await page.evaluate(() => window.BiteSim.__qaMatchGeometricContactsToSolver(
        [
          { midpoint: { x: 0.02, y: 0 } },
          { midpoint: { x: 0.01, y: 0 } },
          { midpoint: { x: 0.50, y: 0 } },
        ],
        [-0.002, -0.001, 0.1],
        [
          { point: { x: 0.01, y: 0 }, distance: -0.001 },
          { point: { x: 0.02, y: 0 }, distance: -0.002 },
        ],
        1e-9,
      ));
      assert.deepEqual(value.pairs, [
        { geometricContactIndex: 0, solverContactIndex: 1 },
        { geometricContactIndex: 1, solverContactIndex: 0 },
      ]);
      assert.deepEqual(value.unmatchedGeometricContacts, [2]);
      assert.deepEqual(value.unmatchedSolverContacts, []);
    });

    await test('preset target placement is independent of the previous simulation pose', async () => {
      await withFreshPage(async (presetPage) => {
        const value = await presetPage.evaluate(() => {
          const apply = () => {
            const select = document.querySelector('#robotPreset');
            select.value = '110kg';
            select.dispatchEvent(new Event('change', { bubbles: true }));
            document.querySelector('#applyRobotPreset').click();
            window.BiteSim.setParams({ weaponMass: 10, forkMass: 5 });
            const state = window.BiteSim.getState();
            return {
              targetX: state.target.positionMm.x,
              forkInitialAngle: state.activeGeometry.fork.initialAngle,
              weaponScene: state.weaponScene,
            };
          };
          window.BiteSim.setParams({ contactModel: 'rigid', simulationDuration: 0.05 });
          window.BiteSim.advance(100);
          const afterMotion = apply();
          document.querySelector('#restoreDefaults').click();
          const fromFreshState = apply();
          return { afterMotion, fromFreshState };
        }, undefined, LONG_SIMULATION_TIMEOUT_MS);
        approxEqual(value.afterMotion.targetX, value.fromFreshState.targetX, 1e-9, '110 kg target X must not depend on history');
        approxEqual(value.afterMotion.forkInitialAngle, value.fromFreshState.forkInitialAngle, 1e-12, '110 kg fork pose must not depend on history');
        assert.deepEqual(value.afterMotion.weaponScene, value.fromFreshState.weaponScene, '110 kg weapon origin must not depend on history');
      });
    });

    await test('rejects non-planar DXF instead of flattening Z coordinates', async () => {
      await withFreshPage(async (dxfPage) => {
        const value = await dxfPage.evaluate(({ dxf }) => {
          window.BiteSim.advance(2);
          const before = document.querySelector('#fileSummary').textContent;
          const beforeState = window.BiteSim.getState();
          let error = null;
          try {
            window.BiteSim.importDxfText('weapon', dxf, 'qa-3d.dxf');
          } catch (caught) {
            error = String(caught?.message || caught);
          }
          let stateError = null;
          try {
            window.BiteSim.getState();
          } catch (caught) {
            stateError = String(caught?.message || caught);
          }
          return { before, after: document.querySelector('#fileSummary').textContent, beforeState, afterState: window.BiteSim.getState(), error, stateError };
        }, { dxf: threeDimensionalDxf });
        assert.ok(value.error, 'non-planar DXF must throw');
        assert.match(value.error, /3D|Z|planar|plane|XY/i);
        assert.equal(value.after, value.before, 'a rejected import must preserve the previous drawing');
        assert.deepEqual(value.afterState, value.beforeState, 'a rejected import must preserve simulation state');
        assert.equal(value.stateError, null, 'state must remain readable after a rejected import');
        assert.deepEqual(dxfPage.exceptions, []);
        assert.deepEqual(dxfPage.consoleErrors, []);
      });
    });

    await test('rejects a nominally closed DXF outline with a 0.2 mm gap', async () => {
      await withFreshPage(async (dxfPage) => {
        const value = await dxfPage.evaluate(({ dxf }) => {
          const before = document.querySelector('#fileSummary').textContent;
          let error = null;
          try {
            window.BiteSim.importDxfText('weapon', dxf, 'qa-open-0.2mm.dxf');
          } catch (caught) {
            error = String(caught?.message || caught);
          }
          let stateError = null;
          try {
            window.BiteSim.getState();
          } catch (caught) {
            stateError = String(caught?.message || caught);
          }
          return { before, after: document.querySelector('#fileSummary').textContent, error, stateError };
        }, { dxf: openGapDxf });
        assert.ok(value.error, 'an open 0.2 mm outline must throw');
        assert.match(value.error, /open|gap|closed|closure|0\.2/i);
        assert.equal(value.after, value.before, 'a rejected import must preserve the previous drawing');
        assert.equal(value.stateError, null, 'state must remain readable after a rejected import');
        assert.deepEqual(dxfPage.exceptions, []);
        assert.deepEqual(dxfPage.consoleErrors, []);
      });
    });

    await test('the first load-bearing default rigid weapon contact is a tooth', async () => {
      await withFreshPage(async (contactPage) => {
        const value = await contactPage.evaluate(() => {
          window.BiteSim.setParams({ contactModel: 'rigid', simulationDuration: 1.25 });
          let state = window.BiteSim.getState();
          let steps = 0;
          while (steps < 2500
            && state.toothHitCount === 0
            && state.bodyImpactCount === 0
            && !state.physics.solverDomainStopped
            && !state.trajectory.completed) {
            window.BiteSim.advance(1);
            steps += 1;
            state = window.BiteSim.getState();
          }
          let settlementSteps = 0;
          while (settlementSteps < 200
            && state.lastImpact
            && state.lastImpact.energyConverged !== true
            && !state.physics.solverDomainStopped) {
            window.BiteSim.advance(1);
            settlementSteps += 1;
            state = window.BiteSim.getState();
          }
          return {
            steps,
            settlementSteps,
            time: state.time,
            toothHitCount: state.toothHitCount,
            bodyImpactCount: state.bodyImpactCount,
            lastImpact: state.lastImpact,
            maxAcceptedPenetration: state.physics.maxAcceptedPenetration,
            solverDomainStopped: state.physics.solverDomainStopped,
            events: [...document.querySelectorAll('#eventLog li')].map((entry) => entry.textContent),
          };
        }, undefined, LONG_SIMULATION_TIMEOUT_MS);
        assert.equal(value.solverDomainStopped, false, 'solver stopped before first weapon contact');
        assert.equal(value.bodyImpactCount, 0, `body/backplate contacted first at ${value.time}s`);
        assert.equal(value.toothHitCount, 1, `no load-bearing tooth contact within ${value.steps} steps`);
        assert.equal(value.lastImpact?.episodeComplete, true);
        assert.equal(value.lastImpact?.energyConverged, true, 'completed tooth episode must close its energy ledger');
        const sourceEnergy = value.lastImpact.rotorEnergyLoss
          + value.lastImpact.chassisEnergyLoss
          + value.lastImpact.forkEnergyLoss
          + value.lastImpact.externalWork;
        const namedDissipation = Math.max(0, value.lastImpact.materialWork)
          + Math.max(0, value.lastImpact.boundaryWork)
          + value.lastImpact.constraintEnergyExchange
          + value.lastImpact.massRemovalEnergy;
        approxEqual(
          sourceEnergy - value.lastImpact.targetMechanicalGain - namedDissipation,
          value.lastImpact.unclassifiedEnergy,
          1e-10,
          'episode energy ledger residual',
        );
        assert.ok(Math.abs(value.lastImpact.unclassifiedEnergy) <= value.lastImpact.energyTolerance);
        assert.ok(value.lastImpact.numericalEnergyGain <= value.lastImpact.energyTolerance);
        for (const [role, penetration] of Object.entries(value.maxAcceptedPenetration)) {
          assert.ok(penetration > 0, `${role} contact path was not exercised`);
          assert.ok(penetration <= 0.00008 + 1e-9, `${role} accepted penetration exceeds the 0.08 mm gate: ${penetration}`);
        }
        assert.deepEqual(contactPage.exceptions, []);
        assert.deepEqual(contactPage.consoleErrors, []);
      });
    });

    await test('a body-first phase remains a separate non-tooth contact class', async () => {
      await withFreshPage(async (bodyPage) => {
        const value = await bodyPage.evaluate(() => {
          window.BiteSim.setParams({ contactModel: 'rigid', weaponInitialAngle: 0, simulationDuration: 0.4 });
          let state = window.BiteSim.getState(); let steps = 0;
          while (steps < 800
            && state.toothHitCount === 0
            && state.bodyImpactCount === 0
            && !state.physics.solverDomainStopped) {
            window.BiteSim.advance(1); steps += 1; state = window.BiteSim.getState();
          }
          return { steps, state };
        }, undefined, LONG_SIMULATION_TIMEOUT_MS);
        assert.equal(value.state.physics.solverDomainStopped, false);
        assert.equal(value.state.toothHitCount, 0);
        assert.equal(value.state.bodyImpactCount, 1);
        assert.equal(value.state.lastImpact?.bodyContact, true);
      });
    });

    await test('material-domain stop commits no provisional impact or material removal', async () => {
      await withFreshPage(async (materialPage) => {
        const value = await materialPage.evaluate(() => {
          window.BiteSim.setParams({ contactModel: 'material', simulationDuration: 0.4 });
          let state = window.BiteSim.getState(); let steps = 0;
          while (steps < 800 && !state.physics.modelDomainStopped && !state.physics.solverDomainStopped) {
            window.BiteSim.advance(1); steps += 1; state = window.BiteSim.getState();
          }
          const stopped = state;
          window.BiteSim.advance(1);
          const repeated = window.BiteSim.getState();
          return { steps, stopped, repeated };
        }, undefined, LONG_SIMULATION_TIMEOUT_MS);
        assert.equal(value.stopped.physics.modelDomainStopped, true);
        assert.equal(value.stopped.physics.solverDomainStopped, false);
        assert.equal(value.stopped.toothHitCount, 0);
        assert.equal(value.stopped.bodyImpactCount, 0);
        assert.equal(value.stopped.lastImpact, null);
        assert.equal(value.stopped.material.removedArea, 0);
        assert.equal(value.stopped.material.removedMass, 0);
        assert.equal(value.stopped.material.work, 0);
        assert.deepEqual(value.repeated, value.stopped, 'advancing a terminal safe frame must be idempotent');
      });
    });

    await test('trajectory cache yields and random frame seeks are deterministic', async () => {
      await withFreshPage(async (seekPage) => {
        const value = await seekPage.evaluate(async () => {
          window.BiteSim.setParams({ contactModel: 'rigid', simulationDuration: 0.006 });
          window.BiteSim.__qaSetBoundaryLimit(1);
          window.BiteSim.__qaStartTrajectory();
          const deadline = performance.now() + 20_000;
          while (!window.BiteSim.__qaTrajectoryDiagnostics()?.ready) {
            if (performance.now() > deadline) throw new Error('trajectory QA timeout');
            await new Promise((resolve) => setTimeout(resolve, 5));
          }
          const diagnostics = window.BiteSim.__qaTrajectoryDiagnostics();
          const order = [7, 2, 10, 0, 7];
          const applied = order.map((index) => ({
            requested: index,
            frame: window.BiteSim.__qaApplyFrame(index),
            state: window.BiteSim.getState(),
          }));
          return { diagnostics, applied };
        }, undefined, LONG_SIMULATION_TIMEOUT_MS);
        assert.equal(value.diagnostics.ready, true);
        assert.ok(value.diagnostics.yieldCount > 0, 'forced boundary scheduling must yield');
        assert.equal(value.diagnostics.frames, 13);
        for (const entry of value.applied) approxEqual(entry.state.time, entry.frame.time, 1e-12, `seek frame ${entry.requested}`);
        assert.deepEqual(value.applied[0].frame, value.applied[4].frame, 'revisiting a cached frame must reproduce it exactly');
        assert.deepEqual(value.applied[0].state, value.applied[4].state, 'public state must be identical after a repeated seek');
      });
    });

    compatPage = await harness.page(pageUrl('?qa&qaForceRapierCompat'));
    await test('falls back from SIMD to the compat backend when SIMD init fails', async () => {
      const value = await compatPage.evaluate(() => ({
        loadState: window.BiteRapierLoadState,
        state: window.BiteSim.getState(),
      }));
      assert.equal(value.loadState.requestedBackend, 'simd');
      assert.equal(value.loadState.backend, 'compat');
      assert.equal(value.loadState.simd, false);
      assert.equal(value.loadState.fallback, true);
      assert.ok(value.loadState.fallbackReason);
      assert.equal(value.loadState.attempts.length, 2);
      assert.equal(value.loadState.attempts[0].backend, 'simd');
      assert.equal(value.loadState.attempts[0].ok, false);
      assert.equal(value.loadState.attempts[1].backend, 'compat');
      assert.equal(value.loadState.attempts[1].ok, true);
      assert.equal(value.state.physics.backend, 'compat');
      assert.equal(value.state.physics.backendVersion, '0.20.0');
      assert.equal(value.state.physics.fallback, true);
      assert.equal(value.state.physics.mode, 'rapier-rig');
    });

    await test('does not emit uncaught browser exceptions during regression scenarios', async () => {
      assert.deepEqual(page.exceptions, []);
      assert.deepEqual(page.consoleErrors, []);
      assert.deepEqual(compatPage.exceptions, []);
      assert.deepEqual(compatPage.consoleErrors, []);
    });
  } finally {
    if (compatPage) await compatPage.close().catch(() => {});
    if (page) await page.close().catch(() => {});
    await harness.stop();
  }

  const failures = results.filter((result) => !result.ok);
  const duration = results.reduce((sum, result) => sum + result.duration, 0);
  console.log(`1..${results.length}`);
  console.log(`# ${results.length - failures.length} passed, ${failures.length} failed (${Math.round(duration)} ms test time)`);
  if (failures.length) process.exitCode = 1;
}

main().catch((error) => {
  console.error(error.stack || error);
  process.exitCode = 1;
});
