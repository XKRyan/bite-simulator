const { spawn } = require('node:child_process');
const { existsSync, mkdtempSync, readFileSync, rmSync, stat, readFile, writeFileSync } = require('node:fs');
const { createHash } = require('node:crypto');
const http = require('node:http');
const { tmpdir } = require('node:os');
const path = require('node:path');

const repo = path.resolve(__dirname, '..', '..');
const pageFile = process.argv[2] || 'edge-final.html';
if (!/^[a-z0-9-]+\.html$/i.test(pageFile)) throw new Error(`invalid audit page name: ${pageFile}`);
const pageMode = process.argv[3] || '';
if (pageMode && !/^[a-z0-9-]+$/i.test(pageMode)) throw new Error(`invalid audit page mode: ${pageMode}`);
const boundedForkHangAudit = pageFile === 'edge-min-fork.html';
const reportName = ({
  'edge-final.html': pageMode === 'prefix' ? 'dxf-prefix-report.json' : 'dxf-full-report.json',
  'edge-fork-fix.html': 'dxf-fork-report.json',
  'edge-min-weapon.html': 'dxf-weapon-report.json',
  'edge-tail.html': 'dxf-tail-report.json',
})[pageFile] || `${path.parse(pageFile).name}-report.json`;
const candidateSha256 = createHash('sha256').update(readFileSync(path.join(__dirname, 'app-combined-bundle.js'))).digest('hex').toUpperCase();
const edge = [
  'C:\\Program Files (x86)\\Microsoft\\Edge\\Application\\msedge.exe',
  'C:\\Program Files\\Microsoft\\Edge\\Application\\msedge.exe',
].find(existsSync);
if (!edge) throw new Error('Microsoft Edge not found');

const delay = (ms) => new Promise((resolve) => setTimeout(resolve, ms));
const profile = mkdtempSync(path.join(tmpdir(), 'bite-combined-s4c-dxf-edge-'));
const contentTypes = {
  '.html': 'text/html; charset=utf-8',
  '.js': 'text/javascript; charset=utf-8',
  '.mjs': 'text/javascript; charset=utf-8',
  '.css': 'text/css; charset=utf-8',
  '.json': 'application/json; charset=utf-8',
  '.svg': 'image/svg+xml',
  '.wasm': 'application/wasm',
};

const server = http.createServer((request, response) => {
  let pathname;
  try { pathname = decodeURIComponent(new URL(request.url, 'http://127.0.0.1').pathname); }
  catch { response.writeHead(400).end('Bad request'); return; }
  const resolved = path.resolve(repo, `.${pathname}`);
  const prefix = `${repo}${path.sep}`;
  if (resolved !== repo && !resolved.startsWith(prefix)) { response.writeHead(403).end('Forbidden'); return; }
  stat(resolved, (statError, info) => {
    const file = !statError && info.isDirectory() ? path.join(resolved, 'index.html') : resolved;
    readFile(file, (error, bytes) => {
      if (error) { response.writeHead(404).end('Not found'); return; }
      response.writeHead(200, {
        'Content-Type': contentTypes[path.extname(file).toLowerCase()] || 'application/octet-stream',
        'Cache-Control': 'no-store',
      });
      response.end(bytes);
    });
  });
});

const listen = () => new Promise((resolve, reject) => {
  server.once('error', reject);
  server.listen(0, '127.0.0.1', resolve);
});

const waitForDevtoolsPort = async (browser) => {
  const marker = path.join(profile, 'DevToolsActivePort');
  const deadline = Date.now() + 30000;
  while (Date.now() < deadline) {
    if (existsSync(marker)) {
      const port = Number(readFileSync(marker, 'utf8').split(/\r?\n/, 1)[0]);
      if (Number.isInteger(port) && port > 0) return port;
    }
    if (browser.exitCode !== null) throw new Error(`Edge exited before DevTools became ready (${browser.exitCode})`);
    await delay(50);
  }
  throw new Error('Timed out waiting for Edge DevTools port');
};

const waitForTarget = async (devtoolsPort) => {
  const deadline = Date.now() + 30000;
  while (Date.now() < deadline) {
    const response = await fetch(`http://127.0.0.1:${devtoolsPort}/json/list`);
    const targets = await response.json();
    const target = targets.find((entry) => entry.type === 'page' && entry.url.includes(`/s4c-private-toi-dxf-combined/${pageFile}`));
    if (target?.webSocketDebuggerUrl) return target;
    await delay(50);
  }
  throw new Error('Timed out waiting for the Edge audit page target');
};

const connect = (url) => new Promise((resolve, reject) => {
  const socket = new WebSocket(url);
  socket.addEventListener('open', () => resolve(socket), { once: true });
  socket.addEventListener('error', () => reject(new Error('Could not connect to Edge DevTools target')), { once: true });
});

const commandChannel = (socket) => {
  let sequence = 0;
  const pending = new Map();
  socket.addEventListener('message', (event) => {
    let message;
    try { message = JSON.parse(String(event.data)); } catch { return; }
    if (!message.id || !pending.has(message.id)) {
      if (message.method === 'Runtime.exceptionThrown') {
        console.error(`Edge page exception: ${message.params?.exceptionDetails?.text || 'unknown exception'}`);
      }
      return;
    }
    const { resolve, reject, timeout } = pending.get(message.id);
    pending.delete(message.id);
    clearTimeout(timeout);
    if (message.error) reject(new Error(`DevTools ${message.error.code}: ${message.error.message}`));
    else resolve(message.result);
  });
  return (method, params = {}) => new Promise((resolve, reject) => {
    sequence += 1;
    const id = sequence;
    const timeout = setTimeout(() => {
      pending.delete(id);
      reject(new Error(`DevTools command timed out: ${method}`));
    }, 20 * 1000);
    pending.set(id, { resolve, reject, timeout });
    socket.send(JSON.stringify({ id: sequence, method, params }));
  });
};

const closeBrowser = async (browser) => {
  if (!browser || browser.exitCode !== null) return;
  const closed = new Promise((resolve) => browser.once('close', resolve));
  browser.kill();
  await Promise.race([closed, delay(3000)]);
};

async function main() {
  let browser = null;
  let socket = null;
  try {
    await listen();
    const port = server.address().port;
    const url = `http://127.0.0.1:${port}/work/s4c-private-toi-dxf-combined/${pageFile}${pageMode ? `?${pageMode}` : ''}`;
    browser = spawn(edge, [
      '--headless=new',
      '--disable-gpu',
      '--no-sandbox',
      '--disable-background-timer-throttling',
      '--disable-renderer-backgrounding',
      '--disable-background-networking',
      '--disable-component-update',
      '--disable-sync',
      '--disable-features=msEdgeSync,msEdgeSignin,msEdgeFirstRunExperience',
      '--no-first-run',
      '--no-default-browser-check',
      '--remote-allow-origins=*',
      '--remote-debugging-port=0',
      `--user-data-dir=${profile}`,
      url,
    ], { cwd: repo, windowsHide: true, stdio: ['ignore', 'ignore', 'ignore'] });
    const devtoolsPort = await waitForDevtoolsPort(browser);
    const target = await waitForTarget(devtoolsPort);
    socket = await connect(target.webSocketDebuggerUrl);
    let send = commandChannel(socket);
    const wallTimeoutMs = boundedForkHangAudit ? 45000 : 12 * 60 * 1000;
    const deadline = Date.now() + wallTimeoutMs;
    let lastProgress = '';
    while (Date.now() < deadline) {
      let response;
      try {
        response = await send('Runtime.evaluate', {
          expression: "document.querySelector('#out')?.textContent || ''",
          returnByValue: true,
        });
      } catch (error) {
        if (!/DevTools command timed out/.test(String(error?.message || error))) throw error;
        try { socket.close(); } catch { /* replace the stalled read-only channel */ }
        await delay(100);
        socket = await connect(target.webSocketDebuggerUrl);
        send = commandChannel(socket);
        const targets = await (await fetch(`http://127.0.0.1:${devtoolsPort}/json/list`)).json();
        const current = targets.find((entry) => entry.id === target.id);
        console.error(`READ-ONLY CDP CHANNEL RECONNECTED: ${current?.title || 'unknown page stage'}`);
        continue;
      }
      if (response?.exceptionDetails) {
        throw new Error(`Edge audit query failed: ${response.exceptionDetails.text || response.exceptionDetails.exception?.description || 'unknown exception'}`);
      }
      const value = response?.result?.value || '';
      if (value && value !== lastProgress) {
        lastProgress = value;
        if (!value.startsWith('{')) console.error(value);
      }
      if (value.startsWith('{')) {
        const report = typeof value === 'string' ? JSON.parse(value) : value;
        if (!report || typeof report !== 'object') throw new Error(`Edge audit returned an invalid report type: ${typeof report}`);
        writeFileSync(path.join(__dirname, reportName), `${JSON.stringify(report, null, 2)}\n`, 'utf8');
        console.log(JSON.stringify(report, null, 2));
        process.exitCode = report.pass ? 0 : 1;
        return;
      }
      await delay(250);
    }
    if (boundedForkHangAudit) {
      const targets = await (await fetch(`http://127.0.0.1:${devtoolsPort}/json/list`)).json();
      const current = targets.find((entry) => entry.id === target.id);
      const stage = current?.title || null;
      const report = {
        pass: false,
        hangConfirmed: Boolean(stage?.startsWith('MIN importing fresh canonical fork ')),
        wallTimeoutMs,
        stage,
        candidateSha256,
        operation: stage === 'MIN importing fresh canonical fork solid'
          ? "fresh BiteSim.importDxfText('shovel', solid outer 80x60 mm)"
          : "fresh BiteSim.importDxfText('shovel', outer 80x60 mm plus asymmetric 20x20 mm hole)",
      };
      console.log(JSON.stringify(report, null, 2));
      process.exitCode = report.hangConfirmed ? 1 : 2;
      return;
    }
    throw new Error('Edge page did not emit its audit report within 12 minutes');
  } finally {
    try { socket?.close(); } catch { /* best-effort DevTools cleanup */ }
    await closeBrowser(browser);
    if (server.listening) await new Promise((resolve) => server.close(resolve));
    const resolved = path.resolve(profile);
    const safePrefix = path.resolve(tmpdir(), 'bite-combined-s4c-dxf-edge-');
    if (resolved.startsWith(safePrefix) && existsSync(resolved)) {
      try { rmSync(resolved, { recursive: true, force: true }); } catch { /* Edge may release profile files late. */ }
    }
  }
}

main().catch((error) => {
  console.error(error?.stack || error);
  process.exitCode = 2;
});
