'use strict';

const { spawn } = require('node:child_process');
const crypto = require('node:crypto');
const fs = require('node:fs');
const http = require('node:http');
const path = require('node:path');

const root = path.resolve(__dirname, '..', '..');
const output = path.join(root, 'outputs', 'bite-simulator');
const candidate = path.join(__dirname, 'app-combined-bundle.js');
const harness = path.join(__dirname, 'checkpoint-harness.html');
const reportFile = path.join(__dirname, 's4c-browser-report.json');
const hash = (file) => crypto.createHash('sha256').update(fs.readFileSync(file)).digest('hex').toUpperCase();
const expectedCandidateSha256 = process.argv.find((entry) => entry.startsWith('--candidate-sha='))
  ?.slice('--candidate-sha='.length).toUpperCase();
if (!expectedCandidateSha256 || hash(candidate) !== expectedCandidateSha256) {
  throw new Error(`checkpoint bundle hash mismatch: ${hash(candidate)}`);
}
const edge = [
  'C:\\Program Files (x86)\\Microsoft\\Edge\\Application\\msedge.exe',
  'C:\\Program Files\\Microsoft\\Edge\\Application\\msedge.exe',
].find(fs.existsSync);
if (!edge) throw new Error('Microsoft Edge not found');
const profile = path.join(__dirname, `.edge-combined-private-toi-${process.pid}-${Date.now()}`);
const mime = {
  '.html': 'text/html; charset=utf-8', '.js': 'text/javascript; charset=utf-8',
  '.css': 'text/css; charset=utf-8', '.wasm': 'application/wasm',
  '.svg': 'image/svg+xml', '.png': 'image/png', '.json': 'application/json',
};
function resolveRequest(url) {
  const pathname = decodeURIComponent(new URL(url, 'http://127.0.0.1').pathname);
  if (pathname === '/' || pathname.endsWith('/checkpoint-harness.html')) return harness;
  const virtual = pathname.match(/^\/candidate\/(.*)$/);
  if (virtual) {
    const relative = virtual[1] || 'index.html';
    if (relative === 'app.js') return candidate;
    const requested = path.resolve(output, relative);
    if (!requested.startsWith(`${output}${path.sep}`)) return null;
    return requested;
  }
  return null;
}
const server = http.createServer((request, response) => {
  const file = resolveRequest(request.url);
  if (!file) return response.writeHead(403).end();
  fs.readFile(file, (error, data) => {
    if (error) return response.writeHead(404).end();
    response.writeHead(200, {
      'Content-Type': mime[path.extname(file)] || 'application/octet-stream',
      'Cache-Control': 'no-store',
    });
    response.end(data);
  });
});
const decode = (text) => text.replaceAll('&quot;', '"').replaceAll('&#39;', "'")
  .replaceAll('&amp;', '&').replaceAll('&lt;', '<').replaceAll('&gt;', '>');
let stdout = ''; let stderr = '';
server.listen(0, '127.0.0.1', () => {
  const page = `http://127.0.0.1:${server.address().port}/checkpoint-harness.html`;
  const child = spawn(edge, [
    '--headless=new', '--disable-gpu', '--no-sandbox',
    '--disable-background-timer-throttling', '--disable-renderer-backgrounding',
    `--user-data-dir=${profile}`, '--virtual-time-budget=180000', '--dump-dom', page,
  ], { windowsHide: true });
  child.stdout.on('data', (chunk) => { stdout += chunk; });
  child.stderr.on('data', (chunk) => { stderr += chunk; });
  child.on('close', (code) => {
    server.close();
    if (profile.startsWith(`${__dirname}${path.sep}`)) fs.rmSync(profile, { recursive: true, force: true });
    const match = stdout.match(/<pre id="out">([\s\S]*?)<\/pre>/);
    let report;
    try {
      report = match ? JSON.parse(decode(match[1])) : {
        schema: 'real-sfree-private-toi-parameter-checkpoint-browser-v1',
        pass: false, error: 'browser evidence element missing', edgeExitCode: code,
        stderr: stderr.slice(-2000),
      };
    } catch (error) {
      report = { schema: 'real-sfree-private-toi-parameter-checkpoint-browser-v1', pass: false, error: String(error) };
    }
    report.bundleSha256 = expectedCandidateSha256;
    fs.writeFileSync(reportFile, `${JSON.stringify(report, null, 2)}\n`, 'utf8');
    console.log(JSON.stringify({
      pass: report.pass, checks: report.checks || null, error: report.error || null,
      report: reportFile, reportSha256: hash(reportFile),
    }, null, 2));
    if (!report.pass) process.exitCode = 1;
  });
});
