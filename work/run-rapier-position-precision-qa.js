const fs = require('fs');
const http = require('http');
const path = require('path');
const vm = require('vm');
const { spawn } = require('child_process');

const root = path.resolve(__dirname, '..');
const asset = path.join(root, 'outputs/bite-simulator/assets/rapier/rapier2d-compat-0.20.0.global.js');
const assetText = fs.readFileSync(asset, 'utf8');

// First prove that the checked-in global asset can be evaluated and initialized
// without importing a package or substituting an engine.
async function vmInit() {
  const context = { console, TextDecoder, TextEncoder };
  context.globalThis = context;
  vm.runInNewContext(assetText, context, { filename: asset });
  if (!context.BiteRapier?.init) throw new Error('BiteRapier.init missing after vm evaluation');
  await context.BiteRapier.init();
  if (context.BiteRapier.version() !== '0.20.0') throw new Error('unexpected Rapier version');
}

const harness = `<!doctype html><meta charset="utf-8"><script>${assetText}</script>
<pre id="out"></pre><script>(async()=>{
  const R = globalThis.BiteRapier;
  await R.init();
  const total = 0.0005;
  const stepsList = [3, 12, 48, 192, 768, 3072];
  const speeds = [0.05, 0.5, 2];
  function run(x0, vx, steps) {
    const world = new R.World({x: 0, y: 0});
    world.integrationParameters.dt = total / steps;
    const body = world.createRigidBody(R.RigidBodyDesc.dynamic()
      .setTranslation(x0, 0).setLinvel(vx, 0));
    world.createCollider(R.ColliderDesc.ball(0.001), body);
    for (let i = 0; i < steps; i++) world.step();
    const x = body.translation().x;
    const exact = x0 + vx * total;
    const errorUm = Math.abs(x - exact) * 1e6;
    world.free();
    return {x, exact, errorUm};
  }
  const rows = [];
  for (const x0 of [0.38, 0]) for (const vx of speeds) {
    const values = stepsList.map(steps => run(x0, vx, steps).errorUm);
    rows.push({x0, vx, values});
  }
  document.querySelector('#out').textContent = JSON.stringify({version:R.version(), rows});
})().catch(e=>document.querySelector('#out').textContent=JSON.stringify({error:String(e&&e.stack||e)}));</script>`;

function findEdge() {
  return [
    'C:\\Program Files (x86)\\Microsoft\\Edge\\Application\\msedge.exe',
    'C:\\Program Files\\Microsoft\\Edge\\Application\\msedge.exe',
  ].find(fs.existsSync);
}

async function browserRun() {
  const edge = findEdge();
  if (!edge) throw new Error('Microsoft Edge not found for real local-engine step test');
  const server = http.createServer((req, res) => {
    res.writeHead(200, {'Content-Type': 'text/html; charset=utf-8', 'Cache-Control': 'no-store'});
    res.end(harness);
  });
  await new Promise(resolve => server.listen(0, '127.0.0.1', resolve));
  const profile = path.join(root, 'work', `edge-rapier-precision-${process.pid}-${Date.now()}`);
  const url = `http://127.0.0.1:${server.address().port}/`;
  const child = spawn(edge, ['--headless=new', '--disable-gpu', '--no-sandbox',
    `--user-data-dir=${profile}`, '--virtual-time-budget=120000', '--dump-dom', url], {windowsHide: true});
  let out = '';
  child.stdout.on('data', c => { out += c; });
  const code = await new Promise(resolve => child.on('close', resolve));
  server.close();
  const match = out.match(/<pre id="out">([\s\S]*?)<\/pre>/);
  if (!match) throw new Error(`browser harness produced no report (exit ${code})`);
  const report = JSON.parse(match[1].replaceAll('&quot;', '"').replaceAll('&amp;', '&'));
  if (report.error) throw new Error(report.error);
  return report;
}

(async () => {
  await vmInit();
  const report = await browserRun();
  console.log(`vm-init=ok rapier=${report.version} units=um steps=3,12,48,192,768,3072`);
  for (const row of report.rows) {
    console.log(`x0=${row.x0} vx=${row.vx}: ${row.values.map(v => v.toFixed(6)).join(' ')}`);
  }
})().catch(error => { console.error(`FAIL: ${error.stack || error}`); process.exitCode = 1; });
