'use strict';

// Deterministic single-IIFE bundle for the work-only private 78F parameter-
// tooth checkpoint. It deliberately domain-stops before hidden-trajectory,
// 80D, KKT, publication and removal authorization.

const crypto = require('node:crypto');
const fs = require('node:fs');
const path = require('node:path');

const root = path.resolve(__dirname, '..', '..');
const planFile = path.resolve(root, 'work/s4c-private-toi-dxf-combined/build-plan.json');
const configFile = path.resolve(root, 'work/s4c-private-toi-dxf-combined/bundle-config.json');
const outputArgument = process.argv.find((entry) => entry.startsWith('--qa-output-dir='));
const outputDirectory = outputArgument
  ? path.resolve(process.cwd(), outputArgument.slice('--qa-output-dir='.length))
  : __dirname;
fs.mkdirSync(outputDirectory, { recursive: true });
const outputFile = path.join(outputDirectory, 'app-combined-bundle.js');
const manifestFile = path.join(outputDirectory, 'bundle-manifest.json');
const EXPECTED_PLAN_DIGEST = '0250944AA850F97B4A75E46AC567B65E6EF4AABAC27FF542A53E7B087606ED30';
const EXPECTED_CONFIG_SHA256 = '34D479B9DE04BE01C6FB49004C912FA997B6EC2AEBD19C0474F375900F772622';
const BANNER = '/* GEOMETRY_AUTHORITY_LOCKED_IIFE_BUNDLE */';

const sha = (bytes) => crypto.createHash('sha256').update(bytes).digest('hex').toUpperCase();
const canonical = (value) => {
  if (value === null || typeof value !== 'object') return JSON.stringify(value);
  if (Array.isArray(value)) return `[${value.map(canonical).join(',')}]`;
  return `{${Object.keys(value).sort().map((key) => `${JSON.stringify(key)}:${canonical(value[key])}`).join(',')}}`;
};
const digest = (label, value) => sha(`${label}\n${canonical(value)}`);

const planning = JSON.parse(fs.readFileSync(planFile, 'utf8'));
if (planning.planDigest !== EXPECTED_PLAN_DIGEST
  || digest('geometry-authority-build-plan-v1', planning.buildPlan) !== EXPECTED_PLAN_DIGEST) {
  throw new Error('locked build plan digest mismatch');
}
if (sha(fs.readFileSync(configFile)) !== EXPECTED_CONFIG_SHA256) throw new Error('locked bundle config mismatch');
const config = JSON.parse(fs.readFileSync(configFile, 'utf8'));
const inputs = planning.buildPlan.inputs.map((entry) => {
  const filename = path.resolve(root, ...entry.path.split('/'));
  const bytes = fs.readFileSync(filename);
  const actualSha256 = sha(bytes);
  if (actualSha256 !== entry.sha256) throw new Error(`${entry.id} source pin mismatch: ${actualSha256}`);
  return { ...entry, filename, source: bytes.toString('utf8'), actualSha256 };
});

function replaceExact(source, before, after, label) {
  if (!source.includes(before)) throw new Error(`${label} transform anchor missing`);
  const result = source.replace(before, after);
  if (result.includes(before)) throw new Error(`${label} transform was not unique`);
  return result;
}
function transform(entry) {
  let source = entry.source.replace(/^'use strict';\s*/, '');
  if (entry.id === 'polygonClipping0157') {
    source = replaceExact(source,
      '!function(t,e){"object"==typeof exports&&"undefined"!=typeof module?module.exports=e():"function"==typeof define&&define.amd?define(e):(t="undefined"!=typeof globalThis?globalThis:t||self).polygonClipping=e()}(this,',
      'module.exports=', 'polygon private CommonJS');
    source = source.replace(/\n?\/\/# sourceMappingURL=[^\r\n]*\s*$/, '');
    if (source.includes('sourceMappingURL')) throw new Error('polygon source map disclosure transform failed');
    // The UMD tail closes the factory argument and invocation as `}));`.
    // After removing the UMD caller, invoke that factory exactly once so the
    // private CommonJS export is the polygon-clipping API, not the function.
    if (!source.endsWith('}));')) throw new Error('polygon UMD tail transform anchor missing');
    source = `${source.slice(0, -4)}}());`;
    if (!source.endsWith('}());')) throw new Error('polygon private CommonJS tail transform failed');
  }
  if (entry.id === 'rapierFreeClusterAdapter') {
    source = replaceExact(source,
      "if (typeof globalThis === 'object') globalThis.BiteS3FreeClusterRapierAdapter = api;",
      '/* private locked bundle: no adapter global export */', 'S3 adapter global');
  }
  if (entry.id === 'coulombKktV3') {
    source = source.replace(
      /const SELF_SHA256 = crypto\.createHash\('sha256'\)\s*\.update\(fs\.readFileSync\(__filename\)\)\s*\.digest\('hex'\)\.toUpperCase\(\);/,
      "const SELF_SHA256 = require('@integrity').pin('coulombKktV3');",
    );
    if (!source.includes("require('@integrity').pin('coulombKktV3')")) throw new Error('v3 self hash transform failed');
  }
  if (entry.id === 'geometryIntervalKernel') {
    source = replaceExact(source,
      'function selfSha256() { return sha256Bytes(fs.readFileSync(__filename)); }',
      "function selfSha256() { return require('@integrity').pin('geometryIntervalKernel'); }", 'geometry self hash');
    source = replaceExact(source,
      'function fileSha256(filename) { return sha256Bytes(fs.readFileSync(filename)); }',
      "function fileSha256(filename) { return require('@integrity').fileSha256(filename); }", 'geometry file hash');
  }
  if (entry.id === 'app238') {
    // Supply-chain stage: preserve the signed app body byte-for-byte inside its
    // module factory. Its own lexical IIFE is legitimate nested application
    // scope; it is not a second top-level bundle envelope.
    if (!source.includes('(() => {') || !/\}\)\(\);\s*$/.test(source)) {
      throw new Error('app238 signed IIFE anchors changed');
    }
  }
  return source;
}

const transformed = new Map(inputs.map((entry) => [entry.id, transform(entry)]));
const pins = Object.fromEntries(inputs.map((entry) => [entry.id, entry.sha256]));
const syncSha = `function __sha256(input){const bytes=typeof input==='string'?new TextEncoder().encode(input):input instanceof Uint8Array?input:new Uint8Array(input);const K=[0x428a2f98,0x71374491,0xb5c0fbcf,0xe9b5dba5,0x3956c25b,0x59f111f1,0x923f82a4,0xab1c5ed5,0xd807aa98,0x12835b01,0x243185be,0x550c7dc3,0x72be5d74,0x80deb1fe,0x9bdc06a7,0xc19bf174,0xe49b69c1,0xefbe4786,0x0fc19dc6,0x240ca1cc,0x2de92c6f,0x4a7484aa,0x5cb0a9dc,0x76f988da,0x983e5152,0xa831c66d,0xb00327c8,0xbf597fc7,0xc6e00bf3,0xd5a79147,0x06ca6351,0x14292967,0x27b70a85,0x2e1b2138,0x4d2c6dfc,0x53380d13,0x650a7354,0x766a0abb,0x81c2c92e,0x92722c85,0xa2bfe8a1,0xa81a664b,0xc24b8b70,0xc76c51a3,0xd192e819,0xd6990624,0xf40e3585,0x106aa070,0x19a4c116,0x1e376c08,0x2748774c,0x34b0bcb5,0x391c0cb3,0x4ed8aa4a,0x5b9cca4f,0x682e6ff3,0x748f82ee,0x78a5636f,0x84c87814,0x8cc70208,0x90befffa,0xa4506ceb,0xbef9a3f7,0xc67178f2];const length=bytes.length,bitHi=Math.floor(length/0x20000000),bitLo=(length<<3)>>>0,padded=new Uint8Array(((length+72)>>6)<<6);padded.set(bytes);padded[length]=128;const view=new DataView(padded.buffer);view.setUint32(padded.length-8,bitHi);view.setUint32(padded.length-4,bitLo);const H=[0x6a09e667,0xbb67ae85,0x3c6ef372,0xa54ff53a,0x510e527f,0x9b05688c,0x1f83d9ab,0x5be0cd19],w=new Uint32Array(64),rotr=(x,n)=>(x>>>n)|(x<<(32-n));for(let o=0;o<padded.length;o+=64){for(let i=0;i<16;i++)w[i]=view.getUint32(o+i*4);for(let i=16;i<64;i++){const a=w[i-15],b=w[i-2];w[i]=(w[i-16]+(rotr(a,7)^rotr(a,18)^(a>>>3))+w[i-7]+(rotr(b,17)^rotr(b,19)^(b>>>10)))>>>0;}let[a,b,c,d,e,f,g,h]=H;for(let i=0;i<64;i++){const t1=(h+(rotr(e,6)^rotr(e,11)^rotr(e,25))+((e&f)^((~e)&g))+K[i]+w[i])>>>0,t2=((rotr(a,2)^rotr(a,13)^rotr(a,22))+((a&b)^(a&c)^(b&c)))>>>0;h=g;g=f;f=e;e=(d+t1)>>>0;d=c;c=b;b=a;a=(t1+t2)>>>0;}H[0]=(H[0]+a)>>>0;H[1]=(H[1]+b)>>>0;H[2]=(H[2]+c)>>>0;H[3]=(H[3]+d)>>>0;H[4]=(H[4]+e)>>>0;H[5]=(H[5]+f)>>>0;H[6]=(H[6]+g)>>>0;H[7]=(H[7]+h)>>>0;}return H.map(v=>v.toString(16).padStart(8,'0')).join('');}`;
const factories = inputs.map((entry) => `/* MODULE_FACTORY:${entry.id} */\n${JSON.stringify(entry.id)}:function(module,exports,require,__filename,__dirname){\n${transformed.get(entry.id)}\n}`).join(',\n');
const order = planning.buildPlan.moduleLoadOrder;
const source = `${BANNER}\n/* BUILD_PLAN_SHA256:${EXPECTED_PLAN_DIGEST} */\n/* MODULE_LOAD_ORDER:${order.join(',')} */\n/* SINGLE_PRIVATE_IIFE_BEGIN */\n(()=>{'use strict';\n${syncSha}\nconst __pins=Object.freeze(${JSON.stringify(pins)});\nconst __integrity=Object.freeze({pin(id){if(!Object.hasOwn(__pins,id))throw new Error('unknown integrity pin');return __pins[id];},fileSha256(name){if(String(name).replaceAll('\\\\','/').endsWith('/outputs/bite-simulator/assets/polygon-clipping/polygon-clipping-0.15.7.umd.min.js'))return __pins.polygonClipping0157;throw new Error('runtime filesystem unavailable');}});\nconst __crypto=Object.freeze({createHash(name){if(name!=='sha256')throw new Error('sha256 only');let chunks='';return{update(value){chunks+=typeof value==='string'?value:new TextDecoder().decode(value);return this;},digest(format){if(format!=='hex')throw new Error('hex only');return __sha256(chunks);}};}});\nconst __fs=Object.freeze({readFileSync(){throw new Error('runtime filesystem unavailable');}});\nconst __path=Object.freeze({resolve(...parts){return parts.join('/').replaceAll('\\\\','/').replace(/\\/+/g,'/');}});\nconst __modules=Object.freeze({\n${factories}\n});\nconst __cache=new Map();\nconst __resolve=(from,request)=>{if(request==='node:crypto')return'@crypto';if(request==='node:fs')return'@fs';if(request==='node:path')return'@path';if(request==='@integrity')return'@integrity';if(String(request).endsWith('event-kkt-coulomb.js'))return'coulombKktV2';if(String(request).endsWith('free-cluster.js'))return'freeClusterP0';if(String(request).endsWith('event-kkt-coulomb-v3.js'))return'coulombKktV3';if(String(request).endsWith('polygon-clipping-0.15.7.umd.min.js'))return'polygonClipping0157';throw new Error('unlocked require '+from+' -> '+request);};\nconst __require=(id)=>{if(id==='@crypto')return __crypto;if(id==='@fs')return __fs;if(id==='@path')return __path;if(id==='@integrity')return __integrity;if(__cache.has(id))return __cache.get(id).exports;const factory=__modules[id];if(!factory)throw new Error('unknown private module '+id);const module={exports:{}};__cache.set(id,module);factory(module,module.exports,(request)=>__require(__resolve(id,request)),'/private/'+id+'.js','/private');return module.exports;};\n/* ENTRY_REQUIRE_ORDER:${order.join(',')} */\n${order.slice(0,-1).map((id) => `__require(${JSON.stringify(id)});`).join('\n')}\n__require('app238');\n})();\n/* SINGLE_PRIVATE_IIFE_END */\n`;
const checkpointResolveAnchor = "if(String(request).endsWith('free-cluster.js'))return'freeClusterP0';";
const checkpointResolveReplacement = "if(String(request).endsWith('free-cluster-prepared.js'))return'preparedFreeCluster';"
  + "if(String(request).endsWith('s3-free-cluster-rapier-adapter.js'))return'rapierFreeClusterAdapter';"
  + "if(String(request).endsWith('material-toi-witness.js'))return'materialToiWitness';"
  + "if(String(request).endsWith('geometry-interval-oracle.js'))return'geometryIntervalKernel';"
  + checkpointResolveAnchor;
if (!source.includes(checkpointResolveAnchor)) throw new Error('checkpoint resolver anchor missing');
const checkpointSource = source.replace(checkpointResolveAnchor, checkpointResolveReplacement);
fs.writeFileSync(outputFile, checkpointSource.replaceAll('\r\n', '\n'), 'utf8');
const bundleSha256 = sha(fs.readFileSync(outputFile));
const manifest = {
  schema: 'geometry-authority-bundle-build-manifest-v1',
  status: 'frozen-bundle',
  builder: {
    path: path.relative(root, __filename).replaceAll('\\', '/'),
    sha256: sha(fs.readFileSync(__filename)),
    runtime: { name: 'node', version: process.version },
    isolatedOutputArgument: '--qa-output-dir',
  },
  buildPlan: planning.buildPlan,
  planDigest: EXPECTED_PLAN_DIGEST,
  artifact: {
    bundlePath: path.relative(root, outputFile).replaceAll('\\', '/'),
    bundleSha256,
    candidateActualSha256: bundleSha256,
  },
};
fs.writeFileSync(manifestFile, `${JSON.stringify(manifest, null, 2)}\n`, 'utf8');
process.stdout.write(`${JSON.stringify({ bundleSha256, manifestSha256: sha(fs.readFileSync(manifestFile)), configSha256: sha(fs.readFileSync(configFile)), planDigest: EXPECTED_PLAN_DIGEST }, null, 2)}\n`);
