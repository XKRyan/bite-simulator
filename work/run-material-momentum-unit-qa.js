/*
 * Read-only source-extraction QA for the material impulse velocity contract.
 * It evaluates applyWeaponTargetImpulse exactly as present in the prototype,
 * against deterministic rigid-body mocks.  Rapier linvel is COM velocity.
 */
'use strict';

const assert = require('node:assert/strict');
const fs = require('node:fs');
const vm = require('node:vm');

const sourcePath = 'work/material-continuous-prototype-app.js';
const source = fs.readFileSync(sourcePath, 'utf8');
const marker = 'function applyWeaponTargetImpulse(physics, worldPoint, impulseOnTarget) {';
const start = source.indexOf(marker);
assert.notEqual(start, -1, '真源码中未找到 applyWeaponTargetImpulse');
let depth = 0; let end = -1;
for (let i = start; i < source.length; i += 1) {
  if (source[i] === '{') depth += 1;
  if (source[i] === '}' && --depth === 0) { end = i + 1; break; }
}
assert.notEqual(end, -1, '函数提取括号不闭合');
const extracted = source.slice(start, end);

const context = {
  state: { params: { robotMass: 2 }, metrics: { inertia: .7 } },
  point: (x, y) => ({ x, y }),
  number: Number,
  positive: (x, fallback) => Number(x) > 0 ? Number(x) : fallback,
  dot: (a, b) => a.x * b.x + a.y * b.y,
  subtract: (a, b) => ({ x: a.x - b.x, y: a.y - b.y }),
  cross: (a, b) => a.x * b.y - a.y * b.x,
  rotate: (p, angle) => ({ x: p.x * Math.cos(angle) - p.y * Math.sin(angle), y: p.x * Math.sin(angle) + p.y * Math.cos(angle) }),
};
vm.createContext(context);
vm.runInContext(`${extracted}; globalThis.apply = applyWeaponTargetImpulse;`, context);

function body({ v = { x: 0, y: 0 }, omega = 0, angle = 0, com = { x: 0, y: 0 }, mass = 1, inertia = 1, pos = { x: 0, y: 0 } } = {}) {
  return {
    v: { ...v }, omega, angle, com, mass, inertia, pos, targetImpulse: null,
    linvel() { return { ...this.v }; }, angvel() { return this.omega; }, rotation() { return this.angle; },
    localCom() { return { ...this.com }; }, mass() { return this.mass; }, principalInertia() { return this.inertia; },
    translation() { return { ...this.pos }; },
    setLinvel(value) { this.v = { ...value }; }, setAngvel(value) { this.omega = value; },
    applyImpulseAtPoint(impulse, p) { this.targetImpulse = { impulse, p }; },
  };
}
function pivotVelocity(v, omega, r) { return { x: v.x + omega * -r.y, y: v.y + omega * r.x }; }
function near(a, b, label) { assert.ok(Math.abs(a - b) < 1e-12, `${label}: ${a} != ${b}`); }

const cases = [0.31, -1.24, 2.41];
const failures = [];
let legacyViolations = 0;
for (const angle of cases) {
  const localCom = { x: .043, y: -.017 };
  const r = context.rotate(localCom, angle);
  const oldOmega = 1.7;
  const chassisV = 1.25;
  // This is a legal revolute-joint state: pivot velocity is (chassisV, 0).
  const oldComV = { x: chassisV + oldOmega * r.y, y: -oldOmega * r.x };
  // Pre-fix source wrote {nextVx, 0} as though it were a hinge velocity.
  // Calculate that old write separately: it must violate the COM contract.
  const oldWritePivot = pivotVelocity({ x: chassisV, y: 0 }, oldOmega, r);
  if (Math.abs(oldWritePivot.x - chassisV) > 1e-12 || Math.abs(oldWritePivot.y) > 1e-12) legacyViolations += 1;
  const physics = {
    targetBody: body(), robotBody: body({ v: { x: chassisV, y: 0 } }),
    weaponBody: body({ v: { x: chassisV, y: 0 }, pos: { x: 0, y: 0 } }),
    forkBody: body({ v: oldComV, omega: oldOmega, angle, com: localCom, mass: .3, inertia: .004 }),
  };
  try {
    context.apply(physics, { x: .02, y: .03 }, { x: 0, y: 0 });
    near(physics.forkBody.v.x, oldComV.x, `zero impulse COM vx angle=${angle}`);
    near(physics.forkBody.v.y, oldComV.y, `zero impulse COM vy angle=${angle}`);
    const pivot = pivotVelocity(physics.forkBody.v, physics.forkBody.omega, r);
    near(pivot.x, chassisV, `zero impulse pivot vx angle=${angle}`);
    near(pivot.y, 0, `zero impulse pivot vy angle=${angle}`);
  } catch (error) { failures.push(error.message); }
}
assert.equal(legacyViolations, cases.length, '测试构造必须让旧的 fork.setLinvel({x:nextVx,y:0}) 全部失效');

// Nonzero X impulse must preserve the fork pivot constraint immediately after
// the direct generalized-coordinate update, before Rapier steps the joint.
{
  const angle = .73; const localCom = { x: .041, y: -.019 }; const r = context.rotate(localCom, angle);
  const oldOmega = .8; const oldChassis = 1.1;
  const oldComV = { x: oldChassis + oldOmega * r.y, y: -oldOmega * r.x };
  const physics = {
    targetBody: body(), robotBody: body({ v: { x: oldChassis, y: 0 } }),
    weaponBody: body({ v: { x: oldChassis, y: 0 }, pos: { x: 0, y: 0 } }),
    forkBody: body({ v: oldComV, omega: oldOmega, angle, com: localCom, mass: .3, inertia: .004 }),
  };
  try {
    context.apply(physics, { x: .02, y: .03 }, { x: .06, y: -.01 });
    const pivot = pivotVelocity(physics.forkBody.v, physics.forkBody.omega, r);
    near(pivot.x, physics.robotBody.v.x, 'nonzero impulse pivot vx');
    near(pivot.y, 0, 'nonzero impulse pivot vy');
  } catch (error) { failures.push(error.message); }
}

if (failures.length) {
  console.error(`FAIL: ${failures.length} COM-velocity contract violation(s) from extracted source`);
  failures.forEach((message) => console.error(`- ${message}`));
  process.exitCode = 1;
} else console.log(`PASS: extracted source preserves nonzero-COM fork velocity and pivot continuity; legacy direct write violates ${legacyViolations}/${cases.length} angled cases`);
