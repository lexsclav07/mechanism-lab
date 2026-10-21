// Physics checks: run the solver headless and compare against textbook answers.
// Run with:  node tests/physics-checks.js   (no dependencies; exits 1 if anything fails)
global.window = {};
for (const f of ['physics', 'model', 'presets']) require(`../js/${f}.js`);
const { Phys: P, Model: M, Presets } = window.ML;

let failed = 0;
function check(name, got, want, tol) {
  const ok = Math.abs(got - want) <= tol;
  if (!ok) failed++;
  console.log(`${ok ? 'pass' : 'FAIL'}  ${name}: got ${+got.toFixed(4)}, want ${want} ± ${tol}`);
}

const g = 9.81;
const scene = (bodies, joints, extra = {}) => ({ time: 0, settings: Object.assign({ gravity: g, air: 0, substeps: 100 }, extra), bodies, joints });
const disc = (id, x, y, m, more = {}) => Object.assign({ id, kind: 'disc', r: 0.05, mass: m, x, y, a: 0, vx: 0, vy: 0, av: 0 }, more);
const ground = (x, y) => ({ body: null, x, y });
const at = (id) => ({ body: id, x: 0, y: 0 });
const run = (s, seconds, hand) => { for (let i = 0; i < Math.round(seconds * 60); i++) P.step(s, 1 / 60, hand); };

// Simple pendulum, small swing: T = 2π√(L/g)
{
  const L = 1, s = scene([disc(1, L * Math.sin(0.05), L * Math.cos(0.05), 1, { r: 0.01 })], [{ kind: 'rod', a: ground(0, 0), b: at(1), length: L }]);
  const crossings = []; let prev = s.bodies[0].x;
  for (let i = 0; i < 1200; i++) { P.step(s, 1 / 60); const x = s.bodies[0].x; if (prev * x < 0) crossings.push(s.time); prev = x; }
  const T = (crossings[crossings.length - 1] - crossings[0]) / ((crossings.length - 1) / 2);
  check('pendulum period (s)', T, +(2 * Math.PI * Math.sqrt(L / g)).toFixed(4), 0.005);
}

// Mass on a spring along a track: T = 2π√(m/k)
{
  const s = scene([{ id: 1, kind: 'box', w: 0.2, h: 0.2, mass: 1, x: 1.2, y: 0, a: 0, vx: 0, vy: 0, av: 0 }], [
    { kind: 'slider', a: ground(1, 0), dir: [1, 0], b: at(1), lock: true, angle0: 0, limits: false, lo: -1, hi: 1 },
    { kind: 'spring', a: ground(0, 0), b: at(1), k: 40, c: 0, rest: 1 }]);
  const crossings = []; let prev = s.bodies[0].x - 1;
  for (let i = 0; i < 600; i++) { P.step(s, 1 / 60); const x = s.bodies[0].x - 1; if (prev * x < 0) crossings.push(s.time); prev = x; }
  const T = (crossings[crossings.length - 1] - crossings[0]) / ((crossings.length - 1) / 2);
  check('spring-mass period (s)', T, +(2 * Math.PI * Math.sqrt(1 / 40)).toFixed(4), 0.005);
}

// Atwood machine, 2 kg vs 1 kg, rope over a 0.2 m wheel
{
  const s = scene([disc(1, -0.2, 1, 2), disc(2, 0.2, 1, 1)], [{ kind: 'pulley', a: at(1), b: at(2), p: ground(0, 0), r: 0.2, length: 0 }]);
  P.index(s); s.joints[0].length = P.pulleyGeom(s, s.joints[0]).length;
  check('pulley rope length incl. wrap (m)', s.joints[0].length, +(2 + Math.PI * 0.2).toFixed(4), 1e-4);
  run(s, 0.5);
  check('Atwood drop in 0.5 s (m)', s.bodies[0].y - 1, +(0.5 * (g / 3) * 0.25).toFixed(4), 0.002);
  check('Atwood rope tension (N)', s.joints[0]._f, +(2 * 2 * 1 * g / 3).toFixed(3), 0.01);
  check('Atwood masses hang straight (x drift, m)', Math.abs(s.bodies[0].x + 0.2), 0, 1e-4);
}

// Slack rope: free fall until taut, then holds m·g
{
  const s = scene([disc(1, 0, 0.5, 1)], [{ kind: 'rope', a: ground(0, 0), b: at(1), length: 1 }]);
  run(s, 0.2);
  check('slack rope free fall in 0.2 s (m)', s.bodies[0].y - 0.5, +(0.5 * g * 0.04).toFixed(4), 0.002);
  run(s, 3);
  check('taut rope tension (N)', s.joints[0]._f, g, 0.05);
}

// Damper: terminal speed m·g/c
{
  const s = scene([disc(1, 0, 1, 1)], [{ kind: 'damper', a: ground(0, 0), b: at(1), c: 20 }]);
  run(s, 2);
  check('damper terminal speed (m/s)', s.bodies[0].vy, +(g / 20).toFixed(4), 0.001);
  const stiff = scene([disc(1, 0, 1, 0.05)], [{ kind: 'damper', a: ground(0, 0), b: at(1), c: 100000 }]);
  run(stiff, 1);
  check('very stiff damper stays finite (speed, m/s)', Math.abs(stiff.bodies[0].vy), 0, 0.001);
}

// Piston lifting 10 kg (98.1 N) straight up at 0.3 m/s
{
  const piston = (force) => scene([disc(1, 0, -1, 10)], [
    { kind: 'piston', a: ground(0, 0), b: at(1), min: 0.5, max: 2, speed: 0.3, force, mode: 'extend', phase: 1 },
    { kind: 'slider', a: ground(0, -1), b: at(1), dir: [0, 1], lock: true, angle0: 0, limits: false, lo: -1, hi: 1 }]);
  let s = piston(500); run(s, 2);
  check('piston holds 10 kg (push, N)', s.joints[0]._f, -98.1, 0.5);
  check('piston speed after spin-up (m/s)', -s.bodies[0].vy, 0.3, 0.005);
  s = piston(50); run(s, 1);
  check('too-weak piston sinks to its stop (m)', P.jointLength(s, s.joints[0]), 0.5, 0.001);
}

// Motors hold their set speed in the presets
for (const id of ['fourbar', 'piston']) {
  const s = M.load(M.serialize(Presets.find((p) => p.id === id).build()));
  const mj = s.joints.find((j) => j.motor), b = () => P.bodyOf(s, mj.a.body != null ? mj.a : mj.b);
  run(s, 1);
  const a0 = b().a; run(s, 5);
  check(`${id} motor speed (rpm)`, (Math.abs(b().a - a0) / 5 / (2 * Math.PI)) * 60, Math.abs(mj.motor), 0.1);
}

// Negative weight floats up: y = ½·(gscale·g)·t²
{
  const s = scene([disc(1, 0, 0, 1, { gscale: -0.5 })], []); run(s, 1);
  check('weight ×−0.5 rises (m)', s.bodies[0].y, -0.5 * 0.5 * g, 0.002);
}

// The mouse hand holding 1 kg: sags m·g/k and pulls with m·g
{
  const s = scene([disc(1, 0, 0, 1)], []), hand = { body: 1, x: 0, y: 0, tx: 0.5, ty: 0, k: 300, c: 31 };
  run(s, 2, hand);
  check('hand sag (m)', s.bodies[0].y, g / 300, 0.001);
  check('hand pull (N)', hand._F, g, 0.05);
}

// Rods are parts. A real (not massless) rod swinging from one end: T = 2π√(2L/3g)
const rodScene = (rods, joints, extra) => {
  const s = scene([], [], extra);
  s.nextId = 1;
  for (const r of rods) M.addBody(s, Object.assign({ kind: 'bar', a: 0 }, r));
  for (const j of joints) M.addJoint(s, j);
  P.index(s);
  return s;
};
{
  const L = 1, th = 0.05, s = rodScene([{ x: (L / 2) * Math.sin(th), y: (L / 2) * Math.cos(th), a: Math.PI / 2 - th, L, mass: 1, ideal: false }],
    [{ kind: 'pivot', a: { body: 1, x: -L / 2, y: 0 }, b: ground(0, 0) }]);
  const crossings = []; let prev = s.bodies[0].x;
  for (let i = 0; i < 1200; i++) { P.step(s, 1 / 60); const x = s.bodies[0].x; if (prev * x < 0) crossings.push(s.time); prev = x; }
  const T = (crossings[crossings.length - 1] - crossings[0]) / ((crossings.length - 1) / 2);
  check('rod swinging from one end, period (s)', T, +(2 * Math.PI * Math.sqrt((2 * L) / (3 * g))).toFixed(4), 0.005);
}

// Old weightless rods convert to rod parts that move exactly as before
for (const pr of Presets) {
  const raw = pr.build();
  if (!raw.joints.some((j) => j.kind === 'rod')) continue;
  const old = JSON.parse(M.serialize(raw)), mig = M.load(M.serialize(raw));
  P.index(old);
  let worst = 0;
  for (let i = 0; i < 600; i++) {
    P.step(old, 1 / 60); P.step(mig, 1 / 60);
    for (const b of old.bodies) { const m = mig.bodies.find((x) => x.id === b.id); worst = Math.max(worst, Math.abs(b.x - m.x), Math.abs(b.y - m.y), Math.abs(b.a - m.a)); }
  }
  check(`"${pr.name}" converted rods move exactly as before (m)`, worst, 0, 0);
}

// Rods pinned end to end: a chain of three hanging from the ground keeps its pins together
{
  const s = rodScene([{ x: 0.5, y: 0, L: 1 }, { x: 1.5, y: 0, L: 1 }, { x: 2.5, y: 0, L: 1 }], [
    { kind: 'pivot', a: { body: 1, x: -0.5, y: 0 }, b: ground(0, 0) },
    { kind: 'pivot', a: { body: 1, x: 0.5, y: 0 }, b: { body: 2, x: -0.5, y: 0 } },
    { kind: 'pivot', a: { body: 2, x: 0.5, y: 0 }, b: { body: 3, x: -0.5, y: 0 } }]);
  let worst = 0;
  for (let i = 0; i < 600; i++) {
    P.step(s, 1 / 60);
    for (const j of s.joints) { const a = P.endWorld(s, j.a), b = P.endWorld(s, j.b); worst = Math.max(worst, Math.hypot(b[0] - a[0], b[1] - a[1])); }
  }
  check('rod chain pin gap after 10 s (m)', worst, 0, 1e-4);
  check('rod chain energy drift over 10 s (J, starts at 0)', P.energy(s).total, 0, 3);
}

// A motor on a light crank rod (0.1 kg) driving a 20 kg slider through a connecting rod
{
  const s = rodScene([{ x: 0.25, y: 0, L: 0.5, mass: 0.1 }, { x: 1.25, y: 0, L: 1.5, mass: 0.3 }], [
    { kind: 'pivot', a: { body: 1, x: -0.25, y: 0 }, b: ground(0, 0), motor: 30, torque: 200 },
    { kind: 'pivot', a: { body: 1, x: 0.25, y: 0 }, b: { body: 2, x: -0.75, y: 0 } }]);
  const blk = M.addBody(s, { kind: 'box', x: 2, y: 0, w: 0.4, h: 0.3, mass: 20 });
  M.addJoint(s, { kind: 'pivot', a: { body: 2, x: 0.75, y: 0 }, b: { body: blk.id, x: 0, y: 0 } });
  M.addJoint(s, { kind: 'slider', a: ground(2, 0), b: { body: blk.id, x: 0, y: 0 }, dir: [1, 0], limits: false, lo: -1, hi: 1 });
  P.index(s);
  run(s, 1);
  const a0 = s.bodies[0].a; run(s, 4);
  check('motor crank on a 0.1 kg rod driving 20 kg (rpm)', (Math.abs(s.bodies[0].a - a0) / 4 / (2 * Math.PI)) * 60, 30, 1.5);
}

// Deleting the part at the middle of a shared pin re-pins the others to each other
{
  const s = rodScene([{ x: -0.5, y: 0, L: 1 }, { x: 0.5, y: 0, L: 1 }, { x: 0, y: 0.5, a: Math.PI / 2, L: 1 }], []);
  const hub = M.addBody(s, { kind: 'disc', x: 0, y: 0, r: 0.05, mass: 1 });
  for (const [id, x] of [[1, 0.5], [2, -0.5], [3, -0.5]]) M.addJoint(s, { kind: 'pivot', a: { body: hub.id, x: 0, y: 0 }, b: { body: id, x, y: 0 } });
  M.remove(s, hub);
  check('re-pinned members after deleting the hub (pins)', s.joints.length, 2, 0);
  run(s, 1);
  let gap = 0;
  for (const j of s.joints) { const a = P.endWorld(s, j.a), b = P.endWorld(s, j.b); gap = Math.max(gap, Math.hypot(b[0] - a[0], b[1] - a[1])); }
  check('re-pinned rods stay together (m)', gap, 0, 1e-4);
}

// Every preset runs 10 s with joints holding and nothing blowing up
for (const pr of Presets) {
  const s = M.load(M.serialize(pr.build()));
  let worst = 0;
  for (let i = 0; i < 600; i++) {
    P.step(s, 1 / 60);
    if (i % 20) continue;
    for (const j of s.joints) {
      if (j.kind !== 'pivot' && j.kind !== 'weld' && j.kind !== 'rod') continue;
      const a = P.endWorld(s, j.a), b = P.endWorld(s, j.b), d = Math.hypot(b[0] - a[0], b[1] - a[1]);
      worst = Math.max(worst, j.kind === 'rod' ? Math.abs(d - j.length) : d);
    }
  }
  const finite = s.bodies.every((b) => Number.isFinite(b.x + b.y + b.a));
  check(`preset "${pr.name}" joint error after 10 s (m)`, finite ? worst : Infinity, 0, 1e-4);
}

console.log(failed ? `\n${failed} check(s) failed` : '\nall checks passed');
process.exit(failed ? 1 : 0);
