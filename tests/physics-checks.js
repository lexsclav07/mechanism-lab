// Physics checks: run the solver headless and compare against textbook answers.
// Run with:  node tests/physics-checks.js   (no dependencies; exits 1 if anything fails)
global.window = {};
for (const f of ['physics', 'model', 'presets', 'forces', 'controls']) require(`../js/${f}.js`);
const { Phys: P, Model: M, Presets, Forces: Fo, Controls: K } = window.ML;

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
const run = (s, seconds, hand) => { s._wantEnds = true; for (let i = 0; i < Math.round(seconds * 60); i++) P.step(s, 1 / 60, hand); };

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


// ---------- statics: reactions, member forces and internal forces against textbook answers ----------
const bar = (id, x1, y1, x2, y2, more = {}) => Object.assign({ id, kind: 'bar', x: (x1 + x2) / 2, y: (y1 + y2) / 2, a: Math.atan2(y2 - y1, x2 - x1),
  L: Math.hypot(x2 - x1, y2 - y1), mass: 0.2 * Math.hypot(x2 - x1, y2 - y1), ideal: false, vx: 0, vy: 0, av: 0, gscale: 1 }, more);
const pinG = (body, x, y, bx) => ({ kind: 'pivot', a: { body, x: bx, y: 0 }, b: ground(x, y), motor: 0 });
const load = (body, x, F, angle = -90) => ({ kind: 'load', a: { body, x, y: 0 }, b: { body, x, y: 0 }, F, angle, follow: false });
const reactions = (s, b) => Fo.onBody(s, b).filter((it) => it.kind === 'reaction' && it.ground);

// Simply supported beam, 4 m, pin at A and roller at B, 1000 N down 1 m from A:
// R_A = 750 N, R_B = 250 N, biggest bending moment 750 N·m under the load.
{
  const s = scene([bar(1, 0, 0, 4, 0)], [pinG(1, 0, 0, -2), { kind: 'slider', a: ground(4, 0), b: { body: 1, x: 2, y: 0 }, dir: [1, 0], lock: false, angle0: 0, limits: false, lo: -1, hi: 1 }, load(1, -1, 1000)], { gravity: 0 });
  run(s, 1);
  const R = reactions(s, s.bodies[0]).map((r) => Fo.tb(r.f));
  check('beam: reaction at A (N, up)', R[0][1], 750, 2);
  check('beam: reaction at B (N, up)', R[1][1], 250, 2);
  const sec = Fo.section(s, s.bodies[0], Fo.onBody(s, s.bodies[0]));
  check('beam: biggest bending moment (N·m)', Math.max(...sec.rows.map((r) => r.M)), 750, 3);
  check('beam: shear just right of A (N)', sec.at(-1.9).V, 750, 2);
  check('beam: shear right of the load (N)', sec.at(0).V, -250, 2);
}

// Simply supported beam under its own weight: M_max = wL²/8 = m g L / 8 at mid-span.
{
  const s = scene([bar(1, 0, 0, 4, 0, { mass: 10 })], [pinG(1, 0, 0, -2), { kind: 'slider', a: ground(4, 0), b: { body: 1, x: 2, y: 0 }, dir: [1, 0], lock: false, angle0: 0, limits: false, lo: -1, hi: 1 }]);
  run(s, 1);
  const sec = Fo.section(s, s.bodies[0], Fo.onBody(s, s.bodies[0]));
  check('beam under its own weight: M at mid-span (N·m)', sec.at(0).M, +(10 * g * 4 / 8).toFixed(3), 0.3);
}

// Cantilever, 2 m, glued to a wall, 500 N down at the tip: the wall gives 500 N up and a
// 1000 N·m anticlockwise moment; the beam's root carries M = -1000 N·m (hogging).
{
  const s = scene([bar(1, 0, 0, 2, 0)], [{ kind: 'weld', a: { body: 1, x: -1, y: 0 }, b: ground(0, 0), angle0: 0 }, load(1, 1, 500)], { gravity: 0 });
  run(s, 1);
  const [R] = reactions(s, s.bodies[0]);
  check('cantilever: wall force (N, up)', Fo.tb(R.f)[1], 500, 1);
  check('cantilever: wall moment (N·m, anticlockwise)', R.mz, 1000, 2);
  const sec = Fo.section(s, s.bodies[0], Fo.onBody(s, s.bodies[0]));
  check('cantilever: moment at the root (N·m)', sec.at(-0.999).M, -1000, 3);
}

// Equilateral truss, 2 m sides, pinned at A, roller at B, 1000 N down at the top joint C.
// Method of joints: the sloping members carry 1000 / (2 sin 60°) = 577.4 N compression and
// the bottom chord 1000 / (2 tan 60°) = 288.7 N tension.
{
  const h = Math.sqrt(3), A = [0, 0], B = [2, 0], C = [1, -h];
  const AB = bar(1, ...A, ...B), AC = bar(2, ...A, ...C), BC = bar(3, ...B, ...C);
  const pin = (b1, x1, b2, x2) => ({ kind: 'pivot', a: { body: b1, x: x1, y: 0 }, b: { body: b2, x: x2, y: 0 }, motor: 0 });
  const s = scene([AB, AC, BC], [
    pinG(1, 0, 0, -1), pin(1, -1, 2, -1), pin(1, 1, 3, -1), pin(2, 1, 3, 1),
    { kind: 'slider', a: ground(2, 0), b: { body: 1, x: 1, y: 0 }, dir: [1, 0], lock: false, angle0: 0, limits: false, lo: -1, hi: 1 },
    load(2, 1, 1000)], { gravity: 0 });
  run(s, 1);
  const N = (b) => Fo.section(s, b, Fo.onBody(s, b)).at(0).N;
  check('truss: bottom chord AB (N, + tension)', N(s.bodies[0]), 288.7, 2);
  check('truss: member AC (N, - compression)', N(s.bodies[1]), -577.4, 3);
  check('truss: member BC (N, - compression)', N(s.bodies[2]), -577.4, 3);
}

// Paused, the read-outs come from one frame run on a copy: a freshly built beam (never run)
// must already show its static reactions.
{
  const pr = Presets.find((p) => p.id === 'beam');
  const z = Fo.snapshot(pr.build()), b = z.bodies.find((x) => x.kind === 'bar');
  const R = Fo.onBody(z, b).filter((it) => it.kind === 'reaction').map((r) => Fo.tb(r.f)[1]);
  check('paused beam example: reaction at A (N)', R[0], 700, 2);
  check('paused beam example: reaction at B (N)', R[1], 300, 2);
}

// Keyboard control: holding a key drives, letting go holds still (no sag, no jerks).
{
  const key = (code) => ({ code, repeat: false, preventDefault() {} });
  const drive = (s, seconds, onFrame) => { for (let i = 0; i < seconds * 60; i++) { K.update(s, 1 / 60); P.step(s, 1 / 60); if (onFrame) onFrame(); } };
  const load = (id) => { const s = M.load(M.serialize(Presets.find((p) => p.id === id).build())); s._wantEnds = false; return s; };

  const ex = load('drive-excavator'), boom = ex.joints.find((j) => j.label === 'boom'), len = () => P.jointLength(ex, boom);
  drive(ex, 0.5); const l0 = len();
  K.down(key('KeyW'), ex); drive(ex, 1); K.up(key('KeyW')); const l1 = len();
  drive(ex, 0.5); const l2 = len(); drive(ex, 2);
  check('excavator: holding W extends the boom (m)', l1 - l0 > 0.05 ? 1 : 0, 1, 0);
  check('excavator: let go, the boom holds (sag in 2 s, mm)', (l2 - len()) * 1000, 0, 0.5);

  const br = load('drive-bridge'), deck = br.bodies.find((b) => b.label === 'M');
  let spin = 0;
  K.down(key('KeyW'), br); drive(br, 4, () => { spin = Math.max(spin, Math.abs(deck.av)); }); K.up(key('KeyW'));
  check('drawbridge: the winch raises it smoothly (fastest turn, rad/s)', spin, 0.15, 0.15);
  check('drawbridge: raised after 4 s of W (degrees)', -deck.a * 180 / Math.PI > 20 ? 1 : 0, 1, 0);

  const arm = load('drive-arm'), up = arm.bodies.find((b) => b.label === 'L_1');
  drive(arm, 0.5); const a0 = up.a; drive(arm, 2);
  check('robot arm: brakes hold it still with no key (degrees moved in 2 s)', Math.abs(up.a - a0) * 180 / Math.PI, 0, 0.5);
  K.releaseAll();
}

// Every part of every example obeys ΣF = m·a and ΣM_G = I·α, using the frame-averaged forces.
for (const pr of Presets) {
  const s = pr.build();
  run(s, 3);
  let worstF = 0, worstM = 0;
  for (const b of s.bodies) {
    if (b.fixed || b._red) continue;
    const bal = Fo.balance(s, b, Fo.onBody(s, b));
    const scaleF = Math.max(1, b.mass * g, ...Fo.onBody(s, b).map((it) => Math.hypot(...it.f)));
    worstF = Math.max(worstF, Math.hypot(bal.F[0] - bal.ma[0], bal.F[1] - bal.ma[1]) / scaleF);
    worstM = Math.max(worstM, Math.abs(bal.M - bal.Ia) / Math.max(1, scaleF * 0.5));
  }
  check(`preset "${pr.name}": ΣF = m·a, worst error (share of the biggest force)`, worstF, 0, 0.01);
  check(`preset "${pr.name}": ΣM = I·α, worst error`, worstM, 0, 0.01);
}

console.log(failed ? `\n${failed} check(s) failed` : '\nall checks passed');
process.exit(failed ? 1 : 0);
