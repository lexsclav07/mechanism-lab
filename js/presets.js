// presets.js: example scenes, built with the same functions the editor uses.
// Coordinates are metres with y pointing DOWN, angles in radians (D converts from degrees).
(function () {
  const ML = (window.ML = window.ML || {});
  const M = ML.Model, P = ML.Phys;
  const D = Math.PI / 180;

  const ground = (x, y) => ({ body: null, x, y });
  function on(scene, b, wx, wy) { // joint end on body b at a world point
    P.index(scene);
    const [x, y] = P.toLocal(b, wx, wy);
    return { body: b.id, x, y };
  }

  // A thin box from p1 to p2, for linkages that have mass.
  function bar(scene, p1, p2, t, mass, label = '', color = 'grey') {
    const dx = p2[0] - p1[0], dy = p2[1] - p1[1];
    return M.addBody(scene, { kind: 'box', x: (p1[0] + p2[0]) / 2, y: (p1[1] + p2[1]) / 2, a: Math.atan2(dy, dx),
      w: Math.hypot(dx, dy) + t, h: t, mass, label, color });
  }

  // Where two circles cross (used to assemble a four-bar linkage). upper = smaller y.
  function circleCross(c1, r1, c2, r2, upper = true) {
    const dx = c2[0] - c1[0], dy = c2[1] - c1[1], d = Math.hypot(dx, dy);
    const a = (r1 * r1 - r2 * r2 + d * d) / (2 * d), hh = Math.sqrt(Math.max(0, r1 * r1 - a * a));
    const bx = c1[0] + (a * dx) / d, by = c1[1] + (a * dy) / d;
    const p = [bx - (hh * dy) / d, by + (hh * dx) / d], q = [bx + (hh * dy) / d, by - (hh * dx) / d];
    return (p[1] < q[1]) === upper ? p : q;
  }

  // The figure from the reference video: a frame M_f hanging from two ground pivots on rods L_r,
  // a block M_s that slides inside it between two springs k, and a pendulum m_p hanging off the block.
  function swingingFrame() {
    const s = M.newScene();
    const Lr = 1.1, theta = 22 * D, W = 4.4, H = 1.3, T = 0.1, anchorY = -1.9;
    const fx = Lr * Math.sin(theta), fy = anchorY + Lr * Math.cos(theta) + H / 2;
    const frame = M.addBody(s, { kind: 'box', x: fx, y: fy, w: W, h: H, hollow: true, wall: T, mass: 6, label: 'M_f' });
    for (const side of [-1, 1]) {
      const corner = [fx + (side * W) / 2, fy - H / 2];
      M.addJoint(s, { kind: 'rod', a: ground((side * W) / 2, anchorY), b: on(s, frame, ...corner), length: Lr, label: 'L_r' });
    }
    const bs = 0.7, x0 = 0.45; // block size, and how far it starts off-centre
    const block = M.addBody(s, { kind: 'box', x: fx + x0, y: fy, w: bs, h: bs, mass: 1.5, label: 'M_s' });
    const inner = W / 2 - T, rest = inner - bs / 2;
    M.addJoint(s, { kind: 'slider', a: { body: frame.id, x: 0, y: 0 }, b: { body: block.id, x: 0, y: 0 }, dir: [1, 0], lo: -rest, hi: rest });
    M.addJoint(s, { kind: 'spring', a: { body: frame.id, x: -inner, y: 0 }, b: { body: block.id, x: -bs / 2, y: 0 }, k: 40, c: 0, rest, label: 'k' });
    M.addJoint(s, { kind: 'spring', a: { body: block.id, x: bs / 2, y: 0 }, b: { body: frame.id, x: inner, y: 0 }, k: 40, c: 0, rest, label: 'k' });
    const lp = 1.5, phi = 35 * D;
    const bob = M.addBody(s, { kind: 'disc', x: block.x + lp * Math.sin(phi), y: block.y + lp * Math.cos(phi), r: 0.14, mass: 1.2, color: 'ink', label: 'm_p' });
    M.addJoint(s, { kind: 'rod', a: { body: block.id, x: 0, y: 0 }, b: { body: bob.id, x: 0, y: 0 }, length: lp, label: 'l_p' });
    return s;
  }

  function doublePendulum() {
    const s = M.newScene();
    s.settings.air = 0; s.settings.substeps = 200; // chaotic, so worth the extra accuracy
    const o = [0, -1.2], L = 1, t1 = 125 * D, t2 = 165 * D;
    const p1 = [o[0] + L * Math.sin(t1), o[1] + L * Math.cos(t1)], p2 = [p1[0] + L * Math.sin(t2), p1[1] + L * Math.cos(t2)];
    const m1 = M.addBody(s, { kind: 'disc', x: p1[0], y: p1[1], r: 0.12, mass: 1, color: 'ink', label: 'm_1' });
    const m2 = M.addBody(s, { kind: 'disc', x: p2[0], y: p2[1], r: 0.12, mass: 1, color: 'ink', label: 'm_2', trace: true });
    M.addJoint(s, { kind: 'rod', a: ground(...o), b: { body: m1.id, x: 0, y: 0 }, length: L, label: 'L_1' });
    M.addJoint(s, { kind: 'rod', a: { body: m1.id, x: 0, y: 0 }, b: { body: m2.id, x: 0, y: 0 }, length: L, label: 'L_2' });
    return s;
  }

  // Crank-rocker four-bar: a motor turns the short crank, the rocker swings back and forth,
  // and the triangular coupler's centre traces a "coupler curve".
  function fourBar() {
    const s = M.newScene();
    const O2 = [-1, 0.6], O4 = [1, 0.6], r2 = 0.5, r3 = 2.0, r4 = 1.5;
    const A = [O2[0], O2[1] - r2], B = circleCross(A, r3, O4, r4, true);
    const mid = [(A[0] + B[0]) / 2, (A[1] + B[1]) / 2], ux = (B[0] - A[0]) / r3, uy = (B[1] - A[1]) / r3;
    const C = [mid[0] + uy * 0.9, mid[1] - ux * 0.9];
    const crank = bar(s, O2, A, 0.12, 0.5, 'r_2', 'dark');
    const rocker = bar(s, O4, B, 0.12, 1, 'r_4', 'grey');
    const cent = [(A[0] + B[0] + C[0]) / 3, (A[1] + B[1] + C[1]) / 3];
    const coupler = M.addBody(s, { kind: 'poly', x: cent[0], y: cent[1], mass: 1, label: 'r_3', trace: true, color: 'light',
      pts: [A, B, C].map(([x, y]) => [x - cent[0], y - cent[1]]) });
    M.addJoint(s, { kind: 'pivot', a: on(s, crank, ...O2), b: ground(...O2), motor: 30 });
    M.addJoint(s, { kind: 'pivot', a: on(s, crank, ...A), b: on(s, coupler, ...A) });
    M.addJoint(s, { kind: 'pivot', a: on(s, coupler, ...B), b: on(s, rocker, ...B) });
    M.addJoint(s, { kind: 'pivot', a: on(s, rocker, ...O4), b: ground(...O4) });
    return s;
  }

  // Two masses, three springs, between two walls. Start one mass off-centre and watch the
  // energy beat back and forth between them (normal modes).
  function coupledOscillators() {
    const s = M.newScene();
    s.settings.air = 0;
    const floor = M.addBody(s, { kind: 'box', x: 0, y: 0.45, w: 5.6, h: 0.2, mass: 10, fixed: true });
    const wl = M.addBody(s, { kind: 'box', x: -2.65, y: -0.15, w: 0.3, h: 1.0, mass: 10, fixed: true });
    const wr = M.addBody(s, { kind: 'box', x: 2.65, y: -0.15, w: 0.3, h: 1.0, mass: 10, fixed: true });
    const bs = 0.6, eq = [-0.9, 0.9], start = [-0.9 + 0.4, 0.9];
    const blocks = eq.map((x, i) => M.addBody(s, { kind: 'box', x: start[i], y: 0.05, w: bs, h: bs, mass: 1, label: `m_${i + 1}`, color: i ? 'light' : 'grey' }));
    blocks.forEach((b, i) => M.addJoint(s, { kind: 'slider', a: on(s, floor, eq[i], 0.05), b: { body: b.id, x: 0, y: 0 }, dir: [1, 0], lo: -0.8, hi: 0.8 }));
    const wallFace = 2.5, r1 = wallFace + eq[0] - bs / 2, rc = eq[1] - eq[0] - bs;
    M.addJoint(s, { kind: 'spring', a: on(s, wl, -wallFace, 0.05), b: { body: blocks[0].id, x: -bs / 2, y: 0 }, k: 40, c: 0, rest: r1, label: 'k' });
    M.addJoint(s, { kind: 'spring', a: { body: blocks[0].id, x: bs / 2, y: 0 }, b: { body: blocks[1].id, x: -bs / 2, y: 0 }, k: 15, c: 0, rest: rc, coils: 8, label: 'k_c' });
    M.addJoint(s, { kind: 'spring', a: { body: blocks[1].id, x: bs / 2, y: 0 }, b: on(s, wr, wallFace, 0.05), k: 40, c: 0, rest: r1, label: 'k' });
    return s;
  }

  // Slider-crank: the motor-driven wheel pushes a piston through a connecting rod.
  function crankPiston() {
    const s = M.newScene();
    const O = [-1.4, 0.2], rc = 0.45, Lrod = 1.7, y = O[1];
    const wheel = M.addBody(s, { kind: 'disc', x: O[0], y: O[1], r: 0.6, mass: 3, label: 'M_c', color: 'light' });
    M.addJoint(s, { kind: 'pivot', a: { body: wheel.id, x: 0, y: 0 }, b: ground(...O), motor: 45 });
    const C = [O[0] + rc, y], Pp = [C[0] + Lrod, y];
    const rod = bar(s, C, Pp, 0.12, 0.6, 'L', 'grey');
    const cylX = Pp[0] + 0.35;
    const top = M.addBody(s, { kind: 'box', x: cylX, y: y - 0.36, w: 1.9, h: 0.12, mass: 10, fixed: true });
    M.addBody(s, { kind: 'box', x: cylX, y: y + 0.36, w: 1.9, h: 0.12, mass: 10, fixed: true });
    const piston = M.addBody(s, { kind: 'box', x: Pp[0], y, w: 0.7, h: 0.58, mass: 1, label: 'M_p', color: 'dark' });
    M.addJoint(s, { kind: 'pivot', a: on(s, wheel, ...C), b: on(s, rod, ...C) });
    M.addJoint(s, { kind: 'pivot', a: on(s, rod, ...Pp), b: on(s, piston, ...Pp) });
    M.addJoint(s, { kind: 'slider', a: on(s, top, Pp[0], y), b: { body: piston.id, x: 0, y: 0 }, dir: [1, 0], limits: false, lo: -0.6, hi: 0.6 });
    return s;
  }

  // A weight on a spring that can also swing: stretching and swinging trade energy chaotically.
  function springPendulum() {
    const s = M.newScene();
    s.settings.air = 0; s.settings.substeps = 200; // chaotic, so worth the extra accuracy
    const o = [0, -1.5], bob = M.addBody(s, { kind: 'disc', x: 0.8, y: -0.3, r: 0.13, mass: 1, color: 'ink', label: 'm', trace: true });
    M.addJoint(s, { kind: 'spring', a: ground(...o), b: { body: bob.id, x: 0, y: 0 }, k: 30, c: 0, rest: 1.0, coils: 14, label: 'k' });
    return s;
  }

  // Atwood machine: two masses on one rope over a fixed pulley. The heavier one wins:
  // a = (m1 - m2) g / (m1 + m2), and the rope tension is 2 m1 m2 g / (m1 + m2).
  function atwood() {
    const s = M.newScene();
    s.settings.air = 0;
    const r = 0.25, top = -1.6;
    const m1 = M.addBody(s, { kind: 'box', x: -r, y: 0.3, w: 0.4, h: 0.4, mass: 2, label: 'm_1' });
    const m2 = M.addBody(s, { kind: 'box', x: r, y: -0.2, w: 0.4, h: 0.3, mass: 1.5, label: 'm_2', color: 'light' });
    const j = M.addJoint(s, { kind: 'pulley', a: { body: m1.id, x: 0, y: -0.2 }, b: { body: m2.id, x: 0, y: -0.15 }, p: ground(0, top), r, length: 0 });
    P.index(s); j.length = P.pulleyGeom(s, j).length;
    return s;
  }

  // Quarter-car suspension: the road (a piston) shakes the wheel; a spring and a damper between
  // wheel and car body soak up the bumps. Try the damper at 0 to see why cars have them.
  function suspension() {
    const s = M.newScene();
    s.settings.air = 0;
    const wheel = M.addBody(s, { kind: 'disc', x: 0, y: 0.9, r: 0.3, mass: 30, label: 'm_w', color: 'dark' });
    const car = M.addBody(s, { kind: 'box', x: 0, y: -0.35, w: 1.4, h: 0.45, mass: 250, label: 'M_b' });
    for (const b of [wheel, car]) M.addJoint(s, { kind: 'slider', a: ground(b.x, b.y), b: { body: b.id, x: 0, y: 0 }, dir: [0, 1], limits: false, lo: -0.4, hi: 0.4 });
    M.addJoint(s, { kind: 'piston', a: ground(0, 1.75), b: { body: wheel.id, x: 0, y: 0.3 }, min: 0.45, max: 0.65, len0: 0.55, speed: 0.35, force: 50000, mode: 'cycle', phase: 1 });
    const k = 25000, sag = (250 * 9.81) / k; // start at rest: pre-squash the spring by the car's weight
    const springTop = [-0.35, -0.125], springBot = [-0.35, 0.9];
    M.addJoint(s, { kind: 'spring', a: { body: car.id, x: -0.35, y: 0.225 }, b: { body: wheel.id, x: -0.35, y: 0 }, k, c: 0, rest: springBot[1] - springTop[1] + sag, coils: 8, label: 'k' });
    M.addJoint(s, { kind: 'damper', a: { body: car.id, x: 0.35, y: 0.225 }, b: { body: wheel.id, x: 0.35, y: 0 }, c: 1500, len0: 1.025, label: 'c' });
    return s;
  }

  ML.Presets = [
    { id: 'frame', name: 'The swinging frame', blurb: 'The mechanism from your video.', build: swingingFrame },
    { id: 'atwood', name: 'Atwood machine', blurb: 'Two masses on a pulley. The heavier one wins, slowly.', build: atwood },
    { id: 'suspension', name: 'Car suspension', blurb: 'A bumpy road, a spring and a damper. Set the damper to 0 to feel the difference.', build: suspension },
    { id: 'double', name: 'Double pendulum', blurb: 'Two rods, total chaos. Never swings the same way twice.', build: doublePendulum },
    { id: 'fourbar', name: 'Four-bar linkage', blurb: 'A motor turns the crank and the coupler draws a loop.', build: fourBar },
    { id: 'coupled', name: 'Coupled oscillators', blurb: 'Two blocks passing their energy back and forth through springs.', build: coupledOscillators },
    { id: 'piston', name: 'Crank and slider', blurb: 'A motor-driven wheel pushing a slider, like an engine in reverse.', build: crankPiston },
    { id: 'springpend', name: 'Spring pendulum', blurb: 'A weight on a spring that swings and bounces at once.', build: springPendulum },
    { id: 'empty', name: 'Blank sheet', blurb: 'Start from nothing.', build: () => M.newScene() },
  ];
})();
