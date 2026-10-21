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

  // F1 front suspension, seen from the front, on a shaker rig like the ones teams test on.
  // Each corner is a double wishbone (short upper arm, long lower arm, both weightless links).
  // A pushrod runs from the bottom of the upright up to a rocker on the chassis shoulder, and
  // the rocker works an inboard spring k and damper c. The left tyre is pinned to its pad and
  // the right one rolls on its pad (pin and roller supports), so the tyres can scrub sideways as
  // the wishbones swing. The posts shake left and right at different rates: heave and roll.
  const K_F1 = 20000, C_F1 = 500; // spring N/m and damper N·s/m at the units (the rocker multiplies them)
  function f1Suspension() {
    const s = M.newScene();
    s.settings.air = 0; s.settings.substeps = 200; // kilonewton loads through light links: worth the accuracy
    // monocoque cross-section, with a post on top for the inboard spring and damper mounts
    const mono = [[-0.26, -0.08], [0.26, -0.08], [0.22, -0.62], [0.12, -0.8], [0.04, -0.8], [0.04, -1.07], [-0.04, -1.07], [-0.04, -0.8], [-0.12, -0.8], [-0.22, -0.62]];
    const cen = P.polyCentroid(mono);
    const chassis = M.addBody(s, { kind: 'poly', x: cen[0], y: cen[1], pts: mono.map(([x, y]) => [x - cen[0], y - cen[1]]), mass: 300, label: 'M_s' });
    for (const sd of [-1, 1]) {
      const X = (x) => sd * x, left = sd < 0;
      const tyre = M.addBody(s, { kind: 'box', x: X(0.8), y: -0.36, w: 0.3, h: 0.72, mass: 30, color: 'dark', label: left ? 'm_u' : '' });
      const pad = M.addBody(s, { kind: 'box', x: X(0.8), y: 0.03, w: 0.44, h: 0.06, mass: 20, color: 'light' });
      M.addJoint(s, { kind: 'slider', a: ground(X(0.8), 0.03), b: { body: pad.id, x: 0, y: 0 }, dir: [0, -sd], limits: false, lo: -0.12, hi: 0.12 }); // guide drawn on the outside
      M.addJoint(s, { kind: 'piston', a: ground(X(0.8), 0.62), b: { body: pad.id, x: 0, y: 0.03 }, min: 0.53, max: 0.59, len0: 0.56, speed: left ? 0.25 : 0.18, force: 30000, mode: 'cycle', phase: 1 });
      if (left) M.addJoint(s, { kind: 'pivot', a: on(s, tyre, X(0.8), 0), b: on(s, pad, X(0.8), 0) });
      else M.addJoint(s, { kind: 'slider', a: on(s, pad, X(0.8), 0), b: on(s, tyre, X(0.8), 0), dir: [1, 0], lock: false, limits: true, lo: -0.15, hi: 0.15 });
      const uo = [X(0.7), -0.56], ui = [X(0.2), -0.5], lo = [X(0.7), -0.16], li = [X(0.2), -0.12];
      const link = (p1, b1, p2, b2) => M.addJoint(s, { kind: 'rod', a: on(s, b1, ...p1), b: on(s, b2, ...p2), length: Math.hypot(p2[0] - p1[0], p2[1] - p1[1]) });
      link(ui, chassis, uo, tyre); // upper wishbone
      link(li, chassis, lo, tyre); // lower wishbone
      const piv = [X(0.2), -0.7], A = [X(0.32), -0.72], B = [X(0.27), -1.02], D = [X(0.249), -0.93];
      const rc = P.polyCentroid([piv, A, B]);
      const rocker = M.addBody(s, { kind: 'poly', x: rc[0], y: rc[1], pts: [piv, A, B].map(([x, y]) => [x - rc[0], y - rc[1]]), mass: 1.5, color: 'light' });
      M.addJoint(s, { kind: 'pivot', a: on(s, rocker, ...piv), b: on(s, chassis, ...piv) });
      link([X(0.66), -0.2], tyre, A, rocker); // pushrod
      const ms = [X(0.04), -1.02], md = [X(0.04), -0.9];
      M.addJoint(s, { kind: 'spring', a: on(s, rocker, ...B), b: on(s, chassis, ...ms), k: K_F1, c: 0, rest: Math.hypot(B[0] - ms[0], B[1] - ms[1]), coils: 7, label: left ? 'k' : '' });
      M.addJoint(s, { kind: 'damper', a: on(s, rocker, ...D), b: on(s, chassis, ...md), c: C_F1, len0: Math.hypot(D[0] - md[0], D[1] - md[1]), label: left ? 'c' : '' });
    }
    return preload(s);
  }

  // Pre-load the springs so a car sits at its design ride height: hold the posts, let it settle,
  // read each spring's force and build that in as pre-compression, then put the car back at its
  // design pose. Three rounds gets it within a hundredth of a millimetre. The pistons keep the
  // holding force they built up, so the car doesn't sag when you press Play.
  function preload(s) {
    P.index(s);
    const design = s.bodies.map((b) => [b.x, b.y, b.a]);
    const springs = s.joints.filter((j) => j.kind === 'spring'), designLen = springs.map((j) => P.jointLength(s, j));
    const pistons = s.joints.filter((j) => j.kind === 'piston'), speeds = pistons.map((j) => j.speed);
    pistons.forEach((j) => { j.speed = 0; });
    const air = s.settings.air;
    s.settings.air = 3; // heavy air drag just to settle quickly
    for (let round = 0; round < 3; round++) {
      for (let i = 0; i < 240; i++) P.step(s, 1 / 60);
      springs.forEach((j, i) => { const F = j.k * (P.jointLength(s, j) - j.rest); j.rest = designLen[i] - F / j.k; });
      s.bodies.forEach((b, i) => { [b.x, b.y, b.a] = design[i]; b.vx = b.vy = b.av = 0; });
    }
    pistons.forEach((j, i) => { j.speed = speeds[i]; j.phase = 1; });
    s.settings.air = air; s.time = 0;
    return s;
  }

  // Rods are parts: they join each other with pins. rod() adds one from p1 to p2.
  const rod = (s, p1, p2, props = {}) => M.addBody(s, Object.assign({ kind: 'bar', x: (p1[0] + p2[0]) / 2, y: (p1[1] + p2[1]) / 2,
    a: Math.atan2(p2[1] - p1[1], p2[0] - p1[0]), L: Math.hypot(p2[0] - p1[0], p2[1] - p1[1]) }, props));
  const pinRods = (s, b1, b2, p) => M.addJoint(s, { kind: 'pivot', a: on(s, b1, ...p), b: b2 ? on(s, b2, ...p) : ground(...p) });

  // A four-bar linkage made only of rods, pinned end to end. A motor turns the short crank; the
  // middle of the coupler rod draws its loop.
  function rodLinkage() {
    const s = M.newScene();
    const O2 = [-0.8, 0.4], O4 = [0.8, 0.4], A = [-0.8, 0], B = circleCross(A, 1.8, O4, 1.3, true);
    const crank = rod(s, O2, A, { label: 'L_2' }), coupler = rod(s, A, B, { label: 'L_3', trace: true }), rocker = rod(s, O4, B, { label: 'L_4' });
    M.addJoint(s, { kind: 'pivot', a: on(s, crank, ...O2), b: ground(...O2), motor: 40, torque: 20 });
    pinRods(s, crank, coupler, A);
    pinRods(s, coupler, rocker, B);
    pinRods(s, rocker, null, O4);
    return s;
  }

  // A scissor lift: two X-shaped stages of rods pinned where they cross and end to end. The
  // bottom right foot rolls along the floor, the platform's right corner rolls along the platform,
  // and a piston opens the X to lift a 10 kg load.
  function scissorLift() {
    const s = M.newScene();
    s.settings.substeps = 200; // a closed loop of rods carrying 25 kg: worth the accuracy
    const w = 0.6, hgt = 0.5, arm = { mass: 2 };
    const pts = (k) => [[-w, -k * hgt], [w, -k * hgt]]; // left and right joints at level k
    const [l0, r0] = pts(0), [l1, r1] = pts(1), [l2, r2] = pts(2);
    const a1 = rod(s, l0, r1, arm), b1 = rod(s, r0, l1, arm), a2 = rod(s, l1, r2, arm), b2 = rod(s, r1, l2, arm);
    const deck = M.addBody(s, { kind: 'box', x: 0, y: -2 * hgt - 0.06, w: 1.6, h: 0.12, mass: 15, label: 'M' });
    const load = M.addBody(s, { kind: 'box', x: 0.25, y: -2 * hgt - 0.27, w: 0.35, h: 0.3, mass: 10, label: 'm', color: 'dark' });
    M.addJoint(s, { kind: 'weld', a: on(s, deck, 0.25, -2 * hgt - 0.12), b: on(s, load, 0.25, -2 * hgt - 0.12), angle0: 0 });
    pinRods(s, a1, null, l0);                                        // fixed foot
    // rolling foot; its stops keep the arms between about 10° (folded) and 57° (fully up)
    M.addJoint(s, { kind: 'slider', a: ground(...r0), b: on(s, b1, ...r0), dir: [1, 0], lock: false, limits: true, lo: -0.5, hi: 0.08 });
    pinRods(s, a1, b1, [0, -hgt / 2]);                               // first X
    pinRods(s, a1, b2, r1); pinRods(s, b1, a2, l1);                  // stages joined end to end
    pinRods(s, a2, b2, [0, -1.5 * hgt]);                             // second X
    pinRods(s, b2, deck, l2);                                        // platform: fixed corner
    M.addJoint(s, { kind: 'slider', a: on(s, deck, ...r2), b: on(s, a2, ...r2), dir: [1, 0], lock: false, limits: true, lo: -0.5, hi: 0.08 }); // rolling corner
    // the piston's stroke covers arm angles of about 16° to 52°. A scissor needs its biggest push
    // when it's nearly flat, so the cylinder is rated well above the load's 250 N weight.
    M.addJoint(s, { kind: 'piston', a: ground(-0.2, 0), b: on(s, a1, 0.3, -0.375), min: 0.61, max: 0.8, len0: 0.625, speed: 0.08, force: 12000, mode: 'cycle', phase: 1 });
    return preload(s); // starts with the piston already holding the load
  }

  ML.Presets = [
    { id: 'frame', name: 'The swinging frame', blurb: 'The mechanism from your video.', build: swingingFrame },
    { id: 'atwood', name: 'Atwood machine', blurb: 'Two masses on a pulley. The heavier one wins, slowly.', build: atwood },
    { id: 'suspension', name: 'Car suspension', blurb: 'A bumpy road, a spring and a damper. Set the damper to 0 to feel the difference.', build: suspension },
    { id: 'f1', name: 'F1 front suspension', blurb: 'Double wishbones, pushrods and rockers on a shaker rig, like the ones F1 teams test on.', build: f1Suspension },
    { id: 'double', name: 'Double pendulum', blurb: 'Two rods, total chaos. Never swings the same way twice.', build: doublePendulum },
    { id: 'fourbar', name: 'Four-bar linkage', blurb: 'A motor turns the crank and the coupler draws a loop.', build: fourBar },
    { id: 'rodlinkage', name: 'Rod linkage', blurb: 'Three rods pinned end to end, driven by a motor crank.', build: rodLinkage },
    { id: 'scissor', name: 'Scissor lift', blurb: 'Rods crossed in X shapes and pinned, opened by a piston.', build: scissorLift },
    { id: 'coupled', name: 'Coupled oscillators', blurb: 'Two blocks passing their energy back and forth through springs.', build: coupledOscillators },
    { id: 'piston', name: 'Crank and slider', blurb: 'A motor-driven wheel pushing a slider, like an engine in reverse.', build: crankPiston },
    { id: 'springpend', name: 'Spring pendulum', blurb: 'A weight on a spring that swings and bounces at once.', build: springPendulum },
    { id: 'empty', name: 'Blank sheet', blurb: 'Start from nothing.', build: () => M.newScene() },
  ];
})();
