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

  // ---------- statics ----------
  // Textbook problems: everything holds still, so the forces are the static answers. Members are
  // weightless (as the problems say), and the loads are the given P values. Weightless, but with
  // some mass: a 0.4 kg rod carrying kilonewtons makes the solver's in-between values chatter.
  const member = (s, p1, p2, label = '', mass = 5) => rod(s, p1, p2, { ideal: false, gscale: 0, mass, label });
  const P_ = (s, b, p, F, angle = -90, label) => {
    const e = on(s, b, ...p);
    return M.addJoint(s, { kind: 'load', a: e, b: { ...e }, F, angle, label: label || M.autoLabel(s, 'load') });
  };
  // A roller: the part rides on a level track on the ground and is free to tilt.
  const roller = (s, b, p) => M.addJoint(s, { kind: 'slider', a: ground(...p), b: on(s, b, ...p), dir: [1, 0], lock: false, limits: false, lo: -0.4, hi: 0.4 });
  // Several rods meeting at one pin (the first is the hub), optionally pinned to the ground too.
  function joint(s, rods, p, grounded = false) {
    rods.slice(1).forEach((r) => pinRods(s, rods[0], r, p));
    if (grounded) pinRods(s, rods[0], null, p);
  }

  // Simply supported beam: pinned at A, roller at B, a 1000 N load 1.5 m from A. By moments
  // about A: R_B = 1000 × 1.5 / 5 = 300 N, so R_A = 700 N. The biggest bending moment is
  // under the load: 700 × 1.5 = 1050 N·m.
  function beam() {
    const s = M.newScene();
    const b = member(s, [-2.5, 0], [2.5, 0], 'L');
    pinRods(s, b, null, [-2.5, 0]);
    roller(s, b, [2.5, 0]);
    P_(s, b, [-1, 0], 1000);
    return s;
  }

  // Wall bracket: a 2 m beam pinned to the wall at A, held level by a cable to a point 1.5 m
  // above A, with 600 N hanging off the end. Moments about A: T sin θ × 2 = 600 × 2, with
  // sin θ = 0.6, so T = 1000 N; then A pushes out with 800 N and carries no vertical load.
  function bracket() {
    const s = M.newScene();
    M.addBody(s, { kind: 'box', x: -0.15, y: -0.6, w: 0.3, h: 2.6, mass: 10, fixed: true }); // the wall
    const b = member(s, [0, 0], [2, 0], 'L');
    pinRods(s, b, null, [0, 0]);
    M.addJoint(s, { kind: 'rope', a: ground(0, -1.5), b: on(s, b, 2, 0), length: 2.5 });
    P_(s, b, [2, 0], 600);
    return s;
  }

  // Cantilever: a 3 m beam glued into a wall (a fixed support), 400 N down at the tip and a
  // 300 N·m anticlockwise couple at the middle. The wall gives 400 N up and an anticlockwise
  // moment of 400 × 3 − 300 = 900 N·m.
  function cantilever() {
    const s = M.newScene();
    M.addBody(s, { kind: 'box', x: -0.15, y: 0, w: 0.3, h: 1.6, mass: 10, fixed: true });
    const b = member(s, [0, 0], [3, 0], 'L');
    M.addJoint(s, { kind: 'weld', a: on(s, b, 0, 0), b: ground(0, 0), angle0: -b.a });
    P_(s, b, [3, 0], 400);
    const e = on(s, b, 1.5, 0);
    M.addJoint(s, { kind: 'couple', a: e, b: { ...e }, M: 300, label: 'C_1' });
    return s;
  }

  // Warren truss bridge: 8 m span, 2 m bays, 1.7 m deep, pinned at the left end and on a roller
  // at the right, with 2 kN on each of the three inner deck joints. Every member is a two-force
  // member: T is tension, C compression. The middle of the bottom chord pulls hardest.
  function truss() {
    const s = M.newScene();
    const H = 1.7, bot = [0, 1, 2, 3, 4].map((i) => [-4 + 2 * i, 0]), top = [0, 1, 2, 3].map((i) => [-3 + 2 * i, -H]);
    const chordB = [0, 1, 2, 3].map((i) => member(s, bot[i], bot[i + 1]));
    const chordT = [0, 1, 2].map((i) => member(s, top[i], top[i + 1]));
    const up = [0, 1, 2, 3].map((i) => member(s, bot[i], top[i]));       // diagonals rising to the right
    const down = [0, 1, 2, 3].map((i) => member(s, top[i], bot[i + 1])); // and falling
    joint(s, [chordB[0], up[0]], bot[0], true);
    for (let i = 1; i < 4; i++) joint(s, [chordB[i - 1], chordB[i], down[i - 1], up[i]], bot[i]);
    joint(s, [chordB[3], down[3]], bot[4]);
    roller(s, chordB[3], bot[4]);
    for (let i = 0; i < 4; i++) joint(s, [up[i], down[i], ...(i > 0 ? [chordT[i - 1]] : []), ...(i < 3 ? [chordT[i]] : [])], top[i]);
    for (let i = 1; i < 4; i++) P_(s, chordB[i - 1], bot[i], 2000);
    return s;
  }

  // ---------- real machines ----------

  // A shape given by its corners in world coordinates.
  function shape(s, pts, props = {}) {
    const c = P.polyCentroid(pts);
    return M.addBody(s, Object.assign({ kind: 'poly', x: c[0], y: c[1], pts: pts.map(([x, y]) => [x - c[0], y - c[1]]) }, props));
  }
  // A thick band along a path (for bent parts like an excavator boom).
  function band(path, t) {
    const side = (sgn) => path.map((p, i) => {
      const a = path[Math.max(0, i - 1)], b = path[Math.min(path.length - 1, i + 1)];
      const [dx, dy] = [b[0] - a[0], b[1] - a[1]], d = Math.hypot(dx, dy);
      return [p[0] - (sgn * t * dy) / d, p[1] + (sgn * t * dx) / d];
    });
    return [...side(1), ...side(-1).reverse()];
  }
  const glue = (s, b1, b2, p) => M.addJoint(s, { kind: 'weld', a: on(s, b1, ...p), b: b2 ? on(s, b2, ...p) : ground(...p), angle0: (b2 ? b2.a : 0) - b1.a });
  const len = (p, q) => Math.hypot(q[0] - p[0], q[1] - p[1]);
  const pistonOn = (s, a, b, props) => { const L = len(P.endWorld(s, a), P.endWorld(s, b)); return M.addJoint(s, Object.assign({ kind: 'piston', a, b, len0: L, mode: 'cycle', phase: 1 }, props)); };
  // Where two circles cross: of the two answers, the one that scores lowest. Used to assemble
  // linkages joint by joint from their link lengths.
  function meet(c1, r1, c2, r2, score) {
    const dx = c2[0] - c1[0], dy = c2[1] - c1[1], d = Math.hypot(dx, dy);
    const a = (r1 * r1 - r2 * r2 + d * d) / (2 * d), hh = Math.sqrt(Math.max(0, r1 * r1 - a * a));
    const bx = c1[0] + (a * dx) / d, by = c1[1] + (a * dy) / d;
    const p = [bx + (hh * dy) / d, by - (hh * dx) / d], q = [bx - (hh * dy) / d, by + (hh * dx) / d];
    return score(p) <= score(q) ? p : q;
  }
  const highest = (p) => p[1], lowest = (p) => -p[1];

  // Oil pumpjack ("nodding donkey"). A motor turns the crank at 8 strokes a minute; the pitman
  // arm rocks the walking beam on its Samson post, and the horsehead lifts the polished rod and
  // the string of sucker rods hanging down the well (lumped here as one 150 kg load) on a cable
  // bridle. The counterweight sits on the crank in line with the crank pin, so it balances the
  // well load and the motor only has to supply the difference.
  function pumpjack() {
    const s = M.newScene();
    const floor = 0.55, piv = [0, -1.7], O = [-1.7, -0.25], r = 0.45;
    M.addBody(s, { kind: 'box', x: -0.3, y: floor + 0.1, w: 4.6, h: 0.2, mass: 10, fixed: true, color: 'light' }); // concrete pad
    shape(s, [[-0.55, floor], [0.55, floor], [0.07, piv[1] + 0.1], [-0.07, piv[1] + 0.1]], { mass: 10, fixed: true }); // Samson post
    M.addBody(s, { kind: 'box', x: O[0], y: (O[1] + floor) / 2 + 0.05, w: 0.7, h: floor - O[1] + 0.1, mass: 10, fixed: true, color: 'dark' }); // gearbox
    const wellhead = M.addBody(s, { kind: 'box', x: 1.62, y: floor - 0.3, w: 0.36, h: 0.6, mass: 10, fixed: true, color: 'light' }); // wellhead
    // walking beam with the horsehead on its front end
    const head = [[1.25, -0.32], [1.42, -0.28], [1.55, -0.16], [1.62, 0], [1.6, 0.2], [1.52, 0.38], [1.4, 0.5], [1.25, 0.56]];
    const beamPts = [[-1.75, -0.08], [1.25, -0.08], ...head, [1.25, 0.08], [-1.75, 0.08]].map(([x, y]) => [x + piv[0], y + piv[1]]);
    const beam = shape(s, beamPts, { mass: 180, label: 'M_b' });
    pinRods(s, beam, null, piv);
    // crank arm with its counterweight, in line with the crank pin
    const crank = M.addBody(s, { kind: 'box', x: O[0] + 0.2, y: O[1], w: 0.75, h: 0.14, mass: 20, color: 'dark' });
    const cw = M.addBody(s, { kind: 'box', x: O[0] + 0.4, y: O[1], w: 0.34, h: 0.42, mass: 260, label: 'm_c', color: 'dark' });
    glue(s, crank, cw, [O[0] + 0.4, O[1]]);
    M.addJoint(s, { kind: 'pivot', a: on(s, crank, ...O), b: ground(...O), motor: 8, torque: 1500 });
    const C = [O[0] + r, O[1]], tail = [piv[0] - 1.65, piv[1]];
    const pitman = rod(s, C, tail, { mass: 15, ideal: false, label: 'L_p' });
    pinRods(s, crank, pitman, C);
    pinRods(s, pitman, beam, tail);
    // bridle cable from the horsehead down to the polished rod, which slides in the wellhead
    const top = [piv[0] + 1.62, piv[1] + 0.75];
    const polished = M.addBody(s, { kind: 'box', x: top[0], y: top[1] + 0.45, w: 0.07, h: 0.9, mass: 150, label: 'm_w', color: 'ink' });
    M.addJoint(s, { kind: 'slider', a: on(s, wellhead, top[0], top[1] + 0.45), b: { body: polished.id, x: 0, y: 0 }, dir: [0, 1], limits: false, lo: -1, hi: 1 });
    M.addJoint(s, { kind: 'rope', a: on(s, beam, piv[0] + 1.62, piv[1]), b: { body: polished.id, x: 0, y: -0.45 }, length: 0.75 });
    return s;
  }

  // Theo Jansen's Strandbeest leg, built from his "holy numbers" (lengths in cm, drawn here
  // at 2 cm per cm). Joints follow Wang's two-circle construction (arXiv 2606.22129). One crank drives a mirrored pair of legs. The foot
  // draws a path with a long flat bottom, so a walking beast's body glides along level.
  function strandbeest() {
    const s = M.newScene();
    const k = 0.02, N = { a: 38, b: 41.5, c: 39.3, d: 40.1, e: 55.8, f: 39.4, g: 36.7, h: 65.7, i: 49, j: 50, k: 61.9, l: 7.8, m: 15 };
    const n = (key) => N[key] * k;
    const O = [0, -0.3], Mp = [O[0], O[1] - n('m')]; // crank straight up to start
    const crank = rod(s, O, Mp, { label: 'm', ideal: false });
    M.addJoint(s, { kind: 'pivot', a: on(s, crank, ...O), b: ground(...O), motor: 25, torque: 30 });
    for (const sd of [-1, 1]) {
      const Z = [O[0] + sd * n('a'), O[1] + n('l')];
      const outer = (p) => -sd * p[0];
      const P1 = meet(Mp, n('j'), Z, n('b'), highest), P2 = meet(Mp, n('k'), Z, n('c'), lowest);
      const P3 = meet(Z, n('d'), P1, n('e'), outer), P4 = meet(P3, n('f'), P2, n('g'), outer);
      const F = meet(P2, n('i'), P4, n('h'), lowest);
      const bar = (p, q, lab) => rod(s, p, q, { ideal: false, label: sd < 0 ? lab : '' });
      const j = bar(Mp, P1, 'j'), kk = bar(Mp, P2, 'k'), b = bar(Z, P1, 'b'), c = bar(Z, P2, 'c'), d = bar(Z, P3, 'd');
      const e = bar(P1, P3, 'e'), f = bar(P3, P4, 'f'), g = bar(P2, P4, 'g'), h = bar(P4, F, 'h'), i = bar(P2, F, 'i');
      const foot = M.addBody(s, { kind: 'disc', x: F[0], y: F[1], r: 0.04, mass: 0.3, color: 'ink', trace: true });
      pinRods(s, crank, j, Mp); pinRods(s, j, kk, Mp);
      pinRods(s, b, null, Z); pinRods(s, b, c, Z); pinRods(s, b, d, Z);
      pinRods(s, j, b, P1); pinRods(s, b, e, P1);
      pinRods(s, kk, c, P2); pinRods(s, c, g, P2); pinRods(s, c, i, P2);
      pinRods(s, d, e, P3); pinRods(s, e, f, P3);
      pinRods(s, f, g, P4); pinRods(s, g, h, P4);
      pinRods(s, h, i, F); pinRods(s, h, foot, F);
    }
    return s;
  }

  // Excavator arm: boom, stick and bucket, each worked by its own hydraulic cylinder. The three
  // cylinders run at different speeds, so the arm keeps finding new digging motions.
  function excavator() {
    const s = M.newScene();
    s.settings.substeps = 200; // a heavy chain of parts on three stiff actuators
    const floor = 0.6;
    M.addBody(s, { kind: 'box', x: -1.6, y: floor - 0.2, w: 2.2, h: 0.4, mass: 10, fixed: true, color: 'dark' }); // tracks
    shape(s, [[-2.6, floor - 0.4], [-0.55, floor - 0.4], [-0.55, -0.6], [-1.2, -0.6], [-1.35, -1.25], [-2.0, -1.25], [-2.0, -0.6], [-2.6, -0.6]], { mass: 10, fixed: true }); // house and cab
    const B0 = [-0.75, -0.45], knee = [0.35, -1.9], BT = [1.3, -1.55];
    const boom = shape(s, band([B0, knee, BT], 0.11), { mass: 250, label: 'M_1' });
    pinRods(s, boom, null, B0);
    const ST = [1.75, 0.0], u = [(BT[0] - ST[0]) / len(ST, BT), (BT[1] - ST[1]) / len(ST, BT)];
    const tail = [BT[0] + u[0] * 0.5, BT[1] + u[1] * 0.5];
    const stick = shape(s, band([tail, BT, ST], 0.08), { mass: 120, label: 'M_2' });
    pinRods(s, stick, boom, BT);
    const bk = [[0, 0], [0.18, -0.12], [0.42, -0.05], [0.58, 0.18], [0.6, 0.42], [0.38, 0.52], [0.36, 0.4], [0.44, 0.22], [0.3, 0.06], [0.12, 0.1]];
    const bucket = shape(s, bk.map(([x, y]) => [ST[0] + x - 0.1, ST[1] + y - 0.05]), { mass: 60, label: 'm', color: 'dark' });
    pinRods(s, bucket, stick, ST);
    const lug = [ST[0] + 0.14, ST[1] - 0.24];
    const at = (b, p) => on(s, b, ...p);
    // each stroke is a share of the cylinder's starting length, kept short of any dead centre
    const cyl = (a, b, speed, force, lo, hi) => { const j = pistonOn(s, a, b, { speed, force }); j.min = +(j.len0 * lo).toFixed(3); j.max = +(j.len0 * hi).toFixed(3); };
    cyl(ground(-0.6, 0), at(boom, [-0.17, -1.03]), 0.09, 60000, 0.82, 1.12);
    cyl(at(boom, [0.2, -2.0]), at(stick, tail), 0.12, 30000, 0.82, 1.12);
    cyl(at(stick, [1.42, -1.15]), at(bucket, lug), 0.1, 15000, 0.97, 1.28);
    return preload(s);
  }

  // Trebuchet: a 400 kg counterweight swings down and whips the long arm round; the sling
  // swings the 5 kg stone out faster still. There's no release hook, so the stone stays in the
  // sling and keeps whirling, but v_max at the top shows the speed it would leave at.
  function trebuchet() {
    const s = M.newScene();
    s.settings.air = 0; s.settings.substeps = 200;
    const floor = 0.5, ax = [0, -2.5];
    M.addBody(s, { kind: 'box', x: 0, y: floor + 0.08, w: 4.4, h: 0.16, mass: 10, fixed: true, color: 'light' });
    shape(s, [[-1.0, floor], [-0.85, floor], [0.04, ax[1]], [-0.04, ax[1]]], { mass: 10, fixed: true });
    shape(s, [[0.85, floor], [1.0, floor], [0.04, ax[1]], [-0.04, ax[1]]], { mass: 10, fixed: true });
    const th = 52 * D, Ll = 2.6, Ls = 0.7;
    const tip = [ax[0] - Ll * Math.cos(th), ax[1] + Ll * Math.sin(th)], back = [ax[0] + Ls * Math.cos(th), ax[1] - Ls * Math.sin(th)];
    const arm = bar(s, tip, back, 0.1, 30, 'L', 'grey');
    pinRods(s, arm, null, ax);
    const cw = M.addBody(s, { kind: 'box', x: back[0], y: back[1] + 0.45, w: 0.6, h: 0.6, mass: 400, label: 'M', color: 'dark' });
    M.addJoint(s, { kind: 'pivot', a: on(s, arm, ...back), b: on(s, cw, ...back) });
    const sling = floor - 0.2 - tip[1];
    const stone = M.addBody(s, { kind: 'disc', x: tip[0], y: tip[1] + sling, r: 0.12, mass: 5, label: 'm', color: 'ink', trace: true });
    M.addJoint(s, { kind: 'rope', a: on(s, arm, ...tip), b: { body: stone.id, x: 0, y: 0 }, length: sling });
    return s;
  }

  // Tuned mass damper, as in Taipei 101. Two identical towers, both knocked sideways at the
  // start. The right one has a pendulum inside, tuned to swing at nearly the tower's own rate
  // and dragged by a damper: it soaks up the sway in a few swings while the plain tower rocks on.
  function tunedMassDamper() {
    const s = M.newScene();
    s.settings.air = 0;
    const H = 3, W = 1.0, m = 40, k = 1000, guy = 1.2, tilt = 4 * D, md = 3;
    // rocking about the base pin: I θ'' = -(k_lat H² − m g H/2) θ, with two guy springs at angle α
    const cos2 = (guy * guy) / (guy * guy + H * H), I = m * (H * H / 3 + W * W / 12);
    const w0 = Math.sqrt((2 * k * cos2 * H * H - (m * 9.81 * H) / 2) / I);
    const mu = (md * H * H) / I, f = 1 / (1 + mu), zeta = Math.sqrt((3 * mu) / (8 * (1 + mu) ** 3)); // Den Hartog's tuning
    const Lp = 9.81 / (f * w0) ** 2, c = 2 * zeta * md * f * w0;
    M.addBody(s, { kind: 'box', x: 0, y: 0.7, w: 6.6, h: 0.2, mass: 10, fixed: true, color: 'light' });
    for (const [cx, damped] of [[-1.6, false], [1.6, true]]) {
      const base = [cx, 0.6];
      const tower = M.addBody(s, { kind: 'box', x: cx, y: base[1] - H / 2, w: W, h: H, hollow: true, wall: 0.06, mass: m, label: damped ? 'M' : 'M_0' });
      pinRods(s, tower, null, base);
      for (const sd of [-1, 1]) {
        const topC = [cx + (sd * W) / 2, base[1] - H], foot = [cx + sd * (W / 2 + guy), base[1]];
        M.addJoint(s, { kind: 'spring', a: on(s, tower, ...topC), b: ground(...foot), k, c: 0, rest: len(topC, foot), coils: 16, label: sd < 0 && !damped ? 'k' : '' });
      }
      if (damped) {
        const hang = [cx, base[1] - H + 0.1], bob = [cx, hang[1] + Lp];
        const ball = M.addBody(s, { kind: 'disc', x: bob[0], y: bob[1], r: 0.16, mass: md, label: 'm_d', color: 'ink' });
        M.addJoint(s, { kind: 'rope', a: on(s, tower, ...hang), b: { body: ball.id, x: 0, y: 0 }, length: Lp });
        const wallP = [cx + W / 2 - 0.06, bob[1]];
        M.addJoint(s, { kind: 'damper', a: { body: ball.id, x: 0, y: 0 }, b: on(s, tower, ...wallP), c, len0: len(bob, wallP), label: 'c' });
      }
      // knock the tower over a little: rotate everything on it about the base pin
      P.index(s);
      for (const b of s.bodies) {
        if (b.fixed || Math.abs(b.x - cx) > 1.3 || b.y > 0.6) continue;
        const dx = b.x - base[0], dy = b.y - base[1];
        [b.x, b.y] = [base[0] + dx * Math.cos(tilt) - dy * Math.sin(tilt), base[1] + dx * Math.sin(tilt) + dy * Math.cos(tilt)];
        if (b.kind !== 'disc') b.a += tilt;
      }
    }
    return s;
  }

  // Windscreen wipers at real car size: a motor turns a short crank, a long link rocks the
  // first wiper's pivot, and a second link copies that motion to the other wiper (a
  // parallelogram, so both sweep together, about 80° each). The blade tips move at about 2 m/s.
  function wipers() {
    const s = M.newScene();
    M.addBody(s, { kind: 'box', x: -0.45, y: 0.22, w: 1.7, h: 0.06, mass: 10, fixed: true }); // the scuttle panel under the glass
    // crank 8 cm, link 50 cm, rocker 12.5 cm: the crank's throw rocks the rocker ±40°, and the
    // lengths keep well clear of the point where the linkage would jam
    const R = 0.125, Lc = 0.5, cr = 0.08, W1 = [0, 0.05], W2 = [-0.75, 0.05];
    const O = [W1[0] + Lc, W1[1] + R], C = [O[0], O[1] - cr];
    const T1 = meet(W1, R, C, Lc, lowest), T2 = [T1[0] + W2[0] - W1[0], T1[1]];
    const crank = rod(s, O, C, { ideal: false });
    M.addJoint(s, { kind: 'pivot', a: on(s, crank, ...O), b: ground(...O), motor: 40, torque: 6 });
    const link1 = rod(s, C, T1, { ideal: false }), link2 = rod(s, T1, T2, { ideal: false });
    pinRods(s, crank, link1, C);
    // Each arm is glued to its rocker. At the middle of the swing the rocker points straight
    // down and the arm points 45° up, so the arm sweeps from nearly flat to nearly upright.
    const dev = Math.atan2(T1[1] - W1[1], T1[0] - W1[0]) - 90 * D, ang = -45 * D + dev;
    for (const [W, T, lab] of [[W1, T1, 'L_1'], [W2, T2, 'L_2']]) {
      const rocker = rod(s, W, T, { ideal: false });
      const tipP = [W[0] + 0.6 * Math.cos(ang), W[1] + 0.6 * Math.sin(ang)];
      const arm = rod(s, W, tipP, { ideal: false, mass: 0.3, label: lab });
      const blade = M.addBody(s, { kind: 'disc', x: tipP[0], y: tipP[1], r: 0.015, mass: 0.1, color: 'ink', trace: true });
      pinRods(s, rocker, null, W);
      glue(s, rocker, arm, W);
      pinRods(s, arm, blade, tipP);
      pinRods(s, rocker, link2, T);
      if (W === W1) pinRods(s, rocker, link1, T);
    }
    return s;
  }

  // Tower crane: a trolley runs out and back along the jib on a hydraulic ram, and the 500 kg
  // load swings on its cable every time the trolley starts or stops. Crane drivers time their
  // moves to cancel that swing; try changing the ram's speed to see how hard it is.
  function towerCrane() {
    const s = M.newScene();
    const floor = 0.6, jibY = -3.2;
    M.addBody(s, { kind: 'box', x: 0, y: floor + 0.15, w: 1.4, h: 0.3, mass: 10, fixed: true, color: 'dark' });
    M.addBody(s, { kind: 'box', x: 0, y: (floor + jibY) / 2, w: 0.32, h: floor - jibY, mass: 10, fixed: true, hollow: true, wall: 0.05 }); // mast
    const jib = M.addBody(s, { kind: 'box', x: 1.0, y: jibY - 0.1, w: 5.6, h: 0.2, mass: 10, fixed: true });
    shape(s, [[-0.12, jibY - 0.2], [0.12, jibY - 0.2], [0, jibY - 1.1]], { mass: 10, fixed: true }); // A-frame
    M.addBody(s, { kind: 'box', x: -1.35, y: jibY + 0.3, w: 0.7, h: 0.6, mass: 10, fixed: true, color: 'dark', label: 'M_c' }); // counterweight
    const x0 = 1.0;
    const trolley = M.addBody(s, { kind: 'box', x: x0, y: jibY + 0.1, w: 0.36, h: 0.2, mass: 80, color: 'light' });
    M.addJoint(s, { kind: 'slider', a: on(s, jib, x0, jibY + 0.1), b: { body: trolley.id, x: 0, y: 0 }, dir: [1, 0], limits: true, lo: -0.6, hi: 2.4 });
    const cable = 2.6, load = M.addBody(s, { kind: 'box', x: x0, y: jibY + 0.2 + cable + 0.2, w: 0.7, h: 0.4, mass: 500, label: 'm', color: 'dark', trace: true });
    M.addJoint(s, { kind: 'rope', a: { body: trolley.id, x: 0, y: 0.1 }, b: { body: load.id, x: 0, y: -0.2 }, length: cable });
    M.addJoint(s, { kind: 'piston', a: on(s, jib, 0.25, jibY + 0.1), b: { body: trolley.id, x: -0.18, y: 0 }, min: 0.55, max: 2.6, len0: 0.57, speed: 0.6, force: 4000, mode: 'cycle', phase: 1 });
    return s;
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
    { id: 'beam', group: 'Statics', opts: { reactions: true }, name: 'Simply supported beam', blurb: 'A pin, a roller and a 1000 N load. Tap the beam for its shear and moment diagrams.', build: beam },
    { id: 'bracket', group: 'Statics', opts: { reactions: true }, name: 'Wall bracket and cable', blurb: 'A beam held level by a cable. Find the cable tension and the reaction at the wall.', build: bracket },
    { id: 'cantilever', group: 'Statics', opts: { reactions: true }, name: 'Cantilever', blurb: 'Glued into a wall, with a tip load and a couple. The wall pushes back with a moment too.', build: cantilever },
    { id: 'truss', group: 'Statics', opts: { reactions: true }, name: 'Truss bridge', blurb: 'A Warren truss: every member in tension (T) or compression (C). Tap a pin to see the method of joints.', build: truss },
    { id: 'pumpjack', group: 'Real machines', name: 'Oil pumpjack', blurb: 'The nodding donkey: a counterweighted crank rocks the beam and lifts the well rods.', build: pumpjack },
    { id: 'excavator', group: 'Real machines', name: 'Excavator arm', blurb: 'Boom, stick and bucket, each on its own hydraulic cylinder.', build: excavator },
    { id: 'crane', group: 'Real machines', name: 'Tower crane', blurb: 'The trolley runs along the jib and the 500 kg load swings on its cable.', build: towerCrane },
    { id: 'tmd', group: 'Real machines', name: 'Tuned mass damper', blurb: 'Two swaying towers. The one with a pendulum inside (like Taipei 101) calms down fast.', build: tunedMassDamper },
    { id: 'wipers', group: 'Real machines', name: 'Windscreen wipers', blurb: 'A motor crank and two links sweep both wipers together.', build: wipers },
    { id: 'strandbeest', group: 'Real machines', name: 'Strandbeest legs', blurb: "Theo Jansen's walking linkage. Watch the feet draw a flat-bottomed stride.", build: strandbeest },
    { id: 'trebuchet', group: 'Real machines', name: 'Trebuchet', blurb: 'A 400 kg counterweight whips a stone round on a sling. Check v_max.', build: trebuchet },
    { id: 'empty', name: 'Blank sheet', blurb: 'Start from nothing.', build: () => M.newScene() },
  ];
})();
