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

  // ---------- vibrations ----------

  // Kapitza's pendulum: an upside-down pendulum that stays up because its pivot is shaken up and
  // down fast. A motor crank (like the jigsaw motor in lab demos) bounces the pivot 8 cm each way
  // at 670 rpm. The rule for a weight on a light rod: it stays up when (a ω)² > 2 g L, here
  // 31 m²/s² against 9.8. Slow the motor below about 400 rpm and it falls over.
  function kapitza() {
    const s = M.newScene();
    s.settings.air = 0; s.settings.substeps = 200; // 11 shakes a second: worth the accuracy
    const O = [0, 0.7], a = 0.08, Lc = 0.45, L = 0.5, tilt = 12 * D;
    const crank = M.addBody(s, { kind: 'disc', x: O[0], y: O[1], r: 0.12, mass: 1, color: 'light' });
    M.addJoint(s, { kind: 'pivot', a: { body: crank.id, x: 0, y: 0 }, b: ground(...O), motor: 670, torque: 60 });
    const pinC = [O[0], O[1] - a], piv = [O[0], pinC[1] - Lc];
    const conrod = rod(s, pinC, piv);
    pinRods(s, crank, conrod, pinC);
    const car = M.addBody(s, { kind: 'box', x: piv[0], y: piv[1], w: 0.16, h: 0.1, mass: 0.5, color: 'dark' });
    pinRods(s, conrod, car, piv);
    M.addJoint(s, { kind: 'slider', a: ground(...piv), b: { body: car.id, x: 0, y: 0 }, dir: [0, 1], lock: true, limits: false, lo: -0.3, hi: 0.3 });
    const top = [piv[0] + L * Math.sin(tilt), piv[1] - L * Math.cos(tilt)];
    const bob = M.addBody(s, { kind: 'disc', x: top[0], y: top[1], r: 0.06, mass: 0.3, color: 'ink', label: 'm', trace: true });
    const arm = rod(s, piv, top, { label: 'L' });
    pinRods(s, car, arm, piv);
    pinRods(s, arm, bob, top);
    return s;
  }

  // Two pendulums joined by a weak spring. Start one swinging and its energy drains into the
  // other, then comes back: beats, the slow swap between the two normal modes.
  function coupledPendulums() {
    const s = M.newScene();
    s.settings.air = 0;
    const L = 1.5, gap = 1.0, hang = 0.5, kick = 14 * D;
    M.addBody(s, { kind: 'box', x: 0, y: -1.6, w: 2.2, h: 0.14, mass: 10, fixed: true }); // the ceiling beam
    const bobs = [-1, 1].map((sd, i) => {
      const piv = [sd * gap / 2, -1.53], th = i === 0 ? kick : 0;
      const end = [piv[0] + L * Math.sin(th), piv[1] + L * Math.cos(th)];
      const bob = M.addBody(s, { kind: 'disc', x: end[0], y: end[1], r: 0.12, mass: 1, color: 'ink', label: `m_${i + 1}`, trace: false });
      const arm = rod(s, piv, end, { ideal: false, mass: 0.1, label: `L_${i + 1}` });
      pinRods(s, arm, null, piv); pinRods(s, arm, bob, end);
      return { arm, piv, th };
    });
    // the spring joins the two rods `hang` below their pivots, relaxed when both hang straight
    const pt = (q) => [q.piv[0] + hang * Math.sin(q.th), q.piv[1] + hang * Math.cos(q.th)];
    M.addJoint(s, { kind: 'spring', a: on(s, bobs[0].arm, ...pt(bobs[0])), b: on(s, bobs[1].arm, ...pt(bobs[1])), k: 3, c: 0, rest: gap, coils: 12, label: 'k' });
    return s;
  }

  // Resonance: one shaking beam, three weights on springs of different stiffness. A motor crank
  // shakes the beam 3 cm each way at 95 rpm, 1.6 times a second, which is the middle spring's own
  // frequency (f = √(k/m) / 2π with k = 100 N/m, m = 1 kg), so that one builds up big swings while
  // the other two hardly move. Change the motor's speed to pick a different one: 60 rpm for the
  // soft spring, 150 rpm for the stiff one.
  function resonance() {
    const s = M.newScene();
    s.settings.air = 0;
    const ks = [40, 100, 250], xs = [-1.2, 0, 1.2], drop = 1.2, a = 0.03;
    const beam = M.addBody(s, { kind: 'box', x: 0, y: -1.2, w: 3.2, h: 0.16, mass: 5, color: 'dark' });
    M.addJoint(s, { kind: 'slider', a: ground(0, -1.2), b: { body: beam.id, x: 0, y: 0 }, dir: [0, 1], lock: true, limits: false, lo: -0.2, hi: 0.2 });
    // the shaker: a crank above the beam, pushing it through a short rod
    const O = [0, -1.8], crank = M.addBody(s, { kind: 'disc', x: O[0], y: O[1], r: 0.08, mass: 1, color: 'light' });
    M.addJoint(s, { kind: 'pivot', a: { body: crank.id, x: 0, y: 0 }, b: ground(...O), motor: 95, torque: 80 });
    const cp = [O[0] + a, O[1]], bt = [0, -1.28], push = rod(s, cp, bt);
    pinRods(s, crank, push, cp); pinRods(s, push, beam, bt);
    ks.forEach((k, i) => {
      const sag = 9.81 / k, y = -1.12 + drop; // hang at rest: the spring already carries the weight
      const m = M.addBody(s, { kind: 'box', x: xs[i], y, w: 0.3, h: 0.24, mass: 1, label: `m_${i + 1}`, color: i === 1 ? 'grey' : 'light' });
      M.addJoint(s, { kind: 'slider', a: ground(xs[i], y), b: { body: m.id, x: 0, y: 0 }, dir: [0, 1], lock: true, limits: false, lo: -1, hi: 1 });
      M.addJoint(s, { kind: 'spring', a: on(s, beam, xs[i], -1.12), b: { body: m.id, x: 0, y: -0.12 }, k, c: 1.5, rest: drop - 0.12 - sag, coils: 10, label: `k_${i + 1}` });
    });
    return s;
  }

  // ---------- linkages ----------

  // Watt's linkage: two equal arms pinned to the frame and a short link between them. The middle
  // of that link moves in a nearly straight line, so cars use it to keep the rear axle going
  // straight up and down. Here the axle bounces on a spring; its path is the traced line.
  function wattLinkage() {
    const s = M.newScene();
    s.settings.air = 0;
    const R = 1.2, h = 0.25, m = 5, k = 300;
    const A = [-R, -h], B = [0, -h], C = [0, h], Dp = [R, h];
    const left = rod(s, A, B, { label: 'L_1' }), right = rod(s, Dp, C, { label: 'L_2' });
    const link = rod(s, B, C, { ideal: false, mass: 1 }); // the axle bracket
    const axle = M.addBody(s, { kind: 'disc', x: 0, y: 0, r: 0.1, mass: m, color: 'ink', label: 'm', trace: true });
    pinRods(s, left, null, A); pinRods(s, right, null, Dp);
    pinRods(s, left, link, B); pinRods(s, right, link, C); pinRods(s, link, axle, [0, 0]);
    // spring from above, carrying the axle's weight when it sits in the middle
    M.addJoint(s, { kind: 'spring', a: ground(0, -1.6), b: { body: axle.id, x: 0, y: 0 }, k, c: 0, rest: 1.6 - (m * 9.81) / k, coils: 12, label: 'k' });
    P.pose(s, { body: axle.id, x: 0, y: 0, tx: 0.02, ty: -0.35 }, 60); // lift it to start it bouncing
    return s;
  }

  // Peaucellier–Lipkin linkage (1864): the first linkage to draw a perfectly straight line. Two
  // long arms and a rhombus of four short ones make point P the "inverse" of Q, and as Q runs round
  // a circle through the fixed pin O, P runs along a straight line. Here P carries a weight on a
  // spring, and the linkage alone keeps it on the line: no track.
  function peaucellier() {
    const s = M.newScene();
    s.settings.air = 0;
    const O = [-1.0, 0], r = 0.5, l1 = 1.4, l2 = 0.5, K = l1 * l1 - l2 * l2, phi = 35 * D;
    const O2 = [O[0] + r, O[1]], Q = [O2[0] + r * Math.cos(phi), O2[1] - r * Math.sin(phi)];
    const q2 = (Q[0] - O[0]) ** 2 + (Q[1] - O[1]) ** 2, Pp = [O[0] + ((Q[0] - O[0]) * K) / q2, O[1] + ((Q[1] - O[1]) * K) / q2];
    const A = meet(O, l1, Q, l2, highest), B = meet(O, l1, Q, l2, lowest);
    const bar = (p, q, lab = '') => rod(s, p, q, { ideal: false, label: lab });
    const oa = bar(O, A, 'l_1'), ob = bar(O, B), crank = bar(O2, Q, 'r');
    const aq = bar(A, Q), bq = bar(B, Q), ap = bar(A, Pp, 'l_2'), bp = bar(B, Pp);
    joint(s, [oa, ob], O, true);
    pinRods(s, crank, null, O2);
    joint(s, [crank, aq, bq], Q);
    joint(s, [oa, aq, ap], A);
    joint(s, [ob, bq, bp], B);
    const pen = M.addBody(s, { kind: 'disc', x: Pp[0], y: Pp[1], r: 0.08, mass: 2, color: 'ink', label: 'P', trace: true });
    joint(s, [ap, bp, pen], Pp);
    // the weight's spring hangs from above the line; relaxed with the weight at the middle
    const top = [O[0] + K / (2 * r), -2.2];
    M.addJoint(s, { kind: 'spring', a: ground(...top), b: { body: pen.id, x: 0, y: 0 }, k: 60, c: 0, rest: 2.2 - (2 * 9.81) / 60, coils: 14, label: 'k' });
    return s;
  }

  // Chebyshev's lambda linkage: a four-bar where the end of the long coupler draws a "D" whose
  // flat side is very nearly straight and is crossed at an almost steady speed. Chebyshev used it
  // in his walking machine. Proportions 1 : 2 : 2.5 : 2.5, with the foot at twice the coupler length.
  function chebyshev() {
    const s = M.newScene();
    const u = 0.4, A = [-0.4, -0.4], Dp = [A[0] + 2 * u, A[1]];
    const B = [A[0], A[1] - u], C = meet(B, 2.5 * u, Dp, 2.5 * u, highest), Mf = [B[0] + 2 * (C[0] - B[0]), B[1] + 2 * (C[1] - B[1])];
    const crank = rod(s, A, B, { ideal: false, label: 'r' });
    const coupler = rod(s, B, Mf, { ideal: false });
    const rocker = rod(s, Dp, C, { ideal: false });
    M.addJoint(s, { kind: 'pivot', a: on(s, crank, ...A), b: ground(...A), motor: 30, torque: 30 });
    pinRods(s, crank, coupler, B);
    pinRods(s, coupler, rocker, C);
    pinRods(s, rocker, null, Dp);
    const foot = M.addBody(s, { kind: 'disc', x: Mf[0], y: Mf[1], r: 0.05, mass: 0.2, color: 'ink', trace: true });
    pinRods(s, coupler, foot, Mf);
    return s;
  }

  // Pintograph: a drawing toy. Two motor cranks at slightly different speeds (4 : 5) push two
  // arms that hold a pen where they meet; the pen draws a looping pattern that slowly turns.
  function pintograph() {
    const s = M.newScene();
    s.settings.air = 0;
    const L1 = [-0.75, 0.6], R1 = [0.75, 0.6], rA = 0.35, rB = 0.42, arm = 1.3;
    const wheel = (c, r, rpm) => {
      const d = M.addBody(s, { kind: 'disc', x: c[0], y: c[1], r: r + 0.06, mass: 1, color: 'light' });
      M.addJoint(s, { kind: 'pivot', a: { body: d.id, x: 0, y: 0 }, b: ground(...c), motor: rpm, torque: 40 });
      return d;
    };
    const wa = wheel(L1, rA, 24), wb = wheel(R1, rB, 30);
    const pa = [L1[0] + rA, L1[1]], pb = [R1[0] - rB, R1[1]];
    const pen = meet(pa, arm, pb, arm, highest);
    const a1 = rod(s, pa, pen, { gscale: 0 }), a2 = rod(s, pb, pen, { gscale: 0 });
    pinRods(s, wa, a1, pa); pinRods(s, wb, a2, pb);
    const tip = M.addBody(s, { kind: 'disc', x: pen[0], y: pen[1], r: 0.03, mass: 0.05, gscale: 0, color: 'ink', label: 'P', trace: true });
    joint(s, [a1, a2, tip], pen);
    return s;
  }

  // Whitworth quick-return mechanism, as in a metal shaper: the crank's pin slides in a slotted
  // lever, the lever swings a link that drives the ram. The cutting stroke takes 240° of the
  // crank's turn and the return only 120°, so the ram comes back twice as fast as it cuts.
  function quickReturn() {
    const s = M.newScene();
    const Q = [0, 1.2], O = [0, 0.4], r = 0.4, T = [0, -0.8], ramY = -0.68, Lk = 0.9;
    const base = M.addBody(s, { kind: 'box', x: 0, y: 1.45, w: 2.8, h: 0.2, mass: 10, fixed: true }); // base
    const crank = M.addBody(s, { kind: 'disc', x: O[0], y: O[1], r: r + 0.08, mass: 3, color: 'light' });
    M.addJoint(s, { kind: 'pivot', a: { body: crank.id, x: 0, y: 0 }, b: ground(...O), motor: 30, torque: 1500 });
    const lever = rod(s, Q, T, { ideal: false, mass: 4, label: 'L' });
    pinRods(s, lever, null, Q);
    const pinP = [O[0], O[1] - r];
    const block = M.addBody(s, { kind: 'box', x: pinP[0], y: pinP[1], w: 0.16, h: 0.1, a: -Math.PI / 2, mass: 1, color: 'dark' });
    pinRods(s, crank, block, pinP);
    // the block slides along the lever's slot (and keeps the lever's angle)
    M.addJoint(s, { kind: 'slider', a: on(s, lever, ...pinP), b: { body: block.id, x: 0, y: 0 }, dir: [1, 0], lock: true, angle0: block.a - lever.a, limits: false, lo: -0.6, hi: 0.6 });
    // a nearly level link from the top of the lever to the ram, which slides along y = ramY
    const R = [T[0] + Math.sqrt(Lk * Lk - (ramY - T[1]) ** 2), ramY], link = rod(s, T, R, { ideal: false, mass: 1 });
    pinRods(s, lever, link, T);
    const ram = M.addBody(s, { kind: 'box', x: R[0] + 0.3, y: R[1] - 0.16, w: 1.2, h: 0.24, mass: 20, label: 'M', color: 'grey' });
    pinRods(s, link, ram, R);
    // the ram's slideway is part of the machine's frame (the base), not the floor
    M.addJoint(s, { kind: 'slider', a: on(s, base, R[0] + 0.3, R[1] - 0.16), b: { body: ram.id, x: 0, y: 0 }, dir: [1, 0], lock: true, limits: false, lo: -1.3, hi: 1.3 });
    return s;
  }

  // ---------- machines you drive ----------
  // The same machines with their motors, pistons and winches on keys (see controls.js). Let go
  // of a key and a piston's valve shuts and a motor brakes, so everything holds where it is.

  const keysOn = (j, out, inn, label) => Object.assign(j, { mode: 'keys', keyOut: out, keyIn: inn, label: label || j.label });

  function driveExcavator() {
    const s = excavator();
    const [boom, stick, bucket] = s.joints.filter((j) => j.kind === 'piston');
    keysOn(boom, 'KeyW', 'KeyS', 'boom'); keysOn(stick, 'KeyI', 'KeyK', 'stick'); keysOn(bucket, 'KeyL', 'KeyJ', 'bucket');
    for (const j of [boom, stick, bucket]) j.speed *= 1.5; // a bit livelier by hand
    return s;
  }

  function driveCrane() {
    const s = towerCrane();
    const ram = s.joints.find((j) => j.kind === 'piston'), cable = s.joints.find((j) => j.kind === 'rope');
    keysOn(ram, 'KeyD', 'KeyA', 'trolley');
    Object.assign(cable, { keyIn: 'KeyW', keyOut: 'KeyS', winch: 0.4, label: 'hoist' });
    return s;
  }

  // A two-joint robot arm: a motor at the shoulder and one at the elbow, each on two keys.
  // They brake when you let go, so the arm holds its pose with the 2 kg load in its hand.
  function robotArm() {
    const s = M.newScene();
    const base = [0, 0.4], elbow = [0.9, -0.3], hand = [1.6, 0.1];
    M.addBody(s, { kind: 'box', x: 0, y: 0.62, w: 0.7, h: 0.44, mass: 10, fixed: true }); // the base
    const upper = rod(s, base, elbow, { ideal: false, mass: 3, label: 'L_1' });
    const fore = rod(s, elbow, hand, { ideal: false, mass: 2, label: 'L_2' });
    const load = M.addBody(s, { kind: 'disc', x: hand[0], y: hand[1], r: 0.1, mass: 2, color: 'ink', label: 'm', trace: true });
    M.addJoint(s, { kind: 'pivot', a: on(s, upper, ...base), b: ground(...base), motor: 20, torque: 150, keyFwd: 'KeyA', keyRev: 'KeyD', idle: 'brake', label: 'shoulder' });
    M.addJoint(s, { kind: 'pivot', a: on(s, fore, ...elbow), b: on(s, upper, ...elbow), motor: 25, torque: 80, keyFwd: 'KeyJ', keyRev: 'KeyL', idle: 'brake', label: 'elbow' });
    M.addJoint(s, { kind: 'weld', a: on(s, fore, ...hand), b: on(s, load, ...hand), angle0: load.a - fore.a });
    return s;
  }

  // A drawbridge (a bascule): the deck hinges at the bank and a winch rope over a pulley on the
  // tower lifts its far end. Reel in to raise it, let out to lower it. Parts here don't touch,
  // so the winch stops letting out when the deck is level (as if it sat on the far bank).
  function drawbridge() {
    const s = M.newScene();
    const hinge = [-1.6, 0], tip = [1.6, 0], top = [-1.6, -3.6];
    M.addBody(s, { kind: 'box', x: -2.2, y: 0.5, w: 1.3, h: 1.0, mass: 10, fixed: true }); // near bank
    M.addBody(s, { kind: 'box', x: 2.35, y: 0.5, w: 1.3, h: 1.0, mass: 10, fixed: true });  // far bank
    M.addBody(s, { kind: 'box', x: -2.05, y: -1.8, w: 0.3, h: 3.6, mass: 10, fixed: true }); // tower
    const deck = M.addBody(s, { kind: 'box', x: 0, y: -0.06, w: 3.2, h: 0.12, mass: 200, label: 'M' });
    M.addJoint(s, { kind: 'pivot', a: on(s, deck, ...hinge), b: ground(...hinge) });
    const j = M.addJoint(s, { kind: 'pulley', a: on(s, deck, tip[0], -0.12), b: ground(-1.9, -3.1), p: ground(...top), r: 0.18, length: 0, keyIn: 'KeyW', keyOut: 'KeyS', winch: 0.5, label: 'winch' });
    P.index(s); j.length = P.pulleyGeom(s, j).length;
    // the drum's limits: the rope's length with the deck level, and with it raised to 75°
    // (measured by posing the deck there, so the rope's wrap round the wheel counts too)
    const keep = [deck.x, deck.y, deck.a], th = -75 * D;
    [deck.x, deck.y, deck.a] = [hinge[0] + 1.6 * Math.cos(th), hinge[1] + 1.6 * Math.sin(th), th];
    j.wind = 0; const raised = P.pulleyGeom(s, j).length;
    [deck.x, deck.y, deck.a] = keep; j.wind = 0; P.pulleyGeom(s, j);
    j.winchMax = j.length; j.winchMin = raised;
    return s;
  }

  // CoreXY, as in many 3D printers and plotters, seen from above (so no gravity). The gantry
  // slides up and down the frame, the head slides along the gantry, and both motors stay put on
  // the frame. Belt A moves the head by x + y and belt B by x − y, so both motors turning the
  // same way move it across, opposite ways move it up or down, and one motor alone moves it on
  // a 45° diagonal. Each arrow key turns both motors the right way; the head draws its path.
  function coreXY() {
    const s = M.newScene();
    s.settings.gravity = 0; s.settings.air = 0.5;
    M.addBody(s, { kind: 'box', x: 0, y: 0, w: 2.1, h: 1.7, hollow: true, wall: 0.06, mass: 10, fixed: true }); // the frame
    const pulley = (x, lab, fwd, rev) => {
      const d = M.addBody(s, { kind: 'disc', x, y: -0.72, r: 0.05, mass: 0.2, color: 'dark', label: lab });
      M.addJoint(s, { kind: 'pivot', a: { body: d.id, x: 0, y: 0 }, b: ground(x, -0.72), motor: 150, torque: 4, keyFwd: fwd, keyRev: rev, idle: 'brake', label: 'motor ' + lab });
      return d;
    };
    const mA = pulley(-0.9, 'A', 'ArrowRight ArrowUp', 'ArrowLeft ArrowDown');
    const mB = pulley(0.9, 'B', 'ArrowRight ArrowDown', 'ArrowLeft ArrowUp');
    const gan = M.addBody(s, { kind: 'box', x: 0, y: 0.1, w: 1.8, h: 0.05, mass: 1, color: 'grey' });
    const gy = M.addJoint(s, { kind: 'slider', a: ground(0, 0.1), b: { body: gan.id, x: 0, y: 0 }, dir: [0, 1], lock: true, limits: true, lo: -0.55, hi: 0.6 });
    const car = M.addBody(s, { kind: 'box', x: 0, y: 0.1, w: 0.18, h: 0.14, mass: 0.4, color: 'dark', label: 'M', trace: true });
    const cx = M.addJoint(s, { kind: 'slider', a: on(s, gan, 0, 0.1), b: { body: car.id, x: 0, y: 0 }, dir: [1, 0], lock: true, limits: true, lo: -0.78, hi: 0.78 });
    const at = (b, x, y, idler) => ({ body: b ? b.id : null, x, y, idler });
    // belt A: x + y (y up); belt B: x − y. (A positive motor turns its pulley anticlockwise.)
    M.addJoint(s, { kind: 'belt', a: { body: mA.id, x: 0, y: 0 }, b: { body: car.id, x: 0, y: 0 }, sx: cx.id, sy: gy.id, cx: 1, cy: -1, r: 0.05, c0: 0,
      path: [at(car, -0.09, -0.02), at(gan, -0.85, -0.02, true), at(mA, 0, 0), at(null, 0.97, -0.76, true), at(gan, 0.85, -0.02, true), at(car, 0.09, -0.02)] });
    M.addJoint(s, { kind: 'belt', a: { body: mB.id, x: 0, y: 0 }, b: { body: car.id, x: 0, y: 0 }, sx: cx.id, sy: gy.id, cx: 1, cy: 1, r: 0.05, c0: 0, dashed: true,
      path: [at(car, 0.09, 0.02), at(gan, 0.85, 0.02, true), at(mB, 0, 0), at(null, -0.97, -0.76, true), at(gan, -0.85, 0.02, true), at(car, -0.09, 0.02)] });
    return s;
  }

  ML.Presets = [
    { id: 'frame', name: 'The swinging frame', blurb: 'The mechanism from your video.', build: swingingFrame },
    { id: 'atwood', name: 'Atwood machine', blurb: 'Two masses on a pulley. The heavier one wins, slowly.', build: atwood },
    { id: 'double', name: 'Double pendulum', blurb: 'Two rods, total chaos. Never swings the same way twice.', build: doublePendulum },
    { id: 'springpend', name: 'Spring pendulum', blurb: 'A weight on a spring that swings and bounces at once.', build: springPendulum },
    { id: 'coupled', name: 'Coupled oscillators', blurb: 'Two blocks passing their energy back and forth through springs.', build: coupledOscillators },
    { id: 'kapitza', group: 'Vibrations', name: 'Kapitza’s pendulum', blurb: 'An upside-down pendulum that stays up because its pivot shakes fast. Slow the motor and it falls.', build: kapitza },
    { id: 'coupledpend', group: 'Vibrations', name: 'Coupled pendulums', blurb: 'Two pendulums and a weak spring: the swing passes from one to the other and back.', build: coupledPendulums },
    { id: 'resonance', group: 'Vibrations', name: 'Resonance', blurb: 'One shaking beam, three springs. Only the one tuned to the shaking builds up big swings.', build: resonance },
    { id: 'suspension', group: 'Vibrations', name: 'Car suspension', blurb: 'A bumpy road, a spring and a damper. Set the damper to 0 to feel the difference.', build: suspension },
    { id: 'fourbar', group: 'Linkages', name: 'Four-bar linkage', blurb: 'A motor turns the crank and the coupler draws a loop.', build: fourBar },
    { id: 'rodlinkage', group: 'Linkages', name: 'Rod linkage', blurb: 'Three rods pinned end to end, driven by a motor crank.', build: rodLinkage },
    { id: 'piston', group: 'Linkages', name: 'Crank and slider', blurb: 'A motor-driven wheel pushing a slider, like an engine in reverse.', build: crankPiston },
    { id: 'watt', group: 'Linkages', name: 'Watt’s linkage', blurb: 'Keeps a car’s rear axle going straight up and down. The traced path is nearly a straight line.', build: wattLinkage },
    { id: 'peaucellier', group: 'Linkages', name: 'Peaucellier–Lipkin linkage', blurb: 'Eight rods that draw an exactly straight line, with no track.', build: peaucellier },
    { id: 'chebyshev', group: 'Linkages', name: 'Chebyshev lambda linkage', blurb: 'A four-bar whose foot draws a D with a nearly straight, steady flat side.', build: chebyshev },
    { id: 'pintograph', group: 'Linkages', name: 'Pintograph', blurb: 'A drawing toy: two cranks at different speeds steer a pen into a slowly turning pattern.', build: pintograph },
    { id: 'beam', group: 'Statics', opts: { reactions: true }, name: 'Simply supported beam', blurb: 'A pin, a roller and a 1000 N load. Tap the beam for its shear and moment diagrams.', build: beam },
    { id: 'bracket', group: 'Statics', opts: { reactions: true }, name: 'Wall bracket and cable', blurb: 'A beam held level by a cable. Find the cable tension and the reaction at the wall.', build: bracket },
    { id: 'cantilever', group: 'Statics', opts: { reactions: true }, name: 'Cantilever', blurb: 'Glued into a wall, with a tip load and a couple. The wall pushes back with a moment too.', build: cantilever },
    { id: 'truss', group: 'Statics', opts: { reactions: true }, name: 'Truss bridge', blurb: 'A Warren truss: every member in tension (T) or compression (C). Tap a pin to see the method of joints.', build: truss },
    { id: 'f1', group: 'Real machines', name: 'F1 front suspension', blurb: 'Double wishbones, pushrods and rockers on a shaker rig, like the ones F1 teams test on.', build: f1Suspension },
    { id: 'scissor', group: 'Real machines', name: 'Scissor lift', blurb: 'Rods crossed in X shapes and pinned, opened by a piston.', build: scissorLift },
    { id: 'pumpjack', group: 'Real machines', name: 'Oil pumpjack', blurb: 'The nodding donkey: a counterweighted crank rocks the beam and lifts the well rods.', build: pumpjack },
    { id: 'excavator', group: 'Real machines', name: 'Excavator arm', blurb: 'Boom, stick and bucket, each on its own hydraulic cylinder.', build: excavator },
    { id: 'crane', group: 'Real machines', name: 'Tower crane', blurb: 'The trolley runs along the jib and the 500 kg load swings on its cable.', build: towerCrane },
    { id: 'tmd', group: 'Real machines', name: 'Tuned mass damper', blurb: 'Two swaying towers. The one with a pendulum inside (like Taipei 101) calms down fast.', build: tunedMassDamper },
    { id: 'wipers', group: 'Real machines', name: 'Windscreen wipers', blurb: 'A motor crank and two links sweep both wipers together.', build: wipers },
    { id: 'quickreturn', group: 'Real machines', name: 'Quick-return shaper', blurb: 'Whitworth’s mechanism: the ram cuts slowly and comes back twice as fast. Watch its speed.', build: quickReturn },
    { id: 'strandbeest', group: 'Real machines', name: 'Strandbeest legs', blurb: "Theo Jansen's walking linkage. Watch the feet draw a flat-bottomed stride.", build: strandbeest },
    { id: 'trebuchet', group: 'Real machines', name: 'Trebuchet', blurb: 'A 400 kg counterweight whips a stone round on a sling. Check v_max.', build: trebuchet },
    { id: 'drive-excavator', group: 'Drive it', name: 'Excavator', blurb: 'Dig with the keys: W / S boom, I / K stick, J / L bucket.', build: driveExcavator },
    { id: 'drive-crane', group: 'Drive it', name: 'Tower crane', blurb: 'A / D runs the trolley, W / S winds the hoist. Try to stop the load swinging.', build: driveCrane },
    { id: 'drive-arm', group: 'Drive it', name: 'Robot arm', blurb: 'Two motors with brakes: A / D for the shoulder, J / L for the elbow.', build: robotArm },
    { id: 'corexy', group: 'Drive it', keysInBlurb: true, name: 'CoreXY', blurb: 'Two fixed motors, two crossed belts, seen from above. The arrow keys move the head (each turns both motors); the head draws its path.', build: coreXY },
    { id: 'drive-bridge', group: 'Drive it', name: 'Drawbridge', blurb: 'Hold W to winch the deck up, S to let it down.', build: drawbridge },
    { id: 'empty', name: 'Blank sheet', blurb: 'Start from nothing.', build: () => M.newScene() },
  ];
})();
