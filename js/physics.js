// physics.js: a small 2D rigid-body solver using XPBD (extended position-based dynamics).
//
// Why XPBD: joints are solved by nudging positions directly, so pivots and rods never
// drift apart or blow up, even with big mass ratios. Each frame is cut into many small
// substeps (h = dt / substeps) with one constraint pass each. That is the trick from
// Müller et al. 2020, "Detailed Rigid Body Simulation with Extended Position Based Dynamics".
//
// Units are SI: metres, kilograms, seconds, radians. World y points DOWN (same as the
// screen), so gravity is +y and a positive angle turns clockwise on screen.
(function () {
  const ML = (window.ML = window.ML || {});

  // ---------- shape geometry and mass properties ----------

  function polyArea(pts) {
    let A = 0;
    for (let i = 0; i < pts.length; i++) {
      const [x1, y1] = pts[i], [x2, y2] = pts[(i + 1) % pts.length];
      A += x1 * y2 - x2 * y1;
    }
    return A / 2;
  }

  function polyCentroid(pts) {
    let A = 0, cx = 0, cy = 0;
    for (let i = 0; i < pts.length; i++) {
      const [x1, y1] = pts[i], [x2, y2] = pts[(i + 1) % pts.length];
      const c = x1 * y2 - x2 * y1;
      A += c; cx += (x1 + x2) * c; cy += (y1 + y2) * c;
    }
    if (Math.abs(A) < 1e-12) return [pts[0][0], pts[0][1]];
    return [cx / (3 * A), cy / (3 * A)];
  }

  // Second moment of area about the origin (which we keep at the centroid), per unit density.
  function polySecondMoment(pts) {
    let J = 0;
    for (let i = 0; i < pts.length; i++) {
      const [x1, y1] = pts[i], [x2, y2] = pts[(i + 1) % pts.length];
      const c = x1 * y2 - x2 * y1;
      J += c * (x1 * x1 + x1 * x2 + x2 * x2 + y1 * y1 + y1 * y2 + y2 * y2);
    }
    return Math.abs(J) / 12;
  }

  const rectJ = (w, h) => (w * h * (w * w + h * h)) / 12;
  // Rods ('bar' bodies) are lines from local (-L/2, 0) to (+L/2, 0); this is their half-thickness
  // for picking them up.
  const BAR_HALF = 0.015;

  function innerSize(b) {
    return [Math.max(0, b.w - 2 * b.wall), Math.max(0, b.h - 2 * b.wall)];
  }

  function shapeArea(b) {
    if (b.kind === 'box') {
      if (!b.hollow) return b.w * b.h;
      const [wi, hi] = innerSize(b);
      return b.w * b.h - wi * hi;
    }
    if (b.kind === 'disc') return Math.PI * b.r * b.r;
    if (b.kind === 'bar') return b.L * 2 * BAR_HALF;
    return Math.abs(polyArea(b.pts));
  }

  // Moment of inertia about the centre of mass, assuming uniform density.
  function inertia(b) {
    const A = Math.max(shapeArea(b), 1e-9);
    if (b.kind === 'box') {
      let J = rectJ(b.w, b.h);
      if (b.hollow) { const [wi, hi] = innerSize(b); J -= rectJ(wi, hi); }
      return (b.mass * J) / A;
    }
    if (b.kind === 'disc') return 0.5 * b.mass * b.r * b.r;
    if (b.kind === 'bar') return (b.mass * b.L * b.L) / 12; // thin rod about its middle
    return (b.mass * polySecondMoment(b.pts)) / A;
  }

  // Fixed bodies get zero inverse mass: the solver can't move them, like the ground.
  function updateMass(b) {
    b._I = Math.max(inertia(b), 1e-9);
    b._invM = b.fixed ? 0 : 1 / b.mass;
    b._invI = b.fixed ? 0 : 1 / b._I;
    // Give every body all of the solver's scratch fields up front, in one order. Fields added
    // later, mid-run, change the object's hidden "shape" in the JavaScript engine, and once the
    // engine has seen a few shapes it stops optimising the solver loops (about 2× slower).
    if (b._ax === undefined) {
      b._fx = b._fy = b._t = 0; b._px = b.x; b._py = b.y; b._pa = b.a; b._f = 0;
      b._facc = 0; b._favg = 0; b._v0 = [0, 0, 0]; b._ax = b._ay = b._aa = 0; b._p0 = [0, 0, 0]; b._vf = [0, 0, 0];
    }
  }

  // ---------- frames ----------

  function toWorld(b, lx, ly) {
    const c = Math.cos(b.a), s = Math.sin(b.a);
    return [b.x + c * lx - s * ly, b.y + s * lx + c * ly];
  }

  function toLocal(b, wx, wy) {
    const c = Math.cos(b.a), s = Math.sin(b.a), dx = wx - b.x, dy = wy - b.y;
    return [c * dx + s * dy, -s * dx + c * dy];
  }

  function rotate(a, x, y) {
    const c = Math.cos(a), s = Math.sin(a);
    return [c * x - s * y, s * x + c * y];
  }

  // Build an id → body lookup and refresh mass data. Cheap, so we just call it often.
  function index(scene) {
    scene._byId = new Map();
    for (const b of scene.bodies) { scene._byId.set(b.id, b); updateMass(b); }
    compileIdeal(scene);
    // A motor has a spinning rotor: J = 0.002 s² × its torque rating, added to what it drives.
    // Without it a strong motor on a light crank rod can whip the crank round at absurd speeds.
    for (const j of scene.joints) {
      if (j.kind !== 'pivot' || !j.motor) continue;
      const d = (j.a.body != null ? scene._byId.get(j.a.body) : null) || (j.b.body != null ? scene._byId.get(j.b.body) : null);
      if (!d || d.fixed || d._red) continue;
      d._I += 0.002 * (j.torque || 0);
      d._invI = 1 / d._I;
    }
    lightLinks(scene);
  }

  // Massless rods. A rod that is marked ideal and is held only by two plain pins, one at each
  // end, to things with mass (or the ground) is solved as a weightless distance constraint
  // between those two things, exactly like a textbook rod; its own pose just follows them. Any
  // other rod (a free end, rod-to-rod, something pinned along it, a motor) uses its real mass.
  // Two ideal rods pinned to each other both use their mass, so the result never depends on order.
  function compileIdeal(scene) {
    for (const j of scene.joints) { j._ideal = null; j._skip = null; }
    const touching = new Map();
    for (const b of scene.bodies) { b._red = null; if (b.kind === 'bar') touching.set(b.id, []); }
    if (!touching.size) return;
    for (const j of scene.joints) {
      for (const k of ['a', 'b', 'p']) { const e = j[k]; if (e && e.body != null && touching.has(e.body)) touching.get(e.body).push(j); }
    }
    for (const b of scene.bodies) {
      if (b.kind !== 'bar' || !b.ideal || b.fixed) continue;
      const js = touching.get(b.id);
      if (js.length !== 2 || js[0] === js[1]) continue;
      const ends = [];
      for (const j of js) {
        if (j.kind !== 'pivot' || j.motor) break;
        const mine = j.a.body === b.id ? j.a : j.b, far = mine === j.a ? j.b : j.a;
        const fb = far.body != null ? scene._byId.get(far.body) : null;
        if (far.body === b.id || (fb && fb.kind === 'bar' && fb.ideal)) break;
        if (Math.abs(mine.y) > 1e-6 || Math.abs(Math.abs(mine.x) - b.L / 2) > 1e-6) break;
        ends.push({ j, side: Math.sign(mine.x), far });
      }
      if (ends.length !== 2 || ends[0].side === ends[1].side) continue;
      const [eA, eB] = ends[0].side < 0 ? ends : [ends[1], ends[0]];
      const firstIsA = scene.joints.indexOf(eA.j) < scene.joints.indexOf(eB.j);
      b._red = { farA: eA.far, farB: eB.far };
      (firstIsA ? eA.j : eB.j)._ideal = b; // solved here, in the joint order an old rod had
      (firstIsA ? eB.j : eA.j)._skip = b;
    }
  }

  // Pins on real rods get solved again each substep (more passes the lighter the rod is next to
  // what it holds), so chains, loops and trusses of rods stay tight. Scenes without real rods
  // are unaffected.
  function lightLinks(scene) {
    scene._light = [];
    for (const j of scene.joints) {
      if (j.kind !== 'pivot' && j.kind !== 'weld') continue;
      const A = bodyOf(scene, j.a), B = bodyOf(scene, j.b);
      if (!((A && A.kind === 'bar' && !A._red) || (B && B.kind === 'bar' && !B._red))) continue;
      if (!A || !B || A.fixed || B.fixed) continue;
      const ratio = Math.max(A.mass, B.mass) / Math.max(1e-9, Math.min(A.mass, B.mass));
      scene._light.push([j, ratio >= 100 ? 3 : ratio >= 10 ? 2 : 1]);
    }
  }

  // Put massless rods back between the two things they join (after the joints move those).
  function poseIdeal(scene) {
    for (const b of scene.bodies) {
      if (!b._red) continue;
      const pA = endWorld(scene, b._red.farA), pB = endWorld(scene, b._red.farB);
      b.x = (pA[0] + pB[0]) / 2; b.y = (pA[1] + pB[1]) / 2;
      b.a = Math.atan2(pB[1] - pA[1], pB[0] - pA[0]);
    }
  }

  // Force along a rod, + for pulling (tension). For a massless rod it's the constraint force;
  // for a real rod with one pin at each end it's worked out from the two pin forces.
  // Returns null when that isn't defined (no pins, or pins along the rod).
  function barAxial(scene, b) {
    if (b._red) return b._favg || b._f || 0;
    const pins = scene.joints.filter((j) => (j.kind === 'pivot' || j.kind === 'weld') && (j.a.body === b.id || j.b.body === b.id));
    if (pins.length !== 2) return null;
    const ux = Math.cos(b.a), uy = Math.sin(b.a);
    let FA = null, FB = null;
    for (const j of pins) {
      const mine = j.a.body === b.id ? j.a : j.b, sg = mine === j.a ? 1 : -1;
      if (Math.abs(Math.abs(mine.x) - b.L / 2) > 1e-6) return null;
      const fx = j._avg ? j._avg.fx : j._fx || 0, fy = j._avg ? j._avg.fy : j._fy || 0; // averaged over the frame
      const F = sg * (fx * ux + fy * uy);
      if (mine.x < 0) FA = F; else FB = F;
    }
    return FA == null || FB == null ? null : (FB - FA) / 2;
  }

  // A joint end is {body, x, y}: local coords on a body, or world coords when body is null (ground).
  const bodyOf = (scene, e) => (e.body == null ? null : scene._byId.get(e.body) || null);
  const angleOf = (scene, e) => { const b = bodyOf(scene, e); return b ? b.a : 0; };

  function endWorld(scene, e) {
    const b = bodyOf(scene, e);
    return b ? toWorld(b, e.x, e.y) : [e.x, e.y];
  }

  // A circle's anchor points (world coords): b.anchors of them evenly round a ring at b.ring × its
  // radius (1 = on the rim), the first at the part's own +x direction, so they turn with it.
  function discAnchors(b) {
    const n = b.kind === 'disc' ? b.anchors || 0 : 0, rr = b.r * (b.ring ?? 1), out = [];
    for (let k = 0; k < n; k++) { const t = (k * 2 * Math.PI) / n; out.push(toWorld(b, rr * Math.cos(t), rr * Math.sin(t))); }
    return out;
  }

  // Is a world point on this body? tol (metres) counts points just outside the edge as hits,
  // so a joint snapped exactly onto an edge or corner still finds its body.
  function hitBody(b, wx, wy, tol = 0) {
    const [lx, ly] = toLocal(b, wx, wy);
    if (b.kind === 'disc') return Math.hypot(lx, ly) <= b.r + tol;
    if (b.kind === 'bar') return Math.abs(lx) <= b.L / 2 + tol && Math.abs(ly) <= BAR_HALF + tol;
    if (b.kind === 'box') {
      if (Math.abs(lx) > b.w / 2 + tol || Math.abs(ly) > b.h / 2 + tol) return false;
      if (!b.hollow) return true;
      const [wi, hi] = innerSize(b);
      return Math.abs(lx) > wi / 2 - tol || Math.abs(ly) > hi / 2 - tol;
    }
    let inside = false;
    const p = b.pts;
    for (let i = 0, k = p.length - 1; i < p.length; k = i++) {
      if ((p[i][1] > ly) !== (p[k][1] > ly) &&
          lx < ((p[k][0] - p[i][0]) * (ly - p[i][1])) / (p[k][1] - p[i][1]) + p[i][0]) inside = !inside;
      if (tol > 0 && segDist(lx, ly, p[i], p[k]) <= tol) return true;
    }
    return inside;
  }

  function segDist(x, y, [ax, ay], [bx, by]) {
    const dx = bx - ax, dy = by - ay, L2 = dx * dx + dy * dy || 1;
    const t = Math.max(0, Math.min(1, ((x - ax) * dx + (y - ay) * dy) / L2));
    return Math.hypot(x - ax - t * dx, y - ay - t * dy);
  }

  // ---------- the two XPBD correction primitives ----------

  // Move point 1 (offset r1 from body 1's centre) and point 2 toward each other to close the
  // gap vector (dx, dy) = p2 - p1. Each body moves in proportion to its "generalised inverse
  // mass" w, which counts both how easy it is to push and how easy it is to spin about that point.
  // compliance = 1/stiffness (0 means perfectly rigid). Returns lambda; force ≈ lambda / h².
  function applyPos(b1, r1x, r1y, b2, r2x, r2y, dx, dy, compliance, h) {
    const c = Math.hypot(dx, dy);
    if (c < 1e-12) return 0;
    const nx = dx / c, ny = dy / c;
    let w = 0;
    if (b1) { const cr = r1x * ny - r1y * nx; w += b1._invM + b1._invI * cr * cr; }
    if (b2) { const cr = r2x * ny - r2y * nx; w += b2._invM + b2._invI * cr * cr; }
    const at = compliance / (h * h);
    if (w + at <= 0) return 0;
    const lam = c / (w + at), px = lam * nx, py = lam * ny;
    if (b1) { b1.x += px * b1._invM; b1.y += py * b1._invM; b1.a += b1._invI * (r1x * py - r1y * px); }
    if (b2) { b2.x -= px * b2._invM; b2.y -= py * b2._invM; b2.a -= b2._invI * (r2x * py - r2y * px); }
    return lam;
  }

  // Rotate two bodies so that (a2 - a1) loses the error c. maxLam optionally caps the
  // correction (that's how motors get a torque limit).
  function applyAng(b1, b2, c, compliance, h, maxLam = Infinity) {
    const w = (b1 ? b1._invI : 0) + (b2 ? b2._invI : 0), at = compliance / (h * h);
    if (w + at <= 0 || c === 0) return 0;
    const lam = Math.max(-maxLam, Math.min(maxLam, c / (w + at)));
    if (b1) b1.a += b1._invI * lam;
    if (b2) b2.a -= b2._invI * lam;
    return lam;
  }

  // Pull two joint ends together (pivots, welds, and the perpendicular part of sliders).
  // With a joint j, also records the force on side a as a vector (j._fx, j._fy) in newtons.
  function joinEnds(scene, ea, eb, h, j) {
    const A = bodyOf(scene, ea), B = bodyOf(scene, eb);
    const p1 = endWorld(scene, ea), p2 = endWorld(scene, eb);
    const dx = p2[0] - p1[0], dy = p2[1] - p1[1], d = Math.hypot(dx, dy);
    const lam = applyPos(A, A ? p1[0] - A.x : 0, A ? p1[1] - A.y : 0,
                         B, B ? p2[0] - B.x : 0, B ? p2[1] - B.y : 0, dx, dy, 0, h);
    if (j) { const k = d > 1e-12 ? lam / (d * h * h) : 0; j._fx = k * dx; j._fy = k * dy; }
    return lam;
  }

  // ---------- joints ----------

  // Offset of a world point from a body's centre (zero for the ground).
  const arm = (b, p) => (b ? [p[0] - b.x, p[1] - b.y] : [0, 0]);

  function jointLength(scene, j) {
    const p1 = endWorld(scene, j.a), p2 = endWorld(scene, j.b);
    return Math.hypot(p2[0] - p1[0], p2[1] - p1[1]);
  }

  // Hold two ends L apart. mode 'rod' works both ways; 'rope' only pulls when taut.
  // maxF caps the force (pistons): the correction per substep is limited to maxF·h².
  // Returns the force in newtons, + for pulling (tension), - for pushing.
  function distance(scene, j, L, h, mode, maxF) {
    const A = bodyOf(scene, j.a), B = bodyOf(scene, j.b);
    const p1 = endWorld(scene, j.a), p2 = endWorld(scene, j.b);
    const dx = p2[0] - p1[0], dy = p2[1] - p1[1], len = Math.hypot(dx, dy);
    if (len < 1e-9) return 0;
    const c = len - L;
    if (mode === 'rope' && c <= 0) return 0; // a slack rope does nothing
    const nx = dx / len, ny = dy / len, [r1x, r1y] = arm(A, p1), [r2x, r2y] = arm(B, p2);
    let w = 0;
    if (A) { const cr = r1x * ny - r1y * nx; w += A._invM + A._invI * cr * cr; }
    if (B) { const cr = r2x * ny - r2y * nx; w += B._invM + B._invI * cr * cr; }
    if (w <= 0) return 0;
    let lam = c / w;
    if (maxF != null) { const lim = maxF * h * h; lam = Math.max(-lim, Math.min(lim, lam)); }
    const px = lam * nx, py = lam * ny;
    if (A) { A.x += px * A._invM; A.y += py * A._invM; A.a += A._invI * (r1x * py - r1y * px); }
    if (B) { B.x -= px * B._invM; B.y -= py * B._invM; B.a -= B._invI * (r2x * py - r2y * px); }
    return lam / (h * h);
  }

  // Pulley geometry. The rope leaves the wheel at a tangent point on each side and wraps round
  // it, so its length is free part + free part + wrapped arc. j.wind (+1/-1) is which way round
  // the wheel the rope goes; it's fixed when the pulley is made, like a real rope's path.
  // Moving an end changes the length at a rate given by the unit vector along its free part (g1, g2).
  function pulleyGeom(scene, j) {
    const pA = endWorld(scene, j.a), pB = endWorld(scene, j.b), c = endWorld(scene, j.p), r = j.r;
    if (!j.wind) { // over the side of the wheel away from the ends (e.g. over the top for hanging loads)
      const cr = (pA[0] - c[0]) * (pB[1] - c[1]) - (pA[1] - c[1]) * (pB[0] - c[0]);
      j.wind = cr > 1e-9 ? -1 : 1;
    }
    const s = j.wind;
    const side = (e, leaving) => {
      const dx = e[0] - c[0], dy = e[1] - c[1], d = Math.hypot(dx, dy);
      if (d <= r * 1.0001) return { t: [c[0], c[1]], f: d, g: d > 1e-9 ? [dx / d, dy / d] : [0, 0] }; // inside the wheel
      const base = Math.atan2(dy, dx), al = Math.acos(r / d), f = Math.sqrt(d * d - r * r);
      const cands = [base + al, base - al].map((q) => [c[0] + r * Math.cos(q), c[1] + r * Math.sin(q)]);
      // the tangent where the rope runs on (or off) the wheel in the winding direction
      const t = cands.find((t) => {
        const vx = -(t[1] - c[1]) * s, vy = (t[0] - c[0]) * s;
        const ux = leaving ? e[0] - t[0] : t[0] - e[0], uy = leaving ? e[1] - t[1] : t[1] - e[1];
        return ux * vx + uy * vy > 0;
      }) || cands[0];
      return { t, f, g: [(e[0] - t[0]) / f, (e[1] - t[1]) / f] };
    };
    const A_ = side(pA, false), B_ = side(pB, true);
    const aA = Math.atan2(A_.t[1] - c[1], A_.t[0] - c[0]), aB = Math.atan2(B_.t[1] - c[1], B_.t[0] - c[0]);
    const n = (x) => ((x % (2 * Math.PI)) + 2 * Math.PI) % (2 * Math.PI);
    const wrap = n(s * (aB - aA));
    return { pA, pB, c, tA: A_.t, tB: B_.t, aA, wind: s, f1: A_.f, f2: B_.f, g1: A_.g, g2: B_.g, wrap, length: A_.f + B_.f + r * wrap };
  }

  // The rope can't get longer than j.length. The wheel can be fixed or ride on a body
  // (then it gets pulled on too, which is how a block and tackle works).
  function solvePulley(scene, j, h) {
    const A = bodyOf(scene, j.a), B = bodyOf(scene, j.b), Wh = bodyOf(scene, j.p);
    const G = pulleyGeom(scene, j);
    j._f = 0; j._d1 = G.f1; j._ja = [0, 0]; j._jb = [0, 0]; // _d1 turns the wheel's spokes in the drawing
    const c = G.length - j.length;
    if (c <= 0) return; // slack
    const parts = [[A, arm(A, G.pA), G.g1], [B, arm(B, G.pB), G.g2], [Wh, arm(Wh, G.c), [-G.g1[0] - G.g2[0], -G.g1[1] - G.g2[1]]]];
    let w = 0;
    for (const [b, r, g] of parts) {
      if (!b) continue;
      const cr = r[0] * g[1] - r[1] * g[0];
      w += b._invM * (g[0] * g[0] + g[1] * g[1]) + b._invI * cr * cr;
    }
    if (w <= 0) return;
    const lam = c / w;
    for (const [b, r, g] of parts) {
      if (!b) continue;
      b.x -= b._invM * lam * g[0]; b.y -= b._invM * lam * g[1];
      b.a -= b._invI * lam * (r[0] * g[1] - r[1] * g[0]);
    }
    j._f = lam / (h * h); // rope tension
    // a load hauled right up to the wheel jams against it instead of going round it
    const G2 = pulleyGeom(scene, j), fmin = 0.1;
    for (const [b, f, g, pE, rec] of [[A, G2.f1, G2.g1, G2.pA, j._ja], [B, G2.f2, G2.g2, G2.pB, j._jb]]) {
      if (!b || f >= fmin) continue;
      const lam = applyPos(b, pE[0] - b.x, pE[1] - b.y, Wh, Wh ? G2.c[0] - Wh.x : 0, Wh ? G2.c[1] - Wh.y : 0, g[0] * (fmin - f), g[1] * (fmin - f), 0, h);
      rec[0] = (lam * g[0]) / (h * h); rec[1] = (lam * g[1]) / (h * h); // the wheel pushing the jammed end away
    }
  }

  // A piston can't go past the ends of its stroke: those are solid metal, with no force limit.
  // (Its push and pull come from pistonDrive, as a real force.)
  function solvePiston(scene, j, h) {
    const len = jointLength(scene, j);
    j._stop = 0;
    if (len > j.max || len < j.min) j._stop = distance(scene, j, Math.min(j.max, Math.max(j.min, len)), h, 'rod');
  }

  // ---------- actuators: motors and pistons ----------

  // Each substep an actuator pushes with whatever force (or torque) brings it to its target
  // speed, but never more than its rating. It's applied like any real force, so the whole
  // machine feels it, and an overloaded actuator really stalls. "Whatever it takes" comes from a
  // PI controller: the P part reacts to the speed error, the I part removes any steady lag.
  // gain is the mass (or moment of inertia) being driven, so the response time is about T.
  // j.integ (the I part) is saved with the scene: it's the force the actuator is already
  // holding, so a loaded piston doesn't sag when you press Play or Reset.
  function drive(j, err, gain, max, h) {
    const T = 0.05, kp = 2 / T, ki = 1 / (T * T);
    const u = gain * (kp * err + ki * (j.integ || 0));
    const out = Math.max(-max, Math.min(max, u));
    if (out === u) j.integ = (j.integ || 0) + err * h; // don't wind up while maxed out
    return out;
  }

  // On keys (j.keyFwd / j.keyRev set), _cmd is the driver's command from -1 to 1 (the app eases
  // it in and out). Let go and a motor either brakes (holds still) or coasts (no torque at all).
  const onKeys = (j) => !!(j.keyFwd || j.keyRev);
  function motorDrive(scene, j, h) {
    const A = bodyOf(scene, j.a), B = bodyOf(scene, j.b);
    let rpm = j.motor;
    if (onKeys(j)) {
      const c = j._cmd || 0;
      if (Math.abs(c) < 1e-3 && j.idle === 'coast') { j._torque = 0; j.integ = 0; return; }
      rpm = Math.abs(j.motor) * c;
    }
    const omega = (rpm * 2 * Math.PI) / 60, rel = (B ? B.av : 0) - (A ? A.av : 0);
    const tau = drive(j, omega - rel, j._gain || 1, j.torque, h);
    if (A) A._t -= tau;
    if (B) B._t += tau;
    j._torque = tau;
  }

  // 'cycle' turns around at each end of the stroke; 'extend' and 'retract' drive one way, then
  // keep pressing against the end stop.
  function pistonDrive(scene, j, h) {
    const A = bodyOf(scene, j.a), B = bodyOf(scene, j.b);
    const p1 = endWorld(scene, j.a), p2 = endWorld(scene, j.b);
    const dx = p2[0] - p1[0], dy = p2[1] - p1[1], len = Math.hypot(dx, dy);
    if (len < 1e-9) return;
    if (j.mode === 'extend') j.phase = 1;
    else if (j.mode === 'retract') j.phase = -1;
    else if (j.mode === 'keys') { /* the driver decides: see dir below */ }
    else if (j.phase > 0 && len >= j.max - 1e-3) j.phase = -1; // turning round doesn't change the load it holds
    else if (j.phase < 0 && len <= j.min + 1e-3) j.phase = 1;
    const nx = dx / len, ny = dy / len, [r1x, r1y] = arm(A, p1), [r2x, r2y] = arm(B, p2);
    const v1 = A ? [A.vx - A.av * r1y, A.vy + A.av * r1x] : [0, 0], v2 = B ? [B.vx - B.av * r2y, B.vy + B.av * r2x] : [0, 0];
    const vlen = (v2[0] - v1[0]) * nx + (v2[1] - v1[1]) * ny;
    // end-of-stroke cushioning, like a real hydraulic cylinder: slow down over the last bit of
    // travel and speed back up over the first bit after turning round (never below 15%), so
    // turning round isn't a hammer blow
    // dir: +1 out, -1 in. On keys it's the driver's command, and 0 shuts the valve: it holds still.
    const dir = j.mode === 'keys' ? j._cmd || 0 : j.phase;
    const toEnd = dir > 0 ? j.max - len : len - j.min, fromEnd = dir > 0 ? len - j.min : j.max - len;
    const ramp = j.speed * 0.2, cushion = Math.max(0.15, Math.min(1, toEnd / ramp, fromEnd / ramp));
    const F = drive(j, dir * j.speed * cushion - vlen, j._gain || 1, j.force, h); // + pushes the ends apart
    const fx = F * nx, fy = F * ny;
    if (A) { A._fx -= fx; A._fy -= fy; A._t -= r1x * fy - r1y * fx; }
    if (B) { B._fx += fx; B._fy += fy; B._t += r2x * fy - r2y * fx; }
    j._push = F;
  }

  // What an actuator is driving: everything joined to its end b (not counting through the
  // actuator itself), as a mass for pistons or a moment of inertia about the pin for motors.
  function drivenLoad(scene, j) {
    const p = endWorld(scene, j.a);
    const side = (end) => {
      if (end.body == null) return Infinity;
      const seen = new Set([end.body]), queue = [end.body];
      let m = 0, I = 0, grounded = false;
      while (queue.length) {
        const b = scene._byId.get(queue.pop());
        if (!b) continue;
        if (b.fixed) { grounded = true; continue; }
        if (!b._red) { m += b.mass; I += b._I + b.mass * ((b.x - p[0]) ** 2 + (b.y - p[1]) ** 2); }
        for (const k of scene.joints) {
          if (k === j) continue;
          const ids = [k.a.body, k.b.body, k.p && k.p.body];
          if (!ids.includes(b.id)) continue;
          for (const o of ids) if (o != null && !seen.has(o)) { seen.add(o); queue.push(o); }
        }
      }
      return j.kind === 'piston' ? Math.max(m, 0.01) : Math.max(I, 1e-4);
    };
    const a = side(j.a), b = side(j.b);
    if (a === Infinity && b === Infinity) return 1;
    return a === Infinity ? b : b === Infinity ? a : (a * b) / (a + b); // two free sides share it
  }

  // Just the parts the actuator is pinned to. The force lands on these first, so if they're
  // very light the controller has to be gentler or it would kick them too hard in one substep.
  function directLoad(scene, j) {
    const p = endWorld(scene, j.a);
    const one = (end) => {
      const b = bodyOf(scene, end);
      if (!b || b.fixed) return Infinity;
      return j.kind === 'piston' ? b.mass : b._I + b.mass * ((b.x - p[0]) ** 2 + (b.y - p[1]) ** 2);
    };
    const a = one(j.a), b = one(j.b);
    if (a === Infinity && b === Infinity) return 1;
    return a === Infinity ? b : b === Infinity ? a : (a * b) / (a + b);
  }

  function solveSlider(scene, j, h) {
    const A = bodyOf(scene, j.a), B = bodyOf(scene, j.b);
    // Recomputed per correction, because each correction moves the bodies.
    const frame = () => {
      const pA = endWorld(scene, j.a), pB = endWorld(scene, j.b);
      const [ux, uy] = rotate(A ? A.a : 0, j.dir[0], j.dir[1]);
      const dx = pB[0] - pA[0], dy = pB[1] - pA[1];
      return { pA, pB, ux, uy, s: dx * ux + dy * uy, e: -dx * uy + dy * ux };
    };
    const offs = (q, pB) => [A ? q[0] - A.x : 0, A ? q[1] - A.y : 0, B ? pB[0] - B.x : 0, B ? pB[1] - B.y : 0];

    // 1. keep the carriage on the rail line
    let f = frame();
    let q = [f.pA[0] + f.s * f.ux, f.pA[1] + f.s * f.uy];
    let r = offs(q, f.pB);
    const lam = applyPos(A, r[0], r[1], B, r[2], r[3], -f.e * f.uy, f.e * f.ux, 0, h);
    j._f = lam / (h * h);
    // the force on side a as a vector: the rail pushes square to itself (sign follows the gap)
    const sg = f.e > 0 ? 1 : -1;
    j._fx = -sg * f.uy * j._f; j._fy = sg * f.ux * j._f; j._m = 0;

    // 2. end stops
    if (j.limits) {
      f = frame();
      const sc = Math.min(j.hi, Math.max(j.lo, f.s));
      if (sc !== f.s) {
        q = [f.pA[0] + sc * f.ux, f.pA[1] + sc * f.uy];
        r = offs(q, f.pB);
        const lamS = applyPos(A, r[0], r[1], B, r[2], r[3], (f.s - sc) * f.ux, (f.s - sc) * f.uy, 0, h);
        const sg = f.s > sc ? 1 : -1; // the end stop pushes along the rail
        j._fx += (sg * f.ux * lamS) / (h * h); j._fy += (sg * f.uy * lamS) / (h * h);
      }
    }
    // 3. no rotation relative to the rail (a "prismatic" joint) unless unlocked (pin in a slot)
    if (j.lock) j._m = applyAng(A, B, angleOf(scene, j.b) - angleOf(scene, j.a) - j.angle0, 0, h) / (h * h);
    j._s = frame().s;
  }

  // ---------- belts (CoreXY) ----------
  // A toothed belt from a motor pulley that moves a carriage along a gantry (slider jx) and the
  // gantry along its rails (slider jy). Its rule: cx·x + cy·y + r·θ stays constant, where x and y
  // are how far along each slider they are and θ is the pulley's angle. A CoreXY machine has two:
  // x + y on one belt, x − y on the other. It's a position constraint like a pin, so the belt
  // carries real force and the motor has to work to move the head.
  const sliderPos = (scene, sj) => {
    const A = bodyOf(scene, sj.a), pA = endWorld(scene, sj.a), pB = endWorld(scene, sj.b), u = rotate(A ? A.a : 0, sj.dir[0], sj.dir[1]);
    return { s: (pB[0] - pA[0]) * u[0] + (pB[1] - pA[1]) * u[1], u };
  };
  function beltParts(scene, j) {
    const jx = scene.joints.find((k) => k.id === j.sx), jy = scene.joints.find((k) => k.id === j.sy);
    if (!jx || !jy) return null;
    const pul = bodyOf(scene, j.a), car = bodyOf(scene, jx.b), gan = bodyOf(scene, jy.b);
    return pul && car && gan ? { jx, jy, pul, car, gan } : null;
  }
  function solveBelt(scene, j, h) {
    const q = beltParts(scene, j);
    j._f = 0;
    if (!q) return;
    const { jx, jy, pul, car, gan } = q, X = sliderPos(scene, jx), Y = sliderPos(scene, jy);
    const C = j.cx * X.s + j.cy * Y.s + j.r * pul.a - j.c0;
    // moving along y moves the gantry and the carriage riding on it together
    const wx = car._invM, wy = gan.fixed || car.fixed ? 0 : 1 / (gan.mass + car.mass), wt = pul._invI;
    const w = j.cx * j.cx * wx + j.cy * j.cy * wy + j.r * j.r * wt;
    if (w <= 0) return;
    const lam = -C / w, dx = j.cx * lam * wx, dy = j.cy * lam * wy;
    car.x += dx * X.u[0]; car.y += dx * X.u[1];
    gan.x += dy * Y.u[0]; gan.y += dy * Y.u[1]; car.x += dy * Y.u[0]; car.y += dy * Y.u[1];
    pul.a += j.r * lam * wt;
    j._f = lam / (h * h); // the belt's pull, in newtons
  }

  function solveJoint(scene, j, h, posing) {
    if (j._skip) { j._f = Math.abs(j._skip._f || 0); return; }
    if (j._ideal) { // a massless rod: one distance constraint between the two things it joins
      const b = j._ideal;
      b._f = distance(scene, { a: b._red.farA, b: b._red.farB }, b.L, h, 'rod');
      j._f = Math.abs(b._f);
      return;
    }
    if (j.kind === 'pivot') {
      j._f = joinEnds(scene, j.a, j.b, h, j) / (h * h);
    } else if (j.kind === 'weld') {
      j._f = joinEnds(scene, j.a, j.b, h, j) / (h * h);
      // _m: the moment the glue puts on side a (+ turns it clockwise on screen, like angles here)
      j._m = applyAng(bodyOf(scene, j.a), bodyOf(scene, j.b), angleOf(scene, j.b) - angleOf(scene, j.a) - j.angle0, 0, h) / (h * h);
    } else if (j.kind === 'rod') {
      j._f = distance(scene, j, j.length, h, 'rod');
    } else if (j.kind === 'rope') {
      j._f = distance(scene, j, j.length, h, 'rope');
    } else if (j.kind === 'pulley') {
      solvePulley(scene, j, h);
    } else if (j.kind === 'piston') {
      solvePiston(scene, j, h);
    } else if (j.kind === 'slider') {
      solveSlider(scene, j, h);
    } else if (j.kind === 'belt') {
      solveBelt(scene, j, h);
    }
  }

  // Springs push and pull with Hooke's law: F = k × stretch (+ means pulling the ends together).
  function springForce(scene, j) {
    const A = bodyOf(scene, j.a), B = bodyOf(scene, j.b);
    const p1 = endWorld(scene, j.a), p2 = endWorld(scene, j.b);
    const dx = p2[0] - p1[0], dy = p2[1] - p1[1], len = Math.hypot(dx, dy);
    j._f = 0;
    if (len < 1e-9) return;
    const F = j.k * (len - j.rest), fx = (F * dx) / len, fy = (F * dy) / len;
    const [r1x, r1y] = arm(A, p1), [r2x, r2y] = arm(B, p2);
    if (A) { A._fx += fx; A._fy += fy; A._t += r1x * fy - r1y * fx; }
    if (B) { B._fx -= fx; B._fy -= fy; B._t -= r2x * fy - r2y * fx; }
    j._f = F;
  }

  // Viscous damping along direction n between two points: F = c × (how fast they separate).
  // Applied implicitly, as an impulse that can only slow things down (never overshoot), so a
  // very strong damper stays stable. Returns the damping force in N (+ resists separating).
  function dampAlong(A, r1, B, r2, n, c, h) {
    const vel = (b, r) => (b ? [b.vx - b.av * r[1], b.vy + b.av * r[0]] : [0, 0]);
    const v1 = vel(A, r1), v2 = vel(B, r2);
    const vrel = (v2[0] - v1[0]) * n[0] + (v2[1] - v1[1]) * n[1];
    let w = 0;
    if (A) { const cr = r1[0] * n[1] - r1[1] * n[0]; w += A._invM + A._invI * cr * cr; }
    if (B) { const cr = r2[0] * n[1] - r2[1] * n[0]; w += B._invM + B._invI * cr * cr; }
    if (w <= 0) return 0;
    const P = (c * h * vrel) / (1 + c * h * w), px = P * n[0], py = P * n[1];
    if (A) { A.vx += px * A._invM; A.vy += py * A._invM; A.av += A._invI * (r1[0] * py - r1[1] * px); }
    if (B) { B.vx -= px * B._invM; B.vy -= py * B._invM; B.av -= B._invI * (r2[0] * py - r2[1] * px); }
    return P / h;
  }

  // Hydraulic cushions: in the last 2 cm of its stroke a piston squeezes oil through a small
  // hole, which brakes it hard but smoothly instead of letting it slam into the end.
  function cushionImpulse(scene, j, h) {
    j._cush = 0;
    const p1 = endWorld(scene, j.a), p2 = endWorld(scene, j.b);
    const dx = p2[0] - p1[0], dy = p2[1] - p1[1], len = Math.hypot(dx, dy), zone = 0.02;
    if (len < 1e-9) return;
    const toMin = len - j.min, toMax = j.max - len;
    if (Math.min(toMin, toMax) > zone) return;
    const A = bodyOf(scene, j.a), B = bodyOf(scene, j.b), n = [dx / len, dy / len], r1 = arm(A, p1), r2 = arm(B, p2);
    const vel = (b, r) => (b ? [b.vx - b.av * r[1], b.vy + b.av * r[0]] : [0, 0]);
    const v1 = vel(A, r1), v2 = vel(B, r2), vlen = (v2[0] - v1[0]) * n[0] + (v2[1] - v1[1]) * n[1];
    const closing = (toMin <= zone && vlen < 0) || (toMax <= zone && vlen > 0);
    if (!closing) return;
    const depth = 1 - Math.max(0, Math.min(toMin <= zone ? toMin : toMax, zone)) / zone; // 0 → 1 into the zone
    j._cush = dampAlong(A, r1, B, r2, n, (j.force / 0.05) * depth, h);
  }

  function damperImpulse(scene, j, h) {
    const A = bodyOf(scene, j.a), B = bodyOf(scene, j.b);
    const p1 = endWorld(scene, j.a), p2 = endWorld(scene, j.b);
    const dx = p2[0] - p1[0], dy = p2[1] - p1[1], len = Math.hypot(dx, dy);
    if (len < 1e-9) return;
    const F = dampAlong(A, arm(A, p1), B, arm(B, p2), [dx / len, dy / len], j.c, h);
    j._f = (j.kind === 'spring' ? j._f : 0) + F;
  }

  // Applied loads, as in a statics problem: a point force P (newtons, at angle `angle` in degrees
  // measured anticlockwise from the +x axis, textbook style, so -90 is straight down) and a
  // couple M (N·m, + anticlockwise). follow = the load turns with the part it's on.
  function loadVector(scene, j) {
    const b = bodyOf(scene, j.a), th = (j.angle * Math.PI) / 180 - (j.follow && b ? b.a : 0);
    return [j.F * Math.cos(th), -j.F * Math.sin(th)]; // world y points down
  }

  function applyLoad(scene, j) {
    const b = bodyOf(scene, j.a);
    if (!b) return;
    if (j.kind === 'couple') { b._t -= j.M; return; } // anticlockwise on screen is -angle here
    const p = endWorld(scene, j.a), [fx, fy] = loadVector(scene, j);
    b._fx += fx; b._fy += fy; b._t += (p[0] - b.x) * fy - (p[1] - b.y) * fx;
  }

  // The force (world vector f at world point p) and couple (mz, anticlockwise +) a joint puts on
  // each thing it touches, from the values its solver recorded. body is null for the ground.
  // Always the same number of ends for a joint kind, so they can be averaged over a frame.
  function endForces(scene, j) {
    const out = [];
    const end = (e, f, mz, kind, p) => out.push({ body: e.body ?? null, p: p || endWorld(scene, e), f, mz: mz || 0, kind });
    if (j._ideal || j._skip) { // a pin on a massless rod: the rod pulls straight along itself
      const b = j._ideal || j._skip, F = b._f || 0;
      const mine = j.a.body === b.id ? j.a : j.b, far = mine === j.a ? j.b : j.a;
      const other = far === b._red.farA ? b._red.farB : b._red.farA;
      const pf = endWorld(scene, far), po = endWorld(scene, other), L = Math.hypot(po[0] - pf[0], po[1] - pf[1]) || 1;
      const u = [(po[0] - pf[0]) / L, (po[1] - pf[1]) / L];
      end(far, [F * u[0], F * u[1]], 0, 'pin', pf);
      end(mine, [-F * u[0], -F * u[1]], 0, 'pin', pf);
    } else if (j.kind === 'pivot' || j.kind === 'weld') {
      // _m is the glue's moment on side a in screen sense; a motor puts -τ on a and +τ on b
      const f = [j._fx || 0, j._fy || 0], m = j.kind === 'weld' ? j._m || 0 : 0, tau = j.motor ? j._torque || 0 : 0;
      const kind = j.motor ? 'motor' : j.kind;
      end(j.a, f, -m + tau, kind);
      end(j.b, [-f[0], -f[1]], m - tau, kind);
    } else if (j.kind === 'slider') {
      const f = [j._fx || 0, j._fy || 0], m = j.lock ? j._m || 0 : 0, p = endWorld(scene, j.b);
      end(j.a, f, -m, 'slider', p);
      end(j.b, [-f[0], -f[1]], m, 'slider', p);
    } else if (j.kind === 'spring' || j.kind === 'damper' || j.kind === 'rope' || j.kind === 'piston' || j.kind === 'rod') {
      const pa = endWorld(scene, j.a), pb = endWorld(scene, j.b), L = Math.hypot(pb[0] - pa[0], pb[1] - pa[1]);
      const F = L > 1e-9 ? j._f || 0 : 0, n = L > 1e-9 ? [(pb[0] - pa[0]) / L, (pb[1] - pa[1]) / L] : [0, 0]; // F > 0 pulls the ends together
      end(j.a, [F * n[0], F * n[1]], 0, j.kind, pa);
      end(j.b, [-F * n[0], -F * n[1]], 0, j.kind, pb);
    } else if (j.kind === 'pulley') {
      const G = pulleyGeom(scene, j), T = j._f || 0, ja = j._ja || [0, 0], jb = j._jb || [0, 0];
      end(j.a, [-T * G.g1[0] + ja[0], -T * G.g1[1] + ja[1]], 0, 'pulley', G.pA);
      end(j.b, [-T * G.g2[0] + jb[0], -T * G.g2[1] + jb[1]], 0, 'pulley', G.pB);
      end(j.p, [T * (G.g1[0] + G.g2[0]) - ja[0] - jb[0], T * (G.g1[1] + G.g2[1]) - ja[1] - jb[1]], 0, 'pulley', G.c);
    } else if (j.kind === 'belt') { // the pulley feels a torque, the head and gantry a pull along their rails
      const q = beltParts(scene, j), F = j._f || 0;
      if (q) {
        const X = sliderPos(scene, q.jx), Y = sliderPos(scene, q.jy), share = q.car.mass / (q.gan.mass + q.car.mass);
        const fx = j.cx * F, fy = j.cy * F;
        end(j.a, [0, 0], -j.r * F, 'belt', [q.pul.x, q.pul.y]);
        end({ body: q.car.id }, [fx * X.u[0] + share * fy * Y.u[0], fx * X.u[1] + share * fy * Y.u[1]], 0, 'belt', [q.car.x, q.car.y]);
        end({ body: q.gan.id }, [(1 - share) * fy * Y.u[0], (1 - share) * fy * Y.u[1]], 0, 'belt', [q.gan.x, q.gan.y]);
      }
    } else if (j.kind === 'load') {
      end(j.a, loadVector(scene, j), 0, 'load');
    } else if (j.kind === 'couple') {
      end(j.a, [0, 0], j.M, 'couple');
    }
    return out;
  }

  // The mouse "hand" while running: a spring from the grabbed point to the pointer, plus
  // damping so it doesn't wobble. k and c are set when you grab (scaled to what you hold).
  function handForce(scene, d) {
    const b = scene._byId.get(d.body);
    d._F = 0;
    if (!b || b.fixed) return;
    const p = toWorld(b, d.x, d.y), fx = d.k * (d.tx - p[0]), fy = d.k * (d.ty - p[1]);
    b._fx += fx; b._fy += fy; b._t += (p[0] - b.x) * fy - (p[1] - b.y) * fx;
    d._F = Math.hypot(fx, fy); d._fx = fx; d._fy = fy;
  }

  function handDamping(scene, d, h) {
    const b = scene._byId.get(d.body);
    if (!b || b.fixed) return;
    const r = arm(b, toWorld(b, d.x, d.y));
    // the damping is part of the hand's force on the body too (dampAlong returns + = resisting)
    d._fx = (d._fx || 0) - dampAlong(null, [0, 0], b, r, [1, 0], d.c, h);
    d._fy = (d._fy || 0) - dampAlong(null, [0, 0], b, r, [0, 1], d.c, h);
  }

  // Total mass of everything joined to a body (not counting the ground), so the hand can be
  // strong enough for whatever it's attached to.
  function clusterMass(scene, id) {
    const seen = new Set([id]), queue = [id];
    let m = 0;
    while (queue.length) {
      const cur = queue.pop(), b = scene.bodies.find((x) => x.id === cur);
      if (b && !b.fixed && !b._red) m += b.mass; // massless rods weigh nothing but still connect
      for (const j of scene.joints) {
        const ids = [j.a.body, j.b.body, j.p && j.p.body];
        if (!ids.includes(cur)) continue;
        for (const o of ids) if (o != null && !seen.has(o)) { seen.add(o); queue.push(o); }
      }
    }
    return Math.max(m, 0.05);
  }

  // ---------- stepping ----------

  // Add this substep's joint forces into the frame averages (see step).
  function accumulate(scene, ends, hand, hacc) {
    for (const j of scene.joints) {
      const ac = j._acc; // written out: a keyed loop here costs as much as the joints
      ac[0] += j._f || 0; ac[1] += j._fx || 0; ac[2] += j._fy || 0; ac[3] += j._m || 0; ac[4] += j._torque || 0;
      if (!ends) continue;
      const ef = endForces(scene, j);
      if (!j._ae) j._ae = ef.map((e) => ({ body: e.body, kind: e.kind, f: [0, 0], mz: 0, mc: 0, p: e.p }));
      ef.forEach((e, i) => {
        const a = j._ae[i], b = e.body != null ? scene._byId.get(e.body) : null;
        a.f[0] += e.f[0]; a.f[1] += e.f[1]; a.mz += e.mz; a.p = e.p;
        if (b) a.mc += (e.p[0] - b.x) * e.f[1] - (e.p[1] - b.y) * e.f[0];
      });
  }
  for (const b of scene.bodies) if (b._red) b._facc += b._f || 0;
  if (hand) {
    const hb = scene._byId.get(hand.body);
    hacc[0] += hand._fx || 0; hacc[1] += hand._fy || 0;
    if (hb) { const hp = toWorld(hb, hand.x, hand.y); hacc[2] += (hp[0] - hb.x) * (hand._fy || 0) - (hp[1] - hb.y) * (hand._fx || 0); }
  }
  }


  function step(scene, dt, hand) {
    index(scene);
    // settings.substeps is per 1/60 s, so a 144 Hz screen doesn't do more work than it needs to
    const S = scene.settings, n = Math.max(1, Math.round(S.substeps * dt * 60)), h = dt / n;
    const damp = Math.exp(-S.air * h);
    for (const j of scene.joints) {
      // The controller works as hard as the parts it's pinned to can safely take in one substep.
      // Whatever else hangs off them only adds inertia (a scissor's leverage can make a 30 kg
      // load feel like tonnes at the piston), so this stays stable and keeps the speed on target.
      if (j.kind === 'piston' || (j.kind === 'pivot' && j.motor)) j._gain = (directLoad(scene, j) * 0.015) / h;
    }
    // Forces change from substep to substep, so for read-outs each one is also averaged over the
    // whole frame (j._avg). Those averages balance the frame's change in speed exactly, which is
    // what lets the free-body diagrams check ΣF = m·a.
    // Per joint end: force, couple, and its moment about that body's centre (mc), since the
    // direction and lever arm both turn during the frame. Reading every kind of joint every
    // substep makes the solver noticeably slower, so it's only done while something shows the
    // results (scene._wantEnds: a free-body diagram, the statics view, a part's force card).
    const ends = !!scene._wantEnds;
    for (const j of scene.joints) { j._acc = [0, 0, 0, 0, 0]; j._ae = null; if (!ends) j._avgEnds = null; }
    for (const b of scene.bodies) { b._facc = 0; b._v0 = [b.vx, b.vy, b.av]; b._p0 = [b.x, b.y, b.a]; }
    const hacc = [0, 0, 0];
    // Winches change a rope's length a little every substep. All at once, a frame's worth of
    // reeling would be closed in one substep, and that turns into a violent jerk.
    const winches = scene.joints.filter((j) => j._lenRate);
    for (let s = 0; s < n; s++) {
      for (const j of winches) j.length = Math.min(j.winchMax ?? Infinity, Math.max(j.winchMin ?? 0.05, j.length + j._lenRate * h));
      // 1. forces: weight (which can be negative: it floats up), springs, motors, pistons, the hand
      for (const b of scene.bodies) { b._fx = 0; b._fy = b.mass * S.gravity * (b.gscale ?? 1); b._t = 0; }
      for (const j of scene.joints) {
        if (j.kind === 'spring') springForce(scene, j);
        else if (j.kind === 'piston') pistonDrive(scene, j, h);
        else if (j.kind === 'pivot' && j.motor) motorDrive(scene, j, h);
        else if (j.kind === 'load' || j.kind === 'couple') applyLoad(scene, j);
      }
      if (hand) handForce(scene, hand);
      // 2. forces change velocities
      for (const b of scene.bodies) {
        b._px = b.x; b._py = b.y; b._pa = b.a;
        if (b.fixed) { b.vx = b.vy = b.av = 0; continue; }
        if (b._red) continue; // massless rods just follow what they join
        b.vx = (b.vx + h * b._fx * b._invM) * damp;
        b.vy = (b.vy + h * b._fy * b._invM) * damp;
        b.av = (b.av + h * b._t * b._invI) * damp;
      }
      // 3. dampers (and spring damping) take speed out
      for (const j of scene.joints) {
        if ((j.kind === 'damper' || j.kind === 'spring') && j.c > 0) damperImpulse(scene, j, h);
        else if (j.kind === 'piston') cushionImpulse(scene, j, h);
      }
      if (hand) handDamping(scene, hand, h);
      // 4. move, then let the joints put things right
      for (const b of scene.bodies) if (!b.fixed && !b._red) { b.x += h * b.vx; b.y += h * b.vy; b.a += h * b.av; }
      for (const j of scene.joints) solveJoint(scene, j, h, false);
      for (const [j, times] of scene._light) { // extra passes for light rods on heavy parts
        for (let k = 0; k < times; k++) {
          const f = j._f || 0, fx = j._fx || 0, fy = j._fy || 0, m = j._m || 0;
          solveJoint(scene, j, h, false);
          j._f += f; j._fx += fx; j._fy += fy;
          if (j.kind === 'weld') j._m += m; // (pins never get _m: keeping each kind's fields fixed keeps the solver fast)
        }
      }
      poseIdeal(scene);
      for (const j of scene.joints) if (j.kind === 'piston') j._f = -(j._push || 0) + j._stop + (j._cush || 0); // + pull, - push
      accumulate(scene, ends, hand, hacc);
      // 5. the velocity is whatever movement actually happened
      for (const b of scene.bodies) {
        if (b.fixed) continue;
        b.vx = (b.x - b._px) / h; b.vy = (b.y - b._py) / h; b.av = (b.a - b._pa) / h;
      }
    }
    for (const j of scene.joints) {
      j._avg = { f: j._acc[0] / n, fx: j._acc[1] / n, fy: j._acc[2] / n, m: j._acc[3] / n, torque: j._acc[4] / n };
      if (ends) j._avgEnds = !j._ae ? null : j._ae.map((a) => ({ body: a.body, kind: a.kind, p: a.p, f: [a.f[0] / n, a.f[1] / n], mz: a.mz / n, mc: a.mc / n }));
    }
    for (const b of scene.bodies) {
      b._favg = b._red ? b._facc / n : 0;
      // acceleration over the frame (world axes; _aa is the angular one, + clockwise on screen)
      b._ax = (b.vx - b._v0[0]) / dt; b._ay = (b.vy - b._v0[1]) / dt; b._aa = (b.av - b._v0[2]) / dt;
      // velocity over the whole frame: steadier for read-outs than the last substep's
      b._vf = [(b.x - b._p0[0]) / dt, (b.y - b._p0[1]) / dt, (b.a - b._p0[2]) / dt];
    }
    if (hand) { hand._avg = ends ? [hacc[0] / n, hacc[1] / n] : null; hand._avgMc = ends ? hacc[2] / n : null; }
    scene._dt = dt;
    scene.time += dt;
  }

  // Edit-mode posing: drag one body and let the joints drag everything else along, with no
  // forces or time involved (like inverse kinematics). Springs just stretch. Velocities reset.
  function pose(scene, drag, iters = 30) {
    index(scene);
    const held = drag ? scene._byId.get(drag.body) : null;
    if (held) { held._invM *= 0.02; held._invI *= 0.02; } // the held body wins arguments
    for (let it = 0; it < iters; it++) {
      if (held) { // fixed bodies can be moved by hand in edit mode too
        const p = toWorld(held, drag.x, drag.y);
        held.x += drag.tx - p[0]; held.y += drag.ty - p[1];
      }
      for (const j of scene.joints) solveJoint(scene, j, 1, true);
      poseIdeal(scene);
    }
    for (const b of scene.bodies) { b.vx = b.vy = b.av = 0; }
    for (const j of scene.joints) { j._f = 0; j._torque = 0; } // posing forces aren't real forces
    index(scene);
  }

  // Energy bookkeeping: handy for checking the sim (with no damping the total should stay flat).
  function energy(scene) {
    index(scene);
    const g = scene.settings.gravity;
    let ke = 0, pe = 0;
    for (const b of scene.bodies) {
      if (b.fixed || b._red) continue;
      ke += 0.5 * b.mass * (b.vx * b.vx + b.vy * b.vy) + 0.5 * b._I * b.av * b.av;
      pe += -b.mass * g * (b.gscale ?? 1) * b.y; // y is down, so height is -y
    }
    for (const j of scene.joints) {
      if (j.kind !== 'spring') continue;
      const ext = jointLength(scene, j) - j.rest;
      pe += 0.5 * j.k * ext * ext;
    }
    return { ke, pe, total: ke + pe };
  }

  ML.Phys = {
    polyArea, polyCentroid, shapeArea, innerSize, updateMass, index,
    toWorld, toLocal, rotate, discAnchors, bodyOf, angleOf, endWorld, hitBody, jointLength, pulleyGeom, barAxial, poseIdeal,
    step, pose, energy, clusterMass, loadVector, endForces, onKeys,
  };
})();
