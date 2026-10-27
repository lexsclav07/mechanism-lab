// editor.js: the build tools. Mouse, pen and touch all go through pointer events.
// Each tool reacts to down / move / up; the gesture in progress lives in `op`.
(function () {
  const ML = (window.ML = window.ML || {});
  const P = ML.Phys, R = ML.Render, M = ML.Model, V = R.View, Rd = ML.Rods;
  let A, canvas;               // ML.App and the canvas, set in init()
  let op = null;               // gesture in progress
  let poly = [];               // corners placed so far with the Shape tool
  let pul = null;              // pulley being placed: { wheel, a } so far
  let lastClick = null;        // for spotting a double-click with the Shape tool
  let cursor = null;           // last pointer position (screen px)
  const pointers = new Map();  // active touches, for two-finger pan and zoom
  let pinch = null;

  const HINTS = {
    select: 'Grab any point and drag it. Paused, you set up the starting pose; running, you pull on it (the pull shows in newtons). Grab a hatched anchor to move it, even mid-swing.',
    camera: 'Tap a point on a part and the camera follows that exact point. Tap empty space to stop following.',
    box: 'Drag on the board to draw a block, or tap for a standard one.',
    poly: 'Tap to place corners, then tap the first corner again (or double-tap) to finish the shape.',
    weight: 'Tap to drop a 1 kg weight. Drop it on something and it sticks to it.',
    circle: 'Tap for a 0.5 m circle, or drag out its size. It has 16 anchor points round its rim (one every 1/16 turn) that pins, rods, springs and ropes snap to.',
    pivot: 'Tap where parts overlap (rods crossing, rod ends meeting) to pin them together. Tap a single part to pin it to the ground.',
    motor: 'Tap where two parts overlap (or on one part) to add a motor there. Set its speed and torque in its card.',
    weld: 'Tap where two parts overlap to glue them into one piece.',
    rod: 'Drag from point to point. Rods join other rods end to end, or anywhere along them. Next to a block or weight, an end in empty space is pinned to the ground; next to a rod it stays free.',
    rope: 'Drag from one point to another for a rope. It only pulls when it’s tight. Make it longer in its card to give it slack.',
    pulley: 'Tap where the wheel goes, then the first thing to tie on, then the second.',
    spring: 'Drag from one point to another. The spring starts at its relaxed length. An end in empty space hooks to a wall.',
    damper: 'Drag from one point to another for a damper. It resists motion along its length (it doesn’t spring back).',
    piston: 'Drag from one point to another for a piston. It pushes in and out on its own. Set speed, force and stroke in its card.',
    slider: 'Press on a block, then drag the way it should slide. On a rod it’s a roller support (free to tilt).',
    load: 'Press on a part where the force acts and drag the way it pushes; the longer the drag, the bigger the force. Tap for 100 N straight down.',
    couple: 'Tap a part to put a turning moment (a couple) on it: 20 N·m anticlockwise. Set its size and direction in its card.',
    erase: 'Tap anything to remove it.',
  };
  // Supports for statics problems, for the Pin and Glue hints: Pin on one part = pinned
  // support, Glue on one part = fixed support, Track on a rod = roller.

  const dist = (a, b) => Math.hypot(a[0] - b[0], a[1] - b[1]);
  const roundMass = (m) => Math.max(0.1, Math.round(m * 10) / 10);
  const TWO_POINT = new Set(['rod', 'rope', 'spring', 'damper', 'piston']);

  // ---------- finding things under the pointer ----------

  const bodiesAt = (p, exceptId) => Rd.partsAt(p, exceptId); // rods within 7 px, other parts within 3 px

  function segDist(p, a, b) {
    const dx = b[0] - a[0], dy = b[1] - a[1], L2 = dx * dx + dy * dy || 1;
    const t = Math.max(0, Math.min(1, ((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / L2));
    return Math.hypot(p[0] - a[0] - t * dx, p[1] - a[1] - t * dy);
  }

  function railEnds(scene, j) {
    const A_ = P.bodyOf(scene, j.a), pA = P.endWorld(scene, j.a), [ux, uy] = P.rotate(A_ ? A_.a : 0, j.dir[0], j.dir[1]);
    return [[pA[0] + ux * j.lo, pA[1] + uy * j.lo], [pA[0] + ux * j.hi, pA[1] + uy * j.hi]];
  }

  // Everything under the pointer, in order: pins, pulleys, ropes / springs / dampers / pistons,
  // rods, other parts (topmost first), then slider rails (a rail runs right through its block,
  // so it mustn't steal the block's clicks).
  function under(sx, sy) {
    const scene = A.S.scene, s = [sx, sy], out = [];
    P.index(scene);
    const nearest = (list, d) => list.map((x) => [x, d(x)]).filter(([, v]) => v != null).sort((p, q) => p[1] - q[1]).map(([x]) => x);
    // a pin hidden in the middle of a black weight comes after the parts (tap again to reach it)
    const hidden = (j) => [j.a, j.b].some((e) => { const b = P.bodyOf(scene, e); return b && b.kind === 'disc' && b.color === 'ink' && Math.hypot(e.x, e.y) < b.r * 0.5; });
    const pinHit = nearest(scene.joints.filter((j) => j.kind === 'pivot' || j.kind === 'weld'), (j) => { const d = dist(s, R.ws(P.endWorld(scene, j.a))); return d < 9 ? d : null; }).slice(0, 1);
    out.push(...pinHit.filter((j) => !hidden(j)));
    out.push(...scene.joints.filter((j) => j.kind === 'pulley' && dist(s, R.ws(P.endWorld(scene, j.p))) < j.r * V.ppm() + 6));
    out.push(...nearest(scene.joints.filter((j) => j.kind === 'load' || j.kind === 'couple'), (j) => {
      const q = R.ws(P.endWorld(scene, j.a));
      if (j.kind === 'couple') { const d = Math.abs(dist(s, q) - 16); return d < 7 ? d : null; }
      const [fx, fy] = P.loadVector(scene, j), F = Math.hypot(fx, fy) || 1, t = [q[0] - (fx / F) * 46, q[1] - (fy / F) * 46];
      const d = segDist(s, q, t); return d < 7 ? d : null;
    }).slice(0, 1));
    out.push(...nearest(scene.joints, (j) => {
      const segs = TWO_POINT.has(j.kind) ? [[j.a, j.b]] : j.kind === 'pulley' ? [[j.a, j.p], [j.p, j.b]] : [];
      const d = Math.min(Infinity, ...segs.map(([e1, e2]) => segDist(s, R.ws(P.endWorld(scene, e1)), R.ws(P.endWorld(scene, e2)))));
      return d < 7 ? d : null;
    }).slice(0, 1));
    const parts = bodiesAt(V.s2w(sx, sy));
    // rods before other parts (they're thin and lie on top), except a weight you tap inside of
    const w = V.s2w(sx, sy), inWeight = parts.filter((b) => b.kind === 'disc' && b.color === 'ink' && Math.hypot(w[0] - b.x, w[1] - b.y) < b.r);
    out.push(...inWeight, ...parts.filter((b) => b.kind === 'bar'), ...parts.filter((b) => b.kind !== 'bar'));
    out.push(...pinHit.filter(hidden));
    out.push(...scene.joints.filter((j) => { if (j.kind !== 'slider') return false; const [e0, e1] = railEnds(scene, j); return segDist(s, R.ws(e0), R.ws(e1)) < 6; }));
    return out.filter((x, i) => out.indexOf(x) === i);
  }

  let lastTap = null;
  function pick(sx, sy, cycle = false) {
    const list = under(sx, sy);
    if (!cycle) return list[0] || null;
    const now = performance.now(), again = lastTap && now - lastTap.t < 1000 && dist(lastTap.s, [sx, sy]) < 6;
    const i = again && list.length ? (lastTap.i + 1) % list.length : 0;
    lastTap = { t: now, s: [sx, sy], i };
    return list[i] || null;
  }

  // Ground anchors under the pointer: every joint end pinned to the ground at that spot
  // (so several joints sharing one anchor move together).
  function anchorsAt(sx, sy) {
    const scene = A.S.scene, out = [];
    let best = 11, at = null;
    for (const j of scene.joints) {
      for (const k of ['a', 'b', 'p']) {
        const e = j[k];
        if (!e || e.body != null) continue;
        const d = dist(R.ws([e.x, e.y]), [sx, sy]);
        if (d < best) { best = d; at = [e.x, e.y]; }
      }
    }
    if (!at) return null;
    for (const j of scene.joints) for (const k of ['a', 'b', 'p']) {
      const e = j[k];
      if (e && e.body == null && Math.hypot(e.x - at[0], e.y - at[1]) < 1e-6) out.push(e);
    }
    return out;
  }

  // Snap to useful points: centres, corners, edge midpoints and existing joints.
  function snap(sx, sy) {
    const scene = A.S.scene;
    let best = null, bd = 10;
    const consider = (p) => { const d = dist(R.ws(p), [sx, sy]); if (d < bd) { bd = d; best = p; } };
    const ring = [[-1, -1], [0, -1], [1, -1], [1, 0], [1, 1], [0, 1], [-1, 1], [-1, 0]];
    for (const b of scene.bodies) {
      consider([b.x, b.y]);
      for (const q of P.discAnchors(b)) consider(q); // a circle's anchor points
      if (b.kind === 'box') {
        for (const [i, k] of ring) consider(P.toWorld(b, (i * b.w) / 2, (k * b.h) / 2));
        if (b.hollow) { const [wi, hi] = P.innerSize(b); for (const [i, k] of ring) consider(P.toWorld(b, (i * wi) / 2, (k * hi) / 2)); }
      } else if (b.kind === 'poly') for (const [lx, ly] of b.pts) consider(P.toWorld(b, lx, ly));
    }
    for (const j of scene.joints) for (const [, p] of R.jointHandles(scene, j)) consider(p);
    for (const p of Rd.snapPoints(['rod', 'pivot', 'motor', 'weld'].includes(A.S.tool))) consider(p);
    if (best) return { p: best, snapped: true };
    const w = V.s2w(sx, sy);
    if (A.S.opts.snapGrid) return { p: [Math.round(w[0] * 10) / 10, Math.round(w[1] * 10) / 10], snapped: true };
    return { p: w, snapped: false };
  }

  // A joint end at world point p: on the topmost body there (except one), or on the ground.
  // On a rod the point is moved onto its centre line.
  function endAt(p, exceptId) { return endOn(bodiesAt(p, exceptId)[0], p); }
  const endOn = (b, p) => Rd.endOn(b, p);

  // ---------- previews (drawn in the accent colour on top of the scene) ----------

  function previewText(ctx, text, s) {
    ctx.font = '12px "IBM Plex Mono", ui-monospace, monospace';
    ctx.textAlign = 'left'; ctx.textBaseline = 'top';
    ctx.lineWidth = 4; ctx.strokeStyle = R.colors().paper; ctx.strokeText(text, s[0] + 12, s[1] + 12);
    ctx.fillStyle = R.colors().accent; ctx.fillText(text, s[0] + 12, s[1] + 12);
  }

  function setPreview(fn) { A.S.preview = fn; }

  function line(ctx, a, b, width, dash) {
    ctx.save(); if (dash) ctx.setLineDash(dash);
    ctx.beginPath(); ctx.moveTo(...a); ctx.lineTo(...b);
    ctx.lineWidth = width; ctx.lineCap = 'round'; ctx.strokeStyle = R.colors().accent; ctx.stroke(); ctx.restore();
  }

  // ---------- tools ----------

  // Grabbing a body while it runs: the hand is a spring sized to everything joined to it.
  function grab(drag) {
    const M_ = P.clusterMass(A.S.scene, drag.body);
    drag.k = 300 * M_;                        // about 2.8 Hz: firm, but you can feel the weight
    drag.c = 2 * 0.9 * Math.sqrt(drag.k * M_); // just under critically damped
    A.S.drag = drag;
  }

  const tools = {
    select: {
      down(sx, sy, e) {
        const S = A.S, scene = S.scene, w = V.s2w(sx, sy);
        // 1. handles on the selected joint (re-attach its ends)
        if (S.selected && !M.isBody(S.selected)) {
          for (const [which, p] of R.jointHandles(scene, S.selected)) {
            if (dist(R.ws(p), [sx, sy]) < 9) { op = { type: 'handle', j: S.selected, which, p }; return; }
          }
        }
        // ...and the end handles on a selected rod (change its length, re-attach the end)
        if (S.selected && S.selected.kind === 'bar' && !S.running) {
          for (const [sg, p] of Rd.endHandles(S.selected)) {
            if (dist(R.ws(p), [sx, sy]) < 9) { op = { type: 'barEnd', b: S.selected, sg, p }; return; }
          }
        }
        // 2. ground anchors: drag them about, even while it's running
        const anchors = anchorsAt(sx, sy);
        if (anchors) {
          // ...and open the support's card (its reactions are on the Forces tab), if it's a pin or glue
          const sup = scene.joints.find((j) => (j.kind === 'pivot' || j.kind === 'weld') && [j.a, j.b].some((e) => anchors.includes(e)));
          if (sup) A.select(sup);
          op = { type: 'anchor', ends: anchors, moved: false }; canvas.style.cursor = 'grabbing'; return;
        }
        // 3. a body: grab the exact point you pressed (a massless rod hands it to what it joins;
        //    a pin grabs the mechanism there)
        const item = pick(sx, sy, true);
        // a second part for comparing: shift-click, or after "Compare with another part"
        const first = S.selected && M.isBody(S.selected) ? S.selected : S.pairFrom;
        if (item && M.isBody(item) && first && item !== first && ((e && e.shiftKey) || S.pairFrom === first)) {
          S.pairFrom = null; A.select({ kind: 'pair', id: 'pair', a: first, b: item });
          A.hint('Comparing two parts: their angles and the forces between them are on the card.');
          op = { type: 'pan', last: [sx, sy] };
          return;
        }
        S.pairFrom = null;
        A.select(item);
        if (item && M.isBody(item)) { const [x, y] = P.toLocal(item, w[0], w[1]); S.probe = { body: item.id, x, y }; } else S.probe = null;
        let hold = null;
        if (item && M.isBody(item)) {
          if (item._red) hold = Rd.grabTarget(item, w);
          else { const [x, y] = P.toLocal(item, w[0], w[1]); hold = { body: item.id, x, y }; }
          if (!hold) A.hint('This rod is pinned to the ground at both ends, so there’s nothing to move.');
        } else if (item && (item.kind === 'pivot' || item.kind === 'weld')) hold = Rd.nodeGrab(item);
        if (hold) {
          const drag = { body: hold.body, x: hold.x, y: hold.y, tx: w[0], ty: w[1] };
          if (S.running) { grab(drag); op = { type: 'grab' }; } else { op = { type: 'pose', drag, moved: false }; S.poseDrag = drag; }
          canvas.style.cursor = 'grabbing';
        } else if (!item) op = { type: 'pan', last: [sx, sy] };
      },
      move(sx, sy) {
        if (!op) return;
        const S = A.S, w = V.s2w(sx, sy);
        if (op.type === 'grab' && S.drag) { S.drag.tx = w[0]; S.drag.ty = w[1]; }
        if (op.type === 'pose') {
          if (S.running) { grab(op.drag); S.poseDrag = null; op.type = 'grab'; return; } // pressed Play mid-drag
          if (!op.moved) { A.checkpoint(); op.moved = true; }
          op.drag.tx = w[0]; op.drag.ty = w[1];
          P.pose(S.scene, op.drag);
          A.touched();
        }
        if (op.type === 'anchor') {
          if (!op.moved && !S.running) A.checkpoint();
          op.moved = true;
          for (const e of op.ends) { e.x = w[0]; e.y = w[1]; }
          if (!S.running) { P.pose(S.scene, null, 20); A.touched(); }
        }
        if (op.type === 'handle') { const s = snap(sx, sy); op.p = s.p; S.snap = s.snapped ? s.p : null; }
        if (op.type === 'barEnd') {
          const s = snap(sx, sy); op.p = s.p; S.snap = s.snapped ? s.p : null;
          const f = R.ws(Rd.endHandles(op.b).find(([sg]) => sg === -op.sg)[1]), q = R.ws(op.p);
          setPreview((ctx) => line(ctx, f, q, 3.6, [6, 4]));
        }
      },
      up() {
        if (!op) return;
        if (op.type === 'grab') A.S.drag = null;
        if (op.type === 'pose') { A.S.poseDrag = null; if (op.moved) A.changed(); }
        if (op.type === 'anchor' && op.moved && !A.S.running) A.changed();
        if (op.type === 'handle') moveHandle(op.j, op.which, op.p);
        if (op.type === 'barEnd' && op.p !== Rd.endHandles(op.b).find(([sg]) => sg === op.sg)[1]) Rd.moveEnd(op.b, op.sg, op.p);
        canvas.style.cursor = '';
      },
    },

    camera: {
      tap: true, down() {}, move() {},
      up(sx, sy) {
        const w = V.s2w(sx, sy), b = bodiesAt(w)[0];
        if (!b) { A.follow(null); return; }
        const p = dist(R.ws([b.x, b.y]), [sx, sy]) < 10 ? [b.x, b.y] : w; // near the centre = the centre
        const [x, y] = P.toLocal(b, p[0], p[1]);
        A.follow({ body: b.id, x, y });
      },
    },

    box: {
      down(sx, sy) { op = { p0: snap(sx, sy).p, s0: [sx, sy] }; op.p1 = op.p0; },
      move(sx, sy) {
        if (!op) return;
        op.p1 = snap(sx, sy).p;
        const [a, b] = [R.ws(op.p0), R.ws(op.p1)];
        const wm = Math.abs(op.p1[0] - op.p0[0]), hm = Math.abs(op.p1[1] - op.p0[1]);
        setPreview((ctx) => {
          ctx.strokeStyle = R.colors().accent; ctx.lineWidth = 1.5; ctx.setLineDash([5, 4]);
          ctx.strokeRect(Math.min(a[0], b[0]), Math.min(a[1], b[1]), Math.abs(b[0] - a[0]), Math.abs(b[1] - a[1]));
          ctx.setLineDash([]); previewText(ctx, `${wm.toFixed(2)} × ${hm.toFixed(2)} m`, b);
        });
      },
      up(sx, sy) {
        const scene = A.S.scene, [x0, y0] = op.p0, [x1, y1] = op.p1;
        let w = Math.abs(x1 - x0), h = Math.abs(y1 - y0), cx = (x0 + x1) / 2, cy = (y0 + y1) / 2;
        if (dist(op.s0, [sx, sy]) < 8) { w = 1; h = 0.5; cx = x0; cy = y0; }
        w = Math.max(w, 0.05); h = Math.max(h, 0.05);
        A.checkpoint();
        const b = M.addBody(scene, { kind: 'box', x: cx, y: cy, w, h, mass: roundMass(w * h * 8), label: M.autoLabel(scene, 'box') });
        A.spawn(b); A.select(b); A.changed();
      },
    },

    poly: {
      down(sx, sy) {
        const p = snap(sx, sy).p, now = performance.now();
        const closing = poly.length >= 3 && dist(R.ws(poly[0]), [sx, sy]) < 10;
        const dbl = lastClick && now - lastClick.t < 350 && dist(lastClick.s, [sx, sy]) < 6;
        lastClick = { t: now, s: [sx, sy] };
        if (closing || (dbl && poly.length >= 3)) { finishPoly(); return; }
        if (!poly.length || dist(p, poly[poly.length - 1]) > 1e-6) poly.push(p);
        drawPolyPreview();
      },
      move() { if (poly.length) drawPolyPreview(); },
      up() {},
    },

    weight: {
      down(sx, sy) { op = { p0: snap(sx, sy).p, s0: [sx, sy], r: 0.12 }; },
      move(sx, sy) {
        if (!op) return;
        if (dist(op.s0, [sx, sy]) >= 6) op.r = Math.max(0.04, dist(op.p0, V.s2w(sx, sy)));
        const c = R.ws(op.p0), rp = op.r * V.ppm();
        setPreview((ctx) => {
          ctx.beginPath(); ctx.arc(c[0], c[1], rp, 0, Math.PI * 2);
          ctx.strokeStyle = R.colors().accent; ctx.lineWidth = 1.5; ctx.setLineDash([5, 4]); ctx.stroke(); ctx.setLineDash([]);
          previewText(ctx, `r = ${op.r.toFixed(2)} m`, [c[0] + rp * 0.7, c[1] + rp * 0.7]);
        });
      },
      up() {
        const scene = A.S.scene, host = bodiesAt(op.p0)[0];
        if (host && host.kind === 'bar') op.p0 = Rd.onPart(host, op.p0);
        A.checkpoint();
        const d = M.addBody(scene, { kind: 'disc', x: op.p0[0], y: op.p0[1], r: op.r, mass: 1, color: 'ink', label: M.autoLabel(scene, 'disc') });
        A.spawn(d);
        if (host) M.addJoint(scene, { kind: 'weld', a: endOn(host, op.p0), b: { body: d.id, x: 0, y: 0 }, angle0: -host.a });
        A.select(d); A.changed();
        if (host) A.hint(`Stuck the weight onto ${host.label || 'the ' + M.kindName(host).toLowerCase()}. Drop one in empty space for a free weight.`);
      },
    },

    // A circle with anchor points every 1/16 turn: tap for a 0.5 m one, or drag out its radius.
    // Unlike a weight it doesn't stick to what it's dropped on: pin or motor its centre.
    circle: {
      down(sx, sy) { op = { p0: snap(sx, sy).p, s0: [sx, sy], r: 0.5 }; },
      move(sx, sy) {
        if (!op) return;
        if (dist(op.s0, [sx, sy]) >= 6) op.r = Math.max(0.05, +dist(op.p0, V.s2w(sx, sy)).toFixed(2));
        const c = R.ws(op.p0), rp = op.r * V.ppm();
        setPreview((ctx) => {
          ctx.beginPath(); ctx.arc(c[0], c[1], rp, 0, Math.PI * 2);
          ctx.strokeStyle = R.colors().accent; ctx.lineWidth = 1.5; ctx.setLineDash([5, 4]); ctx.stroke(); ctx.setLineDash([]);
          for (let k = 0; k < 16; k++) { const t = (k * Math.PI) / 8; R.dot(ctx, [c[0] + rp * Math.cos(t), c[1] + rp * Math.sin(t)], 2, R.colors().accent); }
          previewText(ctx, `r = ${op.r.toFixed(2)} m`, [c[0] + rp * 0.7, c[1] + rp * 0.7]);
        });
      },
      up() {
        if (!op) return;
        const scene = A.S.scene;
        A.checkpoint();
        const d = M.addBody(scene, { kind: 'disc', x: op.p0[0], y: op.p0[1], r: op.r, mass: roundMass(Math.PI * op.r * op.r * 8), color: 'light', anchors: 16, ring: 1, label: M.autoLabel(scene, 'box') });
        A.spawn(d); A.select(d); A.changed();
        A.hint('Pin its centre so it can turn (or put a motor there). Its 16 rim points snap for pins, rods, springs and ropes.', 6000);
      },
    },

    pivot: { tap: true, down() {}, move() {}, up(sx, sy) { Rd.pinAt(snap(sx, sy).p, 'pivot'); } },
    motor: { tap: true, down() {}, move() {}, up(sx, sy) { Rd.pinAt(snap(sx, sy).p, 'motor'); } },
    weld: { tap: true, down() {}, move() {}, up(sx, sy) { Rd.pinAt(snap(sx, sy).p, 'weld'); } },

    // Rods: each end joins a shared pin, a free rod end, a rod's line, or a part (see rods.js).
    rod: {
      down(sx, sy) { const p = snap(sx, sy).p; op = { c1: Rd.classify(p), s0: [sx, sy] }; op.c2 = { type: 'empty', p }; },
      move(sx, sy) {
        if (!op) return;
        const s = snap(sx, sy); A.S.snap = s.snapped ? s.p : null;
        const ex = op.c1.type === 'part' ? op.c1.body.id : op.c1.bar ? op.c1.bar.id : undefined;
        op.c2 = Rd.classify(s.p, ex);
        op.plan = Rd.planRod(op.c1, op.c2);
        setPreview((ctx) => Rd.drawPlan(ctx, op.plan));
      },
      up() {
        if (!op || !op.plan) return;
        if (op.plan.refuse) { A.hint(op.plan.refuse); return; }
        Rd.makeRod(op.plan, op.c1, op.c2);
      },
    },
    rope: lineTool('rope'),
    spring: lineTool('spring'),
    damper: lineTool('damper'),
    piston: lineTool('piston'),

    // Three taps: the wheel, then each end of the rope. The rope starts just tight.
    pulley: {
      tap: true, down() {},
      move() { if (pul) drawPulleyPreview(); },
      up(sx, sy) {
        const scene = A.S.scene, p = snap(sx, sy).p;
        if (!pul) {
          const host = bodiesAt(p)[0];
          pul = { wheel: endOn(host, p), host: host ? host.id : null, wp: p };
          A.hint(`Wheel ${host ? 'on ' + (host.label || 'that part') : 'fixed to the ceiling'}. Now tap the first thing to tie the rope to.`, 0);
        } else if (!pul.a) {
          if (dist(p, pul.wp) < 0.05) return;
          pul.a = endAt(p, pul.host); pul.ap = p;
          A.hint('Now tap the second thing to tie the rope to.', 0);
        } else {
          const b = endAt(p, pul.host);
          if (dist(p, pul.wp) < 0.05) return;
          if (pul.a.body == null && b.body == null) { A.hint('At least one end of the rope needs to be on a part.'); return; }
          A.checkpoint();
          const j = M.addJoint(scene, { kind: 'pulley', a: pul.a, b, p: pul.wheel, r: 0.15, length: 0 });
          P.index(scene); j.length = P.pulleyGeom(scene, j).length; // starts just tight
          pul = null; setPreview(null);
          A.select(j); A.changed(); A.hint('Pulley rigged. Change the rope length in its card to hoist or lower.');
          return;
        }
        drawPulleyPreview();
      },
    },

    slider: {
      down(sx, sy) {
        const p = snap(sx, sy).p, car = bodiesAt(p)[0];
        if (!car) { A.hint('Start on the block that should slide.'); return; }
        op = { car, p0: p, p1: p, s0: [sx, sy] };
      },
      move(sx, sy) {
        if (!op) return;
        op.p1 = V.s2w(sx, sy);
        const d = [op.p1[0] - op.p0[0], op.p1[1] - op.p0[1]];
        const a = R.ws([op.p0[0] - d[0], op.p0[1] - d[1]]), b = R.ws(op.p1);
        setPreview((ctx) => { line(ctx, a, b, 1.5, [14, 3, 2, 3]); previewText(ctx, `travel ±${Math.hypot(d[0], d[1]).toFixed(2)} m`, b); });
      },
      up(sx, sy) {
        if (!op) return;
        const scene = A.S.scene, car = op.car;
        let d = [op.p1[0] - op.p0[0], op.p1[1] - op.p0[1]], L = Math.hypot(d[0], d[1]);
        if (dist(op.s0, [sx, sy]) < 10) { d = [1, 0]; L = 1; }
        const u = [d[0] / L, d[1] / L], host = bodiesAt(op.p0, car.id)[0];
        A.checkpoint();
        const j = M.addJoint(scene, {
          kind: 'slider', a: endOn(host, op.p0), b: endOn(car, op.p0),
          dir: host ? P.rotate(-host.a, u[0], u[1]) : u, angle0: car.a - (host ? host.a : 0), lo: -L, hi: L,
          lock: car.kind !== 'bar', // a rod on a track tilts freely: a roller support
        });
        A.select(j); A.changed();
      },
    },

    // A point load: press where it acts, drag the way it pushes (length = size), or tap for
    // 100 N straight down. It's drawn pointing at the point, like the loads in a statics figure.
    load: {
      down(sx, sy) {
        const p = snap(sx, sy).p, host = bodiesAt(p)[0];
        if (!host) { A.hint('A load goes on a part: press on a rod, block, weight or shape.'); return; }
        op = { host, p: host.kind === 'bar' ? Rd.onPart(host, p) : p, s0: [sx, sy], s1: [sx, sy] };
      },
      move(sx, sy) {
        if (!op) return;
        op.s1 = [sx, sy];
        const { F, angle } = loadFromDrag(op), q = R.ws(op.p), th = (angle * Math.PI) / 180;
        setPreview((ctx) => { ctx.save(); ctx.globalAlpha = 0.85; R.arrowTo(ctx, q, [Math.cos(th), -Math.sin(th)], 46, R.colors().accent, 2.4); ctx.restore();
          previewText(ctx, `${F} N at ${angle}°`, [sx, sy]); });
      },
      up() {
        if (!op) return;
        const scene = A.S.scene, { F, angle } = loadFromDrag(op);
        A.checkpoint();
        const e = endOn(op.host, op.p);
        const j = M.addJoint(scene, { kind: 'load', a: e, b: { ...e }, F, angle, label: M.autoLabel(scene, 'load') });
        A.select(j); A.changed();
        A.hint(`${F} N load on ${op.host.label || 'the ' + M.kindName(op.host).toLowerCase()}. Tap a support or a part to see what it does.`);
      },
    },
    couple: {
      tap: true, down() {}, move() {},
      up(sx, sy) {
        const scene = A.S.scene, p = snap(sx, sy).p, host = bodiesAt(p)[0];
        if (!host) { A.hint('A moment goes on a part: tap a rod, block, weight or shape.'); return; }
        A.checkpoint();
        const e = endOn(host, host.kind === 'bar' ? Rd.onPart(host, p) : p);
        const j = M.addJoint(scene, { kind: 'couple', a: e, b: { ...e }, M: 20, label: M.autoLabel(scene, 'couple') });
        A.select(j); A.changed();
      },
    },

    erase: {
      tap: true, down() {}, move() {},
      up(sx, sy) {
        const item = pick(sx, sy, true);
        if (!item) return;
        A.checkpoint();
        if (item.kind === 'pivot' || item.kind === 'weld') { // the whole shared pin
          const n = Rd.nodeOf(item);
          A.S.scene.joints = A.S.scene.joints.filter((j) => !(n ? n.joints : [item]).includes(j));
        } else M.remove(A.S.scene, item);
        A.select(null); A.changed();
      },
    },
  };

  // Drag length and direction → a tidy force: about 4 N per pixel, rounded to 2 significant
  // figures, at a whole-degree angle (textbook angles: anticlockwise from +x). A tap is 100 N down.
  function loadFromDrag(o) {
    const dx = o.s1[0] - o.s0[0], dy = o.s1[1] - o.s0[1], L = Math.hypot(dx, dy);
    if (L < 8) return { F: 100, angle: -90 };
    let angle = Math.round((Math.atan2(-dy, dx) * 180) / Math.PI);
    for (const a of [-180, -135, -90, -45, 0, 45, 90, 135, 180]) if (Math.abs(angle - a) <= 4) angle = a === -180 ? 180 : a; // snap to the common ones
    return { F: +(L * 4).toPrecision(2), angle };
  }

  // Ropes, springs, dampers and pistons are all drawn by dragging from point to point.
  function lineTool(kind) {
    return {
      down(sx, sy) { op = { p0: snap(sx, sy).p }; op.p1 = op.p0; },
      move(sx, sy) {
        if (!op) return;
        const s = snap(sx, sy); op.p1 = s.p; A.S.snap = s.snapped ? s.p : null;
        const a = R.ws(op.p0), b = R.ws(op.p1), L = dist(op.p0, op.p1);
        setPreview((ctx) => {
          ctx.save(); ctx.globalAlpha = 0.8;
          if (kind === 'spring') R.coil(ctx, a, b, 10, 9);
          else line(ctx, a, b, { rope: 1.6, damper: 2, piston: 5 }[kind]);
          ctx.restore(); previewText(ctx, `${L.toFixed(2)} m`, b);
        });
      },
      up() {
        if (!op) return;
        const scene = A.S.scene, L = dist(op.p0, op.p1);
        if (L < 0.03) { A.hint(`Drag to make a ${kind}: press on one point and let go on another.`); return; }
        const ea = endAt(op.p0), eb = endAt(op.p1, ea.body);
        if (ea.body == null && eb.body == null) { A.hint(`At least one end of the ${kind} needs to be on a part.`); return; }
        const extra = {
          rope: { length: L },
          spring: { rest: L, label: M.autoLabel(scene, 'spring') },
          damper: { c: 20, len0: L, label: M.autoLabel(scene, 'damper') },
          piston: { min: +(L * 0.6).toFixed(2), max: +(L * 1.4).toFixed(2), len0: L },
        }[kind];
        A.checkpoint();
        const j = M.addJoint(scene, Object.assign({ kind, a: ea, b: eb }, extra));
        A.select(j); A.changed();
      },
    };
  }

  function drawPolyPreview() {
    setPreview((ctx) => {
      const pts = poly.map(R.ws);
      if (cursor) pts.push(cursor);
      ctx.beginPath(); pts.forEach((s, i) => (i ? ctx.lineTo(s[0], s[1]) : ctx.moveTo(s[0], s[1])));
      ctx.strokeStyle = R.colors().accent; ctx.lineWidth = 1.5; ctx.setLineDash([5, 4]); ctx.stroke(); ctx.setLineDash([]);
      for (const s of poly.map(R.ws)) R.dot(ctx, s, 3, R.colors().accent);
      if (poly.length >= 3) { const f = R.ws(poly[0]); ctx.beginPath(); ctx.arc(f[0], f[1], 8, 0, Math.PI * 2); ctx.stroke(); }
    });
  }

  function drawPulleyPreview() {
    setPreview((ctx) => {
      const c = R.ws(pul.wp), r = 0.15 * V.ppm();
      ctx.beginPath(); ctx.arc(c[0], c[1], r, 0, Math.PI * 2); ctx.strokeStyle = R.colors().accent; ctx.lineWidth = 1.5; ctx.stroke();
      if (pul.ap) line(ctx, R.ws(pul.ap), c, 1.6);
      if (cursor) line(ctx, c, cursor, 1.2, [5, 4]);
    });
  }

  function finishPoly() {
    const scene = A.S.scene;
    if (poly.length >= 3 && Math.abs(P.polyArea(poly)) > 1e-3) {
      const [cx, cy] = P.polyCentroid(poly), pts = poly.map(([x, y]) => [x - cx, y - cy]);
      A.checkpoint();
      const b = M.addBody(scene, { kind: 'poly', x: cx, y: cy, pts, mass: roundMass(Math.abs(P.polyArea(pts)) * 8), label: M.autoLabel(scene, 'poly') });
      A.spawn(b); A.select(b); A.changed();
    }
    poly = []; setPreview(null);
  }

  // Dragging a joint's handle: ends re-attach to whatever is under them; a pivot or weld keeps
  // its two parts but moves to the new spot; a pulley wheel can move to a part or the ceiling.
  function moveHandle(j, which, p) {
    const scene = A.S.scene;
    A.checkpoint();
    if (which === 'ab') {
      for (const e of [j.a, j.b]) Object.assign(e, endOn(P.bodyOf(scene, e), p));
    } else if (which === 'p') {
      j.p = endAt(p);
    } else {
      const other = which === 'a' ? j.b : j.a;
      Object.assign(j[which], endAt(p, other.body));
    }
    const L = P.jointLength(scene, j);
    if (j.kind === 'rod' || j.kind === 'rope') j.length = L;
    if (j.kind === 'damper') j.len0 = L;
    if (j.kind === 'piston') { j.min = Math.min(j.min, L); j.max = Math.max(j.max, L); }
    if (j.kind === 'pulley') j.length = P.pulleyGeom(scene, j).length;
    A.changed();
  }

  // Parts dragged straight out of the tray land where they're dropped.
  function drop(tool, sx, sy) {
    if (A.S.running) A.pause();
    if (tool === 'box') { op = { p0: snap(sx, sy).p, s0: [sx, sy] }; op.p1 = op.p0; tools.box.up(sx, sy); }
    else if (tool === 'weight') { op = { p0: snap(sx, sy).p, s0: [sx, sy], r: 0.12 }; tools.weight.up(); }
    else if (tool === 'circle') { op = { p0: snap(sx, sy).p, s0: [sx, sy], r: 0.5 }; tools.circle.up(); }
    else if (tool === 'pivot' || tool === 'weld' || tool === 'motor') tools[tool].up(sx, sy);
    op = null;
  }

  // ---------- pointer plumbing ----------

  function startPinch() {
    const [a, b] = [...pointers.values()];
    pinch = { d0: dist(a, b) || 1, z0: V.zoom, w0: V.s2w((a[0] + b[0]) / 2, (a[1] + b[1]) / 2) };
  }

  function zoomAt(sx, sy, zoom) {
    const w = V.s2w(sx, sy);
    V.userMoved = true;
    V.zoom = Math.min(8, Math.max(0.15, zoom));
    if (A.S.follow) return; // the camera keeps its target centred
    V.cx = w[0] - (sx - V.W / 2) / V.ppm();
    V.cy = w[1] - (sy - V.H / 2) / V.ppm();
  }

  function cancel() {
    op = null; poly = []; pul = null; setPreview(null); A.S.snap = null;
    A.S.drag = null; A.S.poseDrag = null;
  }

  const capsHeld = new Map(); // pointer id → key cap it's holding down
  function onDown(e) {
    canvas.setPointerCapture(e.pointerId);
    // a key cap on the board works like the key: hold it to drive
    const cap = R.keyAt(e.offsetX, e.offsetY);
    if (cap) {
      capsHeld.set(e.pointerId, cap); ML.Controls.press(cap, A.S.scene);
      if (!A.S.running) A.play();
      return;
    }
    pointers.set(e.pointerId, [e.offsetX, e.offsetY]);
    if (pointers.size === 2) { cancel(); startPinch(); return; }
    if (pointers.size > 2) return;
    if (e.button === 1 || e.button === 2) { op = { type: 'pan', last: [e.offsetX, e.offsetY] }; return; }
    const t = A.S.tool;
    if (t !== 'select' && t !== 'camera' && A.S.running) A.pause(); // building happens with time stopped
    tools[t].down(e.offsetX, e.offsetY, e);
  }

  function onMove(e) {
    const sx = e.offsetX, sy = e.offsetY;
    cursor = [sx, sy];
    if (pointers.has(e.pointerId)) pointers.set(e.pointerId, [sx, sy]);
    if (pinch && pointers.size >= 2) {
      const [a, b] = [...pointers.values()], mx = (a[0] + b[0]) / 2, my = (a[1] + b[1]) / 2;
      V.userMoved = true;
      V.zoom = Math.min(8, Math.max(0.15, (pinch.z0 * dist(a, b)) / pinch.d0));
      if (!A.S.follow) { V.cx = pinch.w0[0] - (mx - V.W / 2) / V.ppm(); V.cy = pinch.w0[1] - (my - V.H / 2) / V.ppm(); }
      return;
    }
    if (op && op.type === 'pan') {
      if (A.S.follow) A.follow(null); // panning takes the camera back
      V.userMoved = true;
      V.cx -= (sx - op.last[0]) / V.ppm(); V.cy -= (sy - op.last[1]) / V.ppm();
      op.last = [sx, sy]; return;
    }
    const t = A.S.tool;
    if (!op && R.keyAt(sx, sy)) { canvas.style.cursor = 'pointer'; A.S.hover = null; return; } // a key cap
    if (op || t === 'poly' || t === 'pulley') { tools[t].move(sx, sy); return; }
    // hovering
    if (t === 'select' || t === 'erase' || t === 'camera') {
      const overAnchor = t === 'select' && anchorsAt(sx, sy);
      A.S.hover = overAnchor ? null : t === 'camera' ? bodiesAt(V.s2w(sx, sy))[0] || null : pick(sx, sy);
      A.S.snap = null;
      canvas.style.cursor = overAnchor || (t === 'select' && A.S.hover && M.isBody(A.S.hover)) ? 'grab' : t === 'camera' ? 'crosshair' : '';
    } else {
      A.S.hover = null;
      const s = snap(sx, sy); A.S.snap = s.snapped ? s.p : null;
      canvas.style.cursor = 'crosshair';
    }
  }

  function onUp(e) {
    if (capsHeld.has(e.pointerId)) { ML.Controls.release(capsHeld.get(e.pointerId)); capsHeld.delete(e.pointerId); return; }
    pointers.delete(e.pointerId);
    if (pinch) { if (pointers.size < 2) pinch = null; op = null; return; }
    if (op && op.type === 'pan') { op = null; return; }
    const t = A.S.tool;
    if (op || tools[t].tap) tools[t].up(e.offsetX, e.offsetY, e);
    op = null; A.S.snap = null;
    if (t !== 'poly' && t !== 'pulley') setPreview(null);
  }

  function onWheel(e) {
    e.preventDefault();
    zoomAt(e.offsetX, e.offsetY, V.zoom * Math.exp(-e.deltaY * 0.0015));
  }

  function key(e) { // returns true when the editor used the key
    if (A.S.tool === 'poly' && poly.length) {
      if (e.key === 'Enter') { finishPoly(); return true; }
      if (e.key === 'Backspace') { poly.pop(); poly.length ? drawPolyPreview() : setPreview(null); return true; }
    }
    if (e.key === 'Escape' && (op || poly.length || pul)) { cancel(); A.hint('Cancelled.', 1500); return true; }
    return false;
  }

  function init(app, el) {
    A = app; canvas = el;
    el.addEventListener('pointerdown', onDown);
    el.addEventListener('pointermove', onMove);
    el.addEventListener('pointerup', onUp);
    el.addEventListener('pointercancel', (e) => {
      if (capsHeld.has(e.pointerId)) { ML.Controls.release(capsHeld.get(e.pointerId)); capsHeld.delete(e.pointerId); }
      pointers.delete(e.pointerId); pinch = null; cancel();
    });
    el.addEventListener('pointerleave', () => { if (!op) { A.S.hover = null; A.S.snap = null; } });
    el.addEventListener('wheel', onWheel, { passive: false });
    el.addEventListener('contextmenu', (e) => e.preventDefault());
  }

  ML.Editor = { init, cancel, key, zoomAt, drop, HINTS, tools: Object.keys(tools) };
})();
