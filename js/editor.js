// editor.js: the build tools. Mouse, pen and touch all go through pointer events.
// Each tool reacts to down / move / up; the gesture in progress lives in `op`.
(function () {
  const ML = (window.ML = window.ML || {});
  const P = ML.Phys, R = ML.Render, M = ML.Model, V = R.View;
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
    pivot: 'Tap where two parts overlap to pin them together. Tap a single part to pin it to the ground.',
    motor: 'Tap where two parts overlap (or on one part) to add a motor there. Set its speed and torque in its card.',
    weld: 'Tap where two parts overlap to glue them into one piece.',
    rod: 'Drag from one point to another for a stiff, weightless rod. An end in empty space is pinned to the ground.',
    rope: 'Drag from one point to another for a rope. It only pulls when it’s tight. Make it longer in its card to give it slack.',
    pulley: 'Tap where the wheel goes, then the first thing to tie on, then the second.',
    spring: 'Drag from one point to another. The spring starts at its relaxed length. An end in empty space hooks to a wall.',
    damper: 'Drag from one point to another for a damper. It resists motion along its length (it doesn’t spring back).',
    piston: 'Drag from one point to another for a piston. It pushes in and out on its own. Set speed, force and stroke in its card.',
    slider: 'Press on a block, then drag the way it should slide.',
    erase: 'Tap anything to remove it.',
  };

  const dist = (a, b) => Math.hypot(a[0] - b[0], a[1] - b[1]);
  const roundMass = (m) => Math.max(0.1, Math.round(m * 10) / 10);
  const tolW = () => 3 / V.ppm(); // 3 px expressed in metres
  const TWO_POINT = new Set(['rod', 'rope', 'spring', 'damper', 'piston']);

  // ---------- finding things under the pointer ----------

  function bodiesAt(p, exceptId) {
    const out = [];
    const bs = A.S.scene.bodies;
    for (let i = bs.length - 1; i >= 0; i--) if (bs[i].id !== exceptId && P.hitBody(bs[i], p[0], p[1], tolW())) out.push(bs[i]);
    return out;
  }

  function segDist(p, a, b) {
    const dx = b[0] - a[0], dy = b[1] - a[1], L2 = dx * dx + dy * dy || 1;
    const t = Math.max(0, Math.min(1, ((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / L2));
    return Math.hypot(p[0] - a[0] - t * dx, p[1] - a[1] - t * dy);
  }

  function railEnds(scene, j) {
    const A_ = P.bodyOf(scene, j.a), pA = P.endWorld(scene, j.a), [ux, uy] = P.rotate(A_ ? A_.a : 0, j.dir[0], j.dir[1]);
    return [[pA[0] + ux * j.lo, pA[1] + uy * j.lo], [pA[0] + ux * j.hi, pA[1] + uy * j.hi]];
  }

  // Pins first, then pulleys, then rods / ropes / springs / dampers / pistons, then bodies, then
  // slider rails (a rail runs right through its block, so it mustn't steal the block's clicks).
  function pick(sx, sy) {
    const scene = A.S.scene, s = [sx, sy];
    P.index(scene);
    let best = null, bd = 9;
    for (const j of scene.joints) {
      if (j.kind !== 'pivot' && j.kind !== 'weld') continue;
      const d = dist(s, R.ws(P.endWorld(scene, j.a)));
      if (d < bd) { bd = d; best = j; }
    }
    if (best) return best;
    for (const j of scene.joints) {
      if (j.kind === 'pulley' && dist(s, R.ws(P.endWorld(scene, j.p))) < j.r * V.ppm() + 6) return j;
    }
    bd = 7;
    for (const j of scene.joints) {
      const segs = TWO_POINT.has(j.kind) ? [[j.a, j.b]] : j.kind === 'pulley' ? [[j.a, j.p], [j.p, j.b]] : [];
      for (const [e1, e2] of segs) {
        const d = segDist(s, R.ws(P.endWorld(scene, e1)), R.ws(P.endWorld(scene, e2)));
        if (d < bd) { bd = d; best = j; }
      }
    }
    if (best) return best;
    const b = bodiesAt(V.s2w(sx, sy))[0];
    if (b) return b;
    for (const j of scene.joints) {
      if (j.kind !== 'slider') continue;
      const [e0, e1] = railEnds(scene, j);
      if (segDist(s, R.ws(e0), R.ws(e1)) < 6) return j;
    }
    return null;
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
      if (b.kind === 'box') {
        for (const [i, k] of ring) consider(P.toWorld(b, (i * b.w) / 2, (k * b.h) / 2));
        if (b.hollow) { const [wi, hi] = P.innerSize(b); for (const [i, k] of ring) consider(P.toWorld(b, (i * wi) / 2, (k * hi) / 2)); }
      } else if (b.kind === 'poly') for (const [lx, ly] of b.pts) consider(P.toWorld(b, lx, ly));
    }
    for (const j of scene.joints) for (const [, p] of R.jointHandles(scene, j)) consider(p);
    if (best) return { p: best, snapped: true };
    const w = V.s2w(sx, sy);
    if (A.S.opts.snapGrid) return { p: [Math.round(w[0] * 10) / 10, Math.round(w[1] * 10) / 10], snapped: true };
    return { p: w, snapped: false };
  }

  // A joint end at world point p: on the topmost body there (except one), or on the ground.
  function endAt(p, exceptId) { return endOn(bodiesAt(p, exceptId)[0], p); }
  function endOn(b, p) {
    if (!b) return { body: null, x: p[0], y: p[1] };
    const [x, y] = P.toLocal(b, p[0], p[1]);
    return { body: b.id, x, y };
  }

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
      down(sx, sy) {
        const S = A.S, scene = S.scene, w = V.s2w(sx, sy);
        // 1. handles on the selected joint (re-attach its ends)
        if (S.selected && !M.isBody(S.selected)) {
          for (const [which, p] of R.jointHandles(scene, S.selected)) {
            if (dist(R.ws(p), [sx, sy]) < 9) { op = { type: 'handle', j: S.selected, which, p }; return; }
          }
        }
        // 2. ground anchors: drag them about, even while it's running
        const anchors = anchorsAt(sx, sy);
        if (anchors) { op = { type: 'anchor', ends: anchors, moved: false }; canvas.style.cursor = 'grabbing'; return; }
        // 3. a body: grab the exact point you pressed
        const item = pick(sx, sy);
        A.select(item);
        if (item && M.isBody(item)) {
          const [x, y] = P.toLocal(item, w[0], w[1]);
          const drag = { body: item.id, x, y, tx: w[0], ty: w[1] };
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
      },
      up() {
        if (!op) return;
        if (op.type === 'grab') A.S.drag = null;
        if (op.type === 'pose') { A.S.poseDrag = null; if (op.moved) A.changed(); }
        if (op.type === 'anchor' && op.moved && !A.S.running) A.changed();
        if (op.type === 'handle') moveHandle(op.j, op.which, op.p);
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
        A.checkpoint();
        const d = M.addBody(scene, { kind: 'disc', x: op.p0[0], y: op.p0[1], r: op.r, mass: 1, color: 'ink', label: M.autoLabel(scene, 'disc') });
        A.spawn(d);
        if (host) M.addJoint(scene, { kind: 'weld', a: endOn(host, op.p0), b: { body: d.id, x: 0, y: 0 }, angle0: -host.a });
        A.select(d); A.changed();
        if (host) A.hint(`Stuck the weight onto ${host.label || 'the ' + M.kindName(host).toLowerCase()}. Drop one in empty space for a free weight.`);
      },
    },

    pivot: { tap: true, down() {}, move() {}, up(sx, sy) { pinAt(sx, sy, 'pivot'); } },
    motor: { tap: true, down() {}, move() {}, up(sx, sy) { pinAt(sx, sy, 'pivot', { motor: 30, torque: 50 }); } },
    weld: { tap: true, down() {}, move() {}, up(sx, sy) { pinAt(sx, sy, 'weld'); } },

    rod: lineTool('rod'),
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
        });
        A.select(j); A.changed();
      },
    },

    erase: {
      tap: true, down() {}, move() {},
      up(sx, sy) {
        const item = pick(sx, sy);
        if (!item) return;
        A.checkpoint(); M.remove(A.S.scene, item); A.select(null); A.changed();
      },
    },
  };

  // Rods, ropes, springs, dampers and pistons are all drawn by dragging from point to point.
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
          else line(ctx, a, b, { rod: 3.6, rope: 1.6, damper: 2, piston: 5 }[kind]);
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
          rod: { length: L, label: M.autoLabel(scene, 'rod') },
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

  function pinAt(sx, sy, kind, extra = {}) {
    const scene = A.S.scene, p = snap(sx, sy).p, under = bodiesAt(p);
    const what = extra.motor ? 'motor' : kind === 'pivot' ? 'pin' : 'glue';
    if (!under.length) { A.hint(`A ${what} goes on a part: tap one, or where two parts overlap.`); return; }
    const [b1, b2] = under;
    A.checkpoint();
    const props = Object.assign({ kind, a: endOn(b1, p), b: endOn(b2, p) }, extra);
    if (kind === 'weld') props.angle0 = (b2 ? b2.a : 0) - b1.a;
    const j = M.addJoint(scene, props);
    A.select(j); A.changed();
    A.hint(`${what[0].toUpperCase() + what.slice(1)} ${b2 ? 'joins the two parts' : 'fixes it to the ground'}.`);
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

  function onDown(e) {
    canvas.setPointerCapture(e.pointerId);
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
    el.addEventListener('pointercancel', (e) => { pointers.delete(e.pointerId); pinch = null; cancel(); });
    el.addEventListener('pointerleave', () => { if (!op) { A.S.hover = null; A.S.snap = null; } });
    el.addEventListener('wheel', onWheel, { passive: false });
    el.addEventListener('contextmenu', (e) => e.preventDefault());
  }

  ML.Editor = { init, cancel, key, zoomAt, drop, HINTS, tools: Object.keys(tools) };
})();
