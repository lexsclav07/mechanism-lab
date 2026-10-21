// rods.js: how rods and pins join things in the editor.
// Rods are parts (bodies of kind 'bar'), so they join each other with ordinary pins. A "shared
// pin" (node) is every pin or glue joint meeting at one spot; it's stored as a star of joints
// from one hub part to each of the others, and worked out from the joints whenever it's needed.
(function () {
  const ML = (window.ML = window.ML || {});
  const P = ML.Phys, R = ML.Render, M = ML.Model, V = R.View;
  let A; // ML.App, set in init()

  const dist = (a, b) => Math.hypot(a[0] - b[0], a[1] - b[1]);
  const isBar = (b) => b && b.kind === 'bar';
  const ground = (p) => ({ body: null, x: p[0], y: p[1] });
  const isPin = (j) => j.kind === 'pivot' || j.kind === 'weld';
  const byId = (id) => (id == null ? null : A.S.scene._byId.get(id) || null);

  // Parts under a world point, topmost first. A rod counts within 7 px of its line, other parts
  // within 3 px of their outline.
  function partsAt(p, exceptId) {
    const out = [], bs = A.S.scene.bodies;
    for (let i = bs.length - 1; i >= 0; i--) {
      const b = bs[i];
      if (b.id !== exceptId && P.hitBody(b, p[0], p[1], (isBar(b) ? 7 : 3) / V.ppm())) out.push(b);
    }
    return out;
  }

  // A joint end on part b at world point p (on a rod: projected onto its centre line), or the ground.
  function endOn(b, p) {
    if (!b) return ground(p);
    const [x, y] = P.toLocal(b, p[0], p[1]);
    if (isBar(b)) return { body: b.id, x: Math.max(-b.L / 2, Math.min(b.L / 2, x)), y: 0 };
    return { body: b.id, x, y };
  }
  const onPart = (b, p) => (b ? P.endWorld(A.S.scene, endOn(b, p)) : p);

  // Where two rods' centre lines cross (if they do, within both rods).
  function crossing(b1, b2) {
    const a1 = P.toWorld(b1, -b1.L / 2, 0), a2 = P.toWorld(b1, b1.L / 2, 0);
    const c1 = P.toWorld(b2, -b2.L / 2, 0), c2 = P.toWorld(b2, b2.L / 2, 0);
    const d1 = [a2[0] - a1[0], a2[1] - a1[1]], d2 = [c2[0] - c1[0], c2[1] - c1[1]];
    const den = d1[0] * d2[1] - d1[1] * d2[0];
    if (Math.abs(den) < 1e-12) return null;
    const t = ((c1[0] - a1[0]) * d2[1] - (c1[1] - a1[1]) * d2[0]) / den;
    const u = ((c1[0] - a1[0]) * d1[1] - (c1[1] - a1[1]) * d1[0]) / den;
    return t >= 0 && t <= 1 && u >= 0 && u <= 1 ? [a1[0] + t * d1[0], a1[1] + t * d1[1]] : null;
  }

  // Useful snap points on rods: both ends, the middle, and (when building with rods) crossings.
  function snapPoints(withCrossings) {
    const bars = A.S.scene.bodies.filter(isBar), out = [];
    for (const b of bars) for (const x of [-b.L / 2, 0, b.L / 2]) out.push(P.toWorld(b, x, 0));
    if (withCrossings && bars.length <= 200) {
      for (let i = 0; i < bars.length; i++) for (let k = i + 1; k < bars.length; k++) { const c = crossing(bars[i], bars[k]); if (c) out.push(c); }
    }
    return out;
  }

  // ---------- shared pins (nodes) ----------

  const sameEnd = (e1, e2) => (e1.body != null ? e1.body === e2.body : e2.body == null && Math.abs(e1.x - e2.x) < 1e-6 && Math.abs(e1.y - e2.y) < 1e-6);

  // Every pin and glue joint at world point `at`, the ends they hold (members), and the hub:
  // the heaviest block/weight/shape there, else the first rod, else the ground.
  function nodeAtPoint(at) {
    const scene = A.S.scene;
    P.index(scene);
    const joints = scene.joints.filter((j) => isPin(j) && dist(P.endWorld(scene, j.a), at) < 1e-4);
    if (!joints.length) return null;
    const members = [];
    for (const j of joints) for (const e of [j.a, j.b]) if (!members.some((m) => sameEnd(m, e))) members.push(e);
    const parts = members.filter((e) => e.body != null && !isBar(byId(e.body)));
    const hub = parts.sort((e1, e2) => byId(e2.body).mass - byId(e1.body).mass)[0]
      || members.find((e) => e.body != null) || members[0];
    return { p: P.endWorld(scene, joints[0].a), joints, members, hub };
  }

  // The shared pin nearest a screen point, within 9 px.
  function nodeAt(sx, sy) {
    const scene = A.S.scene;
    let best = 9, at = null;
    for (const j of scene.joints) {
      if (!isPin(j)) continue;
      const q = P.endWorld(scene, j.a), d = dist(R.ws(q), [sx, sy]);
      if (d < best) { best = d; at = q; }
    }
    return at ? nodeAtPoint(at) : null;
  }
  const nodeOf = (j) => nodeAtPoint(P.endWorld(A.S.scene, j.a));

  // Is something already attached to rod b at local x?
  const pinnedAt = (b, x) => A.S.scene.joints.some((j) => ['a', 'b', 'p'].some((k) => j[k] && j[k].body === b.id && Math.abs(j[k].x - x) < 1e-6));

  // ---------- drawing a rod ----------

  // What's at one end of a rod being drawn, in order: a shared pin, a free rod end, a rod's
  // line, a block/weight/shape, or empty space. p is the snapped point; except skips a part.
  function classify(p, except) {
    const scene = A.S.scene, s = R.ws(p);
    const node = nodeAt(s[0], s[1]);
    if (node && !node.members.every((m) => m.body === except)) return { type: 'node', node, p: node.p };
    let best = 10, hit = null;
    for (const b of scene.bodies) {
      if (!isBar(b) || b.id === except) continue;
      for (const x of [-b.L / 2, b.L / 2]) {
        const q = P.toWorld(b, x, 0), d = dist(R.ws(q), s);
        if (d < best && !pinnedAt(b, x)) { best = d; hit = { type: 'end', bar: b, x, p: q }; }
      }
    }
    if (hit) return hit;
    best = 7;
    for (const b of scene.bodies) {
      if (!isBar(b) || b.id === except) continue;
      const [x, y] = P.toLocal(b, p[0], p[1]), d = Math.abs(y) * V.ppm();
      if (Math.abs(x) <= b.L / 2 && d < best) { best = d; hit = { type: 'line', bar: b, x, p: P.toWorld(b, x, 0) }; }
    }
    if (hit) return hit;
    const part = partsAt(p, except).find((b) => !isBar(b));
    return part ? { type: 'part', body: part, p } : { type: 'empty', p };
  }

  // What a rod end gets pinned to (null = left free).
  function target(c) {
    if (c.type === 'node') return { ...c.node.hub };
    if (c.type === 'end' || c.type === 'line') return { body: c.bar.id, x: c.x, y: 0 };
    if (c.type === 'part') return endOn(c.body, c.p);
    return null;
  }
  // Ends that make an empty far end a ground pin: a block, weight or shape (directly or as a pin's hub).
  const massive = (c) => c.type === 'part' || (c.type === 'node' && c.node.hub.body != null && !isBar(byId(c.node.hub.body)));
  const solid = (t) => t && (t.body == null || !isBar(byId(t.body))); // ground or a block/weight/shape

  // The whole plan for a rod from c1 to c2: where its ends go, whether it'll be massless, or why not.
  function planRod(c1, c2) {
    let tA = target(c1), tB = target(c2);
    if (!tA && massive(c2)) tA = ground(c1.p); // the quick pendulum: empty end next to a part = ground pin
    if (!tB && massive(c1)) tB = ground(c2.p);
    const L = dist(c1.p, c2.p);
    let refuse = null;
    if (L < 0.03) refuse = 'Drag to make a rod: press on one point and let go on another.';
    else if (tA && tB && tA.body != null && tA.body === tB.body && isBar(byId(tA.body))) refuse = 'Both ends are on the same rod.';
    else if (tA && tB && tA.body == null && tB.body == null) refuse = 'Both ends are on the ground, so it couldn’t move.';
    return { pA: c1.p, pB: c2.p, L, tA, tB, massless: solid(tA) && solid(tB), refuse };
  }

  function makeRod(plan, c1, c2) {
    const scene = A.S.scene, { pA, pB, L } = plan;
    A.checkpoint();
    const b = M.addBody(scene, { kind: 'bar', x: (pA[0] + pB[0]) / 2, y: (pA[1] + pB[1]) / 2, a: Math.atan2(pB[1] - pA[1], pB[0] - pA[0]), L, label: M.autoLabel(scene, 'bar') });
    if (plan.tA) M.addJoint(scene, { kind: 'pivot', a: { body: b.id, x: -L / 2, y: 0 }, b: plan.tA });
    if (plan.tB) M.addJoint(scene, { kind: 'pivot', a: { body: b.id, x: L / 2, y: 0 }, b: plan.tB });
    A.spawn(b); A.select(b); A.changed();
    const ontoGroundPin = [c1, c2].some((c) => c.type === 'node' && c.node.hub.body == null);
    if (!plan.tA && !plan.tB) A.hint('Free rod. Pin an end to fix it to the ground, or join more rods to it.', 6000);
    else if (!plan.tA || !plan.tB) A.hint('The far end is free: pin it with the Pin tool, or draw another rod from it.', 6000);
    else if (ontoGroundPin) A.hint('Joined at a ground pin. To make it a free hinge, tap the pin and switch off “Fixed to the ground”.', 7000);
  }

  // Preview while dragging: the line, a marker at each end (⊙ joins, hatched = ground, ○ free)
  // and the length with "massless" or the rod's mass.
  function drawPlan(ctx, plan) {
    const C = R.colors(), a = R.ws(plan.pA), b = R.ws(plan.pB);
    ctx.save(); ctx.globalAlpha = 0.85;
    ctx.beginPath(); ctx.moveTo(a[0], a[1]); ctx.lineTo(b[0], b[1]);
    ctx.lineWidth = 3.6; ctx.lineCap = 'round'; ctx.strokeStyle = plan.refuse ? C.muted : C.accent; ctx.stroke();
    for (const [t, s, o] of [[plan.tA, a, b], [plan.tB, b, a]]) {
      if (t && t.body == null) R.support(ctx, s, s[0] - o[0], s[1] - o[1]);
      if (t) R.pin(ctx, s);
      else { ctx.beginPath(); ctx.arc(s[0], s[1], 5, 0, Math.PI * 2); ctx.fillStyle = C.paper; ctx.fill(); ctx.lineWidth = 1.5; ctx.strokeStyle = C.accent; ctx.stroke(); }
    }
    ctx.restore();
    const text = plan.refuse ? plan.refuse : `${plan.L.toFixed(2)} m · ${plan.massless ? 'massless' : `${M.barMass(plan.L)} kg`}`;
    ctx.font = '12px "IBM Plex Mono", ui-monospace, monospace'; ctx.textAlign = 'left'; ctx.textBaseline = 'top';
    ctx.lineWidth = 4; ctx.strokeStyle = C.paper; ctx.strokeText(text, b[0] + 12, b[1] + 12);
    ctx.fillStyle = C.accent; ctx.fillText(text, b[0] + 12, b[1] + 12);
  }

  // ---------- Pin, Glue and Motor ----------

  function joint(kind, b1, b2, q, extra = {}) {
    const props = Object.assign({ kind: kind === 'weld' ? 'weld' : 'pivot', a: endOn(b1, q), b: b2 ? endOn(b2, q) : ground(q) }, extra);
    if (props.kind === 'weld') props.angle0 = (b2 ? b2.a : 0) - b1.a;
    return M.addJoint(A.S.scene, props);
  }

  // kind: 'pivot' (pin), 'weld' (glue) or 'motor'. p is the snapped world point.
  function pinAt(p, kind) {
    const scene = A.S.scene, s = R.ws(p);
    const node = nodeAt(s[0], s[1]);
    if (node) { changeNode(node, kind); return; }
    const parts = partsAt(p);
    const what = kind === 'motor' ? 'motor' : kind === 'pivot' ? 'pin' : 'glue';
    if (!parts.length) { A.hint(`A ${what} goes on a part: a rod, block, weight or shape.`); return; }
    // where: the crossing of two rods, else on the first rod's line, else the tapped point
    const bars = parts.filter(isBar);
    let q = p;
    if (bars.length >= 2) q = crossing(bars[0], bars[1]) || onPart(bars[0], p);
    else if (bars.length === 1) q = onPart(bars[0], p);
    const extra = kind === 'motor' ? { motor: 30, torque: 50 } : {};
    A.checkpoint();
    let first;
    if (parts.length === 1) first = joint(kind, parts[0], null, q, extra);
    else {
      const hub = parts.filter((b) => !isBar(b)).sort((x, y) => y.mass - x.mass)[0] || parts[0];
      parts.filter((b) => b !== hub).forEach((b, i) => { const j = joint(kind, b, hub, q, i === 0 ? extra : {}); first = first || j; });
    }
    A.select(first); A.changed();
    const Name = what[0].toUpperCase() + what.slice(1);
    A.hint(parts.length === 1 ? `${Name} fixes it to the ground${isBar(parts[0]) ? ': a rod pinned in the middle makes a see-saw' : ''}.` : `${Name} joins ${parts.length} parts.`);
  }

  // Tapping an existing shared pin changes it instead of stacking another joint on top.
  function changeNode(node, kind) {
    const scene = A.S.scene, under = partsAt(node.p).filter((b) => !node.members.some((m) => m.body === b.id));
    const hubBody = byId(node.hub.body);
    let changes = under.length > 0;
    if (kind === 'pivot') changes = changes || node.joints.some((j) => j.kind === 'weld' || j.motor);
    if (kind === 'weld') changes = changes || node.joints.some((j) => j.kind === 'pivot');
    if (kind === 'motor') changes = !node.joints.some((j) => j.motor);
    if (!changes) {
      A.select(node.joints.find((j) => j.motor) || node.joints[0]);
      A.hint(kind === 'motor' ? 'There’s already a motor here.' : kind === 'weld' ? 'Already glued.' : 'Already pinned.');
      return;
    }
    A.checkpoint();
    if (kind === 'pivot') for (const j of node.joints) { j.kind = 'pivot'; delete j.angle0; j.motor = 0; }
    if (kind === 'weld') for (const j of node.joints) { j.kind = 'weld'; j.motor = 0; j.angle0 = P.angleOf(scene, j.b) - P.angleOf(scene, j.a); }
    if (kind === 'motor') {
      const j = node.joints[0];
      j.kind = 'pivot'; delete j.angle0; j.motor = 30; j.torque = j.torque || 50;
    }
    for (const b of under) joint(kind === 'weld' ? 'weld' : 'pivot', b, hubBody, node.p);
    A.select(node.joints[0]); A.changed();
    A.hint(kind === 'motor' ? 'Motor added to this pin.' : kind === 'weld' ? 'Glued: the parts here are now rigidly joined.' : under.length ? `Pinned ${under.length} more part${under.length > 1 ? 's' : ''} here.` : 'Now a plain pin.');
  }

  // "Fixed to the ground" on a pin's card: grounding a free pin, or freeing a ground pin so the
  // parts there turn round each other instead (a pendulum drawn to empty space becomes a hinge).
  function isGrounded(j) { const n = nodeOf(j); return !!n && n.members.some((m) => m.body == null); }
  function setGrounded(j, on) {
    const scene = A.S.scene, node = nodeOf(j);
    if (!node || isGrounded(j) === on) return;
    if (on) {
      const hub = byId(node.hub.body);
      M.addJoint(scene, { kind: 'pivot', a: endOn(hub, node.p), b: ground(node.p) });
    } else {
      const toGround = node.joints.filter((k) => k.a.body == null || k.b.body == null);
      const welded = toGround.some((k) => k.kind === 'weld');
      scene.joints = scene.joints.filter((k) => !toGround.includes(k));
      const parts = node.members.filter((m) => m.body != null);
      const hub = parts.find((m) => !isBar(byId(m.body))) || parts[0];
      for (const m of parts) {
        if (m === hub) continue;
        const linked = scene.joints.some((k) => isPin(k) && ((sameEnd(k.a, hub) && sameEnd(k.b, m)) || (sameEnd(k.a, m) && sameEnd(k.b, hub))));
        if (!linked) joint(welded ? 'weld' : 'pivot', byId(hub.body), byId(m.body), node.p);
      }
    }
    P.index(scene);
    // if the pin whose card is open was the one removed, show one that's still there
    if (!scene.joints.includes(A.S.selected)) { const n = nodeAtPoint(node.p); A.S.selected = n ? n.joints[0] : null; }
  }

  // ---------- dragging a rod's end (Select tool, a rod selected) ----------

  const endHandles = (b) => [[-1, P.toWorld(b, -b.L / 2, 0)], [1, P.toWorld(b, b.L / 2, 0)]];

  // Move end sg (-1 = A, +1 = B) of rod b to p: the other end stays put, the length changes,
  // the moved end leaves its old pin and re-attaches by the Rod-tool rules; an end that was on
  // the ground stays on the ground. Things attached along the rod keep their distance from the
  // fixed end.
  function moveEnd(b, sg, p) {
    const scene = A.S.scene;
    P.index(scene);
    const xd = (sg * b.L) / 2, xf = -xd, F = P.toWorld(b, xf, 0);
    const c = classify(p, b.id), Q = c.p, L = dist(F, Q);
    if (L < 0.03) { A.hint('That would make the rod too short.'); return; }
    A.checkpoint();
    const wasGround = scene.joints.some((j) => isPin(j) && ((j.a.body === b.id && Math.abs(j.a.x - xd) < 1e-6 && j.b.body == null) || (j.b.body === b.id && Math.abs(j.b.x - xd) < 1e-6 && j.a.body == null)));
    const fixedOnPart = scene.joints.some((j) => isPin(j) && [[j.a, j.b], [j.b, j.a]].some(([m, o]) => m.body === b.id && Math.abs(m.x - xf) < 1e-6 && o.body != null && !isBar(byId(o.body))));
    M.unpin(scene, b, xd);
    const sf = -sg, oldL = b.L;
    for (const j of scene.joints) {
      for (const k of ['a', 'b', 'p']) {
        const e = j[k];
        if (!e || e.body !== b.id) continue;
        const along = Math.max(0, Math.min(L, (e.x - xf) * sg)); // distance from the fixed end
        e.x = (sf * L) / 2 + sg * along; e.y = 0;
      }
    }
    if (Math.abs(b.mass - M.barMass(oldL)) < 1e-9) b.mass = M.barMass(L);
    b.L = L; b.x = (F[0] + Q[0]) / 2; b.y = (F[1] + Q[1]) / 2;
    b.a = sg > 0 ? Math.atan2(Q[1] - F[1], Q[0] - F[0]) : Math.atan2(F[1] - Q[1], F[0] - Q[0]);
    let t = target(c);
    if (!t && (wasGround || fixedOnPart)) t = ground(Q);
    if (t) M.addJoint(scene, { kind: 'pivot', a: { body: b.id, x: (sg * L) / 2, y: 0 }, b: t });
    A.changed();
  }

  // ---------- grabbing ----------

  // A massless rod has no mass of its own to hold, so grabbing it grabs what it's pinned to at
  // the nearer end (or the other end). Returns { body, x, y } to hold, or null.
  function grabTarget(b, w) {
    if (!b._red) return null;
    const [x] = P.toLocal(b, w[0], w[1]);
    const ends = x < 0 ? [b._red.farA, b._red.farB] : [b._red.farB, b._red.farA];
    const e = ends.find((f) => f.body != null);
    return e ? { body: e.body, x: e.x, y: e.y } : null;
  }

  // Pressing a shared pin with Select grabs the mechanism there (by the hub part).
  function nodeGrab(j) {
    const n = nodeOf(j);
    if (!n) return null;
    const parts = n.members.filter((m) => m.body != null && !byId(m.body)._red);
    const m = parts.filter((e) => !isBar(byId(e.body))).sort((e1, e2) => byId(e2.body).mass - byId(e1.body).mass)[0] || parts[0];
    return m ? { body: m.body, x: m.x, y: m.y } : null;
  }

  function init(app) { A = app; }

  ML.Rods = {
    init, partsAt, endOn, onPart, crossing, snapPoints, nodeAt, nodeOf, classify, planRod, makeRod, drawPlan,
    pinAt, isGrounded, setGrounded, endHandles, moveEnd, grabTarget, nodeGrab,
  };
})();
