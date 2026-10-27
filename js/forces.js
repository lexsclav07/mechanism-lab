// forces.js: engineering-mechanics read-outs. For any part it lists every force and moment
// acting on it (its free-body diagram), checks them against Newton's second law, finds the
// reactions at pins and supports, the force one part puts on another, and the internal forces
// (axial N, shear V, bending moment M) along a rod or beam.
//
// The solver works in screen coordinates (y down, angles clockwise). Everything this module
// returns for display is in the textbook convention instead: x to the right, y UP, angles and
// moments anticlockwise positive. Raw force vectors (f) and points (p) stay in world coordinates;
// tb() converts a vector.
(function () {
  const ML = (window.ML = window.ML || {});
  const P = ML.Phys;

  const tb = (v) => [v[0], -v[1]];
  const cross = (r, f) => r[0] * f[1] - r[1] * f[0]; // world sense: + is clockwise on screen
  // Moment of force f at point p about point q, anticlockwise positive.
  const momentAbout = (p, f, q) => -cross([p[0] - q[0], p[1] - q[1]], f);
  const deg = (rad) => { let d = (rad * 180) / Math.PI; d = ((d + 180) % 360 + 360) % 360 - 180; return d === -180 ? 180 : d; };
  const angleOfVec = (v) => deg(Math.atan2(-v[1], v[0])); // direction of a world vector, textbook degrees
  const bodyAngle = (b) => deg(-b.a);
  // A body's velocity [vx, vy, av] for read-outs: averaged over the last frame once it has run.
  const vel = (b) => (b._vf && (b._vf[0] || b._vf[1] || b._vf[2]) ? b._vf : [b.vx, b.vy, b.av]);

  // "L_1" → "L₁" for plain text (subscript digits and the letters Unicode has; others just lose the _).
  const SUBS = { 0: '₀', 1: '₁', 2: '₂', 3: '₃', 4: '₄', 5: '₅', 6: '₆', 7: '₇', 8: '₈', 9: '₉', a: 'ₐ', e: 'ₑ', h: 'ₕ', i: 'ᵢ', k: 'ₖ', l: 'ₗ', m: 'ₘ', n: 'ₙ', o: 'ₒ', p: 'ₚ', r: 'ᵣ', s: 'ₛ', t: 'ₜ', u: 'ᵤ', v: 'ᵥ', x: 'ₓ' };
  const pretty = (t) => { const i = t.indexOf('_'); return i < 0 ? t : t.slice(0, i) + [...t.slice(i + 1)].map((c) => SUBS[c] || c).join(''); };
  const nameOf = (b) => (b ? (b.label ? pretty(b.label) : ML.Model.kindName(b).toLowerCase()) : 'the ground');

  // Every force (and couple) one joint puts on each thing it touches: { body (id, or null for
  // the ground), p, f, mz, mG }. mG is its whole moment about that body's centre, anticlockwise +.
  // Running, these are the solver's averages over the last frame (exact against m·a);
  // otherwise the latest values.
  function forcesFrom(scene, j) {
    const src = j._avgEnds && j._avgEnds.length ? j._avgEnds : P.endForces(scene, j).map((e) => {
      const b = e.body != null ? P.bodyOf(scene, e) : null;
      return Object.assign(e, { mc: b ? cross([e.p[0] - b.x, e.p[1] - b.y], e.f) : 0 });
    });
    return src.map((e) => ({ body: e.body, kind: e.kind, p: e.p, f: e.f.slice(), mz: e.mz, mG: e.mz - e.mc, j }));
  }

  // ---------- free-body diagram of one part ----------

  // Pins on the part get letters A, B, C… from left to right, so the card, the drawing and
  // the sums all use the same names.
  function onBody(scene, b, hand) {
    P.index(scene);
    const items = [], g = scene.settings.gravity;
    const mass = b._red ? 0 : b.mass;
    if (mass && g) items.push({ sym: 'W', name: 'Weight', kind: 'weight', p: [b.x, b.y], f: [0, mass * g * (b.gscale ?? 1)], mz: 0, mG: 0 });
    const pins = [];
    for (const j of scene.joints) {
      if (![j.a, j.b, j.p].some((e) => e && e.body === b.id)) continue;
      const sides = forcesFrom(scene, j).filter((x) => x.body === b.id);
      if (j.kind === 'load' || j.kind === 'couple') { if (sides[0]) items.push(Object.assign(sides[0], { name: j.kind === 'load' ? 'Load' : 'Applied moment', sym: j.label || (j.kind === 'load' ? 'P' : 'C') })); continue; }
      const others = [j.a, j.b, j.p].filter((e) => e && e.body !== b.id);
      const other = others.length ? P.bodyOf(scene, others[0]) : null;
      for (const s of sides) {
        s.other = other ? other.id : null;
        if (['pivot', 'weld', 'motor', 'pin', 'slider'].includes(s.kind)) pins.push(s);
        else items.push(Object.assign(s, describe(j, s.kind, other)));
      }
    }
    // join everything at the same point into one reaction (a shared pin is one pin)
    const groups = [];
    for (const s of pins) {
      const gr = groups.find((q) => Math.hypot(q.p[0] - s.p[0], q.p[1] - s.p[1]) < 1e-4 && (q.kind === 'slider') === (s.kind === 'slider'));
      if (gr) { gr.f = [gr.f[0] + s.f[0], gr.f[1] + s.f[1]]; gr.mz += s.mz; gr.mG += s.mG; gr.parts.push(s); } else groups.push({ p: s.p, f: s.f.slice(), mz: s.mz, mG: s.mG, kind: s.kind, parts: [s] });
    }
    groups.sort((q, r) => q.p[0] - r.p[0] || q.p[1] - r.p[1]);
    groups.forEach((gr, i) => {
      const L = String.fromCharCode(65 + (i % 26));
      const ground = gr.parts.some((s) => s.other == null), weld = gr.parts.some((s) => s.kind === 'weld'), motor = gr.parts.some((s) => s.kind === 'motor');
      const withs = [...new Set(gr.parts.filter((s) => s.other != null).map((s) => nameOf(P.bodyOf(scene, { body: s.other }))))];
      const lock = gr.parts.some((s) => s.j && s.j.lock);
      const what = gr.kind === 'slider' ? (ground ? (lock ? 'Track support' : 'Roller support') : 'Track')
        : ground ? (weld ? 'Fixed support' : 'Pinned support') : weld ? 'Glued joint' : 'Pin';
      items.push({ sym: 'R_' + L, msym: (weld || gr.kind === 'slider' ? 'M_' : motor ? 'τ_' : 'M_') + L, letter: L, kind: 'reaction', name: what + (withs.length ? ` with ${withs.join(', ')}` : ''),
        p: gr.p, f: gr.f, mz: gr.mz, mG: gr.mG, ground, weld, motor,
        roller: gr.kind === 'slider' && !lock ? gr.parts[0].j : null }); // a free track pushes only across itself
    });
    if (hand && hand.body === b.id) {
      const f = hand._avg || [hand._fx || 0, hand._fy || 0], p = P.toWorld(b, hand.x, hand.y);
      items.push({ sym: 'F_h', name: 'Your hand', kind: 'hand', p, f, mz: 0, mG: hand._avgMc != null ? -hand._avgMc : momentAbout(p, f, [b.x, b.y]) });
    }
    const air = scene.settings.air;
    if (air > 0 && mass && !b.fixed && Math.hypot(b.vx, b.vy) + Math.abs(b.av) > 1e-6) {
      items.push({ sym: 'D', name: 'Air resistance', kind: 'drag', p: [b.x, b.y], f: [-air * mass * b.vx, -air * mass * b.vy], mz: air * (b._I || 0) * b.av, mG: air * (b._I || 0) * b.av });
    }
    return items;
  }

  function describe(j, kind, other) {
    const tag = j.label ? j.label : '';
    const sub = tag.includes('_') ? tag.slice(tag.indexOf('_') + 1) : tag;
    switch (kind) {
      case 'spring': return { sym: tag ? `F_${tag.replace('_', '')}` : 'F_k', name: `Spring${tag ? ' ' + pretty(tag) : ''}` };
      case 'damper': return { sym: tag ? `F_${tag.replace('_', '')}` : 'F_c', name: `Damper${tag ? ' ' + pretty(tag) : ''}` };
      case 'rope': return { sym: 'T', name: 'Rope tension' };
      case 'pulley': return { sym: 'T', name: other ? 'Pulley rope' : 'Pulley rope' };
      case 'piston': return { sym: 'F_p', name: 'Piston' };
      default: return { sym: sub ? `F_${sub}` : 'F', name: ML.Model.kindName(j) };
    }
  }

  // Sum of forces, and of moments about point q (default: the centre of mass), against m·a and
  // I·α. All in textbook axes.
  function balance(scene, b, items, q) {
    q = q || [b.x, b.y];
    let fx = 0, fy = 0, m = 0;
    // about the centre each force's moment is exact (mG); about another point q add (G − q) × F
    const G = [b.x, b.y];
    for (const it of items) {
      fx += it.f[0]; fy += it.f[1];
      m += (it.mG ?? it.mz + (it.p ? momentAbout(it.p, it.f, G) : 0)) + momentAbout(G, it.f, q);
    }
    const mass = b._red || b.fixed ? 0 : b.mass;
    const ax = b._ax || 0, ay = b._ay || 0, aa = b._aa || 0;
    // m·a about q, written as the "kinetic moment": I_G·α + (r_G − q) × m·a_G
    const ma = [mass * ax, mass * ay];
    const Ia = b.fixed || b._red ? 0 : -(b._I || 0) * aa + momentAbout([b.x, b.y], ma, q);
    return { F: tb([fx, fy]), M: m, ma: tb(ma), Ia, a: tb([ax, ay]), alpha: -aa };
  }

  // ---------- pins and supports ----------

  // All the pins and glue at one point, and what the pin does to each part there.
  function atPin(scene, p) {
    P.index(scene);
    const joints = scene.joints.filter((j) => (j.kind === 'pivot' || j.kind === 'weld') && Math.hypot(...P.endWorld(scene, j.a).map((v, i) => v - p[i])) < 1e-4);
    const by = new Map();
    for (const j of joints) {
      for (const s of forcesFrom(scene, j)) {
        const k = s.body ?? 'ground';
        if (!by.has(k)) by.set(k, { body: s.body, f: [0, 0], mz: 0 });
        const m = by.get(k); m.f[0] += s.f[0]; m.f[1] += s.f[1]; m.mz += s.mz;
      }
    }
    const members = [...by.values()];
    const ground = members.find((m) => m.body == null);
    // the support reaction is what the ground does to the structure: minus what it takes
    const reaction = ground ? { f: [-ground.f[0], -ground.f[1]], mz: -ground.mz, weld: joints.some((j) => j.kind === 'weld' && (j.a.body == null || j.b.body == null)) } : null;
    // which way each part leaves the pin (rods: towards their far end; others: their centre)
    for (const m of members) {
      const b = m.body != null ? P.bodyOf(scene, { body: m.body }) : null;
      if (!b) continue;
      let d;
      if (b.kind === 'bar') {
        const e1 = P.toWorld(b, -b.L / 2, 0), e2 = P.toWorld(b, b.L / 2, 0);
        const far = Math.hypot(e1[0] - p[0], e1[1] - p[1]) > Math.hypot(e2[0] - p[0], e2[1] - p[1]) ? e1 : e2;
        d = [far[0] - p[0], far[1] - p[1]];
      } else d = [b.x - p[0], b.y - p[1]];
      if (Math.hypot(...d) > 1e-6) m.dir = angleOfVec(d);
    }
    return { p, joints, members, reaction };
  }

  // The angle between two directions (degrees), 0 to 180.
  const between2 = (a, b) => { const d = Math.abs(((a - b) % 360 + 540) % 360 - 180); return d; };

  // ---------- two parts ----------

  // What part A does to part B through the joints between them (and any pin they share).
  function between(scene, A, B) {
    P.index(scene);
    let f = [0, 0], mz = 0, p = null;
    const via = [];
    for (const j of scene.joints) {
      const ends = [j.a, j.b, j.p].filter(Boolean);
      if (!ends.some((e) => e.body === A.id) || !ends.some((e) => e.body === B.id)) continue;
      for (const s of forcesFrom(scene, j)) {
        if (s.body !== B.id) continue;
        f = [f[0] + s.f[0], f[1] + s.f[1]]; mz += s.mz; p = p || s.p;
      }
      via.push(j);
    }
    // parts that share a pin through a third part (the pin's hub) still push on each other there
    const shared = [];
    if (!via.length) {
      for (const j of scene.joints) {
        if (j.kind !== 'pivot' && j.kind !== 'weld') continue;
        const q = P.endWorld(scene, j.a);
        if (shared.some((s) => Math.hypot(s.p[0] - q[0], s.p[1] - q[1]) < 1e-4)) continue;
        const pin = atPin(scene, q), mb = pin.members.find((m) => m.body === B.id);
        if (mb && pin.members.some((m) => m.body === A.id)) shared.push({ p: q, onB: mb });
      }
    }
    return { via, f, mz, p, shared, relAngle: deg(-(B.a - A.a)) };
  }

  // ---------- inside a rod or beam: N, V, M ----------

  // The beam axis: a rod's own line; a block's longer side. It runs left to right on screen
  // (bottom to top for an upright one), like a textbook beam.
  function beamAxis(b) {
    let t = b.kind === 'box' && b.h > b.w ? P.rotate(b.a, 0, 1) : P.rotate(b.a, 1, 0);
    if (t[0] < -1e-6 || (Math.abs(t[0]) <= 1e-6 && t[1] > 0)) t = [-t[0], -t[1]];
    const L = b.kind === 'bar' ? b.L : b.kind === 'box' ? Math.max(b.w, b.h) : 0;
    return { t, L };
  }
  const isBeam = (b) => b.kind === 'bar' || b.kind === 'box';

  // Internal forces at sections along the member, from the left end: cut it there and the left
  // piece must balance. N is + in tension, V + when it pushes the left piece's cut face down,
  // M + sagging (anticlockwise on the left piece's cut face). Weight is spread along the length,
  // and while it moves so is the "m·a" inertia force (d'Alembert), so it balances either way.
  function section(scene, b, items, n = 160) {
    const { t, L } = beamAxis(b);
    if (!L) return null;
    const c = [b.x, b.y], mass = b._red ? 0 : b.mass, g = scene.settings.gravity * (b.gscale ?? 1);
    const ax = b._ax || 0, ay = b._ay || 0, aa = b._aa || 0, w = b.av || 0, air = scene.settings.air || 0;
    const rho = mass / L;
    // distributed load per metre, q(s) = A0 + A1 s (s measured along t from the centre)
    const A0 = [rho * (0 - ax - air * b.vx), rho * (g - ay - air * b.vy)];
    const k = [-aa * t[1] - w * w * t[0], aa * t[0] - w * w * t[1]];
    const A1 = [-rho * k[0], -rho * k[1]];
    const pts = items.filter((it) => it.kind !== 'weight' && it.kind !== 'drag').map((it) => {
      const r = it.p ? [it.p[0] - c[0], it.p[1] - c[1]] : [0, 0];
      return { s: r[0] * t[0] + r[1] * t[1], p: it.p || c, f: it.f, mz: it.mz };
    });
    const s0 = -L / 2;
    const at = (s) => {
      let R = [0, 0], Mw = 0, Mc = 0; // Mw: world-sense moment of forces about the cut; Mc: couples (anticlockwise)
      const rs = [c[0] + s * t[0], c[1] + s * t[1]];
      for (const q of pts) {
        if (q.s >= s) continue;
        R[0] += q.f[0]; R[1] += q.f[1];
        Mw += cross([q.p[0] - rs[0], q.p[1] - rs[1]], q.f); Mc += q.mz;
      }
      const ds = s - s0;
      R[0] += A0[0] * ds + (A1[0] * (s * s - s0 * s0)) / 2;
      R[1] += A0[1] * ds + (A1[1] * (s * s - s0 * s0)) / 2;
      // ∫ (s'−s) q(s') ds' from s0 to s, then crossed with t
      const u0 = s0 - s, I = [-(A0[0] + A1[0] * s) * u0 * u0 / 2 - (A1[0] * u0 * u0 * u0) / 3, -(A0[1] + A1[1] * s) * u0 * u0 / 2 - (A1[1] * u0 * u0 * u0) / 3];
      Mw += cross(t, I);
      // internal force and moment on the left piece = minus everything else on it
      const Rint = [-R[0], -R[1]];
      return { N: Rint[0] * t[0] + Rint[1] * t[1], V: -(Rint[0] * t[1] - Rint[1] * t[0]), M: Mw - Mc };
    };
    const ss = [];
    for (let i = 0; i <= n; i++) ss.push(s0 + (L * i) / n);
    for (const q of pts) if (q.s > s0 + 1e-6 && q.s < -s0 - 1e-6) ss.push(q.s - 1e-7, q.s + 1e-7);
    ss.sort((x, y) => x - y);
    const eps = L * 1e-6;
    const rows = ss.map((s) => Object.assign({ x: s - s0 }, at(Math.min(-s0 - eps, Math.max(s0 + eps, s)))));
    return { L, t, rows, at: (s) => at(s), toS: (p) => (p[0] - c[0]) * t[0] + (p[1] - c[1]) * t[1], s0 };
  }

  // ---------- forces while paused ----------

  // Paused, nothing has been worked out for the current pose, so take one frame on a copy:
  // the forces at this instant (what pressing Play would start with). For a structure that
  // can't move, those are the static loads.
  function snapshot(scene) {
    const z = ML.Model.load(ML.Model.serialize(scene));
    z._wantEnds = true;
    P.step(z, 1 / 60);
    return z;
  }

  ML.Forces = { pretty, vel, forcesFrom, onBody, balance, atPin, between, between2, section, isBeam, beamAxis, snapshot, tb, momentAbout, angleOfVec, bodyAngle, deg, nameOf };
})();
