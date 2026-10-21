// render.js: draws the scene like the textbook figure it's based on. Black line art on white,
// grey bodies, coil springs, dashpots, hatched ground supports, ⊙ pins and italic math labels
// (M_f, k, m_p ...) with live values written the same way (F = 12.4 N).
// Everything is drawn in screen pixels, so line widths stay crisp at any zoom.
(function () {
  const ML = (window.ML = window.ML || {});
  const P = ML.Phys;
  const TAU = Math.PI * 2;
  const FONT_MATH = '"STIX Two Text", "Cambria Math", "Times New Roman", serif';
  const FONT_UI = '"IBM Plex Mono", ui-monospace, monospace';

  // Camera: world metres → screen pixels. 100 px per metre at zoom 1.
  const View = {
    cx: 0, cy: 0, zoom: 1, W: 1, H: 1, userMoved: false,
    ppm() { return 100 * this.zoom; },
    w2s(x, y) { const k = this.ppm(); return [(x - this.cx) * k + this.W / 2, (y - this.cy) * k + this.H / 2]; },
    s2w(sx, sy) { const k = this.ppm(); return [(sx - this.W / 2) / k + this.cx, (sy - this.H / 2) / k + this.cy]; },
  };

  // Colours come from the page's CSS tokens.
  let C = {};
  const FILLS = ['grey', 'light', 'white', 'dark', 'ink'];
  function readColors() {
    const cs = getComputedStyle(document.documentElement);
    const g = (n) => cs.getPropertyValue(n).trim();
    C = { paper: g('--paper'), ink: g('--ink'), muted: g('--ink-2'), accent: g('--accent'),
          grid: g('--grid'), gridMajor: g('--grid-major'), fills: Object.fromEntries(FILLS.map((k) => [k, g('--f-' + k)])) };
  }

  const ws = (p) => View.w2s(p[0], p[1]);
  const norm = (x, y) => { const l = Math.hypot(x, y) || 1; return [x / l, y / l]; };
  const fillOf = (b) => C.fills[b.color] || C.fills.grey; // older saves' crayon colours become grey

  // ---------- primitives ----------

  function rectPath(ctx, b, w, h) {
    [[-w / 2, -h / 2], [w / 2, -h / 2], [w / 2, h / 2], [-w / 2, h / 2]].forEach(([lx, ly], i) => {
      const s = ws(P.toWorld(b, lx, ly));
      i ? ctx.lineTo(s[0], s[1]) : ctx.moveTo(s[0], s[1]);
    });
    ctx.closePath();
  }

  function bodyPath(ctx, b) {
    ctx.beginPath();
    if (b.kind === 'disc') {
      const s = View.w2s(b.x, b.y);
      ctx.arc(s[0], s[1], b.r * View.ppm(), 0, TAU);
    } else if (b.kind === 'box') {
      rectPath(ctx, b, b.w, b.h);
      if (b.hollow) { const [wi, hi] = P.innerSize(b); if (wi > 0 && hi > 0) rectPath(ctx, b, wi, hi); }
    } else {
      b.pts.forEach(([lx, ly], i) => { const s = ws(P.toWorld(b, lx, ly)); i ? ctx.lineTo(s[0], s[1]) : ctx.moveTo(s[0], s[1]); });
      ctx.closePath();
    }
  }

  function extent(b) {
    if (b.kind === 'disc') return b.r;
    if (b.kind === 'box') return Math.hypot(b.w, b.h) / 2;
    return Math.max(...b.pts.map(([x, y]) => Math.hypot(x, y)));
  }

  // Diagonal hatching inside the current clip region, the drafting symbol for "ground".
  function hatch(ctx, cx, cy, r, gap = 7) {
    ctx.beginPath();
    for (let d = -2 * r; d <= 2 * r; d += gap) { ctx.moveTo(cx + d - r, cy + r); ctx.lineTo(cx + d + r, cy - r); }
    ctx.lineWidth = 1; ctx.strokeStyle = C.ink; ctx.stroke();
  }

  function pin(ctx, s, r = 4.5) {
    ctx.beginPath(); ctx.arc(s[0], s[1], r, 0, TAU);
    ctx.fillStyle = C.paper; ctx.fill(); ctx.lineWidth = 1.5; ctx.strokeStyle = C.ink; ctx.stroke();
    ctx.beginPath(); ctx.arc(s[0], s[1], 1.6, 0, TAU); ctx.fillStyle = C.ink; ctx.fill();
  }

  // Pinned ground support: a triangle from the pin to a hatched base, pointing along (dx, dy).
  function support(ctx, s, dx, dy) {
    const [nx, ny] = norm(dx, dy), px = -ny, py = nx;
    const bx = s[0] + nx * 15, by = s[1] + ny * 15;
    ctx.beginPath();
    ctx.moveTo(s[0], s[1]); ctx.lineTo(bx + px * 8, by + py * 8); ctx.lineTo(bx - px * 8, by - py * 8); ctx.closePath();
    ctx.fillStyle = C.paper; ctx.fill(); ctx.lineWidth = 1.3; ctx.strokeStyle = C.ink; ctx.stroke();
    ctx.beginPath(); ctx.moveTo(bx + px * 13, by + py * 13); ctx.lineTo(bx - px * 13, by - py * 13);
    for (let t = -12; t <= 12; t += 5) {
      ctx.moveTo(bx + px * t, by + py * t);
      ctx.lineTo(bx + px * (t + 4) + nx * 6, by + py * (t + 4) + ny * 6);
    }
    ctx.lineWidth = 1.3; ctx.stroke();
  }

  // Flat wall for a spring or damper end: a line across the axis, hatched on the far side.
  function wall(ctx, s, dx, dy) {
    const [nx, ny] = norm(dx, dy), px = -ny, py = nx;
    ctx.beginPath(); ctx.moveTo(s[0] + px * 14, s[1] + py * 14); ctx.lineTo(s[0] - px * 14, s[1] - py * 14);
    for (let t = -13; t <= 13; t += 5) {
      ctx.moveTo(s[0] + px * t, s[1] + py * t);
      ctx.lineTo(s[0] + px * (t + 4) + nx * 6, s[1] + py * (t + 4) + ny * 6);
    }
    ctx.lineWidth = 1.3; ctx.strokeStyle = C.ink; ctx.stroke();
  }

  // A helical coil seen from the side. Squash it and the loops crowd together; stretch it and
  // they open into a wave, just like the springs in the reference animation.
  function coil(ctx, s1, s2, coils = 10, ry = 9) {
    const dx = s2[0] - s1[0], dy = s2[1] - s1[1], L = Math.hypot(dx, dy);
    if (L < 1) return;
    const ux = dx / L, uy = dy / L, px = -uy, py = ux;
    const lead = Math.min(14, L * 0.15), Lc = L - 2 * lead, T = coils * TAU, rx = ry * 0.55;
    const at = (u, v) => [s1[0] + ux * u + px * v, s1[1] + uy * u + py * v];
    ctx.beginPath(); ctx.moveTo(s1[0], s1[1]);
    let p = at(lead, 0); ctx.lineTo(p[0], p[1]);
    const n = coils * 28;
    for (let i = 1; i <= n; i++) {
      const t = (i / n) * T;
      p = at(lead + (Lc * t) / T + rx * (Math.cos(t) - 1), -ry * Math.sin(t));
      ctx.lineTo(p[0], p[1]);
    }
    ctx.lineTo(s2[0], s2[1]);
    ctx.lineWidth = 1.4; ctx.strokeStyle = C.ink; ctx.lineJoin = 'round'; ctx.stroke();
  }

  function dot(ctx, s, r = 2.6, color = C.ink) {
    ctx.beginPath(); ctx.arc(s[0], s[1], r, 0, TAU); ctx.fillStyle = color; ctx.fill();
  }

  // Label like "M_f" or "m_p": italic serif with a subscript, haloed so it reads over lines.
  function label(ctx, text, x, y, align = 'left', size = 16) {
    if (!text) return;
    const i = text.indexOf('_');
    const base = i < 0 ? text : text.slice(0, i), sub = i < 0 ? '' : text.slice(i + 1);
    const big = `italic ${size}px ${FONT_MATH}`, small = `italic ${Math.round(size * 0.68)}px ${FONT_MATH}`;
    ctx.font = big; const wb = ctx.measureText(base).width;
    ctx.font = small; const wsub = sub ? ctx.measureText(sub).width + 1 : 0;
    const x0 = align === 'center' ? x - (wb + wsub) / 2 : align === 'right' ? x - wb - wsub : x;
    ctx.textAlign = 'left'; ctx.textBaseline = 'middle'; ctx.lineJoin = 'round';
    ctx.lineWidth = 4; ctx.strokeStyle = C.paper; ctx.fillStyle = C.ink;
    ctx.font = big; ctx.strokeText(base, x0, y); ctx.fillText(base, x0, y);
    if (sub) { ctx.font = small; ctx.strokeText(sub, x0 + wb + 1, y + size * 0.3); ctx.fillText(sub, x0 + wb + 1, y + size * 0.3); }
    return [x0, y - size * 0.6, x0 + wb + wsub, y + size * 0.7];
  }

  // ---------- bodies ----------

  function drawBody(ctx, b) {
    bodyPath(ctx, b);
    if (b.fixed) {
      ctx.fillStyle = C.paper; ctx.fill('evenodd');
      ctx.save(); bodyPath(ctx, b); ctx.clip('evenodd');
      const s = View.w2s(b.x, b.y); hatch(ctx, s[0], s[1], extent(b) * View.ppm() + 4);
      ctx.restore(); bodyPath(ctx, b);
    } else {
      ctx.fillStyle = fillOf(b); ctx.fill('evenodd');
    }
    ctx.lineWidth = 1.6; ctx.strokeStyle = C.ink; ctx.lineJoin = 'round'; ctx.stroke();
  }

  // Which way a ground symbol at world point p should face: away from what it holds.
  function awayFrom(scene, p, other) {
    const q = other && other.body != null ? P.bodyOf(scene, other) : null;
    const o = q ? [q.x, q.y] : other ? P.endWorld(scene, other) : null;
    if (!o || Math.hypot(p[0] - o[0], p[1] - o[1]) < 1e-6) return [0, -1];
    return norm(p[0] - o[0], p[1] - o[1]);
  }

  function hidesPin(scene, e) { // a pin at the centre of a black weight is hidden inside it
    const b = P.bodyOf(scene, e);
    return b && b.kind === 'disc' && b.color === 'ink' && Math.hypot(e.x, e.y) < b.r * 0.5;
  }

  // ---------- joints ----------

  function drawRail(ctx, scene, j) {
    const A = P.bodyOf(scene, j.a), pA = P.endWorld(scene, j.a);
    const [ux, uy] = P.rotate(A ? A.a : 0, j.dir[0], j.dir[1]);
    const s0 = View.w2s(pA[0] + ux * j.lo, pA[1] + uy * j.lo), s1 = View.w2s(pA[0] + ux * j.hi, pA[1] + uy * j.hi);
    const [sx, sy] = norm(s1[0] - s0[0], s1[1] - s0[1]), nx = -sy, ny = sx;
    const ext = 18;
    ctx.save(); ctx.setLineDash([14, 3, 2, 3]); ctx.beginPath();
    ctx.moveTo(s0[0] - sx * ext, s0[1] - sy * ext); ctx.lineTo(s1[0] + sx * ext, s1[1] + sy * ext);
    ctx.lineWidth = 1; ctx.strokeStyle = C.muted; ctx.stroke(); ctx.restore();
    if (j.limits) {
      ctx.beginPath();
      for (const s of [s0, s1]) { ctx.moveTo(s[0] + nx * 5, s[1] + ny * 5); ctx.lineTo(s[0] - nx * 5, s[1] - ny * 5); }
      ctx.lineWidth = 1.2; ctx.strokeStyle = C.muted; ctx.stroke();
    }
    if (!A) { // rail on the ground: draw the hatched floor under the carriage
      const B = P.bodyOf(scene, j.b);
      let off = 0;
      if (B) {
        const [nwx, nwy] = [-uy, ux];
        if (B.kind === 'box') {
          const [ex, ey] = P.rotate(B.a, 1, 0), [fx, fy] = P.rotate(B.a, 0, 1);
          off = Math.abs(nwx * ex + nwy * ey) * B.w / 2 + Math.abs(nwx * fx + nwy * fy) * B.h / 2;
        } else off = extent(B);
        off *= View.ppm();
      }
      const a = [s0[0] - sx * (ext + 20) + nx * off, s0[1] - sy * (ext + 20) + ny * off];
      const L = Math.hypot(s1[0] - s0[0], s1[1] - s0[1]) + 2 * (ext + 20);
      ctx.beginPath(); ctx.moveTo(a[0], a[1]); ctx.lineTo(a[0] + sx * L, a[1] + sy * L);
      for (let t = 0; t <= L - 4; t += 7) {
        ctx.moveTo(a[0] + sx * t, a[1] + sy * t); ctx.lineTo(a[0] + sx * (t + 5) + nx * 6, a[1] + sy * (t + 5) + ny * 6);
      }
      ctx.lineWidth = 1.2; ctx.strokeStyle = C.ink; ctx.stroke();
    }
  }

  // Ends of a two-point part: a wall (springs, dampers) or a pinned support (rods, ropes,
  // pistons) when the end is on the ground; otherwise a dot or a pin.
  function ends(ctx, scene, j, s1, s2, ground, onBody) {
    for (const [e, s, o] of [[j.a, s1, s2], [j.b, s2, s1]]) {
      if (e.body == null) {
        ground === 'wall' ? wall(ctx, s, s[0] - o[0], s[1] - o[1]) : support(ctx, s, s[0] - o[0], s[1] - o[1]);
        if (ground !== 'wall') pin(ctx, s);
      } else if (onBody === 'pin') { if (!hidesPin(scene, e)) pin(ctx, s); } else dot(ctx, s, onBody === 'knot' ? 2.4 : 2.6);
    }
  }

  function drawSpring(ctx, scene, j) {
    const s1 = ws(P.endWorld(scene, j.a)), s2 = ws(P.endWorld(scene, j.b));
    coil(ctx, s1, s2, j.coils || 10, 9);
    ends(ctx, scene, j, s1, s2, 'wall', 'dot');
  }

  function drawRod(ctx, scene, j) {
    const s1 = ws(P.endWorld(scene, j.a)), s2 = ws(P.endWorld(scene, j.b));
    ctx.beginPath(); ctx.moveTo(s1[0], s1[1]); ctx.lineTo(s2[0], s2[1]);
    ctx.lineWidth = 3.6; ctx.lineCap = 'round'; ctx.strokeStyle = C.ink; ctx.stroke(); ctx.lineCap = 'butt';
    ends(ctx, scene, j, s1, s2, 'support', 'pin');
  }

  // A rope hangs in a curve when it's slack (approximated by a parabola with the right
  // length), and goes straight when taut.
  function drawRope(ctx, scene, j) {
    const p1 = P.endWorld(scene, j.a), p2 = P.endWorld(scene, j.b), s1 = ws(p1), s2 = ws(p2);
    const D = Math.hypot(p2[0] - p1[0], p2[1] - p1[1]);
    ctx.beginPath(); ctx.moveTo(s1[0], s1[1]);
    if (D < j.length - 1e-4 && D > 1e-6) {
      const sag = Math.sqrt((3 * D * (j.length - D)) / 8) * View.ppm();
      let [nx, ny] = norm(-(s2[1] - s1[1]), s2[0] - s1[0]);
      if (ny < 0) { nx = -nx; ny = -ny; } // sag downward
      ctx.quadraticCurveTo((s1[0] + s2[0]) / 2 + nx * 2 * sag, (s1[1] + s2[1]) / 2 + ny * 2 * sag, s2[0], s2[1]);
    } else ctx.lineTo(s2[0], s2[1]);
    ctx.lineWidth = 1.6; ctx.strokeStyle = C.ink; ctx.stroke();
    ends(ctx, scene, j, s1, s2, 'support', 'knot');
  }

  // Dashpot: a cup on one end, a plunger on the other. The plunger really slides in and out
  // of the cup as the damper's length changes.
  function drawDamper(ctx, scene, j) {
    const s1 = ws(P.endWorld(scene, j.a)), s2 = ws(P.endWorld(scene, j.b));
    const L = Math.hypot(s2[0] - s1[0], s2[1] - s1[1]);
    if (L < 2) return;
    const [ux, uy] = norm(s2[0] - s1[0], s2[1] - s1[1]), px = -uy, py = ux;
    const at = (u, v) => [s1[0] + ux * u + px * v, s1[1] + uy * u + py * v];
    const L0 = (j.len0 || L / View.ppm()) * View.ppm(), cup = Math.min(30, L0 * 0.45), hw = 8;
    const base = Math.max(4, Math.min(L0 / 2 - cup / 2, L - cup - 4));
    const plate = Math.max(base + 3, Math.min(base + cup - 2, L - L0 / 2 + cup * 0.1));
    ctx.beginPath(); // fluid
    const f0 = at(base, -hw), f1 = at(plate, hw);
    ctx.moveTo(...f0); ctx.lineTo(...at(plate, -hw)); ctx.lineTo(...f1); ctx.lineTo(...at(base, hw)); ctx.closePath();
    ctx.fillStyle = C.fills.light; ctx.fill();
    ctx.beginPath();
    ctx.moveTo(...s1); ctx.lineTo(...at(base, 0));                          // rod to the cup
    ctx.moveTo(...at(base + cup, -hw)); ctx.lineTo(...at(base, -hw));        // the cup
    ctx.lineTo(...at(base, hw)); ctx.lineTo(...at(base + cup, hw));
    ctx.moveTo(...s2); ctx.lineTo(...at(plate, 0));                         // plunger rod
    ctx.moveTo(...at(plate, -hw + 2.5)); ctx.lineTo(...at(plate, hw - 2.5)); // plunger plate
    ctx.lineWidth = 1.5; ctx.strokeStyle = C.ink; ctx.lineJoin = 'miter'; ctx.stroke();
    ends(ctx, scene, j, s1, s2, 'wall', 'dot');
  }

  // Hydraulic cylinder: the barrel rides on end a, the rod slides out to end b.
  function drawPiston(ctx, scene, j) {
    const s1 = ws(P.endWorld(scene, j.a)), s2 = ws(P.endWorld(scene, j.b));
    const L = Math.hypot(s2[0] - s1[0], s2[1] - s1[1]), k = View.ppm();
    if (L < 2) return;
    const [ux, uy] = norm(s2[0] - s1[0], s2[1] - s1[1]), px = -uy, py = ux;
    const at = (u, v) => [s1[0] + ux * u + px * v, s1[1] + uy * u + py * v];
    const bl = Math.max(16, Math.min(j.min * 0.8 * k, L - 10)), hw = 7;
    const head = Math.max(bl * 0.12, Math.min(bl * 0.9, L - (j.max * k - bl * 0.85)));
    ctx.beginPath(); // rod first, so the barrel covers the part inside it
    ctx.moveTo(...at(head, 0)); ctx.lineTo(...s2);
    ctx.lineWidth = 4; ctx.lineCap = 'butt'; ctx.strokeStyle = C.ink; ctx.stroke();
    ctx.beginPath();
    ctx.moveTo(...at(5, -hw)); ctx.lineTo(...at(bl, -hw)); ctx.lineTo(...at(bl, hw)); ctx.lineTo(...at(5, hw)); ctx.closePath();
    ctx.fillStyle = C.fills.grey; ctx.fill(); ctx.lineWidth = 1.5; ctx.stroke();
    ctx.beginPath(); ctx.moveTo(...s1); ctx.lineTo(...at(5, 0)); ctx.lineWidth = 3; ctx.stroke(); // clevis
    ctx.beginPath(); ctx.moveTo(...at(head, -hw + 2)); ctx.lineTo(...at(head, hw - 2)); // piston head
    ctx.lineWidth = 3; ctx.stroke();
    ends(ctx, scene, j, s1, s2, 'support', 'pin');
  }

  function drawPulley(ctx, scene, j) {
    const G = P.pulleyGeom(scene, j), c = ws(G.c), pA = ws(G.pA), pB = ws(G.pB);
    const r = j.r * View.ppm();
    if (j.p.body == null) { // hung from the ceiling on a bracket
      ctx.beginPath(); ctx.moveTo(c[0], c[1]); ctx.lineTo(c[0], c[1] - r - 14);
      ctx.lineWidth = 2.4; ctx.strokeStyle = C.ink; ctx.stroke();
      wall(ctx, [c[0], c[1] - r - 14], 0, -1);
    }
    ctx.beginPath(); ctx.moveTo(...pA); ctx.lineTo(...ws(G.tA));
    if (G.f1 > 1e-6 && G.f2 > 1e-6) ctx.arc(c[0], c[1], r, G.aA, G.aA + G.wind * G.wrap, G.wind < 0);
    ctx.lineTo(...pB);
    ctx.lineWidth = 1.6; ctx.strokeStyle = C.ink; ctx.stroke();
    // the wheel, with spokes that turn as the rope runs over it
    const turn = (j._d1 ?? G.f1) / j.r * -G.wind;
    ctx.beginPath(); ctx.arc(c[0], c[1], Math.max(2, r - 1), 0, TAU); ctx.fillStyle = C.fills.grey; ctx.fill();
    ctx.lineWidth = 1.5; ctx.stroke();
    ctx.beginPath();
    for (let i = 0; i < 3; i++) { const a = turn + (i * TAU) / 3; ctx.moveTo(c[0], c[1]); ctx.lineTo(c[0] + Math.cos(a) * (r - 3), c[1] + Math.sin(a) * (r - 3)); }
    ctx.lineWidth = 1.2; ctx.stroke();
    pin(ctx, c, Math.min(4.5, r * 0.45));
    for (const [e, sp] of [[j.a, pA], [j.b, pB]]) e.body == null ? support(ctx, sp, 0, 1) : dot(ctx, sp, 2.4);
  }

  function motorArrow(ctx, s, rel, rpm) {
    const r = 13, dir = Math.sign(rpm), a0 = rel, a1 = rel + dir * 4.4;
    ctx.beginPath(); ctx.arc(s[0], s[1], r, Math.min(a0, a1), Math.max(a0, a1));
    ctx.lineWidth = 1.3; ctx.strokeStyle = C.ink; ctx.stroke();
    const tx = s[0] + r * Math.cos(a1), ty = s[1] + r * Math.sin(a1);
    const [hx, hy] = [-Math.sin(a1) * dir, Math.cos(a1) * dir], [qx, qy] = [Math.cos(a1), Math.sin(a1)];
    ctx.beginPath(); ctx.moveTo(tx + hx * 5, ty + hy * 5);
    ctx.lineTo(tx + qx * 3.5, ty + qy * 3.5); ctx.lineTo(tx - qx * 3.5, ty - qy * 3.5); ctx.closePath();
    ctx.fillStyle = C.ink; ctx.fill();
  }

  function drawJointMark(ctx, scene, j) {
    if (j.kind === 'slider') {
      if (!j.lock) pin(ctx, ws(P.endWorld(scene, j.b)));
      return;
    }
    const p = P.endWorld(scene, j.a), s = ws(p);
    const ground = j.a.body == null ? j.a : j.b.body == null ? j.b : null;
    if (ground) {
      const [dx, dy] = awayFrom(scene, p, ground === j.a ? j.b : j.a);
      support(ctx, s, dx, dy);
    }
    if (j.kind === 'weld') {
      ctx.beginPath(); ctx.moveTo(s[0], s[1] - 5); ctx.lineTo(s[0] + 5, s[1]); ctx.lineTo(s[0], s[1] + 5); ctx.lineTo(s[0] - 5, s[1]); ctx.closePath();
      ctx.fillStyle = C.ink; ctx.fill();
      return;
    }
    if (j.motor) motorArrow(ctx, s, P.angleOf(scene, j.b) - P.angleOf(scene, j.a), j.motor);
    pin(ctx, s);
  }

  const DRAW = { spring: drawSpring, damper: drawDamper, rod: drawRod, rope: drawRope, piston: drawPiston, pulley: drawPulley };

  // ---------- labels and live values ----------

  function drawLabels(ctx, scene) {
    for (const b of scene.bodies) {
      if (!b.label) continue;
      // leader line from the lower-right of the outline, like the M_f callout in the figure
      let lp;
      if (b.kind === 'disc') lp = [b.x + b.r * 0.7071, b.y + b.r * 0.7071];
      else {
        const pts = b.kind === 'box'
          ? [[-b.w / 2, -b.h / 2], [b.w / 2, -b.h / 2], [b.w / 2, b.h / 2], [-b.w / 2, b.h / 2]] : b.pts;
        lp = pts.map(([x, y]) => P.toWorld(b, x, y)).reduce((m, q) => (q[0] + q[1] > m[0] + m[1] ? q : m));
      }
      const s = ws(lp);
      ctx.beginPath(); ctx.moveTo(s[0], s[1]); ctx.lineTo(s[0] + 14, s[1] + 14);
      ctx.lineWidth = 0.9; ctx.strokeStyle = C.ink; ctx.stroke();
      placed.push(label(ctx, b.label, s[0] + 16, s[1] + 20));
    }
    for (const j of scene.joints) {
      if (!j.label || !DRAW[j.kind] || j.kind === 'pulley') continue;
      const [s1, s2, nx, ny] = sides(scene, j, -1);
      const off = j.kind === 'spring' || j.kind === 'damper' ? 21 : 13;
      // beside an upright part, the text runs outward from it instead of being centred on the offset
      const align = Math.abs(nx) > 0.6 ? (nx < 0 ? 'right' : 'left') : 'center';
      placed.push(label(ctx, j.label, (s1[0] + s2[0]) / 2 + nx * off, (s1[1] + s2[1]) / 2 + ny * off, align, 15));
    }
  }

  // Screen ends of a two-point part and a unit normal: side -1 = upper/left (labels), +1 = lower/right (values).
  function sides(scene, j, side) {
    const s1 = ws(P.endWorld(scene, j.a)), s2 = ws(P.endWorld(scene, j.b));
    let [nx, ny] = norm(-(s2[1] - s1[1]), s2[0] - s1[0]);
    if ((ny < 0 || (ny === 0 && nx > 0)) === (side > 0)) { nx = -nx; ny = -ny; }
    return [s1, s2, nx, ny];
  }

  const num = (v) => {
    const a = Math.abs(v), t = a >= 100 ? a.toFixed(0) : a >= 10 ? a.toFixed(1) : a.toFixed(2);
    return (v < 0 && +t !== 0 ? '−' : '') + t;
  };

  // A live value written like the figure's annotations: "F = 12.4 N", the symbol in italics.
  // Values dodge labels and values already placed this frame (up, then down); with no room
  // they're skipped. nx: beside an upright part (|nx| large) the text runs outward from x.
  let placed = [];
  function note(ctx, sym, value, x0, y0, color = C.ink, nx = 0) {
    const i = sym.indexOf('_'), base = i < 0 ? sym : sym.slice(0, i), sub = i < 0 ? '' : sym.slice(i + 1);
    const it = `italic 14px ${FONT_MATH}`, sm = `italic 10px ${FONT_MATH}`, rm = `14px ${FONT_MATH}`;
    ctx.font = it; const w1 = ctx.measureText(base).width;
    ctx.font = sm; const w2 = sub ? ctx.measureText(sub).width + 1 : 0;
    ctx.font = rm; const rest = ` = ${value}`, w3 = ctx.measureText(rest).width;
    const w = w1 + w2 + w3, hgt = 16, x = Math.abs(nx) > 0.6 ? x0 + Math.sign(nx) * (w / 2 + 2) : x0;
    const free = (y) => placed.every((r) => x + w / 2 + 3 < r[0] || x - w / 2 - 3 > r[2] || y + hgt / 2 < r[1] || y - hgt / 2 > r[3]);
    const y = [0, -18, 18, -36, 36].map((d) => y0 + d).find(free);
    if (y == null) return;
    placed.push([x - w / 2, y - hgt / 2, x + w / 2, y + hgt / 2]);
    let cx = x - w / 2;
    ctx.textAlign = 'left'; ctx.textBaseline = 'middle'; ctx.lineJoin = 'round'; ctx.lineWidth = 4;
    ctx.strokeStyle = C.paper; ctx.fillStyle = color;
    for (const [font, text, dy, adv] of [[it, base, 0, w1], [sm, sub, 4, w2], [rm, rest, 0, w3]]) {
      if (!text) continue;
      ctx.font = font; ctx.strokeText(text, cx, y + dy); ctx.fillText(text, cx, y + dy); cx += adv;
    }
  }

  function drawValues(ctx, scene, ui) {
    const busy = scene.bodies.length + scene.joints.length > 30; // too many: only the selected part
    const show = (item) => !busy || item === ui.selected;
    const run = ui.running;
    for (const j of scene.joints) {
      if (!show(j)) continue;
      if (j.kind === 'pulley') {
        if (!run || !j._f) continue;
        const c = ws(P.endWorld(scene, j.p));
        note(ctx, 'T', `${num(j._f)} N`, c[0], c[1] + j.r * View.ppm() + 16);
        continue;
      }
      if (j.kind === 'pivot') {
        if (!j.motor) continue;
        const s = ws(P.endWorld(scene, j.a));
        if (run) note(ctx, 'τ', `${num(Math.abs(j._torque || 0))} N·m`, s[0], s[1] + 30);
        else note(ctx, 'n', `${j.motor} rpm`, s[0], s[1] + 30);
        continue;
      }
      if (!DRAW[j.kind]) continue;
      let sym = 'F', F;
      if (j.kind === 'spring') F = run ? j._f || 0 : j.k * (P.jointLength(scene, j) - j.rest); // paused: no damping part
      else if (!run) continue;
      else if (j.kind === 'rope') { sym = 'T'; F = j._f || 0; if (F < 0.05) continue; }
      else F = j._f || 0;
      if (j.kind !== 'spring' && Math.abs(F) < 0.05) continue;
      const [s1, s2, nx, ny] = sides(scene, j, 1);
      const off = j.kind === 'spring' || j.kind === 'damper' ? 22 : j.kind === 'piston' ? 20 : 14;
      note(ctx, sym, `${num(F)} N`, (s1[0] + s2[0]) / 2 + nx * off, (s1[1] + s2[1]) / 2 + ny * off, C.ink, nx);
    }
    for (const b of scene.bodies) { // speeds: weights, plus whatever is selected
      if (b.fixed || !show(b) || (b.kind !== 'disc' && b !== ui.selected)) continue;
      const v = Math.hypot(b.vx, b.vy);
      if (v < 0.01 && b !== ui.selected) continue;
      const s = View.w2s(b.x, b.y), r = (b.kind === 'disc' ? b.r : Math.min(extent(b), 0.6)) * View.ppm();
      note(ctx, 'v', `${num(v)} m/s`, s[0], s[1] - r - 14);
    }
  }

  // ---------- selection and handles ----------

  // The draggable points of a joint (handles in the editor).
  function jointHandles(scene, j) {
    if (j.kind === 'pivot' || j.kind === 'weld') return [['ab', P.endWorld(scene, j.a)]];
    if (j.kind === 'slider') return [];
    const out = [['a', P.endWorld(scene, j.a)], ['b', P.endWorld(scene, j.b)]];
    if (j.kind === 'pulley') out.push(['p', P.endWorld(scene, j.p)]);
    return out;
  }

  function jointPath(ctx, scene, j) {
    ctx.beginPath();
    if (j.kind === 'pivot' || j.kind === 'weld') { const s = ws(P.endWorld(scene, j.a)); ctx.arc(s[0], s[1], 9, 0, TAU); return; }
    if (j.kind === 'pulley') {
      const c = ws(P.endWorld(scene, j.p)), a = ws(P.endWorld(scene, j.a)), b = ws(P.endWorld(scene, j.b));
      ctx.moveTo(c[0] + j.r * View.ppm() + 4, c[1]); ctx.arc(c[0], c[1], j.r * View.ppm() + 4, 0, TAU);
      ctx.moveTo(...a); ctx.lineTo(...c); ctx.lineTo(...b);
      return;
    }
    let p1, p2;
    if (j.kind === 'slider') {
      const A = P.bodyOf(scene, j.a), pA = P.endWorld(scene, j.a), [ux, uy] = P.rotate(A ? A.a : 0, j.dir[0], j.dir[1]);
      p1 = [pA[0] + ux * j.lo, pA[1] + uy * j.lo]; p2 = [pA[0] + ux * j.hi, pA[1] + uy * j.hi];
    } else { p1 = P.endWorld(scene, j.a); p2 = P.endWorld(scene, j.b); }
    const s1 = ws(p1), s2 = ws(p2); ctx.moveTo(s1[0], s1[1]); ctx.lineTo(s2[0], s2[1]);
  }

  function highlight(ctx, scene, item, strong) {
    ctx.save();
    ctx.globalAlpha = strong ? 1 : 0.45; ctx.strokeStyle = C.accent; ctx.lineCap = 'round'; ctx.lineJoin = 'round';
    if (item.mass != null) { // a body (joints have no mass)
      bodyPath(ctx, item); ctx.lineWidth = 2.5; ctx.stroke();
    } else {
      jointPath(ctx, scene, item);
      const wide = item.kind === 'spring' || item.kind === 'damper' || item.kind === 'piston' || item.kind === 'rod';
      ctx.lineWidth = wide ? 18 : 3;
      if (wide) ctx.globalAlpha *= 0.18;
      ctx.stroke();
      if (strong) {
        ctx.globalAlpha = 1;
        for (const [, p] of jointHandles(scene, item)) {
          const s = ws(p); ctx.beginPath(); ctx.rect(s[0] - 4.5, s[1] - 4.5, 9, 9);
          ctx.fillStyle = C.paper; ctx.fill(); ctx.lineWidth = 1.6; ctx.stroke();
        }
      }
    }
    ctx.restore();
  }

  // ---------- grid, traces, overlays ----------

  function drawGrid(ctx) {
    const k = View.ppm(), step = k >= 160 ? 0.1 : k >= 60 ? 0.25 : 1;
    const [x0, y0] = View.s2w(0, 0), [x1, y1] = View.s2w(View.W, View.H);
    for (const major of [false, true]) {
      const st = major ? 1 : step;
      if (!major && step === 1) continue;
      ctx.beginPath();
      for (let x = Math.floor(x0 / st) * st; x <= x1; x += st) { const s = View.w2s(x, 0)[0]; ctx.moveTo(Math.round(s) + 0.5, 0); ctx.lineTo(Math.round(s) + 0.5, View.H); }
      for (let y = Math.floor(y0 / st) * st; y <= y1; y += st) { const s = View.w2s(0, y)[1]; ctx.moveTo(0, Math.round(s) + 0.5); ctx.lineTo(View.W, Math.round(s) + 0.5); }
      ctx.lineWidth = 1; ctx.strokeStyle = major ? C.gridMajor : C.grid; ctx.stroke();
    }
  }

  function drawTraces(ctx, traces) {
    ctx.save(); ctx.strokeStyle = C.accent; ctx.lineWidth = 1.4; ctx.lineJoin = 'round';
    for (const pts of traces.values()) {
      const n = pts.length, chunks = 10;
      for (let c = 0; c < chunks; c++) {
        const i0 = Math.floor((c * n) / chunks), i1 = Math.min(n, Math.floor(((c + 1) * n) / chunks) + 1);
        if (i1 - i0 < 2) continue;
        ctx.globalAlpha = 0.1 + (0.7 * (c + 1)) / chunks; // older = fainter
        ctx.beginPath();
        for (let i = i0; i < i1; i++) { const s = ws(pts[i]); i === i0 ? ctx.moveTo(s[0], s[1]) : ctx.lineTo(s[0], s[1]); }
        ctx.stroke();
      }
    }
    ctx.restore();
  }

  // The "hand": a dashed line from the point you grabbed to the pointer, with the pull in newtons.
  function drawHand(ctx, hand) {
    const a = ws(hand.from), b = ws(hand.to);
    ctx.save(); ctx.setLineDash([5, 4]); ctx.beginPath(); ctx.moveTo(...a); ctx.lineTo(...b);
    ctx.lineWidth = 1.4; ctx.strokeStyle = C.accent; ctx.stroke(); ctx.restore();
    dot(ctx, a, 3.2, C.accent);
    ctx.beginPath(); ctx.arc(b[0], b[1], 6, 0, TAU); ctx.lineWidth = 1.6; ctx.strokeStyle = C.accent; ctx.stroke();
    if (hand.F != null) note(ctx, 'F', `${num(hand.F)} N`, b[0] + 42, b[1] - 16, C.accent);
  }

  function drawFollow(ctx, p) {
    const s = ws(p);
    ctx.beginPath(); ctx.arc(s[0], s[1], 9, 0, TAU);
    for (const [dx, dy] of [[1, 0], [-1, 0], [0, 1], [0, -1]]) { ctx.moveTo(s[0] + dx * 6, s[1] + dy * 6); ctx.lineTo(s[0] + dx * 13, s[1] + dy * 13); }
    ctx.lineWidth = 1.4; ctx.strokeStyle = C.accent; ctx.stroke();
  }

  function drawOverlay(ctx, scene) {
    // gravity arrow, as in the reference figure (just under the top bar, on the right)
    const g = scene.settings.gravity;
    if (g !== 0) {
      const x = View.W - 40, y = 84;
      ctx.beginPath(); ctx.moveTo(x, y); ctx.lineTo(x, y + 20); ctx.lineWidth = 1.3; ctx.strokeStyle = C.ink; ctx.stroke();
      ctx.beginPath(); ctx.moveTo(x - 4, y + 13); ctx.lineTo(x, y + 21); ctx.lineTo(x + 4, y + 13); ctx.stroke();
      label(ctx, 'g', x + 8, y + 11, 'left', 17);
    }
    // scale bar, bottom left
    const k = View.ppm();
    const len = [0.05, 0.1, 0.2, 0.25, 0.5, 1, 2, 5, 10].find((m) => m * k >= 70) || 10;
    const x = 16, y = View.H - 24, w = len * k;
    ctx.beginPath(); ctx.moveTo(x, y - 4); ctx.lineTo(x, y + 4); ctx.moveTo(x, y); ctx.lineTo(x + w, y); ctx.moveTo(x + w, y - 4); ctx.lineTo(x + w, y + 4);
    ctx.lineWidth = 1.2; ctx.strokeStyle = C.ink; ctx.stroke();
    ctx.font = `12px ${FONT_UI}`; ctx.fillStyle = C.muted; ctx.textAlign = 'center'; ctx.textBaseline = 'bottom';
    ctx.fillText(len >= 1 ? `${len} m` : `${Math.round(len * 100)} cm`, x + w / 2, y - 5);
  }

  function drawSnap(ctx, p) {
    const s = ws(p);
    ctx.beginPath(); ctx.arc(s[0], s[1], 7, 0, TAU);
    ctx.moveTo(s[0] - 11, s[1]); ctx.lineTo(s[0] + 11, s[1]); ctx.moveTo(s[0], s[1] - 11); ctx.lineTo(s[0], s[1] + 11);
    ctx.lineWidth = 1.2; ctx.strokeStyle = C.accent; ctx.stroke();
  }

  // New parts pop in with a little overshoot ("ease out back"). Returns the scale for this frame.
  function popScale(ui, id) {
    const t0 = ui.spawn && ui.spawn.get(id);
    if (t0 == null) return 1;
    const t = (performance.now() - t0) / 260;
    if (t >= 1) { ui.spawn.delete(id); return 1; }
    const c = 1.9, u = t - 1;
    return Math.max(0.05, 1 + (c + 1) * u * u * u + c * u * u);
  }

  // ---------- frame ----------

  function render(ctx, scene, ui) {
    P.index(scene);
    placed = [];
    ctx.fillStyle = C.paper; ctx.fillRect(0, 0, View.W, View.H);
    if (ui.opts.grid) drawGrid(ctx);
    drawTraces(ctx, ui.traces);
    for (const j of scene.joints) if (j.kind === 'slider') drawRail(ctx, scene, j);
    for (const b of scene.bodies) {
      const k = popScale(ui, b.id);
      if (k === 1) { drawBody(ctx, b); continue; }
      const s = View.w2s(b.x, b.y);
      ctx.save(); ctx.translate(s[0], s[1]); ctx.scale(k, k); ctx.translate(-s[0], -s[1]);
      drawBody(ctx, b); ctx.restore();
    }
    for (const kind of ['spring', 'damper', 'rope', 'rod', 'piston', 'pulley']) {
      for (const j of scene.joints) if (j.kind === kind) DRAW[kind](ctx, scene, j);
    }
    for (const j of scene.joints) if (j.kind === 'pivot' || j.kind === 'weld' || j.kind === 'slider') drawJointMark(ctx, scene, j);
    if (ui.opts.labels) drawLabels(ctx, scene);
    if (ui.opts.values) drawValues(ctx, scene, ui);
    if (ui.hover && ui.hover !== ui.selected) highlight(ctx, scene, ui.hover, false);
    if (ui.selected) highlight(ctx, scene, ui.selected, true);
    if (ui.preview) ui.preview(ctx);
    if (ui.snap) drawSnap(ctx, ui.snap);
    if (ui.followPt) drawFollow(ctx, ui.followPt);
    if (ui.hand) drawHand(ctx, ui.hand);
    drawOverlay(ctx, scene);
  }

  ML.Render = {
    View, readColors, render, jointHandles, colors: () => C, FILLS,
    // primitives the editor reuses for tool previews
    bodyPath, drawBody, coil, pin, support, dot, label, ws,
  };
})();
