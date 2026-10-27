// equations.js: the whole drawing written out as equations, the way you'd set the problem up
// by hand. For every part that can move: Newton's second law (ΣFx = m·ax, ΣFy = m·ay) and
// ΣM about its centre of mass = I_G·α, first with symbols, then with today's numbers put in so
// you can check them. Plus what's given (masses, stiffnesses, loads…), the laws of each
// component (W = m·g, F = k(L − L0)…), and a count of the unknown reactions.
//
// Pins and supports get one letter each for the whole drawing (A, B, C… left to right), so the
// same pin force shows up with opposite signs on the two parts it joins (Newton's third law).
// Textbook axes: x to the right, y up, angles and moments anticlockwise.
(function () {
  const ML = (window.ML = window.ML || {});
  const P = ML.Phys;
  const F = () => ML.Forces;

  const esc = (t) => String(t).replace(/[&<>]/g, (c) => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;' })[c]);
  // a number to 3 or 4 significant figures, with a real minus sign
  const n = (v) => {
    const a = Math.abs(v), t = a < 5e-4 ? '0' : a >= 1000 ? a.toFixed(0) : a >= 100 ? a.toFixed(1) : a >= 10 ? a.toFixed(2) : a.toFixed(3);
    return (v < 0 && t !== '0' ? '−' : '') + t;
  };
  // an angle in degrees: whole numbers stay whole (−90°, not −90.00°)
  const dg = (v) => { const t = String(+Math.abs(v).toFixed(1)); return (v < 0 && t !== '0' ? '−' : '') + t; };
  const tiny = (k) => Math.abs(k) < 1e-9;
  // a symbol like 'A_x' or 'F_k1' as HTML (italic, subscript) and as plain text
  const symH = (s) => { const i = s.indexOf('_'); return i < 0 ? `<i>${esc(s)}</i>` : `<i>${esc(s.slice(0, i))}</i><sub>${esc(s.slice(i + 1))}</sub>`; };
  const symT = (s) => F().pretty(s);
  const short = (b) => (b.label ? b.label.replace('_', '') : ML.Model.kindName(b).toLowerCase() + b.id);

  // ---------- naming ----------

  // One letter per pin / support / track point in the whole drawing, left to right.
  function pinLetters(scene) {
    const pts = [];
    for (const j of scene.joints) {
      if (j.kind !== 'pivot' && j.kind !== 'weld' && j.kind !== 'slider') continue;
      const p = j.kind === 'slider' ? P.endWorld(scene, j.b) : P.endWorld(scene, j.a);
      if (!pts.some((q) => Math.hypot(q[0] - p[0], q[1] - p[1]) < 1e-4)) pts.push(p);
    }
    pts.sort((a, b) => a[0] - b[0] || a[1] - b[1]);
    const name = (i) => String.fromCharCode(65 + (i % 26)) + (i >= 26 ? Math.floor(i / 26) : '');
    return (p) => { const i = pts.findIndex((q) => Math.hypot(q[0] - p[0], q[1] - p[1]) < 1e-4); return i < 0 ? '?' : name(i); };
  }

  // Symbols for the components (springs, ropes…) so the same one reads the same on every part.
  function jointSymbols(scene) {
    const out = new Map(), count = {};
    const base = { spring: 'F_k', damper: 'F_c', rope: 'T', pulley: 'T', piston: 'F_p', belt: 'F_b', rod: 'F_r' };
    const many = (k) => scene.joints.filter((j) => base[j.kind] === base[k]).length > 1;
    for (const j of scene.joints) {
      if (j.kind === 'load') { out.set(j.id, j.label || 'P'); continue; }
      if (j.kind === 'couple') { out.set(j.id, j.label || 'C'); continue; }
      if (!base[j.kind]) continue;
      count[base[j.kind]] = (count[base[j.kind]] || 0) + 1;
      const tag = j.label && (j.kind === 'spring' || j.kind === 'damper') ? j.label.replace('_', '') : many(j.kind) ? String(count[base[j.kind]]) : '';
      const b = base[j.kind];
      out.set(j.id, tag ? (b.includes('_') ? b + tag : b + '_' + tag) : b);
    }
    return out;
  }

  // ---------- one part ----------

  // Each term: { s: symbol, c: coefficient text (e.g. 'cos 30°' or '(−1.20)'), k: coefficient
  // value, v: the symbol's value, sign: ±1 }. Its contribution is sign × k × v.
  function partEquations(scene, b, ctx) {
    const items = F().onBody(scene, b, ctx.hand), G = [b.x, b.y];
    const X = [], Y = [], Mz = [];
    const arm = (p) => [p[0] - G[0], -(p[1] - G[1])]; // from G, textbook axes
    for (const it of items) {
      const f = F().tb(it.f), p = it.p || G;
      let [rx, ry] = arm(p);
      // While parts move, the solver's moment is averaged over the frame, and the arm moves
      // during it too. Use the frame-average arm (nudged across the force) so the sums add up
      // exactly to what the solver felt. For still parts the nudge is zero.
      const f2 = f[0] * f[0] + f[1] * f[1];
      if (it.mG != null && f2 > 1e-18) {
        const dl = (it.mG - (it.mz || 0) - (rx * f[1] - ry * f[0])) / f2;
        rx += dl * f[1]; ry -= dl * f[0];
      }
      if (it.kind === 'reaction') {
        // a pin: its letter; with two members the second one gets the minus sign
        const L = ctx.letter(p), pin = ctx.pins.get(L) || { members: [] }, others = pin.members.filter((m) => m !== b.id);
        const shared = pin.members.length > 2 || (pin.members.length === 2 && pin.ground);
        const sign = !pin.ground && pin.members.length === 2 && pin.members[0] !== b.id ? -1 : 1;
        const sub = shared ? ',' + short(b) : '';
        if (it.roller) { // a roller or free track: one unknown, the push N across the track
          const j = it.roller, A = j.a.body != null ? P.bodyOf(scene, j.a) : null;
          const u = P.rotate(A ? A.a : 0, j.dir[0], j.dir[1]), nT = F().tb([-u[1], u[0]]), uT = F().tb(u);
          if (Math.abs(f[0] * uT[0] + f[1] * uT[1]) < 1e-6 * Math.max(1, Math.hypot(f[0], f[1]))) { // not at an end stop
            const up = nT[1] > 1e-9 || (tiny(nT[1]) && nT[0] > 0) ? 1 : -1, nn = [nT[0] * up, nT[1] * up];
            const th = (Math.atan2(nn[1], nn[0]) * 180) / Math.PI, Ns = `N_${L}${sub}`, v = (f[0] * nn[0] + f[1] * nn[1]) * sign;
            if (!tiny(nn[0])) X.push({ s: Ns, c: `cos ${dg(th)}°`, k: nn[0], v, sign, post: true });
            if (!tiny(nn[1])) Y.push(tiny(nn[1] - 1) ? { s: Ns, k: 1, v, sign } : { s: Ns, c: `sin ${dg(th)}°`, k: nn[1], v, sign, post: true });
            const d = rx * nn[1] - ry * nn[0];
            if (!tiny(d)) Mz.push({ s: Ns, c: `(${n(d)})`, k: d, v, sign });
            continue;
          }
        }
        const Lx = `${L}_x${sub}`, Ly = `${L}_y${sub}`, vx = f[0] * sign, vy = f[1] * sign;
        X.push({ s: Lx, k: 1, v: vx, sign });
        Y.push({ s: Ly, k: 1, v: vy, sign });
        if (Math.abs(rx) > 1e-4) Mz.push({ s: Ly, c: `(${n(rx)})`, k: rx, v: vy, sign });
        if (Math.abs(ry) > 1e-4) Mz.push({ s: Lx, c: `(${n(-ry)})`, k: -ry, v: vx, sign });
        if (Math.abs(it.mz) > 1e-9 || it.weld || it.motor) {
          const Ms = `${it.motor && !it.weld ? 'τ' : 'M'}_${L}${sub}`;
          Mz.push({ s: Ms, k: 1, v: it.mz * sign, sign });
        }
        void others;
        continue;
      }
      if (it.kind === 'couple') { Mz.push({ s: ctx.sym.get(it.j.id) || 'C', k: 1, v: it.mz, sign: 1 }); continue; }
      if (it.kind === 'weight') {
        const W = `W_${short(b)}`;
        if (tiny(f[1])) continue; // weightless
        Y.push({ s: W, k: 1, v: -f[1], sign: -1 });
        continue; // acts at G: no moment about G
      }
      // anything else acts along a direction θ: F cos θ, F sin θ, and moment F·d with d = rx sin θ − ry cos θ
      const mag = Math.hypot(f[0], f[1]);
      if (mag < 1e-9) continue;
      const th = Math.atan2(f[1], f[0]), deg = (th * 180) / Math.PI;
      const s = it.j ? ctx.sym.get(it.j.id) || it.sym : it.sym;
      // straight along an axis reads as ±F, not F cos 90°
      const ax = (k, c) => (tiny(k) ? null : tiny(Math.abs(k) - 1) ? { s, k: Math.sign(k), v: mag, sign: 1 } : { s, c, k, v: mag, sign: 1, post: true });
      const tx = ax(Math.cos(th), `cos ${dg(deg)}°`), ty = ax(Math.sin(th), `sin ${dg(deg)}°`);
      if (tx) X.push(tx);
      if (ty) Y.push(ty);
      const d = rx * Math.sin(th) - ry * Math.cos(th);
      if (Math.abs(d) > 1e-6) Mz.push({ s, c: `(${n(d)})`, k: d, v: mag, sign: 1 });
      if (it.mz) Mz.push({ s: s + "'", k: 1, v: it.mz, sign: 1 });
    }
    const bal = F().balance(scene, b, items);
    return { b, X, Y, Mz, bal, still: Math.hypot(...bal.ma) < 1e-3 * Math.max(1, b.mass * 9.81) && Math.abs(bal.Ia) < 1e-3 * Math.max(1, b.mass) };
  }

  // A sum of terms, written with symbols (sym) or with the numbers put in (num).
  function sumOf(terms, mode) {
    if (!terms.length) return '0';
    return terms.map((t, i) => {
      // a plain term (no coefficient written) shows the coefficient's sign: k = −1 reads "− P"
      const es = t.sign * (t.c ? 1 : Math.sign(t.k || 1));
      const op = i === 0 ? (es < 0 ? '−' : '') : es < 0 ? ' − ' : ' + ';
      if (mode === 'num') {
        if (!t.c) return op + (t.v < 0 ? `(${n(t.v)})` : n(t.v));
        return op + `(${t.post ? n(t.v) + '·' + n(t.k) : n(t.k) + '·' + n(t.v)})`;
      }
      const s = mode === 'html' ? symH(t.s) : symT(t.s);
      return op + (t.post ? `${s} ${t.c}` : t.c ? `${t.c} ${s}` : s);
    }).join('');
  }
  const total = (terms) => terms.reduce((a, t) => a + t.sign * t.k * t.v, 0);

  // ---------- the whole drawing ----------

  function build(scene, hand) {
    P.index(scene);
    const letter = pinLetters(scene), sym = jointSymbols(scene);
    // which parts (and the ground) meet at each pin letter
    const pins = new Map();
    for (const j of scene.joints) {
      if (j.kind !== 'pivot' && j.kind !== 'weld' && j.kind !== 'slider') continue;
      const L = letter(j.kind === 'slider' ? P.endWorld(scene, j.b) : P.endWorld(scene, j.a));
      if (!pins.has(L)) pins.set(L, { members: [], ground: false, kinds: new Set(), lock: false });
      const pin = pins.get(L);
      pin.kinds.add(j.kind); if (j.kind === 'slider' && j.lock) pin.lock = true;
      for (const e of [j.a, j.b]) {
        if (e.body == null) pin.ground = true;
        else if (!pin.members.includes(e.body)) pin.members.push(e.body);
      }
    }
    for (const pin of pins.values()) pin.members.sort((a, b) => a - b);
    const ctx = { letter, sym, pins, hand };
    const parts = scene.bodies.filter((b) => !b.fixed).map((b) => partEquations(scene, b, ctx));
    const g = scene.settings.gravity;

    // what's given and the component laws
    const given = [], laws = [];
    given.push(['g', `${n(g)} m/s²`]);
    for (const b of scene.bodies) {
      if (b.fixed) continue;
      if (b._red) { given.push([`${short(b)}`, `massless rod, ${n(b.L)} m (a two-force member)`]); continue; }
      given.push([`m_${short(b)}`, `${n(b.mass)} kg${(b.gscale ?? 1) === 0 ? ' (weightless: no W)' : ''}`], [`I_${short(b)}`, `${n(b._I || 0)} kg·m² about G`]);
      if (g && (b.gscale ?? 1) !== 0) laws.push({ s: `W_${short(b)}`, eq: `m·g = ${n(b.mass)} × ${n(g * (b.gscale ?? 1))}`, v: b.mass * g * (b.gscale ?? 1), u: 'N' });
    }
    for (const j of scene.joints) {
      const s = sym.get(j.id);
      if (j.kind === 'spring') {
        const L = P.jointLength(scene, j);
        given.push([`k (${symT(s)})`, `${n(j.k)} N/m, relaxed length ${n(j.rest)} m`]);
        laws.push({ s, eq: `k(L − L₀) = ${n(j.k)} × (${n(L)} − ${n(j.rest)})`, v: j.k * (L - j.rest), u: 'N' });
      } else if (j.kind === 'damper') {
        given.push([`c (${symT(s)})`, `${n(j.c)} N·s/m`]);
        laws.push({ s, eq: 'c × (how fast its ends separate)', v: j._avg ? j._avg.f : j._f || 0, u: 'N' });
      } else if (j.kind === 'rope' || j.kind === 'pulley') {
        given.push([`rope ${symT(s)}`, `${n(j.length)} m long`]);
        laws.push({ s, eq: 'tension (≥ 0; 0 when slack)', v: j._avg ? j._avg.f : j._f || 0, u: 'N' });
      } else if (j.kind === 'piston') {
        laws.push({ s, eq: `pushes or pulls up to ${n(j.force)} N to move at ${n(j.speed)} m/s`, v: -(j._avg ? j._avg.f : j._f || 0), u: 'N' });
      } else if (j.kind === 'load') {
        given.push([symT(s), `${n(j.F)} N at ${dg(j.angle)}°${j.follow ? ' to the part' : ''}`]);
      } else if (j.kind === 'couple') {
        given.push([symT(s), `${n(j.M)} N·m (anticlockwise +)`]);
      } else if (j.kind === 'pivot' && j.motor) {
        given.push([`motor at ${letter(P.endWorld(scene, j.a))}`, `${n(j.motor)} rpm, up to ${n(j.torque)} N·m`]);
      }
    }

    // unknown reactions: 2 per pin (x and y), +1 for glue (a moment), 1 for a track that lets the
    // part tilt (just the push across it), 2 for a locked track
    let unknowns = 0;
    const unk = [];
    for (const [L, pin] of pins) {
      const between = Math.max(1, pin.members.length - (pin.ground ? 0 : 1)); // forces in a pin joining k parts: k−1 pairs (k with the ground)
      let per = pin.kinds.has('slider') ? (pin.lock ? 2 : 1) : 2;
      if (pin.kinds.has('weld')) per += 1;
      unknowns += per * between;
      unk.push(`${L}: ${per * between}`);
    }
    const allStill = parts.every((p) => p.still);
    return { parts, given, laws, unknowns, unk, equations: parts.length * 3, allStill, sym };
  }

  // ---------- as HTML and as text ----------

  function toHTML(r) {
    const row = (lhs, rhs) => `<div class="eq"><span class="eq-l">${lhs}</span><span class="eq-r">${rhs}</span></div>`;
    let h = '';
    h += '<section><h4>Given</h4><div class="eq-given">' + r.given.map(([k, v]) => `<span>${symH(k)} = ${esc(v)}</span>`).join('') + '</div></section>';
    if (r.laws.length) h += '<section><h4>Component laws (now)</h4>' + r.laws.map((l) => row(`${symH(l.s)} = ${esc(l.eq)}`, `= ${n(l.v)} ${l.u}`)).join('') + '</section>';
    for (const p of r.parts) {
      const b = p.b, nm = b.label ? symH(b.label) : esc(short(b));
      const about = b._red ? 'massless rod, so ΣF = 0 and ΣM = 0' : `m = ${n(b.mass)} kg, I<sub>G</sub> = ${n(b._I || 0)} kg·m²${p.still ? ', not accelerating' : ''}`;
      h += `<section><h4>${nm} <small>${about}</small></h4>`;
      for (const [name, terms, rhsSym, rhsVal] of [
        ['Σ<i>F</i><sub>x</sub>', p.X, 'm·a<sub>x</sub>', p.bal.ma[0]], ['Σ<i>F</i><sub>y</sub>', p.Y, 'm·a<sub>y</sub>', p.bal.ma[1]], ['Σ<i>M</i><sub>G</sub>', p.Mz, 'I<sub>G</sub>·α', p.bal.Ia]]) {
        const lhsV = total(terms);
        h += row(`${name}: ${sumOf(terms, 'html')} = ${p.still ? '0' : rhsSym}`, '');
        h += row(`<span class="eq-num">= ${sumOf(terms, 'num')}</span>`, `<span class="eq-num">= ${n(lhsV)} ${p.still ? '' : `vs ${n(rhsVal)}`}</span>`);
      }
      h += '</section>';
    }
    h += `<section><h4>Unknowns</h4><p>${r.equations} equations (3 per moving part) and ${r.unknowns} unknown reaction components (${esc(r.unk.join(', ')) || 'none'}).`
      + (r.allStill ? (r.unknowns === r.equations ? ' Nothing is accelerating and they match: statically determinate, so you can solve every reaction from these equations.'
        : r.unknowns > r.equations ? ' More unknowns than equations: statically indeterminate (you’d need how the parts bend to split the loads).'
          : ' Fewer unknowns than equations: it’s a mechanism, so it can move.') : ' Parts are accelerating, so the m·a and I·α sides count too.') + '</p></section>';
    return h;
  }

  function toText(r) {
    const lines = ['Mechanism Lab: equations (x right, y up, moments anticlockwise about each part’s centre of mass G)', '', 'Given:'];
    for (const [k, v] of r.given) lines.push(`  ${symT(k)} = ${v}`);
    if (r.laws.length) { lines.push('', 'Component laws (now):'); for (const l of r.laws) lines.push(`  ${symT(l.s)} = ${l.eq} = ${n(l.v)} ${l.u}`); }
    for (const p of r.parts) {
      lines.push('', `${p.b.label ? symT(p.b.label) : short(p.b)}  (${p.b._red ? 'massless rod' : `m = ${n(p.b.mass)} kg, I_G = ${n(p.b._I || 0)} kg·m²`})`);
      for (const [name, terms, rhsSym, rhsVal] of [['ΣFx', p.X, 'm·ax', p.bal.ma[0]], ['ΣFy', p.Y, 'm·ay', p.bal.ma[1]], ['ΣM_G', p.Mz, 'I_G·α', p.bal.Ia]]) {
        lines.push(`  ${name}: ${sumOf(terms, 'text')} = ${p.still ? '0' : rhsSym}`);
        lines.push(`        = ${sumOf(terms, 'num')} = ${n(total(terms))}${p.still ? '' : ` (vs ${n(rhsVal)})`}`);
      }
    }
    lines.push('', `${r.equations} equations, ${r.unknowns} unknown reaction components.`);
    return lines.join('\n');
  }

  ML.Equations = { build, toHTML, toText, total };
})();
