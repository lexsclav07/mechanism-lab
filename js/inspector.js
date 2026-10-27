// inspector.js: the card that opens when you tap a part. Every setting has a slider for
// quick changes and a box for typing an exact value, and the card shows the part's live
// numbers in real units (N, N·m, m/s, J).
(function () {
  const ML = (window.ML = window.ML || {});
  const P = ML.Phys, M = ML.Model, R = ML.Render;
  let A, card;
  let live = []; // [element, () => text] pairs refreshed a few times a second
  let panels = []; // [element, build(element)] blocks rebuilt a few times a second (force tables, charts)

  // tiny DOM helper: h('div', {class: 'x'}, child, 'text', ...)
  function h(tag, attrs = {}, ...kids) {
    const el = document.createElement(tag);
    for (const [k, v] of Object.entries(attrs)) {
      if (k.startsWith('on')) el.addEventListener(k.slice(2), v);
      else if (v !== false && v != null) el.setAttribute(k, v === true ? '' : v);
    }
    for (const c of kids.flat()) if (c != null && c !== false) el.append(c);
    return el;
  }

  const fmt = (v, d = 2) => {
    const a = Math.abs(v), t = a >= 1000 ? a.toFixed(0) : a >= 100 ? a.toFixed(Math.max(0, d - 2)) : a >= 10 ? a.toFixed(Math.max(0, d - 1)) : a.toFixed(d);
    return (v < 0 && +t !== 0 ? '−' : '') + t;
  };
  const tidy = (v) => +(+v).toPrecision(4); // what goes in the number boxes

  // A setting: label, a number box (with its unit) and a slider underneath.
  // log = true for things that span decades (mass, stiffness, force). Each drag or edit is one undo step.
  // `toS`/`fromS` map between the stored value and the slider when they differ (weight).
  // apply: 'tweak' for settings (copied into the start state), 'change' when the edit moves or
  // re-joins things (lengths, pins), which makes the current layout the new start state.
  function field({ id, label, unit = '', min, max, step = 0.01, log = false, get, set, ends, toS = (v) => v, fromS = (v) => v, disabled = false, apply = 'tweak' }) {
    const item = A.S.selected;
    const toR = (v) => (log ? Math.log10(Math.max(v, min)) : v), fromR = (r) => (log ? 10 ** r : r);
    const box = h('input', { type: 'number', id, step: 'any', value: tidy(get()), inputmode: 'decimal', 'aria-label': `${label} (${unit})`, disabled });
    const range = h('input', { type: 'range', id: id + '-r', 'aria-label': label, tabindex: '-1', disabled,
      min: toR(min), max: toR(max), step: log ? 0.005 : step, value: toR(toS(get())) });
    let began = false;
    const begin = () => { if (!began) { A.checkpoint(); began = true; } };
    range.addEventListener('input', () => {
      let v = fromR(parseFloat(range.value));
      v = log ? +v.toPrecision(2) : +(Math.round(v / step) * step).toFixed(6);
      begin(); set(fromS(v)); box.value = tidy(get());
      apply === 'change' ? A.changed({ keepPanel: true }) : A.tweak(item, { keepPanel: true });
    });
    box.addEventListener('input', () => {
      const v = parseFloat(box.value);
      if (!Number.isFinite(v)) return;
      begin(); set(v); range.value = toR(toS(get()));
      apply === 'change' ? A.changed({ keepPanel: true }) : A.tweak(item, { keepPanel: true });
    });
    for (const el of [range, box]) el.addEventListener('change', () => { began = false; });
    return h('div', { class: 'field' },
      h('div', { class: 'f-top' }, h('label', { for: id }, label), h('span', { class: 'f-num' }, box, unit ? h('span', { class: 'unit' }, unit) : null)),
      range,
      ends ? h('div', { class: 'ends' }, h('span', {}, ends[0]), h('span', {}, ends[1])) : null);
  }

  function toggle({ id, label, get, set, apply = 'tweak' }) {
    const item = A.S.selected;
    const cb = h('input', { type: 'checkbox', id, role: 'switch', checked: !!get() });
    cb.addEventListener('change', () => { A.checkpoint(); set(cb.checked); apply === 'change' ? A.changed() : A.tweak(item); });
    return h('label', { class: 'switch', for: id }, h('span', {}, label), cb);
  }

  function choice({ label, options, get, set }) {
    const item = A.S.selected;
    return h('div', { class: 'field' }, h('span', { class: 'f-label' }, label),
      h('div', { class: 'seg', role: 'group', 'aria-label': label }, ...options.map(([v, name]) =>
        h('button', { type: 'button', 'aria-pressed': String(get() === v), onclick: () => { A.checkpoint(); set(v); A.tweak(item); } }, name))));
  }

  function fills(b) {
    const names = { grey: 'Grey', light: 'Light grey', white: 'White', dark: 'Dark grey', ink: 'Black' };
    return h('div', { class: 'swatches', role: 'group', 'aria-label': 'Fill' }, ...R.FILLS.map((k) =>
      h('button', { type: 'button', class: 'sw', 'data-f': k, title: names[k], 'aria-label': names[k], 'aria-pressed': String(b.color === k),
        onclick: () => { A.checkpoint(); b.color = k; A.tweak(b); } })));
  }

  function stat(label, fn) {
    const v = h('output', {}, fn());
    live.push([v, fn]);
    return h('div', { class: 'stat' }, h('span', {}, label), v);
  }
  const liveBox = (...rows) => h('div', { class: 'live' }, h('h3', {}, 'Live'), ...rows);
  const note = (text) => h('p', { class: 'note' }, text);
  const running = () => A.S.running;
  const fa = (j) => (j._avg ? j._avg.f : j._f || 0); // averaged over the last frame
  const ta = (j) => (j._avg ? j._avg.torque : j._torque || 0);
  const pushPull = (F) => (Math.abs(F) < 0.005 ? '0 N' : `${fmt(Math.abs(F), 1)} N ${F > 0 ? 'pull' : 'push'}`);

  // ---------- engineering mechanics read-outs ----------
  // Signs follow the textbook: x to the right, y up, angles and moments anticlockwise +.

  const Fo = () => ML.Forces;
  const panel = (build) => { const el = h('div', { class: 'mech' }); panels.push([el, build]); build(el); return el; };
  const sym = (t) => { const i = t.indexOf('_'); return h('i', { class: 'm' }, i < 0 ? t : [t.slice(0, i), h('sub', {}, t.slice(i + 1))]); };
  const num = (v, d = 1) => (Math.abs(v) < 0.005 ? '0' : fmt(v, d));
  const degs = (v) => `${num(v, 1)}°`;
  const nameOf = (b) => (b ? b.label || M.kindName(b).toLowerCase() : 'ground');
  const named = (b) => (b && b.label ? sym(b.label) : nameOf(b));
  // the analysed copy of a part or joint (paused, the read-outs come from a copy: see app.js)
  const inAn = (an, item) => (an ? (M.isBody(item) ? an._byId.get(item.id) : an.joints.find((j) => j.id === item.id)) : null);
  const nowWord = () => (A.S.running ? 'now' : 'at this instant (paused)');

  function table(head, rows, foot) {
    return h('table', { class: 'ftab' },
      h('thead', {}, h('tr', {}, ...head.map((x, i) => h('th', { scope: 'col', class: i ? 'n' : '' }, x)))),
      h('tbody', {}, ...rows.map((r) => h('tr', { class: r.cls || '' }, ...r.cells.map((c, i) => h(i ? 'td' : 'th', { class: i ? 'n' : '', scope: i ? null : 'row' }, c))))),
      foot ? h('tfoot', {}, ...foot.map((r) => h('tr', { class: r.cls || '' }, ...r.cells.map((c, i) => h(i ? 'td' : 'th', { class: i ? 'n' : '', scope: i ? null : 'row' }, c))))) : null);
  }

  // Forces on a part: its free-body diagram as a table, the sums, and Newton's second law.
  function forcesPanel(b) {
    const S = A.S;
    return panel((el) => {
      const an = A.analysis(), bz = inAn(an, b);
      if (!bz) { el.replaceChildren(note('Working out the forces…')); return; }
      const items = Fo().onBody(an, bz, an === S.scene ? S.drag : null);
      const pts = items.filter((it) => it.kind === 'reaction');
      const about = pts.find((it) => it.letter === S.momentAbout) || null;
      const q = about ? about.p : [bz.x, bz.y];
      const bal = Fo().balance(an, bz, items, q);
      const mAbout = (it) => (it.mG ?? 0) + Fo().momentAbout([bz.x, bz.y], it.f, q);
      const rows = items.map((it) => {
        const f = Fo().tb(it.f), couple = it.kind === 'couple';
        return { cells: [h('span', { class: 'fname' }, sym(it.sym), h('small', {}, it.name)), couple ? '' : num(f[0]), couple ? '' : num(f[1]), num(mAbout(it))] };
      });
      const big = Math.max(1, ...items.map((it) => Math.hypot(...it.f)));
      const still = Math.hypot(...bal.ma) < big * 0.005 && Math.abs(bal.Ia) < big * 0.005;
      const foot = [{ cls: 'sum', cells: [h('span', {}, 'Σ (total)'), num(bal.F[0]), num(bal.F[1]), num(bal.M)] }];
      if (!bz.fixed && !bz._red) foot.push({ cells: [h('span', {}, 'm·a, I·α'), num(bal.ma[0]), num(bal.ma[1]), num(bal.Ia)] });
      const status = bz.fixed ? `Fixed in place: the ground holds it with ${num(-bal.F[0])} N across, ${num(-bal.F[1])} N up and ${num(-bal.M)} N·m.`
        : bz._red ? 'A massless rod: the forces at its two ends are equal and opposite, along the rod (a two-force member).'
          : still ? 'In equilibrium: ΣFx = 0, ΣFy = 0 and ΣM = 0, so it isn’t accelerating.'
            : `Not in equilibrium: the totals equal m·a and I·α (Newton’s second law). a = ${num(Math.hypot(...bal.a), 2)} m/s², α = ${num(bal.alpha, 2)} rad/s².`;
      const pick = h('select', { 'aria-label': 'Take moments about', onchange: (e) => { S.momentAbout = e.target.value; } },
        h('option', { value: 'G' }, 'its centre of mass G'), ...pts.map((it) => h('option', { value: it.letter, selected: it === about }, `pin ${it.letter}`)));
      const keep = el.querySelector('select');
      el.replaceChildren(
        h('p', { class: 'note' }, `Forces on it ${nowWord()}, in N (x →, y ↑). Moments in N·m, anticlockwise +.`),
        table(['Force', 'Fx', 'Fy', 'M'], rows, foot),
        h('div', { class: 'about' }, h('span', {}, 'Moments about'), keep && document.activeElement === keep ? keep : pick),
        note(status));
    });
  }

  // N, V and M along a rod or beam, as three small charts (x from its left end).
  // floor: the smallest range worth drawing, so rounding noise around 0 stays a flat line
  function chart(rows, key, label, unit, floor = 1e-6) {
    const W = 268, H = 64, pad = 4, xs = rows.map((r) => r.x), ys = rows.map((r) => (Math.abs(r[key]) < floor * 0.01 ? 0 : r[key]));
    const L = xs[xs.length - 1] || 1;
    let lo = Math.min(0, ...ys), hi = Math.max(0, ...ys);
    if (hi - lo < floor) { const mid = (hi + lo) / 2; lo = Math.min(lo, mid - floor / 2); hi = Math.max(hi, mid + floor / 2); }
    const span = hi - lo;
    const X = (x) => pad + ((W - 2 * pad) * x) / L, Y = (y) => pad + ((H - 2 * pad) * (hi - y)) / span;
    const d = rows.map((r, i) => `${i ? 'L' : 'M'}${X(r.x).toFixed(1)},${Y(r[key]).toFixed(1)}`).join('');
    const area = `${d}L${X(L).toFixed(1)},${Y(0).toFixed(1)}L${X(0).toFixed(1)},${Y(0).toFixed(1)}Z`;
    let iMax = 0; ys.forEach((v, i) => { if (Math.abs(v) > Math.abs(ys[iMax])) iMax = i; });
    const svg = document.createElementNS('http://www.w3.org/2000/svg', 'svg');
    svg.setAttribute('viewBox', `0 0 ${W} ${H}`); svg.setAttribute('class', 'dgm'); svg.setAttribute('role', 'img');
    svg.setAttribute('aria-label', `${label} along the member: largest ${fmt(ys[iMax], 1)} ${unit} at ${fmt(xs[iMax], 2)} m`);
    svg.innerHTML = `<path d="${area}" class="dgm-area"/><line x1="${pad}" x2="${W - pad}" y1="${Y(0)}" y2="${Y(0)}" class="dgm-zero"/>`
      + `<path d="${d}" class="dgm-line"/><circle cx="${X(xs[iMax])}" cy="${Y(ys[iMax])}" r="3" class="dgm-dot"/>`
      + `<line class="dgm-x" y1="${pad}" y2="${H - pad}" x1="-9" x2="-9"/>`;
    const maxText = () => (Math.abs(ys[iMax]) < floor * 0.01 ? `0 ${unit} all along` : `max ${fmt(ys[iMax], 1)} ${unit} at x = ${fmt(xs[iMax], 2)} m`);
    const tip = h('span', { class: 'dgm-tip' }, maxText());
    const box = h('figure', { class: 'dgm-box' }, h('figcaption', {}, h('b', {}, label), tip), svg);
    // hover: a crosshair and the value under the pointer
    svg.addEventListener('pointermove', (e) => {
      const r = svg.getBoundingClientRect(), x = (((e.clientX - r.left) / r.width) * W - pad) / (W - 2 * pad) * L;
      let k = 0; for (let i = 0; i < xs.length; i++) if (Math.abs(xs[i] - x) < Math.abs(xs[k] - x)) k = i;
      const line = svg.querySelector('.dgm-x'); line.setAttribute('x1', X(xs[k])); line.setAttribute('x2', X(xs[k]));
      tip.textContent = `${fmt(ys[k], 1)} ${unit} at x = ${fmt(xs[k], 2)} m`;
      box.dataset.hover = '1';
    });
    svg.addEventListener('pointerleave', () => { delete box.dataset.hover; tip.textContent = maxText(); svg.querySelector('.dgm-x').setAttribute('x1', -9); svg.querySelector('.dgm-x').setAttribute('x2', -9); });
    return box;
  }

  function insidePanel(b) {
    return panel((el) => {
      if (el.querySelector('[data-hover]')) return; // don't rebuild under the pointer
      const an = A.analysis(), bz = inAn(an, b);
      if (!bz) { el.replaceChildren(); return; }
      const sec = Fo().section(an, bz, Fo().onBody(an, bz, an === A.S.scene ? A.S.drag : null));
      if (!sec) { el.replaceChildren(); return; }
      el.replaceChildren(
        h('p', { class: 'note' }, `Inside it, cut at distance x from its left end: N = axial force (+ tension), V = shear, M = bending moment (+ sagging). ${fmt(sec.L, 2)} m long.`),
        ...(() => { // one noise floor for all three: a thousandth of the biggest value in any of them
          const big = Math.max(...sec.rows.map((r) => Math.max(Math.abs(r.N), Math.abs(r.V), Math.abs(r.M)))), fl = Math.max(1e-3, big * 1e-3);
          return [chart(sec.rows, 'N', 'N, axial', 'N', fl), chart(sec.rows, 'V', 'V, shear', 'N', fl), chart(sec.rows, 'M', 'M, bending', 'N·m', fl)];
        })());
    });
  }

  // The point you tapped on a part: where it is, how it's moving, and (in a beam) the internal
  // forces at that cross-section.
  function probePanel(b) {
    return panel((el) => {
      const pr = A.S.probe;
      if (!pr || pr.body !== b.id) { el.replaceChildren(); return; }
      const an = A.analysis(), bz = inAn(an, b) || b, p = P.toWorld(b, pr.x, pr.y);
      const vb = Fo().vel(b), r = [p[0] - b.x, p[1] - b.y], v = Fo().tb([vb[0] - vb[2] * r[1], vb[1] + vb[2] * r[0]]);
      const rows = [
        ['Position', `(${num(p[0], 2)}, ${num(-p[1], 2)}) m`],
        ['Velocity', Math.hypot(...v) < 0.005 ? '0 m/s (still)' : `${num(Math.hypot(...v), 2)} m/s at ${degs(Fo().angleOfVec([v[0], -v[1]]))}`],
      ];
      if (Fo().isBeam(b) && an) {
        const sec = Fo().section(an, bz, Fo().onBody(an, bz, an === A.S.scene ? A.S.drag : null), 4);
        const s0 = sec.toS(p), x = s0 - sec.s0, at = sec.at(Math.max(sec.s0 + 1e-6, Math.min(-sec.s0 - 1e-6, s0)));
        rows.push(['Cut at x', `${num(x, 2)} m`], ['N (axial)', `${num(at.N)} N`], ['V (shear)', `${num(at.V)} N`], ['M (bending)', `${num(at.M)} N·m`]);
      }
      el.replaceChildren(h('div', { class: 'live' }, h('h3', {}, 'The point you tapped'), ...rows.map(([k, val]) => h('div', { class: 'stat' }, h('span', {}, k), h('output', {}, val)))));
    });
  }

  function studyBox(b) {
    const S = A.S;
    const kids = [
      h('div', { class: 'stat' }, h('span', {}, 'Angle θ (from horizontal)'), (() => { const o = h('output', {}, ''); live.push([o, () => degs(Fo().bodyAngle(b))]); o.textContent = degs(Fo().bodyAngle(b)); return o; })()),
      toggleOpt('fbd', 'Free-body diagram on the drawing'),
      S.opts.fbd ? toggleOpt('comp', 'Split forces into x and y parts') : null,
      forcesPanel(b),
      probePanel(b),
    ];
    if (Fo().isBeam(b)) kids.push(insidePanel(b));
    kids.push(h('button', { type: 'button', class: 'btn', onclick: () => { S.pairFrom = b; A.hint('Now tap another part to compare the two (or shift-click it next time).', 6000); } }, 'Compare with another part…'));
    return h('div', { class: 'study' }, ...kids.filter(Boolean));
  }

  // A view option (stored with the other display options), not a setting of the part.
  function toggleOpt(key, label) {
    const cb = h('input', { type: 'checkbox', id: 'i-o-' + key, role: 'switch', checked: !!A.S.opts[key] });
    cb.addEventListener('change', () => { A.S.opts[key] = cb.checked; A.saveOpts(); refresh(); });
    return h('label', { class: 'switch', for: 'i-o-' + key }, h('span', {}, label), cb);
  }

  // What the pin does to each part at it, and the support reaction if it's on the ground.
  function pinPanel(j) {
    return panel((el) => {
      const an = A.analysis(), jz = inAn(an, j);
      if (!jz) { el.replaceChildren(note('Working out the forces…')); return; }
      const pt = P.endWorld(an, jz.a), pin = Fo().atPin(an, pt);
      const rows = pin.members.filter((m) => m.body != null).map((m) => {
        const b = P.bodyOf(an, { body: m.body }), f = Fo().tb(m.f), F = Math.hypot(...f);
        // a rod pushed away from the pin is in compression, pulled towards it in tension
        let sub = M.kindName(b);
        if (b.kind === 'bar' && m.dir != null) {
          const u = [Math.cos((m.dir * Math.PI) / 180), Math.sin((m.dir * Math.PI) / 180)], ax = -(f[0] * u[0] + f[1] * u[1]);
          sub = `at ${num(m.dir, 0)}°` + (Math.abs(ax) > 0.05 ? `, ${ax > 0 ? 'T' : 'C'} ${num(Math.abs(ax))} N` : '');
        }
        return { cells: [h('span', { class: 'fname' }, b.label ? sym(b.label) : M.kindName(b).toLowerCase(), h('small', {}, sub)), num(f[0]), num(f[1]), F > 0.005 ? `${num(F)} ∠${num(Fo().angleOfVec(m.f), 0)}°` : '0'] };
      });
      const kids = [h('p', { class: 'note' }, `What the pin does to each part ${nowWord()}, in N (x →, y ↑, angle anticlockwise from →). T = the rod is in tension, C = compression.`), table(['Part', 'Fx', 'Fy', '|F| ∠'], rows)];
      // loads acting right at this joint (on one of the parts here), as in the method of joints
      const here = an.joints.filter((k) => k.kind === 'load' && Math.hypot(...P.endWorld(an, k.a).map((v, i) => v - pt[i])) < 1e-3);
      if (here.length) kids.push(h('div', { class: 'live' }, h('h3', {}, 'Loads at this joint'), ...here.map((k) => {
        const f = Fo().tb(P.loadVector(an, k));
        return h('div', { class: 'stat' }, h('span', {}, sym(k.label || 'P')), h('output', {}, `${num(f[0])} N, ${num(f[1])} N`));
      })));
      if (pin.reaction) {
        const f = Fo().tb(pin.reaction.f);
        kids.push(h('div', { class: 'live' }, h('h3', {}, 'Support reaction (ground on the structure)'),
          h('div', { class: 'stat' }, h('span', {}, sym('R_x')), h('output', {}, `${num(f[0])} N`)),
          h('div', { class: 'stat' }, h('span', {}, sym('R_y')), h('output', {}, `${num(f[1])} N`)),
          h('div', { class: 'stat' }, h('span', {}, '|R|'), h('output', {}, `${num(Math.hypot(...f))} N at ${degs(Fo().angleOfVec(pin.reaction.f))}`)),
          pin.reaction.weld ? h('div', { class: 'stat' }, h('span', {}, sym('M_R')), h('output', {}, `${num(pin.reaction.mz)} N·m`)) : null));
      }
      const dirs = pin.members.filter((m) => m.dir != null);
      if (dirs.length) {
        const ang = dirs.map((m) => h('div', { class: 'stat' }, h('span', {}, ['Direction of ', named(P.bodyOf(an, { body: m.body }))]), h('output', {}, degs(m.dir))));
        if (dirs.length === 2) ang.push(h('div', { class: 'stat' }, h('span', {}, 'Angle between them'), h('output', {}, degs(Fo().between2(dirs[0].dir, dirs[1].dir)))));
        kids.push(h('div', { class: 'live' }, h('h3', {}, 'Angles at the pin'), ...ang));
      }
      el.replaceChildren(...kids);
    });
  }

  // Two parts picked together: their angles and what one does to the other.
  function pairCard(pair) {
    const { a, b } = pair;
    return [
      h('p', { class: 'note' }, ['Comparing ', named(a), ' and ', named(b), '. Tap a part to go back to one.']),
      panel((el) => {
        const an = A.analysis(), az = inAn(an, a), bz = inAn(an, b);
        if (!az || !bz) { el.replaceChildren(note('One of these parts is gone.')); return; }
        const ta = Fo().bodyAngle(az), tbb = Fo().bodyAngle(bz), rel = Fo().deg((tbb - ta) * Math.PI / 180);
        const kids = [h('div', { class: 'live' }, h('h3', {}, 'Angles'),
          h('div', { class: 'stat' }, h('span', {}, ['θ of ', named(a)]), h('output', {}, degs(ta))),
          h('div', { class: 'stat' }, h('span', {}, ['θ of ', named(b)]), h('output', {}, degs(tbb))),
          h('div', { class: 'stat' }, h('span', {}, 'Turned relative to each other'), h('output', {}, degs(rel))),
          h('div', { class: 'stat' }, h('span', {}, 'Angle between their lines'), h('output', {}, degs(Math.min(Fo().between2(ta, tbb), 180 - Fo().between2(ta, tbb))))),
          h('div', { class: 'stat' }, h('span', {}, 'Distance between centres'), h('output', {}, `${num(Math.hypot(bz.x - az.x, bz.y - az.y), 3)} m`)),
          h('div', { class: 'stat' }, h('span', {}, 'Relative spin'), h('output', {}, `${num(-(Fo().vel(bz)[2] - Fo().vel(az)[2]), 2)} rad/s`)))];
        const bt = Fo().between(an, az, bz);
        if (bt.via.length) {
          const f = Fo().tb(bt.f), F = Math.hypot(...f);
          kids.push(h('div', { class: 'live' }, h('h3', {}, ['Force ', named(a), ' puts on ', named(b)]),
            h('div', { class: 'stat' }, h('span', {}, 'Through'), h('output', {}, bt.via.map((j) => M.kindName(j).toLowerCase()).join(', '))),
            h('div', { class: 'stat' }, h('span', {}, sym('F_x')), h('output', {}, `${num(f[0])} N`)),
            h('div', { class: 'stat' }, h('span', {}, sym('F_y')), h('output', {}, `${num(f[1])} N`)),
            h('div', { class: 'stat' }, h('span', {}, '|F|'), h('output', {}, F > 0.005 ? `${num(F)} N at ${degs(Fo().angleOfVec(bt.f))}` : '0 N')),
            Math.abs(bt.mz) > 0.005 ? h('div', { class: 'stat' }, h('span', {}, 'Moment'), h('output', {}, `${num(bt.mz)} N·m`)) : null),
            note(['And ', named(b), ' puts the same force back on ', named(a), ', the opposite way (Newton’s third law).']));
        } else if (bt.shared.length) {
          for (const sh of bt.shared) {
            const f = Fo().tb(sh.onB.f);
            kids.push(note(['They share a pin (through another part). That pin pushes on ', named(b), ` with ${num(f[0])} N across and ${num(f[1])} N up.`]));
          }
        } else kids.push(note('They aren’t joined directly, so they don’t push on each other.'));
        el.replaceChildren(...kids);
      }),
    ];
  }

  // ---------- bodies ----------

  // A rod's length changes about its middle: things pinned at its ends stay at the ends, things
  // along it keep their place (clamped to the new length). The mass follows if it was the default.
  function setBarLength(b, L) {
    const old = b.L;
    for (const j of A.S.scene.joints) {
      for (const k of ['a', 'b', 'p']) {
        const e = j[k];
        if (!e || e.body !== b.id) continue;
        e.x = Math.abs(Math.abs(e.x) - old / 2) < 1e-6 ? Math.sign(e.x) * (L / 2) : Math.max(-L / 2, Math.min(L / 2, e.x));
      }
    }
    if (Math.abs(b.mass - M.barMass(old)) < 1e-9) b.mass = M.barMass(L);
    b.L = L;
  }

  function barCard(b) {
    const scene = A.S.scene, g = () => scene.settings.gravity;
    const speed = () => Math.hypot(b.vx, b.vy), red = !!b._red;
    const heavy = Math.max(0, ...scene.joints.filter((j) => j.a.body === b.id || j.b.body === b.id)
      .map((j) => P.bodyOf(scene, j.a.body === b.id ? j.b : j.a)).filter((o) => o && o.kind !== 'bar').map((o) => o.mass));
    const kids = [
      note(`${fmt(b.L)} m long · ${red ? 'massless' : `${fmt(b.mass)} kg`}`),
      field({ id: 'i-len', label: 'Length', unit: 'm', min: 0.05, max: 10, get: () => b.L, set: (v) => setBarLength(b, Math.max(0.03, v)), apply: 'change' }),
      toggle({ id: 'i-ideal', label: 'Massless (textbook rod)', get: () => b.ideal, set: (v) => (b.ideal = v) }),
      note(!b.ideal ? 'Using its real mass.'
        : red ? 'Weightless, like a textbook rod: it only joins two things at its ends.'
          : `Can’t be massless right now (it has a free end, joins other rods, has something along it, or a motor), so it’s using its ${fmt(b.mass)} kg.`),
      field({ id: 'i-mass', label: 'Mass', unit: 'kg', min: 0.01, max: 100, log: true, disabled: red, get: () => b.mass, set: (v) => (b.mass = Math.max(0.01, v)), ends: ['light', 'heavy'] }),
    ];
    if (!red && heavy / b.mass >= 500) kids.push(note(`Something pinned to it is ${fmt(heavy / b.mass, 0)}× heavier. Above about 500× the pins start to stretch a little; a heavier rod fixes it.`));
    kids.push(
      toggle({ id: 'i-fixed', label: 'Fixed in place', get: () => b.fixed, set: (v) => { b.fixed = v; b.vx = b.vy = b.av = 0; } }),
      toggle({ id: 'i-trace', label: 'Draw its path', get: () => b.trace, set: (v) => { b.trace = v; if (!v) A.S.traces.delete(b.id); } }),
      h('button', { type: 'button', class: 'btn', onclick: () => A.follow(A.S.follow && A.S.follow.body === b.id ? null : { body: b.id, x: 0, y: 0 }) },
        A.S.follow && A.S.follow.body === b.id ? 'Stop following' : 'Follow with camera'),
      liveBox(
        stat('Force along it', () => { if (!running()) return 'press Play'; const F = P.barAxial(scene, b); return F == null ? 'n/a (pinned along it)' : pushPull(F); }),
        stat('Speed', () => `${fmt(speed())} m/s`),
        stat('Spin', () => `${fmt((b.av * 60) / (2 * Math.PI), 1)} rpm`),
        stat('Kinetic energy', () => (red ? '0 J (massless)' : `${fmt(0.5 * b.mass * speed() ** 2 + 0.5 * (b._I || 0) * b.av * b.av)} J`)),
        red ? null : stat('Weight force', () => `${fmt((b.gscale ?? 1) * b.mass * g(), 1)} N`)));
    return kids;
  }

  // Resize a circle (radius r, anchor ring as a share of r) so things fixed on its anchor ring
  // stay on their anchors and things on the rim stay on the rim; anything else keeps its place
  // as a share of the radius.
  function scaleDisc(b, r, ring) {
    const oldR = b.r, oldRing = (b.ring ?? 1) * oldR;
    for (const j of A.S.scene.joints) for (const k of ['a', 'b', 'p']) {
      const e = j[k];
      if (!e || e.body !== b.id) continue;
      const d = Math.hypot(e.x, e.y);
      if (d < 1e-9) continue;
      const f = b.anchors && Math.abs(d - oldRing) < 1e-6 ? (ring * r) / d : Math.abs(d - oldR) < 1e-6 ? r / d : r / oldR;
      e.x *= f; e.y *= f;
    }
    b.r = r; b.ring = ring;
  }

  function bodyCard(b) {
    if (b.kind === 'bar') return barCard(b);
    const g = () => A.S.scene.settings.gravity;
    const speed = () => Math.hypot(b.vx, b.vy);
    const size = b.kind === 'disc' ? `${fmt(b.r * 200, 0)} cm across` : b.kind === 'box' ? `${fmt(b.w)} m × ${fmt(b.h)} m` : `${fmt(P.shapeArea(b))} m²`;
    const kids = [
      note(size),
      fills(b),
      field({ id: 'i-mass', label: 'Mass', unit: 'kg', min: 0.05, max: 500, log: true, get: () => b.mass, set: (v) => (b.mass = Math.max(0.01, v)), ends: ['light', 'heavy'] }),
      // Weight is stored as a multiple of mass × g (so it follows the planet) but shown in newtons.
      field({ id: 'i-weight', label: 'Weight', unit: 'N', min: -3, max: 3, step: 0.05, disabled: g() === 0,
        get: () => (b.gscale ?? 1) * b.mass * g(), set: (N) => { if (g()) b.gscale = N / (b.mass * g()); },
        toS: (N) => (g() ? N / (b.mass * g()) : 1), fromS: (s) => s * b.mass * g(), ends: ['floats up', 'normal ×3'] }),
    ];
    if (g() === 0) kids.push(note('No gravity right now, so nothing weighs anything.'));
    if (b.kind === 'disc') {
      kids.push(field({ id: 'i-r', label: 'Radius', unit: 'm', min: 0.02, max: 2, get: () => b.r, set: (v) => scaleDisc(b, Math.max(0.01, v), b.ring ?? 1), apply: 'change' }));
      kids.push(choice({ label: 'Anchor points', options: [[0, 'None'], [4, '4'], [8, '8'], [16, '16'], [32, '32']], get: () => b.anchors || 0,
        set: (v) => { b.anchors = v; if (v && b.ring == null) b.ring = 1; } }));
      if (b.anchors) {
        kids.push(field({ id: 'i-ring', label: 'Anchor ring', unit: '% of r', min: 10, max: 100, step: 1, get: () => Math.round((b.ring ?? 1) * 100), set: (v) => scaleDisc(b, b.r, Math.max(0.05, Math.min(1, v / 100))), apply: 'change' }),
          note(`One every 1/${b.anchors} turn (${+(360 / b.anchors).toFixed(2)}°). Pins, rods, springs and ropes snap to them.`));
      }
    }
    if (b.kind === 'box') kids.push(toggle({ id: 'i-hollow', label: 'Hollow frame', get: () => b.hollow, set: (v) => { b.hollow = v; b.wall = Math.min(b.wall, Math.min(b.w, b.h) / 2 - 0.01); } }));
    kids.push(
      toggle({ id: 'i-fixed', label: 'Fixed in place', get: () => b.fixed, set: (v) => { b.fixed = v; b.vx = b.vy = b.av = 0; } }),
      toggle({ id: 'i-trace', label: 'Draw its path', get: () => b.trace, set: (v) => { b.trace = v; if (!v) A.S.traces.delete(b.id); } }),
      h('button', { type: 'button', class: 'btn', onclick: () => A.follow(A.S.follow && A.S.follow.body === b.id ? null : { body: b.id, x: 0, y: 0 }) },
        A.S.follow && A.S.follow.body === b.id ? 'Stop following' : 'Follow with camera'),
      liveBox(
        stat('Speed', () => `${fmt(speed())} m/s`),
        stat('Spin', () => `${fmt((b.av * 60) / (2 * Math.PI), 1)} rpm`),
        stat('Kinetic energy', () => `${fmt(0.5 * b.mass * speed() ** 2 + 0.5 * (b._I || 0) * b.av * b.av)} J`),
        stat('Weight force', () => `${fmt((b.gscale ?? 1) * b.mass * g(), 1)} N`)),
    );
    return kids;
  }

  // ---------- keyboard control ----------

  const Ctl = () => ML.Controls;
  // Free key pairs, tried in order when a part is first put on keys: [backwards/in, forwards/out].
  const PAIRS = {
    pivot: [['KeyA', 'KeyD'], ['ArrowLeft', 'ArrowRight'], ['KeyJ', 'KeyL'], ['KeyZ', 'KeyX'], ['KeyN', 'KeyM'], ['KeyF', 'KeyH']],
    piston: [['KeyS', 'KeyW'], ['ArrowDown', 'ArrowUp'], ['KeyK', 'KeyI'], ['KeyG', 'KeyT'], ['KeyH', 'KeyY'], ['KeyB', 'KeyV']],
    winch: [['KeyQ', 'KeyE'], ['KeyU', 'KeyO'], ['KeyC', 'KeyV'], ['Comma', 'Period']],
  };
  function freePair(j, kind) {
    const used = new Set(Ctl().bindings(A.S.scene).filter((b) => b.j !== j).map((b) => b.code));
    return PAIRS[kind].find((p) => !used.has(p[0]) && !used.has(p[1])) || PAIRS[kind][0];
  }

  // A key button: shows the bound key; tap it, then press the key you want.
  function keyField(j, field, label) {
    const C = Ctl(), code = j[field];
    const cap = h('button', { type: 'button', class: 'keycap' + (code ? '' : ' none'), 'aria-label': `${label}: ${code ? C.label(code) : 'no key'}. Press to change.` }, code ? C.label(code) : 'none');
    cap.addEventListener('click', () => {
      cap.textContent = 'press a key…'; cap.classList.add('listening');
      C.listen((got) => {
        if (got) { A.checkpoint(); for (const k of ['keyFwd', 'keyRev', 'keyOut', 'keyIn']) if (k !== field && j[k] === got) j[k] = ''; j[field] = got; A.tweak(j); }
        else { cap.textContent = code ? C.label(code) : 'none'; cap.classList.remove('listening'); }
      }, (bad) => A.hint(`${bad === 'Space' ? 'Space stays Play / Pause' : C.label(bad) + ' can’t be used'}: pick another key.`, 3000));
    });
    const clear = code ? h('button', { type: 'button', class: 'ib small keyx', 'aria-label': `Remove the ${label.toLowerCase()} key`, onclick: () => { A.checkpoint(); j[field] = ''; A.tweak(j); } }, '×') : null;
    const others = code ? A.S.scene.joints.filter((k) => k !== j && Ctl().bound(k).some((b) => b.code === code)) : [];
    const warn = code && C.SHORTCUTS[code] ? `Takes over ${C.label(code)}’s shortcut (${C.SHORTCUTS[code]}).` : '';
    const also = others.length ? `Also drives ${others.map((k) => (k.label ? k.label.replace('_', '') : M.kindName(k).toLowerCase())).join(', ')}.` : '';
    return h('div', { class: 'keyrow' }, h('span', {}, label), h('span', { class: 'keyset' }, cap, clear),
      warn || also ? h('small', {}, [warn, also].filter(Boolean).join(' ')) : null);
  }

  function motorKeys(j) {
    const on = !!(j.keyFwd || j.keyRev);
    const kids = [choice({ label: 'Runs', options: [['always', 'All the time'], ['keys', 'On keys']], get: () => (on ? 'keys' : 'always'),
      set: (v) => { if (v === 'keys') { const [r, f] = freePair(j, 'pivot'); j.keyRev = r; j.keyFwd = f; } else { j.keyRev = j.keyFwd = ''; } } })];
    if (on) {
      kids.push(
        keyField(j, 'keyFwd', 'Forwards'), keyField(j, 'keyRev', 'Backwards'),
        choice({ label: 'Keys work', options: [['hold', 'While held'], ['toggle', 'Press on / off']], get: () => j.keyMode, set: (v) => { j.keyMode = v; j._latch = 0; } }),
        choice({ label: 'When it stops', options: [['brake', 'Brake'], ['coast', 'Coast']], get: () => j.idle, set: (v) => (j.idle = v) }),
        note(`The speed above is its top speed; it eases up to it. Brake holds it still (up to its max torque); coast lets it spin freely.`));
    }
    return h('div', { class: 'keys' }, h('h3', { class: 'sec' }, 'Keyboard'), ...kids);
  }

  function pistonKeys(j) {
    if (j.mode !== 'keys') return null;
    return h('div', { class: 'keys' }, h('h3', { class: 'sec' }, 'Keyboard'),
      keyField(j, 'keyOut', 'Push out'), keyField(j, 'keyIn', 'Pull in'),
      note('Let go and its valve shuts: it holds where it is, up to its max force.'));
  }

  function winchKeys(j) {
    const on = !!(j.keyIn || j.keyOut);
    const kids = [toggle({ id: 'i-winch', label: 'Winch on keys', get: () => on, set: (v) => { if (v) { const [i, o] = freePair(j, 'winch'); j.keyIn = i; j.keyOut = o; } else j.keyIn = j.keyOut = ''; } })];
    if (on) {
      kids.push(keyField(j, 'keyIn', 'Reel in'), keyField(j, 'keyOut', 'Let out'),
        field({ id: 'i-winch-speed', label: 'Winch speed', unit: 'm/s', min: 0.02, max: 3, log: true, get: () => j.winch, set: (v) => (j.winch = Math.max(0.005, v)), ends: ['slow', 'fast'] }));
    }
    return h('div', { class: 'keys' }, h('h3', { class: 'sec' }, 'Keyboard'), ...kids);
  }

  // ---------- joints ----------

  function jointCard(j) {
    const scene = A.S.scene;
    const len = () => P.jointLength(scene, j);
    if (j.kind === 'spring') {
      return [
        field({ id: 'i-k', label: 'Stiffness k', unit: 'N/m', min: 1, max: 100000, log: true, get: () => j.k, set: (v) => (j.k = Math.max(0, v)), ends: ['soft', 'stiff'] }),
        field({ id: 'i-c', label: 'Damping c', unit: 'N·s/m', min: 0, max: 100, step: 0.1, get: () => j.c, set: (v) => (j.c = Math.max(0, v)), ends: ['bouncy', 'damped'] }),
        field({ id: 'i-rest', label: 'Relaxed length', unit: 'm', min: 0, max: 10, get: () => j.rest, set: (v) => (j.rest = Math.max(0, v)) }),
        h('button', { type: 'button', class: 'btn', onclick: () => { A.checkpoint(); j.rest = +len().toFixed(4); A.tweak(j); } }, 'Relax it at its current length'),
        liveBox(
          stat('Force', () => pushPull(running() ? fa(j) : j.k * (len() - j.rest))),
          stat('Stretch', () => `${fmt((len() - j.rest) * 100, 1)} cm`),
          stat('Stored energy', () => `${fmt(0.5 * j.k * (len() - j.rest) ** 2)} J`)),
      ];
    }
    if (j.kind === 'damper') {
      return [
        note('Resists the ends moving together or apart, in proportion to how fast they move. It never springs back.'),
        field({ id: 'i-c', label: 'Damping c', unit: 'N·s/m', min: 0.1, max: 10000, log: true, get: () => j.c, set: (v) => (j.c = Math.max(0, v)), ends: ['gentle', 'stiff as treacle'] }),
        liveBox(stat('Force', () => (running() ? pushPullfa(j) : 'press Play')), stat('Length', () => `${fmt(len())} m`)),
      ];
    }
    if (j.kind === 'rod') {
      return [
        note('A stiff, weightless link with a pin at each end.'),
        field({ id: 'i-len', label: 'Length', unit: 'm', min: 0.05, max: 10, get: () => j.length, set: (v) => (j.length = Math.max(0.01, v)) }),
        liveBox(stat('Force', () => (running() ? pushPullfa(j) : 'press Play'))),
      ];
    }
    if (j.kind === 'rope') {
      return [
        note('Pulls only when it’s tight. Make it longer than the gap to give it slack.'),
        field({ id: 'i-len', label: 'Length', unit: 'm', min: 0.05, max: 10, get: () => j.length, set: (v) => (j.length = Math.max(0.01, v)) }),
        liveBox(stat('Tension', () => (running() ? `${fmt(fa(j), 1)} N` : 'press Play')),
          stat('State', () => (len() < j.length - 1e-3 ? `slack by ${fmt((j.length - len()) * 100, 1)} cm` : 'tight'))),
        winchKeys(j),
      ];
    }
    if (j.kind === 'pulley') {
      return [
        field({ id: 'i-len', label: 'Rope length', unit: 'm', min: 0.1, max: 20, get: () => j.length, set: (v) => (j.length = Math.max(0.05, v)), ends: ['hoist', 'lower'] }),
        field({ id: 'i-wr', label: 'Wheel radius', unit: 'm', min: 0.03, max: 1, get: () => j.r, set: (v) => (j.r = Math.max(0.01, v)) }),
        liveBox(stat('Rope tension', () => (running() ? `${fmt(fa(j), 1)} N` : 'press Play'))),
        winchKeys(j),
      ];
    }
    if (j.kind === 'pivot') {
      const kids = [toggle({ id: 'i-motor-on', label: 'Motor', get: () => !!j.motor, set: (v) => { j.motor = v ? j._lastRpm || 30 : 0; if (!v) j._lastRpm = 0; } })];
      kids.push(...pinShared(j));
      if (j.motor) {
        kids.push(
          field({ id: 'i-rpm', label: 'Speed', unit: 'rpm', min: -300, max: 300, step: 1, get: () => j.motor, set: (v) => { j.motor = v || 0.01; j._lastRpm = j.motor; }, ends: ['backwards', 'forwards'] }),
          field({ id: 'i-torque', label: 'Max torque', unit: 'N·m', min: 0.1, max: 10000, log: true, get: () => j.torque, set: (v) => (j.torque = Math.max(0, v)), ends: ['weak', 'strong'] }),
          motorKeys(j));
      }
      kids.push(liveBox(
        stat('Pin force', () => (running() ? `${fmt(Math.abs(fa(j)), 1)} N` : 'press Play')),
        j.motor ? stat('Torque used', () => (running() ? `${fmt(Math.abs(ta(j)), 1)} of ${fmt(j.torque, 0)} N·m` : 'press Play')) : null,
        j.motor ? stat('Actual speed', () => { const a = P.bodyOf(scene, j.a), b = P.bodyOf(scene, j.b); return `${fmt((((b ? b.av : 0) - (a ? a.av : 0)) * 60) / (2 * Math.PI), 1)} rpm`; }) : null));
      return kids;
    }
    if (j.kind === 'piston') {
      return [
        choice({ label: 'Does', options: [['cycle', 'In and out'], ['extend', 'Push out'], ['retract', 'Pull in'], ['keys', 'Keys']], get: () => j.mode,
          set: (v) => { j.mode = v; if (v === 'keys' && !j.keyOut && !j.keyIn) { const [i, o] = freePair(j, 'piston'); j.keyIn = i; j.keyOut = o; } } }),
        pistonKeys(j),
        field({ id: 'i-speed', label: 'Speed', unit: 'm/s', min: 0.01, max: 5, log: true, get: () => j.speed, set: (v) => (j.speed = Math.max(0.001, v)), ends: ['slow', 'fast'] }),
        field({ id: 'i-force', label: 'Max force', unit: 'N', min: 1, max: 100000, log: true, get: () => j.force, set: (v) => (j.force = Math.max(0, v)), ends: ['weak', 'strong'] }),
        h('div', { class: 'pair' },
          field({ id: 'i-min', label: 'Shortest', unit: 'm', min: 0.05, max: 10, get: () => j.min, set: (v) => (j.min = Math.max(0.01, Math.min(v, j.max))) }),
          field({ id: 'i-max', label: 'Longest', unit: 'm', min: 0.05, max: 10, get: () => j.max, set: (v) => (j.max = Math.max(v, j.min)) })),
        liveBox(
          stat('Length', () => `${fmt(len())} m`),
          stat('Force', () => (running() ? pushPullfa(j) : 'press Play')),
          stat('Going', () => (j.mode === 'keys' ? ((j._cmd || 0) > 0.01 ? 'out' : (j._cmd || 0) < -0.01 ? 'in' : 'holding') : j.phase > 0 ? 'out' : 'in'))),
      ];
    }
    if (j.kind === 'weld') return [note('Holds two parts together as one solid piece. On one part, it’s a fixed support: it stops the part moving and turning.'), ...pinShared(j)];
    if (j.kind === 'load') {
      const comp = () => { const v = Fo().tb(P.loadVector(scene, j)); return `${fmt(v[0], 1)} N, ${fmt(v[1], 1)} N`; };
      const dirBtn = (a, label) => h('button', { type: 'button', 'aria-pressed': String(j.angle === a), onclick: () => { A.checkpoint(); j.angle = a; A.tweak(j); } }, label);
      return [
        note('A force pushing on the part at this point, like P in a statics problem. It acts the whole time, running or paused.'),
        field({ id: 'i-F', label: 'Size', unit: 'N', min: 1, max: 100000, log: true, get: () => j.F, set: (v) => (j.F = Math.max(0, v)), ends: ['small', 'big'] }),
        field({ id: 'i-ang', label: 'Direction θ', unit: '°', min: -180, max: 180, step: 1, get: () => j.angle, set: (v) => (j.angle = v), ends: ['−180°', '+180°'] }),
        h('div', { class: 'seg', role: 'group', 'aria-label': 'Quick directions' }, dirBtn(-90, '↓ down'), dirBtn(90, '↑ up'), dirBtn(180, '← left'), dirBtn(0, '→ right')),
        note('θ is measured anticlockwise from the +x axis: 0° points right, 90° up, −90° down.'),
        toggle({ id: 'i-follow', label: 'Turns with the part', get: () => j.follow, set: (v) => (j.follow = v) }),
        liveBox(stat('Components (x, y)', comp)),
      ];
    }
    if (j.kind === 'couple') {
      return [
        note('A pure turning moment (a couple) on the part. It turns it without pushing it anywhere, so where it sits on the part doesn’t matter.'),
        field({ id: 'i-M', label: 'Moment', unit: 'N·m', min: -1000, max: 1000, step: 1, get: () => j.M, set: (v) => (j.M = v), ends: ['clockwise', 'anticlockwise'] }),
        note('+ is anticlockwise, − is clockwise.'),
      ];
    }
    if (j.kind === 'slider') {
      return [
        toggle({ id: 'i-limits', label: 'End stops', get: () => j.limits, set: (v) => (j.limits = v) }),
        toggle({ id: 'i-tilt', label: 'Block can tilt', get: () => !j.lock, set: (v) => { j.lock = !v; j.angle0 = P.angleOf(scene, j.b) - P.angleOf(scene, j.a); } }),
        liveBox(stat('Position', () => `${fmt(j._s || 0)} m`), stat('Track force', () => (running() ? `${fmt(Math.abs(fa(j)), 1)} N` : 'press Play'))),
      ];
    }
    return [];
  }

  // For pins and glue: how many parts share it, and whether it's fixed to the ground.
  function pinShared(j) {
    const n = ML.Rods.nodeOf(j);
    if (!n) return [];
    const parts = n.members.filter((m) => m.body != null).length;
    return [
      parts > 2 ? note(`Shared by ${parts} parts.`) : null,
      toggle({ id: 'i-grounded', label: 'Fixed to the ground', get: () => ML.Rods.isGrounded(j), set: (v) => ML.Rods.setGrounded(j, v), apply: 'change' }),
    ].filter(Boolean);
  }

  function refresh() {
    live = []; panels = [];
    if (ML.Controls) ML.Controls.cancel(); // a key being bound is dropped when the card changes
    const it = A.S.selected;
    card.hidden = !it;
    if (!it) { card.replaceChildren(); return; }
    const isBody = M.isBody(it);
    if (it.kind === 'pair') {
      const close = h('button', { type: 'button', class: 'ib small', 'aria-label': 'Close', onclick: () => A.select(null) });
      close.innerHTML = '<svg viewBox="0 0 24 24" aria-hidden="true"><path d="M6 6l12 12M18 6 6 18" stroke="currentColor" stroke-width="2" stroke-linecap="round"/></svg>';
      card.replaceChildren(h('div', { class: 'insp-head' }, h('h2', {}, 'Two parts'), close), ...pairCard(it));
      return;
    }
    const name = h('h2', {}, M.kindName(it));
    if (it.label) {
      const i = it.label.indexOf('_');
      name.append(' ', h('span', { class: 'math' }, i < 0 ? it.label : [it.label.slice(0, i), h('sub', {}, it.label.slice(i + 1))]));
    }
    const close = h('button', { type: 'button', class: 'ib small', 'aria-label': 'Close', onclick: () => A.select(null) });
    close.innerHTML = '<svg viewBox="0 0 24 24" aria-hidden="true"><path d="M6 6l12 12M18 6 6 18" stroke="currentColor" stroke-width="2" stroke-linecap="round"/></svg>';
    const head = h('div', { class: 'insp-head' }, name, close);
    const remove = h('button', { type: 'button', class: 'btn remove',
      onclick: () => { A.checkpoint(); M.remove(A.S.scene, it); A.select(null); A.changed(); } }, 'Remove');
    // Parts and pins have two tabs: their settings, and the forces on them (statics and dynamics).
    const tabbed = isBody || it.kind === 'pivot' || it.kind === 'weld';
    const tab = tabbed ? A.S.opts.tab || 'setup' : 'setup';
    const tabs = tabbed ? h('div', { class: 'seg tabs', role: 'group', 'aria-label': 'Show' }, ...[['setup', 'Setup'], ['forces', 'Forces']].map(([k, label]) =>
      h('button', { type: 'button', 'aria-pressed': String(tab === k), onclick: () => { A.S.opts.tab = k; A.saveOpts(); refresh(); } }, label))) : null;
    const body = tab === 'forces' ? (isBody ? [studyBox(it)] : [...pinShared(it), pinPanel(it)]) : [...(isBody ? bodyCard(it) : jointCard(it)), remove];
    card.replaceChildren(head, ...(tabs ? [tabs] : []), ...body);
  }

  function tick() {
    for (const [el, fn] of live) { const t = fn(); if (el.textContent !== t) el.textContent = t; }
    for (const [el, build] of panels) build(el);
  }

  function init(app, el) { A = app; card = el; }

  ML.Inspector = { init, refresh, tick };
})();
