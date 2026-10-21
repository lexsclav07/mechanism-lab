// inspector.js: the card that opens when you tap a part. Every setting has a slider for
// quick changes and a box for typing an exact value, and the card shows the part's live
// numbers in real units (N, N·m, m/s, J).
(function () {
  const ML = (window.ML = window.ML || {});
  const P = ML.Phys, M = ML.Model, R = ML.Render;
  let A, card;
  let live = []; // [element, () => text] pairs refreshed a few times a second

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
  const pushPull = (F) => (Math.abs(F) < 0.005 ? '0 N' : `${fmt(Math.abs(F), 1)} N ${F > 0 ? 'pull' : 'push'}`);

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
    if (b.kind === 'disc') kids.push(field({ id: 'i-r', label: 'Radius', unit: 'm', min: 0.02, max: 2, get: () => b.r, set: (v) => (b.r = Math.max(0.01, v)) }));
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
          stat('Force', () => pushPull(running() ? j._f || 0 : j.k * (len() - j.rest))),
          stat('Stretch', () => `${fmt((len() - j.rest) * 100, 1)} cm`),
          stat('Stored energy', () => `${fmt(0.5 * j.k * (len() - j.rest) ** 2)} J`)),
      ];
    }
    if (j.kind === 'damper') {
      return [
        note('Resists the ends moving together or apart, in proportion to how fast they move. It never springs back.'),
        field({ id: 'i-c', label: 'Damping c', unit: 'N·s/m', min: 0.1, max: 10000, log: true, get: () => j.c, set: (v) => (j.c = Math.max(0, v)), ends: ['gentle', 'stiff as treacle'] }),
        liveBox(stat('Force', () => (running() ? pushPull(j._f || 0) : 'press Play')), stat('Length', () => `${fmt(len())} m`)),
      ];
    }
    if (j.kind === 'rod') {
      return [
        note('A stiff, weightless link with a pin at each end.'),
        field({ id: 'i-len', label: 'Length', unit: 'm', min: 0.05, max: 10, get: () => j.length, set: (v) => (j.length = Math.max(0.01, v)) }),
        liveBox(stat('Force', () => (running() ? pushPull(j._f || 0) : 'press Play'))),
      ];
    }
    if (j.kind === 'rope') {
      return [
        note('Pulls only when it’s tight. Make it longer than the gap to give it slack.'),
        field({ id: 'i-len', label: 'Length', unit: 'm', min: 0.05, max: 10, get: () => j.length, set: (v) => (j.length = Math.max(0.01, v)) }),
        liveBox(stat('Tension', () => (running() ? `${fmt(j._f || 0, 1)} N` : 'press Play')),
          stat('State', () => (len() < j.length - 1e-3 ? `slack by ${fmt((j.length - len()) * 100, 1)} cm` : 'tight'))),
      ];
    }
    if (j.kind === 'pulley') {
      return [
        field({ id: 'i-len', label: 'Rope length', unit: 'm', min: 0.1, max: 20, get: () => j.length, set: (v) => (j.length = Math.max(0.05, v)), ends: ['hoist', 'lower'] }),
        field({ id: 'i-wr', label: 'Wheel radius', unit: 'm', min: 0.03, max: 1, get: () => j.r, set: (v) => (j.r = Math.max(0.01, v)) }),
        liveBox(stat('Rope tension', () => (running() ? `${fmt(j._f || 0, 1)} N` : 'press Play'))),
      ];
    }
    if (j.kind === 'pivot') {
      const kids = [toggle({ id: 'i-motor-on', label: 'Motor', get: () => !!j.motor, set: (v) => { j.motor = v ? j._lastRpm || 30 : 0; if (!v) j._lastRpm = 0; } })];
      kids.push(...pinShared(j));
      if (j.motor) {
        kids.push(
          field({ id: 'i-rpm', label: 'Speed', unit: 'rpm', min: -300, max: 300, step: 1, get: () => j.motor, set: (v) => { j.motor = v || 0.01; j._lastRpm = j.motor; }, ends: ['backwards', 'forwards'] }),
          field({ id: 'i-torque', label: 'Max torque', unit: 'N·m', min: 0.1, max: 10000, log: true, get: () => j.torque, set: (v) => (j.torque = Math.max(0, v)), ends: ['weak', 'strong'] }));
      }
      kids.push(liveBox(
        stat('Pin force', () => (running() ? `${fmt(Math.abs(j._f || 0), 1)} N` : 'press Play')),
        j.motor ? stat('Torque used', () => (running() ? `${fmt(Math.abs(j._torque || 0), 1)} of ${fmt(j.torque, 0)} N·m` : 'press Play')) : null,
        j.motor ? stat('Actual speed', () => { const a = P.bodyOf(scene, j.a), b = P.bodyOf(scene, j.b); return `${fmt((((b ? b.av : 0) - (a ? a.av : 0)) * 60) / (2 * Math.PI), 1)} rpm`; }) : null));
      return kids;
    }
    if (j.kind === 'piston') {
      return [
        choice({ label: 'Does', options: [['cycle', 'In and out'], ['extend', 'Push out'], ['retract', 'Pull in']], get: () => j.mode, set: (v) => (j.mode = v) }),
        field({ id: 'i-speed', label: 'Speed', unit: 'm/s', min: 0.01, max: 5, log: true, get: () => j.speed, set: (v) => (j.speed = Math.max(0.001, v)), ends: ['slow', 'fast'] }),
        field({ id: 'i-force', label: 'Max force', unit: 'N', min: 1, max: 100000, log: true, get: () => j.force, set: (v) => (j.force = Math.max(0, v)), ends: ['weak', 'strong'] }),
        h('div', { class: 'pair' },
          field({ id: 'i-min', label: 'Shortest', unit: 'm', min: 0.05, max: 10, get: () => j.min, set: (v) => (j.min = Math.max(0.01, Math.min(v, j.max))) }),
          field({ id: 'i-max', label: 'Longest', unit: 'm', min: 0.05, max: 10, get: () => j.max, set: (v) => (j.max = Math.max(v, j.min)) })),
        liveBox(
          stat('Length', () => `${fmt(len())} m`),
          stat('Force', () => (running() ? pushPull(j._f || 0) : 'press Play')),
          stat('Going', () => (j.phase > 0 ? 'out' : 'in'))),
      ];
    }
    if (j.kind === 'weld') return [note('Holds two parts together as one solid piece.'), ...pinShared(j), liveBox(stat('Glue force', () => (running() ? `${fmt(Math.abs(j._f || 0), 1)} N` : 'press Play')))];
    if (j.kind === 'slider') {
      return [
        toggle({ id: 'i-limits', label: 'End stops', get: () => j.limits, set: (v) => (j.limits = v) }),
        toggle({ id: 'i-tilt', label: 'Block can tilt', get: () => !j.lock, set: (v) => { j.lock = !v; j.angle0 = P.angleOf(scene, j.b) - P.angleOf(scene, j.a); } }),
        liveBox(stat('Position', () => `${fmt(j._s || 0)} m`), stat('Track force', () => (running() ? `${fmt(Math.abs(j._f || 0), 1)} N` : 'press Play'))),
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
    live = [];
    const it = A.S.selected;
    card.hidden = !it;
    if (!it) { card.replaceChildren(); return; }
    const isBody = M.isBody(it);
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
    card.replaceChildren(head, ...(isBody ? bodyCard(it) : jointCard(it)), remove);
  }

  function tick() { for (const [el, fn] of live) { const t = fn(); if (el.textContent !== t) el.textContent = t; } }

  function init(app, el) { A = app; card = el; }

  ML.Inspector = { init, refresh, tick };
})();
