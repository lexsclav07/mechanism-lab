// scope.js: a recorder and graph. While the machine runs it samples about 30 times a second:
// kinetic energy, gravitational and spring energy, and each part's position, speed, angle and
// spin. The graph drawer plots either the energies (is energy conserved? where does it go?) or
// one motion quantity for a part, and the data can be copied as CSV for a spreadsheet.
//
// Textbook axes as everywhere else: x to the right, y up, angles and spin anticlockwise.
(function () {
  const ML = (window.ML = window.ML || {});
  const P = ML.Phys;

  const MAX = 4000;          // samples kept (about two minutes)
  const DT = 1 / 30;         // sim seconds between samples
  let rows = [], datum = null, lastT = -1;

  // What each motion quantity is, and where it sits in a sample's per-part array.
  const QUANTITIES = [
    { id: 'x', name: 'x', unit: 'm', at: 0 },
    { id: 'y', name: 'y', unit: 'm', at: 1 },
    { id: 'v', name: 'speed', unit: 'm/s', at: 2 },
    { id: 'th', name: 'angle', unit: '°', at: 3 },
    { id: 'w', name: 'spin', unit: 'rad/s', at: 4 },
  ];
  const moving = (b) => !b.fixed && !b._red;

  // Energy of the scene right now (J): kinetic (moving and spinning), gravitational (relative to
  // where the parts started) and stored in springs. Motors, pistons, air drag and your hand
  // add or remove energy, so the total is only constant without them.
  function energy(scene) {
    P.index(scene);
    const g = scene.settings.gravity;
    let ke = 0, pg = 0, ps = 0;
    for (const b of scene.bodies) {
      if (!moving(b)) continue;
      ke += 0.5 * b.mass * (b.vx * b.vx + b.vy * b.vy) + 0.5 * (b._I || 0) * b.av * b.av;
      pg -= b.mass * g * (b.gscale ?? 1) * b.y; // y points down on screen
    }
    for (const j of scene.joints) if (j.kind === 'spring') ps += 0.5 * j.k * (P.jointLength(scene, j) - j.rest) ** 2;
    return { ke, pg, ps };
  }

  function clear() { rows = []; datum = null; lastT = -1; }

  // Called after each frame's physics step; keeps one sample per 1/30 s of simulated time.
  function sample(scene) {
    if (scene.time < lastT - 1e-9) clear(); // the clock went back (reset, undo)
    if (rows.length && scene.time - lastT < DT - 1e-9) return;
    const e = energy(scene);
    if (datum === null) datum = e.pg;
    const parts = {};
    for (const b of scene.bodies) {
      if (!moving(b)) continue;
      parts[b.id] = [b.x, -b.y, Math.hypot(b.vx, b.vy), ML.Forces.bodyAngle(b), -b.av];
    }
    rows.push({ t: scene.time, ke: e.ke, pg: e.pg - datum, ps: e.ps, tot: e.ke + e.pg - datum + e.ps, parts });
    lastT = scene.time;
    if (rows.length > MAX + 200) rows.splice(0, rows.length - MAX);
  }

  const count = () => rows.length;
  const nameOf = (b) => (b.label ? ML.Forces.pretty(b.label) : ML.Model.kindName(b).toLowerCase() + ' ' + b.id);

  // The lines to draw: [{ name, color (0–3), v: [value per row] }].
  function lines(scene, mode, quantity, partId) {
    if (mode === 'energy') {
      return [
        { name: 'Kinetic', color: 0, v: rows.map((r) => r.ke) },
        { name: 'Gravity', color: 1, v: rows.map((r) => r.pg) },
        { name: 'Springs', color: 2, v: rows.map((r) => r.ps) },
        { name: 'Total', color: 3, v: rows.map((r) => r.tot), heavy: true },
      ];
    }
    const q = QUANTITIES.find((x) => x.id === quantity) || QUANTITIES[0];
    const bodies = scene.bodies.filter(moving).filter((b) => partId == null || partId === 'all' || b.id === partId);
    return bodies.slice(0, 6).map((b, i) => ({ name: nameOf(b), color: i % 4, dash: i >= 4, v: rows.map((r) => (r.parts[b.id] ? r.parts[b.id][q.at] : null)) }));
  }

  const nice = (v) => {
    const a = Math.abs(v);
    return a === 0 ? '0' : a >= 100 ? v.toFixed(0) : a >= 10 ? v.toFixed(1) : a >= 1 ? v.toFixed(2) : a >= 0.01 ? v.toFixed(3) : v.toExponential(1);
  };
  // axis ticks at 1, 2 or 5 × 10^n
  function ticks(lo, hi, want) {
    const span = hi - lo || 1, raw = span / want, p = 10 ** Math.floor(Math.log10(raw)), f = raw / p;
    const step = (f < 1.5 ? 1 : f < 3.5 ? 2 : f < 7.5 ? 5 : 10) * p, out = [];
    for (let v = Math.ceil(lo / step) * step; v <= hi + step * 1e-9; v += step) out.push(Math.abs(v) < step * 1e-9 ? 0 : v);
    return out;
  }

  // Draw into a canvas. `hover` is a pixel x (or null) for the crosshair readout.
  function draw(canvas, ls, unit, hover) {
    const dpr = window.devicePixelRatio || 1, r = canvas.getBoundingClientRect();
    if (!r.width || !r.height) return null;
    if (canvas.width !== Math.round(r.width * dpr) || canvas.height !== Math.round(r.height * dpr)) { canvas.width = Math.round(r.width * dpr); canvas.height = Math.round(r.height * dpr); }
    const ctx = canvas.getContext('2d');
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    const W = r.width, H = r.height, cs = getComputedStyle(document.documentElement), g = (n) => cs.getPropertyValue(n).trim();
    const ink = g('--ink'), muted = g('--ink-2'), rule = g('--rule'), cols = [g('--s1'), g('--s2'), g('--s3'), g('--s4')];
    ctx.clearRect(0, 0, W, H);
    ctx.font = '11px ' + g('--font-num'); ctx.textBaseline = 'middle';
    const L = 52, R = 10, T = 8, B = 20, pw = W - L - R, ph = H - T - B;
    if (rows.length < 2) {
      ctx.fillStyle = muted; ctx.textAlign = 'center'; ctx.fillText('Press Play: the graph fills in as the machine runs.', W / 2, H / 2);
      return null;
    }
    const t0 = rows[0].t, t1 = rows[rows.length - 1].t;
    let lo = Infinity, hi = -Infinity;
    for (const l of ls) for (const v of l.v) if (v != null && Number.isFinite(v)) { if (v < lo) lo = v; if (v > hi) hi = v; }
    if (!Number.isFinite(lo)) { lo = 0; hi = 1; }
    if (hi - lo < 1e-9) { lo -= 0.5; hi += 0.5; }
    const pad = (hi - lo) * 0.06; lo -= pad; hi += pad;
    const X = (t) => L + ((t - t0) / (t1 - t0 || 1)) * pw, Y = (v) => T + (1 - (v - lo) / (hi - lo)) * ph;
    ctx.lineWidth = 1; ctx.textAlign = 'right';
    for (const v of ticks(lo, hi, 5)) {
      const y = Math.round(Y(v)) + 0.5;
      ctx.strokeStyle = v === 0 ? muted : rule; ctx.beginPath(); ctx.moveTo(L, y); ctx.lineTo(W - R, y); ctx.stroke();
      ctx.fillStyle = muted; ctx.fillText(nice(v), L - 6, y);
    }
    ctx.textAlign = 'center'; ctx.fillStyle = muted;
    for (const t of ticks(t0, t1, Math.max(2, Math.floor(pw / 90)))) ctx.fillText(nice(t) + ' s', X(t), H - B / 2 + 1);
    ctx.save(); ctx.beginPath(); ctx.rect(L, T, pw, ph); ctx.clip();
    for (const l of ls) {
      ctx.strokeStyle = cols[l.color]; ctx.lineWidth = l.heavy ? 2.2 : 1.6; ctx.setLineDash(l.dash ? [5, 3] : []);
      ctx.beginPath(); let pen = false;
      rows.forEach((row, i) => {
        const v = l.v[i];
        if (v == null || !Number.isFinite(v)) { pen = false; return; }
        pen ? ctx.lineTo(X(row.t), Y(v)) : ctx.moveTo(X(row.t), Y(v)); pen = true;
      });
      ctx.stroke();
    }
    ctx.restore(); ctx.setLineDash([]);
    ctx.fillStyle = muted; ctx.textAlign = 'left'; ctx.fillText(unit, 4, T + 4);
    // crosshair: the values at the pointer
    let readout = null;
    if (hover != null && hover >= L && hover <= W - R) {
      const t = t0 + ((hover - L) / pw) * (t1 - t0);
      let i = 0, best = Infinity;
      rows.forEach((row, k) => { const d = Math.abs(row.t - t); if (d < best) { best = d; i = k; } });
      ctx.strokeStyle = ink; ctx.lineWidth = 1; ctx.beginPath(); ctx.moveTo(X(rows[i].t) + 0.5, T); ctx.lineTo(X(rows[i].t) + 0.5, T + ph); ctx.stroke();
      readout = { t: rows[i].t, values: ls.map((l) => [l.name, l.v[i], l.color]) };
    }
    return { readout, cols };
  }

  // Everything recorded, as CSV: time, the energies, then every part's quantities.
  function csv(scene) {
    const ids = [...new Set(rows.flatMap((r) => Object.keys(r.parts)))].map(Number);
    const bodies = ids.map((id) => scene.bodies.find((b) => b.id === id)).filter(Boolean);
    const head = ['t (s)', 'kinetic (J)', 'gravity (J)', 'springs (J)', 'total (J)'];
    for (const b of bodies) for (const q of QUANTITIES) head.push(`${nameOf(b)} ${q.name} (${q.unit})`);
    const f = (v) => (v == null ? '' : +v.toPrecision(7));
    const out = [head.map((h) => (/[",]/.test(h) ? `"${h.replace(/"/g, '""')}"` : h)).join(',')];
    for (const r of rows) {
      const cells = [r.t, r.ke, r.pg, r.ps, r.tot];
      for (const b of bodies) for (const q of QUANTITIES) cells.push(r.parts[b.id] ? r.parts[b.id][q.at] : null);
      out.push(cells.map(f).join(','));
    }
    return out.join('\n');
  }

  // How much the total energy has changed since the first sample, as { joules, share }.
  function drift() {
    if (rows.length < 2) return null;
    const a = rows[0].tot, b = rows[rows.length - 1].tot, big = Math.max(...rows.map((r) => Math.abs(r.ke) + Math.abs(r.pg) + r.ps), 1e-9);
    return { joules: b - a, share: (b - a) / big };
  }

  ML.Scope = { QUANTITIES, energy, clear, sample, count, lines, draw, csv, drift };
})();
