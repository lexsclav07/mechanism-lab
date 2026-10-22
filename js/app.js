// app.js: ties it together. Owns the state, the animation loop, play / pause / reset,
// undo and redo, the menus, the scoreboard, autosave and keyboard shortcuts.
(function () {
  const ML = (window.ML = window.ML || {});
  const P = ML.Phys, R = ML.Render, M = ML.Model, V = R.View;
  const $ = (id) => document.getElementById(id);
  const STORE = 'mechlab.scene.v2', OLD_STORE = 'mechlab.scene.v1', OPTS = 'mechlab.opts.v3';
  const TOOLS = ['select', 'camera', 'box', 'weight', 'poly', 'pivot', 'motor', 'weld', 'rod', 'rope']; // number keys 1–0
  const PLANETS = [['Earth', 9.81], ['Moon', 1.62], ['Mars', 3.71], ['Jupiter', 24.79], ['Zero-G', 0]];
  const calm = matchMedia('(prefers-reduced-motion: reduce)');

  const S = {
    scene: M.newScene(), running: false, speed: 1, tool: 'select',
    selected: null, hover: null, snap: null, preview: null,
    drag: null,      // the hand while running: { body, x, y (grabbed point), tx, ty (pointer), k, c }
    poseDrag: null,  // the same while paused (posing, no forces)
    hand: null, follow: null, followPt: null, // what the renderer draws for those
    opts: { values: true, labels: true, grid: false, snapGrid: false, reactions: false, fbd: false, comp: true },
    probe: null,     // the point last tapped on a part, in its own frame: { body, x, y }
    pairFrom: null,  // "Compare with another part": waiting for the second tap
    an: null,        // the scene the force read-outs come from (see analysis())
    traces: new Map(), spawn: new Map(), freshTags: true,
    records: { speed: 0, force: 0 }, // the scoreboard, since the last reset
    design: null,  // the scene as it was when Play was last pressed (Reset goes back to it)
    dirty: true,   // edited since then?
    undo: [], redo: [],
  };

  // ---------- small helpers ----------

  const store = {
    get(k) { try { return localStorage.getItem(k); } catch { return null; } },
    set(k, v) { try { localStorage.setItem(k, v); } catch { /* private window etc. */ } },
  };

  let hintTimer = 0;
  function hint(text, ms = 4500) {
    $('hint').textContent = text;
    clearTimeout(hintTimer);
    if (ms) hintTimer = setTimeout(() => { $('hint').textContent = ''; }, ms);
  }

  function autosave() { store.set(STORE, !S.dirty && S.design ? S.design : M.serialize(S.scene)); }
  const saveOpts = () => store.set(OPTS, JSON.stringify(S.opts));
  const clearRecords = () => { S.records = { speed: 0, force: 0 }; };

  // Fix any joints an edit pulled apart (e.g. a resized part), keeping velocities.
  function relax() {
    const vel = S.scene.bodies.map((b) => [b.vx, b.vy, b.av]);
    P.pose(S.scene, null, 20);
    S.scene.bodies.forEach((b, i) => { [b.vx, b.vy, b.av] = vel[i]; });
  }

  // ---------- state changes (the editor and the part card call these) ----------

  function checkpoint() {
    S.undo.push(M.serialize(S.scene));
    if (S.undo.length > 200) S.undo.shift();
    S.redo = [];
    syncButtons();
  }

  function changed({ keepPanel = false } = {}) {
    S.dirty = true;
    P.index(S.scene);
    if (!S.running) relax();
    if (!keepPanel) ML.Inspector.refresh();
    syncGravity(); autosave(); syncButtons();
  }

  const touched = () => { S.dirty = true; };

  // Camera follow: target = { body, x, y } (a point in that body's own frame), or null.
  function follow(target) {
    S.follow = target; S.followPt = null;
    $('follow-chip').hidden = !target;
    if (target) {
      V.userMoved = true;
      const b = M.findById(S.scene, target.body), lab = $('follow-label');
      lab.replaceChildren('Camera following ');
      if (b && b.label) { // "m_p" → italic m with subscript p, like the drawing
        const i = b.label.indexOf('_'), it = document.createElement('i');
        it.className = 'm'; it.textContent = i < 0 ? b.label : b.label.slice(0, i);
        lab.append(it);
        if (i >= 0) { const sub = document.createElement('sub'); sub.textContent = b.label.slice(i + 1); lab.append(sub); }
      } else lab.append(b ? `the ${M.kindName(b).toLowerCase()}` : 'a point');
    }
    ML.Inspector.refresh();
  }

  // Setting edits (mass, stiffness, colour, gravity…) are copied into the saved start state too,
  // so Reset keeps your tweak instead of quietly undoing it. Positions and speeds aren't copied.
  function tweak(item, { keepPanel = false } = {}) {
    if (!S.design || S.dirty) { changed({ keepPanel }); return; }
    const d = JSON.parse(S.design);
    if (!item) d.settings = Object.assign({}, S.scene.settings);
    else {
      const list = M.isBody(item) ? d.bodies : d.joints, k = list.findIndex((x) => x.id === item.id);
      if (k >= 0) {
        const keep = M.isBody(item) ? { x: list[k].x, y: list[k].y, a: list[k].a, vx: list[k].vx, vy: list[k].vy, av: list[k].av } : {};
        list[k] = Object.assign(JSON.parse(M.serialize(item)), keep);
      }
    }
    S.design = M.serialize(d);
    P.index(S.scene);
    if (!S.running) relax();
    if (!keepPanel) ML.Inspector.refresh();
    syncGravity(); autosave();
  }
  const spawn = (b) => { if (!calm.matches) S.spawn.set(b.id, performance.now()); };

  function select(item) {
    S.selected = item; S.hover = null;
    if (!item || !M.isBody(item) || (S.probe && S.probe.body !== item.id)) S.probe = null;
    ML.Inspector.refresh();
  }

  function restore(text) {
    const selId = S.selected && S.selected.id;
    S.scene = M.load(text);
    S.selected = selId != null ? M.findById(S.scene, selId) : null;
    S.hover = null; S.drag = null;
    ML.Editor.cancel(); ML.Inspector.refresh(); syncGravity();
  }

  // ---------- simulation controls ----------

  function play() {
    if (S.dirty || !S.design) { S.design = M.serialize(S.scene); S.dirty = false; clearRecords(); S.traces.clear(); }
    S.running = true; syncButtons(); ML.Inspector.refresh();
  }

  function pause() { S.running = false; S.drag = null; syncButtons(); autosave(); ML.Inspector.refresh(); }

  function reset() {
    if (!S.design) { hint('Reset takes you back to where you last pressed Play.'); return; }
    S.running = false; S.traces.clear(); clearRecords();
    restore(S.design); S.dirty = false;
    syncButtons(); autosave();
  }

  function undo() {
    if (!S.undo.length) return;
    pause(); S.redo.push(M.serialize(S.scene));
    restore(S.undo.pop()); S.dirty = true; autosave(); syncButtons();
  }

  function redo() {
    if (!S.redo.length) return;
    pause(); S.undo.push(M.serialize(S.scene));
    restore(S.redo.pop()); S.dirty = true; autosave(); syncButtons();
  }

  function loadPreset(id, autoplay = true) {
    const pr = ML.Presets.find((p) => p.id === id);
    if (!pr) return;
    if (S.scene.bodies.length) checkpoint();
    S.running = false; S.traces.clear(); S.design = null;
    S.scene = M.load(M.serialize(pr.build()));
    for (const b of S.scene.bodies) spawn(b);
    S.selected = null; follow(null); ML.Editor.cancel();
    fitView(); changed();
    if (autoplay && S.scene.bodies.length) play();
    if (pr.opts) { Object.assign(S.opts, pr.opts); saveOpts(); } // e.g. statics examples show their support reactions
    if (pr.id !== 'empty') hint(pr.blurb, 5000);
  }

  function setTool(t) {
    S.tool = t; S.hover = null; ML.Editor.cancel();
    for (const b of document.querySelectorAll('[data-tool]')) b.setAttribute('aria-pressed', String(b.dataset.tool === t));
    hint(ML.Editor.HINTS[t], 6000);
  }

  // Frame everything in view, leaving room for the HUD at the top and bottom.
  function fitView() {
    V.userMoved = false;
    const sc = S.scene;
    P.index(sc);
    const pts = [];
    for (const b of sc.bodies) {
      const r = b.kind === 'disc' ? b.r : b.kind === 'box' ? Math.hypot(b.w, b.h) / 2 : b.kind === 'bar' ? b.L / 2 : Math.max(...b.pts.map(([x, y]) => Math.hypot(x, y)));
      pts.push([b.x - r, b.y - r], [b.x + r, b.y + r]);
    }
    for (const j of sc.joints) pts.push(P.endWorld(sc, j.a), P.endWorld(sc, j.b));
    if (!pts.length) { V.cx = 0; V.cy = 0; V.zoom = 1; return; }
    const xs = pts.map((p) => p[0]), ys = pts.map((p) => p[1]);
    const [x0, x1, y0, y1] = [Math.min(...xs), Math.max(...xs), Math.min(...ys) - 0.3, Math.max(...ys) + 0.2];
    const top = 110, bottom = 110; // px taken by the top bar and the tray
    const availH = Math.max(120, V.H - top - bottom);
    V.zoom = Math.min(3.5, Math.max(0.2, 0.92 * Math.min(V.W / ((x1 - x0 + 0.3) * 100), availH / ((y1 - y0 + 0.3) * 100))));
    V.cx = (x0 + x1) / 2;
    V.cy = (y0 + y1) / 2 - (top - bottom) / 2 / V.ppm();
  }

  function syncButtons() {
    $('play').setAttribute('aria-pressed', String(S.running));
    $('play-label').textContent = S.running ? 'Pause' : 'Play';
    $('play-icon').innerHTML = S.running
      ? '<rect x="5.5" y="4" width="4.5" height="16" rx="1"/><rect x="14" y="4" width="4.5" height="16" rx="1"/>'
      : '<path d="M6 4v16l14-8z"/>';
    $('undo').disabled = !S.undo.length;
    $('redo').disabled = !S.redo.length;
  }

  function syncGravity() {
    const g = S.scene.settings.gravity, p = PLANETS.find(([, v]) => Math.abs(v - g) < 0.005);
    $('gravity-label').textContent = p ? p[0] : `${g.toFixed(2)} m/s²`;
  }

  // ---------- forces for the read-outs ----------

  // Running, the scene itself has the forces (averaged over each frame). Paused, nothing has
  // been worked out for this exact pose, so one frame is run on a copy (ML.Forces.snapshot) and
  // kept until something changes: the forces at this instant, which for a structure that can't
  // move are its static loads.
  let snapKey = null, snapScene = null;
  function analysis() {
    const want = !!(S.selected || S.opts.reactions);
    S.scene._wantEnds = want;
    if (!want) return null;
    if (S.running) return S.scene;
    const key = M.serialize(S.scene);
    if (key !== snapKey) { snapKey = key; snapScene = ML.Forces.snapshot(S.scene); }
    return snapScene;
  }

  // ---------- the loop ----------

  function advance(dt) {
    P.step(S.scene, dt, S.drag);
    if (S.scene.bodies.some((b) => !Number.isFinite(b.x + b.y + b.a))) { // blew up: undo the run
      S.running = false;
      if (S.design) restore(S.design);
      hint('Whoa, that got out of hand, so it was reset. Try a softer spring or a heavier part.', 7000);
      syncButtons(); return;
    }
    const rec = S.records;
    for (const b of S.scene.bodies) {
      if (b.fixed) continue;
      rec.speed = Math.max(rec.speed, Math.hypot(b.vx, b.vy));
      if (!b.trace) continue;
      const tr = S.traces.get(b.id) || [];
      tr.push([b.x, b.y]);
      if (tr.length > 1600) tr.splice(0, tr.length - 1500);
      S.traces.set(b.id, tr);
    }
    for (const j of S.scene.joints) rec.force = Math.max(rec.force, Math.abs(j._avg ? j._avg.f : j._f || 0));
  }

  let canvas, ctx, last = performance.now(), frameN = 0;

  function resize() {
    const r = canvas.parentElement.getBoundingClientRect(), dpr = window.devicePixelRatio || 1;
    V.W = Math.max(1, r.width); V.H = Math.max(1, r.height);
    canvas.width = Math.round(V.W * dpr); canvas.height = Math.round(V.H * dpr);
    canvas.style.width = V.W + 'px'; canvas.style.height = V.H + 'px';
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    if (!V.userMoved) fitView(); // keep refitting until the player pans or zooms
  }

  const fmt = (v) => (v >= 100 ? v.toFixed(0) : v >= 10 ? v.toFixed(1) : v.toFixed(2));

  function frame(now) {
    const dt = Math.min(1 / 30, (now - last) / 1000);
    last = now;
    if (S.running) advance(dt * S.speed);
    P.index(S.scene);
    if (S.follow) { // glide the camera toward the followed point
      const b = S.scene._byId.get(S.follow.body);
      if (!b) follow(null);
      else {
        const p = P.toWorld(b, S.follow.x, S.follow.y), k = 1 - Math.exp(-dt * 10);
        V.cx += (p[0] - V.cx) * k; V.cy += (p[1] - V.cy) * k; S.followPt = p;
      }
    }
    const held = S.drag || S.poseDrag, hb = held && S.scene._byId.get(held.body);
    S.hand = hb ? { from: P.toWorld(hb, held.x, held.y), to: [held.tx, held.ty], F: S.drag ? S.drag._F || 0 : null } : null;
    S.freshTags = frameN++ % 6 === 0;
    if (S.freshTags || S.running) S.an = analysis();
    S.fbd = S.opts.fbd && S.selected && M.isBody(S.selected) && S.an ? S.selected.id : null;
    const pb = S.probe && S.scene._byId.get(S.probe.body);
    S.probePt = pb ? P.toWorld(pb, S.probe.x, S.probe.y) : null;
    if (S.freshTags) {
      ML.Inspector.tick();
      $('st-time').textContent = `${S.scene.time.toFixed(1)} s`;
      $('st-speed').textContent = `${fmt(S.records.speed)} m/s`;
      $('st-force').textContent = `${fmt(S.records.force)} N`;
    }
    R.render(ctx, S.scene, S);
    requestAnimationFrame(frame);
  }

  // ---------- menus ----------

  function closePops() {
    for (const p of document.querySelectorAll('.pop')) p.hidden = true;
    for (const b of document.querySelectorAll('[aria-haspopup]')) b.setAttribute('aria-expanded', 'false');
  }

  function popover(btnId, popId, fill) {
    const btn = $(btnId), pop = $(popId);
    btn.addEventListener('click', () => {
      const open = pop.hidden;
      closePops();
      if (open) { fill(pop); pop.hidden = false; btn.setAttribute('aria-expanded', 'true'); }
    });
  }

  const el = (tag, attrs, ...kids) => {
    const e = document.createElement(tag);
    for (const [k, v] of Object.entries(attrs || {})) k.startsWith('on') ? e.addEventListener(k.slice(2), v) : e.setAttribute(k, v);
    e.append(...kids);
    return e;
  };

  function optSwitch(key, label) {
    const cb = el('input', { type: 'checkbox', role: 'switch', id: 'o-' + key });
    cb.checked = !!S.opts[key];
    cb.addEventListener('change', () => { S.opts[key] = cb.checked; saveOpts(); });
    return el('label', { class: 'switch', for: 'o-' + key }, el('span', {}, label), cb);
  }

  function fillExamples(pop) {
    const items = [];
    let group = null;
    for (const p of ML.Presets) {
      const g = p.group || 'Classics';
      if (p.id !== 'empty' && g !== group) { items.push(el('h3', {}, g)); group = g; }
      if (p.id === 'empty') items.push(el('hr'));
      items.push(el('button', { type: 'button', class: 'pick', onclick: () => { closePops(); loadPreset(p.id, p.id !== 'empty'); } },
        el('b', {}, p.name), el('small', {}, p.blurb)));
    }
    pop.replaceChildren(...items);
  }

  function fillGravity(pop) {
    const st = S.scene.settings;
    const planets = el('div', { class: 'planets' }, ...PLANETS.map(([name, g]) => el('button', {
      type: 'button', class: 'planet', 'aria-pressed': String(Math.abs(st.gravity - g) < 0.005),
      onclick: () => { checkpoint(); st.gravity = g; tweak(null); fillGravity(pop); },
    }, el('b', {}, name), el('small', {}, `${g} m/s²`))));
    const air = el('input', { type: 'checkbox', role: 'switch', id: 'o-air' });
    air.checked = st.air > 0;
    air.addEventListener('change', () => { checkpoint(); st.air = air.checked ? 0.05 : 0; tweak(null); });
    pop.replaceChildren(planets, el('hr', {}), el('label', { class: 'switch', for: 'o-air' }, el('span', {}, 'Air resistance'), air));
  }

  function fillMenu(pop) {
    const item = (label, fn) => el('button', { type: 'button', class: 'tb', onclick: () => { closePops(); fn(); } }, label);
    pop.replaceChildren(
      optSwitch('values', 'Live values'), optSwitch('labels', 'Textbook labels'),
      optSwitch('reactions', 'Support reactions and rod forces'),
      optSwitch('grid', 'Grid'), optSwitch('snapGrid', 'Snap to grid'),
      el('hr', {}),
      item('Fit everything on screen', fitView),
      item('Save or load a build…', openScene),
      item('Clear everything', () => { loadPreset('empty', false); hint('Cleared. Undo brings it back.'); }));
  }

  function openScene() {
    $('scene-json').value = M.serialize(S.scene); $('scene-msg').textContent = '';
    $('scene-dialog').showModal();
  }

  function wireSceneDialog() {
    const dlg = $('scene-dialog'), ta = $('scene-json'), msg = $('scene-msg');
    $('scene-copy').addEventListener('click', () => {
      navigator.clipboard.writeText(ta.value).then(
        () => { msg.textContent = 'Copied. Paste it into a text file to keep it.'; },
        () => { ta.select(); msg.textContent = 'Press Ctrl+C to copy the selected text.'; });
    });
    $('scene-load').addEventListener('click', () => {
      try {
        const sc = M.load(ta.value);
        checkpoint(); S.running = false; S.design = null; S.traces.clear();
        S.scene = sc; S.selected = null; ML.Inspector.refresh(); fitView(); changed();
        dlg.close(); hint('Build loaded.');
      } catch (err) { msg.textContent = `That didn’t load: ${err.message} Paste text copied from this box.`; }
    });
    $('scene-close').addEventListener('click', () => dlg.close());
  }

  // Tray buttons: a tap picks the tool; dragging a block, weight, pin or glue off the tray
  // drops one onto the board where you let go.
  const DRAGGABLE = new Set(['box', 'weight', 'pivot', 'motor', 'weld']);
  function wireTray() {
    let drag = null, justDragged = false;
    for (const btn of document.querySelectorAll('[data-tool]')) {
      const t = btn.dataset.tool;
      btn.addEventListener('click', () => {
        if (justDragged) { justDragged = false; return; }
        setTool(S.tool === t && t !== 'select' ? 'select' : t); // tap again to put it down
      });
      if (!DRAGGABLE.has(t)) continue;
      btn.addEventListener('pointerdown', (e) => { drag = { t, x: e.clientX, y: e.clientY, ghost: null, id: e.pointerId }; });
      btn.addEventListener('pointermove', (e) => {
        if (!drag || drag.id !== e.pointerId) return;
        if (!drag.ghost && Math.hypot(e.clientX - drag.x, e.clientY - drag.y) > 10) {
          btn.setPointerCapture(e.pointerId);
          drag.ghost = el('div', { class: 'ghost' });
          drag.ghost.innerHTML = btn.querySelector('svg').outerHTML;
          document.body.append(drag.ghost);
        }
        if (drag.ghost) { drag.ghost.style.left = e.clientX + 'px'; drag.ghost.style.top = e.clientY + 'px'; }
      });
      const end = (e) => {
        if (!drag || drag.id !== e.pointerId) return;
        if (drag.ghost) {
          drag.ghost.remove(); justDragged = true; setTimeout(() => { justDragged = false; }, 0);
          const r = canvas.getBoundingClientRect(), under = document.elementFromPoint(e.clientX, e.clientY);
          if (e.type === 'pointerup' && under === canvas) ML.Editor.drop(t, e.clientX - r.left, e.clientY - r.top);
        }
        drag = null;
      };
      btn.addEventListener('pointerup', end);
      btn.addEventListener('pointercancel', end);
    }
  }

  // ---------- keyboard ----------

  function onKey(e) {
    const tag = (e.target.tagName || '').toLowerCase();
    if (tag === 'input' || tag === 'textarea') { if (e.key === 'Escape') e.target.blur(); return; }
    if ($('scene-dialog').open) return;
    if (ML.Editor.key(e)) { e.preventDefault(); return; }
    const mod = e.ctrlKey || e.metaKey;
    if (mod && e.key.toLowerCase() === 'z') { e.preventDefault(); e.shiftKey ? redo() : undo(); return; }
    if (mod && e.key.toLowerCase() === 'y') { e.preventDefault(); redo(); return; }
    if (mod) return;
    const k = e.key.toLowerCase();
    if (e.key === ' ') { e.preventDefault(); S.running ? pause() : play(); }
    else if (k === 'r') reset();
    else if (k === 's') toggleSlowmo();
    else if (k === 'f') fitView();
    else if (e.key === 'Escape') { closePops(); select(null); setTool('select'); if (S.follow) follow(null); }
    else if ((e.key === 'Delete' || e.key === 'Backspace') && S.selected && S.selected.kind !== 'pair') {
      e.preventDefault(); checkpoint(); M.remove(S.scene, S.selected); select(null); changed();
    } else if (/^[0-9]$/.test(e.key)) setTool(TOOLS[(+e.key + 9) % 10]);
    else if (e.key === '+' || e.key === '=') ML.Editor.zoomAt(V.W / 2, V.H / 2, V.zoom * 1.25);
    else if (e.key === '-') ML.Editor.zoomAt(V.W / 2, V.H / 2, V.zoom / 1.25);
  }

  function toggleSlowmo() {
    S.speed = S.speed === 1 ? 0.25 : 1;
    $('slowmo').setAttribute('aria-pressed', String(S.speed !== 1));
    hint(S.speed === 1 ? 'Back to normal speed.' : 'Slow motion: a quarter of real speed.', 2500);
  }

  // ---------- start ----------

  function start(data) {
    canvas = $('stage-canvas'); ctx = canvas.getContext('2d');
    R.readColors();
    document.fonts && document.fonts.ready.then(() => R.readColors());

    try { Object.assign(S.opts, JSON.parse(store.get(OPTS) || '{}')); } catch { /* ignore */ }
    ML.Inspector.init(App, $('inspector'));
    ML.Rods.init(App);
    ML.Editor.init(App, canvas);
    new ResizeObserver(resize).observe(canvas.parentElement);
    resize();

    $('play').addEventListener('click', () => (S.running ? pause() : play()));
    $('reset').addEventListener('click', reset);
    $('slowmo').addEventListener('click', toggleSlowmo);
    $('follow-stop').addEventListener('click', () => follow(null));
    $('undo').addEventListener('click', undo);
    $('redo').addEventListener('click', redo);
    popover('examples-btn', 'examples-pop', fillExamples);
    popover('gravity-btn', 'gravity-pop', fillGravity);
    popover('menu-btn', 'menu-pop', fillMenu);
    document.addEventListener('pointerdown', (e) => { if (!e.target.closest('.pop, [aria-haspopup]')) closePops(); });
    wireTray(); wireSceneDialog();
    document.addEventListener('keydown', onKey);
    addEventListener('pagehide', autosave);

    // what to show first: a live-update snapshot, then the autosave, then the video's contraption
    let text = data && data.scene;
    const fromSnapshot = !!text;
    if (!text) text = store.get(STORE) || store.get(OLD_STORE); // older saves convert their rods on load
    let loaded = false;
    if (text) { try { S.scene = M.load(text); loaded = true; } catch { /* fall through */ } }
    setTool('select');
    if (loaded) {
      if (data && data.design) { S.design = data.design; S.dirty = false; }
      fitView(); ML.Inspector.refresh(); syncButtons(); syncGravity();
      if (!fromSnapshot) hint('Welcome back! Your last build is right where you left it.');
      if (data && data.running) play();
    } else {
      loadPreset('frame');
      hint('This is the mechanism from your video. Grab any point to pull on it, or take parts from the tray and build your own.', 7000);
    }
    requestAnimationFrame(frame);
  }

  const App = { S, checkpoint, changed, tweak, touched, spawn, select, pause, play, hint, saveOpts, follow, analysis: () => S.an };
  ML.App = App;

  // Keep the build across live updates of this page.
  const hot = window.claude && window.claude.hot;
  if (hot && hot.snapshot) hot.snapshot(() => ({ scene: M.serialize(S.scene), design: S.design, running: S.running }));
  const boot = () => (hot && hot.ready ? hot.ready(start) : start((hot && hot.data) || {}));
  if (document.readyState === 'loading') document.addEventListener('DOMContentLoaded', boot); else boot();
})();
