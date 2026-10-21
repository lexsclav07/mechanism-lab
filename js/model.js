// model.js: the scene is plain JSON-friendly data, so undo, reset and save/load are all
// just "serialise the scene to a string and back". Keys starting with _ are solver scratch
// values and are never saved.
(function () {
  const ML = (window.ML = window.ML || {});

  const BODY_DEFAULTS = {
    kind: 'box', x: 0, y: 0, a: 0, vx: 0, vy: 0, av: 0,
    mass: 1, gscale: 1, fixed: false, color: 'grey', label: '', trace: false,
  };

  // gscale scales a body's weight: 1 = normal, 0 = weightless, negative = floats up.
  // motor is in rpm (0 = just a pin) and torque is the motor's limit in N·m.
  // Pistons cycle between min and max length (m) at `speed` (m/s), with at most `force` (N).
  const JOINT_DEFAULTS = {
    pivot: { motor: 0, torque: 50 },
    weld: { angle0: 0 },
    rod: { length: 1 },
    rope: { length: 1 },
    pulley: { length: 2, r: 0.15 },
    spring: { k: 50, c: 0.2, rest: 1, coils: 10 },
    damper: { c: 20 },
    piston: { min: 0.5, max: 1.5, speed: 0.4, force: 400, mode: 'cycle', phase: 1 },
    slider: { dir: [1, 0], lock: true, angle0: 0, limits: true, lo: -1, hi: 1 },
  };

  function newScene() {
    // substeps are per 1/60 s: 100 → the solver runs at 6000 Hz. More substeps = less
    // numerical energy loss (it shrinks in proportion to the step size).
    return { version: 2, time: 0, nextId: 1, settings: { gravity: 9.81, air: 0.02, substeps: 100 }, bodies: [], joints: [] };
  }

  // Rods are parts: a line of length L from local (-L/2, 0) to (+L/2, 0), 0.2 kg per metre.
  // ideal = "massless (textbook rod)": used when the rod is only pinned at its two ends.
  const barMass = (L) => Math.max(0.05, +(0.2 * L).toPrecision(2));

  function addBody(scene, props) {
    const b = Object.assign({}, BODY_DEFAULTS, { id: scene.nextId++ }, props);
    if (b.kind === 'box') { b.wall = b.wall ?? 0.1; b.hollow = b.hollow ?? false; }
    if (b.kind === 'bar') { b.ideal = b.ideal ?? true; b.mass = props.mass ?? barMass(b.L); b.color = props.color ?? 'ink'; }
    scene.bodies.push(b);
    return b;
  }

  function addJoint(scene, props) {
    const j = Object.assign({ id: scene.nextId++, label: '' }, JOINT_DEFAULTS[props.kind], props);
    scene.joints.push(j);
    return j;
  }

  const serialize = (scene) => JSON.stringify(scene, (k, v) => (k[0] === '_' ? undefined : v));

  function load(text) {
    const raw = JSON.parse(text);
    if (!raw || !Array.isArray(raw.bodies) || !Array.isArray(raw.joints)) throw new Error('This is not a Mechanism Lab scene.');
    const scene = Object.assign(newScene(), raw);
    scene.settings = Object.assign(newScene().settings, raw.settings);
    scene.bodies = raw.bodies.map((b) => {
      const out = Object.assign({}, BODY_DEFAULTS, b);
      if (b.style && !b.color) out.color = b.style === 'ink' ? 'ink' : b.style === 'paper' ? 'white' : 'grey'; // older saves
      delete out.style;
      return out;
    });
    scene.joints = raw.joints.map((j) => Object.assign({ label: '' }, JOINT_DEFAULTS[j.kind], j));
    const maxId = Math.max(0, ...scene.bodies.map((b) => b.id), ...scene.joints.map((j) => j.id || 0));
    scene.joints.forEach((j, i) => { if (j.id == null) j.id = maxId + 1 + i; });
    scene.nextId = Math.max(scene.nextId, maxId + scene.joints.length + 1);
    migrateRods(scene);
    ML.Phys.index(scene);
    return scene;
  }

  // Rods used to be weightless links (joint kind 'rod') that couldn't be joined to each other.
  // Each one becomes a rod part with a pin at each end, put in the old rod's place in the joint
  // order and marked massless, so it moves exactly as it did. The rod keeps its id and label.
  function migrateRods(scene) {
    if (!scene.joints.some((j) => j.kind === 'rod')) return;
    const P = ML.Phys;
    P.index(scene);
    const out = [];
    for (const j of scene.joints) {
      if (j.kind !== 'rod') { out.push(j); continue; }
      const pA = P.endWorld(scene, j.a), pB = P.endWorld(scene, j.b), L = j.length;
      scene.bodies.push(Object.assign({}, BODY_DEFAULTS, {
        id: j.id, kind: 'bar', x: (pA[0] + pB[0]) / 2, y: (pA[1] + pB[1]) / 2, a: Math.atan2(pB[1] - pA[1], pB[0] - pA[0]),
        L, mass: barMass(L), ideal: true, color: 'ink', label: j.label || '' }));
      for (const [x, far] of [[-L / 2, j.a], [L / 2, j.b]]) {
        out.push(Object.assign({ label: '' }, JOINT_DEFAULTS.pivot, { id: scene.nextId++, kind: 'pivot', a: { body: j.id, x, y: 0 }, b: far }));
      }
    }
    scene.joints = out;
    scene.version = 2;
  }

  const findById = (scene, id) => scene.bodies.find((b) => b.id === id) || scene.joints.find((j) => j.id === id) || null;
  const isBody = (item) => item && item.mass != null;
  const touches = (j, id) => j.a.body === id || j.b.body === id || (j.p && j.p.body === id);

  // Take the pins and glue off a body (only those at local x = `at` when given). Where the body
  // was the middle of a shared pin (other parts were pinned or glued to it at one spot), those
  // others get pinned to each other there instead, so taking one rod out of a truss doesn't
  // drop the rest.
  function unpin(scene, body, at = null) {
    const P = ML.Phys;
    P.index(scene);
    const nodes = new Map(), gone = new Set();
    for (const j of scene.joints) {
      if ((j.kind !== 'pivot' && j.kind !== 'weld') || !touches(j, body.id)) continue;
      const mine = j.a.body === body.id ? j.a : j.b, other = mine === j.a ? j.b : j.a;
      if (at != null && Math.abs(mine.x - at) > 1e-6) continue;
      gone.add(j);
      if (other.body === body.id) continue;
      const p = P.endWorld(scene, mine), key = p.map((v) => v.toFixed(5)).join(',');
      if (!nodes.has(key)) nodes.set(key, { weld: false, others: [] });
      const n = nodes.get(key);
      if (j.kind === 'weld') n.weld = true;
      if (!n.others.some((o) => o.body === other.body && (o.body != null || (o.x === other.x && o.y === other.y)))) n.others.push(other);
    }
    scene.joints = scene.joints.filter((j) => !gone.has(j));
    for (const n of nodes.values()) {
      const hub = n.others.find((o) => o.body != null);
      if (n.others.length < 2 || !hub) continue;
      for (const o of n.others) {
        if (o === hub) continue;
        const props = { kind: n.weld ? 'weld' : 'pivot', a: { ...hub }, b: { ...o } };
        if (n.weld) props.angle0 = P.angleOf(scene, o) - P.angleOf(scene, hub);
        addJoint(scene, props);
      }
    }
    P.index(scene);
  }

  // Deleting a body also deletes every joint attached to it (re-pinning shared pins, see unpin).
  function remove(scene, item) {
    if (isBody(item)) {
      unpin(scene, item);
      scene.bodies = scene.bodies.filter((b) => b !== item);
      scene.joints = scene.joints.filter((j) => !touches(j, item.id));
      ML.Phys.index(scene);
    } else {
      scene.joints = scene.joints.filter((j) => j !== item);
    }
  }

  // Next free label with this letter: M_1, M_2 ... (users can rename to anything, like M_f).
  function nextLabel(scene, letter) {
    const used = new Set([...scene.bodies, ...scene.joints].map((x) => x.label));
    for (let i = 1; ; i++) if (!used.has(`${letter}_${i}`)) return `${letter}_${i}`;
  }

  const KIND_NAMES = { box: 'Block', disc: 'Weight', poly: 'Shape', bar: 'Rod', pivot: 'Pin', weld: 'Glue', rod: 'Rod', rope: 'Rope',
    pulley: 'Pulley', spring: 'Spring', damper: 'Damper', piston: 'Piston', slider: 'Track' };
  const kindName = (item) => (item.kind === 'pivot' && item.motor ? 'Motor' : KIND_NAMES[item.kind] || item.kind);

  // Textbook-style labels for new parts, like the figure: M for bodies, m for weights,
  // k for springs, c for dampers, L for rods. Ropes, pulleys and pistons go unlabelled.
  const LETTERS = { box: 'M', poly: 'M', disc: 'm', bar: 'L', spring: 'k', damper: 'c' };
  const autoLabel = (scene, kind) => (LETTERS[kind] ? nextLabel(scene, LETTERS[kind]) : '');

  ML.Model = { newScene, addBody, addJoint, serialize, load, findById, isBody, remove, unpin, nextLabel, autoLabel, kindName, barMass };
})();
