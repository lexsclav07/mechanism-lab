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
    return { version: 1, time: 0, nextId: 1, settings: { gravity: 9.81, air: 0.02, substeps: 100 }, bodies: [], joints: [] };
  }

  function addBody(scene, props) {
    const b = Object.assign({}, BODY_DEFAULTS, { id: scene.nextId++ }, props);
    if (b.kind === 'box') { b.wall = b.wall ?? 0.1; b.hollow = b.hollow ?? false; }
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
    ML.Phys.index(scene);
    return scene;
  }

  const findById = (scene, id) => scene.bodies.find((b) => b.id === id) || scene.joints.find((j) => j.id === id) || null;
  const isBody = (item) => item && item.mass != null;
  const touches = (j, id) => j.a.body === id || j.b.body === id || (j.p && j.p.body === id);

  // Deleting a body also deletes every joint attached to it.
  function remove(scene, item) {
    if (isBody(item)) {
      scene.bodies = scene.bodies.filter((b) => b !== item);
      scene.joints = scene.joints.filter((j) => !touches(j, item.id));
    } else {
      scene.joints = scene.joints.filter((j) => j !== item);
    }
  }

  // Next free label with this letter: M_1, M_2 ... (users can rename to anything, like M_f).
  function nextLabel(scene, letter) {
    const used = new Set([...scene.bodies, ...scene.joints].map((x) => x.label));
    for (let i = 1; ; i++) if (!used.has(`${letter}_${i}`)) return `${letter}_${i}`;
  }

  const KIND_NAMES = { box: 'Block', disc: 'Weight', poly: 'Shape', pivot: 'Pin', weld: 'Glue', rod: 'Rod', rope: 'Rope',
    pulley: 'Pulley', spring: 'Spring', damper: 'Damper', piston: 'Piston', slider: 'Track' };
  const kindName = (item) => (item.kind === 'pivot' && item.motor ? 'Motor' : KIND_NAMES[item.kind] || item.kind);

  // Textbook-style labels for new parts, like the figure: M for bodies, m for weights,
  // k for springs, c for dampers, L for rods. Ropes, pulleys and pistons go unlabelled.
  const LETTERS = { box: 'M', poly: 'M', disc: 'm', spring: 'k', damper: 'c', rod: 'L' };
  const autoLabel = (scene, kind) => (LETTERS[kind] ? nextLabel(scene, LETTERS[kind]) : '');

  ML.Model = { newScene, addBody, addJoint, serialize, load, findById, isBody, remove, nextLabel, autoLabel, kindName };
})();
