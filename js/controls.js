// controls.js: driving machines from the keyboard. Motors, pistons and winches (ropes and
// pulley ropes) can be bound to keys in their cards; holding a key sends them a command.
// Keys are stored as KeyboardEvent.code ('KeyD', 'ArrowLeft'), which names the physical key,
// so W/A/S/D stay in the same place on any keyboard layout.
(function () {
  const ML = (window.ML = window.ML || {});

  const held = new Set();       // keys down right now
  let capture = null;           // waiting for a key to bind: { done(code) }

  // Keys that keep their usual job and can't be bound.
  const RESERVED = new Set(['Space', 'Escape', 'Tab', 'Enter', 'NumpadEnter', 'MetaLeft', 'MetaRight', 'ControlLeft', 'ControlRight', 'AltLeft', 'AltRight', 'ShiftLeft', 'ShiftRight', 'CapsLock', 'ContextMenu']);
  // Keys with a shortcut that a binding takes over (said on the card, so it's no surprise).
  const SHORTCUTS = { KeyR: 'reset', KeyS: 'slow motion', KeyF: 'fit to screen', Delete: 'delete', Backspace: 'delete', Equal: 'zoom in', Minus: 'zoom out' };
  for (let i = 0; i <= 9; i++) SHORTCUTS['Digit' + i] = 'tool ' + i;

  // How a key looks on its key cap.
  function label(code) {
    if (!code) return '';
    const arrows = { ArrowLeft: '←', ArrowRight: '→', ArrowUp: '↑', ArrowDown: '↓' };
    if (arrows[code]) return arrows[code];
    if (/^Key[A-Z]$/.test(code)) return code.slice(3);
    if (/^Digit\d$/.test(code)) return code.slice(5);
    if (/^Numpad\d$/.test(code)) return 'Num ' + code.slice(6);
    const named = { Comma: ',', Period: '.', Slash: '/', Semicolon: ';', Quote: "'", BracketLeft: '[', BracketRight: ']', Backslash: '\\', Minus: '-', Equal: '=', Backquote: '`', Backspace: '⌫', Delete: 'Del', ShiftLeft: 'Shift' };
    return named[code] || code;
  }

  // Where a key sits on a keyboard, left to right (so A comes before D and W before S whatever
  // their jobs), for listing a pair of keys in the order your fingers find them.
  const ROWS = ['Backquote Digit1 Digit2 Digit3 Digit4 Digit5 Digit6 Digit7 Digit8 Digit9 Digit0 Minus Equal',
    'Tab KeyQ KeyW KeyE KeyR KeyT KeyY KeyU KeyI KeyO KeyP BracketLeft BracketRight', 'CapsLock KeyA KeyS KeyD KeyF KeyG KeyH KeyJ KeyK KeyL Semicolon Quote',
    'ShiftLeft KeyZ KeyX KeyC KeyV KeyB KeyN KeyM Comma Period Slash'];
  const ARROWS = { ArrowLeft: 0, ArrowUp: 1, ArrowDown: 1, ArrowRight: 2 };
  function keyOrder(code) {
    if (code in ARROWS) return 100 + ARROWS[code] + (code === 'ArrowDown' ? 0.5 : 0);
    for (let r = 0; r < ROWS.length; r++) { const i = ROWS[r].split(' ').indexOf(code); if (i >= 0) return i + r * 0.25 + (r === 3 ? 0.5 : 0); }
    return 200;
  }

  // Every bound key in the scene: [{ j, code, role }] (role: 'fwd', 'rev', 'out', 'in').
  const FIELDS = { pivot: [['keyFwd', 'fwd'], ['keyRev', 'rev']], piston: [['keyOut', 'out'], ['keyIn', 'in']], rope: [['keyIn', 'in'], ['keyOut', 'out']], pulley: [['keyIn', 'in'], ['keyOut', 'out']] };
  function bound(j) {
    if (j.kind === 'piston' && j.mode !== 'keys') return [];
    if (j.kind === 'pivot' && !j.motor) return [];
    // a role can have several keys, space-separated ('ArrowRight ArrowUp'): any of them works
    return (FIELDS[j.kind] || []).flatMap(([f, role]) => (j[f] || '').split(' ').filter(Boolean).map((code) => ({ j, code, role })));
  }
  const bindings = (scene) => scene.joints.flatMap(bound);
  const isBound = (scene, code) => scene.joints.some((j) => bound(j).some((b) => b.code === code));

  // A key went down. Returns true when it belongs to the machine (or to a key being bound).
  function down(e, scene) {
    if (capture) {
      e.preventDefault();
      if (e.code === 'Escape') { const c = capture; capture = null; c.done(null); return true; }
      if (RESERVED.has(e.code)) { capture.refused(e.code); return true; }
      const c = capture; capture = null; c.done(e.code);
      return true;
    }
    if (!isBound(scene, e.code)) return false;
    e.preventDefault();
    if (!e.repeat) {
      held.add(e.code);
      // a motor on 'toggle' flips on (or off) with each press
      for (const b of bindings(scene)) {
        if (b.code !== e.code || b.j.kind !== 'pivot' || b.j.keyMode !== 'toggle') continue;
        const want = b.role === 'fwd' ? 1 : -1;
        b.j._latch = b.j._latch === want ? 0 : want;
      }
    }
    return true;
  }

  function up(e) { held.delete(e.code); }
  // The key caps drawn on the board can be pressed with a mouse or a finger too.
  const press = (code, scene) => down({ code, repeat: false, preventDefault() {} }, scene);
  const release = (code) => held.delete(code);
  function releaseAll() { held.clear(); }

  // Each frame, before stepping: turn held keys into commands. A command eases toward its target
  // over 0.2 s, like a hydraulic valve opening, so a key press isn't a hammer blow.
  function update(scene, dt) {
    for (const j of scene.joints) {
      const keys = bound(j);
      if (!keys.length) { j._cmd = 0; continue; }
      const on = (role) => keys.some((b) => b.role === role && held.has(b.code));
      let target;
      if (j.kind === 'pivot') target = j.keyMode === 'toggle' ? j._latch || 0 : (on('fwd') ? 1 : 0) - (on('rev') ? 1 : 0);
      else target = (on('out') ? 1 : 0) - (on('in') ? 1 : 0);
      if (j.kind === 'rope' || j.kind === 'pulley') { // a winch: the solver pays the rope in or out (physics.js)
        j._lenRate = target * (j.winch || 0.3);
        j._cmd = target;
        continue;
      }
      const c = j._cmd || 0, step = dt / 0.2;
      j._cmd = Math.abs(target - c) <= step ? target : c + Math.sign(target - c) * step;
    }
  }

  // Wait for the next key press to bind. done(code or null if cancelled); refused(code) for a
  // key that can't be bound.
  function listen(done, refused) { capture = { done, refused }; }
  const listening = () => !!capture;
  const cancel = () => { if (capture) { const c = capture; capture = null; c.done(null); } };

  ML.Controls = { label, keyOrder, bindings, bound, isBound, down, up, press, release, releaseAll, update, listen, listening, cancel, held, RESERVED, SHORTCUTS };
})();
