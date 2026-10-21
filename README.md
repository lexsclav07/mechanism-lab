# Mechanism Lab

A 2D mechanism sandbox that looks like a textbook figure. Build contraptions from blocks, weights,
pins, motors, rods, ropes, pulleys, springs, dampers, pistons and tracks, press Play, and watch them
move with real forces and speeds in SI units (N, m/s, N·m, J).

It started from a classic dynamics figure: a frame hung on two rods, a block sliding inside it
between two springs, and a pendulum hanging off the block. That mechanism is the first example.
Others include an Atwood machine, a quarter-car suspension, an F1 front suspension (double
wishbones, pushrods and rockers on a shaker rig), a double pendulum and a four-bar linkage.

## Running it

Open `index.html` in any browser (Firefox or Chrome on Fedora or Windows both work). There's no build
step and nothing to install. Fonts come from Google Fonts; offline, it falls back to system fonts.

If your browser is fussy about local files, serve the folder instead:

```sh
python3 -m http.server 8000   # then open http://localhost:8000
```

## Playing

| Group  | Part     | What it does |
|--------|----------|--------------|
| Move   | Grab     | Drag any point. Paused: sets the starting pose. Running: pulls with a spring "hand" and shows the force in newtons. Hatched ground anchors can be dragged even mid-swing. |
|        | Follow   | Tap a point on a part and the camera tracks it. |
| Bodies | Block, Weight, Shape | Rigid bodies. Weights dropped on a body stick to it. Any body's weight can be set positive or negative (negative floats up). |
| Joints | Pin, Motor, Glue, Track | Pin two parts together (or one to the ground), drive a pin with a motor (speed in rpm, max torque in N·m), glue parts rigidly, or let a block slide on a track. |
| Links  | Rod, Rope, Pulley | A stiff weightless link; a rope that only pulls when tight; a rope over a wheel. |
| Forces | Spring, Damper, Piston | Hooke's-law spring (k, damping, relaxed length); viscous damper (c); hydraulic piston (speed, max force, stroke). |

Tap a part to open its card, where every setting has a slider and a box for an exact value, plus
live readouts. Space plays and pauses, R resets to where you last pressed Play, and Ctrl+Z / Ctrl+Y
undo and redo. Builds save themselves in the browser, and "Save or load" in the ⋯ menu copies a
build as text.

## How the physics works

`js/physics.js` is a small rigid-body solver using XPBD (extended position-based dynamics, Müller et
al. 2020). Each frame is split into about 100 substeps. Joints are solved by correcting positions
directly, so pins and rods don't drift apart. Springs and weight are ordinary forces. Dampers are
solved implicitly so even very stiff ones stay stable. Motors and pistons are force-limited speed
controllers, so an overloaded actuator really stalls. Pulley ropes wrap round the wheel, and the wrap
counts towards the rope's length.

The solver loses a little energy over time (it shrinks as the step gets smaller). Bodies don't
collide with each other.

## Layout

```
index.html          page, styles and the HUD
js/physics.js       solver: bodies, joints, actuators, stepping
js/render.js        textbook-style drawing on a canvas
js/model.js         scene data, defaults, save/load
js/editor.js        the tools: placing parts, grabbing, snapping, camera follow
js/inspector.js     the card for the tapped part
js/presets.js       example scenes
js/app.js           state, loop, undo, menus, autosave
tests/physics-checks.js   headless checks against textbook answers
```

## Tests

```sh
node tests/physics-checks.js
```

It checks pendulum and spring periods, an Atwood machine's acceleration and rope tension, rope
slack, damper terminal speed, piston force and stall, motor speed under load, negative weight, the
grab hand, and that every example runs for 10 seconds with its joints holding.
