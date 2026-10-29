# Mechanism Lab

A 2D mechanism sandbox that looks like a textbook figure. Build contraptions from blocks, weights,
pins, motors, rods, ropes, pulleys, springs, dampers, pistons and tracks, press Play, and watch them
move with real forces and speeds in SI units (N, m/s, N·m, J).

It started from a classic dynamics figure: a frame hung on two rods, a block sliding inside it
between two springs, and a pendulum hanging off the block. That mechanism is the first example.
Others include an Atwood machine, a quarter-car suspension, an F1 front suspension (double
wishbones, pushrods and rockers on a shaker rig), a double pendulum, a four-bar linkage, a
motor-driven rod linkage and a piston-driven scissor lift.

The "Vibrations" examples include Kapitza's pendulum (an upside-down pendulum held up by
shaking its pivot), coupled pendulums swapping their swing, and a resonance demo where only the
spring tuned to the shaking builds up. The "Linkages" examples include Watt's and
Peaucellier–Lipkin's straight-line linkages, Chebyshev's lambda linkage and a pintograph
drawing machine. The real machines also include a Whitworth quick-return shaper.

The "Statics" examples are textbook problems with known answers: a simply supported beam, a wall
bracket held by a cable, a cantilever with a couple, and a Warren truss bridge.

The "Real machines" examples are modelled on things you can go and look at:

| Example | What it shows |
|---|---|
| Oil pumpjack | A nodding donkey at 8 strokes a minute. The crank counterweight balances the 150 kg well load. |
| Excavator arm | Boom, stick and bucket on three hydraulic cylinders, each stroke kept clear of dead centre. |
| Tower crane | A trolley runs along the jib and the 500 kg load swings on its cable. |
| Tuned mass damper | Two towers knocked sideways. The one with a tuned pendulum and damper inside (like Taipei 101, tuned with Den Hartog's formulas) settles in about 20 s. The other keeps rocking. |
| Windscreen wipers | Crank-rocker plus parallelogram at real car size. Both arms sweep about 80°. |
| Strandbeest legs | Theo Jansen's linkage from his holy numbers. The feet trace the famous flat-bottomed stride. |
| Trebuchet | A 400 kg counterweight whips a stone round on a sling at about 30 m/s. |

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
| Bodies | Block, Weight, Circle, Shape | Rigid bodies. A circle has anchor points every 1/16 turn round its rim (4, 8, 16 or 32 in its card, on the rim or a smaller ring) that pins, rods, springs and ropes snap to. Weights dropped on a body stick to it. Any body's weight can be set positive or negative (negative floats up). |
| Joints | Pin, Motor, Glue, Track | Pin two parts together (or one to the ground), drive a pin with a motor (speed in rpm, max torque in N·m), glue parts rigidly, or let a block slide on a track. |
| Links  | Rod, Rope, Pulley | A rod is a real part: join rods end to end, pin one anywhere along another, pin two where they cross, glue them into shapes, or put a motor on their joint. Rods weigh 0.2 kg/m by default, or switch on "Massless" for an ideal link. Also a rope that only pulls when tight, and a rope over a wheel. |
| Forces | Spring, Damper, Piston, Load, Moment | Hooke's-law spring (k, damping, relaxed length); viscous damper (c); hydraulic piston (speed, max force, stroke); a point load P (size and angle, drag the way it pushes); a couple C (N·m). |

With the Rod tool, drag from point to point. Starting or ending on a rod's end, on a rod, or on a
body joins the new rod there. A loose end next to a body is pinned to the ground. Tapping an
overlapping spot again picks the next part underneath. A pin's card shows how many parts share it
and has a "Fixed to the ground" switch.

**Exact sizes.** Every part's card has its angle θ (degrees, anticlockwise from horizontal).
Blocks have width and height. Shapes list their sides (AB, BC…) and corner angles (∠B, ∠C…) to
type in exactly: the last side and the angles at its two ends close the shape. While a shape is
selected its corners are lettered on the drawing, and while you draw one the Shape tool shows
each side's length and angle.

**Dark mode.** The ⋯ menu has a Theme switch: Light (the default), Dark, or Auto to follow your
system. It's remembered in the browser.

**Labels.** Labels and values hold their place and update about ten times a second, so they
don't jitter. Drag any label with the Grab tool to lock it where you put it (a dotted leader
points back to its part); drop it back by its part to let it follow again. "Lock labels in place"
in the ⋯ menu locks them all where they are; switching it off frees them all.

Each part's card has an "On the drawing" section to switch its own labels on or off: its name,
its value (a spring's force, a rod's T or C, a load's size…) and its speed. "Textbook labels" and
"Live values" in the ⋯ menu switch them for everything at once.

Tap a part to open its card, where every setting has a slider and a box for an exact value, plus
live readouts. Space plays and pauses, R resets to where you last pressed Play, and Ctrl+Z / Ctrl+Y
undo and redo. Builds save themselves in the browser, and "Save or load" in the ⋯ menu copies a
build as text.

## Driving machines with the keyboard

Any motor, piston or rope can be put on keys, so you can build a machine and then drive it.

- **Motor:** in its card set "Runs" to "On keys" (it picks A / D, then ← / →, J / L…). It turns
  while a key is held, or "Press on / off" makes the keys switches. When it stops it can brake
  (hold still, up to its max torque) or coast. Its speed setting is its top speed.
- **Piston:** set "Does" to "Keys" (W / S, then ↑ / ↓, I / K…). Hold one key to push out and the
  other to pull in. Let go and the valve shuts, so it holds where it is.
- **Rope or pulley rope:** switch on "Winch on keys" (Q / E…) to reel it in or let it out, at the
  winch speed you set. That's how the crane hoists and the drawbridge lifts.

Tap a key button in the card and press any key to change it (Space stays Play / Pause). A bound
key takes over that key's shortcut, and the card says when it does. One key can drive several
parts at once, like both wheels of a cart. Pressing a bound key while paused starts the machine.
The keys are drawn beside each part, light up while held, and can be pressed with a mouse or a
finger too (on a phone or tablet). The "Drive it" examples are ready to go: an excavator, a tower
crane, a robot arm, a drawbridge, and a CoreXY machine (two fixed motors and two crossed belts,
as in many 3D printers: each arrow key turns both motors, and one motor alone moves the head
diagonally).

## Studying engineering mechanics with it

Everything uses the textbook sign convention: x to the right, y up, angles and moments
anticlockwise positive, SI units.

- **Supports.** Pin on a single part makes a pinned support (R_x, R_y). Glue on a single part
  makes a fixed support (R_x, R_y and a moment). A rod on a Track is a roller (one reaction,
  square to the track).
- **Loads.** The Load tool puts a force P on a part. Tap for 100 N straight down, or drag the way
  it pushes. The Moment tool adds a couple.
- **Free-body diagrams.** Tap a part and open its Forces tab. It lists every force on the part
  (weight, each pin or support reaction R_A, R_B…, springs, ropes, pistons, loads, couples) with
  F_x, F_y and the moment of each. Moments can be taken about the centre of mass or any pin. The
  totals are checked against m·a and I·α, so you see "in equilibrium" for a structure and
  Newton's second law for a moving part. Switch on the free-body diagram to see that part on
  its own with every force drawn as an arrow, split into x and y parts if you like, with or
  without the numbers on the arrows. It's for the part you switched it on for, and goes away
  when you pick something else.
- **Internal forces.** For a rod or a block used as a beam, the Forces tab draws the axial force
  N, shear V and bending moment M along its length (hover for values). The point you tapped shows
  N, V and M at that cross-section.
- **Pins and supports.** Tap a pin to see what it does to each part. Rods are named by direction
  and marked T (tension) or C (compression), with any loads acting at that joint: the method of
  joints. A support shows its reaction, and a fixed one its moment too. The angles between the
  parts meeting there are listed.
- **Two parts.** Shift-click a second part (or use "Compare with another part…") to see their
  angles, the angle between them, and the force one puts on the other.
- **Statics view.** "Support reactions and rod forces" in the ⋯ menu labels every support
  reaction and every rod's T or C on the drawing. It's saved with the build (the statics
  examples have it on), so it doesn't follow you into other builds.
- **Equations.** Σ Equations (or E) opens a drawer at the bottom with the whole drawing written
  out to solve by hand: what's given, each component's law (W = m·g, F = k(L − L₀)…), and for
  every part ΣF_x = m·a_x, ΣF_y = m·a_y and ΣM_G = I_G·α, first with symbols, then with the
  numbers put in. Pins get one letter each across the drawing (A, B, C… left to right), so the
  force a pin puts on one part shows up with the opposite sign on the other; a roller is one
  unknown N square to its track. It ends with the count of equations against unknown reactions,
  which tells you whether the structure is statically determinate. "Copy as text" copies it all.

Paused, the forces are worked out for the pose on screen: the state at that instant, which for a
structure that can't move is its static answer. Running, they're averaged over each frame, which
is what makes ΣF = m·a and ΣM = I·α hold exactly.

## How the physics works

`js/physics.js` is a small rigid-body solver using XPBD (extended position-based dynamics, Müller et
al. 2020). Each frame is split into about 100 substeps. Joints are solved by correcting positions
directly, so pins and rods don't drift apart. Springs and weight are ordinary forces. Dampers are
solved implicitly so even very stiff ones stay stable. Motors and pistons are force-limited speed
controllers, so an overloaded actuator really stalls. A motor also adds a little rotor inertia, so a
light crank doesn't jitter. Pistons slow down near the ends of their stroke and have a hydraulic
cushion in the last 2 cm, like real cylinders, so they don't slam into their stops. After turning
round they ramp back up to speed instead of jumping straight to it. A massless
rod between two pins is solved as a single distance constraint. Pins on light rods get extra
solver passes, so a 0.1 kg rod can carry a 20 kg load without stretching. Pulley ropes wrap round the wheel, and the wrap
counts towards the rope's length.

The solver loses a little energy over time (it shrinks as the step gets smaller). Bodies don't
collide with each other.

## Layout

```
index.html          page, styles and the HUD
js/physics.js       solver: bodies, joints, actuators, stepping
js/render.js        textbook-style drawing on a canvas
js/model.js         scene data, defaults, save/load (old builds' rods convert to rod parts)
js/rods.js          rod rules: what a rod end joins, shared pins, ground switch, end handles
js/editor.js        the tools: placing parts, grabbing, snapping, camera follow
js/inspector.js     the card for the tapped part
js/controls.js      keyboard control: key bindings, held keys → motor, piston and winch commands
js/forces.js        free-body diagrams, reactions, N/V/M inside members (engineering mechanics)
js/equations.js     the whole drawing written out as equations (the Equations drawer)
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
grab hand, a rod pendulum's period, rod chains, a motor crank on a light rod, pins healing when
a part is deleted, that old saves move identically after conversion, statics answers (beam
reactions and bending moments, a beam under its own weight, a cantilever, a truss by the method
of joints), keyboard control (a piston drives and then holds, a winch lifts smoothly, motor
brakes hold), ΣF = m·a and ΣM = I·α for every part of every example, that the Equations drawer's
written-out sums add up to the same totals, and that every example runs for 10 seconds with its joints holding.
