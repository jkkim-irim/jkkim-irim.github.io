---
layout: page
title: ALLEX Playground
description: Drive a bimanual robot's hands in the browser — real MJCF, MuJoCo physics via WebAssembly, no install. Early prototype.
img: assets/img/allex-playground.jpg
importance: 0
category: work
redirect: https://jkkim-irim.github.io/allex-playground/
related_publications: false
---

An in-browser sandbox for **ALLEX**, a fixed-base bimanual humanoid with two
15-DOF dexterous hands. The robot is not a game asset — it is the same MJCF used
for simulation research, loaded unmodified and stepped by MuJoCo compiled to
WebAssembly.

Why that matters: each finger's distal joint is driven by a **4th-order
polynomial coupling** (`equality/joint` with `polycoef`). URDF cannot express it
— `mimic` is linear-only — and no game engine's articulation system has the
concept at all. Running the real MJCF is the only way the hand moves correctly.

**Controls** — mouse moves the hand, `W`/`S` height, `Q`/`E` grip rotation,
`Space` closes the hand, `Tab` switches hands. A gamepad, if connected, takes
over the grip axis for analogue pressure.

Physics runs at the model's own 2 ms timestep, substepped to the display, at
roughly 3.5x realtime on a desktop CPU — about 29% of a 60 fps frame budget.
