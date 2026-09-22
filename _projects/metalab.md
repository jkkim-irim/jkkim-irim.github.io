---
layout: page
title: MetaLab
description: Engine-agnostic robot RL simulator — define a task once as a single Python contract and train/evaluate it identically on Newton and Genesis.
img: assets/img/metalab.jpg
importance: -2
category: work
redirect: https://github.com/jkkim-irim/metalab
related_publications: false
---

**MetaLab** is an engine-agnostic robot reinforcement-learning simulator framework.
A task is written once as a single Python *contract*; the same environment then
trains and evaluates on two physics engines, **Newton** and **Genesis**, through a
unified API, on a single local GPU.

- One contract file per task; engine-specific code lives in per-engine backends.
- Pinned engine sources and per-engine `uv` environments from one setup script.
- Web launchpad and CLI for training, evaluation, standalone simulation, and
  engine-parity comparison.
