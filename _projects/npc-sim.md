---
layout: page
title: NPC Sim
description: An NPC village where the LoRA actually updates every night. This page runs the fast layer in your browser — 12×12 grid, five needs, hand-tuned policy against random.
img: assets/img/npc-sim.jpg
importance: 0.5
category: work
redirect: https://jkkim-irim.github.io/npc-sim/
related_publications: false
---

A village simulator built to watch **continual learning break, hold, and recover.**
Each NPC carries a LoRA adapter that is genuinely trained overnight on that day's
log — not a scripted personality that only pretends to change.

The project runs on two clocks. The **fast layer** — a 12×12 grid, five needs that
grow every step, a policy choosing one of five actions — is what this page runs,
entirely in your browser, no install. The **slow layer** — QLoRA night training on
Qwen2.5-3B plus three probe sets measuring identity, remembered facts, and general
ability — needs a 5090 and stays on the research machine.

**What the slow layer already showed.** Thirty simulated nights, 100 optimizer steps
each: general-ability NLL rose from 2.28 to 3.56 — **+1.285**, catastrophic
forgetting, observed rather than assumed. Identity drifted back up **+0.725** after
bottoming out on day 6, even with identity anchors force-inserted into every batch.
What did *not* appear was a retention curve: facts born on days 1–10 were recalled
as well as facts born on days 21–30. The cause was self-inflicted — rehearsal
sampled the whole past uniformly, which erases the very age gradient it was meant
to expose.

**What this page measures.** 니즈충족률 = `1 − mean(worst need)` across every step.
The worst need, not the average: an NPC who only ever eats scores well on an average
while starving socially. Three candidate formulas were compared over 30 days × 10
seeds before this one was fixed — the average formula still awards 0.775 to a policy
whose hunger has pinned at 1.0.

**Controls** — 재생 / 일시정지 / 처음으로, a seed field (the sim is deterministic, so
a seed replays exactly), speed to ×24 (one simulated day per second, so thirty days
finish in about thirty seconds), and a policy toggle. Switch to **랜덤 행동** and the
number collapses; that gap is the stage's pass criterion.
