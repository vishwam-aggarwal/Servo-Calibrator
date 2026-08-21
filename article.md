---
title: "Your Servo Is Lying To You"
description: "Nine hobby servos, 55,000 measured moves, and the calibration trick that turns a wobbly robotic arm into a precise one."
pubDate: 2026-08-21
tags: ["Robotics", "Embedded", "Control Theory"]
draft: true
---

I wanted to build the best robotic arm I could out of hobby servos — no exotic actuators, no $400 Dynamixels, just the same $8–$25 servos everyone already has a drawer full of. The bottleneck turned out not to be torque, or speed, or even the servos themselves. It was math.

Every hobby-servo project I'd seen — including my own earlier one, [Servo Calibrator](/projects/servo-calibrator/) — points the shaft with the same two numbers: a minimum pulse width, a maximum pulse width, and a straight line drawn between them. Command 90°, get roughly 1500µs; command 45°, get roughly 1125µs. It's the formula in every tutorial, every library, every datasheet's "quick start." It's also, it turns out, quietly wrong — and the wrongness isn't random. It's a real, measurable, servo-specific curve that a straight line simply can't capture.

So I built a bench rig — an Arduino, an AS5600 magnetic encoder riding on the output shaft as ground truth, and a Python driver that could sweep, measure, and grade any servo I threw at it — and set out to answer one question with actual data, not vibes: **how much accuracy is that straight line actually costing me, and is it worth fixing?**

The fix itself isn't exotic — it's a lookup table. Instead of one straight line from min-pulse to max-pulse, sweep the real servo across its full range, record the actual angle at a bunch of pulse widths, and interpolate between the two nearest real measurements instead of trusting a formula. This obviously works in principle. What I wanted to know was how much it's worth, in real degrees, on real hardware, across more than one servo — because a result from a single unit could just be that unit having a bad day.

## The bench

Nine servos, three families — a Miuzei 25kg Servo, a knockoff MG996R, and an MG90D, three units each — went through the same protocol: a direction-averaged fine sweep against the AS5600 as ground truth, then thousands of independent accuracy trials. Every trial picked a random target angle *and* a random model independently, computed that model's predicted pulse, physically commanded it, waited a full second, and measured the real resulting angle. Target and model are drawn independently on purpose — an earlier pass through this data paired them, and it quietly let a bad model borrow a good model's positioning within the same round. Decoupling them was the difference between a flattering result and a true one.

55,029 independent trials, six models compared per servo (the naive 2-point linear formula, plus 10/20/30/40/50-point lookup tables), across every unit — not a residual against the calibration curve that produced the table in the first place.

<div class="chart-figure">
<p class="chart-title">Mean error vs. model complexity, pooled across all nine servos</p>
<svg viewBox="0 0 680 260" role="img" aria-label="Line chart showing mean absolute error dropping sharply from the naive linear model at 1.33 degrees to a 10-point table at 0.43 degrees, then staying nearly flat through a 50-point table at 0.41 degrees">
<g stroke="var(--border)" stroke-width="1">
<line x1="48" y1="220" x2="656" y2="220" />
<line x1="48" y1="153.3" x2="656" y2="153.3" />
<line x1="48" y1="86.7" x2="656" y2="86.7" />
<line x1="48" y1="20" x2="656" y2="20" />
</g>
<g font-family="var(--font-mono)" font-size="11" fill="var(--text-faint)" text-anchor="end">
<text x="40" y="224">0.0&#176;</text>
<text x="40" y="157.3">0.5&#176;</text>
<text x="40" y="90.7">1.0&#176;</text>
<text x="40" y="24">1.5&#176;</text>
</g>
<g font-family="var(--font-mono)" font-size="11" fill="var(--text-faint)" text-anchor="middle">
<text x="48" y="240">linear2</text>
<text x="169.6" y="240">table10</text>
<text x="291.2" y="240">table20</text>
<text x="412.8" y="240">table30</text>
<text x="534.4" y="240">table40</text>
<text x="656" y="240">table50</text>
</g>
<line x1="48" y1="42.7" x2="656" y2="42.7" stroke="var(--series-2)" stroke-width="1.5" stroke-dasharray="6 4"/>
<path d="M 169.6 162.3 L 291.2 162.9 L 412.8 164.1 L 534.4 164.5 L 656 165.1" fill="none" stroke="var(--series-1)" stroke-width="2.5" stroke-linecap="round" stroke-linejoin="round"/>
<circle cx="48" cy="42.7" r="5" fill="var(--bg-raised)" stroke="var(--series-2)" stroke-width="2.5"/>
<text x="48" y="30.7" font-size="11" fill="var(--text)" font-weight="600" text-anchor="middle">1.33&#176;</text>
<circle cx="169.6" cy="162.3" r="4" fill="var(--bg-raised)" stroke="var(--series-1)" stroke-width="2.5"/>
<text x="169.6" y="150.3" font-size="11" fill="var(--text)" font-weight="600" text-anchor="middle">0.43&#176;</text>
<circle cx="291.2" cy="162.9" r="4" fill="var(--bg-raised)" stroke="var(--series-1)" stroke-width="2.5"/>
<circle cx="412.8" cy="164.1" r="4" fill="var(--bg-raised)" stroke="var(--series-1)" stroke-width="2.5"/>
<circle cx="534.4" cy="164.5" r="4" fill="var(--bg-raised)" stroke="var(--series-1)" stroke-width="2.5"/>
<circle cx="656" cy="165.1" r="4" fill="var(--bg-raised)" stroke="var(--series-1)" stroke-width="2.5"/>
<text x="656" y="153.1" font-size="11" fill="var(--text)" font-weight="600" text-anchor="middle">0.41&#176;</text>
</svg>
<div class="chart-legend">
<span><span class="swatch" style="background: var(--series-1);"></span>Calibration table (10&ndash;50 points)</span>
<span><span class="swatch" style="background: var(--series-2);"></span>Naive 2-point linear model</span>
</div>
<p class="chart-caption">Grand mean absolute error across all nine units, six models, 55,029 independent physically-measured trials.</p>
</div>

The cliff happens once, between the naive formula and the first real table. After that, the line is flat — `table50` is within noise of `table10`, on every family, every time. If you're storing this on an ATmega with 2KB of RAM, that's good news: you don't need fifty points of precision to get fifty points' worth of accuracy. Ten to twenty is the whole win.

## The penalty isn't the same for every servo

Averaged per family, the naive formula's mean error is 0.87° for the Miuzei 25kg Servo, 1.16° for the knockoff MG996R, and **1.97°** for the MG90D — more than double the Miuzei's. The MG90D's calibration curve simply bows further from a straight line than the other two designs do; on one individual unit its naive error hit 2.75°. For that servo, a lookup table isn't optional insurance — it's the difference between a servo that behaves linearly and one that doesn't.

| Family | Naive (linear2) | Calibrated (table20) | Backlash (mean) | Lost in testing |
|---|---|---|---|---|
| Miuzei 25kg Servo | 0.87° | 0.40° | 1.17° | 0 of 3 |
| Knockoff MG996R | 1.16° | 0.31° | 1.03° | 2 of 3 |
| MG90D | 1.97° | 0.57° | 1.33° | 0 of 3 |

The knockoff MG996R deserves its own aside: once calibrated, its accuracy is the best of the three families (as low as 0.18° on one unit) — but two different physical units died mid-study, both from encoder-freeze failures that took the motor with them. The study's original plan was even an 8-hour accuracy phase; the very first candidate tested — a knockoff MG996R — died about 3.5 hours in, which is why every unit after it ran a 3-hour standard instead, and a second knockoff MG996R later died 61 minutes into *that* standard, which is why this family alone now runs on a 1-hour cap. If you build with these: budget for spares.

## Why a single joint can shrug off a degree, and an arm can't

A pan/tilt camera mount can absorb a degree of error and nobody notices the horizon tilt. A robotic arm can't — every joint's error doesn't just sit there, it rides on top of every joint downstream of it, compounding into wherever the end effector actually lands.

<div class="chart-figure">
<p class="chart-title">End-effector drift: naive linear model vs. calibrated table, three-joint arm</p>
<svg viewBox="0 0 620 300" role="img" aria-label="Diagram of a three-joint arm with two overlapping circles at the end effector showing positional drift, roughly 8.4 millimeters for the naive linear model versus 2.7 millimeters for a calibrated table">
<rect x="30" y="240" width="60" height="14" fill="var(--bg-raised)" stroke="var(--border)" stroke-width="1.5"/>
<circle cx="60" cy="232" r="11" fill="var(--bg-raised)" stroke="var(--accent)" stroke-width="2.5"/>
<line x1="60" y1="232" x2="210" y2="170" stroke="var(--text-faint)" stroke-width="2"/>
<text x="130" y="192" font-family="var(--font-mono)" font-size="11" fill="var(--text-faint)" text-anchor="middle">120mm</text>
<circle cx="210" cy="170" r="9" fill="var(--bg-raised)" stroke="var(--accent)" stroke-width="2.5"/>
<line x1="210" y1="170" x2="340" y2="120" stroke="var(--text-faint)" stroke-width="2"/>
<text x="270" y="137" font-family="var(--font-mono)" font-size="11" fill="var(--text-faint)" text-anchor="middle">120mm</text>
<circle cx="340" cy="120" r="8" fill="var(--bg-raised)" stroke="var(--accent)" stroke-width="2.5"/>
<line x1="340" y1="120" x2="440" y2="82" stroke="var(--text-faint)" stroke-width="2"/>
<text x="385" y="98" font-family="var(--font-mono)" font-size="11" fill="var(--text-faint)" text-anchor="middle">120mm</text>
<circle cx="440" cy="82" r="46" fill="none" stroke="var(--series-2)" stroke-width="2" stroke-dasharray="5 4"/>
<circle cx="440" cy="82" r="17" fill="none" stroke="var(--series-1)" stroke-width="2.5"/>
<circle cx="440" cy="82" r="3" fill="var(--text)"/>
<line x1="486" y1="82" x2="540" y2="52" stroke="var(--border)" stroke-width="1" stroke-dasharray="3 3"/>
<text x="546" y="48" font-family="var(--font-mono)" font-size="11" fill="var(--series-2)" text-anchor="start">&#8776;8.4mm &mdash; linear2</text>
<line x1="457" y1="82" x2="540" y2="102" stroke="var(--border)" stroke-width="1" stroke-dasharray="3 3"/>
<text x="546" y="106" font-family="var(--font-mono)" font-size="11" fill="var(--series-1)" text-anchor="start">&#8776;2.7mm &mdash; table20</text>
</svg>
<div class="chart-legend">
<span><span class="swatch" style="background: var(--series-1);"></span>Calibrated (table20)</span>
<span><span class="swatch" style="background: var(--series-2);"></span>Naive linear model</span>
</div>
<p class="chart-caption">Three joints &times; 120mm links, this study's average angular error &mdash; order-of-magnitude estimate, not a full kinematic stack-up.</p>
</div>

Take a modest three-joint arm — shoulder, elbow, wrist, each riding on a 120mm link, well within reach of servos this size. Run this study's average numbers through it: the naive straight-line formula puts roughly **2.8mm** of drift on the end effector from each joint's error, on its own. Stack three of them and a build commanded to hit an exact point can land more than **8mm** away from where you told it to go — enough to miss a socket, misplace a component, or draw a line that visibly isn't straight.

Swap in a 20-point calibration table built from the same kind of sweep this study ran, and each joint's error drops from ~1.3° to ~0.4° — cutting that same three-joint arm's drift to roughly **2.7mm**. That's the difference between an arm whose accuracy is limited by its own math, and one whose accuracy is limited by the thing that should actually be limiting it: backlash, structural flex, and how tightly it was built.

## So, what's the best arm I can build?

Bench-tested and totaled up: any of these three families, run through a 10-to-20-point calibration table instead of the textbook formula, lands within half a degree of its commanded target — sub-millimeter positioning on a typical hobby-arm link. That's not a marginal win worth shrugging off; it's the difference between a robot arm that reaches *roughly* where you tell it to, and one that actually lands there.

Which family to build with comes down to reach, load, and how many spares you're willing to keep on a shelf, not whether it's worth calibrating — on this bench, that was never really in question. The Miuzei is the steady, reliable default for a base or shoulder joint. The MG90D is small, cheap, and needs the table most, which makes it a strong pick for a wrist or gripper specifically because calibration erases its biggest weakness. The knockoff MG996R has the best calibrated accuracy of the three, but earned its reliability caveat honestly — keep spares.
