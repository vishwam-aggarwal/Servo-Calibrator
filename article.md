---
title: "Your Servo Is Lying To You"
description: "Nine hobby servos, 55,000 measured moves, and the calibration trick that turns a wobbly robotic arm into a precise one."
pubDate: 2026-08-21
tags: ["Robotics", "Embedded", "Control Theory"]
draft: false
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
<line x1="48" y1="42.7" x2="169.6" y2="162.3" stroke="var(--series-2)" stroke-width="2" stroke-dasharray="6 4"/>
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

## The servos aren't built the same, mechanically

Accuracy wasn't the only thing this bench measured. Every unit's full range-finding sweep and fine calibration sweep also produced two more numbers worth knowing before you design around one of these: how much pulse width actually buys you a degree of rotation, and how much backlash — the gap between approaching an angle from above versus below — a static calibration table can't touch at all.

<div class="chart-figure">
<p class="chart-title">Pulse width required per degree of rotation, by family</p>
<svg viewBox="0 0 680 260" role="img" aria-label="Bar chart showing the knockoff MG996R needs 10.8 microseconds of pulse width per degree of rotation, versus 7.4 for the Miuzei 25kg Servo and 7.5 for the MG90D">
<line x1="48" y1="216.0" x2="656" y2="216.0" stroke="var(--border)" stroke-width="1"/>
<text x="40" y="220.0" font-family="var(--font-mono)" font-size="11" fill="var(--text-faint)" text-anchor="end">0</text>
<line x1="48" y1="167.0" x2="656" y2="167.0" stroke="var(--border)" stroke-width="1"/>
<text x="40" y="171.0" font-family="var(--font-mono)" font-size="11" fill="var(--text-faint)" text-anchor="end">3</text>
<line x1="48" y1="118.0" x2="656" y2="118.0" stroke="var(--border)" stroke-width="1"/>
<text x="40" y="122.0" font-family="var(--font-mono)" font-size="11" fill="var(--text-faint)" text-anchor="end">6</text>
<line x1="48" y1="69.0" x2="656" y2="69.0" stroke="var(--border)" stroke-width="1"/>
<text x="40" y="73.0" font-family="var(--font-mono)" font-size="11" fill="var(--text-faint)" text-anchor="end">10</text>
<line x1="48" y1="20.0" x2="656" y2="20.0" stroke="var(--border)" stroke-width="1"/>
<text x="40" y="24.0" font-family="var(--font-mono)" font-size="11" fill="var(--text-faint)" text-anchor="end">13</text>
<rect x="94.3" y="104.4" width="110" height="111.6" fill="var(--series-1)"/>
<text x="149.3" y="94.4" font-family="var(--font-mono)" font-size="12" fill="var(--text)" font-weight="600" text-anchor="middle">7.4 &#181;s/&#176;</text>
<text x="149.3" y="236" font-family="var(--font-mono)" font-size="11" fill="var(--text-faint)" text-anchor="middle">Miuzei 25kg Servo</text>
<rect x="297.0" y="53.2" width="110" height="162.8" fill="var(--series-1)"/>
<text x="352.0" y="43.2" font-family="var(--font-mono)" font-size="12" fill="var(--text)" font-weight="600" text-anchor="middle">10.8 &#181;s/&#176;</text>
<text x="352.0" y="236" font-family="var(--font-mono)" font-size="11" fill="var(--text-faint)" text-anchor="middle">Knockoff MG996R</text>
<rect x="499.7" y="102.9" width="110" height="113.1" fill="var(--series-1)"/>
<text x="554.7" y="92.9" font-family="var(--font-mono)" font-size="12" fill="var(--text)" font-weight="600" text-anchor="middle">7.5 &#181;s/&#176;</text>
<text x="554.7" y="236" font-family="var(--font-mono)" font-size="11" fill="var(--text-faint)" text-anchor="middle">MG90D</text>
</svg>
<p class="chart-caption">All nine units were swept across a similar ~1.6&ndash;1.8ms pulse span. The knockoff MG996R needs ~46% more of it per degree of real rotation than the other two families &mdash; a real gearing difference, consistent across all three of its own units, not a calibration artifact.</p>
</div>

The gearing difference matters for planning a build (a narrower pulse span buys less usable range on the MG996R), but it's a separate question from backlash — the up-sweep and down-sweep of the same fine calibration disagreeing at the same commanded pulse, purely from mechanical slop in the gear train. A lookup table stores one angle per pulse; it has no way to represent "depends which direction you came from," and can't fix this even in principle.

<div class="chart-figure">
<p class="chart-title">Backlash: mean vs. worst-case disagreement, by family</p>
<svg viewBox="0 0 680 290" role="img" aria-label="Grouped bar chart of mean and maximum backlash by servo family. The MG90D has both the highest mean backlash at 1.33 degrees and the highest worst-case backlash at 2.93 degrees">
<line x1="48" y1="246.0" x2="656" y2="246.0" stroke="var(--border)" stroke-width="1"/>
<text x="40" y="250.0" font-family="var(--font-mono)" font-size="11" fill="var(--text-faint)" text-anchor="end">0.0</text>
<line x1="48" y1="189.5" x2="656" y2="189.5" stroke="var(--border)" stroke-width="1"/>
<text x="40" y="193.5" font-family="var(--font-mono)" font-size="11" fill="var(--text-faint)" text-anchor="end">0.8</text>
<line x1="48" y1="133.0" x2="656" y2="133.0" stroke="var(--border)" stroke-width="1"/>
<text x="40" y="137.0" font-family="var(--font-mono)" font-size="11" fill="var(--text-faint)" text-anchor="end">1.6</text>
<line x1="48" y1="76.5" x2="656" y2="76.5" stroke="var(--border)" stroke-width="1"/>
<text x="40" y="80.5" font-family="var(--font-mono)" font-size="11" fill="var(--text-faint)" text-anchor="end">2.5</text>
<line x1="48" y1="20.0" x2="656" y2="20.0" stroke="var(--border)" stroke-width="1"/>
<text x="40" y="24.0" font-family="var(--font-mono)" font-size="11" fill="var(--text-faint)" text-anchor="end">3.3</text>
<rect x="101.3" y="165.9" width="44" height="80.1" fill="var(--series-1)"/>
<text x="123.3" y="157.9" font-family="var(--font-mono)" font-size="10.5" fill="var(--text)" text-anchor="middle">1.17&#176;</text>
<rect x="153.3" y="101.5" width="44" height="144.5" fill="var(--series-2)"/>
<text x="175.3" y="93.5" font-family="var(--font-mono)" font-size="10.5" fill="var(--text)" text-anchor="middle">2.11&#176;</text>
<text x="149.3" y="266" font-family="var(--font-mono)" font-size="11" fill="var(--text-faint)" text-anchor="middle">Miuzei 25kg Servo</text>
<rect x="304.0" y="175.5" width="44" height="70.5" fill="var(--series-1)"/>
<text x="326.0" y="167.5" font-family="var(--font-mono)" font-size="10.5" fill="var(--text)" text-anchor="middle">1.03&#176;</text>
<rect x="356.0" y="120.0" width="44" height="126.0" fill="var(--series-2)"/>
<text x="378.0" y="112.0" font-family="var(--font-mono)" font-size="10.5" fill="var(--text)" text-anchor="middle">1.84&#176;</text>
<text x="352.0" y="266" font-family="var(--font-mono)" font-size="11" fill="var(--text-faint)" text-anchor="middle">Knockoff MG996R</text>
<rect x="506.7" y="154.9" width="44" height="91.1" fill="var(--series-1)"/>
<text x="528.7" y="146.9" font-family="var(--font-mono)" font-size="10.5" fill="var(--text)" text-anchor="middle">1.33&#176;</text>
<rect x="558.7" y="45.3" width="44" height="200.7" fill="var(--series-2)"/>
<text x="580.7" y="37.3" font-family="var(--font-mono)" font-size="10.5" fill="var(--text)" text-anchor="middle">2.93&#176;</text>
<text x="554.7" y="266" font-family="var(--font-mono)" font-size="11" fill="var(--text-faint)" text-anchor="middle">MG90D</text>
</svg>
<div class="chart-legend">
<span><span class="swatch" style="background: var(--series-1);"></span>Mean backlash</span>
<span><span class="swatch" style="background: var(--series-2);"></span>Worst-case (max) backlash</span>
</div>
<p class="chart-caption">Up-sweep vs. down-sweep disagreement at shared pulse values, from the same fine calibration sweep that built each unit's ground truth. The MG90D&mdash;already the servo whose curve bows furthest from a straight line&mdash;also has the most backlash, on average and at its worst.</p>
</div>

It's not a coincidence that the MG90D leads both charts in this section as well as the accuracy table above it: whatever makes its mechanism less linear also tends to make it looser. None of that shows up in a lookup table's numbers directly, but it's exactly the kind of thing a table can't fix — if a build's joints need to settle to a repeatable position regardless of approach direction, that's a mechanical design problem (a hard stop, a spring preload, a consistent single approach direction in software), not a calibration one.

## Why a single joint can shrug off a degree, and an arm can't

A pan/tilt camera mount can absorb a degree of error and nobody notices the horizon tilt. A robotic arm can't — every joint's error doesn't just sit there, it rides on top of every joint downstream of it, compounding into wherever the end effector actually lands.

<div class="chart-figure">
<p class="chart-title">End-effector drift radius: naive linear model vs. calibrated table, three-joint arm</p>
<svg viewBox="0 0 780 300" role="img" aria-label="Diagram of a three-joint arm with two circles at the end effector showing positional drift radius, roughly 8.4 millimeter radius for the naive linear model versus 2.7 millimeter radius for a calibrated table">
<rect x="30" y="240" width="60" height="14" fill="var(--bg-raised)" stroke="var(--border)" stroke-width="1.5"/>
<circle cx="60" cy="232" r="11" fill="var(--bg-raised)" stroke="var(--accent)" stroke-width="2.5"/>
<line x1="60" y1="232" x2="210" y2="170" stroke="var(--text-faint)" stroke-width="2"/>
<text x="130" y="192" font-family="var(--font-mono)" font-size="11" fill="var(--text-faint)" text-anchor="middle">120mm</text>
<circle cx="210" cy="170" r="9" fill="var(--bg-raised)" stroke="var(--accent)" stroke-width="2.5"/>
<line x1="210" y1="170" x2="340" y2="120" stroke="var(--text-faint)" stroke-width="2"/>
<text x="270" y="137" font-family="var(--font-mono)" font-size="11" fill="var(--text-faint)" text-anchor="middle">120mm</text>
<circle cx="340" cy="120" r="8" fill="var(--bg-raised)" stroke="var(--accent)" stroke-width="2.5"/>
<line x1="340" y1="120" x2="470" y2="88" stroke="var(--text-faint)" stroke-width="2"/>
<text x="405" y="97" font-family="var(--font-mono)" font-size="11" fill="var(--text-faint)" text-anchor="middle">120mm</text>
<circle cx="470" cy="88" r="46" fill="none" stroke="var(--series-2)" stroke-width="2" stroke-dasharray="5 4"/>
<circle cx="470" cy="88" r="17" fill="none" stroke="var(--series-1)" stroke-width="2.5"/>
<circle cx="470" cy="88" r="3" fill="var(--text)"/>
<line x1="470" y1="88" x2="511.7" y2="68.6" stroke="var(--series-2)" stroke-width="1.5"/>
<line x1="511.7" y1="68.6" x2="580" y2="40" stroke="var(--border)" stroke-width="1" stroke-dasharray="3 3"/>
<text x="586" y="38" font-family="var(--font-mono)" font-size="11" fill="var(--series-2)" text-anchor="start">radius &#8776; 8.4mm</text>
<text x="586" y="53" font-family="var(--font-mono)" font-size="10" fill="var(--text-faint)" text-anchor="start">linear2</text>
<line x1="470" y1="88" x2="486.4" y2="83.6" stroke="var(--series-1)" stroke-width="1.5"/>
<line x1="486.4" y1="83.6" x2="580" y2="110" stroke="var(--border)" stroke-width="1" stroke-dasharray="3 3"/>
<text x="586" y="108" font-family="var(--font-mono)" font-size="11" fill="var(--series-1)" text-anchor="start">radius &#8776; 2.7mm</text>
<text x="586" y="123" font-family="var(--font-mono)" font-size="10" fill="var(--text-faint)" text-anchor="start">table20</text>
</svg>
<div class="chart-legend">
<span><span class="swatch" style="background: var(--series-1);"></span>Calibrated (table20) &mdash; 2.7mm drift radius</span>
<span><span class="swatch" style="background: var(--series-2);"></span>Naive linear model &mdash; 8.4mm drift radius</span>
</div>
<p class="chart-caption">Three joints &times; 120mm links, this study's average angular error &mdash; order-of-magnitude estimate, not a full kinematic stack-up. Both circles show a <strong>radius</strong>, not a diameter: the distance from the commanded point to where the end effector actually lands, not the width across the uncertainty region.</p>
</div>

Take a modest three-joint arm — shoulder, elbow, wrist, each riding on a 120mm link, well within reach of servos this size. Run this study's average numbers through it: the naive straight-line formula puts roughly **2.8mm** of drift on the end effector from each joint's error, on its own. Stack three of them and a build commanded to hit an exact point can land more than **8mm** away from where you told it to go — enough to miss a socket, misplace a component, or draw a line that visibly isn't straight.

Swap in a 20-point calibration table built from the same kind of sweep this study ran, and each joint's error drops from ~1.3° to ~0.4° — cutting that same three-joint arm's drift to roughly **2.7mm**. That's the difference between an arm whose accuracy is limited by its own math, and one whose accuracy is limited by the thing that should actually be limiting it: backlash, structural flex, and how tightly it was built.

## So, what's the best arm I can build?

Bench-tested and totaled up: run through a 10-to-20-point calibration table instead of the textbook formula, the Miuzei and the knockoff MG996R both land within half a degree of their commanded target — sub-millimeter positioning on a typical hobby-arm link. The MG90D lands a little behind that, at 0.57° — just over a millimeter at the same link length — but that's still a huge jump from its naive 1.97°, just not quite sub-millimeter. Either way, that's not a marginal win worth shrugging off; it's the difference between a robot arm that reaches *roughly* where you tell it to, and one that actually lands there.

Which family to build with comes down to reach, load, and how many spares you're willing to keep on a shelf, not whether it's worth calibrating — on this bench, that was never really in question. The Miuzei is the steady, reliable default for a base or shoulder joint. The MG90D is small, cheap, and needs the table most, which makes it a strong pick for a wrist or gripper specifically because calibration erases its biggest weakness. The knockoff MG996R has the best calibrated accuracy of the three, but earned its reliability caveat honestly — keep spares.
