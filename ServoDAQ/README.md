# ServoDAQ

A second, unrelated tool in this repo — a companion firmware
(`ServoDAQ_Companion/ServoDAQ_Companion.ino`) plus a Python host driver
(`ServoDAQ_Host/`), not the Web-Serial browser app documented in the
[main README](../README.md). It exists for one-off bench
characterization/article-testing work — comparing a naive single-step
stall-detection sweep against a smarter coarse+fine, rate-based
`find_range()` on real hardware — and is deliberately kept separate from
`ServoCalibrator.html`/`ServoCalibrator_Companion`: that tool stays
untouched, this kind of exploratory work goes in its own files instead.

No zero reference, no model, no calibration table — that's the host's
job. The firmware's only responsibilities are talking to the servo,
reading the AS5600, and turning raw counts into a signed, monotonic,
multi-turn-unwrapped centidegree position (see the `.ino`'s own
file-level comment for how). Everything else — range-finding, deciding
what "the edge" means, plotting — lives in `ServoDAQ_Host/`.

## Protocol

115200 baud, one command per line, `\n`-terminated ASCII.

- `PING` → `OK PONG`
- `US <pulseUs>` → `OK <pulseUs> <centideg>` (blocks until settled)
- `CAP <pulseUs> <delayMs>` → streamed `CP <tMs> <centideg>` samples,
  `CAPEND <count>` — diagnostic raw step-response capture, for seeing
  motion the settle detector smooths over.

## Host scripts

- `servo_daq.py` — the library: `ServoDAQLink`, `sweep()`,
  `naive_stall_sweep()` (the dumb baseline), `find_range()`/`find_edge()`/
  `scan_until_weak()` (the coarse+fine rate-based algorithm).
- `study_range.py` — the full per-servo characterization pipeline, one
  connection/session: naive sweep → smart `find_range()` → fine (5µs,
  both directions) calibration sweep → N-point lookup-table accuracy
  test (see below). Written to run identically across different servos
  (same code path/phases/thresholds every time) — only the port and the
  accuracy-test duration are meant to vary between invocations.
- `plot_naive_vs_smart.py` — same comparison, but plots naive/coarse/fine
  as three separately colored traces (one connection/session, so the
  centideg reference is consistent across all of it).
- `plot_full_range.py` — reads a saved run's CSVs (no hardware access)
  and draws one combined-axis chart spanning the whole tested range, with
  markers at `find_range()`'s reported min/max.
- `probe_low_jump.py`, `hand_stall_test.py`, `watch_full_swing_jump.py`,
  `full_test_record.py` — the diagnostic scripts written for the
  investigation below; kept as they're generally useful for digging into
  a specific servo's step response, not one-shot throwaways.

`../data/` holds whatever a given run saved (CSVs + PNGs) — regenerated
each session, not version-controlled.

## Investigation: a real stall-recovery backoff, not a bug (2026-08-16)

First real `find_range()` comparison against a second (digital) servo
showed the naive and smart low-side edges disagreeing by roughly 300°
— naive settling around 310–320µs at a wildly different reported angle
than smart's 345–350µs. The high side never showed this; only the low
side, and only past where the smart algorithm's own weakening detector
already flags the real edge.

Six theories were ruled out in order, each on direct evidence rather
than assumption:

1. **Unwrap algorithm bug.** A `CAP`-based fine-grained capture
   (`probe_low_jump.py`, 30ms/sample) across the transition showed a
   smooth, monotonic ~31,000-centideg climb over ~1.6–1.8s — not a
   single discontinuous jump between adjacent samples. `updatePositionTracking()`'s
   wrap detection only misfires on a genuine same-tick discontinuity
   bigger than half a revolution; this wasn't one.
2. **A stray/bad AS5600 sample.** A single bad reading doesn't
   reproduce the same magnitude and S-curve shape (slow start, ramp,
   decelerate to a clean stop) across independent runs. This did,
   repeatedly.
3. **Real horn rotation.** Directly watched: the horn did not move
   during an early capture that still showed the full climb on the
   encoder.
4. **Magnet/sensor mounting.** Magnet and sensor were fully reseated;
   the anomaly's magnitude was unchanged (~311° vs. ~313°) and it still
   happened on the same side at the same threshold. A loose mount would
   have changed something.
5. **Shared power supply (voltage sag under stall current).** Ruled out
   directly — the AS5600 runs on a separate, regulated supply.
6. **Shared ground (conducted ground bounce under stall current).**
   Ruled out directly — the ground is a star topology, not shared with
   the servo's return path.

The actual mechanism, confirmed two ways:

- **Watched it happen.** Using `watch_full_swing_jump.py` (drive to max,
  then min, pause, then command past the edge) and `full_test_record.py`
  (the same sequence, but recorded continuously — a stitched
  pulse-vs-time and angle-vs-time chart across the whole thing, not just
  the final leg), the horn was directly observed to reverse and rotate
  ~300° over about 1.5–2s, matching the encoder's own timing and
  magnitude exactly.
- **Servo-disconnected control.** With the servo physically unpowered
  (zero current, motor fully disconnected), the identical command
  sequence produced a perfectly flat reading (±1 count of sensor jitter)
  for a full 3-second capture — no drift at all. The effect requires the
  motor to be powered and drawing current; it isn't in the sensor, the
  firmware, or the unwrap math, all of which ran the exact same code
  path in both cases.

**Conclusion:** this digital servo has its own internal stall/overcurrent
protection. Driven hard enough past its real mechanical limit for long
enough, it deliberately repositions itself away from the stall — a real,
controlled, servo-executed move (hence the realistic accelerate/cruise/decelerate
shape), not a sensor artifact. `find_range()`'s coarse+fine algorithm had
been avoiding this by luck: its fine scan's small steps and immediate
stop-on-first-weakness happened to back off before ever dwelling in a
hard stall long enough to trigger it. The coarse scan (50µs steps) had
much less margin — and did in fact hit the real thing live during
testing, walking 350→300µs in one step and landing squarely in the
backoff (see `plot_full_range.py`'s output, and the fix below).

### The fix

`scan_until_weak()` (`ServoDAQ_Host/servo_daq.py`) previously only
checked for steps *smaller* than its own self-measured reference rate
(weakness). It had no defense against a step that's backwards or
implausibly large, so a coarse step landing in a hard stall would be
silently accepted as if it were a genuine, fast edge.

It now also ends the scan — handled exactly like reaching a weak step,
not raised as an error — when a step's delta reverses direction relative
to the scan itself, past the same noise floor weakness already uses (so
ordinary ±1-count sensor jitter while genuinely stopped never trips it).
No separate magnitude/"too-large" check or fixed absolute-degree glitch
filter was added — every backoff observed on real hardware was a
reversal, never a same-direction overshoot, which is what "backing off
from an over-extension" should physically look like. `find_edge()`
recovers through its existing fine-pass fallback automatically; no
caller-side changes were needed.

Verified live: `find_range()` now completes cleanly through a run that
previously would have returned a corrupted low edge (350µs, matching
every other clean run), with no false positive on the unaffected high
side.

**Open question, not yet answered:** the exact trigger condition (how
much cumulative stall dwell time is required) isn't characterized —
different runs triggered it with and without a deliberate extra pause.
`find_range()`'s current safety margin against this is real (verified
above) but not precisely measured.

## N-point lookup-table accuracy test (2026-08-17)

`study_range.py`'s third phase, once the fine (5µs) calibration sweep
is captured: direction-averages the up/down sweep into one ground-truth
pulse→angle curve, builds a 2-point linear baseline and 10/20/30/40/50-
point lookup tables from it (`build_table()` — evenly spaced by pulse,
each breakpoint snapped to an actually-measured grid point, not a
fabricated interpolation), then validates against **real hardware**:
repeatedly picks one random target angle, computes every model's
predicted pulse for it via `angle_to_pulse()` (binary search + linear
interpolation, same algorithm the real embedded tools use, run in the
angle→pulse direction since that's the actual usage this validates),
commands each in randomized order (so no model is systematically
first/last and biased by leftover backlash from whichever move happened
right before it), and measures the real resulting angle. This is the
actual end-to-end accuracy that matters — how close the servo lands to
where you asked it to go — not a synthetic curve-fit residual.

Runs to a wall-clock deadline, not a fixed sample count, so a multi-hour
run just works. Two explicit safety measures, both requested after a
servo has died from exactly this kind of unattended extended-duration
stress before: a 1-second rest after every single move, and every
predicted pulse clamped to `[min_pulse_us, max_pulse_us]` regardless of
what the interpolation computes — the test never drives the servo
anywhere the fine sweep didn't already prove is safe. Data safety:
phase-1 results are saved to disk *before* the accuracy test starts, and
every trial row is flushed immediately, so a crash or interruption deep
into an hours-long run never risks losing everything before it.

**First real run** (stamp `20260817-005841`): 3 hours unattended, 1411
rounds, 8466 real trials, one servo.

| model | points | mean\|err\| | median\|err\| | p90\|err\| | max\|err\| | rms |
|---|---|---|---|---|---|---|
| linear2 | 2 | 0.476° | 0.405° | 0.932° | 2.191° | 0.603° |
| table10 | 10 | 0.358° | 0.270° | 0.762° | 2.805° | 0.479° |
| table20 | 20 | 0.344° | 0.261° | 0.735° | 2.342° | 0.455° |
| table30 | 30 | 0.338° | 0.264° | 0.728° | 2.245° | 0.441° |
| table40 | 40 | 0.349° | 0.271° | 0.756° | 2.075° | 0.454° |
| table50 | 50 | 0.343° | 0.276° | 0.718° | 2.003° | 0.441° |

Any table beats the naive 2-point assumption by a real, consistent
margin (mean ~25-29% lower, median ~33-35% lower, RMS ~20-27% lower,
across every table size — not just the largest). Diminishing returns
past ~20-30 points — table20/30/40/50 cluster tightly together, and
going from 20 to 50 points buys essentially nothing further on this
servo, consistent with this project's existing 20-point convention not
being undersized for a servo like this one. One honestly-reported
counterintuitive result: `table10`'s max error (2.805°) is worse than
`linear2`'s (2.191°) even though it wins on every other metric — a
coarser table can locally interpolate worse than a straight line if a
breakpoint lands badly relative to a local kink; spot-checked directly
(trial 902, target 144.855° → landed at 147.66°, pulse in range, no
resemblance to the stall-recovery corruption above, which produced
errors three orders of magnitude larger) and it's real, not a data
artifact.

Raw data: `ServoDAQ/data/accuracy_trials_20260817-005841.csv` (8466
rows) / `accuracy_summary_20260817-005841.csv` — both untracked, still
on disk locally. Not yet done: the same run on the other 7 servos this
tooling was built to test identically.
