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
- `US <pulseUs>` → `OK <pulseUs> <centideg>` (blocks until settled) |
  `ERR NOT_SETTLED <pulseUs> <centideg>` (the settle timeout elapsed
  without ever meeting the criterion — `centideg` here is just the last
  raw reading, not a trustworthy position; see "A servo that spins a
  full turn instead of stalling" below for why this reply exists)
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
repeatedly picks a target angle, computes a model's predicted pulse for
it via `angle_to_pulse()` (binary search + linear interpolation, same
algorithm the real embedded tools use, run in the angle→pulse direction
since that's the actual usage this validates), commands it, and measures
the real resulting angle. This is the actual end-to-end accuracy that
matters — how close the servo lands to where you asked it to go — not a
synthetic curve-fit residual.

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

### A real methodological flaw, caught and fixed the same day

**First run** (stamp `20260817-005841`, since superseded): paired one
target angle with all 6 models per round, model order shuffled within
the round. This clustered 5 of 6 moves per round into ~3µs nudges
between nearly-identical predicted pulses — all 6 models predicting
close together for the same angle — too small to decisively re-engage
the servo's mechanism, letting real backlash/deadband play show up as
scatter unrelated to any model's actual accuracy. Worse: whenever
`linear2` wasn't first in its round, the servo was already sitting near
the *good* answer because a table model had just moved there chasing the
same target — quietly flattering the naive baseline.

Diagnosed directly, not assumed: mean\|err\| by position-within-round
climbed steadily (0.315°→0.402° across positions 2–6) while raw jump
*distance* showed no correlation with error at all (even 1000+µs jumps
had the lowest mean error of any bucket), and signed error stayed ~0 at
every position — ruling out both "big jumps settle badly" and a
directional bias, and pointing specifically at growing *scatter* from
near-zero-delta moves within a round.

**Fix**: `run_accuracy_test()` decouples target angle and model
completely — every trial is fully independent, its own random angle
*and* its own independently-chosen model (drawn from a shuffled bag that
refills on empty, so sample counts stay balanced across models over the
run, not just in expectation). Every move is now a genuine, decisive
jump; no model can benefit from another model's positioning.

**Corrected run** (stamp `20260817-114844`): 3 hours unattended, 6493
fully independent trials (~1082/model).

| model | points | mean\|err\| | median\|err\| | p90\|err\| | max\|err\| | rms |
|---|---|---|---|---|---|---|
| linear2 | 2 | 0.938° | 0.865° | 1.822° | 3.896° | 1.131° |
| table10 | 10 | 0.344° | 0.264° | 0.691° | 2.572° | 0.476° |
| table20 | 20 | 0.338° | 0.252° | 0.670° | 2.594° | 0.488° |
| table30 | 30 | 0.339° | 0.256° | 0.658° | 2.992° | 0.479° |
| table40 | 40 | 0.319° | 0.250° | 0.617° | 2.671° | 0.443° |
| table50 | 50 | 0.312° | 0.247° | 0.596° | 2.427° | 0.427° |

The real margin is much larger than first measured: mean error drops
~63-67% (0.94°→0.31-0.34°), median ~69-71%, RMS ~58-62%, across every
table size — the flawed run had reported only ~25-29%/~33-35%/~20-27%.
`linear2`'s real error also isn't uniform across the range: it climbs
steeply toward one end (angle-decile means ~0.37°→~1.87° peak, the
textbook signature of a straight line failing where the real curve bows
away from it), while even `table10` stays roughly flat (~0.28-0.46°)
everywhere. `linear2` was never actually competitive — the flawed
methodology just hid it by letting it borrow better models' positioning.

Diminishing returns past ~20-30 points still holds: `table20` through
`table50` cluster within ~0.03° of each other on mean error;
consistent with this project's existing 20-point convention not being
undersized for a servo like this one.

One methodological lesson, not a servo finding: the flawed run's one
"counterintuitive" result — `table10`'s max error beating `linear2`'s —
didn't survive the fix. In the corrected run `linear2` has the worst max
of all six models. Max is a single-extreme statistic; noisy enough to
flip entirely once a real measurement bias was removed — don't trust a
surprising max comparison until it replicates.

### Two small follow-ups, same day

- **Degrees, not centidegrees, in the accuracy CSV**: `target_angle_deg`/
  `actual_angle_deg`/`error_deg` replace the old `*_centideg` columns —
  this file has no downstream reader in the codebase (only ad hoc
  analysis), so there's no interoperability reason to match the rest of
  the project's centideg convention, and degrees reads far easier raw.
  Internal math is still centidegrees throughout, same as everywhere
  else in this project.
- **`motor_type`/`unit` in every filename** (`type<N>_unit<M>_<timestamp>_*.csv`),
  ahead of the real 8-servo study — see [`MOTOR_TYPES.md`](MOTOR_TYPES.md)
  for the actual inventory. `type0`/`unit0` is reserved for
  unlabeled/test runs when those CLI args are omitted.

Raw data: `ServoDAQ/data/accuracy_trials_20260817-114844.csv` (6493
rows) / `accuracy_summary_20260817-114844.csv` — both untracked, still
on disk locally at time of writing. Not yet done: the same run on the
other 7 servos in `MOTOR_TYPES.md`.

### Degrees in every CSV, not just the accuracy one (2026-08-17)

The "degrees, not centidegrees" fix above only ever touched the
accuracy-trials CSV. A later `type1_unit1` run crashed (laptop failure)
partway through its accuracy phase; that stamp's data was discarded
(the accuracy CSV was itself well-formed up to its last flushed row —
the crash just meant the run never reached its planned duration, not
that anything was corrupted) rather than kept as a partial result.

Before restarting, extended the same fix to every other CSV
`study_range.py` writes — `naive_low/high`, `fine_up/down`,
`smart_coarse/fine_low/high`, and `summary` all now carry `angle_deg`
instead of raw `centideg`, matching the accuracy CSV's own convention.
`plot_naive_vs_smart.py` (which duplicates the same phase-1 CSV writes
for standalone naive-vs-smart runs) got the identical fix.
`plot_full_range.py` — the one script that re-reads these CSVs from
disk instead of plotting from live in-memory data — was updated to read
the new column and relabel its axes/annotations. The four one-off
investigation scripts' raw capture CSVs
(`probe_low_jump.py`/`hand_stall_test.py`/`watch_full_swing_jump.py`/
`full_test_record.py`) got the same treatment for consistency, though
their in-memory-plotted PNGs are unaffected (still centideg, since
those plot straight from the captured trace, never reloading the CSV).

Internal math is untouched everywhere — ground truth, tables, and every
stall/rate threshold still work in centidegrees, same as before; only
the value actually written to a CSV row gets converted, at the point of
writing (`to_deg_trace()` in both `study_range.py` and
`plot_naive_vs_smart.py`). Purely a Python-side change: the firmware
and its wire protocol are untouched. Applied proactively, before
generating any more data, so no future CSV needs a retroactive
unit correction the way the accuracy CSV alone did previously.

### First real unit of the 8-servo study: type1_unit1 (2026-08-17)

Restarted the crashed run above on the now-fully-degrees CSVs: full
`study_range.py COM9 3 1 1` against `type1_unit1` (Miuzei 25kg Servo,
first of 2 units of that type — see `MOTOR_TYPES.md`). Completed
cleanly, no errors, servo returned to center. Stamp
`20260817-170727`.

Phase 1: naive low edge 310µs, naive high edge 2100µs; smart (real)
range 345–2080µs, -38.76° to 194.06° — a ~232.8° stroke, in the same
ballpark as the wide-range digital servo characterized earlier in this
file, though a different physical unit/model.

Phase 2 (accuracy test, 3h, 6482 independent trials):

| model | points | n | mean\|err\| | max\|err\| | rms |
|---|---|---|---|---|---|
| linear2 | 2 | 1081 | 1.0847° | 4.4880° | 1.2196° |
| table10 | 10 | 1080 | 0.3906° | 2.7480° | 0.5305° |
| table20 | 20 | 1080 | 0.3342° | 2.5040° | 0.4590° |
| table30 | 30 | 1081 | 0.3348° | 2.6940° | 0.4765° |
| table40 | 40 | 1080 | 0.2932° | 2.4130° | 0.4134° |
| table50 | 50 | 1080 | 0.3048° | 2.3750° | 0.4325° |

(`summarize_accuracy()` only ever computed mean/max/rms — no median —
same columns as every other run's summary CSV, unchanged by this run.)
The same qualitative picture as the reference run above holds on this
unit too: `linear2`'s mean error
is ~3x any table model's, and returns diminish sharply past ~10-20
points — `table20` sits within noise of `table50` on every metric here.
Nothing in this first unit contradicts the reference run's conclusions;
it reads as a second, independent confirmation on different physical
hardware rather than a new finding.

Raw data: `ServoDAQ/data/accuracy_trials_type1_unit1_20260817-170727.csv`
(6482 rows) / `accuracy_summary_type1_unit1_20260817-170727.csv`, both
untracked, still on disk locally at time of writing. **Remaining**: 7
more units — `type1_unit2`, `type2_unit1` through `unit3`, `type3_unit1`
through `unit3`.

`type1_unit2`'s own full 3h run (stamp `20260818-161957`) hit a
mid-run `PermissionError` on the serial port at trial 3100 (~86.5min
in) — a transient USB/driver drop (the port re-enumerated cleanly once
the process died; reconnecting and recentering worked immediately), not
a firmware or algorithm problem. Partial data (523-524 samples/model)
is on disk but the run hasn't been redone yet, so `type1_unit2` isn't a
completed unit of the study.

## Investigation: a servo that spins a full turn instead of stalling, and the settle-report bug that hid it (2026-08-18)

First `study_range.py` run against `type3_unit1` (MG90D) made the
servo start visibly spinning continuously during phase 1, unlike every
unit tested so far. Two real, distinct bugs, found by looking directly
at the data rather than guessing — the second guess (a sensor glitch)
didn't survive scrutiny either, and got retracted on the evidence, not
just abandoned.

### Bug 1: the firmware could ack a move it never actually settled

`reportSettled()` in `ServoDAQ_Companion.ino` was called from both the
real-convergence path (`SETTLE_DWELL_TICKS` of sustained low deltas)
and the timeout path (`SETTLE_TIMEOUT_MS`, 3s, elapsed without that
ever happening) — and sent the *identical* `OK <pulseUs> <centideg>`
reply either way. The host had no way to tell a genuine settle from a
timed-out, possibly-still-moving reading; `converged` only decided
which value got reported (averaged stable window vs. last raw
reading), never reached the wire. Fixed: the timeout case now sends
`ERR NOT_SETTLED <pulseUs> <centideg>` instead — a real, distinct
signal, not a fake success. Verified with `arduino-cli compile`/
`upload` and confirmed live (`PING`/`US` round-trip) before any of the
investigation below.

### Bug 2 (or: not a bug) — this isn't a sensor glitch, it's a real decision boundary

With the honest error visible, the first theory was a stray AS5600
read corrupting the wrap-safe accumulator — the exact failure class
this project already found once before (see "Multi-turn position
tracking" above). It didn't hold up: the observed jump magnitudes
across several incidents (-1246.91°, -2572.73°, -10852.65°, ...)
aren't integer multiples of 360° the way a miscounted-wrap origin
requires (nearest multiples were off by 92-212°), and the implied
angular rate for the *first* incident (~415°/s, ~2.08°/tick at 200Hz)
is far below the ~180°/tick threshold the firmware's own wrap logic
needs to even misfire. The numbers are fully consistent with real,
continuous physical rotation, correctly tracked — not a misread.

Running the actual production functions (`naive_stall_sweep`,
`find_edge`), not a proxy script, showed this isn't rare either: 3-4
out of 5 attempts hit it, consistently landing in a narrow 250-270us
band. A fully instrumented raw-curve probe (5us steps, no stopping
logic at all) showed why no threshold could ever catch it in advance:
the response stays smooth and consistent (~0.2-1.6°/step) right up to
and including the step *immediately before* the failure — if anything
that last step was one of the largest deltas in the whole run, not a
fade toward zero. Sweeping `WEAKENING_FRACTION` from 0.15 to 0.55
confirmed it empirically: every value failed at essentially the same
~250-260us point, because there's nothing in the pre-failure data for
any fraction to key off. Below some pulse threshold, this servo
(explicitly built without mechanical hard stops) takes the commanded
target as a cue to spin the long way around to reach it, instead of
stalling — a discrete decision boundary internal to the servo's own
control logic, not a gradual mechanical/electrical falloff. No amount
of slope-watching can see a step function coming.

### The fix: make crossing the boundary once, safely, part of normal operation

Since the edge can't be predicted in advance, the fix stops trying to
avoid it and instead makes triggering it once cheap and recoverable.
In `servo_daq.py`:

- `naive_stall_sweep()`/`scan_until_weak()` catch `NotSettledError`
  from `move_to()` and treat it exactly like reaching the edge — report
  the last point actually reached normally, stop immediately, never
  take the confirmatory step past a limit that's already proven real.
- `ServoDAQLink.recover_from_wrap()` corrects the position reference
  after recovering: moves to a pulse whose real position is already
  trusted, measures the *actual* drift there (not assumed to be a
  clean -360° — the magnitudes above prove that assumption would be
  wrong), and folds the exact difference into a running
  `centideg_offset` that `move_to()` applies transparently to every
  reading for the rest of the session.
- First end-to-end test (`study_range.py` smoke test) found a real gap
  in that design: the immediate recovery point can *also* land past
  the boundary and fail the same way (the edge isn't perfectly
  repeatable run to run — observed anywhere from 250 to 270us).
  `recover_from_wrap()` now takes an ordered list of fallback
  candidates (nearest first, then the scan's own known-safe starting
  pulse) instead of a single point, caught live from a real crash
  traceback, not anticipated in advance.

Verified on real hardware: 10/10 trials (`naive_stall_sweep` +
`find_edge`, 5x each, low side) completed without raising, edges
consistently found at the real 265-325us limit, reported-position
drift held under 4° across all 10 crossings (the internal
`centideg_offset` climbed past 1.8 million along the way — real
excursions kept happening, just fully corrected for). A full
`study_range.py` smoke test (0.25h, `type3_unit1`) then ran clean
end-to-end, including through a real edge-crossing on the low side —
range found (265-270us to 2075-2100us, -72.6° to 168.7°, ~241°
stroke), accuracy test completed normally (644 trials), servo returned
to center. This is a no-op for any servo that never triggers
`NOT_SETTLED` — `centideg_offset` stays exactly 0 and the new exception
handlers never execute — confirmed by tracing the code paths for
`type1_unit1`/`type1_unit2` (both completed full studies with the old
firmware without ever hitting this), not yet re-verified against their
actual hardware.

`type3_unit1` smoke-test results (stamp `20260818-191520`, ~107
samples/model): same shape as every prior unit — `linear2` mean
1.03°, table models cluster 0.47-0.56°, diminishing returns past
~10-20 points. Smoke test only (15min default), not yet a completed
study unit — the full 3h run is still to do.
