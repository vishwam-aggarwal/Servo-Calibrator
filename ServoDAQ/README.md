# ServoDAQ

A second, unrelated tool in this repo — a companion firmware
(`ServoDAQ_Companion/ServoDAQ_Companion.ino`) plus a Python host driver
(`ServoDAQ_Host/`), not the Web-Serial browser app documented in the
[main README](../README.md). It exists for one-off bench
characterization/article-testing work — comparing a naive single-step
stall-detection sweep against a smarter coarse+fine, rate-based
`find_range()` on real hardware — and is deliberately kept separate from
`website/app.html`/`ServoCalibrator_Companion`: that tool stays
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
  investigation below. Deleted 2026-08-19 once their findings were fully
  captured here and in `CLAUDE.md`: ad hoc debugging tools for that
  specific, already-resolved question, not part of the standard
  `study_range.py` procedure. Still recoverable from git history.

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
a firmware or algorithm problem. That partial run's data (523-524
samples/model) was discarded 2026-08-18 rather than kept; a clean full
rerun (stamp `20260818-225108`) is in progress as of this writing.

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

### type3_unit1's full study, completed clean (2026-08-18)

Full 3h run (stamp `20260818-194150`) — the first real, complete run on
the `NOT_SETTLED`-fixed firmware: 7766 independent trials, zero
`NOT_SETTLED` events anywhere in the log, zero warnings, servo returned
to center. Range 265-2075µs, -73.6° to 241.6° (~241.6° stroke).

| model | points | n | mean\|err\| | max\|err\| | rms |
|---|---|---|---|---|---|
| linear2 | 2 | 1294 | 1.1274° | 3.3780° | 1.3672° |
| table10 | 10 | 1294 | 0.6587° | 1.7160° | 0.7521° |
| table20 | 20 | 1294 | 0.6225° | 1.7700° | 0.7226° |
| table30 | 30 | 1295 | 0.6244° | 1.8380° | 0.7067° |
| table40 | 40 | 1294 | 0.6024° | 1.7010° | 0.6827° |
| table50 | 50 | 1295 | 0.6338° | 1.6720° | 0.7092° |

Same shape as every other unit — `linear2` clearly worst, table models
cluster with diminishing returns past ~10-20 points — but every number
runs noticeably higher than `type1_unit1`'s (table models ~0.60-0.66°
here vs. ~0.29-0.39° there). Cross-checked against the MATLAB
toolkit's own slope analysis (see below): `type3_unit1`'s calibration
curve has real, measurable waviness — RMS deviation from a straight
line 0.854° vs. `type1_unit1`'s 0.595° — so even the table models have
a less-linear curve to fit against. Second completed unit of the
planned 8-unit study; 6 remain.

## MATLAB analysis toolkit (2026-08-18/19)

`MATLAB/` — a standalone visualization toolkit for the study's own
output, built alongside the data collection itself. Not previously
written up here despite existing since 2026-08-18; this is that
write-up, folded in with the refinements made the following day.

`setup.m` forces itself and `../data` onto the path, then parses every
`study_range.py` CSV into one `MotorTypeData` object per motor type
actually present on disk — type/unit counts discovered from filenames,
never hardcoded, so it automatically picks up new units as the study
progresses (and stays correct if a unit was studied more than once —
most recent stamp wins, since a rerun supersedes an earlier
partial/smoke run and different stamps sit on different, incomparable
position references). Every `*angle_deg` column across every table is
normalized once, here, so the identified min-pulse edge sits at 0° for
every downstream script automatically. `motorTypeNames.m` is a small
lookup mirroring `MOTOR_TYPES.md`'s inventory (type number -> physical
model name); `setup.m` resolves it onto each unit, and every plot now
names units by their actual model ("Miuzei 25kg Servo Unit 2"), never
bare "type1 unit2" — "Type `<N>`" only survives as a last-resort
fallback for a type with no lookup entry.

Four standalone plotting scripts, each calling `setup()` itself:

- `plotHardStops.m` — naive/coarse/fine range-finding overlay per
  unit, min/max edges marked with leader-lined callout labels. Drops a
  sweep's first-point outlier and any non-monotonic tail generically
  (not per-unit/type) — real artifacts caught investigating this
  session's `type1_unit1`/`type3_unit1` data: a stall-recovery backoff
  and a one-off stale first reading, both logged rather than silently
  discarded.
- `plotCalibration.m` — up/down/average fine sweep with hysteresis
  shading, a Savitzky-Golay-smoothed local-slope panel (raw
  point-to-point differentiation was too noisy from AS5600 quantization
  to show real bowing), and a zoomed view of the hysteresis band over a
  fixed window anchored at each unit's own min pulse (the real band is
  too thin to see at full-range scale).
- `plotAccuracy.m` — per-trial error distribution (`boxchart`) plus
  mean/RMS/max summary statistics per model.
- `plotErrorVsAngle.m` — mean |error| by target-angle decile per
  model, showing *where* in the range each model is worst rather than
  only its aggregate error; caught real non-monotonic error patterns
  (not a simple one-directional bow) that line up with the slope
  panel's own waviness findings.

Every legend uses `eastoutside`, not `best` — `best` let a legend box
sit directly over a real trace once actual data (the error-vs-angle
plot's `linear2` peak) made the axes large enough for that overlap to
happen. Within one type's figure, every unit's axes now share
identical scale via a shared `syncAxes()` helper (synced per-row where
rows carry different measurements, e.g. calibration's curve/slope/zoom)
so units are directly visually comparable instead of each silently
auto-scaling to its own range.

## `ABS_CEIL_US` silently capped every unit's max pulse in the whole study (2026-08-25)

Real bug, found by cross-checking against a third, independent tool
(`Servo_Test.ino`, a minimal standalone sketch outside this repo) on
the same bench servo: `find_range()` and `ServoCalibrator_Companion`'s
own `CAL` both reported a max edge around 2070–2085µs, while
`Servo_Test.ino` found a real edge at 2670µs on the identical hardware.
First guess (the shared rate-based edge detection, `scan_until_weak()`,
being too conservative) didn't survive testing —
`CAL_WEAKENING_FRACTION` at 0.10 instead of 0.35 barely moved the
result (2085µs), which was the real clue: the servo's *actual* commanded
position had stopped changing at all past ~2076µs, no matter what
larger pulse was written.

Root cause: `ABS_CEIL_US` (`servo_daq.py` and `ServoDAQ_Companion.ino`
both had it at `3100`) gets passed straight to `Servo::attach()`, which
encodes the ceiling internally as `(MAX_PULSE_WIDTH - max)/4` in a
**signed `int8_t`** (`Servo.h`: `MAX_PULSE_WIDTH` 2400) — `(2400-3100)/4
= -175` doesn't fit in `int8_t` and silently wraps to 81, giving an
*actual* enforced ceiling of `2400 - 81*4 = 2076µs`. Every commanded
pulse above ~2076µs was silently rewritten to 2076µs the whole time —
indistinguishable from a genuine stall, which is exactly what
`find_range()` dutifully (and correctly, given the input it was
actually receiving) reported. `Servo_Test.ino` never hit this because
its own author had already worked out the library's real representable
max (2912µs) and capped itself there deliberately.

**This bug was live for the entire 9-unit study.** Looking back at
every unit's reported max pulse (see the top-level `CLAUDE.md`'s 9-unit
table): every single one clusters at 2065–2080µs regardless of servo
family or gear ratio — not what nine independent mechanical limits
should look like. That lines up exactly with users having reported
type1/type3 (rated ~270°) consistently measuring short of spec, and
type2 (rated ~180°) measuring short too.

**Fixed**: `ABS_CEIL_US` changed from 3100 to 2912 in both
`ServoDAQ_Companion.ino` and `servo_daq.py` (kept in sync by hand per
`servo_daq.py`'s own docstring), and identically in
`ServoCalibrator_Companion.ino` (a separate tool in this repo with the
same bug — see the main `CLAUDE.md`). `CAL_WEAKENING_FRACTION` was
reverted to its original 0.35; it was never the problem.

**Verified on two units so far**, both on real hardware:
- `type1_unit3` (Miuzei 25kg Servo): 325–2075µs (236.2°) →
  **325–2655µs (313.1°)**. A full 3h re-run (5970 trials) on the
  corrected range found the same qualitative result as before the fix —
  `linear2` still clearly worst (now by an even larger margin, since a
  straight line fits worse over a wider real range), table models still
  cluster with diminishing returns past ~10–20 points.
- `type3_unit3` (MG90D): 340–2075µs (229.8°) → **305–2620µs (308.2°)**
  (smoke-tested only so far, 637 trials, same shape).

**Not yet done**: re-running the other 7 units. Every range/stroke
number in `CLAUDE.md`'s 9-unit table, and `website/data.md`'s per-unit
charts, still reflect the pre-fix, capped measurements except for
`type1_unit3`'s range-finding chart (corrected as of this entry — see
`website/data.md`'s own top-of-page caveat). The study's central,
qualitative conclusion — an N-point table beats the naive 2-point
linear formula, with rapidly diminishing returns past ~10–20 points —
held up on both re-verified units under the real, wider range; only the
absolute range/stroke numbers were wrong.
