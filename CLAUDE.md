# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working
with code in this repository.

## What this is

One companion-firmware + Web Serial browser-app pair for characterizing
a hobby RC servo — Arduino Nano, AS5600 magnetic encoder on the servo's
output shaft as ground truth, one self-contained HTML file talking
[Web Serial](https://developer.mozilla.org/en-US/docs/Web/API/Web_Serial_API),
no build step:

- **`ServoCalibrator.html` + `ServoCalibrator_Companion/`** — one button
  (`CALIBRATE`) stall-scans a servo's real mechanical pulse range and
  builds a direction-averaged 20-point calibration lookup table, fully
  automated, on-device. Once calibrated, the same page drives/visualizes
  live trajectories against that table — a step, a continuous
  post-trajectory square wave, or a continuous trajectory-free sine wave
  — with three auto-scaled rolling charts (position/velocity/error) and
  a live toggle between the 2-point linear formula and the 20-point
  table, so the table's accuracy win is visible interactively. Table
  import/export (JSON) lets you skip recalibrating a servo you've
  already measured.

**Purely for characterization, not installation.** No direction test, no
horn-install step, no logical zero shift — always the servo's own
physical frame, `[0, maxAngleDeg]`. Turning a measured range/table into
an actual `RCServoMotorDriver`/`PCA9685MotorDriver` constructor
(direction, mounting offset, logical framing) is left to whatever
application code consumes this tool's exported JSON — that's an
installation concern, this tool has no way to know it.

Depends on two sibling libraries, **Universal-Motor-Interface** and
**Universal-Trajectory-Interface** (both mine, both currently private —
see the README's dependency table). Grew out of `Servo_Auto_Calibrator`
(a hand-built, single-servo characterization project) — its raw CSVs are
archived in [`historical-data/`](historical-data/); see that folder's
own README for what they are.

## History: merged from two separate tools (2026-08-09)

This repo originally shipped **two** separate tools: a wizard-based
`ServoCalibrator.html` (range-finding + an interactive installation
wizard — horn position, direction test, fine trim, logical zero shift —
ending in a generated `RCServoMotorDriver` constructor) and a
`TrajectoryDemo.html` (live trajectory visualizer against a
compile-time-fixed calibration table). Both are still in git history if
the old wizard flow is ever needed as reference.

**Merged into the single tool described above, per explicit direction**:
purely characterization (no wizard, no generated constructor — "the
constructor should be made on UMI side"), always physical-frame
`[0, maxAngleDeg]` ("minimum is always 0, just like the table"), one
button that runs the whole calibration ("one button for calibration that
runs it"), then the existing trajectory-demo interface unchanged, plus
table import/export. This was a real architecture change, not just a UI
merge — see below for what actually had to change and why.

### Why `RCServoMotorDriver` couldn't be reused for this

`RCServoMotorDriver` binds `calTable`/`calTableLen` at **construction**
time — correct for a real application (you know your calibration up
front), wrong for a tool that builds a table live, at runtime, from a
physical sweep and needs to actually use it immediately afterward. So
`ServoCalibrator_Companion.ino` doesn't construct one at all: it drives a
raw `Servo` object directly and calls `ServoCalibrationTable.h`'s free
functions (`computeServoPulseUs()`, the same function
`RCServoMotorDriver::angleToPulseUs()` calls internally) itself, with a
runtime `useTable` flag choosing which `calTable` argument (the real
table, or `nullptr`) goes in on any given tick. This is the same
approach the old `TrajectoryDemo_Companion` used for its live
`MODEL LINEAR`/`MODEL TABLE` switch — this merge just extended it to
also cover the calibration-building side, not only consumption.

### Why the table can't be `PROGMEM`, and what that breaks

Every earlier `CalPoint` table in this project (`GenTable.h`, the
`RCServoCalTableExample`) was a **compile-time** array, declared
`PROGMEM`, read via `pgm_read_word()` — real flash access on AVR's
Harvard architecture. `CALIBRATE` builds its table from a **live physical
sweep at runtime**, so it inherently lives in ordinary SRAM — there is no
way to make a runtime-computed array `PROGMEM` (that's a compile-time
placement, not a runtime one). Pointing UMI's own `pgm_read_word()`-based
readers (`lookupPulseUsFromTable()`, `validateCalTable()`) at a RAM
address would misread RAM as flash and return garbage — documented as a
footgun in `ServoCalibrationTable.h`'s own header comment, and now the
actual reason this firmware can't just call those two functions
directly.

**Fix**: `ServoCalibrator_Companion.ino` has its own
`lookupPulseFromRamTable()`/`validateRamCalTable()` — the minimal
necessary re-implementation of UMI's exact same algorithms (binary
search + linear interpolation; length/ordering/coverage/plausibility
checks) against plain RAM instead of `PROGMEM`. Everything that *doesn't*
touch `PROGMEM` — the `CalPoint` type itself, the linear-formula branch
of `computeServoPulseUs()` (never reads the table pointer at all when
it's `nullptr`), and the shared value-only helpers
(`isPlausiblePulseUs()`, `roundClampToInt16()`,
`CAL_TABLE_MIN_POINTS`/`MAX_POINTS`) — still comes straight from
`ServoCalibrationTable.h` unmodified. If a future revision ever wants
this firmware to fall back to a compile-time default table (e.g. "use
this known-good table until the user recalibrates"), that default *can*
be a real `PROGMEM` array read through UMI's own functions — the
RAM-only path is specifically for the "built live, this session" case.

### Protocol: request/response calibration, autonomous trajectory streaming, in one firmware

The two predecessor firmwares were architecturally different in kind:
`ServoCalibrator_Companion` (old) was host-paced request/response;
`TrajectoryDemo_Companion` ran autonomously and streamed telemetry
continuously via a non-blocking reader. The merged firmware keeps both
patterns, cleanly separated by *when* they're used rather than trying to
unify them into one shape:

- `CALIBRATE` is a **deliberately blocking** one-shot routine (adapted
  directly from UMI's own `examples/RCServoAutoCalibration` — stall-scan
  both directions with a sliding-window net-delta check, sweep twice,
  average) — no other command is serviced while it runs, same as a human
  would expect a physical 1–3 minute sweep to occupy the board. The
  browser's own `sendCommand()` just uses a long timeout (240s) rather
  than needing any new client-side protocol machinery.
- Once calibrated, `loop()` is non-blocking again: `GO`/`SQUARE`/`SINE`/
  `STOP`/`MODEL` are request/response as before, and telemetry streams
  continuously at ~50Hz alongside them, exactly like the old
  `TrajectoryDemo_Companion`.
- `CALRESULT`'s wire format is deliberately **identical** to `IMPORT`'s
  expected input (`<maxAngleDeg> <minPulseUs> <maxPulseUs>` + 20
  `<pulseUs> <angleCentideg>` pairs) — a saved export can be replayed
  as an `IMPORT` command byte-for-byte, and the browser's own
  `parseCalResult()`/`buildImportCommand()` are exact inverses of each
  other. `GETTABLE` re-emits the same shape on demand (without
  recalibrating) so a page reload/reconnect can recover state as long as
  the board itself hasn't reset.
- `zeroRefAngle`/`signConv` (the AS5600 live-reading reference this
  project has used since `TrajectoryDemo_Companion`) come **directly out
  of `CALIBRATE`'s own stall-scan data** now (the low-endpoint's
  running-angle value, and the sign of the endpoint-to-endpoint delta) —
  no separate dedicated probe move needed, since the scan already visits
  both endpoints. `IMPORT` still needs its own quick physical re-anchor
  (~2 small moves) since an imported table carries the pulse curve but no
  live sensor reference for *this* mounting/session.

### Verified on real hardware (2026-08-09)

Full command sequence run against real hardware, not just read-reviewed:
`PING`; `GO` before calibration correctly rejected
(`ERR NOT_CALIBRATED`); a real `CALIBRATE` run (350–2630µs, 214.7° stroke
— matches this servo's previously-known range closely); `GETTABLE`
byte-identical to the `CALRESULT` that produced it; `GO`/`MODEL LINEAR`/
`MODEL TABLE`/`MODEL BOGUS` (correctly rejected)/`SQUARE`/`SINE`/`STOP`
all working; `IMPORT` re-importing the exact just-calibrated table
(including the re-anchor step), `GETTABLE` afterward still exactly
matching the original `CALRESULT`; a malformed `IMPORT` (wrong point
count) correctly rejected. Separately, the browser side was mock-tested
via `claude-in-chrome` against the real page code (not a
reimplementation): the full connect → auto-`GETTABLE`-recovery →
`CALIBRATE` → unlock flow, `Export`'s downloaded JSON matching the
applied calibration exactly, `Import`'s round-trip building the correct
`IMPORT` command and updating the UI, and a malformed import file
rejected client-side before ever touching the serial link.

## Real bugs found calibrating a second (digital) servo (2026-08-10)

First real `CALIBRATE` run against a new servo — a branded digital unit,
different from the analog one this firmware was developed and tested
against — failed immediately: the low-limit stall-scan drove all the way
to the hard safety floor (200µs) without ever detecting a plateau.
**Not a bug** — this servo's real range (or its own internal pulse clamp)
genuinely sits outside bounds tuned for the analog servo. Widened
`ABS_FLOOR_US`/`ABS_CEIL_US` (200/2900 → 80/3100) and it stalled cleanly.

What followed were three real, distinct firmware bugs, found one at a
time via actual hardware diagnostics (added trace output, read the real
data, changed the theory to match) — not guessed and shipped:

1. **A blind `delay(400)` before the first angle sample, on a jump that
   could be nearly the servo's full range.** `sweepUp()` commands its
   `fromUs` in one shot right after the opposite-end stall scan left off
   — for this faster/differently-geared digital servo, that jump could
   complete inside the blind window, so the very first tracked sample was
   already wrong. Fixed by never delaying blind before `updateRunningAngle()`
   starts tracking a move.
2. **That fix alone didn't help** — a real run showed 18 of 20 table
   points stuck at exactly `0`. Added trace output (per-step
   `pulse`/`cur`/`nextIdx`) rather than guessing again, and found the
   actual mechanism: this servo can sit with **zero** measurable movement
   for an extended stretch after a big commanded jump, then snap most of
   the way there in one step — long enough that even `pollUntilSettled()`
   waiting 300ms for sustained stability (bumped up from 120ms, tested,
   made **zero difference** to the corrupted output — proving it wasn't a
   "briefly looked stable mid-ramp" timing issue at all) still returned
   long before the real motion started. Fixed properly: `rampTo()`
   re-commands any big jump in the same small steps/pace the sweep loop
   itself already uses successfully (`SWEEP_STEP_US`/`SWEEP_SETTLE_MS`),
   so no single command sent to this servo is ever large — sidesteps
   whatever internal latency/queuing causes the snap, regardless of the
   exact mechanism. Centralized through `writePulse()`/`currentPulseUs`
   so every big-jump call site (stall-scan start, sweep start, the
   between-scans return to center, `IMPORT`'s re-anchor) ramps from a
   correctly-tracked baseline instead of guessing.
3. **Ramping fixed the zeros, but the values were still wrong** — now
   clustered in a narrow ~30µs band near the low endpoint, and *below*
   the servo's own established minimum pulse. Instrumented further
   (printed the raw `up[]`/`down[]` arrays directly): `sweepUp()` had 18
   points crammed into ~30µs (the signature of one bad reading spiking
   the tracked angle by tens of degrees in a single call, so *all* of
   that step's target crossings got interpolated within one narrow
   window), `sweepDown()` was corrupted end to end. Root cause: a stray
   AS5600 read, at any point, permanently corrupts
   `updateRunningAngle()`'s wrap-safe accumulator — the exact failure
   class this project already hit once before with a *different* AS5600
   wrapper's continuous mode, just not yet guarded against in this
   firmware's own hand-rolled tracking. Since every step is now provably
   small (fix #2 made that true everywhere), it's finally safe to reject
   implausible single-step deltas outright without risking a legitimate
   big jump being mistaken for a glitch: `REJECT_THRESHOLD_DEG` (20°)
   in `updateRunningAngle()` discards an implausible reading rather than
   trusting it, without advancing `lastRawDeg`, so the next (presumably
   good) sample compares against the same last-known-good reference.

**This also overturned an earlier "result"**: every prior successful-looking
run on this servo had reported 250–2080µs / ~76° stroke — a real,
reproducible number, just wrong, corrupted by exactly the bugs above
(the stall-scan itself was affected, not only the sweep). The real range
is **350–2080µs, 237.4° stroke** — confirmed reproducible across two
independent clean runs post-fix (range, and every table point, within
1–2µs of each other) — a genuinely wide-range digital servo, not the
narrow ~76° the corrupted runs kept reporting.

## Real bug found in the browser app, live (2026-08-10)

First real browser session against the fixed firmware surfaced a
protocol-level bug in `ServoCalibrator.html` itself, not the device —
caught from the user's own pasted serial log, not a lab test:

An `IMPORT` was sent, the firmware genuinely started processing it
(`# IMPORT: re-anchoring live zero reference...` proves it), but
`IMPORT`'s client-side timeout was still `15000ms` — left over from
before `rampTo()` existed. `IMPORT`'s re-anchor now does up to three
`rampTo()` moves, and the first one (from wherever the servo happens to
be sitting to `minPulseUs`) can legitimately take 20+ seconds on its
own. The 15s client timeout fired while the device was still correctly
working in the background — **a client-side timeout doesn't stop the
device**, it only gives up on that one promise. The UI let more commands
through right after (`GO`, a second `IMPORT`, `CALIBRATE`), which queued
up behind the still-in-flight first `IMPORT` on the wire. Once the first
reply *did* eventually arrive, it got consumed by whichever
`sendCommand()` call happened to be waiting at that moment — not
necessarily the one that sent it — and every reply after that was
misdelivered by one slot (`GO` showed an `IMPORT` usage error; `CALIBRATE`
logged the same stale error too). Not several bugs, one root cause:
overlapping commands are never safe against this firmware's blocking,
single-threaded design, and nothing prevented the UI from sending them.

Fixed two ways, not just the one timeout value that happened to trigger
it:
- `IMPORT`'s timeout: `15000ms` → `90000ms`, generous margin over the
  real worst case.
- **A command busy-lock** (`commandBusy`/`setBusy()`/`sendGuarded()`):
  every command button (`Calibrate`/`Go`/`Square`/`Sine`/`Stop`/the model
  toggle/the import file label) is disabled for the duration of any one
  in-flight command, structurally preventing the overlap that caused
  this — not just tuning the one timeout that happened to be too short
  this time. Every `link.sendCommand()` call site now goes through
  `sendGuarded()` instead of calling it directly.

Verified via `claude-in-chrome` mock testing against the real page code:
a deliberately slow-to-resolve command correctly disables every other
command button (confirmed a second click while busy never even sends —
the button was truly `disabled`, not just visually dimmed); buttons
correctly re-enable after both a successful resolve and a rejected/timed-out
one; `sendGuarded()` correctly refuses to send when already busy with a
clear message.

## A second live-session bug: boot-time serial garbage misdelivered as a reply (2026-08-10)

Reloaded the app after the busy-lock fix above; the very next real
session hit a *different* command-desync bug, from the user's own pasted
log again: `> IMPORT ...` was immediately followed by
`# import failed: Import rejected: ERR NOT_CALIBRATED` — but right below
that, `# IMPORT: re-anchoring live zero reference...` then `< OK` proved
the device had genuinely accepted and completed the import. The UI just
never found out.

Root cause, one line up in the log: `< �)# ServoCalibrator_Companion
booting...` — garbled bytes (a common artifact of the USB-serial adapter
resyncing right as opening the port resets the board via DTR) prefixed
what should have been a clean `#`-only boot-log line. `SerialLink._onLine()`
only recognizes a line as ignorable log noise if it *starts* with `#` —
this one didn't, because of the garbage prefix, so it fell through to the
generic reply path and got queued as if it were an answer to some future
command, even though nothing had been sent yet. The next real command
(`GETTABLE`, sent automatically on connect) consumed that stray line as
its own reply instead of waiting for the device's real one; the device's
*actual* `GETTABLE` reply then queued up in turn and got consumed by the
next real command (`IMPORT`) instead — misdelivering every reply after
the boot garbage by one slot, same failure shape as the busy-lock bug
above, different trigger (a garbled boot byte, not an overlapping
command).

Fixed by discarding whatever's in `link.lineQueue` right after the ready
banner is confirmed, before sending the first real command
(`GETTABLE`) — nothing legitimate can be queued that early in a session,
so anything sitting there at that point is unambiguously boot-window
noise, safe to drop unconditionally. Verified via `claude-in-chrome`:
injected a stray line before the ready banner arrived, confirmed it's
queued, then confirmed it's fully drained before `GETTABLE` is sent, and
that `GETTABLE`'s real reply is correctly applied afterward.

Two real bugs in a row from real usage, not lab conditions — both the
same underlying failure mode (a stray/late/misrouted line silently
consumed by the wrong pending command, cascading into everything after
it), triggered by two different, unrelated causes. Worth remembering if
a third variant ever shows up: the request/response `SerialLink` design
has no general defense against *any* unexpected line reaching the reply
queue — each fix so far has closed one specific way that can happen
(overlapping commands; boot-time garbage) rather than the queue itself
being made robust to it in general.

## ServoDAQ: a separate bench-characterization tool (2026-08-16)

`ServoDAQ/` is a **second, unrelated tool** in this repo — a companion
firmware (`ServoDAQ_Companion/ServoDAQ_Companion.ino`) plus a Python host
driver (`ServoDAQ_Host/servo_daq.py`, `study_range.py`), not the
Web-Serial browser app described above. It exists for one-off bench
characterization/article-testing work (comparing a naive stall-detection
sweep against a smarter rate-based `find_range()`, on real hardware) and
is deliberately kept out of `ServoCalibrator.html`/`ServoCalibrator_Companion`
— that tool stays untouched; this kind of exploratory work goes in its
own files instead.

Protocol: `PING`, `US <pulseUs>` (write + block until settled, reply
`OK <pulseUs> <position>`), `CAP <pulseUs> <delayMs>` (diagnostic raw
step-response capture, streamed). No zero reference, no model — that's
the host's job, same philosophy as `ServoCalibrator_Companion`.

### Multi-turn position tracking added, wire format switched to centidegrees

The board originally reported the AS5600's raw 0–4095 count directly.
Per explicit direction, this was replaced with on-device multi-turn
tracking: `updatePositionTracking()` in the `.ino` is the only place
anything reads the encoder, runs first every tick (before any
mode-specific logic), and folds each new raw sample against the
previous one — a same-direction jump bigger than half a revolution
(2048 counts) between two consecutive samples is treated as a wrap and
credited to a signed lap counter (`turnCount`) rather than mistaken for
real motion. This is safe because tick() samples at 200Hz (5ms) and
CAP samples even faster — no hobby servo can complete a full revolution
between two consecutive samples at that rate. Position is reported as
signed centidegrees (degrees×100) — `n=2` chosen because the AS5600's
native resolution is ~8.79 centidegrees/count, so centidegrees resolve
~8.8× finer than the sensor's own quantization without adding fake
precision, and match the `angleCentideg` convention already used
elsewhere in this project. `US`'s and `CAP`'s reply formats both
changed accordingly; `servo_daq.py`'s old `wrapped_delta_counts()`/
`unwrap_trace()` host-side wrap-correction helpers were deleted since
the board's own values are already monotonic — a plain subtraction is
now always the true delta.

A first real hardware run surfaced a large single-step jump in the
naive low-side sweep, right near this servo's low mechanical limit —
the same physical unit with previously-documented (see the
`ServoCalibrator_Companion` history above) sit-then-snap behavior near
its travel limits. A `REJECT_THRESHOLD_COUNTS` glitch filter (mirroring
that earlier fix's `REJECT_THRESHOLD_DEG`) was added to investigate,
but this was **not requested** and the user already has testing showing
it isn't needed here — reverted back to the plain wrap-only logic above
(confirmed byte-identical compile size to before the detour). The
firmware currently ships with **no** glitch/reject filtering; the
anomaly itself is a known, open, unresolved observation, not something
this session's firmware papers over.

## ServoDAQ: a real stall-recovery backoff, not a bug (2026-08-16/17)

A `find_range()` run against the same digital servo from the section
above showed its low-side edge disagreeing with the naive sweep by
~300° — reproducible, same magnitude, same side, every run. Chased
through six ruled-out theories, each on direct evidence (unwrap
algorithm bug — the transition is smooth over ~1.6–1.8s, not a single
discontinuous jump; a stray AS5600 sample — too reproducible in
magnitude/shape to be random; real horn rotation — watched directly, it
didn't move; magnet/sensor mounting — fully reseated, anomaly unchanged;
shared power supply; shared ground — both ruled out directly, separate
regulated supply and star ground respectively), down to the real cause:
**this digital servo runs its own internal stall/overcurrent
protection.** Driven hard enough past its real mechanical limit for
long enough, it deliberately repositions itself away from the stall —
real, controlled, servo-executed motion (confirmed by directly watching
it happen, and by a servo-disconnected control: zero drift with zero
motor current, identical command sequence). Full blow-by-blow —
including the two live-observation experiments that pinned this down —
is written up in [`ServoDAQ/README.md`](ServoDAQ/README.md); this entry
is the short version.

`scan_until_weak()` (`servo_daq.py`) only ever checked for steps
*smaller* than its self-measured reference rate (weakness) — no defense
against a step that's backwards or implausibly large, so a coarse scan
that outran the weakening detector (confirmed live: one real run walked
350→300µs in a single 50µs step before anything looked wrong) would
silently accept the backoff as a genuine edge. **Fix**: a step now also
ends the scan — handled exactly like reaching a weak step, not raised as
an error — when its delta reverses direction relative to the scan
itself, past the same noise floor weakness already uses. No separate
magnitude/"too-large" check and no fixed absolute-degree glitch filter
— every backoff observed on real hardware was a reversal, never a
same-direction overshoot, which is what "backing off from an
over-extension" should physically look like; monotonicity alone turned
out to be the real signature. `find_edge()` recovers through its
existing fine-pass fallback automatically. Verified live both ways:
`find_range()` now completes cleanly through a run that previously would
have returned a corrupted low edge, with no false positive on the
unaffected high side.

**New host scripts** (`ServoDAQ_Host/`, all diagnostic, none touching
firmware): `plot_naive_vs_smart.py` (now plots naive/coarse/fine as
three separately colored traces instead of one merged "smart" line),
`plot_full_range.py` (single combined-axis view with markers at
`find_range()`'s reported min/max), `probe_low_jump.py`/
`hand_stall_test.py`/`watch_full_swing_jump.py`/`full_test_record.py`
(the investigation scripts that isolated and confirmed the root cause —
the last one records a continuous pulse-vs-time + angle-vs-time timeline
across a whole multi-phase sequence, stitching per-phase `CAP` bursts
onto one clock). Those four one-off investigation scripts were deleted
2026-08-19, once their findings were fully captured in this file and
`ServoDAQ/README.md` — ad hoc debugging tools for a specific
already-resolved question, not part of the standard `study_range.py`
procedure; still recoverable from git history if a similar question
ever comes up again.

**`study_range.py` now also runs a fine calibration sweep**, once
`find_range()` has reliably located the real min/max: plain `sweep()`
(no stall detection needed — the range is already established safe) at
5µs steps across the full range, in both directions (min→max, then
max→min), saved as `fine_up_<stamp>.csv`/`fine_down_<stamp>.csv`. Doing
both directions separately (rather than one round trip) isolates
direction-dependent backlash from angle-dependent nonlinearity, same
reasoning as `historical-data/hysteresis_data.csv` from the project this
repo grew out of. On the servo tested (2026-08-17 session, stamp
`20260817-002340`): 347 points/direction, 233.1° measured stroke,
backlash consistently **negative across every shared point** (never
crosses zero) — down reads 0.92° higher than up on average, peaking at
1.66° near 1430µs. A one-signed offset like that is real mechanical
backlash, not sensor scatter. The same run also now saves the smart
algorithm's own coarse/fine sub-traces (`smart_coarse_low/high_*.csv`,
`smart_fine_low/high_*.csv`), so one connection/session produces every
trace needed to plot naive vs. coarse vs. fine vs. the fine calibration
sweep together, all on the same centideg zero reference. (Reminder:
opening the serial port resets the board and re-zeros that reference —
CSVs from *different* sessions are on different, incomparable zero
references. Don't combine across stamps.)

A results page (charts built from that session's CSVs — naive vs.
coarse vs. fine with min/max markers, the up/down fine sweep, and the
hysteresis delta) was published as a private Claude artifact,
["Servo Bench Readout"](https://claude.ai/code/artifact/8c7a669a-15ba-4ec8-bfe8-f2d6076e6e5b).
Not the source of truth — the CSVs under `ServoDAQ/data/` (untracked,
regenerated per session, still on disk locally) and this file are.

`ServoDAQ/data/` remains untracked (never has been, even for the
original ServoDAQ commit) — CSV/PNG outputs are regenerated per session,
not curated the way `historical-data/` is.

### N-point table accuracy vs. the naive 2-point assumption (2026-08-17)

Built into `study_range.py` as a third phase after the fine sweep (see
`ServoDAQ/README.md` for the full mechanics: direction-averaged ground
truth, `build_table()`/`angle_to_pulse()`). Validates against **real
hardware, live**, not just a residual against the ground-truth curve:
every trial picks a target angle, computes a model's predicted pulse for
it, physically commands it, and measures the actual resulting angle.

**First run (stamp `20260817-005841`, since superseded — see below)**
paired one target angle with all 6 models per round, shuffling model
order within the round. This was a real methodological flaw: 5 of 6
moves per round ended up as ~3µs nudges between nearly-identical
predicted pulses (all 6 models predicting close together for the same
angle) — too small to decisively re-engage the servo's mechanism,
letting real backlash/deadband play show up as scatter unrelated to any
model's actual accuracy. Worse, whenever `linear2` wasn't first in a
round, the servo was already sitting near the *good* answer because a
table model had just moved there chasing the same target — quietly
flattering the naive baseline. Caught by direct diagnosis, not
assumption: mean\|err\| by position-in-round climbed 0.315°→0.402°
across positions 2–6 while jump *distance* alone showed no correlation
with error at all, and signed error stayed ~0 at every position (ruling
out a directional bias) — the signature of growing scatter from
near-zero-delta moves, not a big-jump settling problem.

**Fix**: `run_accuracy_test()` now decouples target angle and model
completely — every trial is fully independent (its own random angle
*and* its own independently-chosen model, drawn from a shuffled bag that
refills on empty so sample counts stay balanced), so every move is a
genuine, decisive jump. Re-ran 3 hours unattended, same safety measures
(1s rest after every move, every pulse clamped to
`[min_pulse_us, max_pulse_us]`), stamp `20260817-114844`: 6493
independent trials (~1082/model).

| model | points | mean\|err\| | median\|err\| | p90\|err\| | max\|err\| | rms |
|---|---|---|---|---|---|---|
| linear2 | 2 | 0.938° | 0.865° | 1.822° | 3.896° | 1.131° |
| table10 | 10 | 0.344° | 0.264° | 0.691° | 2.572° | 0.476° |
| table20 | 20 | 0.338° | 0.252° | 0.670° | 2.594° | 0.488° |
| table30 | 30 | 0.339° | 0.256° | 0.658° | 2.992° | 0.479° |
| table40 | 40 | 0.319° | 0.250° | 0.617° | 2.671° | 0.443° |
| table50 | 50 | 0.312° | 0.247° | 0.596° | 2.427° | 0.427° |

**The corrected margin is much larger than first measured**: mean error
drops ~63–67% (0.94°→0.31-0.34°), median ~69-71%, RMS ~58-62%, across
every table size — the flawed run had reported only ~25-29%/~33-
35%/~20-27%. `linear2`'s real error also isn't uniform: it climbs
steeply toward one end of the range (angle-decile means ~0.37°→~1.87°
peak) — the textbook signature of a straight line failing where the real
curve bows away from it — while even `table10` stays roughly flat
(~0.28-0.46°) across the whole range. `linear2` was never actually
competitive; the flawed methodology just hid it by letting it borrow
better models' positioning within a shared round.

**Diminishing returns past ~20-30 points still holds.** `table20`
through `table50` cluster within ~0.03° of each other on mean error;
`table50` edges out the others on every metric this run, but the gap to
`table20` is within noise. This project's existing 20-point convention
(`ServoCalibrationTable.h`, used elsewhere in this repo) isn't undersized
for a servo like this one.

**A methodological lesson, not a servo finding**: the flawed run's one
"counterintuitive" result — `table10`'s max error beating `linear2`'s —
didn't survive the fix. In the corrected run `linear2` has the worst max
of *all six* models. Max is a single-extreme statistic over ~1082
samples; noisy enough to flip entirely once a real measurement bias was
removed. Don't trust a surprising max comparison until it replicates.

Two small follow-up fixes, same day: the accuracy CSV now writes angle
columns in plain degrees (`target_angle_deg`/`actual_angle_deg`/
`error_deg`) instead of centidegrees — this file has no downstream
reader in the codebase, so there's no interoperability reason to match
the rest of the project's centideg convention, and degrees reads far
easier raw. And every output filename now carries `motor_type`/`unit`
(plain integers, `type<N>_unit<M>_<timestamp>_*.csv`) ahead of the
planned real 8-servo study — see `ServoDAQ/MOTOR_TYPES.md` for the
actual inventory (type 1: Miuzei 25kg Servo ×2, type 2: knockoff MG996R
×3, type 3: MG90D ×3). `type0`/`unit0` is reserved for unlabeled/test
runs when those args are omitted.

Raw trials for the corrected run: `ServoDAQ/data/accuracy_trials_20260817-114844.csv`
(6493 rows, untracked, still on disk at time of writing). Per-model
summary: `accuracy_summary_20260817-114844.csv`.

**Not yet done**: the same run on the other 7 servos in `MOTOR_TYPES.md`
(`study_range.py` is written to run identically across all of them —
same code path, same phases, same thresholds; only port,
accuracy-hours, and motor_type/unit differ between invocations).

### Degrees in every sweep/calibration CSV, and a lost type1_unit1 run (2026-08-17)

A laptop crash killed an in-progress `study_range.py` run for
`type1_unit1` (stamp `20260817-152410`) partway through its accuracy
phase — the accuracy CSV itself was well-formed up to the last flushed
row (the per-row `flush()` documented in `study_range.py`'s own
docstring did its job), but the run never reached its planned duration,
so the whole stamp's output (naive/smart/fine/summary/accuracy — all
untracked under `ServoDAQ/data/`) was discarded rather than kept as a
partial result.

Before restarting that run, extended the previous session's "degrees,
not centidegrees, in the accuracy CSV" fix (see `ServoDAQ/README.md`'s
"Two small follow-ups" entry) to **every** CSV `study_range.py` and
`plot_naive_vs_smart.py` write — `naive_low/high`, `fine_up/down`,
`smart_coarse/fine_low/high`, and `summary` now all carry an `angle_deg`
column instead of raw `centideg`, plus the same conversion in the
one-off investigation scripts' capture CSVs
(`probe_low_jump.py`/`hand_stall_test.py`/`watch_full_swing_jump.py`/
`full_test_record.py`). `plot_full_range.py` (the only script that
re-reads these CSVs rather than plotting from live in-memory data) was
updated to read the new column and relabel its plot accordingly. Same
convention as before: internal math (ground truth, tables, stall/rate
thresholds) stays in centidegrees throughout — only the values actually
written to a CSV get converted, at the point of writing. Purely a
Python-side change; the firmware and its wire protocol are untouched
(still centidegrees). Done proactively, before generating any more
data, specifically so future CSVs never need a retroactive/after-the-
fact unit correction the way the accuracy CSV alone did previously.

Also added a repo-specific `wrap-up` skill
(`.claude/skills/wrap-up/SKILL.md`) codifying this repo's actual
end-of-session sequence (docs → branch → commit/push → PR → merge →
memory, each done only if not already done) after doing that sequence
by hand enough times to be worth writing down.

### First unit of the multi-servo study: type1_unit1 (2026-08-17)

Restarted after the crash above, on the now-all-degrees CSVs: full
`study_range.py` run against `type1_unit1` (Miuzei 25kg Servo, first of
2 units — see `ServoDAQ/MOTOR_TYPES.md`), 3h accuracy phase, 6482
independent trials, stamp `20260817-170727`. Range: 345–2080µs,
-38.76° to 194.06° (~232.8° stroke).

| model | points | mean\|err\| | max\|err\| | rms |
|---|---|---|---|---|
| linear2 | 2 | 1.085° | 4.488° | 1.220° |
| table10 | 10 | 0.391° | 2.748° | 0.531° |
| table20 | 20 | 0.334° | 2.504° | 0.459° |
| table30 | 30 | 0.335° | 2.694° | 0.477° |
| table40 | 40 | 0.293° | 2.413° | 0.413° |
| table50 | 50 | 0.305° | 2.375° | 0.433° |

Same shape as the earlier reference run in `ServoDAQ/README.md`:
`linear2` clearly worst (~3x every table model's mean error), and
diminishing returns past ~10-20 points hold again — `table20` is
already within noise of `table50`. First real data point toward the
planned 8-unit study; 7 units remain (`type1_unit2`, `type2_unit1-3`,
`type3_unit1-3`). Raw data:
`ServoDAQ/data/accuracy_trials_type1_unit1_20260817-170727.csv` (6482
rows) / `accuracy_summary_type1_unit1_20260817-170727.csv`, both
untracked.

`type1_unit2`'s own full 3h run (stamp `20260818-161957`) hit an
unrelated mid-run `PermissionError` on the serial port at trial 3100
(~86.5min in) — transient USB/driver drop, not a firmware/algorithm
issue, servo recovered fine once reconnected. That partial run's data
was discarded (2026-08-18) rather than kept, and a clean full rerun
(stamp `20260818-225108`) is in progress as of this writing.

## ServoDAQ: a servo that spins a full turn instead of stalling, and the settle-report bug that hid it (2026-08-18)

First `study_range.py` run against `type3_unit1` (MG90D) made the
servo visibly spin continuously instead of behaving like every servo
tested so far. Root cause, found by looking at the actual data rather
than guessing, took two real fixes:

**1. The firmware was lying about settling.** `reportSettled()` in
`ServoDAQ_Companion.ino` sent an identical `OK <pulseUs> <centideg>`
reply whether the shaft genuinely converged (`SETTLE_DWELL_TICKS` of
sustained low deltas) or `SETTLE_TIMEOUT_MS` (3s) elapsed without that
ever happening — the host had no way to tell a real settle from a
timed-out, possibly-still-moving reading. Fixed: the timeout case now
replies `ERR NOT_SETTLED <pulseUs> <centideg>` instead. Verified via
`arduino-cli compile`/`upload` and real hardware.

**2. Once that was visible, the real behavior turned out not to be a
sensor glitch.** First guess (a stray AS5600 read corrupting the
wrap-safe accumulator, the same failure class documented below in
"Multi-turn position tracking") didn't survive scrutiny: the observed
jump magnitudes (e.g. -1246.91°, -2572.73°, -10852.65°) aren't
integer multiples of 360° the way a miscounted-wrap origin would
require, and the implied angular rate (~415°/s) is far below what
would even engage the firmware's wrap-detection logic falsely. Real,
continuous physical rotation, correctly tracked — not a misread.
Running the actual production functions (`naive_stall_sweep`,
`find_edge`) on real hardware showed this isn't rare either: 60-80% of
attempts hit it, consistently in a narrow 250-270us band. A fully
instrumented raw-curve probe (5us steps, no stopping logic) showed
why: the response is smooth and consistent (~0.2-1.6°/step) right up
to and including the step immediately before the failure — no gradual
weakening to detect. Sweeping `WEAKENING_FRACTION` from 0.15 to 0.55
confirmed it: every value failed at the same ~250-260us point, because
there's no slope change in the data for any threshold to key off.
Below that pulse, this servo takes the commanded target as a cue to
spin the long way around instead of stalling — a discrete decision
boundary, not continuous physics, and fundamentally undetectable by
watching the slope of the pulses before it.

**The fix**, since the edge can't be predicted in advance: make
crossing it once, safely, part of the normal algorithm instead of an
uncaught crash. `naive_stall_sweep()`/`scan_until_weak()`
(`servo_daq.py`) now catch a `NotSettledError` from `move_to()` and
treat it exactly like reaching the edge — report the last point
actually reached normally, and stop, rather than taking a
confirmatory step past a limit that's provably real. Recovering also
corrects the position reference: `ServoDAQLink.recover_from_wrap()`
moves to a pulse whose real position is already trusted and measures
the actual drift rather than assuming a clean -360° (the magnitudes
above prove that assumption would be wrong), folding the exact
difference into a running `centideg_offset` applied transparently to
every `move_to()` reading for the rest of the session. A first
end-to-end test found a real gap in this: the immediate recovery point
itself can *also* be past the (not perfectly repeatable) edge and fail
the same way, so `recover_from_wrap()` now takes an ordered list of
fallback candidates (nearest first, then the scan's own known-safe
starting pulse) instead of a single point.

Verified on real hardware: 10/10 trials (`naive_stall_sweep` +
`find_edge`, 5x each) completed without raising, edges consistently
found at the real 265-325us limit, and reported-position drift held
under 4° across all 10 crossings (vs. spiraling past -12,000° 
unguarded). A full `study_range.py` smoke test then ran end-to-end
clean, including through a real edge-crossing on the low side. This is
a no-op for any servo that never triggers `NOT_SETTLED` (the offset
stays exactly 0, the new exception handlers never execute) — confirmed
by code inspection for `type1_unit1`/`type1_unit2`, not yet
re-verified against their actual hardware.

`type3_unit1` (MG90D) smoke-test results (stamp `20260818-191520`,
0.25h/644 trials, ~107/model): range -72.6° to 168.7° (~241° stroke);
same shape as every prior unit -- `linear2` mean 1.03°, table models
0.47-0.56°, diminishing returns past ~10-20 points. Smoke test only,
not a completed study unit yet.

### type3_unit1's full study, completed clean (2026-08-18)

Full 3h run (stamp `20260818-194150`), the first real run on the
NOT_SETTLED-fixed firmware end to end: 7766 independent trials, zero
`NOT_SETTLED` events, zero warnings, servo returned to center. Range
265-2075µs, -73.6° to 241.6° (~241.6° stroke).

| model | points | mean\|err\| | max\|err\| | rms |
|---|---|---|---|---|
| linear2 | 2 | 1.127° | 3.378° | 1.367° |
| table10 | 10 | 0.659° | 1.716° | 0.752° |
| table20 | 20 | 0.623° | 1.770° | 0.723° |
| table30 | 30 | 0.624° | 1.838° | 0.707° |
| table40 | 40 | 0.602° | 1.701° | 0.683° |
| table50 | 50 | 0.634° | 1.672° | 0.709° |

Same shape as every other unit -- `linear2` clearly worst, table
models cluster with diminishing returns past ~10-20 points -- but
every number runs noticeably higher than `type1_unit1`'s (table models
~0.60-0.66° here vs. ~0.29-0.39° there). Consistent with
`MATLAB/plotCalibration.m`'s slope panel: this unit's calibration
curve has real, measurable waviness (RMS deviation from a straight
line 0.854° vs. `type1_unit1`'s 0.595°), so even the table models have
a less-linear curve to work with. Second completed unit of the planned
8-unit study (`type1_unit1`, `type3_unit1`); 6 remain.

## MATLAB analysis toolkit (2026-08-18/19)

`MATLAB/` (never documented here until now, despite existing since
2026-08-18): a small standalone toolkit for visualizing the multi-servo
study's own output, built alongside the actual data collection rather
than after it.

`setup.m` forces itself and `../ServoDAQ/data` onto the path, parses
every `study_range.py` CSV into one `MotorTypeData` object per motor
type actually found on disk -- type/unit counts discovered from
filenames, never hardcoded, so it automatically picks up new units as
the study progresses. Most recent stamp wins per unit (a rerun
supersedes an earlier partial/smoke one). Every `*angle_deg` column
across every table is normalized once, here, so every plotting script
gets the identified min-pulse edge sitting at 0° for free.
`MATLAB/motorTypeNames.m` is a small lookup (mirroring
`ServoDAQ/MOTOR_TYPES.md`) from type number to physical servo model
name; `setup.m` resolves it onto each `MotorTypeData.TypeName`, and
every plot now refers to units by name ("Miuzei 25kg Servo Unit 2"),
never bare "type1 unit2" -- "Type `<N>`" only survives as a last-resort
fallback for a type with no lookup entry.

Four standalone plotting scripts (each calls `setup()` itself):
`plotHardStops.m` (naive/coarse/fine range-finding overlay, min/max
edges marked with leader-lined callout labels, generic first-point-
outlier and non-monotonic-tail cleanup for the real stall-recovery-
backoff/spin-past-the-edge artifacts found this session),
`plotCalibration.m` (up/down/average fine sweep with hysteresis
shading, a Savitzky-Golay-smoothed local-slope panel for spotting
bowing, a zoomed view of the hysteresis band), `plotAccuracy.m`
(per-trial error distribution plus mean/RMS/max summary statistics,
naive `linear2` vs. the N-point tables), and `plotErrorVsAngle.m`
(mean |error| by target-angle decile per model -- where each model is
worst, not just its aggregate). Every legend uses `eastoutside`, not
`best` -- `best` let a legend box sit directly over a real trace on the
error-vs-angle plot once real data made the axes range large enough
for that to happen. Within one type's figure, every unit's axes share
identical scale (a shared `syncAxes()` helper, synced per-row where
rows carry different measurements) so units are directly visually
comparable instead of each silently auto-scaling to its own range.

## Requirements & dependencies

Same as documented in the [README](README.md) — `ServoCalibrator_Companion`
now unconditionally needs **both** Universal-Motor-Interface
(`ServoCalibrationTable.h`) and Universal-Trajectory-Interface
(`TrapezoidalProfile`), plus RobTillaart's `AS5600` (public). No PCA9685
support in this firmware yet (the old wizard-era version had it — see
git history; `ServoCalibrationTable.h`'s math is transport-agnostic, so
adding it back would mean swapping the raw `Servo` calls for
`PCA9685Backend` ones, not touching the calibration/table logic at all).
