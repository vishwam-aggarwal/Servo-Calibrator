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

## Requirements & dependencies

Same as documented in the [README](README.md) — `ServoCalibrator_Companion`
now unconditionally needs **both** Universal-Motor-Interface
(`ServoCalibrationTable.h`) and Universal-Trajectory-Interface
(`TrapezoidalProfile`), plus RobTillaart's `AS5600` (public). No PCA9685
support in this firmware yet (the old wizard-era version had it — see
git history; `ServoCalibrationTable.h`'s math is transport-agnostic, so
adding it back would mean swapping the raw `Servo` calls for
`PCA9685Backend` ones, not touching the calibration/table logic at all).
