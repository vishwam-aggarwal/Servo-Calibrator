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

## Requirements & dependencies

Same as documented in the [README](README.md) — `ServoCalibrator_Companion`
now unconditionally needs **both** Universal-Motor-Interface
(`ServoCalibrationTable.h`) and Universal-Trajectory-Interface
(`TrapezoidalProfile`), plus RobTillaart's `AS5600` (public). No PCA9685
support in this firmware yet (the old wizard-era version had it — see
git history; `ServoCalibrationTable.h`'s math is transport-agnostic, so
adding it back would mean swapping the raw `Servo` calls for
`PCA9685Backend` ones, not touching the calibration/table logic at all).
