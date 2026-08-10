# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working
with code in this repository.

## What this is

Two companion-firmware + Web Serial browser-app pairs for working with a
hobby RC servo, sharing the same hardware pattern (Arduino Nano, AS5600
magnetic encoder on the servo's output shaft as ground truth, one
self-contained HTML file per app talking [Web Serial](https://developer.mozilla.org/en-US/docs/Web/API/Web_Serial_API),
no build step):

1. **Servo Calibrator** (`ServoCalibrator.html` + `ServoCalibrator_Companion/`)
   — finds a servo's real mechanical pulse range/travel (stall-detected,
   not guessed) and walks through a guided installation/calibration
   wizard (logical angle range → horn install → direction test → fine
   trim → test drive), ending in a ready-to-paste `RCServoMotorDriver`/
   `PCA9685MotorDriver` constructor.
2. **Trajectory Demo** (`TrajectoryDemo.html` + `TrajectoryDemo_Companion/`)
   — a small, single-joint, ODrive-GUI-style live trajectory
   visualizer/commander: set v<sub>max</sub>/a<sub>max</sub>, command a
   step, a continuous post-trajectory square wave, or a continuous
   trajectory-free sine wave, and watch setpoint vs. AS5600-measured
   actual position/velocity/error live in rolling auto-scaled charts.
   Includes a live toggle between the 2-point linear pulse↔angle formula
   and a 20-point calibration lookup table, so the table's accuracy
   improvement is visible interactively — color-coded error trace plus a
   running mean-error comparison — instead of only recalled from prior
   test data.

Both grew out of `Servo_Auto_Calibrator` (a hand-built, single-servo
characterization project — endpoint/hysteresis/repeatability, then a
whole investigation into whether a calibration lookup table is worth
adding to `RCServoMotorDriver` — see that project's own history for the
"why" behind the numbers these tools now measure/use automatically) and
depend on two sibling libraries, **Universal-Motor-Interface** and
**Universal-Trajectory-Interface** (both mine, both currently private —
see the README's dependency table for exactly what that means for
building this repo's firmware).

## Final goal: merge into one interface

**These are not meant to stay two separate tools.** The end state is a
single web app that does both calibration (what Servo Calibrator does
today) and trajectory planning/visualization (what Trajectory Demo does
today) — connect once, calibrate a servo, then immediately drive and
visualize trajectories against that same calibration, without
disconnecting, reconnecting, or re-entering any setup. Not started yet as
of this commit — the two tools were deliberately built and validated
*separately* first, so the merge starts from two known-working pieces
instead of building both halves of a combined tool at once, untested.

Notes for whoever does the merge (kept here so they don't have to
re-derive this from scratch):

- **Firmware angle→pulse math should converge on `TrajectoryDemo_Companion`'s
  approach, not `ServoCalibrator_Companion`'s.** Both ultimately need the
  same real math (`ServoCalibrationTable.h`'s `computeServoPulseUs()`/
  `validateCalTable()`, from Universal-Motor-Interface), but
  `TrajectoryDemo_Companion` calls those functions directly against a raw
  `Servo` object instead of going through `RCServoMotorDriver`,
  specifically because it needs to swap the calibration model live at
  runtime (`MODEL LINEAR`/`MODEL TABLE`) and `RCServoMotorDriver` binds
  its table at construction with no runtime setter. A merged firmware
  wanting both wizard-driven calibration moves *and* live trajectory
  commands to share one angle↔pulse code path should follow that same
  pattern.
- **The two protocols are different in kind, not just content, and both
  patterns need to coexist in a merge.** `ServoCalibrator_Companion` is
  host-paced request/response (`CONFIG`/`MOVE`/`READ`/`ZERO`/`STOP` — the
  PC drives every step, one at a time, blocking `readLine()`).
  `TrajectoryDemo_Companion` runs autonomously and streams telemetry
  continuously at a fixed ~50Hz rate regardless of whether a command just
  arrived, using a *non-blocking* line reader
  (`readSerialNonBlocking()`) so it can keep streaming between commands.
  A merged firmware likely wants the non-blocking pattern throughout
  (request/response commands processed between telemetry ticks, telemetry
  only actually streaming while a trajectory/wizard-live-view is active)
  rather than trying to bolt autonomous streaming onto the blocking
  reader `ServoCalibrator_Companion` uses today.
- **The two apps' UI shells are already visually consistent on purpose**
  — same CSS custom-property token system, same card/pill/button/tab
  component classes, same SVG line-chart helper pattern (`svgEl()`,
  `niceStep()`, `drawRollingChart()`/`drawLineChart()`) — so the merge is
  mostly an information-architecture problem (how a calibration wizard
  and a live trajectory view coexist in one flow/page — tabs? a
  post-calibration "now drive it" step?) rather than a visual-design
  problem. Whichever shell wins, reuse its token set rather than
  reconciling two independently-evolved palettes.
- **Trajectory Demo's rolling-chart/telemetry-streaming JS
  (`SerialLink` with a dedicated `onTelemetry` callback separate from
  the command/response queue, plus the `requestAnimationFrame`-decoupled
  render loop) is the piece Servo Calibrator has no equivalent of and
  would need if, post-merge, the calibration wizard ever wants a live
  chart too** (e.g. watching the encoder in real time during the fine-trim
  step, instead of only reading a value after each discrete move).

## Servo Calibrator

Full protocol reference, wiring, safety notes, and known limitations are
in the [README](README.md) (public-facing) — this section is
implementation notes for working on the code itself.

- **`ServoCalibrator_Companion.ino`** — dumb, host-paced command
  executor. Supports either a raw RC servo on a digital pin or a PCA9685
  channel (via Universal-Motor-Interface's `PCA9685Backend`), exactly one
  at a time, chosen at runtime via `CONFIG`. No calibration layer in the
  firmware at all — that's the point, this firmware is for *discovering*
  the range/direction/zero that feed a calibrated driver, not consuming
  one. Encoder is Universal-Encoder-Interface's `AS5600EncoderDriver` in
  continuous mode. `READ`'s third response field
  (`AS5600EncoderDriver::magnetStatusCode()`) reports *why* a reading is
  bad (no magnet / too weak / too strong / chip not responding), not just
  a valid/invalid bit — matters once the servo's installed in its real
  application, where the magnet/sensor air gap may no longer match the
  bench setup.
- **`ServoCalibrator.html`** — all real logic (stall detection,
  settle-until-stable polling, the wizard's direction/offset math, live
  charts, file save/load) lives here; the firmware doesn't know any of it
  exists. Design decisions worth knowing before changing the flow:
  range-finding and installation are deliberately separable/saveable
  steps (range is a property of the bare servo; install position/
  direction/trim are properties of one specific mounting, possibly done
  later by someone else with just the saved JSON); direction is a live
  y/n hardware test in the wizard, not a pre-filled toggle, because the
  point is to *feel* like the source `RCServoCalibration.ino` wizard's
  guided experience; no backlash compensation in the generated
  constructor (`RCServoMotorDriver`/`PCA9685MotorDriver` have no hook for
  it — see the pivot note in `Servo_Auto_Calibrator`'s own history for
  why that was judged not worth adding in general).
- Real bugs found via actual browser+hardware testing (not just code
  review) worth knowing about if touching this flow again: a stray
  `STOP` at the end of range-finding used to leave the wizard's first
  move silently failing (`ERR NOT_CONFIGURED`) with no visible cause
  until the raw serial log was read — fixed with an explicit
  `configApplied` staleness flag + an in-wizard "Apply configuration
  now" banner; a live readout bug where flipping direction moved the
  servo correctly but displayed the *raw* (un-transformed) AS5600 angle,
  looking wrong even though positioning was fine the whole time — fixed
  with `rawAngleDegToLogicalDeg()`, the algebraic inverse of the move
  math, verified by round-tripping known values; a firmware-RAM-reset
  gotcha where the AS5600's software zero (set via `ZERO`) doesn't
  survive a board reset/reflash between range-finding and wizard use,
  silently offsetting every subsequent readout by a large constant —
  fixed with self-detecting `ensureZeroAnchored()` drift correction
  rather than assuming the board's state survives.

## Trajectory Demo

- **`TrajectoryDemo_Companion.ino`** — runs the trajectory itself
  on-device (Universal-Trajectory-Interface's `TrapezoidalProfile`,
  planned/evaluated at a fixed ~50Hz alongside continuous telemetry
  streaming) rather than being paced by the host. State machine:
  `IDLE`/`MOVE`/`SQUARE`/`SINE`, all funneling through one
  `currentSetpointDeg()` that's the single authoritative "where should we
  be right now" for whichever mode is active — every mode transition
  (`GO`/`SQUARE`/`SINE`/`STOP`, and the live `MODEL` switch) replans from
  that live value, never from a stale target, so nothing jumps when
  switching. `SQUARE` is "post-trajectory" (every edge is still a real
  `TrapezoidalProfile` move, replanned smoothly from wherever the setpoint
  currently is if a transition hasn't finished when the next one is due
  — a period too short for the chosen v/a legitimately produces a
  triangle-ish oscillation that never reaches its extremes, which is
  correct behavior, not a bug). `SINE` is analytic and deliberately
  bypasses the trajectory profile entirely ("no traj" — no v/a clamp,
  amplitude×2π×freq *is* the effective peak speed), easing in via one
  ordinary trapezoidal move to center first so starting it never jumps.
  `MODEL LINEAR`/`MODEL TABLE` flips which calibration model computes the
  pulse width, live, without touching the setpoint/mode state at all —
  see [Final goal](#final-goal-merge-into-one-interface) above for why
  this firmware calls `ServoCalibrationTable.h` directly instead of
  going through `RCServoMotorDriver`.
- **`TrajectoryDemo.html`** — side-by-side layout (a fixed-height flex
  page, controls in a ~360px left column, charts filling the right column
  — deliberately not a top-to-bottom scroll, so settings and the live
  charts are visible together) with a Step/Square/Sine tab strip, a
  Linear/Table model toggle, and three auto-scaled rolling charts
  (position, velocity, error — the error chart's line is colored per
  point by which model was active, plus a live mean-error-per-model
  readout, so a live model switch is visible both as a chart color change
  and as an actual number). Rendering is decoupled from the ~50Hz serial
  arrival rate onto its own `requestAnimationFrame` loop — telemetry
  parsing just buffers cheaply, a separate loop redraws from whatever's
  currently buffered.
- A real firmware bug was caught and fixed via testing, not just review:
  the boot AS5600 zero-reference was briefly hardcoded to `0.0f` instead
  of using `pollUntilSettled()`'s actual return value — silently correct
  only when the servo happened to already be near physical zero at boot
  (true by coincidence in one early smoke test), silently wrong whenever
  it wasn't (true once a *different* prior sketch left the servo at the
  opposite end of its range, exposing a real ~151° offset on first boot).
  Fixed by capturing the actual settled value instead of assuming it.

## Requirements & dependencies

Same as documented in the [README](README.md) for `ServoCalibrator_Companion`,
plus, for `TrajectoryDemo_Companion` specifically: **Universal-Trajectory-Interface**
(mine, private) for `TrapezoidalProfile`, and Universal-Motor-Interface's
`ServoCalibrationTable.h` (used directly, not via `RCServoMotorDriver` —
see [Final goal](#final-goal-merge-into-one-interface) above for why).
Both companion firmwares also need RobTillaart's `AS5600` library
(public). Browser requirement is the same for both apps: Chrome or Edge,
served over `http://` (Web Serial does not reliably work opened directly
as a `file://` URL — confirmed for `ServoCalibrator.html`; unconfirmed
either way for `TrajectoryDemo.html`, untested via `file://`).
