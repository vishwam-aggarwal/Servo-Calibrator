# Servo Calibrator

A browser-based tool that characterizes a hobby RC servo — finds its real
pulse range and builds a 20-point calibration lookup table, fully
automated, one button — then lets you drive and visualize trajectories
against that calibration live, using an AS5600 magnetic encoder as
ground truth instead of a protractor and guesswork.

**Purely for characterization.** This tool doesn't generate a driver
constructor, doesn't ask about horn installation, direction, or a
logical zero point — it always works in the servo's own physical frame,
`[0, maxAngle]`. Turning a measured range/table into an actual
`RCServoMotorDriver`/`PCA9685MotorDriver` constructor (direction, mounting
offset, logical framing) is an application concern, left to your own
[Universal-Motor-Interface](#the-calibration-table--universal-motor-interface)
code.

No install, no build step. It's one HTML file that talks to an Arduino
over [Web Serial](https://developer.mozilla.org/en-US/docs/Web/API/Web_Serial_API).

> **Status:** the firmware is a self-contained FSM (`ServoCalibrator_Companion.ino`)
> verified against real hardware, including a full calibrate → drive
> round-trip. The app (`ServoCalibrator.html`) was rewritten against
> that firmware and verified against the real page code — real captured
> hardware wire data fed straight into `SerialLink`, calibration
> parsing, and the charts — but **not yet exercised through an actual
> live `navigator.serial` session** (the browser's native port picker
> needs a human at the keyboard; that pass is still open). See
> [Known limitations](#known-limitations) for what's still rough. **The
> firmware depends on two of my other libraries that aren't public
> yet** — see [Dependencies](#requirements--dependencies) before you try
> to build it.

## Contents

- [Why this exists](#why-this-exists)
- [Quick start](#quick-start)
- [What it looks like in use](#what-it-looks-like-in-use)
- [How it works](#how-it-works)
- [The calibration table & Universal-Motor-Interface](#the-calibration-table--universal-motor-interface)
- [Export](#export)
- [Wiring](#wiring)
- [Requirements & dependencies](#requirements--dependencies)
- [Safety notes](#safety-notes)
- [Serial protocol reference](#serial-protocol-reference)
- [Known limitations](#known-limitations)
- [License](#license)

## Why this exists

Hobby servos have a real, measurable non-linear bow in their pulse↔angle
response — a plain 2-point linear formula (the usual assumption) leaves
several degrees of avoidable error on the table. Characterizing that
curve by hand (jogging pulses, reading a protractor, building a table
yourself) is slow and imprecise. This tool automates the whole thing: an
AS5600 magnetic encoder on the servo's output shaft gives ground-truth
angle feedback, and the firmware itself stall-scans the servo's real
mechanical limits and sweeps the full range to build a direction-averaged
lookup table — no hand-measurement, no guessing. Then, because the same
rig is already wired up, it doubles as a small live trajectory
visualizer — closer to a mini ODrive GUI than a calibration wizard — so
you can actually *see* what that calibration buys you before writing any
application code.

## Quick start

1. **Flash the firmware.** Open `ServoCalibrator_Companion/ServoCalibrator_Companion.ino`
   in the Arduino IDE (or `arduino-cli`) and upload it to your board.
   Servo signal is fixed at pin `A3` (change `SERVO_PIN` in the sketch if
   you need a different pin — unlike the firmware's predecessor, this
   isn't runtime-configurable, since there's no wizard step left to ask).
2. **Wire it up**: servo signal to `A3`, AS5600 on the board's I²C bus
   (`SDA`/`SCL`), servo power from an external supply sized for your
   servo (not the Arduino's own 5V pin on most boards — see
   [Wiring](#wiring)).
3. **Open the app.** `ServoCalibrator.html` needs to be served over
   `http://` — Web Serial does not reliably work when a page is opened
   directly as a `file://` URL. The easiest way:
   ```bash
   # from the folder containing ServoCalibrator.html
   python -m http.server 8000
   # or: npx serve
   ```
   then open `http://localhost:8000/ServoCalibrator.html` in **Chrome or
   Edge** (Web Serial isn't implemented in Firefox or Safari).
4. **Connect, then Calibrate.** Click *Connect…*, pick your serial port
   in the browser's device picker, then click **Calibrate** — one
   button, fully automated, usually done in under a minute. The command
   acknowledges immediately and the run streams progress asynchronously;
   the command/chart interface below unlocks once the app sees the
   table finish (there's no separate "done" reply — see
   [Serial protocol reference](#serial-protocol-reference)). Every large
   move the firmware makes during calibration is deliberately slow and
   incremental (small steps, a short pause between each) rather than one
   instant jump — found to be necessary against at least one real servo
   that behaved oddly when commanded a big pulse change in one shot (see
   `CLAUDE.md` for the story); it's expected, not a stall. This firmware
   doesn't remember a calibration across a reconnect — recalibrate fresh
   every session.

## What it looks like in use

- **Calibration**: click Calibrate. The firmware coarse-then-fine scans
  outward from center in both directions — each scan self-calibrates its
  own baseline step rate from its first few real steps, then watches
  every step after that for a weak, reversed, or oversized-jump response
  *relative to that rate* (no fixed threshold) — to find the real safe
  pulse range, then sweeps it twice (once each direction) to build a
  20-point direction-averaged table. Progress streams into the log live.
  Result: a summary (`350–2630µs · 214.01° stroke · 20-point table`) and
  an **Export table…** button.
- **Live trace**: command a single point-to-point move (a real,
  v<sub>max</sub>/a<sub>max</sub>-limited trapezoidal move to a target
  angle — no continuous square-wave/sine-wave generator; this firmware
  deliberately only ever streams one planned move at a time). Three
  auto-scaled rolling charts (position, velocity, error) show setpoint
  vs. AS5600-measured actual, live — velocity's "actual" trace comes
  straight from the firmware's own encoder-derivative measurement, not a
  client-side numeric differentiation.
- **See the table's improvement, interactively**: a toggle switches
  between the plain 2-point linear formula and the just-calibrated
  20-point table (at rest only — like every other command, `MODEL` is
  rejected while a move is actively running) — and the error chart's
  line is colored by which model was active, next to a running
  mean-error-per-model comparison, so the improvement is something you
  watch happen, not a number you take on faith.

## How it works

Two pieces:

- **`ServoCalibrator_Companion.ino`** — a self-contained finite state
  machine, one `switch`/`case` in `loop()`, fixed 50Hz tick, running
  both the calibration routine and trajectory execution itself,
  on-device. Unlike a blocking one-shot design, `CAL` acknowledges
  immediately and the calibration routine runs as part of the normal
  tick loop — the board never stops responding to serial while it runs
  (`PING`/`ABORT` both still work mid-calibration). Once calibrated,
  [`TrapezoidalProfile`](https://github.com/vishwam-aggarwal/Universal-Trajectory-Interface)
  is planned/evaluated on-device at that same fixed ~50Hz, alongside
  telemetry that streams unconditionally every tick regardless of
  state — the app never paces the motion, it only sends target/limit
  commands and plots what comes back.
- **`ServoCalibrator.html`** — the app. Connection, calibration
  triggering/export, and the three live charts all live here; the
  firmware doesn't know anything about SVG or JSON. Because the
  firmware's own calibration table isn't re-anchored to a 0° low
  endpoint (see [Serial protocol reference](#serial-protocol-reference)),
  the app does that framing itself, client-side, so the UI still
  presents the established physical `[0, maxAngleDeg]` convention.

## The calibration table & Universal-Motor-Interface

The table this tool builds is a plain array of `{pulseUs, angleCentideg}`
pairs (`CalPoint`, from [Universal-Motor-Interface](https://github.com/vishwam-aggarwal/Universal-Motor-Interface)'s
`ServoCalibrationTable.h`) — the same type `RCServoMotorDriver`/
`PCA9685MotorDriver` accept as an optional constructor argument, falling
back to the strict 2-point linear formula when omitted. Physical testing
across two servos (140 to 22,000+ samples, in the investigation that
motivated this whole tool) found a 20-point table cuts mean positioning
error 2–6× vs. the linear formula, for ~80 bytes of `PROGMEM`.

**This tool deliberately doesn't generate the constructor call itself.**
The old wizard-based version of this repo did (see git history) — this
version doesn't, because direction, logical zero, and mounting offset are
all *installation*-specific decisions this characterization-only tool has
no way to know. Export the table (see [below](#export)), paste its
points into a `PROGMEM` array in your own project, and pass it to
`RCServoMotorDriver`'s/`PCA9685MotorDriver`'s table-accepting constructor
overload alongside whatever direction/offset your actual installation
needs.

## Export

**Export** downloads the current calibration as a small JSON file:
`{maxAngleDeg, minPulseUs, maxPulseUs, points: [{pulseUs, angleCentideg}, …]}`
(already re-anchored to the app's own `[0, maxAngleDeg]` framing — see
[How it works](#how-it-works)). There's no matching **Import** — this
firmware has no command that accepts a pushed-in table, unlike its
predecessor. Every session starts uncalibrated; re-run Calibrate each
time you reconnect or reset the board. Export exists purely for keeping
a record of what a servo measured at, not for skipping a future physical
recalibration of that same servo.

## Wiring

- **Servo signal** → `A3` (fixed in firmware — see [Quick start](#quick-start)
  if you need a different pin).
- **AS5600** → the board's I²C bus (`SDA`/`SCL` — `A4`/`A5` on an
  Uno/Nano), with the magnet mounted on the servo's output shaft, centered
  over the chip.
- **Servo power** → an external supply sized for your servo's stall
  current, **not** the Arduino's own 5V/Vin pin on most small boards —
  clone/small-board regulators usually can't source enough current, and
  Vin specifically needs headroom above 5V to regulate down cleanly (it
  is not a direct 5V input). Common ground between the supply, the servo,
  and the Arduino either way.

## Requirements & dependencies

**Hardware:**

- An Arduino-compatible board with I²C and a free PWM-capable digital
  pin (`A3` by default).
- An AS5600 breakout, magnet mounted on the servo's output shaft.

**Software, to build the firmware:**

| Library | Used for | Status |
|---|---|---|
| [RobTillaart's `AS5600`](https://github.com/RobTillaart/AS5600) | Low-level AS5600 I²C register access | Public — `arduino-cli lib install "AS5600"` or via Library Manager |
| **Universal-Motor-Interface** (mine) | `ServoCalibrationTable.h`'s `CalPoint` type and angle↔pulse math — used **unconditionally**, called directly rather than through `RCServoMotorDriver` (which binds its table at construction, not at runtime) | **Not public yet** |
| **Universal-Trajectory-Interface** (mine) | `TrapezoidalProfile` — used **unconditionally** | **Not public yet** |

In short: **the firmware as committed here won't compile for anyone
without access to both Universal-Motor-Interface and
Universal-Trajectory-Interface** — neither is optional, regardless of
which trajectory mode you use.

**Browser:** Chrome or Edge (desktop) — Web Serial isn't available in
Firefox or Safari.

## Safety notes

- **Calibrate with a bare horn.** No linkage, mechanism, gearbox, or load
  attached — a loaded mechanism can make a stall look like normal
  resistance and vice versa. Recalibrate (re-run `CAL`) if you change the
  mechanical load after the fact.
- `CAL` deliberately drives the servo into its mechanical end stops to
  find them (with active recovery if the servo spins past a limit
  instead of stalling, rather than just timing out — see `CLAUDE.md`).
  It stops advancing within a small margin of first detecting no motion,
  so it only grinds against a stop briefly — but it is intentionally
  doing that, twice (once per direction), by design.
- A hard pulse-width safety ceiling (80–3100µs, fixed in firmware) exists
  as a fail-safe in case stall detection doesn't trigger for some reason
  (e.g. an encoder fault) — a scan hitting it ends the run without a
  usable table, rather than silently accepting a bad range. If that
  happens to you, it likely means your servo's real range is wider than
  this default too — widen `ABS_FLOOR_US`/`ABS_CEIL_US` in the firmware
  and reflash.

## Serial protocol reference

115200 baud, one command per line, newline-terminated ASCII. Genuinely
different in shape from the predecessor firmware this repo used to have
at this path — see `CLAUDE.md`'s 2026-08-21 entry for the full design
story if you're diffing against an older version of this doc.

| Command | Response |
|---|---|
| `PING` | `OK PING` |
| `CAL` | `OK CAL` — acknowledges immediately; the run itself streams asynchronously (see below), no final reply |
| `ABORT` | `OK ABORT` \| `ERR ALREADY_IDLE` — cancels a calibration *or* a move; exempt from the busy-gate below |
| `ACCEL <aMaxDegS2>` | `OK ACCEL` |
| `VEL <vMaxDegS>` | `OK VEL` |
| `POS <targetDeg>` | `OK POS` — **not range-checked server-side**; the app clamps client-side instead |
| `GO` | `OK GO` \| `ERR NOT_CALIBRATED` — starts a move to the most recently set `POS`, at the most recent `ACCEL`/`VEL` |
| `MODEL LINEAR\|TABLE` | `OK MODEL` \| `ERR NOT_CALIBRATED` |

While a calibration or move is running, every command except `ABORT` and
`PING` is rejected with `ERR BUSY`.

**Calibration streams three line shapes while it runs**, none of them a
reply to any pending command:

```
<elapsedMs> PROGRESS <stateName> <tick> <lastSentUs> <rawPos>
<elapsedMs> TABLE <index> <pulseUs> <angleCentideg> <rawPos>
<elapsedMs> ERR <code> <rawPos>
```

`TABLE` fires twice per index (20 points, down-pass then up-pass —
the up-pass value is the final, direction-averaged one); the app detects
completion by recognizing index 19 arriving *ascending* (immediately
after index 18), since there's no dedicated "done" message. `angleCentideg`
is a **raw** encoder reading relative to wherever the board happened to
boot — not re-anchored so index 0 reads exactly 0° the way the old
firmware's table was; the app reframes it client-side (see
[How it works](#how-it-works)). `ERR` here is an asynchronous internal
failure (e.g. a settle timing out), not a synchronous command rejection.

Once calibrated, telemetry streams every tick (~50Hz), unconditionally,
regardless of state:

```
TELEM <ms> <targetCentideg> <targetVelCentidegPerSec> <actualCentideg> <actualVelCentidegPerSec>
```

No mode/model field (unlike the old `T,...` line) — the app tracks which
model is live itself, since only it ever changes `MODEL`. A completed
move is signaled by exactly one `PROGRESS TRAJ_WAIT ...` line, printed
the instant the shaft settles. Lines starting with `#` are informational
(boot banner) — not part of the command/response or telemetry protocol.

## Known limitations

- **The browser app hasn't been exercised against a real live serial
  connection yet** — verified by feeding real captured hardware wire
  data directly into the actual page code (`claude-in-chrome`, not a
  reimplementation), but the `navigator.serial` port-picker itself needs
  a human at the keyboard to click through, and that pass hasn't
  happened yet.
- **No calibration persists across a reconnect.** This firmware has no
  `GETTABLE`-equivalent — recalibrate every session, even if the board
  itself was never reset.
- **No table import**, only export — there's no command this firmware
  accepts a pushed-in table through. Export is for record-keeping, not
  for skipping a future recalibration.
- Tested on Chrome/Edge over `http://localhost`; opening the app
  directly via `file://` has not been confirmed to work reliably with
  Web Serial.
- No PCA9685 support in this firmware (the old wizard-era version had
  it; this characterization-only rewrite doesn't yet — `ServoCalibrationTable.h`'s
  math is transport-agnostic, so adding it back is mostly a matter of
  swapping the raw `Servo` calls for `PCA9685Backend` ones).
- Servo pin is fixed at compile time (`A3`) — no runtime pin
  configuration, unlike the old wizard-era firmware.
- No installer/packaged build — meant to stay a single portable HTML
  file, served locally.

Issues and pull requests welcome.

## License

MIT — see [`LICENSE`](LICENSE).
