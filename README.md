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

> **Status:** built and tested against real hardware (an AS5600 + a
> generic analog servo on an Arduino Nano clone), including a full
> calibrate → drive → export → import → drive round-trip. See
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
- [Import / export](#import--export)
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
   in the browser's device picker, then click **Calibrate** — one button,
   a few minutes, fully automated. Once it finishes, the command/chart
   interface below unlocks. Every large move the firmware makes during
   `CALIBRATE`/`IMPORT` is deliberately slow and incremental (small steps,
   a short pause between each) rather than one instant jump — found to
   be necessary against at least one real servo that behaved oddly when
   commanded a big pulse change in one shot (see `CLAUDE.md` for the
   story); it's expected, not a stall.

## What it looks like in use

- **Calibration**: click Calibrate. The firmware stall-scans outward from
  center in both directions (a sliding-window net-delta check, not a
  single-step one — a mechanism visibly slows before it actually stops,
  and a naive check false-triggers on that creep) to find the real safe
  pulse range, then sweeps that range twice (once each direction) to
  build a 20-point direction-averaged table. Progress streams into the
  log live. Result: a summary (`350–2630µs · 214.01° stroke · 20-point
  table`) and an **Export table…** button.
- **Live trace**: command a single step, a continuous *post-trajectory*
  square wave (every edge still a real, v<sub>max</sub>/a<sub>max</sub>-limited
  trapezoidal move — just auto-repeating), or a continuous sine wave that
  deliberately skips trajectory shaping entirely. Three auto-scaled
  rolling charts (position, velocity, error) show setpoint vs.
  AS5600-measured actual, live.
- **See the table's improvement, interactively**: a live toggle switches
  between the plain 2-point linear formula and the just-calibrated
  20-point table — mid-move, without touching whatever's currently
  running — and the error chart's line is colored by which model was
  active, next to a running mean-error-per-model comparison, so the
  improvement is something you watch happen, not a number you take on
  faith.

## How it works

Two pieces:

- **`ServoCalibrator_Companion.ino`** — runs both the calibration
  routine and live trajectory execution itself, on-device.
  `CALIBRATE` is a deliberately blocking, one-shot routine (like a human
  would expect a physical range-finding sweep to occupy the board for
  its own duration); once calibrated, [`TrapezoidalProfile`](https://github.com/vishwam-aggarwal/Universal-Trajectory-Interface)
  is planned/evaluated on-device at a fixed ~50Hz alongside continuous
  telemetry streaming, independent of the host — the app never paces
  the motion, it only sends target/limit commands and plots what comes
  back.
- **`ServoCalibrator.html`** — the app. Connection, calibration
  triggering/import/export, and the three live charts all live here;
  the firmware doesn't know anything about SVG or JSON.

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
no way to know. Export the table (see [below](#import--export)), paste
its points into a `PROGMEM` array in your own project, and pass it to
`RCServoMotorDriver`'s/`PCA9685MotorDriver`'s table-accepting constructor
overload alongside whatever direction/offset your actual installation
needs.

## Import / export

- **Export** downloads the current calibration (whether freshly
  calibrated or previously imported) as a small JSON file:
  `{maxAngleDeg, minPulseUs, maxPulseUs, points: [{pulseUs, angleCentideg}, …]}`.
- **Import** loads a previously-exported file and pushes it to the board
  over serial (`IMPORT` — same fields, same order) — skips the full
  physical stall-scan/sweep, but still does a physical re-anchor (up to
  3 moves) to re-zero the AS5600's live reference for the current
  session/mounting, since an imported table carries the pulse curve but
  not a live sensor zero reference. Those moves are the same slow,
  incremental kind `CALIBRATE` uses (see [Quick start](#quick-start)) —
  if the servo's current position is far from the table's `minPulseUs`,
  the first one alone can take 20+ seconds; the app's own `IMPORT`
  timeout is generous (90s) to match. Reconnecting to a board that's
  still calibrated (never reset since) also auto-recovers that state on
  connect, without needing to re-import anything.

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
  resistance and vice versa. Recalibrate (re-run `CALIBRATE`) if you
  change the mechanical load after the fact.
- `CALIBRATE` deliberately drives the servo into its mechanical end stops
  to find them. It stops advancing within a small margin of first
  detecting no motion, so it only grinds against a stop briefly — but it
  is intentionally doing that, twice (once per direction), by design.
- A hard pulse-width safety ceiling (80–3100µs, fixed in firmware —
  widened once already from a narrower default after a real servo's real
  range fell outside it) exists as a fail-safe in case stall detection
  doesn't trigger for some reason (e.g. an encoder fault) —
  `CALIBRATE` aborts with a clear error instead of silently accepting a
  bad range if either scan hits it. If that happens to you, it likely
  means your servo's real range is wider than this default too — widen
  `ABS_FLOOR_US`/`ABS_CEIL_US` in the firmware and reflash.

## Serial protocol reference

115200 baud, one command per line, newline-terminated ASCII:

| Command | Response |
|---|---|
| `PING` | `OK PONG` |
| `CALIBRATE` | `CALRESULT <maxAngleDeg> <minPulseUs> <maxPulseUs> <pulse0> <cdeg0> … <pulse19> <cdeg19>` \| `ERR CAL_FAILED <reason>` |
| `GETTABLE` | same shape as `CALRESULT` \| `ERR NOT_CALIBRATED` |
| `IMPORT <maxAngleDeg> <minPulseUs> <maxPulseUs> <pulse0> <cdeg0> … <pulse19> <cdeg19>` | `OK` \| `ERR <msg>` |
| `GO <targetDeg> <vMaxDegS> <aMaxDegS2>` | `OK` \| `ERR <msg>` |
| `SQUARE <lowDeg> <highDeg> <periodS> <vMaxDegS> <aMaxDegS2>` | `OK` \| `ERR <msg>` |
| `SINE <centerDeg> <amplitudeDeg> <freqHz>` | `OK` \| `ERR <msg>` |
| `STOP` | `OK` |
| `MODEL LINEAR\|TABLE` | `OK` \| `ERR <msg>` |

`GO`/`SQUARE`/`SINE`/`MODEL` all return `ERR NOT_CALIBRATED` until a
`CALIBRATE` or `IMPORT` has succeeded. Once calibrated, telemetry streams
continuously at ~50Hz regardless of whether a move is active:

```
T,<t_ms>,<setpoint_deg>,<setpoint_vel_degs>,<actual_deg>,<mode>,<model>
```

`<mode>` is `IDLE`/`MOVE`/`SQUARE`/`SINE`, `<model>` is `LINEAR`/`TABLE` —
both sent explicitly (not left for the host to infer) so a live `MODEL`
switch, or a sine's velocity legitimately crossing zero without actually
stopping, both show up unambiguously in the stream. Lines starting with
`#` are informational (boot banner, `CALIBRATE` progress) — not part of
the command/response or telemetry protocol.

## Known limitations

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
