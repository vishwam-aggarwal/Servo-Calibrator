# Servo Calibrator

A browser-based tool that finds a hobby servo's real pulse range and helps
you install/calibrate it — using an AS5600 magnetic encoder as ground
truth, instead of a protractor and guesswork.

Two things it does, independently:

1. **Range finder** — walks the servo out to its real mechanical limits
   (stall-detected, not guessed) and reports the safe min/max pulse width
   and total travel. Save the result as a file, or hand the numbers to
   any servo library that wants them.
2. **Installation & calibration wizard** — a guided, step-by-step flow
   (logical angle range → horn install → direction test → fine trim →
   test drive → generated code) for mounting a servo in an application and
   getting a correct, direction-aware calibration — all driven live by the
   encoder instead of eyeballing it.

No install, no build step. It's one HTML file that talks to an Arduino
over [Web Serial](https://developer.mozilla.org/en-US/docs/Web/API/Web_Serial_API).

> **Status:** built and tested against real hardware (an AS5600 + a
> generic analog servo on an Arduino Nano clone) — see
> [Known limitations](#known-limitations) for what's still rough around
> the edges. **The firmware currently depends on two of my other
> libraries that aren't public yet** — see
> [Dependencies](#requirements--dependencies) before you try to build it.

## Contents

- [Why this exists](#why-this-exists)
- [Quick start](#quick-start)
- [What it looks like in use](#what-it-looks-like-in-use)
- [How it works](#how-it-works)
- [The generated constructor & Universal-Motor-Interface](#the-generated-constructor--universal-motor-interface)
- [Wiring](#wiring)
- [Requirements & dependencies](#requirements--dependencies)
- [Safety notes](#safety-notes)
- [Serial protocol reference](#serial-protocol-reference)
- [Known limitations](#known-limitations)
- [License](#license)

## Why this exists

Calibrating a hobby servo — finding its real pulse range, figuring out
which way is "positive," and lining up a mechanical zero point — is
normally a manual, iterative process: command a pulse width, look at the
horn, guess again. It works, but it's imprecise and slow, and there's no
record of what you actually measured afterward.

This tool replaces the eyeballing with a magnetic encoder (an
[AS5600](https://ams-osram.com/products/sensor-solutions/position-sensors/ams-as5600-position-sensor)
on the servo's output shaft) read back live over serial, so every step —
finding the mechanical limits, checking which way is positive, fine-trimming
alignment — is a real measurement instead of a guess. And because it's a
single web page, there's nothing to install: flash one small sketch once,
open one HTML file, done.

## Quick start

1. **Flash the firmware.** Open `ServoCalibrator_Companion/ServoCalibrator_Companion.ino`
   in the Arduino IDE (or `arduino-cli`) and upload it to your board. You
   only need to do this once — pin numbers, motor type, etc. are all
   configured later, live, from the app.
2. **Wire it up**: servo signal to any free digital pin, AS5600 on the
   board's I²C bus (`SDA`/`SCL`), servo power from an external supply
   sized for your servo (not the Arduino's own 5V pin on most boards —
   see [Wiring](#wiring)).
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
4. **Connect, configure, and go.** Click *Connect to board…*, pick your
   serial port in the browser's device picker, tell it which pin/channel
   your servo is on, and either run the range finder or the full wizard.

## What it looks like in use

- **Range finder**: enter a center pulse and some safety bounds, hit
  start. It walks the pulse width outward in both directions, watching
  the encoder, and stops each direction the moment the shaft actually
  stops moving — not at a pre-guessed limit. You get a live chart of the
  full pulse↔angle curve, the found min/max pulse width and total travel,
  and (optionally) a measured backlash/hysteresis figure. Save the result
  to a file if you're characterizing a servo before it's mounted in
  anything.
- **Installation wizard**: once you have a range result (fresh or
  reloaded from a save), walk through naming your logical angle
  convention, moving to an install position, testing which way is
  "positive" for your mechanism, fine-trimming alignment against a
  physical reference (witness marks, a pin-drop hole, whatever you're
  using), and test-driving a few angles — ending in a ready-to-paste
  constructor.

## How it works

Two pieces:

- **`ServoCalibrator_Companion.ino`** — a small, generic Arduino sketch.
  It runs *no* calibration logic itself; it's a dumb command executor
  that understands a handful of serial commands (`CONFIG`, `MOVE`,
  `READ`, `ZERO`, `STOP` — see the [protocol reference](#serial-protocol-reference)).
  Flash it once; every actual decision (which pin, what thresholds, when
  to stop) comes from the app at runtime, so nothing ever needs
  reflashing per project.
- **`ServoCalibrator.html`** — the app. All the real logic — stall
  detection, settle-until-stable polling, the wizard's direction/offset
  math, live charts, file save/load — runs here, in your browser, talking
  to the board over Web Serial. Nothing is sent anywhere except over the
  USB serial connection to your Arduino.

## The generated constructor & Universal-Motor-Interface

The wizard's last step generates a ready-to-paste C++ constructor, e.g.:

```cpp
RCServoMotorDriver myServo(
    17,
    3.518952f,   // maxAngleRad
    0.000000f,   // logicalZeroShiftRad
    404,
    2602,
    0.000000f,   // calibrationOffsetRad
    1.0f    // direction
);
```

This is written for **Universal-Motor-Interface**, a small library
(mine, separately) that gives you a single consistent `IMotorDriver` API
across different motor backends — an RC servo on a pin, a channel on a
PCA9685 board, and others — so your own control code doesn't care which
one it's actually talking to.

> **Universal-Motor-Interface isn't public yet — it's coming soon.**
> Watch [my GitHub profile](https://github.com/vishwam-aggarwal) for it.
> The constructor this tool generates is written against its exact API
> (verified against the driver's real source, not guessed), ready to drop
> in the moment it's released.

**You don't need to adopt it to get value from what this tool measures.**
The range finder, the save/load workflow, and the wizard's actual
measurements (real min/max pulse, real travel, direction, a
physically-verified zero offset) are all useful on their own — feed them
into whatever servo code you're already writing, by hand, with or without
this specific library. The generated constructor is a convenience for
later, not the point of the tool.

That said — this specific repo's *firmware* separately depends on another
one of my libraries (Universal-Encoder-Interface) to build at all, not
just to use the generated constructor. See
[Dependencies](#requirements--dependencies) for exactly what's needed and
what's still unreleased.

## Wiring

- **Servo signal** → any free digital pin (avoid pins used for Serial;
  double-check your board's pinout if you're also driving other PWM
  peripherals from the same timer).
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

- An Arduino-compatible board with I²C and at least one free PWM-capable
  digital pin.
- An AS5600 breakout, magnet mounted on the servo's output shaft.

**Software, to build the firmware — in order of how likely you already
have them:**

| Library | Used for | Status |
|---|---|---|
| [RobTillaart's `AS5600`](https://github.com/RobTillaart/AS5600) | Low-level AS5600 I²C register access | Public — `arduino-cli lib install "AS5600"` or via Library Manager |
| [Adafruit PWM Servo Driver Library](https://github.com/adafruit/Adafruit-PWM-Servo-Driver-Library) | PCA9685 support only | Public |
| **Universal-Encoder-Interface** (mine) | Wraps the AS5600 library behind a generic encoder interface — used **unconditionally**, every build needs it | **Not public yet** |
| **Universal-Motor-Interface** (mine) | Its `PCA9685Backend` for raw PCA9685 pulse writes — only needed for the PCA9685 config path, not plain RC-servo use | **Not public yet** |

In short: **the firmware as committed here won't compile for anyone
without access to Universal-Encoder-Interface**, regardless of whether
you're using a plain RC servo or a PCA9685 — that dependency isn't
optional. Universal-Motor-Interface is only needed if you're
configuring a PCA9685 channel; a plain-pin RC servo setup doesn't touch
it at build time at all (only the *generated constructor* references it,
as C++ source text you'd paste into your own project later — see
[above](#the-generated-constructor--universal-motor-interface)).

**Browser:** Chrome or Edge (desktop) for the app — Web Serial isn't
available in Firefox or Safari.

## Safety notes

- **Range-find with a bare horn.** No linkage, mechanism, gearbox, or
  load attached — same reason any manual range-finding process tells you
  this: a loaded mechanism can make a stall look like normal resistance
  and vice versa. Do the installation wizard's steps *after* the servo is
  actually mounted; do the range finder *before*.
- The range finder deliberately drives the servo into its mechanical end
  stops to find them. It stops advancing within a small margin of first
  detecting no motion, so it only grinds against a stop briefly — but it
  is intentionally doing that, once, by design.
- A hard pulse-width safety ceiling (configurable) exists as a fail-safe
  in case stall detection doesn't trigger for some reason (e.g. an
  encoder fault) — if you ever see a "hit hard safety limit" message,
  stop and check the servo/wiring before rerunning.

## Serial protocol reference

115200 baud, one command per line, newline-terminated ASCII:

| Command | Response |
|---|---|
| `PING` | `OK PONG` |
| `CONFIG SERVO <pin> <minUs> <maxUs>` | `OK` \| `ERR <msg>` |
| `CONFIG PCA9685 <addrHex> <channel> <freqHz>` | `OK` \| `ERR <msg>` |
| `CONFIG ENCODER AS5600` | `OK` \| `ERR <msg>` |
| `MOVE <us>` | `OK` \| `ERR NOT_CONFIGURED` |
| `READ` | `DATA <angle_deg> <valid 0\|1> <status>` |
| `ZERO <angle_deg>` | `OK` \| `ERR NOT_CONFIGURED` |
| `STOP` | `OK` |

`<status>` (from the AS5600's real hardware status bits, not just a
generic valid/invalid flag) is one of `OK`, `NO_MAGNET`, `TOO_WEAK`,
`TOO_STRONG`, `NOT_RESPONDING` — so a bad reading tells you *why*, useful
if the encoder's magnet-to-sensor air gap ends up different once the
servo is actually mounted in its final application than it was on the
bench. Lines starting with `#` are informational/boot-banner only, not
part of the command/response protocol.

## Known limitations

- Tested on Chrome/Edge over `http://localhost`; opening the app directly
  via `file://` has not been confirmed to work reliably with Web Serial.
- The firmware's motor configuration and the encoder's software zero
  offset both live in the Arduino's RAM and are lost on any board reset
  (power cycle, reconnect, reflash). The app detects and recovers from
  both cases automatically where it can, but if something behaves
  unexpectedly after a board reset mid-session, reconnecting and
  reapplying configuration is the first thing to try.
- PCA9685 support is implemented and configuration-tested, but hasn't
  had the same depth of real-hardware exercise as the plain RC-servo
  path.
- No installer/packaged build — it's meant to stay a single portable HTML
  file, served locally.

Issues and pull requests welcome.

## License

MIT — see [`LICENSE`](LICENSE).
