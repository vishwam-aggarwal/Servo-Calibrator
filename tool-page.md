---
title: "Servo Calibrator"
description: "A one-button browser tool that finds a hobby servo's real pulse range and builds a 20-point calibration table, using an AS5600 magnetic encoder as ground truth."
draft: true
---

Every hobby servo ships with the same assumption baked into every tutorial: pick a minimum pulse, a maximum pulse, draw a straight line between them. [It's quietly wrong](/articles/hobby-servo-calibration/) — the real pulse-to-angle curve bows away from that line, sometimes by several degrees. Servo Calibrator is the tool that measures your own servo's real curve instead of assuming one.

Wire up an Arduino Nano, an AS5600 magnetic encoder, and the servo you want to characterize; open one self-contained HTML page in the browser; click **Calibrate**. The firmware stall-scans both directions to find the servo's true mechanical limits, sweeps the full range twice, and builds a direction-averaged 20-point lookup table — no protractor, no hand measurement, usually done in under a minute. Once calibrated, the same page drives and plots a live trajectory against that table, with a toggle to compare it against the naive 2-point line in real time.

This tool is purely for characterization — it doesn't ask about horn position, direction, or a logical zero point, and it always reports the servo's own physical range, `[0, maxAngle]`. Turning that into an actual motor-driver constructor for your project is a separate, application-specific step.

## Wiring

![Fritzing wiring diagram showing an Arduino Nano connected to an AS5600 magnetic encoder over I2C and to a servo's signal line, with the servo powered from a separate external supply.](/images/servo-calibrator/hookup.png)

The AS5600 never touches the servo — it's a contactless magnetic angle sensor that just watches a magnet spin underneath it. That means the one physical requirement is a **diametrically magnetized** magnet (poles across the diameter, not the faces) mounted on the servo's output shaft, centered over the chip, a millimeter or two above the package. I didn't build any special mount for mine — just glued the magnet onto a spare servo horn and bolted the horn back on.

Everything else is ordinary wiring: servo signal to a digital pin (`A3` in the stock firmware), AS5600 on the board's I²C bus (`SDA`/`SCL` — `A4`/`A5` on an Uno/Nano), common ground tying the Arduino, the AS5600, and the servo together.

The one thing worth getting right is power. **Run the servo off its own external 4.8&ndash;6V supply, not the Arduino's 5V or Vin pin** — a small board's onboard regulator generally can't source a servo's stall current. 4.8&ndash;6V covers most hobby servos, but check your particular servo's datasheet before wiring it up; some digital servos want a narrower or higher range. Either way, tie that supply's ground to the Arduino's ground, or both the encoder readings and the servo's behavior get unreliable.

## Getting it running

1. **Install two libraries in the Arduino IDE** — both free, no private access needed:
   - **`AS5600`** (RobTillaart's) via Library Manager (`Sketch → Include Library → Manage Libraries…`, search `AS5600`), or `arduino-cli lib install "AS5600"`.
   - **[Universal-Trajectory-Interface](https://github.com/vishwam-aggarwal/Universal-Trajectory-Interface)** — not in Library Manager's index, so install it manually: download the repo as a ZIP from GitHub and use `Sketch → Include Library → Add .ZIP Library…`, or `git clone` it straight into your sketchbook's `libraries/` folder. Restart the IDE if it was already open.
2. **Flash the firmware.** Open `ServoCalibrator_Companion/ServoCalibrator_Companion.ino` from the [GitHub repo](https://github.com/vishwam-aggarwal/Servo-Calibrator) in the Arduino IDE and upload it to your board, same as any other sketch. Servo signal is fixed at pin `A3` in the sketch (edit `SERVO_PIN` if you need a different one).
3. **Serve the app.** `ServoCalibrator.html` needs `http://`, not a `file://` URL, for Web Serial to work reliably — `python -m http.server 8000` from the folder it's in, then open it in **Chrome or Edge** (Web Serial isn't implemented in Firefox or Safari).
4. **Connect, then Calibrate.** Click *Connect…*, pick your board's serial port, then click **Calibrate**. One button, fully automated — the trajectory/chart interface unlocks once the sweep finishes.

Full protocol reference, safety notes, and known limitations are in the [project README](https://github.com/vishwam-aggarwal/Servo-Calibrator).
