# Historical data

Raw measurement CSVs from the hand-built characterization project
(`Servo_Auto_Calibrator`) that both tools in this repo grew out of —
archived here when that project's local sketchbook folder was cleaned up
(2026-08-09), since this data isn't reproduced anywhere else. Kept for
reference/reproducibility, not consumed by either tool at runtime.

- **`calibration_data.csv`** — per-step (pulse_us, angle_deg, valid) from
  the original stall-detected endpoint/linearity sweep. Confirmed this
  servo's real mechanical range and quantified how close its pulse↔angle
  response is to a straight line in each direction.
- **`hysteresis_data.csv`** — per-pulse (direction, pulse_us, angle_deg,
  valid) from a same-range up/down sweep, isolating direction-dependent
  backlash from the angle-dependent nonlinearity the calibration sweep
  found (they're confounded if measured together — see below).
- **`repeatability_data.csv`** — 140-trial single-target positioning
  results (compensated vs. uncompensated), the follow-up test that asked
  whether the measured backlash actually matters for real point-to-point
  moves, and whether a simple undershoot-and-reapproach compensation
  scheme helps.

This data — and the servo it came from — is what the calibration lookup
table both tools now use was validated against: the same nonlinearity
characterized here (~3.6–3.8° max deviation from a 2-point line) is what
led to the 20-point lookup table both `ServoCalibrator.html`'s wizard
output and `TrajectoryDemo.html`'s live comparison are built around. See
the main [`CLAUDE.md`](../CLAUDE.md) for how that investigation connects
to what's actually in this repo today.
