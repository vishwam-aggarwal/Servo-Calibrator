"""
study_range.py

Full per-servo characterization pipeline, one connection/session:
naive sweep -> smart find_range() -> fine (5us, both directions)
calibration sweep -> N-point lookup-table accuracy test. Not part of the
servo_daq library -- an experiment that uses it, kept separate so the
library stays reusable primitives and this stays "how we ran this
particular study." Written to be run identically across multiple
different servos (same code path, same thresholds, same phases every
time) so results are comparable servo-to-servo -- only the port, the
accuracy-test duration, and which physical unit is plugged in are meant
to vary between runs. motor_type/unit (plain integers -- see
../MOTOR_TYPES.md for what each number actually is) get folded into
every output filename so results from the planned multi-servo study
stay sortable/parseable later instead of just being a pile of
timestamps with no way to tell which physical unit produced which file.

Once find_range() has reliably located the real min/max (see
../README.md's investigation writeup for why "reliably" needed its own
fix), runs a fine calibration sweep across that exact range at 5us
steps, in both directions (min->max, then max->min) -- the up/down pair
separates direction-dependent backlash from the angle-dependent
nonlinearity a single-direction sweep would confound (same reasoning as
historical-data/hysteresis_data.csv from the project this repo grew out
of). Plain sweep() (servo_daq.py), no stall detection needed here --
find_range() already established a safe range to walk inside of, so
every step is guaranteed to be real, settled motion.

Then, the accuracy test (see ../CLAUDE.md's "Planned next step" this
implements): direction-averages the fine sweep into one ground-truth
pulse->angle curve, builds a 2-point linear baseline and 10/20/30/40/50-
point lookup tables from it (same binary-search + linear-interpolation
algorithm the real embedded tools use), then repeatedly picks a target
angle AND a model *independently at random* (see run_accuracy_test()'s
own docstring for why: an earlier version paired one target angle with
all 6 models per round, which clustered 5 of 6 moves into ~3us nudges
too small to decisively move the servo, letting real backlash/deadband
scatter -- unrelated to any model's actual accuracy -- inflate every
model's error roughly equally while swamping the finer differences
between them), computes that model's predicted pulse for that angle,
commands it, and measures the real resulting angle -- the actual
end-to-end accuracy that matters (how close the servo lands to where you
asked it to go), not just a curve-fit residual. Runs until a wall-clock
deadline, not a fixed sample count, so "run it for a few hours" just
works.

Data safety for a long unattended run: naive/smart/fine results are
saved to CSV immediately once captured, *before* the (potentially
hours-long) accuracy test starts -- a crash deep into the accuracy test
never risks losing the already-good calibration data. The accuracy
test's own trial CSV is flushed after every single move, so an
interrupted run still leaves complete, usable data up to the last
finished move, and its summary is computed from whatever's on disk even
if the run didn't reach its deadline.

Servo safety: MOVE_REST_S is a deliberate pause after every move in the
accuracy test specifically (naive/smart/fine phases are short, already-
verified sequences, left as they were) -- explicit request after a
servo died from exactly this kind of unattended extended-duration
stress before. Every commanded pulse is also clamped to
[min_pulse_us, max_pulse_us] as found by find_range() -- the accuracy
test never drives the servo anywhere the fine sweep didn't already prove
is safe, however the interpolation math resolves. The whole run is
wrapped in try/finally so the servo returns to CENTER_US on any error,
not left sitting at an extreme position.

Position values are signed centidegrees (degrees x100) straight from the
board -- already unwrapped/monotonic across turns by the firmware itself,
no wrap correction needed here or in whatever plots this CSV later.

Usage: python study_range.py [PORT] [ACCURACY_HOURS] [MOTOR_TYPE] [UNIT]
  PORT: defaults to COM9.
  ACCURACY_HOURS: wall-clock duration of the accuracy test. Defaults to
    0.25 (15min) -- deliberately short unless explicitly asked for more,
    so a routine run never silently turns into a multi-hour commitment.
  MOTOR_TYPE / UNIT: plain integers identifying which physical servo
    this run is characterizing -- see ../MOTOR_TYPES.md. Default to "0"
    (unlabeled/test run) if omitted, with a warning printed -- always
    pass real values for anything meant to be kept as study data.
"""

import csv
import os
import random
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from servo_daq import ServoDAQLink, CENTER_US, naive_stall_sweep, find_range, sweep

DATA_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "data")
FINE_STEP_US = 5

# ------------------------------------------------------------------
# Accuracy test: N-point table vs. the naive 2-point linear assumption,
# validated against real hardware at random target angles.
# ------------------------------------------------------------------
TABLE_SIZES = {"linear2": 2, "table10": 10, "table20": 20, "table30": 30, "table40": 40, "table50": 50}
MOVE_REST_S = 1.0              # deliberate rest after every move in the accuracy test -- see module docstring
DEFAULT_ACCURACY_HOURS = 0.25  # ~15min default; pass a real duration explicitly for a long run
PROGRESS_EVERY = 50            # print a progress line every N trials


def save_csv(path, rows, header):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(header)
        w.writerows(rows)


def build_ground_truth(fine_up, fine_down):
    """Direction-averaged pulse->angle curve (centideg, float) at every
    pulse shared by both fine sweeps. Sorted by pulse ascending -- for
    this servo (and any servo whose real range find_range() already
    validated as monotonic) that's also ascending by angle."""
    up = dict(fine_up)
    down = dict(fine_down)
    common = sorted(set(up) & set(down))
    return [(p, (up[p] + down[p]) / 2.0) for p in common]


def build_table(ground_truth, n_points):
    """n_points breakpoints evenly spaced by PULSE across the ground
    truth's own range (matches how the physical sweep itself is
    structured), each snapped to the nearest actually-measured grid
    point so every table breakpoint is a real reading, not an
    interpolated fabrication. n_points=2 is exactly the naive linear
    baseline -- just the two endpoints."""
    pulses = [p for p, _ in ground_truth]
    gt_map = dict(ground_truth)
    lo, hi = pulses[0], pulses[-1]
    table, seen = [], set()
    for i in range(n_points):
        target = lo + (hi - lo) * i / (n_points - 1)
        nearest = min(pulses, key=lambda p: abs(p - target))
        if nearest in seen:
            continue
        seen.add(nearest)
        table.append((nearest, gt_map[nearest]))
    return table


def angle_to_pulse(table, target_angle):
    """Inverse lookup: target angle (centideg) -> predicted pulse (us),
    binary search + linear interpolation over a table sorted by pulse
    (== sorted by angle here). Same algorithm the real embedded tools
    use, just run in the opposite direction (this test asks "what pulse
    should I send to reach this angle", the real usage this validates).
    Clamped to the table's own domain -- never extrapolates."""
    if target_angle <= table[0][1]:
        return table[0][0]
    if target_angle >= table[-1][1]:
        return table[-1][0]
    lo, hi = 0, len(table) - 1
    while hi - lo > 1:
        mid = (lo + hi) // 2
        if table[mid][1] <= target_angle:
            lo = mid
        else:
            hi = mid
    p0, a0 = table[lo]
    p1, a1 = table[hi]
    frac = (target_angle - a0) / (a1 - a0)
    return p0 + frac * (p1 - p0)


def run_accuracy_test(link, ground_truth, min_us, max_us, hours, out_path, seed=None):
    """Randomized end-to-end accuracy test. Every trial is fully
    independent: its own random target angle AND its own independently-
    chosen model -- no more testing all 6 models against one shared
    target in a clustered round (see the investigation this replaced:
    when models shared a target, 5 of 6 moves per round were ~3us nudges
    between nearly-identical predicted pulses -- too small to decisively
    re-engage the servo's mechanism, letting real backlash/deadband play
    show up as growing, model-unrelated *scatter* that inflated every
    model's absolute error roughly equally but swamped the finer,
    between-model distinctions). Every move here is a genuine jump to a
    fresh random angle, so there's no tiny-delta regime for that to hide
    in. Model choice is drawn from a shuffled bag that's refilled each
    time it empties (not plain independent random choice) so sample
    counts stay balanced across models over the run, not just in
    expectation.

    Runs until `hours` wall-clock time elapses; every row (including a
    wall-clock timestamp and how long that move's settle actually took)
    is flushed immediately, so an interrupted run still leaves complete,
    usable data up to the last finished move.

    All internal math (table breakpoints, angle_to_pulse(), the target
    draw, the error itself) stays in centidegrees, matching every other
    number in this project (see servo_daq.py's own module comment) and
    the ground_truth this function is handed. Only the CSV columns are
    converted to plain degrees at the point of writing -- this file has
    no downstream reader in the codebase (only ad hoc analysis), so
    there's no interoperability reason to keep it in centidegrees, and
    plain degrees is much easier to eyeball in a raw row."""
    rng = random.Random(seed)
    models = {name: build_table(ground_truth, n) for name, n in TABLE_SIZES.items()}
    min_angle, max_angle = ground_truth[0][1], ground_truth[-1][1]

    start = time.time()
    deadline = start + hours * 3600
    trial = 0
    model_bag = []
    os.makedirs(os.path.dirname(out_path), exist_ok=True)
    with open(out_path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["trial", "timestamp", "model", "target_angle_deg", "predicted_pulse_us",
                     "actual_pulse_us", "actual_angle_deg", "error_deg", "settle_s"])
        while time.time() < deadline:
            if not model_bag:
                model_bag = list(models.keys())
                rng.shuffle(model_bag)
            name = model_bag.pop()

            target_angle = rng.uniform(min_angle, max_angle)  # centideg -- internal math unit
            pulse = int(round(angle_to_pulse(models[name], target_angle)))
            pulse = max(min_us, min(max_us, pulse))  # never trust interpolation alone -- clamp to the known-safe range

            t0 = time.time()
            actual_pulse, actual_angle = link.move_to(pulse)  # actual_angle: centideg, straight from the board
            settle_s = time.time() - t0

            error = actual_angle - target_angle  # still centideg here
            w.writerow([trial, f"{time.time():.3f}", name, round(target_angle / 100, 3), pulse,
                         actual_pulse, round(actual_angle / 100, 2), round(error / 100, 3), round(settle_s, 3)])
            f.flush()
            time.sleep(MOVE_REST_S)

            trial += 1
            if trial % PROGRESS_EVERY == 0:
                elapsed_min = (time.time() - start) / 60
                remaining_min = (deadline - time.time()) / 60
                print(f"  trial {trial}: {elapsed_min:.1f}min elapsed, ~{remaining_min:.1f}min remaining")
    return trial


def summarize_accuracy(trials_path, summary_path):
    """Per-model mean/max/RMS absolute error (degrees). Reads whatever's
    actually in the trials CSV -- safe to call after a partial/
    interrupted run, not just a clean completion."""
    by_model = {}
    with open(trials_path, newline="") as f:
        for row in csv.DictReader(f):
            by_model.setdefault(row["model"], []).append(float(row["error_deg"]))

    rows = []
    for name in TABLE_SIZES:
        errs = by_model.get(name, [])
        if not errs:
            continue
        n = len(errs)
        mean_abs = sum(abs(e) for e in errs) / n
        max_abs = max(abs(e) for e in errs)
        rms = (sum(e * e for e in errs) / n) ** 0.5
        rows.append([name, TABLE_SIZES[name], n, round(mean_abs, 4), round(max_abs, 4), round(rms, 4)])
    save_csv(summary_path, rows, ["model", "points", "n_samples", "mean_abs_error_deg", "max_abs_error_deg", "rms_error_deg"])
    return rows


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else "COM9"
    accuracy_hours = float(sys.argv[2]) if len(sys.argv) > 2 else DEFAULT_ACCURACY_HOURS
    motor_type = sys.argv[3] if len(sys.argv) > 3 else "0"
    unit = sys.argv[4] if len(sys.argv) > 4 else "0"
    if motor_type == "0" or unit == "0":
        print("warning: motor_type/unit not given -- labeling this run type0_unit0 (unlabeled/test), "
              "see MOTOR_TYPES.md for the real inventory numbers")
    stamp = f"type{motor_type}_unit{unit}_{time.strftime('%Y%m%d-%H%M%S')}"

    with ServoDAQLink(port) as link:
        print(f"connecting to {port}...")
        if not link.wait_ready():
            print("warning: no ready banner seen -- board may already be running")
        link.ping()
        print("PING ok")

        try:
            # ---- Phase 1: naive + smart + fine (short, a few minutes) ----
            print("naive sweep, low side (1500us -> down, 10us steps, stop on stall)...")
            low_edge, low_trace = naive_stall_sweep(link, CENTER_US, 10, -1)
            print(f"  naive low edge: {low_edge[0]}us ({low_edge[1]} centideg), {len(low_trace)} steps")

            link.move_to(CENTER_US)
            print("naive sweep, high side (1500us -> up, 10us steps, stop on stall)...")
            high_edge, high_trace = naive_stall_sweep(link, CENTER_US, 10, +1)
            print(f"  naive high edge: {high_edge[0]}us ({high_edge[1]} centideg), {len(high_trace)} steps")

            link.move_to(CENTER_US)
            print("smart coarse+fine algorithm...")
            smart = find_range(link)
            min_us, max_us = smart["min_pulse_us"], smart["max_pulse_us"]
            print(
                f"  smart range: {min_us}us ({smart['min_centideg']} centideg) to "
                f"{max_us}us ({smart['max_centideg']} centideg)"
            )

            link.move_to(min_us)
            print(f"fine calibration sweep, min -> max ({min_us}us -> {max_us}us, {FINE_STEP_US}us steps)...")
            t0 = time.time()
            fine_up = sweep(link, min_us, max_us, FINE_STEP_US)
            print(f"  {len(fine_up)} points, {time.time() - t0:.1f}s")

            print(f"fine calibration sweep, max -> min ({max_us}us -> {min_us}us, {FINE_STEP_US}us steps)...")
            t0 = time.time()
            fine_down = sweep(link, max_us, min_us, FINE_STEP_US)
            print(f"  {len(fine_down)} points, {time.time() - t0:.1f}s")

            # Save phase-1 results NOW, before the long accuracy test starts --
            # a crash deep into that test must never risk this data too.
            low_path = os.path.join(DATA_DIR, f"naive_low_{stamp}.csv")
            high_path = os.path.join(DATA_DIR, f"naive_high_{stamp}.csv")
            summary_path = os.path.join(DATA_DIR, f"summary_{stamp}.csv")
            fine_up_path = os.path.join(DATA_DIR, f"fine_up_{stamp}.csv")
            fine_down_path = os.path.join(DATA_DIR, f"fine_down_{stamp}.csv")
            coarse_low_path = os.path.join(DATA_DIR, f"smart_coarse_low_{stamp}.csv")
            coarse_high_path = os.path.join(DATA_DIR, f"smart_coarse_high_{stamp}.csv")
            fine_edge_low_path = os.path.join(DATA_DIR, f"smart_fine_low_{stamp}.csv")
            fine_edge_high_path = os.path.join(DATA_DIR, f"smart_fine_high_{stamp}.csv")

            save_csv(low_path, low_trace, ["pulse_us", "centideg"])
            save_csv(high_path, high_trace, ["pulse_us", "centideg"])
            save_csv(fine_up_path, fine_up, ["pulse_us", "centideg"])
            save_csv(fine_down_path, fine_down, ["pulse_us", "centideg"])
            save_csv(coarse_low_path, smart["low_coarse_trace"], ["pulse_us", "centideg"])
            save_csv(coarse_high_path, smart["high_coarse_trace"], ["pulse_us", "centideg"])
            save_csv(fine_edge_low_path, smart["low_fine_trace"], ["pulse_us", "centideg"])
            save_csv(fine_edge_high_path, smart["high_fine_trace"], ["pulse_us", "centideg"])
            save_csv(summary_path, [
                ["naive", "low", low_edge[0], low_edge[1]],
                ["naive", "high", high_edge[0], high_edge[1]],
                ["smart", "low", min_us, smart["min_centideg"]],
                ["smart", "high", max_us, smart["max_centideg"]],
            ], ["method", "side", "pulse_us", "centideg"])
            print(f"\nphase 1 saved:\n  {low_path}\n  {high_path}\n  {coarse_low_path}\n  {coarse_high_path}\n"
                  f"  {fine_edge_low_path}\n  {fine_edge_high_path}\n  {fine_up_path}\n  {fine_down_path}\n"
                  f"  {summary_path}")

            # ---- Phase 2: N-point table accuracy test (potentially hours) ----
            ground_truth = build_ground_truth(fine_up, fine_down)
            print(f"\nground truth: {len(ground_truth)} direction-averaged points")

            accuracy_path = os.path.join(DATA_DIR, f"accuracy_trials_{stamp}.csv")
            accuracy_summary_path = os.path.join(DATA_DIR, f"accuracy_summary_{stamp}.csv")
            print(f"accuracy test: {sorted(TABLE_SIZES.values())} models, independently random target angle "
                  f"AND model per trial, {MOVE_REST_S}s rest/move, running for {accuracy_hours}h...")
            try:
                n_trials = run_accuracy_test(link, ground_truth, min_us, max_us, accuracy_hours, accuracy_path)
                print(f"  {n_trials} independent trials completed")
            except Exception as e:
                print(f"\naccuracy test stopped early ({e}) -- summarizing whatever was captured")

            if os.path.exists(accuracy_path):
                summary_rows = summarize_accuracy(accuracy_path, accuracy_summary_path)
                print("\naccuracy summary (degrees):")
                print(f"  {'model':<10}{'points':>8}{'n':>8}{'mean|err|':>12}{'max|err|':>12}{'rms':>10}")
                for r in summary_rows:
                    print(f"  {r[0]:<10}{r[1]:>8}{r[2]:>8}{r[3]:>12.4f}{r[4]:>12.4f}{r[5]:>10.4f}")
                print(f"\nsaved:\n  {accuracy_path}\n  {accuracy_summary_path}")

        finally:
            try:
                link.move_to(CENTER_US)
                print("\nreturned to center")
            except Exception as e:
                print(f"\nwarning: failed to return to center: {e}")


if __name__ == "__main__":
    main()
