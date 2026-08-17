"""
study_range.py

Data-collection script comparing the naive single-step stall-detection
sweep against the smart coarse+fine rate-based algorithm (find_range()),
on the same servo, same session. Not part of the servo_daq library --
an experiment that uses it, kept separate so the library stays reusable
primitives and this stays "how we ran this particular study."

Once find_range() has reliably located the real min/max (see
../README.md's investigation writeup for why "reliably" needed its own
fix), this also runs a fine calibration sweep across that exact range at
5us steps, in both directions (min->max, then max->min) -- the up/down
pair separates direction-dependent backlash from the angle-dependent
nonlinearity a single-direction sweep would confound (same reasoning as
historical-data/hysteresis_data.csv from the project this repo grew out
of). Plain sweep() (servo_daq.py), no stall detection needed here --
find_range() already established a safe range to walk inside of, so
every step is guaranteed to be real, settled motion.

Saves every raw trace to CSV under ../data/, timestamped, so results are
re-plottable/re-analyzable without re-running the hardware -- naive
low/high, smart's own coarse/fine sub-traces, and the fine up/down
calibration sweep are all one connection/session, so every CSV from one
run shares the same centideg zero reference and can be safely combined
on one chart (see the "opening the serial port resets the board" note
on ServoDAQLink -- CSVs from *different* runs/sessions are on different,
incomparable zero references).

Position values are signed centidegrees (degrees x100) straight from the
board -- already unwrapped/monotonic across turns by the firmware itself,
no wrap correction needed here or in whatever plots this CSV later.

Usage: python study_range.py [PORT]   (defaults to COM9)
"""

import csv
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from servo_daq import ServoDAQLink, CENTER_US, naive_stall_sweep, find_range, sweep

DATA_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "data")
FINE_STEP_US = 5


def save_csv(path, rows, header):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(header)
        w.writerows(rows)


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else "COM9"
    stamp = time.strftime("%Y%m%d-%H%M%S")

    with ServoDAQLink(port) as link:
        print(f"connecting to {port}...")
        if not link.wait_ready():
            print("warning: no ready banner seen -- board may already be running")
        link.ping()
        print("PING ok")

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

        link.move_to(CENTER_US)

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

        print(f"\nsaved:\n  {low_path}\n  {high_path}\n  {coarse_low_path}\n  {coarse_high_path}\n"
              f"  {fine_edge_low_path}\n  {fine_edge_high_path}\n  {fine_up_path}\n  {fine_down_path}\n  {summary_path}")


if __name__ == "__main__":
    main()
