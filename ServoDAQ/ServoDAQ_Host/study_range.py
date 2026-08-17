"""
study_range.py

Data-collection script comparing the naive single-step stall-detection
sweep against the smart coarse+fine rate-based algorithm (find_range()),
on the same servo, same session. Not part of the servo_daq library --
an experiment that uses it, kept separate so the library stays reusable
primitives and this stays "how we ran this particular study."

Saves every raw trace to CSV under ../data/, timestamped, so results are
re-plottable/re-analyzable without re-running the hardware.

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
from servo_daq import ServoDAQLink, CENTER_US, naive_stall_sweep, find_range

DATA_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "data")


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
        print(
            f"  smart range: {smart['min_pulse_us']}us ({smart['min_centideg']} centideg) to "
            f"{smart['max_pulse_us']}us ({smart['max_centideg']} centideg)"
        )

        low_path = os.path.join(DATA_DIR, f"naive_low_{stamp}.csv")
        high_path = os.path.join(DATA_DIR, f"naive_high_{stamp}.csv")
        summary_path = os.path.join(DATA_DIR, f"summary_{stamp}.csv")

        save_csv(low_path, low_trace, ["pulse_us", "centideg"])
        save_csv(high_path, high_trace, ["pulse_us", "centideg"])
        save_csv(summary_path, [
            ["naive", "low", low_edge[0], low_edge[1]],
            ["naive", "high", high_edge[0], high_edge[1]],
            ["smart", "low", smart["min_pulse_us"], smart["min_centideg"]],
            ["smart", "high", smart["max_pulse_us"], smart["max_centideg"]],
        ], ["method", "side", "pulse_us", "centideg"])

        print(f"\nsaved:\n  {low_path}\n  {high_path}\n  {summary_path}")


if __name__ == "__main__":
    main()
