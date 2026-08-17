"""
plot_naive_vs_smart.py

Follow-up to study_range.py: that script only ever saved the *endpoints*
of the smart coarse+fine scans (in summary_*.csv), not their full traces,
so there was nothing to actually overlay against the naive sweep's full
trace on a plot. This script re-runs both the naive sweep and find_range()
on real hardware -- in one single serial connection, same as
study_range.py -- and plots them together per side (low/high).

Position reference note: opening the serial port resets the board (DTR),
which re-zeros ServoDAQ_Companion's multi-turn tracking from wherever the
horn happens to sit at boot. That reference is only consistent *within
one connection* -- comparing centideg values across two separate runs
(two separate `with ServoDAQLink(...)` connections) is comparing two
different zero points, not a real discrepancy. That's why this script
does NOT reuse the previous run's naive_*.csv files (as an earlier
version of this script did, incorrectly) -- naive and smart are both
re-walked here, in the same connection, so they're on the same reference
and an overlay is actually meaningful.

Usage: python plot_naive_vs_smart.py [PORT]   (defaults to COM9)
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
        low_edge, naive_low = naive_stall_sweep(link, CENTER_US, 10, -1)
        print(f"  naive low edge: {low_edge[0]}us ({low_edge[1]} centideg)")

        link.move_to(CENTER_US)
        print("naive sweep, high side (1500us -> up, 10us steps, stop on stall)...")
        high_edge, naive_high = naive_stall_sweep(link, CENTER_US, 10, +1)
        print(f"  naive high edge: {high_edge[0]}us ({high_edge[1]} centideg)")

        link.move_to(CENTER_US)
        print("smart coarse+fine algorithm (same connection, same reference)...")
        smart = find_range(link, CENTER_US)
        print(
            f"  smart range: {smart['min_pulse_us']}us ({smart['min_centideg']} centideg) to "
            f"{smart['max_pulse_us']}us ({smart['max_centideg']} centideg)"
        )
        link.move_to(CENTER_US)  # leave the servo parked at center when done

    smart_low = smart["low_coarse_trace"] + smart["low_fine_trace"]
    smart_high = smart["high_coarse_trace"] + smart["high_fine_trace"]

    save_csv(os.path.join(DATA_DIR, f"naive_low_{stamp}.csv"), naive_low, ["pulse_us", "centideg"])
    save_csv(os.path.join(DATA_DIR, f"naive_high_{stamp}.csv"), naive_high, ["pulse_us", "centideg"])
    save_csv(os.path.join(DATA_DIR, f"smart_low_{stamp}.csv"), smart_low, ["pulse_us", "centideg"])
    save_csv(os.path.join(DATA_DIR, f"smart_high_{stamp}.csv"), smart_high, ["pulse_us", "centideg"])
    save_csv(os.path.join(DATA_DIR, f"summary_{stamp}.csv"), [
        ["naive", "low", low_edge[0], low_edge[1]],
        ["naive", "high", high_edge[0], high_edge[1]],
        ["smart", "low", smart["min_pulse_us"], smart["min_centideg"]],
        ["smart", "high", smart["max_pulse_us"], smart["max_centideg"]],
    ], ["method", "side", "pulse_us", "centideg"])
    print(f"saved traces + summary under {DATA_DIR} (stamp {stamp})")

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, (ax_low, ax_high) = plt.subplots(1, 2, figsize=(12, 5))

    ax_low.plot(*zip(*naive_low), "o-", label="naive", color="#1f77b4", markersize=3)
    ax_low.plot(*zip(*smart_low), "o-", label="smart (coarse+fine)", color="#d62728", markersize=3)
    ax_low.set_title("Low side (1500us -> down)")
    ax_low.set_xlabel("pulse (us)")
    ax_low.set_ylabel("position (centideg)")
    ax_low.invert_xaxis()
    ax_low.legend()
    ax_low.grid(True, alpha=0.3)

    ax_high.plot(*zip(*naive_high), "o-", label="naive", color="#1f77b4", markersize=3)
    ax_high.plot(*zip(*smart_high), "o-", label="smart (coarse+fine)", color="#d62728", markersize=3)
    ax_high.set_title("High side (1500us -> up)")
    ax_high.set_xlabel("pulse (us)")
    ax_high.set_ylabel("position (centideg)")
    ax_high.legend()
    ax_high.grid(True, alpha=0.3)

    fig.suptitle("Naive stall-sweep vs smart coarse+fine range-find (same connection/reference)")
    fig.tight_layout()

    plot_path = os.path.join(DATA_DIR, f"naive_vs_smart_{stamp}.png")
    fig.savefig(plot_path, dpi=150)
    print(f"saved plot:\n  {plot_path}")


if __name__ == "__main__":
    main()
