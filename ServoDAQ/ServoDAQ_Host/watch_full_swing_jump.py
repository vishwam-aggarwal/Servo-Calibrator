"""
watch_full_swing_jump.py

Visual-confirmation run of the low-side phantom-drift anomaly (see
probe_low_jump.py / this session's investigation): drives the servo
through its full measured range first (so there's no ambiguity about
"maybe it was already near the edge"), then deliberately triggers the
anomaly and captures it on CAP while a human watches the horn directly.

Sequence: max (2080us) -> min (350us) -> 2s pause -> command 330us,
capturing the whole 330us transition on CAP (30ms/sample, matches
probe_low_jump.py) so the trace lines up with what was visually observed.

Usage: python watch_full_swing_jump.py [PORT]   (defaults to COM9)
"""

import csv
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from servo_daq import ServoDAQLink

DATA_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "data")
MAX_US = 2080
MIN_US = 350
TARGET_US = 330
DELAY_S = 2
CAP_DELAY_MS = 30


def cap(link, pulse_us, delay_ms, timeout=8.0):
    reply = link.send_command(f"CAP {pulse_us} {delay_ms}", timeout=timeout)
    if not reply.startswith("CAPSTART"):
        raise RuntimeError(f"CAP rejected: {reply}")
    samples = []
    deadline = time.time() + timeout
    while time.time() < deadline:
        line = link.ser.readline().decode(errors="replace").strip()
        if not line or line.startswith("#"):
            continue
        if line.startswith("CP "):
            _, t_ms, centideg = line.split()
            samples.append((int(t_ms), int(centideg)))
        elif line.startswith("CAPEND"):
            return samples
    raise TimeoutError("CAP capture did not finish in time")


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else "COM9"

    with ServoDAQLink(port) as link:
        print(f"connecting to {port}...")
        if not link.wait_ready():
            print("warning: no ready banner seen -- board may already be running")
        link.ping()
        print("PING ok")

        print(f"moving to max ({MAX_US}us)...")
        _, c = link.move_to(MAX_US)
        print(f"  settled: {MAX_US}us -> {c} centideg")

        print(f"moving to min ({MIN_US}us)...")
        _, c = link.move_to(MIN_US)
        print(f"  settled: {MIN_US}us -> {c} centideg")

        print(f"pausing {DELAY_S}s...")
        time.sleep(DELAY_S)

        print(f"CAP {TARGET_US} {CAP_DELAY_MS} -- watch the horn now...")
        samples = cap(link, TARGET_US, delay_ms=CAP_DELAY_MS, timeout=8.0)
        print(f"  got {len(samples)} samples")

        prev = c
        for t_ms, cd in samples:
            step = cd - prev
            print(f"  t={t_ms:5d}ms  centideg={cd:7d}  step={step:+6d}")
            prev = cd

        stamp = time.strftime("%Y%m%d-%H%M%S")
        os.makedirs(DATA_DIR, exist_ok=True)
        csv_path = os.path.join(DATA_DIR, f"watch_full_swing_{stamp}.csv")
        with open(csv_path, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["t_ms", "angle_deg"])
            w.writerows([(t, round(c / 100.0, 2)) for t, c in samples])
        print(f"saved: {csv_path}")

        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        ts = [t for t, _ in samples]
        cs = [cd for _, cd in samples]
        fig, ax = plt.subplots(figsize=(8, 5))
        ax.plot(ts, cs, "o-", color="#9467bd", markersize=3)
        ax.set_xlabel("time since CAP 330 issued (ms)")
        ax.set_ylabel("position (centideg)")
        ax.set_title(f"Full swing (max->min) then commanded to {TARGET_US}us")
        ax.grid(True, alpha=0.3)
        fig.tight_layout()
        plot_path = os.path.join(DATA_DIR, f"watch_full_swing_{stamp}.png")
        fig.savefig(plot_path, dpi=150)
        print(f"saved: {plot_path}")

        print("returning to center...")
        link.move_to(1500)


if __name__ == "__main__":
    main()
