"""
hand_stall_test.py

Isolates whether "smooth phantom position drift with zero real rotation"
(seen driving this servo past its real low-side mechanical limit, see
probe_low_jump.py) is caused by the servo being STALLED specifically, as
opposed to anything specific to that extreme low-pulse region. If a
normal, well-within-range command that's stalled by hand (holding the
horn stationary) shows the same kind of smooth false climb, that points
at something electrical happening during any stall (e.g. motor stall
current inducing bias into the AS5600's field reading) rather than a
mechanical fault (magnet slip -- already ruled out, magnet's glued
solid) or anything specific to the low-pulse edge case itself.

Usage: python hand_stall_test.py [PORT] [FROM_US] [TO_US] [DELAY_MS]
  Defaults: COM9 800 1200 30
"""

import csv
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from servo_daq import ServoDAQLink

DATA_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "data")


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
    from_us = int(sys.argv[2]) if len(sys.argv) > 2 else 800
    to_us = int(sys.argv[3]) if len(sys.argv) > 3 else 1200
    delay_ms = int(sys.argv[4]) if len(sys.argv) > 4 else 30

    with ServoDAQLink(port) as link:
        print(f"connecting to {port}...")
        if not link.wait_ready():
            print("warning: no ready banner seen -- board may already be running")
        link.ping()
        print("PING ok")

        print(f"settling at {from_us}us (starting point)...")
        _, centideg = link.move_to(from_us)
        print(f"  settled: {from_us}us -> {centideg} centideg")

        print("\n>>> Hold the horn firmly stationary now.")
        for i in (5, 4, 3, 2, 1):
            print(f"    commanding in {i}...")
            time.sleep(1)

        print(f"CAP {to_us} {delay_ms} -- watching while you hold it...")
        samples = cap(link, to_us, delay_ms=delay_ms, timeout=8.0)
        print(f"  got {len(samples)} samples")

        prev = centideg
        for t_ms, c in samples:
            step = c - prev
            print(f"  t={t_ms:5d}ms  centideg={c:7d}  step={step:+6d}")
            prev = c

        total_change = samples[-1][1] - centideg
        print(f"\ntotal reported change over capture: {total_change:+d} centideg "
              f"(you were holding it stationary -- this should be ~0 if the encoder is trustworthy under stall)")

        stamp = time.strftime("%Y%m%d-%H%M%S")
        os.makedirs(DATA_DIR, exist_ok=True)
        csv_path = os.path.join(DATA_DIR, f"hand_stall_{stamp}.csv")
        with open(csv_path, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["t_ms", "angle_deg"])
            w.writerows([(t, round(c / 100.0, 2)) for t, c in samples])
        print(f"saved: {csv_path}")

        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        ts = [t for t, _ in samples]
        cs = [c for _, c in samples]
        fig, ax = plt.subplots(figsize=(8, 5))
        ax.plot(ts, cs, "o-", color="#2ca02c", markersize=3)
        ax.set_xlabel("time since CAP issued (ms)")
        ax.set_ylabel("position (centideg)")
        ax.set_title(f"Hand-stall test: {from_us}us -> {to_us}us, horn held stationary by hand")
        ax.grid(True, alpha=0.3)
        fig.tight_layout()
        plot_path = os.path.join(DATA_DIR, f"hand_stall_{stamp}.png")
        fig.savefig(plot_path, dpi=150)
        print(f"saved: {plot_path}")

        print("\nrelease the horn and letting it settle back to center now...")
        link.move_to(1500)


if __name__ == "__main__":
    main()
