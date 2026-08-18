"""
probe_low_jump.py

Follow-up to the naive-vs-smart discrepancy: the naive low sweep jumps
from -8446 centideg (340us) to +22834 centideg (330us) in a single 10us
step, and this jump reproduces almost exactly (same endpoint, same
trigger pulse) across separate runs -- too repeatable to be a random
stray-sample glitch. This script uses CAP (raw high-rate capture,
already tracked through the same updatePositionTracking() tick() uses)
to look at what actually happens *inside* that one US step, at much
finer time resolution than US's own settle detector reports.

Sequence: settle at 350us (known-good, pre-jump plateau per the earlier
naive/smart traces), then CAP at 330us with delayMs=0 (as fast as I2C
allows) to see whether the transition is one discontinuous multi-hundred-
count jump between consecutive samples (~5ms apart or less) -- which
would mean the encoder itself reported a same-tick change bigger than
the unwrap algorithm's half-revolution assumption, an inherent ambiguity
the algorithm cannot resolve correctly -- or a smooth/gradual multi-step
creep that just happens to net out to a big total position change, which
would point somewhere else entirely (e.g. re-settling logic, not the
unwrap math itself).

Usage: python probe_low_jump.py [PORT]   (defaults to COM9)
"""

import csv
import sys
import os
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from servo_daq import ServoDAQLink

DATA_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "data")


def cap(link, pulse_us, delay_ms, timeout=5.0):
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

        print("settling at 350us (pre-jump plateau)...")
        _, centideg = link.move_to(350)
        print(f"  settled: 350us -> {centideg} centideg")

        delay_ms = int(sys.argv[2]) if len(sys.argv) > 2 else 30
        print(f"CAP 330 {delay_ms} -- watching the transition over ~{delay_ms*100}ms...")
        samples = cap(link, 330, delay_ms=delay_ms, timeout=8.0)
        print(f"  got {len(samples)} samples")

        prev = centideg
        max_step = 0
        max_step_at = None
        for t_ms, c in samples:
            step = c - prev
            if abs(step) > abs(max_step):
                max_step = step
                max_step_at = t_ms
            print(f"  t={t_ms:5d}ms  centideg={c:7d}  step={step:+6d}")
            prev = c

        print(f"\nlargest single-sample step: {max_step:+d} centideg at t={max_step_at}ms")
        print(f"(for reference, one raw AS5600 count ~= 8.79 centideg; "
              f"half a revolution (the unwrap threshold) = 18000 centideg)")

        stamp = time.strftime("%Y%m%d-%H%M%S")
        os.makedirs(DATA_DIR, exist_ok=True)
        csv_path = os.path.join(DATA_DIR, f"probe_low_jump_{stamp}.csv")
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
        ax.plot(ts, cs, "o-", color="#d62728", markersize=3)
        ax.set_xlabel("time since CAP 330 issued (ms)")
        ax.set_ylabel("position (centideg)")
        ax.set_title("Position vs time while commanded to 330us\n(from a settled 350us start)")
        ax.grid(True, alpha=0.3)
        fig.tight_layout()
        plot_path = os.path.join(DATA_DIR, f"probe_low_jump_{stamp}.png")
        fig.savefig(plot_path, dpi=150)
        print(f"saved: {plot_path}")


if __name__ == "__main__":
    main()
