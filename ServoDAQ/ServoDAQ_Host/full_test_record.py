"""
full_test_record.py

Continuous recording of the whole max->min->pause->330us sequence (see
watch_full_swing_jump.py), stitched into one single timeline instead of
one CAP capture per phase, so both a pulse-vs-time (commanded us) and a
position-vs-time (measured angle) chart can be drawn over the *entire*
test, phase transitions included -- "the true test" per request, not
just the final 330us leg in isolation.

CAP's own buffer is fixed at 100 samples per call (see the .ino), so a
motion phase that takes longer than one buffer's worth of samples is
covered by re-issuing CAP with the same pulse (a no-op to the servo --
it just keeps doing whatever it was already doing) back to back until a
simple settle check passes or a burst cap is hit. Each burst's own
t_ms resets to 0, so bursts are stitched onto one continuous timeline
using the host's own wall-clock time at the moment each CAP command is
sent -- accurate to within normal serial round-trip jitter (a few ms),
plenty for this purpose.

Usage: python full_test_record.py [PORT]   (defaults to COM9)
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
PAUSE_S = 2
CAP_DELAY_MS = 30
SETTLE_WINDOW = 15
SETTLE_THRESH_CENTIDEG = 20
MAX_BURSTS_PER_PHASE = 5


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


def is_settled(samples):
    if len(samples) < SETTLE_WINDOW:
        return False
    window = [c for _, c in samples[-SETTLE_WINDOW:]]
    return (max(window) - min(window)) <= SETTLE_THRESH_CENTIDEG


def record_phase(link, label, pulse_us, global_t0, trace, transitions,
                  max_bursts=MAX_BURSTS_PER_PHASE, delay_ms=CAP_DELAY_MS, fixed_bursts=None):
    """Repeatedly CAPs at pulse_us, appending (global_ms, centideg) samples
    to `trace`, until settled or max_bursts/fixed_bursts is reached.
    Returns the running global elapsed ms after this phase."""
    transitions.append((time.time() - global_t0) * 1000.0, )
    print(f"-- {label}: commanding {pulse_us}us --")
    n_bursts = fixed_bursts if fixed_bursts is not None else max_bursts
    for burst in range(n_bursts):
        send_t = (time.time() - global_t0) * 1000.0
        samples = cap(link, pulse_us, delay_ms)
        for t_ms, centideg in samples:
            trace.append((send_t + t_ms, centideg))
        print(f"   burst {burst+1}: {len(samples)} samples, "
              f"last centideg={samples[-1][1]}, elapsed so far={trace[-1][0]:.0f}ms")
        if fixed_bursts is None and is_settled(samples):
            print("   settled, moving on")
            break
    return trace[-1][0]


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else "COM9"

    with ServoDAQLink(port) as link:
        print(f"connecting to {port}...")
        if not link.wait_ready():
            print("warning: no ready banner seen -- board may already be running")
        link.ping()
        print("PING ok")

        trace = []          # (global_ms, centideg)
        transitions = []    # global_ms of each commanded-pulse change, paired with pulse_us below
        pulses = []          # parallel to transitions: the pulse_us commanded at that point

        global_t0 = time.time()

        pulses.append(MAX_US)
        record_phase(link, "swing to max", MAX_US, global_t0, trace, transitions)

        pulses.append(MIN_US)
        record_phase(link, "swing to min", MIN_US, global_t0, trace, transitions)

        pulses.append(MIN_US)  # pulse doesn't change, but mark the start of the held pause
        pause_bursts = max(1, round(PAUSE_S * 1000 / (100 * CAP_DELAY_MS)))
        record_phase(link, f"hold at min for {PAUSE_S}s", MIN_US, global_t0, trace, transitions,
                     fixed_bursts=pause_bursts)

        pulses.append(TARGET_US)
        record_phase(link, "command target (the anomaly)", TARGET_US, global_t0, trace, transitions,
                     max_bursts=6)

        print(f"\ntotal samples recorded: {len(trace)}, total span: {trace[-1][0]:.0f}ms")

        stamp = time.strftime("%Y%m%d-%H%M%S")
        os.makedirs(DATA_DIR, exist_ok=True)

        trace_path = os.path.join(DATA_DIR, f"full_test_trace_{stamp}.csv")
        with open(trace_path, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["t_ms", "angle_deg"])
            w.writerows([(round(t, 1), round(c / 100.0, 2)) for t, c in trace])
        print(f"saved: {trace_path}")

        pulse_path = os.path.join(DATA_DIR, f"full_test_pulse_{stamp}.csv")
        with open(pulse_path, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["t_ms", "pulse_us"])
            w.writerows([(round(t, 1), p) for t, p in zip(transitions, pulses)])
        print(f"saved: {pulse_path}")

        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        ts = [t for t, _ in trace]
        cs = [c for _, c in trace]
        degs = [c / 100.0 for c in cs]

        # Build a step function for the pulse-vs-time chart: hold each
        # commanded value until the next transition, extended to the end
        # of the recorded trace.
        step_ts, step_us = [], []
        for i, (t, p) in enumerate(zip(transitions, pulses)):
            step_ts.append(t)
            step_us.append(p)
            end_t = transitions[i + 1] if i + 1 < len(transitions) else ts[-1]
            step_ts.append(end_t)
            step_us.append(p)

        fig, (ax_pulse, ax_angle) = plt.subplots(2, 1, figsize=(11, 8), sharex=True)

        ax_pulse.plot(step_ts, step_us, color="#1f77b4", linewidth=2)
        ax_pulse.set_ylabel("commanded pulse (us)")
        ax_pulse.set_title("Pulse width vs time")
        ax_pulse.grid(True, alpha=0.3)
        for t, p in zip(transitions, pulses):
            ax_pulse.axvline(t, color="#999999", linestyle=":", linewidth=0.8)

        ax_angle.plot(ts, degs, color="#d62728", linewidth=1.2)
        ax_angle.set_ylabel("measured position (deg)")
        ax_angle.set_xlabel("time (ms)")
        ax_angle.set_title("Encoder position vs time")
        ax_angle.grid(True, alpha=0.3)
        for t, p in zip(transitions, pulses):
            ax_angle.axvline(t, color="#999999", linestyle=":", linewidth=0.8)

        fig.suptitle("Full test: max -> min -> 2s hold -> target (330us)")
        fig.tight_layout()

        plot_path = os.path.join(DATA_DIR, f"full_test_{stamp}.png")
        fig.savefig(plot_path, dpi=150)
        print(f"saved: {plot_path}")

        print("returning to center...")
        link.move_to(1500)


if __name__ == "__main__":
    main()
