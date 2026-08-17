"""
ServoDAQ host driver.

Talks to ServoDAQ_Companion.ino over serial (115200 baud, one command per
line, \n-terminated ASCII). See ../ServoDAQ_Companion/ServoDAQ_Companion.ino
for the protocol this drives -- PING and US only; range-finding, models,
and everything else lives here, not on the board.

Position values from the board are signed centidegrees (degrees x100),
already unwrapped across turns by the firmware itself -- see the .ino's
own file-level comment for how. Nothing here needs to (or should) do any
wraparound correction on them; a plain subtraction between any two
readings is always the true delta, and a trace plotted straight out of
this module is monotonic by construction.
"""

import argparse
import time

import serial

# Mirrors the same-named constants in ServoDAQ_Companion.ino -- no shared
# config file between the two, so these have to be kept in sync by hand.
CENTER_US    = 1500
ABS_FLOOR_US = 80
ABS_CEIL_US  = 3100


class ServoDAQLink:
    """One serial connection to the board. Opening the port resets it
    (DTR, same as any Arduino serial terminal), so connect() waits for
    the '# READY' boot banner before anything else is sent.
    """

    def __init__(self, port, baud=115200, timeout=3.0):
        self.ser = serial.Serial(port, baud, timeout=timeout)

    def wait_ready(self, timeout=6.0):
        deadline = time.time() + timeout
        while time.time() < deadline:
            line = self.ser.readline().decode(errors="replace").strip()
            if line.startswith("# READY"):
                return True
        return False

    def send_command(self, cmd, timeout=5.0):
        """Send one command, return its reply line. '#' lines are boot/log
        noise, not replies -- skipped, never returned."""
        self.ser.write((cmd + "\n").encode())
        deadline = time.time() + timeout
        while time.time() < deadline:
            line = self.ser.readline().decode(errors="replace").strip()
            if not line or line.startswith("#"):
                continue
            return line
        raise TimeoutError(f"no reply to {cmd!r} within {timeout}s")

    def ping(self):
        reply = self.send_command("PING")
        if reply != "OK PONG":
            raise RuntimeError(f"unexpected PING reply: {reply!r}")

    def move_to(self, pulse_us):
        """US <pulseUs> -> (pulseUs, centideg). Blocks until the board
        reports settled. centideg is signed, unwrapped centidegrees --
        see this module's docstring. Timeout here (5s) is deliberately
        above the firmware's own SETTLE_TIMEOUT_MS (3s) -- a client
        timeout shorter than the device's own worst case is exactly the
        bug that caused a real command-desync problem elsewhere in this
        project's history, not repeating that here."""
        reply = self.send_command(f"US {pulse_us}", timeout=5.0)
        parts = reply.split()
        if parts[0] != "OK":
            raise RuntimeError(f"US {pulse_us} rejected: {reply}")
        return int(parts[1]), int(parts[2])

    def close(self):
        self.ser.close()

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()


# Raw AS5600 resolution, carried over from the firmware's own conversion
# (360/4096 deg/count) purely to re-express old raw-count-based defaults
# below in the new centidegree units -- not used for anything else here.
CENTIDEG_PER_COUNT = 36000 / 4096   # ~8.79


def sweep(link, start_us, end_us, step_us):
    """Plain traversal -- no stall detection, no early exit, no window.
    Walks every point from start_us to end_us at step_us and records the
    settled reading (centidegrees) at each. For seeing the actual shape
    of the response directly, rather than inferring where it breaks down
    from a windowed algorithm that a creep artifact can fool (see the
    340->330us investigation this was built to follow up on -- the
    firmware's own multi-turn tracking has since made the wrap half of
    that investigation moot; the creep/plateau half is still real)."""
    direction = 1 if end_us >= start_us else -1
    pulses = list(range(start_us, end_us + direction, direction * abs(step_us)))
    if pulses[-1] != end_us:
        pulses.append(end_us)

    trace = []
    for pulse in pulses:
        _, centideg = link.move_to(pulse)
        trace.append((pulse, centideg))
    return trace


def naive_stall_sweep(link, start_us, step_us, direction, stall_threshold=round(CENTIDEG_PER_COUNT),
                       floor_us=ABS_FLOOR_US, ceil_us=ABS_CEIL_US):
    """The dead-simple baseline for comparison against find_range(): step
    by step_us, compare each new reading only to the immediately previous
    one -- no window, no reference rate, no coarse/fine split. Stops the
    first time a single step's delta drops to stall_threshold or below.
    Reports the point where the stall was observed (not one step back --
    unlike find_range()'s convention, this method has no notion of "the
    step before," it just walks until the encoder stalls and calls that
    the limit). stall_threshold is in centidegrees; the default (~1 raw
    AS5600 count's worth) preserves the same real angular sensitivity the
    old raw-count threshold had. Returns ((pulse, centideg), full_trace)."""
    trace = []
    pulse = start_us
    _, prev_centideg = link.move_to(pulse)
    trace.append((pulse, prev_centideg))

    while True:
        pulse += direction * step_us
        if pulse < floor_us or pulse > ceil_us:
            raise RuntimeError(f"hit hard bound ({floor_us}..{ceil_us}) without detecting a stall")
        _, centideg = link.move_to(pulse)
        trace.append((pulse, centideg))
        if abs(centideg - prev_centideg) <= stall_threshold:   # plain subtraction -- already unwrapped
            return (pulse, centideg), trace
        prev_centideg = centideg


# ------------------------------------------------------------------
# Range-finding: coarse-then-fine, rate-based. Replaces an earlier
# windowed-net-movement version that looked for near-zero movement over
# a fixed span -- correct for a hard stop, but this servo doesn't go
# straight to zero response at its real limit (350us, established via
# the 1500->80us sweep): it visibly WEAKENS first, then only goes flat
# after a further transition into a separate, harder secondary stop
# further on. A near-zero criterion has no choice but to push through
# that whole transition every time to find its "flat enough" window.
#
# This version self-calibrates a reference per-step rate from the first
# few steps of each scan (assumed normal territory), then watches for a
# single step's delta to drop below a fraction of that reference -- the
# signature of the response changing character, caught right as it
# starts rather than once it's gone fully flat.
# ------------------------------------------------------------------
COARSE_STEP_US     = 50
FINE_STEP_US       = 5
REFERENCE_STEPS    = 5      # initial steps used to establish each scan's own baseline rate
WEAKENING_FRACTION = 0.35   # a step counts as "weak" below this fraction of the reference rate
RATE_WINDOW        = 1      # trigger on the first weak step, not a sustained run -- see scan_until_weak()'s own comment
MARGIN_US          = 100    # fixed gap before the fine pass starts, decoupled from COARSE_STEP_US -- the real transition
                             # zone measured only ~20us wide (350->330us), well under one 50us coarse step, so a coarse
                             # step can straddle the whole thing and register as unusually LARGE, not weak; backing up
                             # only "one coarse step" isn't reliably enough margin to land back in normal territory


def scan_until_weak(link, start_us, step_us, direction,
                     reference_steps=REFERENCE_STEPS, weakening_fraction=WEAKENING_FRACTION,
                     rate_window=RATE_WINDOW, floor_us=ABS_FLOOR_US, ceil_us=ABS_CEIL_US):
    """Steps from start_us in `direction` by step_us. Once `reference_steps`
    steps have been taken, their average delta becomes this scan's
    baseline rate; every step after that is compared against it. The
    first step whose delta falls below `weakening_fraction` of the
    baseline (sustained for `rate_window` consecutive steps, default 1 --
    real data showed the weak step immediately followed by the big
    transition itself, so a longer run would miss it) ends the scan.

    Returns ((edge_pulse, edge_centideg), trace, reference_rate). The
    edge reported is the LAST point reached via a normal-strength step --
    one point *before* the step that revealed the weakness, since that
    step's own destination is already on the far side of it. reference_rate
    is in centidegrees/step -- a ratio against it (weakening_fraction) is
    what actually drives detection, so no unit conversion needed here.

    Raises RuntimeError if the hard pulse bound is hit first -- a real
    anomaly (bad wiring/power, or the bound too tight for this servo),
    not something to silently continue past.
    """
    trace = []
    pulse = start_us
    _, prev_centideg = link.move_to(pulse)
    trace.append((pulse, prev_centideg))

    reference_deltas = []
    reference_rate = None
    weak_streak = 0

    while True:
        pulse += direction * step_us
        if pulse < floor_us or pulse > ceil_us:
            raise RuntimeError(f"hit hard bound ({floor_us}..{ceil_us}) without finding the limit")

        _, centideg = link.move_to(pulse)
        delta = abs(centideg - prev_centideg)   # plain subtraction -- already unwrapped
        trace.append((pulse, centideg))

        if reference_rate is None:
            reference_deltas.append(delta)
            if len(reference_deltas) >= reference_steps:
                reference_rate = sum(reference_deltas) / len(reference_deltas)
        elif delta < weakening_fraction * reference_rate:
            weak_streak += 1
            if weak_streak >= rate_window:
                edge_index = len(trace) - rate_window - 1   # one point before the run of weak steps started
                return trace[edge_index], trace, reference_rate
        else:
            weak_streak = 0

        prev_centideg = centideg


def find_edge(link, center_us, direction):
    """Coarse-then-fine rate-based scan in one direction from center_us.
    Returns ((pulse, centideg), coarse_trace, fine_trace, coarse_rate, fine_rate)."""
    (coarse_pulse, _), coarse_trace, coarse_rate = scan_until_weak(
        link, center_us, COARSE_STEP_US, direction
    )

    # Fixed margin, not "one coarse step" -- see MARGIN_US's own comment.
    fine_start = coarse_pulse - direction * MARGIN_US
    fine_edge, fine_trace, fine_rate = scan_until_weak(
        link, fine_start, FINE_STEP_US, direction
    )
    return fine_edge, coarse_trace, fine_trace, coarse_rate, fine_rate


def find_range(link, center_us=CENTER_US):
    """Finds both mechanical endpoints outward from center_us. Returns a
    dict with both edges (pulse + settled centideg), every coarse/fine
    trace, and each scan's own measured reference rate."""
    link.move_to(center_us)   # known starting point before scanning either direction
    (max_pulse, max_centideg), high_coarse, high_fine, high_coarse_rate, high_fine_rate = find_edge(link, center_us, +1)

    link.move_to(center_us)   # back to center before scanning the other way
    (min_pulse, min_centideg), low_coarse, low_fine, low_coarse_rate, low_fine_rate = find_edge(link, center_us, -1)

    return {
        "min_pulse_us": min_pulse, "min_centideg": min_centideg,
        "max_pulse_us": max_pulse, "max_centideg": max_centideg,
        "high_coarse_trace": high_coarse, "high_fine_trace": high_fine,
        "low_coarse_trace": low_coarse, "low_fine_trace": low_fine,
        "high_coarse_rate": high_coarse_rate, "high_fine_rate": high_fine_rate,
        "low_coarse_rate": low_coarse_rate, "low_fine_rate": low_fine_rate,
    }


def main():
    parser = argparse.ArgumentParser(description="ServoDAQ host driver")
    parser.add_argument("port", help="serial port, e.g. COM9")
    args = parser.parse_args()

    with ServoDAQLink(args.port) as link:
        print(f"connecting to {args.port}...")
        if not link.wait_ready():
            print("warning: no ready banner seen -- board may already be running")

        link.ping()
        print("PING ok")

        print("finding pulse range...")
        result = find_range(link)
        print(
            f"range: {result['min_pulse_us']}us ({result['min_centideg']} centideg) "
            f"to {result['max_pulse_us']}us ({result['max_centideg']} centideg)"
        )
        print(
            f"reference rates -- low: coarse {result['low_coarse_rate']:.1f}, fine {result['low_fine_rate']:.1f} | "
            f"high: coarse {result['high_coarse_rate']:.1f}, fine {result['high_fine_rate']:.1f} (centideg/step)"
        )


if __name__ == "__main__":
    main()
