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


class NotSettledError(RuntimeError):
    """US reported ERR NOT_SETTLED: SETTLE_TIMEOUT_MS elapsed without the
    shaft ever meeting the settle criterion. centideg here is just
    whatever the last raw reading was, not a real position -- some
    servos (see servo_daq.py's own history) take an unreachable pulse as
    a cue to spin toward it the long way around instead of stalling, so
    this pulse's own reading is never trustworthy. Carries pulse_us and
    centideg (offset-corrected, same as move_to()'s return) purely for
    diagnostics -- callers should not treat it as a real position."""
    def __init__(self, pulse_us, centideg):
        super().__init__(f"US {pulse_us} did not settle within the timeout (last reading {centideg} centideg)")
        self.pulse_us = pulse_us
        self.centideg = centideg


class ServoDAQLink:
    """One serial connection to the board. Opening the port resets it
    (DTR, same as any Arduino serial terminal), so connect() waits for
    the '# READY' boot banner before anything else is sent.
    """

    def __init__(self, port, baud=115200, timeout=3.0):
        self.ser = serial.Serial(port, baud, timeout=timeout)
        # See recover_from_wrap() -- corrects every centideg this link
        # reports from here on, after a NotSettledError episode moved the
        # shaft somewhere the wrap-safe accumulator didn't expect. Zero
        # (a no-op) until that ever actually happens.
        self.centideg_offset = 0

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
        see this module's docstring -- plus centideg_offset (see
        recover_from_wrap()), transparently, so every caller downstream
        of a wrap-recovery just sees corrected values with no special
        handling of their own. Timeout here (5s) is deliberately above
        the firmware's own SETTLE_TIMEOUT_MS (3s) -- a client timeout
        shorter than the device's own worst case is exactly the bug that
        caused a real command-desync problem elsewhere in this project's
        history, not repeating that here.

        Raises NotSettledError (not a plain RuntimeError) specifically
        for ERR NOT_SETTLED -- callers that need to treat "never settled"
        as a real, expected outcome (not a wire-protocol failure) can
        catch that distinctly; anything else not starting with OK
        (OUT_OF_RANGE, USAGE, BUSY) is a genuine protocol-level error and
        still raises plain RuntimeError."""
        reply = self.send_command(f"US {pulse_us}", timeout=5.0)
        parts = reply.split()
        if parts[0] == "ERR" and len(parts) > 1 and parts[1] == "NOT_SETTLED":
            raise NotSettledError(int(parts[2]), int(parts[3]) + self.centideg_offset)
        if parts[0] != "OK":
            raise RuntimeError(f"US {pulse_us} rejected: {reply}")
        return int(parts[1]), int(parts[2]) + self.centideg_offset

    def recover_from_wrap(self, candidates):
        """Call right after a NotSettledError to recenter onto a pulse
        whose true position is already known, and correct
        centideg_offset so every reading for the rest of this session is
        back in a physically consistent frame.

        `candidates` is an ordered list of (pulse_us, known_centideg)
        fallback points, tried nearest/most-specific first. One point
        isn't enough: real hardware showed the recovery move itself can
        also hit NotSettledError -- the danger zone's edge isn't
        perfectly repeatable run to run (observed anywhere from 250 to
        270us across trials), so "the last good pulse from this walk"
        isn't always reliably safe to return to on the very next
        command. Each caller should pass at least one further-back point
        it's confident is genuinely clear of the edge (e.g. this scan's
        own starting pulse) as a fallback beyond the immediate one.
        Raises the last NotSettledError if every candidate fails --
        at that point something more serious is going on than this
        function's job to paper over.

        Deliberately does NOT assume the excursion was one clean 360deg
        lap -- real measurements on hardware that actually does this
        (spins toward an unreachable target instead of stalling) showed
        excursions of very different, non-360-multiple sizes each time,
        consistent with an unstable "confused" drive state rather than a
        single deliberate rotation. So the correction is measured, not
        assumed: move to a pulse whose real position we already trust,
        see what the (still-uncorrected-for-this-event) tracker now
        reports there, and fold the exact difference into
        centideg_offset. self-correcting regardless of how large or
        irregular the real excursion turned out to be, and costs nothing
        extra -- recovering onto a known-good pulse is something the
        caller needs to do anyway."""
        last_err = None
        for known_pulse_us, known_centideg in candidates:
            try:
                pulse_us, reported = self.move_to(known_pulse_us)
            except NotSettledError as e:
                last_err = e
                continue
            drift = reported - known_centideg
            self.centideg_offset -= drift
            return pulse_us, known_centideg
        raise last_err

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
    old raw-count threshold had. Returns ((pulse, centideg), full_trace).

    A NotSettledError from move_to() is treated as reaching the edge, not
    a real stall -- some servos take a pulse past their real limit as a
    cue to spin toward it the long way around instead of stalling
    (confirmed on real hardware: the response stays normal, no gradual
    weakening, right up to and including the very last good step -- see
    servo_daq.py's own history -- so this can't be predicted in advance
    by any threshold on the *previous* step's delta; catching the actual
    failure is the only reliable signal). The failing pulse's own
    reading is never trusted -- the edge is the last point actually
    reached normally -- and link.recover_from_wrap() corrects the
    position reference before returning, so every reading after this
    call (including whatever the caller does next in the same session)
    is back in a physically consistent frame."""
    trace = []
    pulse = start_us
    _, prev_centideg = link.move_to(pulse)
    trace.append((pulse, prev_centideg))

    while True:
        pulse += direction * step_us
        if pulse < floor_us or pulse > ceil_us:
            raise RuntimeError(f"hit hard bound ({floor_us}..{ceil_us}) without detecting a stall")
        try:
            _, centideg = link.move_to(pulse)
        except NotSettledError:
            edge_pulse = pulse - direction * step_us   # last point actually reached normally
            # edge_pulse first (nearest, likely still correct), start_us
            # as a further-back fallback in case even edge_pulse isn't
            # safe to return to right now -- see recover_from_wrap()'s
            # own docstring for why one candidate isn't always enough.
            link.recover_from_wrap([(edge_pulse, prev_centideg), (trace[0][0], trace[0][1])])
            return (edge_pulse, prev_centideg), trace
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
WEAKENING_FRACTION = 0.35   # a step counts as "weak" below this fraction of the reference rate;
                             # also the noise floor for the reversal check below (see scan_until_weak())
RATE_WINDOW        = 1      # trigger on the first weak/reversed step, not a sustained run -- see scan_until_weak()'s own comment
MARGIN_US          = 100    # fixed gap before the fine pass starts, decoupled from COARSE_STEP_US -- the real transition
                             # zone measured only ~20us wide (350->330us), well under one 50us coarse step, so a coarse
                             # step can straddle the whole thing and register as unusually LARGE, not weak; backing up
                             # only "one coarse step" isn't reliably enough margin to land back in normal territory


def scan_until_weak(link, start_us, step_us, direction,
                     reference_steps=REFERENCE_STEPS, weakening_fraction=WEAKENING_FRACTION,
                     rate_window=RATE_WINDOW, floor_us=ABS_FLOOR_US, ceil_us=ABS_CEIL_US):
    """Steps from start_us in `direction` by step_us. Once `reference_steps`
    steps have been taken, their average delta becomes this scan's
    baseline rate; every step after that is compared against it. The scan
    ends (sustained for `rate_window` consecutive steps, default 1 -- real
    data showed the weak/reversed step immediately followed by the big
    transition itself, so a longer run would miss it) as soon as a step is
    either:
      - weak: |delta| below `weakening_fraction` of the reference rate, or
      - reversed: delta's sign contradicts `direction`, and it's not just
        noise (same weakening_fraction floor, so an ordinary +-1 count
        jitter while genuinely stopped never counts).
    A real servo response only ever continues (or stalls) in the commanded
    direction; a non-trivial reversal means something else happened -- a
    digital servo's own internal stall-protection backoff moving the shaft
    back toward safety, observed directly on real hardware (see
    full_test_record.py's investigation) always as a reversal, never a
    same-direction overshoot, which is exactly what "backing off from an
    over-extension" should look like. Both checks are purely relative to
    the reference rate this scan measured for itself -- no fixed absolute
    degree/centideg number.

    Returns ((edge_pulse, edge_centideg), trace, reference_rate). The edge
    reported is the LAST point reached via a normal step -- one point
    *before* the step that revealed weakness/reversal, since that step's
    own destination is already on the far side of it. Treating a reversal
    exactly like reaching the edge (rather than raising) means find_edge()
    recovers from it automatically via its existing fine-pass fallback --
    no special handling needed by callers.

    Raises RuntimeError only if the hard pulse bound is hit first -- a
    real anomaly (bad wiring/power, or the bound too tight for this
    servo), not something to silently continue past.

    A NotSettledError from move_to() ends the scan the same way a
    weak/reversed step does -- report the last point reached normally as
    the edge -- but unconditionally, not gated behind rate_window or the
    weakening_fraction threshold: a step that never settled isn't a rate
    heuristic, it's confirmed proof this pulse isn't real position data
    (some servos spin toward an unreachable target instead of stalling;
    real hardware shows zero gradual precursor in the step *before* this
    one, so no threshold on delta could ever catch it in advance -- see
    servo_daq.py's own history). link.recover_from_wrap() corrects the
    position reference onto the trusted last-good point before returning,
    so a caller (e.g. find_edge()'s subsequent fine pass, or whatever
    runs after this scan in the same session) sees consistent readings
    afterward, not a phantom offset baked into everything.
    """
    trace = []
    pulse = start_us
    _, prev_centideg = link.move_to(pulse)
    trace.append((pulse, prev_centideg))

    reference_deltas = []
    reference_rate = None
    end_streak = 0

    while True:
        pulse += direction * step_us
        if pulse < floor_us or pulse > ceil_us:
            raise RuntimeError(f"hit hard bound ({floor_us}..{ceil_us}) without finding the limit")

        try:
            _, centideg = link.move_to(pulse)
        except NotSettledError:
            # trace[-1] first (nearest, likely still correct), trace[0]
            # (this scan's own starting pulse) as a further-back fallback
            # -- see recover_from_wrap()'s own docstring for why.
            link.recover_from_wrap([trace[-1], trace[0]])
            return trace[-1], trace, (reference_rate if reference_rate is not None else 0.0)
        signed_delta = centideg - prev_centideg   # plain subtraction -- already unwrapped
        delta = abs(signed_delta)
        trace.append((pulse, centideg))

        if reference_rate is None:
            reference_deltas.append(delta)
            if len(reference_deltas) >= reference_steps:
                reference_rate = sum(reference_deltas) / len(reference_deltas)
        else:
            weak = delta < weakening_fraction * reference_rate
            reversed_ = (not weak) and (signed_delta > 0) != (direction > 0)
            if weak or reversed_:
                end_streak += 1
                if end_streak >= rate_window:
                    edge_index = len(trace) - rate_window - 1   # one point before the run started
                    return trace[edge_index], trace, reference_rate
            else:
                end_streak = 0

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
