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


# ------------------------------------------------------------------
# Shared edge-detection constants, used by both naive_stall_sweep() and
# scan_until_weak(). Three independent, deliberately different ways a
# step can mean "found the edge" -- matching three different real
# failure modes seen on real hardware, none of which reliably implies
# the others:
#   1. stall     -- the response goes flat (a genuine mechanical/
#                    electrical limit).
#   2. reversal   -- the very next step goes backward relative to the
#                    commanded direction (e.g. a digital servo's own
#                    internal stall-protection backing off).
#   3. big jump   -- the next step is a real, same-direction settle,
#                    but far larger than the established local rate --
#                    not incremental motion, the servo losing its
#                    mechanical reference and free-spinning (or the
#                    step immediately before a NotSettledError would
#                    have fired, for a method that isn't checking for
#                    that directly). The step BEFORE the jump is the
#                    edge, not the jump's own destination.
# A NotSettledError (case 3 taken to its extreme -- never even settling)
# is handled separately in both functions; these three are for the
# "every reply came back OK" case.
# ------------------------------------------------------------------
REFERENCE_STEPS   = 5     # initial steps used to establish each scan's own baseline rate
BIG_JUMP_MULTIPLE = 3.0   # a same-direction step this many times the reference rate is
                          # not real incremental motion


def naive_stall_sweep(link, start_us, step_us, direction, stall_threshold=round(CENTIDEG_PER_COUNT),
                       reference_steps=REFERENCE_STEPS, big_jump_multiple=BIG_JUMP_MULTIPLE,
                       floor_us=ABS_FLOOR_US, ceil_us=ABS_CEIL_US):
    """The dead-simple baseline for comparison against find_range(): step
    by step_us, compare each new reading only to the immediately previous
    one. Stops the first time a step shows any of the three signatures
    documented above this function (see that comment for the reasoning):

      1. stall: |delta| <= stall_threshold. Reports the point where the
         stall was observed (not one step back -- unlike find_range()'s
         convention, this case has no notion of "the step before," it
         just walks until the encoder stalls and calls that the limit).
         stall_threshold is in centidegrees; the default (~1 raw AS5600
         count's worth) preserves the same real angular sensitivity the
         old raw-count threshold had.
      2. reversal: delta's sign contradicts `direction`. Reports the
         PREVIOUS point -- the reversed step's own destination is
         already on the far side of the real edge, not trustworthy.
      3. big jump: once `reference_steps` normal steps have established
         a baseline rate, |delta| more than `big_jump_multiple` times
         that rate is not real incremental motion. Reports the PREVIOUS
         point, same reasoning as reversal.

    A NotSettledError from move_to() is treated as reaching the edge too
    -- the extreme case where the servo doesn't even settle at all, not
    just settles somewhere unexpected (confirmed on real hardware: the
    response stays normal, no gradual weakening, right up to and
    including the very last good step -- see servo_daq.py's own history
    -- so this can't be predicted in advance by any threshold on the
    *previous* step's delta; catching the actual failure is the only
    reliable signal there). The failing pulse's own reading is never
    trusted -- the edge is the last point actually reached normally --
    and link.recover_from_wrap() corrects the position reference before
    returning, so every reading after this call (including whatever the
    caller does next in the same session) is back in a physically
    consistent frame.

    Returns ((pulse, centideg), full_trace, hit_not_settled) -- the third
    value is True exactly when a NotSettledError happened during this
    call, so a caller can flag the unit as one where sitting at/near
    this edge is worth staying further away from later (the real run
    this was built for: every accuracy-test trial immediately after one
    that landed at the established edge came back roughly a full lap
    off, even though the edge trial itself showed no other sign of
    distress -- see study_range.py's own history)."""
    trace = []
    pulse = start_us
    link.move_to(pulse)   # first call covers the jump into start_us, which can be large
                           # (e.g. straight from the opposite edge) -- the firmware's own
                           # settle check can report "settled" before the shaft has actually
                           # finished a long transient (real hardware: an 1170us jump reported
                           # settled 8 centideg short of a fine sweep's own ~100 centideg
                           # reading across the same interval -- see study_range.py's own
                           # history). A second call to the SAME pulse has no further distance
                           # to cover, so it can only either confirm genuine rest (near-identical
                           # reading) or catch the still-drifting tail the first call missed --
                           # its reading is the one actually trusted as the baseline.
    _, prev_centideg = link.move_to(pulse)
    trace.append((pulse, prev_centideg))

    reference_deltas = []
    reference_rate = None
    first_step = True

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
            return (edge_pulse, prev_centideg), trace, True

        signed_delta = centideg - prev_centideg   # plain subtraction -- already unwrapped
        delta = abs(signed_delta)

        if first_step:
            # The step right off a standing start reliably reads weaker
            # than this servo's real cruise rate -- real hardware: naive-
            # low's own first step measured 17 centideg against a typical
            # 50-150 centideg for every step after it; naive-high's landed
            # at 8 centideg, on the wrong side of stall_threshold purely by
            # chance (a second move_to() to the very same pulse read
            # identically, ruling out a sensing/settling glitch -- this is
            # a real stiction/breakaway-torque effect). Judging it the same
            # as every other step makes this "dead simple" baseline more
            # conservative than intended, not more naive. Recorded like
            # normal, just never allowed to end the scan or feed the
            # reference-rate baseline.
            trace.append((pulse, centideg))
            prev_centideg = centideg
            first_step = False
            continue

        # Case 2: reversal -- checked before case 1, a reversed step is
        # never also a stall (stall_threshold is small; a real reversal
        # is a real, non-trivial move in the wrong direction).
        if (signed_delta > 0) != (direction > 0) and delta > stall_threshold:
            trace.append((pulse, centideg))
            return (pulse - direction * step_us, prev_centideg), trace, False

        # Case 3: big same-direction jump, once a baseline is established.
        if reference_rate is not None and delta > big_jump_multiple * reference_rate:
            trace.append((pulse, centideg))
            return (pulse - direction * step_us, prev_centideg), trace, False

        trace.append((pulse, centideg))

        # Case 1: stall.
        if delta <= stall_threshold:
            return (pulse, centideg), trace, False

        if reference_rate is None:
            reference_deltas.append(delta)
            if len(reference_deltas) >= reference_steps:
                reference_rate = sum(reference_deltas) / len(reference_deltas)

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
# REFERENCE_STEPS and BIG_JUMP_MULTIPLE are shared with naive_stall_sweep() --
# defined once, above that function.
WEAKENING_FRACTION = 0.35   # a step counts as "weak" below this fraction of the reference rate;
                             # also the noise floor for the reversal check below (see scan_until_weak())
RATE_WINDOW        = 1      # trigger on the first weak/reversed step, not a sustained run -- see scan_until_weak()'s own comment
EDGE_BACKOFF_STEPS = 1       # how many good steps before the weak/reversed one to report as "the edge" --
                             # find_edge() asks for more than this on the fine pass specifically (real data:
                             # a fine-pass edge reported only 1 step, 5us, off the real danger zone left an
                             # accuracy-test target that landed 1us past it; see study_range.py's own history)
MARGIN_US          = 100    # fixed gap before the fine pass starts, decoupled from COARSE_STEP_US -- the real transition
                             # zone measured only ~20us wide (350->330us), well under one 50us coarse step, so a coarse
                             # step can straddle the whole thing and register as unusually LARGE, not weak; backing up
                             # only "one coarse step" isn't reliably enough margin to land back in normal territory


def scan_until_weak(link, start_us, step_us, direction,
                     reference_steps=REFERENCE_STEPS, weakening_fraction=WEAKENING_FRACTION,
                     rate_window=RATE_WINDOW, edge_backoff_steps=EDGE_BACKOFF_STEPS,
                     big_jump_multiple=BIG_JUMP_MULTIPLE,
                     floor_us=ABS_FLOOR_US, ceil_us=ABS_CEIL_US):
    """Steps from start_us in `direction` by step_us. Once `reference_steps`
    steps have been taken, their average delta becomes this scan's
    baseline rate; every step after that is compared against it. The scan
    ends (sustained for `rate_window` consecutive steps, default 1 -- real
    data showed the weak/reversed step immediately followed by the big
    transition itself, so a longer run would miss it) as soon as a step is
    any of:
      - weak: |delta| below `weakening_fraction` of the reference rate, or
      - reversed: delta's sign contradicts `direction`, and it's not just
        noise (same weakening_fraction floor, so an ordinary +-1 count
        jitter while genuinely stopped never counts), or
      - a big jump: |delta| above `big_jump_multiple` times the reference
        rate -- a real, same-direction settle, just far larger than
        incremental motion (the servo losing its mechanical reference and
        free-spinning, rather than gradually weakening or reversing). This
        is the case that let naive_stall_sweep() run all the way to the
        absolute pulse floor without ever detecting a stall on real
        hardware -- see servo_daq.py's own history.
    A real servo response only ever continues (or stalls) in the commanded
    direction, at roughly its established rate; a non-trivial reversal
    means something else happened -- a digital servo's own internal
    stall-protection backoff moving the shaft back toward safety, observed
    directly on real hardware (see full_test_record.py's investigation)
    always as a reversal, never a same-direction overshoot, which is
    exactly what "backing off from an over-extension" should look like.
    All three checks are purely relative to the reference rate this scan
    measured for itself -- no fixed absolute degree/centideg number.

    The edge reported is `edge_backoff_steps` good steps before the one
    that revealed weakness/reversal (default 1 -- immediately before it),
    not necessarily the very last good point: real hardware showed 1
    step (5us on the fine pass) wasn't always enough margin -- an
    accuracy-test target landing just past a fine-pass edge reported
    that way triggered the same spin-the-long-way behavior on a later,
    unrelated move (see study_range.py's own history). Treating a
    reversal exactly like reaching the edge (rather than raising) means
    find_edge() recovers from it automatically via its existing fine-pass
    fallback -- no special handling needed by callers.

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

    Returns ((edge_pulse, edge_centideg), trace, reference_rate,
    hit_not_settled) -- hit_not_settled is True exactly when a
    NotSettledError happened during this call. See naive_stall_sweep()'s
    own docstring for what a caller should do with it (find_edge()/
    find_range() just propagate it up, study_range.py acts on it).
    """
    trace = []
    pulse = start_us
    link.move_to(pulse)   # first call covers the (possibly large) jump into start_us --
                           # second call re-confirms settle at the same pulse before its
                           # reading is trusted. See naive_stall_sweep()'s own comment for why.
    _, prev_centideg = link.move_to(pulse)
    trace.append((pulse, prev_centideg))

    reference_deltas = []
    reference_rate = None
    end_streak = 0
    first_step = True

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
            return trace[-1], trace, (reference_rate if reference_rate is not None else 0.0), True
        signed_delta = centideg - prev_centideg   # plain subtraction -- already unwrapped
        delta = abs(signed_delta)
        trace.append((pulse, centideg))

        if first_step:
            # The step right off a standing start reliably reads weaker
            # than this servo's real cruise rate -- see naive_stall_sweep()'s
            # own comment for the real-hardware evidence. Recorded like any
            # other step, just excluded from the reference-rate average so
            # it doesn't drag REFERENCE_STEPS' worth of baseline down by one
            # systematically-low sample.
            first_step = False
            prev_centideg = centideg
            continue

        if reference_rate is None:
            reference_deltas.append(delta)
            if len(reference_deltas) >= reference_steps:
                reference_rate = sum(reference_deltas) / len(reference_deltas)
        else:
            weak = delta < weakening_fraction * reference_rate
            reversed_ = (not weak) and (signed_delta > 0) != (direction > 0)
            big_jump = (not weak) and (not reversed_) and (delta > big_jump_multiple * reference_rate)
            if weak or reversed_ or big_jump:
                end_streak += 1
                if end_streak >= rate_window:
                    edge_index = len(trace) - rate_window - edge_backoff_steps
                    return trace[edge_index], trace, reference_rate, False
            else:
                end_streak = 0

        prev_centideg = centideg


FINE_EDGE_BACKOFF_STEPS = 2   # extra margin on the fine pass's own reported edge specifically --
                              # real hardware showed the default 1 step (5us) wasn't always enough:
                              # an accuracy-test target landing just past a fine-pass edge reported
                              # that way triggered a spin-the-long-way event on a later, unrelated
                              # move (see study_range.py's own history). The coarse pass keeps the
                              # default 1-step backoff -- MARGIN_US already gives the fine pass a
                              # generous 100us head start, this is about the fine pass's own final
                              # answer, not the coarse pass feeding into it.


def find_edge(link, center_us, direction):
    """Coarse-then-fine rate-based scan in one direction from center_us.
    Returns ((pulse, centideg), coarse_trace, fine_trace, coarse_rate,
    fine_rate, hit_not_settled) -- hit_not_settled is True if either the
    coarse or the fine pass needed a NotSettledError recovery (see
    scan_until_weak()'s own docstring)."""
    (coarse_pulse, _), coarse_trace, coarse_rate, coarse_hit = scan_until_weak(
        link, center_us, COARSE_STEP_US, direction
    )

    # Fixed margin, not "one coarse step" -- see MARGIN_US's own comment.
    fine_start = coarse_pulse - direction * MARGIN_US
    fine_edge, fine_trace, fine_rate, fine_hit = scan_until_weak(
        link, fine_start, FINE_STEP_US, direction, edge_backoff_steps=FINE_EDGE_BACKOFF_STEPS
    )
    return fine_edge, coarse_trace, fine_trace, coarse_rate, fine_rate, (coarse_hit or fine_hit)


def find_range(link, center_us=CENTER_US):
    """Finds both mechanical endpoints outward from center_us. Returns a
    dict with both edges (pulse + settled centideg), every coarse/fine
    trace, each scan's own measured reference rate, and whether either
    side ever needed a NotSettledError recovery (see find_edge()'s own
    docstring) -- study_range.py uses that to decide whether this unit's
    accuracy test should keep its random targets away from the exact
    edge."""
    link.move_to(center_us)   # known starting point before scanning either direction
    (max_pulse, max_centideg), high_coarse, high_fine, high_coarse_rate, high_fine_rate, high_hit = find_edge(link, center_us, +1)

    link.move_to(center_us)   # back to center before scanning the other way
    (min_pulse, min_centideg), low_coarse, low_fine, low_coarse_rate, low_fine_rate, low_hit = find_edge(link, center_us, -1)

    return {
        "min_pulse_us": min_pulse, "min_centideg": min_centideg,
        "max_pulse_us": max_pulse, "max_centideg": max_centideg,
        "high_coarse_trace": high_coarse, "high_fine_trace": high_fine,
        "low_coarse_trace": low_coarse, "low_fine_trace": low_fine,
        "high_coarse_rate": high_coarse_rate, "high_fine_rate": high_fine_rate,
        "low_coarse_rate": low_coarse_rate, "low_fine_rate": low_fine_rate,
        "high_hit_not_settled": high_hit, "low_hit_not_settled": low_hit,
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
        if result['low_hit_not_settled'] or result['high_hit_not_settled']:
            print(
                f"note: needed a NotSettledError recovery -- low={result['low_hit_not_settled']}, "
                f"high={result['high_hit_not_settled']} (see servo_daq.py's own history)"
            )


if __name__ == "__main__":
    main()
