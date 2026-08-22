/*
  ServoCalibrator_Companion

  This IS the ServoCalibrator.html companion firmware -- as of 2026-08-21,
  the version living here is the FSM redesign originally developed under
  a separate `ServoAutoCalibrator/` folder (see CLAUDE.md's own history
  for the full design story: active spin-recovery, coarse+fine stall-scan
  with real-hardware-driven fixes). It replaces the earlier, structurally
  different companion firmware that used to live at this path (a
  host-paced, mostly-blocking executor with IMPORT/EXPORT/GETTABLE and
  continuous SQUARE/SINE generators -- still recoverable from git history
  if ever needed as reference, but the FSM design is the one going
  forward per explicit direction: "this is what I want to be the final"
  version of this tool).

  On-device servo range-finding/calibration, as a finite state machine --
  coarse+fine stall-scan calibration (with active spin-recovery), then a
  single point-to-point trapezoidal trajectory move on demand (CMD_GO,
  no continuous square-wave/sine-wave generator -- this firmware
  deliberately only ever streams one planned move at a time, unlike the
  SQUARE/SINE generators the older companion firmware here used to have).
  Every tick, regardless of state, streams a TELEM line (see
  printTelemetry()) reporting both the trajectory's current target and
  the actual measured shaft state, for ServoCalibrator.html to plot live.

  Runs on one fixed-rate tick (see TICK_INTERVAL_MS below); the state
  machine itself is a single switch/case in loop(), one case per enum
  value in CalibrationState. The housekeeping that runs ahead of that
  switch every tick -- reading the AS5600, checking for an incoming
  serial command that might reject outright or force a state change
  (e.g. ABORT) before that tick's state logic runs, and streaming
  telemetry -- happens the same way regardless of which state is active.

  Real protocol differences from the old companion firmware that
  ServoCalibrator.html has to account for (see that file's own comments
  for how): CAL acknowledges immediately and streams progress
  asynchronously rather than blocking on one final reply; GETTABLE
  re-streams the current session's already-built calTable on request but
  there's still no way to recover one across a reconnect (opening the
  port always reboots the board via DTR, wiping calTable, so a fresh CAL
  is still required every session) and no IMPORT at all; GO takes its
  target/limits from separate ACCEL/VEL/POS commands rather than one
  combined call; and calTable's angleCentideg values are raw encoder
  readings relative to wherever the board happened to boot, NOT
  re-anchored so index 0 reads exactly 0 the way the old firmware's table
  was.
*/

#define LOOP_FREQUENCY_HZ 50    // fixed tick rate -- every timing constant below derives from this
#define SERVO_PIN A3            // matches ServoDAQ_Companion's wiring
// Passed to servo.attach() below -- matches ServoDAQ_Companion's own
// ABS_FLOOR_US/ABS_CEIL_US. Without these, Servo::attach(pin) defaults to
// the library's own MIN_PULSE_WIDTH/MAX_PULSE_WIDTH (544/2400) and
// writeMicroseconds() silently clamps every commanded pulse to that range
// -- found on real hardware: every pulse below ~544us was silently
// rewritten to 544us, so the servo never actually received them. Looked
// exactly like the servo physically freezing below ~540us (coarse stall,
// a fully flat 100us fine-margin sweep, and an unconditional diagnostic
// sweep all reading the same encoder position from 540 down to 250) --
// it was never physical at all, and running ServoDAQ_Companion (whose
// attach() already passes
// these) against the same servo the same session proved it: smooth,
// unremarkable motion the whole way down to ~260-330us.
#define ABS_FLOOR_US 80
#define ABS_CEIL_US 3100
#define CENTER_US 1500          // pulse commanded at the start of calibration
#define CAL_SETTLE_TIMEOUT_MS 3000   // exit route 1 of every waiting state: give up if it never settles
#define CAL_SETTLE_WINDOW_SAMPLES 10 // N -- how many recent raw readings the running average is over (to tune later)
#define CAL_SETTLE_WINDOW_COUNTS 2   // max |current raw - running average| to call it settled (to tune later)
// M -- how many recent raw readings filteredPosition is averaged over.
// NOT what TELEM reports anymore (see printTelemetry() -- that now
// sends the raw, unfiltered position directly, per explicit direction).
// filteredPosition itself still matters for real functional uses this
// value tunes the responsiveness of: the calibration table's own
// angleCentideg values (recordTableEntry()), a GO move's planned
// starting point (currentAngleDeg()), and the calibration scan's own
// edge-detection (judgeScanStep()). Was 10 (200ms at this 50Hz tick),
// then 3 (60ms) once TELEM was still routing through it and read too
// smooth; now that TELEM bypasses it entirely, bumped back up slightly
// to 5 (100ms) -- a little more settling margin for the uses that still
// need it, now that doing so no longer costs the live trace anything.
#define CAL_POSITION_FILTER_SAMPLES 5

// STATE_CAL_DOWN_*/STATE_CAL_UP_* run both the coarse and fine passes --
// same states, same edge-detection logic either way, just a different
// step size and starting position (currentStepUs/currentPhase below), the
// same way the Python host's find_edge() reuses one scan_until_weak() for
// both. Coarse finds the edge's rough location fast; fine (once the phase
// switches, see beginFinePass()) backs up by CAL_FINE_MARGIN_US and
// re-scans slowly across just that margin for a precise result.
#define CAL_COARSE_STEP_US 50            // coarse-pass step size (to tune later)
#define CAL_FINE_STEP_US 5               // fine-pass step size (to tune later)
#define CAL_FINE_MARGIN_US 100           // how far back (toward center) from the reported coarse edge the fine
                                          // pass starts (to tune later)

// Edge-detection is a direct port of this project's own ServoDAQ host's
// scan_until_weak() (servo_daq.py) -- coarse and fine only, the naive
// comparison sweep that host script also has is a study-only baseline,
// never part of this firmware. Each of the four scans (coarse-down,
// fine-down, coarse-up, fine-up) self-calibrates its OWN fresh reference
// rate from its own first CAL_REFERENCE_STEPS real steps -- NOT one
// baseline shared across the whole run -- then watches every step after
// that for one of three signatures, all purely RELATIVE to that scan's
// own rate (no fixed absolute count/threshold):
//   - weak: |delta| below CAL_WEAKENING_FRACTION of the reference rate
//     -- the response is dying out, the edge is right here.
//   - reversed: delta's sign disagrees with stepDirection, past the same
//     weakening-fraction floor (so ordinary +-1-count jitter while
//     genuinely stopped never counts) -- a digital servo's own internal
//     stall-protection backing off, observed on real hardware always as
//     a reversal, never a same-direction overshoot.
//   - big jump: |delta| more than CAL_BIG_JUMP_MULTIPLE times the
//     reference rate -- a real, same-direction settle, just far larger
//     than incremental motion (the servo losing its mechanical
//     reference and free-spinning, not gradually weakening or reversing).
// See judgeScanStep() for the actual per-step logic.
#define CAL_REFERENCE_STEPS 5        // steps used to self-calibrate each scan's own baseline rate before any judgment starts
#define CAL_WEAKENING_FRACTION 0.35f // see "weak" above
#define CAL_BIG_JUMP_MULTIPLE 3.0f   // see "big jump" above

// How many good steps before the one that revealed weakness/reversal/a
// big jump gets reported as the edge -- not necessarily the very last
// good point. The fine pass gets more margin than coarse specifically:
// real hardware (this project's own ServoDAQ host) showed 1 fine step
// (5us) wasn't always enough -- an accuracy-test target landing just
// past a fine-pass edge reported that way triggered the same spin-the-
// long-way behavior on a later, unrelated move. MARGIN_US already gives
// the fine pass a generous 100us head start; this is about the fine
// pass's own final answer, not the coarse pass feeding into it.
#define CAL_EDGE_BACKOFF_STEPS 1        // coarse pass
#define CAL_FINE_EDGE_BACKOFF_STEPS 2   // fine pass

#define CAL_TABLE_POINTS 20   // number of evenly-spaced points, minUs..maxUs inclusive, recorded into calTable

// Trajectory defaults -- deg/sec, deg/sec^2, matching TrapezoidalProfile's
// (UTI's) own working unit for TrajectoryLimits.vMax/aMax. Everything from
// here down through the STATE_TRAJ_* states works in degrees, not pulse
// microseconds -- conversion to a pulse only happens once, at the last
// step (see pulseForAngleCentideg()), the same place calTable's own
// angleCentideg values get looked up.
#define CAL_DEFAULT_VEL_DEG_PER_SEC 80.0f
#define CAL_DEFAULT_ACCEL_DEG_PER_SEC2 300.0f

#include <Wire.h>
#include <AS5600.h>
#include <Servo.h>
#include <TrapezoidalProfile.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

const unsigned long TICK_INTERVAL_MS = 1000UL / LOOP_FREQUENCY_HZ;

const int LINE_BUF_LEN = 32;
char lineBuf[LINE_BUF_LEN];
int lineLen = 0;

AS5600 encoder;
Servo servo;

// All calibration states are named CAL_<routine>_<substate> -- STATE_CAL_
// DOWN_*/STATE_CAL_UP_* serve both the coarse and fine passes (see the
// CAL_COARSE_STEP_US/CAL_FINE_STEP_US block above for why there's no
// separate set of fine states). STATE_CAL_UP_CENTER sits between the down
// edge being found and the up sweep actually starting -- the up sweep
// always starts from CENTER_US, not from wherever the down sweep/fine
// pass left the servo, so both directions are scanned from the same
// starting point instead of the up sweep's first step being whatever
// distance happens to separate it from the down edge.
enum CalibrationState {
  STATE_IDLE,
  STATE_CAL_CENTER,
  STATE_CAL_DOWN_WRITE,
  STATE_CAL_DOWN_WAIT,
  STATE_CAL_RECOVER_WAIT,
  STATE_CAL_UP_CENTER,
  STATE_CAL_UP_WRITE,
  STATE_CAL_UP_WAIT,
  STATE_CAL_TABLE_WRITE,
  STATE_CAL_TABLE_WAIT,
  STATE_CAL_DONE,
  // Trajectory moves -- a separate routine from calibration (hence no
  // CAL_ prefix), gated on isCalibrated, entered via CMD_GO. See
  // pulseForAngleCentideg()/currentAngleDeg() for the degrees<->pulse
  // conversion these three lean on.
  STATE_TRAJ_PLAN,
  STATE_TRAJ_STREAM,
  STATE_TRAJ_WAIT,
  STATE_TRAJ_DONE,
  // On-demand table query -- yet another separate routine (hence no
  // CAL_/TRAJ_ prefix), gated on isCalibrated, entered via CMD_GETTABLE.
  // Just replays the current session's already-built calTable back over
  // the wire, one entry per tick -- no servo motion, no settling, so
  // it's not in isWaitingState()'s list and can't time out. See its own
  // case body in loop().
  STATE_GETTABLE_SEND,
};

// Which pass a STATE_CAL_DOWN_*/STATE_CAL_UP_* run is currently doing --
// decides both the step size (currentStepUs) and what an edge hit means
// (PHASE_COARSE: back up and keep going, in finer steps; PHASE_FINE: this
// is the real edge, record it and move on). See beginCoarsePass()/
// beginFinePass().
enum ScanPhase {
  PHASE_COARSE,
  PHASE_FINE,
};

// For isolating one stage of calibration at a time on real hardware
// (find_range()'s coarse pass didn't match ServoDAQ's own ground truth
// for this servo -- rather than guess at the whole pipeline, watch each
// stage on its own). STAGE_ALL is the normal, uninterrupted run; any
// other value makes the matching case body land on STATE_CAL_DONE
// instead of continuing to the next stage, once that one stage's own
// exit condition fires. STAGE_COARSE_DOWN/STAGE_COARSE_UP/STAGE_FINE_DOWN/
// STAGE_FINE_UP wired up so far -- more get added here as we get to
// testing them.
enum TestStage {
  STAGE_ALL,
  STAGE_COARSE_DOWN,
  STAGE_COARSE_UP,
  STAGE_FINE_DOWN,
  STAGE_FINE_UP,
};

// Lookup for incoming serial commands -- parseCommand() below maps a raw
// line to one of these, and handleLine() switches on the enum rather
// than re-comparing strings at every call site as more commands get added.
enum SerialCommand {
  CMD_UNKNOWN,
  CMD_PING,
  CMD_CAL,
  CMD_ABORT,
  CMD_ACCEL,
  CMD_VEL,
  CMD_POS,
  CMD_MODEL,
  CMD_GO,
  CMD_STOPAFTER,
  CMD_GETTABLE,
};

// Every error the firmware can raise, in one place -- throwError() below
// maps one of these to its wire string via errorCodeToString().
enum ErrorCode {
  ERR_CAL_TIMEOUT,       // a calibration wait (CENTER/DOWN/UP/TABLE) timed out
  ERR_TRAJ_TIMEOUT,      // STATE_TRAJ_WAIT timed out -- streamed setpoints but never settled
  ERR_RECOVERY_FAILED,   // STATE_CAL_RECOVER_WAIT ran out of candidates -- see its own comment
  ERR_CAL_ABS_BOUND,     // a scan hit ABS_FLOOR_US/ABS_CEIL_US without ever finding the edge --
                          // mirrors this project's ServoDAQ host's own explicit floor_us/ceil_us
                          // RuntimeError, a real fail-safe in case weak/reversed/big-jump detection
                          // somehow never triggers (e.g. an encoder fault), not just
                          // servo.attach()'s own silent PWM clamp
};

// One calibration table entry -- pulseUs/angleCentideg. Same idea as this
// project's sibling firmware's own CalPoint (ServoCalibrationTable.h), but
// NOT the same field width for angleCentideg: UMI's version is int16_t
// because it assumes a bounded physical servo range (well under 327.68deg
// in practice); this project tracks a continuous, unwrapped, multi-turn
// position (see updatePositionTracking()) with no such bound -- confirmed
// on real hardware to matter, not just theoretical: a servo spinning past
// its limit produced an angle over 327deg, silently wrapping negative in
// an int16_t field. int32_t (== long on AVR) has no realistic overflow
// case for any position this firmware could actually produce.
//
// Declared up here, not down by calTable/interpolateTable() where it's
// actually used, because Arduino's auto-generated function prototypes are
// inserted right after the #includes -- interpolateTable()'s CalPoint*
// parameter needs the type visible by that point, not just by the time
// its own definition is reached.
struct CalPoint {
  int16_t pulseUs;
  int32_t angleCentideg;
};

// ------------------------------------------------------------------
// Multi-turn position tracking -- mirrors ServoDAQ_Companion's
// updatePositionTracking() exactly (same algorithm, same reasoning; see
// that file's own header comment for the full writeup). The AS5600 only
// ever reports a raw 12-bit angle, 0-4095, wrapping every revolution --
// it has no concept of "which lap." A same-direction jump bigger than
// half a revolution (2048 counts) between two *consecutive* samples is
// therefore treated as a wrap, credited to a signed lap counter, rather
// than mistaken for real motion. Safe at this tick rate for the same
// reason it's safe there: nothing this firmware commands can complete a
// full revolution between two ticks 20ms apart -- confirmed true even
// for the failure mode that motivated this (a servo spinning
// uncontrolled past its limit instead of stalling, ~400 counts/20ms
// tick on real hardware -- an order of magnitude under the threshold).
//
// Before this, filteredPosition/settleBuffer/etc. all carried the raw
// wrapped value directly -- exactly the bug that let a real spin event
// land near 0 and get misread as a small, plausible position instead of
// the actual multi-thousand-count displacement it was.
//
// Placed here, after CalPoint/the enums rather than up by AS5600 encoder/
// Servo servo where it conceptually belongs, for the same auto-prototype
// reason CalPoint itself is up here: Arduino inserts every function's
// prototype right before the *first* function definition in the file --
// if updatePositionTracking() were that first function, every other
// function's prototype (which need CalibrationState/ErrorCode/CalPoint
// visible) would get hoisted above those types too.
const int WRAP_THRESHOLD = 2048;

int32_t turnCount = 0;
int prevRaw = 0;
long totalCounts = 0;

// The only place anywhere in this program that reads the encoder.
// Updates the lap counter and returns the new continuous total.
long updatePositionTracking() {
  int raw = encoder.readAngle();
  int naiveDelta = raw - prevRaw;
  if (naiveDelta < -WRAP_THRESHOLD)      turnCount++;   // wrapped forward, 4095->0
  else if (naiveDelta > WRAP_THRESHOLD)  turnCount--;   // wrapped backward, 0->4095
  prevRaw = raw;
  totalCounts = (long)turnCount * 4096L + raw;
  return totalCounts;
}

CalibrationState currentState = STATE_IDLE;
bool calibrationRunning = false;   // true from CMD_CAL until either the table finishes (see
                                    // STATE_CAL_TABLE_WAIT) or changeState() lands back on STATE_IDLE
                                    // (abort or timeout) -- handleLine() rejects every command except
                                    // ABORT while this is true
bool trajectoryRunning = false;    // same idea as calibrationRunning, for CMD_GO instead of CMD_CAL
bool tableSendRunning = false;     // same idea again, for CMD_GETTABLE instead of CMD_CAL/CMD_GO --
                                    // see STATE_GETTABLE_SEND

// Declared up here (ahead of where they conceptually belong, alongside
// resetScanState()/judgeScanStep() further down) purely so changeState()
// below can clear them on landing at STATE_IDLE -- see their own full
// comments at judgeScanStep()'s own declaration site for what each does.
// Without this, a flag left true by an aborted/timed-out run could
// silently exempt a genuinely real step from judgment on the *next* CAL
// run.
bool scanResetPending = false;
bool skipNextStepCheck = false;

bool justEnteredState = false;             // set by changeState(), consumed once at the top of the
                                            // switch each tick -- lets a case body run one-time entry
                                            // actions without touching currentState/stateEnteredMs itself
unsigned long lastTickMs = 0;
unsigned long tickCount = 0;    // incremented once per real tick, in loop() -- see printProgress().
                                 // A wall-clock timestamp alone can't tell "no ticks happened" apart
                                 // from "ticks happened but nothing changed"; consecutive tickCount
                                 // values let a stall be pinned to an exact tick, not just a rough time.
unsigned long stateEnteredMs = 0;          // millis() timestamp of the current state's entry tick
unsigned long elapsedStateTimeMs = 0;      // now - stateEnteredMs, recomputed every tick in the
                                            // housekeeping below -- states just compare this against
                                            // their own timeout, never touch stateEnteredMs directly

// The only place that ever writes currentState/stateEnteredMs/
// elapsedStateTimeMs/justEnteredState -- every transition, anywhere in the
// file, goes through here instead of a case body assigning currentState
// directly. Resets the elapsed-time tracking immediately (not on the next
// tick's recompute), so a case body that reads elapsedStateTimeMs right
// after calling this always sees 0, never a stale value.
void changeState(CalibrationState newState) {
  currentState = newState;
  stateEnteredMs = millis();
  elapsedStateTimeMs = 0;
  justEnteredState = true;
  if (newState == STATE_IDLE) {
    // Landing on STATE_IDLE always means neither a calibration, a
    // trajectory move, nor a table send is still in progress, regardless
    // of which path got us here (abort, timeout, or -- for
    // tableSendRunning specifically -- STATE_GETTABLE_SEND's own normal
    // finish, which lands directly on STATE_IDLE rather than a separate
    // DONE state the way CAL/TRAJ do). calibrationRunning/
    // trajectoryRunning's OWN success paths clear themselves explicitly
    // too, since those end on STATE_CAL_DONE/STATE_TRAJ_DONE, not here --
    // this block is what catches the abort/timeout paths for them.
    // scanResetPending/skipNextStepCheck cleared here too for the same
    // reason: either one left true by an aborted/timed-out run could
    // silently exempt a genuinely real step from judgment on the *next*
    // CAL run otherwise.
    calibrationRunning = false;
    trajectoryRunning = false;
    tableSendRunning = false;
    scanResetPending = false;
    skipNextStepCheck = false;
  }
}

// Every state that waits on something (a settle, so far) rather than
// acting and moving straight on -- the list loop() checks against for the
// shared timeout below, so a new waiting state added later just means
// adding it here, not writing its own timeout branch. STATE_CAL_RECOVER_
// WAIT is the one deliberate exception -- its own timeout means even a
// known-safe recovery candidate didn't settle, which needs a different
// response (try the next candidate) than the generic "give up," so it's
// checked directly in its own case body instead -- see there.
bool isWaitingState(CalibrationState s) {
  return s == STATE_CAL_CENTER || s == STATE_CAL_DOWN_WAIT || s == STATE_CAL_UP_CENTER ||
         s == STATE_CAL_UP_WAIT || s == STATE_CAL_TABLE_WAIT || s == STATE_TRAJ_WAIT;
}

// Which ErrorCode the shared timeout check (in loop()) raises for state s
// -- one place mapping "which wait timed out" to "what to call it," kept
// next to isWaitingState() since a state has to be in that list before
// this is ever consulted for it. Every calibration wait still shares
// ERR_CAL_TIMEOUT; only the trajectory wait gets its own code.
ErrorCode timeoutErrorFor(CalibrationState s) {
  if (s == STATE_TRAJ_WAIT) return ERR_TRAJ_TIMEOUT;
  return ERR_CAL_TIMEOUT;
}

bool abortRequested = false;   // set by handleLine() on CMD_ABORT; the tick housekeeping
                                // (not handleLine itself) is what actually forces STATE_IDLE

long prevTotalCounts = 0;      // previous tick's totalCounts, for the derivative below
long encoderDerivative = 0;    // this tick's delta, counts/tick since tick period is fixed --
                                // now computed from totalCounts (updatePositionTracking()), so a
                                // real fast spin shows as a large derivative, not a wrap artifact

bool inMotion = false;   // set by writeServoUs(), consumed once by updateSettled()
bool isSettled = false;  // updateSettled() is the only thing that ever sets this true

// Running-average window over the last CAL_SETTLE_WINDOW_SAMPLES readings
// -- a circular buffer plus its running sum, so each tick is an O(1)
// update rather than resumming the whole window. long, not int -- these
// carry totalCounts now (see updatePositionTracking()), which is
// unbounded (multi-turn), not the old raw 0..4095 value; int (16-bit on
// AVR) would silently overflow after a only few real turns.
long settleBuffer[CAL_SETTLE_WINDOW_SAMPLES];
int settleBufferCount = 0;   // valid samples so far, caps at CAL_SETTLE_WINDOW_SAMPLES
int settleBufferIndex = 0;   // next slot to overwrite
long settleBufferSum = 0;

// Running-average window over the last CAL_POSITION_FILTER_SAMPLES
// readings -- separate from settleBuffer above (different purpose: this
// one is a general-purpose smoothed position for reporting/telemetry, so
// unlike settleBuffer it's never reset on a fresh move -- it just keeps
// smoothing continuously through motion too). long, not int -- same
// totalCounts-is-unbounded reasoning as settleBuffer above.
long filterBuffer[CAL_POSITION_FILTER_SAMPLES];
int filterBufferCount = 0;
int filterBufferIndex = 0;
long filterBufferSum = 0;
long filteredPosition = 0;   // current M-sample running average -- what printMessage() reports

// Called every tick. Keeps filteredPosition as the running average of the
// last (up to) CAL_POSITION_FILTER_SAMPLES readings.
void updateFilteredPosition(long currentRaw) {
  if (filterBufferCount < CAL_POSITION_FILTER_SAMPLES) {
    filterBufferSum += currentRaw;
    filterBuffer[filterBufferCount] = currentRaw;
    filterBufferCount++;
  } else {
    filterBufferSum -= filterBuffer[filterBufferIndex];
    filterBufferSum += currentRaw;
    filterBuffer[filterBufferIndex] = currentRaw;
    filterBufferIndex = (filterBufferIndex + 1) % CAL_POSITION_FILTER_SAMPLES;
  }
  filteredPosition = filterBufferSum / filterBufferCount;
}

// Every message goes through here: prefixed with the time elapsed since
// the current state was entered, suffixed with the current filtered
// position. More fields likely get appended here later.
void printMessage(const char* msg) {
  Serial.print(elapsedStateTimeMs);
  Serial.print(F(" "));
  Serial.print(msg);
  Serial.print(F(" "));
  Serial.println(filteredPosition);
}

// ErrorCode -> wire string. One entry per ErrorCode value, same idea as
// parseCommand()'s table but in the other direction.
const char* errorCodeToString(ErrorCode code) {
  switch (code) {
    case ERR_CAL_TIMEOUT:     return "CAL_TIMEOUT";
    case ERR_TRAJ_TIMEOUT:    return "TRAJ_TIMEOUT";
    case ERR_RECOVERY_FAILED: return "RECOVERY_FAILED";
    case ERR_CAL_ABS_BOUND:   return "CAL_ABS_BOUND";
    default:                  return "UNKNOWN";
  }
}

// Errors go through here rather than printMessage() directly, so every
// error is uniformly tagged "ERR " -- more error-specific behavior
// (state changes, counters, whatever) likely gets added here later.
void throwError(ErrorCode code) {
  char buf[LINE_BUF_LEN];
  snprintf(buf, sizeof(buf), "ERR %s", errorCodeToString(code));
  printMessage(buf);
}

int lastSentUs = CENTER_US;   // the pulse most recently commanded -- kept current by writeServoUs()

// The only place that ever commands the servo -- always goes through here
// so a fresh move always marks inMotion and lastSentUs stays accurate,
// rather than every call site having to remember to.
void writeServoUs(int us) {
  servo.writeMicroseconds(us);
  lastSentUs = us;
  inMotion = true;
}

// CalibrationState -> short human-readable name, for printProgress()
// below -- reading a raw enum number back out of a log while trying to
// figure out where a run got stuck isn't worth doing by hand.
const char* stateName(CalibrationState s) {
  switch (s) {
    case STATE_IDLE:            return "IDLE";
    case STATE_CAL_CENTER:      return "CAL_CENTER";
    case STATE_CAL_DOWN_WRITE:  return "CAL_DOWN_WRITE";
    case STATE_CAL_DOWN_WAIT:   return "CAL_DOWN_WAIT";
    case STATE_CAL_RECOVER_WAIT: return "CAL_RECOVER_WAIT";
    case STATE_CAL_UP_CENTER:   return "CAL_UP_CENTER";
    case STATE_CAL_UP_WRITE:    return "CAL_UP_WRITE";
    case STATE_CAL_UP_WAIT:     return "CAL_UP_WAIT";
    case STATE_CAL_TABLE_WRITE: return "CAL_TABLE_WRITE";
    case STATE_CAL_TABLE_WAIT:  return "CAL_TABLE_WAIT";
    case STATE_CAL_DONE:        return "CAL_DONE";
    case STATE_TRAJ_PLAN:       return "TRAJ_PLAN";
    case STATE_TRAJ_STREAM:     return "TRAJ_STREAM";
    case STATE_TRAJ_WAIT:       return "TRAJ_WAIT";
    case STATE_TRAJ_DONE:       return "TRAJ_DONE";
    case STATE_GETTABLE_SEND:   return "GETTABLE_SEND";
    default:                    return "UNKNOWN";
  }
}

// Reports the current state, tickCount, and lastSentUs, alongside the
// usual elapsed-time/filteredPosition printMessage() already appends --
// called every tick during calibration (see isCalibrationState() below)
// to fill the gap CENTER/DOWN/UP/TABLE otherwise leave completely silent
// between TABLE lines/errors (confirmed on real hardware: a full
// coarse+fine range-find produced zero output for ~16 seconds,
// indistinguishable from the servo actually being stuck without this),
// and once when a trajectory move finishes, so a stall or a wrong final
// position is visible either way. State name and tickCount specifically
// so a stuck run can be pinned to an exact state and an exact tick, not
// just narrowed down to a rough time window from the host side.
void printProgress() {
  char buf[48];
  snprintf(buf, sizeof(buf), "PROGRESS %s %lu %d", stateName(currentState), tickCount, lastSentUs);
  printMessage(buf);
}

// Every calibration-routine state -- not STATE_IDLE, not STATE_CAL_DONE
// (nothing left to report once the table's built), not STATE_TRAJ_* (that
// side gets one printProgress() call at the end instead, from STATE_TRAJ_
// WAIT, not a per-tick stream). The list loop() checks against for the
// per-tick printProgress() call.
bool isCalibrationState(CalibrationState s) {
  return s == STATE_CAL_CENTER || s == STATE_CAL_DOWN_WRITE || s == STATE_CAL_DOWN_WAIT ||
         s == STATE_CAL_RECOVER_WAIT ||
         s == STATE_CAL_UP_CENTER || s == STATE_CAL_UP_WRITE || s == STATE_CAL_UP_WAIT ||
         s == STATE_CAL_TABLE_WRITE || s == STATE_CAL_TABLE_WAIT;
}

// Trajectory parameters, set from serial (ACCEL/VEL/POS) and read by
// STATE_TRAJ_PLAN. Degrees/deg-per-sec/deg-per-sec^2, matching UTI's
// TrapezoidalProfile -- NOT pulse microseconds like the rest of this
// firmware's other position variables (lastSentUs etc.); conversion to a
// pulse only happens at the very last step, see pulseForAngleCentideg().
float accelLimitDegPerSec2 = CAL_DEFAULT_ACCEL_DEG_PER_SEC2;
float velLimitDegPerSec = CAL_DEFAULT_VEL_DEG_PER_SEC;
float targetPositionDeg = 0.0f;

bool useTable = false;    // false = linear (2-point) model, true = the full calTable --
                           // set by CMD_MODEL, read by pulseForAngleCentideg()
bool isCalibrated = false;   // true once STATE_CAL_TABLE_WAIT finishes the table -- CMD_GO
                              // is rejected (ERR NOT_CALIBRATED) until this is true

TrapezoidalProfile trajProfile;   // planned once by STATE_TRAJ_PLAN, evaluated every tick by
                                   // STATE_TRAJ_STREAM until it reports settled
unsigned long trajStartMs = 0;    // millis() at the moment STATE_TRAJ_PLAN planned the move --
                                   // STATE_TRAJ_STREAM's elapsed time is relative to this

// The trajectory's own current target, in the same centideg-scaled terms
// printTelemetry() reports the actual measured position/velocity in --
// both set every STATE_TRAJ_STREAM tick, from the same pos/vel that
// tick's evaluate() call already produces to compute the pulse it writes
// (see that case body). lastTrajTargetCentideg is deliberately NOT reset
// anywhere else -- it holds at wherever the most recent move was headed
// (0, i.e. the boot reference, before the first GO ever runs), the same
// way a real setpoint would, so the html app's position-error chart
// (actual - target, matching this project's existing ServoCalibrator.html
// convention) reads a small, meaningful settling error after a move
// finishes rather than snapping to some arbitrary stale or undefined
// value. trajTargetVelCentidegPerSec, in contrast, IS explicitly zeroed
// the instant STATE_TRAJ_STREAM hands off to STATE_TRAJ_WAIT (see that
// case body) -- once nothing is actively being commanded to move, the
// planned velocity is genuinely 0, not "whatever the last tick's value
// happened to be."
int32_t lastTrajTargetCentideg = 0;
int32_t trajTargetVelCentidegPerSec = 0;

// Streamed every tick, unconditionally -- idle, mid-calibration, or
// mid-move -- unlike printProgress() (gated to isCalibrationState()): the
// html app's rolling charts need continuous coverage of the whole
// session, not just the calibration routine. Reports both the trajectory's
// current target and the actual measured shaft state, in centidegrees/
// centideg-per-second, so the html app can plot position and velocity
// directly and then compute + plot position error (actual - target,
// matching this project's existing ServoCalibrator.html convention)
// itself -- one fewer thing this firmware needs to get right, since the
// html app already has both numbers it needs to do that subtraction on
// its own.
//
// Both actualCentideg and actualVelCentidegPerSec are RAW/unfiltered --
// per explicit direction, TELEM should show the real, unsmoothed signal
// rather than whatever filteredPosition happens to be tuned to for its
// other uses (the calibration table, a GO move's planned start, the
// scan's own edge-detection -- see filteredPosition's own comment).
// actualCentideg comes from rawAngleCentideg() (totalCounts, not
// filteredPosition); actualVelCentidegPerSec comes from encoderDerivative
// (the raw per-tick delta computed once in loop() -- see its own
// comment), which was already unfiltered even before this change: the
// position filter's whole purpose is smoothing AS5600 quantization noise
// for the *filtered* uses, so differencing an already-smoothed signal
// would just reintroduce lag without removing any noise a second time.
//
// Deliberately its own wire format (TELEM, space-separated, printed
// directly rather than through printMessage()) instead of folding into
// PROGRESS -- printMessage()'s elapsed-time-since-state-entry prefix and
// raw-filteredPosition suffix are framing built for state-transition
// logging, not a fixed-rate telemetry stream a chart wants to index by
// wall-clock time.
void printTelemetry() {
  int32_t actualCentideg = rawAngleCentideg();
  int32_t actualVelCentidegPerSec = (int32_t)(encoderDerivative * 36000L / 4096L * (long)LOOP_FREQUENCY_HZ);
  Serial.print(F("TELEM "));
  Serial.print(millis());
  Serial.print(' ');
  Serial.print(lastTrajTargetCentideg);
  Serial.print(' ');
  Serial.print(trajTargetVelCentidegPerSec);
  Serial.print(' ');
  Serial.print(actualCentideg);
  Serial.print(' ');
  Serial.println(actualVelCentidegPerSec);
}

int currentStepUs = CAL_COARSE_STEP_US;   // step size STATE_CAL_DOWN_WRITE/STATE_CAL_UP_WRITE use --
                                           // CAL_COARSE_STEP_US or CAL_FINE_STEP_US, set by
                                           // beginCoarsePass()/beginFinePass() further down
ScanPhase currentPhase = PHASE_COARSE;

TestStage stopAfterStage = STAGE_ALL;   // set by CMD_STOPAFTER -- see TestStage's own comment

int stepDirection = -1;   // +1 while sweeping up, -1 while sweeping down -- set explicitly at
                           // CENTER's own transition and at the down-edge -> up-sweep handoff
                           // (see their case bodies), read only by the reversal check below

// Per-scan reference-rate + edge-backoff state -- a direct port of this
// project's own ServoDAQ host's scan_until_weak(): each of the four
// scans (coarse-down, fine-down, coarse-up, fine-up) self-calibrates its
// OWN fresh baseline rate from its own first CAL_REFERENCE_STEPS real
// steps, rather than sharing one continuous baseline across the whole
// run -- see resetScanState()/judgeScanStep().
float scanRefDeltaSum = 0.0f;
int scanRefDeltaCount = 0;
float scanReferenceRate = -1.0f;   // -1 = not yet established this scan
bool scanFirstStepPending = true;  // true until this scan's own first real step -- excluded from
                                    // judgment and from the reference-rate average: the step right
                                    // off a standing start reliably reads weaker than this servo's
                                    // real cruise rate (breakaway/stiction), confirmed on real
                                    // hardware by this project's own ServoDAQ host
bool scanPrevSettled = false;      // isSettled as of the last call, to catch the settle-edge below
bool scanEdgeFound = false;        // this tick's judgeScanStep() result -- set once in loop()'s
                                    // housekeeping, read by STATE_CAL_DOWN_WAIT/STATE_CAL_UP_WAIT's
                                    // own case bodies
long scanPrevPosition = 0;         // this scan's last ACCEPTED reading -- what the next step's delta is measured against

// Last two ACCEPTED (pulse, position) pairs, most-recent first -- backing
// off CAL_EDGE_BACKOFF_STEPS/CAL_FINE_EDGE_BACKOFF_STEPS good steps
// before a weak/reversed/big-jump step means "N steps back through this
// history," not just "one step." Both seeded to the scan's own starting
// point at reset, so backing off 1 or 2 steps from the very first judged
// step still lands somewhere real (the scan's own start), never garbage.
int scanBack1Us = 0;   long scanBack1Pos = 0;   // 1 good step back
int scanBack2Us = 0;   long scanBack2Pos = 0;   // 2 good steps back

// scanResetPending and skipNextStepCheck are both declared earlier,
// alongside calibrationRunning/trajectoryRunning, so changeState() can
// clear them on landing at STATE_IDLE -- see that declaration site for
// why. What each actually does: scanResetPending is set right when a
// scan's own starting pulse is written -- CENTER_US for a coarse pass,
// the fine pass's own margin-move target for a fine pass -- and consumed
// on THAT settle by judgeScanStep(), which resets this scan's
// reference-rate/backoff state from it rather than judging it as a step.
// skipNextStepCheck is distinct: it exempts a settle from judgment
// WITHOUT resetting anything -- used when recovering to a candidate
// mid-scan, where scanBack1Us/scanBack2Us need to keep meaning what they
// already meant, since the caller uses scanBack1Us as the edge once
// spinRecovered fires.

// Resets the per-scan reference-rate/backoff state for a fresh scan --
// consumed by judgeScanStep() the instant a scan's own starting pulse
// (CENTER_US for coarse, a fine pass's own margin-move target) settles.
// startUs/startPos are that settle's own committed pulse and measured
// position.
void resetScanState(int startUs, long startPos) {
  scanRefDeltaSum = 0.0f;
  scanRefDeltaCount = 0;
  scanReferenceRate = -1.0f;
  scanFirstStepPending = true;
  scanPrevPosition = startPos;
  scanBack1Us = startUs; scanBack1Pos = startPos;
  scanBack2Us = startUs; scanBack2Pos = startPos;
}

// Evaluated once per settle, not once per tick -- checking a raw per-tick
// derivative would compare noisy in-flight motion against noisy in-flight
// motion; checking net displacement between two settled positions is the
// same comparison this project's own ServoDAQ host makes (one delta per
// commanded move, not per tick).
//
// A direct port of that host's scan_until_weak() step judgment: weak
// (|delta| below CAL_WEAKENING_FRACTION of this scan's own reference
// rate), reversed (delta's sign disagrees with stepDirection, past the
// same weakening-fraction floor), or a big jump (|delta| above
// CAL_BIG_JUMP_MULTIPLE times the reference rate) -- see the constants'
// own comments for what each catches. All three are purely relative to
// THIS scan's own self-measured rate; there's no fixed absolute
// count/threshold anywhere in this function, unlike this file's earlier
// design. Returns true the instant an edge is found -- there's no
// debounce/sustained-run requirement (matches every scan_until_weak()
// call site in servo_daq.py: "trigger on the first weak/reversed step,
// not a sustained run").
//
// Returns false (never ends the scan) for: a non-settle tick, a
// scan-reset settle (scanResetPending), a recovery-candidate settle
// (skipNextStepCheck), and this scan's own first real step
// (scanFirstStepPending) -- see each flag's own comment.
bool judgeScanStep(long currentPos) {
  bool justSettled = isSettled && !scanPrevSettled;
  scanPrevSettled = isSettled;
  if (!justSettled) {
    return false;
  }

  if (scanResetPending) {
    scanResetPending = false;
    resetScanState(lastSentUs, currentPos);
    return false;
  }
  if (skipNextStepCheck) {
    skipNextStepCheck = false;
    return false;
  }

  long signedDelta = currentPos - scanPrevPosition;
  long delta = (signedDelta < 0) ? -signedDelta : signedDelta;

  if (scanFirstStepPending) {
    scanFirstStepPending = false;
    scanBack2Us = scanBack1Us; scanBack2Pos = scanBack1Pos;
    scanBack1Us = lastSentUs; scanBack1Pos = currentPos;
    scanPrevPosition = currentPos;
    return false;
  }

  bool edgeFound = false;
  if (scanReferenceRate < 0.0f) {
    // Still building this scan's own baseline -- no judgment possible yet.
    scanRefDeltaSum += (float)delta;
    scanRefDeltaCount++;
    if (scanRefDeltaCount >= CAL_REFERENCE_STEPS) {
      scanReferenceRate = scanRefDeltaSum / (float)scanRefDeltaCount;
    }
  } else {
    bool actualNegative = (signedDelta < 0);
    bool expectedNegative = (stepDirection < 0);
    bool weak = ((float)delta < CAL_WEAKENING_FRACTION * scanReferenceRate);
    // guarded by !weak so ordinary jitter while genuinely stopped never
    // gets misread as a reversal -- matches scan_until_weak()'s own
    // weakening_fraction noise floor for this check.
    bool reversed = (!weak) && (actualNegative != expectedNegative);
    bool bigJump = (!weak) && (!reversed) && ((float)delta > CAL_BIG_JUMP_MULTIPLE * scanReferenceRate);
    edgeFound = weak || reversed || bigJump;
  }

  if (edgeFound) {
    return true;   // this bad step is never "accepted" -- backoff history stays as-is
  }

  scanBack2Us = scanBack1Us; scanBack2Pos = scanBack1Pos;
  scanBack1Us = lastSentUs; scanBack1Pos = currentPos;
  scanPrevPosition = currentPos;
  return false;
}

// Switches to full-size steps. Called at CENTER -> down sweep, and again
// once the down side's fine pass finds the real min (coarse restarts for
// the up sweep).
void beginCoarsePass() {
  currentStepUs = CAL_COARSE_STEP_US;
  currentPhase = PHASE_COARSE;
}

// Set once STATE_CAL_RECOVER_WAIT settles at a safe candidate -- consumed
// by STATE_CAL_DOWN_WAIT/STATE_CAL_UP_WAIT as an edge-found signal
// alongside judgeScanStep()'s own return value.
bool spinRecovered = false;

// Which recovery candidate STATE_CAL_RECOVER_WAIT is currently trying --
// see recoveryCandidateUs().
int recoverCandidateIndex = 0;

// Where STATE_CAL_RECOVER_WAIT tries moving the servo back to once a
// DOWN_WAIT/UP_WAIT settle-timeout is treated as a real servo-triggered
// spin/stall-recovery event -- documented for this servo class elsewhere
// in this project (see CLAUDE.md's ServoDAQ history: driven hard enough
// past its real limit, this servo class deliberately repositions itself
// instead of stalling, real motion the Arduino doesn't control once it
// starts). No way to prevent that first move from happening -- a settle-
// timeout is inherently reactive, it can only notice after the fact (see
// its own comment in loop()) -- so lean into it instead: recover to a
// known-safe pulse, then treat reaching the edge THIS way as no different
// from reaching it via a plain stall.
//
// Candidate 0 is scanBack1Us -- the nearest point this scan has already
// directly confirmed moves normally, regardless of coarse or fine phase
// (mirrors this project's own ServoDAQ host's recover_from_wrap(), which
// recovers onto a pulse whose real position is already trusted, not the
// one that just failed). Candidate 1 is CENTER_US, always safe by
// construction (the very first point every run visits) -- the fallback
// if even the nearest candidate somehow also fails to settle. Mirrors
// that same host function's own "ordered list of fallback candidates,"
// needed there for the identical reason: a single nearest point wasn't
// always enough.
int recoveryCandidateUs(int index) {
  if (index == 0) {
    return scanBack1Us;
  }
  return CENTER_US;
}

// Writes the fine pass's own starting pulse directly -- targetUs, an
// absolute pulse, not an adjustment from wherever lastSentUs currently
// is. That distinction matters: right as a coarse pass ends, lastSentUs
// is the BAD pulse that triggered the edge, not the confirmed-good edge
// itself -- real hardware showed computing the margin from that bad/
// overshoot pulse could still land inside whatever stuck/recovering
// state caused the overshoot in the first place. Mirrors this project's
// ServoDAQ host's own fine_start = coarse_pulse - direction*MARGIN_US: a
// single move straight from wherever the coarse scan left off to the
// fine pass's own start, computed from the REPORTED edge, never the
// failing pulse. The caller transitions to the matching *_WAIT state
// after calling this, same as any other write; scanResetPending means
// that settle resets this scan's own reference-rate/backoff state
// (rather than being judged as a step) before the real first fine step
// is taken, by STATE_CAL_DOWN_WRITE/UP_WRITE once this WAIT's isSettled
// branch fires.
void beginFinePass(int targetUs) {
  currentStepUs = CAL_FINE_STEP_US;
  currentPhase = PHASE_FINE;
  scanResetPending = true;
  writeServoUs(targetUs);
}

int minUs = 0;   // set once the fine-down pass reports its edge
int maxUs = 0;   // set once the fine-up pass reports its edge

CalPoint calTable[CAL_TABLE_POINTS];
int tableIndex = 0;   // which calTable entry STATE_CAL_TABLE_WRITE/STATE_CAL_TABLE_WAIT is on
int getTableIndex = 0;   // which calTable entry STATE_GETTABLE_SEND is on -- deliberately its own
                          // variable, not a reuse of tableIndex: the two never run concurrently
                          // (CMD_GETTABLE is rejected via the busy-gate while calibrationRunning),
                          // but sharing one counter across two logically distinct purposes is the
                          // kind of thing that quietly breaks the next time that assumption changes

// The table is built from two passes, not one -- a down sweep
// (maxUs->minUs, where the fine-up pass already left the servo, so it
// starts with no extra jump) followed immediately by an up sweep
// (minUs->maxUs, starting from exactly where the down sweep just ended,
// again no jump). Both passes visit the identical CAL_TABLE_POINTS
// pulses (tablePulseUs(index) is the same function either way), just in
// opposite order, and the up pass's recordTableEntry() call averages its
// reading into the down pass's own -- direction-averaged ground truth,
// the same convention this project's sibling ServoDAQ tool uses
// (build_table()/hysteresis) and that UMI's own RCServoAutoCalibration
// example uses for the edge scan itself. true = on the down pass (first,
// raw), false = on the up pass (second, averaged) -- see
// recordTableEntry().
bool tableSweepDown = true;

// The commanded pulse for table entry `index` -- CAL_TABLE_POINTS points
// evenly spaced across [minUs, maxUs] inclusive (index 0 is exactly
// minUs, CAL_TABLE_POINTS-1 is exactly maxUs). Same function for both
// sweep directions -- only the order STATE_CAL_TABLE_WAIT visits indices
// in differs.
int tablePulseUs(int index) {
  return minUs + (int)((long)(maxUs - minUs) * index / (CAL_TABLE_POINTS - 1));
}

// filteredPosition (continuous, unwrapped counts -- see
// updatePositionTracking()) -> centidegrees, via plain integer math (exact
// ratio 36000/4096 -- no float needed). Used by recordTableEntry() (one
// reading per settle, during calibration) and currentAngleDeg() further
// down (a GO move's planned starting point) -- both want the smoothed
// value. printTelemetry() does NOT use this (see rawAngleCentideg()
// right below) -- per explicit direction, TELEM reports the raw,
// unfiltered position, decoupled from whatever filteredPosition is
// tuned to for those other, still-filtered uses.
// Defined here, ahead of recordTableEntry()'s own use of it, but Arduino
// auto-generates a forward declaration for every function in the file
// (the same reason CalPoint had to be hand-placed near the top instead --
// see its own comment -- except that applies to types, which don't get
// this treatment, only function signatures do), so call-before-definition
// order elsewhere in this file is never actually a problem.
int32_t currentAngleCentideg() {
  return (int32_t)(filteredPosition * 36000L / 4096L);
}

// Same conversion as currentAngleCentideg(), but from totalCounts (the
// raw, unfiltered running position updatePositionTracking() maintains
// every tick) instead of filteredPosition -- what printTelemetry()
// reports as the actual position, per explicit direction that TELEM
// should show the real, unsmoothed signal rather than whatever
// filteredPosition happens to be averaged over for its other uses.
int32_t rawAngleCentideg() {
  return (int32_t)(totalCounts * 36000L / 4096L);
}

// Streams one calTable entry over the wire, as "TABLE <idx> <pulseUs>
// <angleCentideg>" -- shared by recordTableEntry() below (a genuinely new
// reading, just measured) and STATE_GETTABLE_SEND (an existing entry from
// an already-built table, just being replayed back out on request), so
// both stay in the exact same wire shape automatically rather than two
// independent snprintf() calls silently drifting apart.
void printTableEntry(int idx) {
  char buf[40];
  // %ld for angleCentideg -- it's int32_t/long now, not int; a bare %d
  // there would read the wrong bytes off the varargs stack on AVR (16-bit
  // int).
  snprintf(buf, sizeof(buf), "TABLE %d %d %ld", idx, calTable[idx].pulseUs, calTable[idx].angleCentideg);
  printMessage(buf);
}

// Called from STATE_CAL_TABLE_WAIT once a table point settles. No
// truncating cast to int16_t here -- angleCentideg is int32_t precisely so
// this doesn't need one. Index progression is the caller's job now
// (STATE_CAL_TABLE_WAIT), since which direction "next" means depends on
// tableSweepDown.
void recordTableEntry() {
  int32_t angleCentideg = currentAngleCentideg();
  if (tableSweepDown) {
    // First (down) pass -- raw reading, nothing to average against yet.
    calTable[tableIndex].pulseUs = lastSentUs;
    calTable[tableIndex].angleCentideg = angleCentideg;
  } else {
    // Second (up) pass -- same pulse as the down pass already recorded
    // here (both sweeps visit tablePulseUs(tableIndex)); average the two
    // readings into the final value.
    calTable[tableIndex].angleCentideg = (calTable[tableIndex].angleCentideg + angleCentideg) / 2;
  }
  printTableEntry(tableIndex);
}

// The 2-point "linear" model, in the same CalPoint shape as calTable --
// just its first and last entries (minUs/its angle, maxUs/its angle).
// Built once, right when the full table finishes (see STATE_CAL_TABLE_
// WAIT), so LINEAR and TABLE modes can share one lookup function below
// instead of a separate linear-formula branch.
CalPoint linearTable[2];

// Binary search + linear interpolation -- same algorithm as UMI's own
// lookupPulseUsFromTable() (and this project's sibling firmware's
// lookupPulseFromRamTable(), its RAM-safe mirror of it, for the same
// reason ours is standalone here: calTable is built at runtime, so it
// lives in plain RAM, never PROGMEM -- UMI's own table functions read
// PROGMEM and would misread a RAM array as flash on AVR). Generalized to
// any table of at least 2 points, ascending by angleCentideg, so it works
// unmodified for both models: pass linearTable (len 2) or calTable (len
// CAL_TABLE_POINTS). Angles outside the table's range clamp to the
// nearest endpoint, matching UMI's own documented behavior.
uint16_t interpolateTable(const CalPoint* table, uint8_t len, int32_t angleCentideg) {
  if (angleCentideg <= table[0].angleCentideg) return table[0].pulseUs;
  if (angleCentideg >= table[len - 1].angleCentideg) return table[len - 1].pulseUs;

  uint8_t lo = 0, hi = len - 1;
  while (hi - lo > 1) {
    uint8_t mid = (lo + hi) / 2;
    if (table[mid].angleCentideg <= angleCentideg) lo = mid; else hi = mid;
  }

  // int32_t, not int/uint16_t: on AVR (16-bit int), a uint16_t difference
  // promotes to *unsigned* int, so a locally-decreasing pulseUs across an
  // angle-ascending step would wrap to a huge positive value instead of
  // going negative -- same guard UMI's own version uses, for the same
  // reason (pulse isn't assumed monotonic, only angle is).
  int32_t angleSpan = table[hi].angleCentideg - table[lo].angleCentideg;
  int32_t pulseSpan = (int32_t)table[hi].pulseUs - table[lo].pulseUs;
  float t = (angleSpan != 0) ? (float)(angleCentideg - table[lo].angleCentideg) / (float)angleSpan : 0.0f;
  float us = table[lo].pulseUs + t * (float)pulseSpan;
  if (us < 0.0f) us = 0.0f;
  if (us > 65535.0f) us = 65535.0f;
  return (uint16_t)(us + 0.5f);
}

// The one place LINEAR vs TABLE actually branches -- everything upstream
// (planning, streaming) is model-agnostic and just asks for a pulse.
uint16_t pulseForAngleCentideg(int32_t angleCentideg) {
  if (useTable) {
    return interpolateTable(calTable, CAL_TABLE_POINTS, angleCentideg);
  }
  return interpolateTable(linearTable, 2, angleCentideg);
}

// Current shaft angle, in degrees -- same raw-AS5600-frame conversion
// recordTableEntry() uses for calTable's own angleCentideg values (plain
// degrees here instead of centidegrees, since this feeds q0 into
// TrapezoidalProfile::plan(), which works in whatever unit the caller
// gives it -- degrees, to match targetPositionDeg/velLimitDegPerSec/
// accelLimitDegPerSec2).
float currentAngleDeg() {
  return (float)filteredPosition * 360.0f / 4096.0f;
}

// Discards whatever's in the running-average window -- called whenever a
// fresh move starts, so samples from before that move never get averaged
// in with samples from after it.
void resetSettleWindow() {
  settleBufferCount = 0;
  settleBufferIndex = 0;
  settleBufferSum = 0;
}

// Called every tick, before the switch. A move just commanded (inMotion)
// unconditionally forces isSettled false, consumes the flag, and resets
// the running-average window. Otherwise: push currentRaw into the window,
// and once it's full, isSettled is true exactly when currentRaw is within
// CAL_SETTLE_WINDOW_COUNTS of the window's running average -- same idea as
// the Python host's reference-rate self-calibration, just windowed instead
// of a fixed early-steps baseline.
void updateSettled(long currentRaw) {
  if (inMotion) {
    isSettled = false;
    inMotion = false;
    resetSettleWindow();
    return;
  }

  if (settleBufferCount < CAL_SETTLE_WINDOW_SAMPLES) {
    settleBufferSum += currentRaw;
    settleBuffer[settleBufferCount] = currentRaw;
    settleBufferCount++;
  } else {
    settleBufferSum -= settleBuffer[settleBufferIndex];
    settleBufferSum += currentRaw;
    settleBuffer[settleBufferIndex] = currentRaw;
    settleBufferIndex = (settleBufferIndex + 1) % CAL_SETTLE_WINDOW_SAMPLES;
  }

  if (settleBufferCount < CAL_SETTLE_WINDOW_SAMPLES) {
    return;   // window not full yet -- not enough history to judge, leave isSettled as-is
  }

  long average = settleBufferSum / CAL_SETTLE_WINDOW_SAMPLES;
  long diff = currentRaw - average;
  if (diff < 0) diff = -diff;
  isSettled = (diff <= CAL_SETTLE_WINDOW_COUNTS);
}

// parseCommand() only ever sees the first token now (see tokenizeLine()
// below) -- a plain strcmp against the whole line stopped being enough
// once ACCEL/VEL/POS needed an argument after the command word.
SerialCommand parseCommand(const char* cmdTok) {
  if (strcmp(cmdTok, "PING") == 0) return CMD_PING;
  if (strcmp(cmdTok, "CAL") == 0) return CMD_CAL;
  if (strcmp(cmdTok, "ABORT") == 0) return CMD_ABORT;
  if (strcmp(cmdTok, "ACCEL") == 0) return CMD_ACCEL;
  if (strcmp(cmdTok, "VEL") == 0) return CMD_VEL;
  if (strcmp(cmdTok, "POS") == 0) return CMD_POS;
  if (strcmp(cmdTok, "MODEL") == 0) return CMD_MODEL;
  if (strcmp(cmdTok, "GO") == 0) return CMD_GO;
  if (strcmp(cmdTok, "STOPAFTER") == 0) return CMD_STOPAFTER;
  if (strcmp(cmdTok, "GETTABLE") == 0) return CMD_GETTABLE;
  return CMD_UNKNOWN;
}

const int MAX_TOKENS = 2;   // command + at most 1 argument (ACCEL/VEL/POS/MODEL/STOPAFTER)
char* tok[MAX_TOKENS];

// Splits line in place on spaces (strtok mutates it, fine -- line is
// always lineBuf, a scratch buffer nothing else reads after this).
// Returns how many tokens were found (0 for a blank line).
int tokenizeLine(char* line) {
  int n = 0;
  char* p = strtok(line, " ");
  while (p != NULL && n < MAX_TOKENS) {
    tok[n++] = p;
    p = strtok(NULL, " ");
  }
  return n;
}

// Dispatch on the parsed command. CAL starts a calibration run (STATE_IDLE
// -> STATE_CAL_CENTER); ABORT just raises abortRequested -- the tick
// housekeeping is what actually forces the state back to STATE_IDLE, so
// every state gets the same abort handling instead of each one checking
// serial itself. ABORT while already idle is rejected -- there's nothing
// running to abort. ACCEL/VEL/POS each take one numeric argument (degrees,
// deg/sec, deg/sec^2) and just update their own variable -- read by
// STATE_TRAJ_PLAN whenever GO next plans a move, not applied retroactively
// to one already streaming. MODEL LINEAR/TABLE switches useTable; anything
// else is rejected. GO plans and starts a trajectory move to
// targetPositionDeg -- rejected (ERR NOT_CALIBRATED) until isCalibrated.
// GETTABLE replays the current session's already-built calTable back
// over the wire (STATE_GETTABLE_SEND) -- also rejected (ERR NOT_
// CALIBRATED) until isCalibrated; unlike CAL/GO it does NOT ack
// immediately, since the whole send finishes well under a second (see
// STATE_GETTABLE_SEND's own comment) -- its one reply comes only once
// every point has actually gone out. While calibrationRunning,
// trajectoryRunning, or tableSendRunning, every command except ABORT is
// rejected outright -- nothing overlaps a run in progress. PING is the
// other deliberate exception: a pure liveness check, no side effects, so
// a host app can confirm the connection is alive without having to wait
// out whatever run is in progress. Anything unrecognized, or with the
// wrong number of arguments, is rejected too.
void handleLine(char* line) {
  int n = tokenizeLine(line);
  if (n == 0) return;

  SerialCommand cmd = parseCommand(tok[0]);
  if ((calibrationRunning || trajectoryRunning || tableSendRunning) && cmd != CMD_ABORT && cmd != CMD_PING) {
    Serial.println(F("ERR BUSY"));
    return;
  }
  switch (cmd) {
    case CMD_PING:
      Serial.println(F("OK PING"));
      break;
    case CMD_CAL:
      calibrationRunning = true;
      changeState(STATE_CAL_CENTER);
      Serial.println(F("OK CAL"));
      break;
    case CMD_ABORT:
      if (currentState == STATE_IDLE) {
        Serial.println(F("ERR ALREADY_IDLE"));
      } else {
        abortRequested = true;
        Serial.println(F("OK ABORT"));
      }
      break;
    case CMD_ACCEL:
      if (n != 2) { Serial.println(F("ERR USAGE")); break; }
      accelLimitDegPerSec2 = atof(tok[1]);
      Serial.println(F("OK ACCEL"));
      break;
    case CMD_VEL:
      if (n != 2) { Serial.println(F("ERR USAGE")); break; }
      velLimitDegPerSec = atof(tok[1]);
      Serial.println(F("OK VEL"));
      break;
    case CMD_POS:
      if (n != 2) { Serial.println(F("ERR USAGE")); break; }
      targetPositionDeg = atof(tok[1]);
      Serial.println(F("OK POS"));
      break;
    case CMD_MODEL:
      if (n != 2) { Serial.println(F("ERR USAGE")); break; }
      if (strcmp(tok[1], "LINEAR") == 0) {
        useTable = false;
        Serial.println(F("OK MODEL"));
      } else if (strcmp(tok[1], "TABLE") == 0) {
        useTable = true;
        Serial.println(F("OK MODEL"));
      } else {
        Serial.println(F("ERR USAGE"));
      }
      break;
    case CMD_GO:
      if (!isCalibrated) {
        Serial.println(F("ERR NOT_CALIBRATED"));
        break;
      }
      trajectoryRunning = true;
      changeState(STATE_TRAJ_PLAN);
      Serial.println(F("OK GO"));
      break;
    case CMD_STOPAFTER:
      if (n != 2) { Serial.println(F("ERR USAGE")); break; }
      if (strcmp(tok[1], "ALL") == 0) {
        stopAfterStage = STAGE_ALL;
        Serial.println(F("OK STOPAFTER"));
      } else if (strcmp(tok[1], "COARSE_DOWN") == 0) {
        stopAfterStage = STAGE_COARSE_DOWN;
        Serial.println(F("OK STOPAFTER"));
      } else if (strcmp(tok[1], "COARSE_UP") == 0) {
        stopAfterStage = STAGE_COARSE_UP;
        Serial.println(F("OK STOPAFTER"));
      } else if (strcmp(tok[1], "FINE_DOWN") == 0) {
        stopAfterStage = STAGE_FINE_DOWN;
        Serial.println(F("OK STOPAFTER"));
      } else if (strcmp(tok[1], "FINE_UP") == 0) {
        stopAfterStage = STAGE_FINE_UP;
        Serial.println(F("OK STOPAFTER"));
      } else {
        Serial.println(F("ERR USAGE"));
      }
      break;
    case CMD_GETTABLE:
      if (!isCalibrated) {
        Serial.println(F("ERR NOT_CALIBRATED"));
        break;
      }
      tableSendRunning = true;
      getTableIndex = 0;
      changeState(STATE_GETTABLE_SEND);
      // No immediate "OK" here -- see this function's own header comment
      // and STATE_GETTABLE_SEND for why GETTABLE's reply is deferred
      // instead of acknowledging up front like CAL/GO do.
      break;
    default:
      Serial.println(F("ERR UNKNOWN_CMD"));
      break;
  }
}

// Non-blocking: consumes whatever bytes are waiting, dispatches on a
// complete \n/\r-terminated line, otherwise returns immediately without
// blocking the tick.
void checkSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (lineLen > 0) {        // ignore blank lines (e.g. a lone \r before \n)
        lineBuf[lineLen] = '\0';
        handleLine(lineBuf);
        lineLen = 0;
      }
    } else if (lineLen < LINE_BUF_LEN - 1) {
      lineBuf[lineLen++] = c;   // silently drop anything past LINE_BUF_LEN
    }
  }
}

void setup() {
  Serial.begin(115200);
  while (!Serial) {}
  delay(300);   // let the USB-serial adapter settle before printing

  Serial.println(F("# ServoCalibrator_Companion booting..."));

  Wire.begin();
  Wire.setClock(400000);  // Fast-mode I2C -- keeps one read comfortably inside a tick
  encoder.begin();
  if (!encoder.isConnected()) {
    Serial.println(F("# FATAL: AS5600 not detected. Halting."));
    while (true) {}
  }

  servo.attach(SERVO_PIN, ABS_FLOOR_US, ABS_CEIL_US);
  prevRaw = encoder.readAngle();          // seed updatePositionTracking()'s wrap detector --
                                           // boot position becomes 0 total counts
  prevTotalCounts = totalCounts;          // = 0 too, seeding encoderDerivative's own baseline

  Serial.println(F("# READY"));
}

void loop() {
  unsigned long now = millis();
  if (now - lastTickMs < TICK_INTERVAL_MS) {
    return;
  }
  lastTickMs = now;
  tickCount++;

  // Reflects time in whatever state we were in as of *this* tick's start,
  // before checkSerial() (or anything else below) gets a chance to call
  // changeState() -- changeState() always overwrites this directly with
  // its own fresh millis() read, so computing it here first (rather than
  // subtracting `now` from a stateEnteredMs that a same-tick changeState()
  // may have already moved forward) can never underflow.
  elapsedStateTimeMs = now - stateEnteredMs;

  checkSerial();   // before the switch -- a command this tick can reject or redirect state first

  long currentTotal = updatePositionTracking();   // the only encoder read -- see its own comment
  encoderDerivative = currentTotal - prevTotalCounts;
  prevTotalCounts = currentTotal;

  updateFilteredPosition(currentTotal);   // every tick -- keeps filteredPosition current for printMessage()
  updateSettled(currentTotal);   // every tick -- reacts to any move writeServoUs() just commanded

  printTelemetry();   // every tick, unconditionally, regardless of state -- see its own comment

  if (isCalibrationState(currentState)) {   // every tick, the whole way through calibration -- see printProgress()
    printProgress();
  }

  // Gated to an active down/up pass specifically (coarse or fine, either
  // one) -- STATE_CAL_CENTER's own settle is not a pass step and must
  // never reach judgeScanStep() (its own case body calls resetScanState()
  // directly instead). STATE_CAL_UP_CENTER *is* included, even though its
  // own settle isn't judged either (scanResetPending, set before that
  // move is written -- see DOWN_WAIT's own case body) -- it still needs
  // to reach judgeScanStep() so the coarse-up scan's reference-rate/
  // backoff state actually gets reset from the settled-at-CENTER_US
  // position. Real hardware showed why re-anchoring like this matters (in
  // this file's earlier design, the equivalent miss silently ended the up
  // pass right on its first step, landing on STATE_CAL_DONE with nothing
  // printed -- looked like a hang rather than a false detection).
  if (currentState == STATE_CAL_DOWN_WRITE || currentState == STATE_CAL_DOWN_WAIT ||
      currentState == STATE_CAL_RECOVER_WAIT ||
      currentState == STATE_CAL_UP_CENTER ||
      currentState == STATE_CAL_UP_WRITE || currentState == STATE_CAL_UP_WAIT) {
    scanEdgeFound = judgeScanStep(filteredPosition);
  }

  if (abortRequested) {   // exit route 3, shared by every state: abort forces STATE_IDLE
    abortRequested = false;
    changeState(STATE_IDLE);
  }

  // Shared timeout, checked here rather than duplicated in every waiting
  // state's own case body -- see isWaitingState()'s own comment. Which
  // error code that is depends on which wait timed out -- see
  // timeoutErrorFor()'s own comment.
  //
  // STATE_CAL_DOWN_WAIT/STATE_CAL_UP_WAIT are the one deliberate
  // exception: a coarse/fine step that never settles at all isn't a plain
  // stall (which settles, just near-zero delta) -- it's a real servo-
  // triggered spin/stall-recovery event, documented for this servo class
  // elsewhere in this project. Lean into it rather than just giving up --
  // recover to a known-safe pulse (see recoveryCandidateUs()'s own
  // comment), then treat reaching it as the edge signal itself, the same
  // way this project's ServoDAQ sibling tool's naive_stall_sweep()/
  // scan_until_weak() treat a NotSettledError as reaching the edge rather
  // than an error to just report.
  if (isWaitingState(currentState) && elapsedStateTimeMs >= CAL_SETTLE_TIMEOUT_MS) {
    if (currentState == STATE_CAL_DOWN_WAIT || currentState == STATE_CAL_UP_WAIT) {
      // scanBack1Us already holds "the last pulse this scan directly
      // confirmed moves normally" -- judgeScanStep() maintains it
      // continuously, so there's nothing to (re)compute here the way
      // this file's earlier design needed to.
      recoverCandidateIndex = 0;
      skipNextStepCheck = true;
      writeServoUs(recoveryCandidateUs(recoverCandidateIndex));
      changeState(STATE_CAL_RECOVER_WAIT);
    } else {
      throwError(timeoutErrorFor(currentState));
      changeState(STATE_IDLE);
    }
  }

  bool enteringState = justEnteredState;
  justEnteredState = false;

  switch (currentState) {
    case STATE_IDLE:
      break;

    case STATE_CAL_CENTER:
      if (enteringState) {
        writeServoUs(CENTER_US);
      }
      if (isSettled) {
        // settled, per updateSettled() -- the timeout exit route is
        // handled generically above, for every waiting state at once.
        // This is the one genuine start of a new calibration run, so
        // it's the one place the coarse-down scan's own reference-rate/
        // backoff state gets reset -- directly, not through
        // judgeScanStep()/scanResetPending (CENTER's own settle
        // deliberately never reaches that shared path -- see loop()'s
        // own comment) -- and stepDirection set.
        resetScanState(CENTER_US, filteredPosition);
        stepDirection = -1;
        beginCoarsePass();
        changeState(STATE_CAL_DOWN_WRITE);
      }
      break;

    case STATE_CAL_DOWN_WRITE:
      // One-tick state: take the next step down, then immediately hand
      // off to STATE_CAL_DOWN_WAIT to watch it. Hard pulse-bound check
      // mirrors this project's ServoDAQ host's own explicit floor_us/
      // ceil_us RuntimeError -- a real fail-safe in case weak/reversed/
      // big-jump detection somehow never triggers (e.g. an encoder
      // fault), not just servo.attach()'s own silent PWM clamp.
      {
        int nextUs = lastSentUs - currentStepUs;
        if (nextUs < ABS_FLOOR_US || nextUs > ABS_CEIL_US) {
          throwError(ERR_CAL_ABS_BOUND);
          calibrationRunning = false;
          changeState(STATE_IDLE);
          break;
        }
        lastSentUs = nextUs;
      }
      writeServoUs(lastSentUs);
      changeState(STATE_CAL_DOWN_WAIT);
      break;

    case STATE_CAL_DOWN_WAIT: {
      // A direct port of this project's ServoDAQ host's find_edge():
      // judgeScanStep() (called in loop()'s housekeeping, before this
      // switch runs) already decided whether this settle found the edge
      // -- scanEdgeFound -- using the exact same weak/reversed/big-jump
      // logic for both coarse and fine (see its own comment); spinRecovered
      // is the other way an edge gets reported, via an active-recovery
      // settle-timeout (mirrors that same host's NotSettledError ->
      // recover_from_wrap()). Recovery always reports scanBack1Us
      // directly (mirrors trace[-1], no further backoff, regardless of
      // phase); a rate-detected edge backs off CAL_EDGE_BACKOFF_STEPS
      // (coarse) or CAL_FINE_EDGE_BACKOFF_STEPS (fine) good steps instead
      // -- see the constants' own comments for why fine gets more margin.
      bool edgeFound = spinRecovered || scanEdgeFound;
      bool viaRecovery = spinRecovered;
      spinRecovered = false;
      if (edgeFound) {
        int edgeUs = (viaRecovery || currentPhase == PHASE_COARSE) ? scanBack1Us : scanBack2Us;
        if (currentPhase == PHASE_COARSE) {
          if (stopAfterStage == STAGE_COARSE_DOWN) {
            // isolating this one stage for testing -- see TestStage's
            // own comment. calibrationRunning cleared explicitly, same
            // as the normal finish line in STATE_CAL_TABLE_WAIT -- this
            // lands on STATE_CAL_DONE too, not STATE_IDLE, so
            // changeState() won't clear it automatically.
            calibrationRunning = false;
            changeState(STATE_CAL_DONE);
          } else {
            // Rough location only -- refine in fine steps, a fixed
            // CAL_FINE_MARGIN_US back toward center from the reported
            // coarse edge (not from wherever lastSentUs currently sits --
            // see beginFinePass()'s own comment for why that distinction
            // matters).
            beginFinePass(edgeUs + CAL_FINE_MARGIN_US);
            changeState(STATE_CAL_DOWN_WAIT);
          }
        } else {
          minUs = edgeUs;
          if (stopAfterStage == STAGE_FINE_DOWN) {
            // isolating this one stage for testing -- see TestStage's own
            // comment. minUs is already recorded above; just stop here
            // instead of continuing on to the up sweep.
            calibrationRunning = false;
            changeState(STATE_CAL_DONE);
          } else {
            // Up sweep always starts from CENTER_US, not from wherever the
            // down sweep/fine pass left off -- see STATE_CAL_UP_CENTER's
            // own comment. stepDirection/beginCoarsePass() happen once
            // that return move has actually settled, not here.
            // scanResetPending exempts this move's own settle from being
            // judged as a step (same mechanism beginFinePass()'s margin
            // move uses) and resets the coarse-up scan's own
            // reference-rate/backoff state from wherever it lands.
            scanResetPending = true;
            writeServoUs(CENTER_US);
            changeState(STATE_CAL_UP_CENTER);
          }
        }
      } else if (isSettled) {
        // settled with a normal, valid step -- keep sweeping
        changeState(STATE_CAL_DOWN_WRITE);
      }
      break;
    }

    case STATE_CAL_RECOVER_WAIT:
      // Not in isWaitingState()'s generic timeout list on purpose -- see
      // that list's own comment for why: a timeout here means even a
      // known-safe recovery candidate didn't settle, which gets a
      // different response (try the next candidate) than the generic
      // "give up," so it's checked directly, right here, instead.
      if (isSettled) {
        spinRecovered = true;
        changeState(stepDirection < 0 ? STATE_CAL_DOWN_WAIT : STATE_CAL_UP_WAIT);
      } else if (elapsedStateTimeMs >= CAL_SETTLE_TIMEOUT_MS) {
        recoverCandidateIndex++;
        if (recoverCandidateIndex > 1) {
          // Both candidates failed to settle -- CENTER_US not settling is
          // not something this firmware has a further fallback for.
          // Genuinely can't recover; give up for real.
          throwError(ERR_RECOVERY_FAILED);
          calibrationRunning = false;
          changeState(STATE_IDLE);
        } else {
          skipNextStepCheck = true;
          writeServoUs(recoveryCandidateUs(recoverCandidateIndex));
          changeState(STATE_CAL_RECOVER_WAIT);   // re-enter cleanly, resets the timeout window
        }
      }
      break;

    case STATE_CAL_UP_CENTER:
      // The down edge was just found; the servo is sitting wherever the
      // fine pass left it. writeServoUs(CENTER_US) and
      // scanResetPending=true already happened in the DOWN_WAIT branch
      // that got us here, same one-shot-write-then-wait pattern as every
      // other transition -- this state just watches it settle. It IS
      // gated into judgeScanStep() (see loop()'s own comment) so the
      // coarse-up scan's reference-rate/backoff state gets reset from
      // this settled-at-CENTER_US position, exactly like beginFinePass()'s
      // margin move resets the fine scan that follows it.
      if (isSettled) {
        stepDirection = 1;
        beginCoarsePass();
        changeState(STATE_CAL_UP_WRITE);
      }
      break;

    case STATE_CAL_UP_WRITE:
      // Mirrors STATE_CAL_DOWN_WRITE -- += instead of -=.
      {
        int nextUs = lastSentUs + currentStepUs;
        if (nextUs < ABS_FLOOR_US || nextUs > ABS_CEIL_US) {
          throwError(ERR_CAL_ABS_BOUND);
          calibrationRunning = false;
          changeState(STATE_IDLE);
          break;
        }
        lastSentUs = nextUs;
      }
      writeServoUs(lastSentUs);
      changeState(STATE_CAL_UP_WAIT);
      break;

    case STATE_CAL_UP_WAIT: {
      // Mirrors STATE_CAL_DOWN_WAIT throughout, see its own comments.
      bool edgeFound = spinRecovered || scanEdgeFound;
      bool viaRecovery = spinRecovered;
      spinRecovered = false;
      if (edgeFound) {
        int edgeUs = (viaRecovery || currentPhase == PHASE_COARSE) ? scanBack1Us : scanBack2Us;
        if (currentPhase == PHASE_COARSE) {
          if (stopAfterStage == STAGE_COARSE_UP) {
            // isolating this one stage for testing -- see TestStage's own
            // comment. Same calibrationRunning note as STATE_CAL_DOWN_WAIT's
            // own branch.
            calibrationRunning = false;
            changeState(STATE_CAL_DONE);
          } else {
            beginFinePass(edgeUs - CAL_FINE_MARGIN_US);
            changeState(STATE_CAL_UP_WAIT);
          }
        } else {
          maxUs = edgeUs;
          if (stopAfterStage == STAGE_FINE_UP) {
            // isolating this one stage for testing -- see TestStage's own
            // comment. maxUs is already recorded above; stop here instead
            // of continuing on into building the table.
            calibrationRunning = false;
            changeState(STATE_CAL_DONE);
          } else {
            // Table build starts with the down pass, from exactly where
            // the fine-up sweep just left the servo (maxUs) -- see
            // tableSweepDown's own comment.
            tableIndex = CAL_TABLE_POINTS - 1;
            tableSweepDown = true;
            changeState(STATE_CAL_TABLE_WRITE);
          }
        }
      } else if (isSettled) {
        changeState(STATE_CAL_UP_WRITE);
      }
      break;
    }

    case STATE_CAL_TABLE_WRITE:
      // One-tick state: command the next table point, then immediately
      // hand off to STATE_CAL_TABLE_WAIT to watch it settle.
      writeServoUs(tablePulseUs(tableIndex));
      changeState(STATE_CAL_TABLE_WAIT);
      break;

    case STATE_CAL_TABLE_WAIT:
      // Timeout exit route handled generically above, same as every other
      // waiting state. On settle: record this entry, then either the next
      // point (in whichever direction this pass is going), the other
      // pass, or, once both are done, calibration is done -- see
      // tableSweepDown's own comment for why there are two passes at all.
      if (isSettled) {
        recordTableEntry();
        if (tableSweepDown) {
          if (tableIndex > 0) {
            tableIndex--;
            changeState(STATE_CAL_TABLE_WRITE);
          } else {
            // Down pass just finished at minUs -- the up pass starts
            // from this exact same point, so no extra move is needed
            // before its own first WRITE.
            tableSweepDown = false;
            changeState(STATE_CAL_TABLE_WRITE);
          }
        } else {
          if (tableIndex < CAL_TABLE_POINTS - 1) {
            tableIndex++;
            changeState(STATE_CAL_TABLE_WRITE);
          } else {
            linearTable[0] = calTable[0];                    // minUs and its measured angle
            linearTable[1] = calTable[CAL_TABLE_POINTS - 1];  // maxUs and its measured angle
            isCalibrated = true;
            calibrationRunning = false;   // table complete -- see changeState()'s own comment
                                           // for why this isn't handled there too
            changeState(STATE_CAL_DONE);
          }
        }
      }
      break;

    case STATE_CAL_DONE:
      break;

    case STATE_TRAJ_PLAN: {
      // One-tick state: plan from wherever the shaft actually is right
      // now to targetPositionDeg, at the current ACCEL/VEL limits, then
      // immediately hand off to STATE_TRAJ_STREAM to run it.
      TrajectoryLimits limits;
      limits.vMax = velLimitDegPerSec;
      limits.aMax = accelLimitDegPerSec2;
      trajProfile.plan(currentAngleDeg(), targetPositionDeg, limits);
      trajStartMs = millis();
      changeState(STATE_TRAJ_STREAM);
      break;
    }

    case STATE_TRAJ_STREAM: {
      // Every tick: ask the profile where it should be right now, convert
      // that (degrees) to a pulse through whichever model MODEL selected
      // (pulseForAngleCentideg() is the only place that decision is made),
      // and write it. evaluate() returning false means the profile's own
      // duration has elapsed -- not the same thing as the shaft having
      // physically arrived, which STATE_TRAJ_WAIT checks next.
      float t = (millis() - trajStartMs) / 1000.0f;
      float pos, vel, accel;
      bool stillMoving = trajProfile.evaluate(t, pos, vel, accel);
      int32_t angleCentideg = (int32_t)(pos * 100.0f + (pos >= 0.0f ? 0.5f : -0.5f));
      uint16_t pulse = pulseForAngleCentideg(angleCentideg);
      writeServoUs(pulse);

      // Feeds printTelemetry() -- the html app's position/velocity charts
      // plot this planned target against the actual measured values
      // printTelemetry() reports alongside it every tick.
      lastTrajTargetCentideg = angleCentideg;
      trajTargetVelCentidegPerSec = (int32_t)(vel * 100.0f + (vel >= 0.0f ? 0.5f : -0.5f));

      if (!stillMoving) {
        // The plan has ended -- nothing is being commanded to move
        // anymore, so the reported target velocity should genuinely read
        // 0 from here on, not linger at whatever this last tick's value
        // happened to be (see trajTargetVelCentidegPerSec's own comment).
        // lastTrajTargetCentideg is left alone -- it correctly holds at
        // the destination through STATE_TRAJ_WAIT and beyond.
        trajTargetVelCentidegPerSec = 0;
        changeState(STATE_TRAJ_WAIT);
      }
      break;
    }

    case STATE_TRAJ_WAIT:
      // Same isSettled/timeout machinery every other waiting state uses.
      // Ideally this settles almost immediately (the last streamed
      // setpoint IS where the profile planned to end up) -- real delay
      // here is just the servo's own response lag plus whatever phase lag
      // updateFilteredPosition()'s smoothing adds, not further motion.
      if (isSettled) {
        printProgress();   // one report, the actual final position -- not a per-tick
                            // stream during STATE_TRAJ_STREAM, just this one at the end
        trajectoryRunning = false;   // move complete -- see changeState()'s own comment
                                      // for why this isn't handled there too
        changeState(STATE_TRAJ_DONE);
      }
      break;

    case STATE_TRAJ_DONE:
      break;

    case STATE_GETTABLE_SEND:
      // One entry per tick, oldest-first -- same one-thing-per-tick FSM
      // shape this file uses everywhere else (STATE_CAL_TABLE_WRITE/WAIT
      // et al.), rather than blasting all CAL_TABLE_POINTS lines out of
      // one loop() call. No motion or settling involved: calTable is
      // already sitting in RAM from the last CAL run, this just replays
      // it back out on request via the same printTableEntry() formatter
      // recordTableEntry() itself uses. The one "OK GETTABLE" reply --
      // unlike CAL/GO's immediate ack -- lands only once every point has
      // actually gone out: at CAL_TABLE_POINTS ticks (well under a
      // second at this 50Hz rate), a plain synchronous request/response
      // round trip fits comfortably inside a normal command timeout,
      // unlike CAL (up to ~a minute) or GO (however long the move
      // takes), which both need to ack up front so the caller isn't
      // blocked that long. (An ABORT landing mid-send would force
      // STATE_IDLE without this reply ever going out -- a real but
      // low-stakes edge case: nothing physical is in flight to abort
      // here, the caller just sees its GETTABLE time out instead of
      // erroring cleanly.)
      printTableEntry(getTableIndex);
      getTableIndex++;
      if (getTableIndex >= CAL_TABLE_POINTS) {
        changeState(STATE_IDLE);   // tableSendRunning cleared there
        Serial.println(F("OK GETTABLE"));
      }
      break;
  }
}
