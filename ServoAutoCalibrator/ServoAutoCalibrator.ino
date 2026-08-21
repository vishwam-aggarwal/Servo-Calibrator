/*
  ServoAutoCalibrator

  On-device servo range-finding/calibration, as a finite state machine.
  States and transitions still to be filled in -- for now, just the
  fixed-tick skeleton plus the housekeeping that runs ahead of the state
  switch every tick: reading the AS5600, and checking for an incoming
  serial command that might reject outright or force a state change
  (e.g. ABORT) before that tick's state logic runs.

  Runs on one fixed-rate tick (see TICK_INTERVAL_MS below); the state
  machine itself is a single switch/case in loop(), one case per enum
  value in CalibrationState.
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
// a fully flat 100us fine-margin sweep, RAWSWEEP all reading the same
// encoder position from 540 down to 250) -- it was never physical at
// all, and running ServoDAQ_Companion (whose attach() already passes
// these) against the same servo the same session proved it: smooth,
// unremarkable motion the whole way down to ~260-330us.
#define ABS_FLOOR_US 80
#define ABS_CEIL_US 3100
#define CENTER_US 1500          // pulse commanded at the start of calibration
#define CAL_SETTLE_TIMEOUT_MS 3000   // exit route 1 of every waiting state: give up if it never settles
#define CAL_SETTLE_WINDOW_SAMPLES 10 // N -- how many recent raw readings the running average is over (to tune later)
#define CAL_SETTLE_WINDOW_COUNTS 2   // max |current raw - running average| to call it settled (to tune later)
#define CAL_POSITION_FILTER_SAMPLES 10   // M -- how many recent raw readings the reported "filtered position" is averaged over (to tune later)

// STATE_CAL_DOWN_*/STATE_CAL_UP_* run both the coarse and fine passes --
// same states, same edge-detection logic either way, just a different
// step size and starting position (currentStepUs/currentPhase below), the
// same way the Python host's find_edge() reuses one scan_until_weak() for
// both. Coarse finds the edge's rough location fast; fine (once the phase
// switches, see beginFinePass()) backs up by CAL_FINE_MARGIN_US and
// re-scans slowly across just that margin for a precise result.
#define CAL_COARSE_STEP_US 50            // coarse-pass step size (to tune later)
#define CAL_FINE_STEP_US 5               // fine-pass step size (to tune later)
#define CAL_FINE_MARGIN_US 100           // how far back (toward center) from the coarse edge the fine pass starts,
                                          // AND how much ground it covers -- the fine pass always walks this whole
                                          // margin (see CAL_FINE_SWEEP_STEPS below), not just a safety cushion
                                          // before an early exit (to tune later)

// The coarse-sweep edge check compares SETTLED POSITIONS, one sample per
// step, not a per-tick reading -- see updateStepDelta()'s own comment for
// why. Its stall/reversal checks compare a raw per-step count (a step's
// net displacement means the same thing regardless of how many ticks it
// took to settle, so no LOOP_FREQUENCY_HZ-derived scaling is needed
// there); its anomaly check compares a step *rate* instead (counts per
// commanded microsecond) specifically so the same baseline stays valid
// across a coarse<->fine step-size change -- see stepRateBuffer's own
// comment.
#define CAL_STEP_DELTA_WINDOW_SAMPLES 10 // how many recent step rates the "normal step" baseline is averaged over (to tune later)
#define CAL_STEP_JUMP_WINDOW 3.0f        // how much LARGER than the baseline (counts/us) a step rate has to be
                                          // before we call it an oversized jump -- one-directional on purpose, see
                                          // updateStepDelta()'s own comment: real hardware showed a step being
                                          // SMALLER than baseline (weakening) isn't a reliable "found the edge"
                                          // signal on its own (500us measured rate ~0.34 against baseline ~1.4-1.5,
                                          // diff ~1.1, yet the real hard stop wasn't until 450us); stepDeltaStall
                                          // (absolute, separate) is what catches that. This is only for the other
                                          // extreme -- a step moving much farther than normal (to tune further)
#define CAL_STEP_STALL_COUNTS 2          // max |step delta| itself (absolute, no baseline needed) before we call it a stall (to tune later)

// The fine pass always walks the *entire* CAL_FINE_MARGIN_US margin, this
// many fixed CAL_FINE_STEP_US steps, rather than stopping at the first
// stalled reading -- that was the actual design: CAL_FINE_MARGIN_US sets
// how much ground the fine pass covers, not just a safety cushion before
// a trigger-happy exit. A stall along the way is noted (see
// lastNormalFineUs) and swept straight through; only an unambiguous
// reversed/anomaly ends it early, same as this project's Python-host
// history did it. Integer division is exact here -- both operands are
// tuned together (to tune later).
#define CAL_FINE_SWEEP_STEPS (CAL_FINE_MARGIN_US / CAL_FINE_STEP_US)

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
  // A deliberately dumb sweep -- write, wait for settle, write the next
  // step, repeat, all the way from rawSweepStartUs to rawSweepEndUs, no
  // stall/anomaly/reversal logic anywhere in it. For getting a raw,
  // unfiltered ground truth from real hardware when the smart detection's
  // own result is in doubt -- see CMD_RAWSWEEP.
  STATE_RAW_SWEEP_WRITE,
  STATE_RAW_SWEEP_WAIT,
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
  CMD_CAL,
  CMD_ABORT,
  CMD_ACCEL,
  CMD_VEL,
  CMD_POS,
  CMD_MODEL,
  CMD_GO,
  CMD_STOPAFTER,
  CMD_RAWSWEEP,
};

// Every error the firmware can raise, in one place -- throwError() below
// maps one of these to its wire string via errorCodeToString().
enum ErrorCode {
  ERR_CAL_TIMEOUT,       // a calibration wait (CENTER/DOWN/UP/TABLE) timed out
  ERR_TRAJ_TIMEOUT,      // STATE_TRAJ_WAIT timed out -- streamed setpoints but never settled
  ERR_RECOVERY_FAILED,   // STATE_CAL_RECOVER_WAIT ran out of candidates -- see its own comment
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
bool rawSweepRunning = false;      // same idea again, for CMD_RAWSWEEP
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
    // Landing on STATE_IDLE always means neither a calibration nor a
    // trajectory run is still in progress, regardless of which path got
    // us here (abort, timeout) -- each success path clears its own flag
    // explicitly instead, since those end on STATE_CAL_DONE/STATE_TRAJ_
    // DONE, not here.
    calibrationRunning = false;
    trajectoryRunning = false;
    rawSweepRunning = false;
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
         s == STATE_CAL_UP_WAIT || s == STATE_CAL_TABLE_WAIT || s == STATE_TRAJ_WAIT ||
         s == STATE_RAW_SWEEP_WAIT;
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
    case STATE_RAW_SWEEP_WRITE: return "RAW_SWEEP_WRITE";
    case STATE_RAW_SWEEP_WAIT:  return "RAW_SWEEP_WAIT";
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
// WAIT, not a per-tick stream). STATE_RAW_SWEEP_* included too -- same
// "print every tick of a multi-step hardware test" need, name's just a
// holdover from before that existed. The list loop() checks against for
// the per-tick printProgress() call.
bool isCalibrationState(CalibrationState s) {
  return s == STATE_CAL_CENTER || s == STATE_CAL_DOWN_WRITE || s == STATE_CAL_DOWN_WAIT ||
         s == STATE_CAL_RECOVER_WAIT ||
         s == STATE_CAL_UP_CENTER || s == STATE_CAL_UP_WRITE || s == STATE_CAL_UP_WAIT ||
         s == STATE_CAL_TABLE_WRITE || s == STATE_CAL_TABLE_WAIT ||
         s == STATE_RAW_SWEEP_WRITE || s == STATE_RAW_SWEEP_WAIT;
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

int currentStepUs = CAL_COARSE_STEP_US;   // step size STATE_CAL_DOWN_WRITE/STATE_CAL_UP_WRITE use --
                                           // CAL_COARSE_STEP_US or CAL_FINE_STEP_US, set by
                                           // beginCoarsePass()/beginFinePass() further down --
                                           // declared up here since updateStepDelta() below needs
                                           // it (to normalize a step rate) before either of those
                                           // functions is defined
ScanPhase currentPhase = PHASE_COARSE;

TestStage stopAfterStage = STAGE_ALL;   // set by CMD_STOPAFTER -- see TestStage's own comment

int rawSweepEndUs = 0;
int rawSweepStepUs = 0;         // signed -- negative sweeps down, positive sweeps up

// Running-average baseline over the last CAL_STEP_DELTA_WINDOW_SAMPLES
// step RATES (counts moved per commanded microsecond, not a raw count) --
// "how fast does a normal step actually move the shaft, per us
// commanded." Rate, not raw count, specifically so ONE continuous
// baseline stays valid whether the step that produced it was a 50us
// coarse step or a 5us fine one: a real, un-anomalous response moves
// roughly the same distance per commanded microsecond either way, even
// though the raw per-step count differs 10x between them. Built up
// continuously across the *whole* calibration run -- coarse, fine, down,
// up, all sharing one baseline -- and only ever reset by CENTER's own
// settle (see its case body), which is the one point that's genuinely
// the start of a new run, not by beginCoarsePass()/beginFinePass()
// switching resolution or direction. There's no real reason to throw
// away a perfectly good baseline just because the step size changed.
float stepRateBuffer[CAL_STEP_DELTA_WINDOW_SAMPLES];
int stepRateBufferCount = 0;
int stepRateBufferIndex = 0;
float stepRateBufferSum = 0.0f;
float stepRateAverage = 0.0f;

void updateStepRateAverage(float currentRate) {
  if (stepRateBufferCount < CAL_STEP_DELTA_WINDOW_SAMPLES) {
    stepRateBufferSum += currentRate;
    stepRateBuffer[stepRateBufferCount] = currentRate;
    stepRateBufferCount++;
  } else {
    stepRateBufferSum -= stepRateBuffer[stepRateBufferIndex];
    stepRateBufferSum += currentRate;
    stepRateBuffer[stepRateBufferIndex] = currentRate;
    stepRateBufferIndex = (stepRateBufferIndex + 1) % CAL_STEP_DELTA_WINDOW_SAMPLES;
  }
  stepRateAverage = stepRateBufferSum / stepRateBufferCount;
}

long lastSettledPosition = 0;    // filteredPosition as of the most recent settle this run
bool stepDeltaPrevSettled = false;   // isSettled as of the last call, to catch the settle-edge below
bool stepDeltaAnomaly = false;   // true for exactly the tick a settle's step rate was flagged as an oversized jump
bool stepDeltaStall = false;     // true for exactly the tick a settle's step delta was flagged as ~zero motion
bool stepDeltaReversed = false;  // true for exactly the tick a settle's step delta went the wrong direction

int stepDirection = -1;   // +1 while sweeping up, -1 while sweeping down -- set explicitly at
                           // CENTER's own transition and at the down-edge -> up-sweep handoff
                           // (see their case bodies), read only by the reversal check below

bool skipNextStepCheck = false;   // set by beginFinePass() -- see its own comment for why the
                                   // margin back-off's own settle is judged by neither of the
                                   // three checks below, nor fed into the rate baseline, but
                                   // still updates lastSettledPosition so the very next (real,
                                   // uniform-size) step compares against the right reference

// Evaluated once per settle, not once per tick -- checking a raw per-tick
// derivative would compare noisy in-flight motion against noisy in-flight
// motion; checking net displacement between two settled positions is the
// same comparison the Python host made (one delta per commanded move, not
// per tick), and it's what actually distinguishes a normal step from a
// stall, a reversal, or an oversized jump.
//
// Three independent checks, not one -- they catch different things, need
// different amounts of history, and one doesn't imply another:
//   - stepDeltaStall: |stepDelta| itself is ~0 -- a hard stop, encoder
//     just isn't moving. Absolute, needs no baseline at all, live from
//     the very first step of the whole run.
//   - stepDeltaReversed: stepDelta's sign disagrees with stepDirection.
//     Also needs no baseline -- just the sign, known from tick one.
//   - stepDeltaAnomaly: an oversized same-direction jump -- a step rate
//     far from the established baseline. This one does need
//     CAL_STEP_DELTA_WINDOW_SAMPLES of history before it means anything,
//     same as before; the baseline being continuous now (not reset per
//     phase) just means that history builds up once, at the start of the
//     run, rather than being thrown away and rebuilt every time the step
//     size changes.
// Any one of the three is how STATE_CAL_DOWN_WAIT/STATE_CAL_UP_WAIT
// decide an edge was hit -- what that means (back up and refine, vs. the
// real edge) depends on currentPhase, decided in the case body, not here.
//
// All three assume every judged step is a *uniform* size (either
// currentStepUs's coarse or fine value, never a one-off hybrid) -- real
// hardware showed why that assumption matters: beginFinePass()'s margin
// back-off used to get folded into the same commanded move as the first
// fine step, so that "step" actually moved ~20x currentStepUs. Both
// stepDirection (a fixed per-sweep sign) and the rate normalization
// (dividing by currentStepUs) silently assume the move's real size and
// direction match currentStepUs -- which is only true again once the
// margin move is its own separate, unjudged settle (skipNextStepCheck
// above), not combined with the step that follows it.
void updateStepDelta(long currentPos) {
  bool justSettled = isSettled && !stepDeltaPrevSettled;
  stepDeltaPrevSettled = isSettled;
  stepDeltaAnomaly = false;
  stepDeltaStall = false;
  stepDeltaReversed = false;
  if (!justSettled) {
    return;
  }

  long stepDelta = currentPos - lastSettledPosition;
  lastSettledPosition = currentPos;

  if (skipNextStepCheck) {
    skipNextStepCheck = false;
    return;
  }

  long absStepDelta = (stepDelta < 0) ? -stepDelta : stepDelta;
  stepDeltaStall = (absStepDelta <= CAL_STEP_STALL_COUNTS);

  bool actualNegative = (stepDelta < 0);
  bool expectedNegative = (stepDirection < 0);
  // guarded by !stepDeltaStall so ordinary +-1-count noise while
  // genuinely stalled never gets misread as a reversal
  stepDeltaReversed = (!stepDeltaStall) && (actualNegative != expectedNegative);

  // fabsf, not a signed rate: currentStepUs is a magnitude (always
  // positive), but stepDelta carries the sweep's own sign (negative going
  // down, positive going up) -- a signed rate would make the down sweep
  // fill the baseline with ~-1.2..-1.8 and the up sweep's first (and
  // every) step, at a perfectly normal +1.2..+1.8, look wildly anomalous
  // by comparison purely from the sign flip. Direction is already
  // checked separately and correctly by stepDeltaReversed above; this
  // check is only ever about magnitude.
  float stepRate = fabsf((float)stepDelta / (float)currentStepUs);
  if (stepRateBufferCount >= CAL_STEP_DELTA_WINDOW_SAMPLES) {
    // One-directional on purpose -- only a rate SUBSTANTIALLY LARGER than
    // baseline counts as an anomaly now. Real hardware showed a smaller
    // rate isn't a reliable "found the edge" signal on its own: a step
    // can weaken well before the real edge (500us measured ~0.34 counts/us
    // against a ~1.4-1.5 baseline, yet the actual hard stop -- a genuine
    // stepDeltaStall -- wasn't until 450us, one more coarse step down).
    // stepDeltaStall (absolute, unrelated to this baseline) is what
    // catches a real stop; this check is specifically for the other
    // extreme this project has seen on real hardware -- a step moving
    // much FARTHER than normal (the servo losing its mechanical
    // reference and spinning past its limit instead of stalling).
    stepDeltaAnomaly = (stepRate - stepRateAverage > CAL_STEP_JUMP_WINDOW);
  }

  // Only feed a genuinely normal step into the baseline -- a stall,
  // reversal, or already-flagged anomaly is exactly the kind of sample
  // that would drag "what does normal look like" toward the edge-
  // transition zone itself. Real hardware showed this happen: the down
  // edge's own weakening/stall samples got folded into the same
  // continuous baseline the up sweep judged its first steps against, so
  // a genuinely normal up-sweep step (rate consistent with the whole rest
  // of the sweep) looked anomalous purely because the baseline had
  // already been dragged down by the down edge's near-zero rates.
  if (!stepDeltaStall && !stepDeltaAnomaly && !stepDeltaReversed) {
    updateStepRateAverage(stepRate);
  }
}

// Seeds lastSettledPosition/stepDeltaPrevSettled from wherever we just
// settled, and clears the step-rate baseline -- called exactly once,
// from CENTER's own case body, since that's the one point that's
// genuinely the start of a new calibration run (or the start of a fresh
// run after an earlier abort). beginCoarsePass()/beginFinePass() do NOT
// call this -- see the step-rate buffer's own comment for why a
// resolution or direction change isn't a reason to throw the baseline
// away. Does NOT call changeState() -- that's always done explicitly by
// the case body that calls this, right after calling it, so every
// transition is visible directly in the switch rather than hidden inside
// a setup function.
void ResetStates() {
  lastSettledPosition = filteredPosition;
  stepDeltaPrevSettled = true;
  stepRateBufferCount = 0;
  stepRateBufferIndex = 0;
  stepRateBufferSum = 0.0f;
}

// Switches to full-size steps. Called at CENTER -> down sweep, and again
// once the down side's fine pass finds the real min (coarse restarts for
// the up sweep) -- does NOT touch the step-rate baseline either time
// (see ResetStates()'s own comment).
void beginCoarsePass() {
  currentStepUs = CAL_COARSE_STEP_US;
  currentPhase = PHASE_COARSE;
}

// How many fine steps have been taken so far this fine pass -- counted in
// STATE_CAL_DOWN_WRITE/STATE_CAL_UP_WRITE, compared against
// CAL_FINE_SWEEP_STEPS to know when the fixed margin has been fully
// walked. Reset to 0 by beginFinePass().
int fineStepsTaken = 0;

// The last pulse, during the current fine pass, that actually moved
// normally (settled with a real, un-stalled step) -- the real reported
// edge once the fine sweep ends, NOT wherever the sweep happened to stop.
// Seeded to coarseConfirmedUs by beginFinePass() (the one point already
// proven-good going in) and only ever advances further in the sweep's own
// direction from there (each fine step moves strictly further from
// center than the last), so it can never regress to something less
// extreme than coarseConfirmedUs -- the min/maxUs clamp against
// coarseConfirmedUs below is a defensive backstop, not something this
// should ever actually need to correct.
int lastNormalFineUs = 0;

// The last coarse pulse that actually moved normally -- captured right
// before beginFinePass() is called (currentStepUs is still the coarse
// pass's own step size at that point), one pull-back-by-a-single-step
// away from lastSentUs (the coarse pulse that just stalled/reversed/
// anomalied). This is ground truth the coarse pass itself already
// established: the servo demonstrably moved normally out to here. The
// fine pass exists to refine that into a tighter number, never to
// contradict it -- see lastNormalFineUs and its use in
// STATE_CAL_DOWN_WAIT/STATE_CAL_UP_WAIT below.
int coarseConfirmedUs = 0;

// lastSentUs at the moment a STATE_CAL_DOWN_WAIT/STATE_CAL_UP_WAIT settle-
// timeout is treated as a servo-triggered spin/stall-recovery event (see
// the generic timeout check in loop()) -- captured there, before recovery
// overwrites lastSentUs with a safe candidate, so the coarse-phase
// coarseConfirmedUs math (which needs to know which pulse just failed)
// still has the right input.
int lastAttemptedUs = 0;

// Set once STATE_CAL_RECOVER_WAIT settles at a safe candidate -- consumed
// by STATE_CAL_DOWN_WAIT/STATE_CAL_UP_WAIT as a fourth "found the edge"
// signal, alongside stepDeltaStall/stepDeltaAnomaly/stepDeltaReversed.
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
// Candidate 0 is the nearest point already directly confirmed to move
// normally (coarseConfirmedUs mid-coarse-phase, lastNormalFineUs mid-
// fine-phase) -- close by, most likely to work, no long trip back.
// Candidate 1 is CENTER_US, always safe by construction (the very first
// point every run visits) -- the fallback if even the nearest candidate
// somehow also fails to settle. Mirrors this project's own ServoDAQ
// sibling tool's recover_from_wrap(), which needed the same "ordered list
// of fallback candidates" once a single nearest point wasn't always
// enough there either.
int recoveryCandidateUs(int index) {
  if (index == 0) {
    return (currentPhase == PHASE_COARSE) ? coarseConfirmedUs : lastNormalFineUs;
  }
  return CENTER_US;
}

// Called from STATE_CAL_DOWN_WAIT/STATE_CAL_UP_WAIT when a coarse-pass
// edge hit -- not the real edge yet, just its rough location. Backs the
// last-commanded pulse up by marginAdjustmentUs (positive to move back up
// toward center for the down side, negative to move back down for the up
// side) and switches to fine steps from there.
//
// Writes the margin back-off as ITS OWN move, not combined with the first
// fine step -- real hardware showed why that combining was wrong: it made
// that one "step" actually move ~20x currentStepUs (margin + one fine
// step), which broke both the direction check (the margin dominates,
// often in the OPPOSITE sense of the sweep's own stepDirection, so a
// perfectly normal response got misread as reversed) and the rate
// normalization (dividing a ~100us-produced delta by a 5us currentStepUs).
// The caller transitions to the matching *_WAIT state after calling this,
// same as any other write; skipNextStepCheck (consumed by
// updateStepDelta()) means that WAIT's settle is watched for timeout the
// same as always, just not judged by stall/reversed/anomaly or folded
// into the rate baseline -- once it settles, the caller falls through to
// its own isSettled branch and takes the real first fine step normally,
// now a uniform, cleanly-judged currentStepUs move like every step after
// it.
void beginFinePass(int marginAdjustmentUs) {
  currentStepUs = CAL_FINE_STEP_US;
  currentPhase = PHASE_FINE;
  skipNextStepCheck = true;
  fineStepsTaken = 0;
  lastNormalFineUs = coarseConfirmedUs;
  writeServoUs(lastSentUs + marginAdjustmentUs);
}

int minUs = 0;   // set once the fine pass ends -- see lastNormalFineUs
int maxUs = 0;   // set once the fine pass ends -- see lastNormalFineUs

CalPoint calTable[CAL_TABLE_POINTS];
int tableIndex = 0;   // which calTable entry STATE_CAL_TABLE_WRITE/STATE_CAL_TABLE_WAIT is on

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

// Called from STATE_CAL_TABLE_WAIT once a table point settles.
// totalCounts (continuous, unwrapped -- see updatePositionTracking())
// -> centidegrees via plain integer math (exact ratio 36000/4096 -- no
// float needed for a one-shot conversion like this, though it'd hardly
// matter at only CAL_TABLE_POINTS calls). No truncating cast to int16_t
// here -- angleCentideg is int32_t precisely so this doesn't need one.
// Index progression is the caller's job now (STATE_CAL_TABLE_WAIT), since
// which direction "next" means depends on tableSweepDown.
void recordTableEntry() {
  int32_t angleCentideg = filteredPosition * 36000L / 4096L;
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

  char buf[40];
  // %ld for angleCentideg -- it's int32_t/long now, not int; a bare %d
  // there would read the wrong bytes off the varargs stack on AVR (16-bit
  // int).
  snprintf(buf, sizeof(buf), "TABLE %d %d %ld", tableIndex,
           calTable[tableIndex].pulseUs, calTable[tableIndex].angleCentideg);
  printMessage(buf);
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
  if (strcmp(cmdTok, "CAL") == 0) return CMD_CAL;
  if (strcmp(cmdTok, "ABORT") == 0) return CMD_ABORT;
  if (strcmp(cmdTok, "ACCEL") == 0) return CMD_ACCEL;
  if (strcmp(cmdTok, "VEL") == 0) return CMD_VEL;
  if (strcmp(cmdTok, "POS") == 0) return CMD_POS;
  if (strcmp(cmdTok, "MODEL") == 0) return CMD_MODEL;
  if (strcmp(cmdTok, "GO") == 0) return CMD_GO;
  if (strcmp(cmdTok, "STOPAFTER") == 0) return CMD_STOPAFTER;
  if (strcmp(cmdTok, "RAWSWEEP") == 0) return CMD_RAWSWEEP;
  return CMD_UNKNOWN;
}

const int MAX_TOKENS = 4;   // command + up to 3 arguments (CMD_RAWSWEEP's startUs/endUs/stepUs)
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
// While calibrationRunning or trajectoryRunning, every command except
// ABORT is rejected outright -- nothing overlaps a run in progress.
// Anything unrecognized, or with the wrong number of arguments, is
// rejected too.
void handleLine(char* line) {
  int n = tokenizeLine(line);
  if (n == 0) return;

  SerialCommand cmd = parseCommand(tok[0]);
  if ((calibrationRunning || trajectoryRunning || rawSweepRunning) && cmd != CMD_ABORT) {
    Serial.println(F("ERR BUSY"));
    return;
  }
  switch (cmd) {
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
    case CMD_RAWSWEEP:
      // RAWSWEEP <startUs> <endUs> <stepUs> -- no smart logic anywhere in
      // this: write startUs, wait for settle, then step by stepUs (its own
      // sign, not inferred) toward endUs, settle, repeat, until endUs is
      // reached or passed. Pure ground truth for when the smart
      // detection's own result is in doubt.
      if (n != 4) { Serial.println(F("ERR USAGE")); break; }
      rawSweepEndUs = atoi(tok[2]);
      rawSweepStepUs = atoi(tok[3]);
      rawSweepRunning = true;
      writeServoUs(atoi(tok[1]));
      changeState(STATE_RAW_SWEEP_WAIT);
      Serial.println(F("OK RAWSWEEP"));
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

  Serial.println(F("# ServoAutoCalibrator booting..."));

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

  if (isCalibrationState(currentState)) {   // every tick, the whole way through calibration -- see printProgress()
    printProgress();
  }

  // Gated to an active down/up pass specifically (coarse or fine, either
  // one) -- STATE_CAL_CENTER's own settle is not a pass step and must
  // never reach updateStepDelta() (ResetStates() already seeds the
  // baseline from that settle, or from the previous pass's last step,
  // wherever it's called). STATE_CAL_UP_CENTER *is* included, even though
  // its own settle isn't judged either (skipNextStepCheck, set before that
  // move is written -- see its call site) -- it still needs to reach
  // updateStepDelta() so lastSettledPosition gets re-anchored to the
  // settled-at-CENTER_US position. Real hardware showed why that matters:
  // without it, lastSettledPosition stays stale at the down edge's
  // position across the whole recenter move, so the first real coarse-up
  // step's delta is computed against a position from clear across the
  // sweep -- an enormous, spurious "jump" that silently ends the up pass
  // right on its first step (lands on STATE_CAL_DONE, which prints
  // nothing, so it looked like a hang rather than a false detection).
  if (currentState == STATE_CAL_DOWN_WRITE || currentState == STATE_CAL_DOWN_WAIT ||
      currentState == STATE_CAL_RECOVER_WAIT ||
      currentState == STATE_CAL_UP_CENTER ||
      currentState == STATE_CAL_UP_WRITE || currentState == STATE_CAL_UP_WAIT) {
    updateStepDelta(filteredPosition);
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
      lastAttemptedUs = lastSentUs;
      if (currentPhase == PHASE_COARSE) {
        // Same math STATE_CAL_DOWN_WAIT/STATE_CAL_UP_WAIT's own coarse
        // branch already uses on a plain stall -- undo the one coarse
        // step that just failed to find the last pulse that moved
        // normally, computed now while lastAttemptedUs still holds it
        // (lastSentUs is about to become the recovery candidate instead).
        coarseConfirmedUs = (stepDirection < 0) ? (lastAttemptedUs + currentStepUs)
                                                 : (lastAttemptedUs - currentStepUs);
      }
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
        // it's the one place the step-rate baseline gets seeded/cleared
        // (see ResetStates()'s own comment) and stepDirection set.
        ResetStates();
        stepDirection = -1;
        beginCoarsePass();
        changeState(STATE_CAL_DOWN_WRITE);
      }
      break;

    case STATE_CAL_DOWN_WRITE:
      // One-tick state: take the next step down, then immediately hand
      // off to STATE_CAL_DOWN_WAIT to watch it.
      lastSentUs -= currentStepUs;
      if (currentPhase == PHASE_FINE) {
        fineStepsTaken++;
      }
      writeServoUs(lastSentUs);
      changeState(STATE_CAL_DOWN_WAIT);
      break;

    case STATE_CAL_DOWN_WAIT:
      if (currentPhase == PHASE_FINE) {
        // The fine pass always walks the *entire* CAL_FINE_MARGIN_US
        // margin (CAL_FINE_SWEEP_STEPS fixed steps) rather than stopping
        // at the first stalled reading -- see CAL_FINE_SWEEP_STEPS's own
        // comment. A stall along the way is just noted (lastNormalFineUs
        // isn't advanced) and swept straight through; only an unambiguous
        // reversed move, oversized-jump anomaly, or recovered-from-spin
        // event (spinRecovered -- see loop()'s own comment) ends it
        // early -- none of those need repeating to mean something the
        // way one flat reading does.
        bool sweepDone = spinRecovered;
        spinRecovered = false;
        if (!sweepDone && (stepDeltaAnomaly || stepDeltaReversed)) {
          sweepDone = true;
        } else if (!sweepDone && isSettled) {
          // fineStepsTaken == 0 means this settle is the margin move
          // itself, not a real fine step -- skipNextStepCheck already
          // exempted it from judgment (stepDeltaStall reads false for it,
          // same as a genuinely normal step, so it can't be told apart
          // from one here), so it must not touch lastNormalFineUs or
          // count toward CAL_FINE_SWEEP_STEPS either.
          if (fineStepsTaken > 0) {
            if (!stepDeltaStall) {
              lastNormalFineUs = lastSentUs;   // this step actually moved
            }
            sweepDone = (fineStepsTaken >= CAL_FINE_SWEEP_STEPS);
          }
          if (!sweepDone) {
            changeState(STATE_CAL_DOWN_WRITE);
          }
        }
        if (sweepDone) {
          // The real down edge: the last pulse that actually moved, not
          // wherever the fixed sweep happened to end. Clamped against
          // coarseConfirmedUs as a defensive backstop -- see
          // lastNormalFineUs's own comment for why that should never
          // actually fire.
          minUs = lastNormalFineUs;
          if (minUs > coarseConfirmedUs) {
            minUs = coarseConfirmedUs;
          }
          if (stopAfterStage == STAGE_FINE_DOWN) {
            // isolating this one stage for testing -- see TestStage's own
            // comment. minUs is already recorded above; just stop here
            // instead of continuing on to the up sweep.
            calibrationRunning = false;
            changeState(STATE_CAL_DONE);
          } else {
            // Up sweep always starts from CENTER_US, not from wherever the
            // down sweep/fine pass left off -- see STATE_CAL_UP_CENTER's
            // own comment. stepDirection/beginCoarsePass() happen once that
            // return move has actually settled, not here. skipNextStepCheck
            // exempts this move's own settle from being judged as a step
            // (same mechanism beginFinePass()'s margin move uses) -- it
            // still updates lastSettledPosition, just doesn't flag this
            // huge jump as a stall/anomaly/reversal.
            skipNextStepCheck = true;
            writeServoUs(CENTER_US);
            changeState(STATE_CAL_UP_CENTER);
          }
        }
      } else if (spinRecovered || stepDeltaStall || stepDeltaAnomaly || stepDeltaReversed) {
        // coarse phase -- settled, but not with a normal step delta -- it
        // barely moved (stall), moved the wrong way (reversed), moved too
        // far from the established rate baseline (anomaly), or never
        // settled at all and had to be recovered from (spinRecovered --
        // see loop()'s own comment) -- see updateStepDelta()'s own
        // comment for what each of the first three needs. No debounce
        // here (unlike the fine pass above) -- coarse's result only sets
        // where the fine pass starts, not the final number.
        bool viaRecovery = spinRecovered;
        spinRecovered = false;
        if (stopAfterStage == STAGE_COARSE_DOWN) {
          // isolating this one stage for testing -- see TestStage's
          // own comment. Stop right here, don't start the fine pass.
          // calibrationRunning cleared explicitly, same as the normal
          // finish line in STATE_CAL_TABLE_WAIT -- this lands on
          // STATE_CAL_DONE too, not STATE_IDLE, so changeState() won't
          // clear it automatically.
          calibrationRunning = false;
          changeState(STATE_CAL_DONE);
        } else {
          if (!viaRecovery) {
            // rough location only -- back up and refine in fine steps.
            // coarseConfirmedUs: lastSentUs is the pulse that just
            // stalled/reversed/anomalied, currentStepUs is still the
            // coarse step size right here (beginFinePass() hasn't
            // switched it to fine yet) -- adding it back gives the
            // previous coarse pulse, the last one that moved normally.
            // Via recovery, coarseConfirmedUs is already set -- computed
            // in loop() at the moment the spin was detected, from
            // lastAttemptedUs (the pulse that failed), since lastSentUs
            // here is now the recovered safe pulse instead and this same
            // formula would give a nonsense answer applied to it.
            coarseConfirmedUs = lastSentUs + currentStepUs;
          }
          // beginFinePass() already wrote the margin move itself, so we
          // wait for THAT to settle (skipNextStepCheck means it won't be
          // judged) before the real first fine step, taken normally by
          // STATE_CAL_DOWN_WRITE once this WAIT's isSettled branch fires.
          beginFinePass(CAL_FINE_MARGIN_US);
          changeState(STATE_CAL_DOWN_WAIT);
        }
      } else if (isSettled) {
        // settled with a normal, valid coarse step -- keep sweeping
        changeState(STATE_CAL_DOWN_WRITE);
      }
      break;

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
      // fine pass's fixed sweep left it. writeServoUs(CENTER_US) and
      // skipNextStepCheck=true already
      // happened in the DOWN_WAIT branch that got us here, same one-shot-
      // write-then-wait pattern as every other transition -- this state
      // just watches it settle. It IS gated into updateStepDelta() (see
      // loop()'s own comment) so lastSettledPosition gets re-anchored
      // here, but skipNextStepCheck means this big jump itself is never
      // judged as a stall/anomaly/reversal -- exactly like beginFinePass()'s
      // margin move. The step-rate baseline (stepRateAverage) is left
      // alone here regardless (skipNextStepCheck's early return in
      // updateStepDelta() skips that too) -- same "continuously held
      // buffer" reasoning as beginCoarsePass()/beginFinePass() themselves.
      if (isSettled) {
        stepDirection = 1;
        beginCoarsePass();
        changeState(STATE_CAL_UP_WRITE);
      }
      break;

    case STATE_CAL_UP_WRITE:
      // Mirrors STATE_CAL_DOWN_WRITE -- += instead of -=.
      lastSentUs += currentStepUs;
      if (currentPhase == PHASE_FINE) {
        fineStepsTaken++;
      }
      writeServoUs(lastSentUs);
      changeState(STATE_CAL_UP_WAIT);
      break;

    case STATE_CAL_UP_WAIT:
      // Mirrors STATE_CAL_DOWN_WAIT throughout, see its own comments.
      if (currentPhase == PHASE_FINE) {
        bool sweepDone = spinRecovered;
        spinRecovered = false;
        if (!sweepDone && (stepDeltaAnomaly || stepDeltaReversed)) {
          sweepDone = true;
        } else if (!sweepDone && isSettled) {
          if (fineStepsTaken > 0) {
            if (!stepDeltaStall) {
              lastNormalFineUs = lastSentUs;
            }
            sweepDone = (fineStepsTaken >= CAL_FINE_SWEEP_STEPS);
          }
          if (!sweepDone) {
            changeState(STATE_CAL_UP_WRITE);
          }
        }
        if (sweepDone) {
          // Here "more conservative" means lower pulse, less travel --
          // the mirror image of STATE_CAL_DOWN_WAIT's own clamp.
          maxUs = lastNormalFineUs;
          if (maxUs < coarseConfirmedUs) {
            maxUs = coarseConfirmedUs;
          }
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
      } else if (spinRecovered || stepDeltaStall || stepDeltaAnomaly || stepDeltaReversed) {
        // mirrors STATE_CAL_DOWN_WAIT's own coarse branch, see its comments
        bool viaRecovery = spinRecovered;
        spinRecovered = false;
        if (stopAfterStage == STAGE_COARSE_UP) {
          // isolating this one stage for testing -- see TestStage's own
          // comment. Stop right here, don't start the fine pass. Same
          // calibrationRunning note as STATE_CAL_DOWN_WAIT's own branch.
          calibrationRunning = false;
          changeState(STATE_CAL_DONE);
        } else {
          if (!viaRecovery) {
            coarseConfirmedUs = lastSentUs - currentStepUs;
          }
          beginFinePass(-CAL_FINE_MARGIN_US);
          changeState(STATE_CAL_UP_WAIT);
        }
      } else if (isSettled) {
        changeState(STATE_CAL_UP_WRITE);
      }
      break;

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
      {
        // TEMPORARY diagnostic trace -- investigating reported jerky
        // motion on small moves. Remove once diagnosed.
        char dbuf[64];
        snprintf(dbuf, sizeof(dbuf), "TRAJSTEP %lu %ld %u", (unsigned long)(t * 1000), (long)angleCentideg, pulse);
        printMessage(dbuf);
      }
      if (!stillMoving) {
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

    case STATE_RAW_SWEEP_WRITE: {
      // One-tick state, no detection logic at all -- just: are we past
      // rawSweepEndUs yet (sign-aware, since rawSweepStepUs can be either
      // direction)? If so, stop. Otherwise take the next step and wait.
      int nextUs = lastSentUs + rawSweepStepUs;
      bool overshot = (rawSweepStepUs < 0) ? (nextUs < rawSweepEndUs) : (nextUs > rawSweepEndUs);
      if (overshot) {
        rawSweepRunning = false;
        changeState(STATE_IDLE);
      } else {
        writeServoUs(nextUs);
        changeState(STATE_RAW_SWEEP_WAIT);
      }
      break;
    }

    case STATE_RAW_SWEEP_WAIT:
      // Timeout exit route handled generically above, same as every other
      // waiting state. Settle alone (no stall/anomaly/reversed check --
      // this sweep doesn't stop early for anything but the timeout or
      // reaching rawSweepEndUs) is what moves it to the next step.
      if (isSettled) {
        changeState(STATE_RAW_SWEEP_WRITE);
      }
      break;
  }
}
