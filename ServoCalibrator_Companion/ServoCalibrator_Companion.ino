/*
  ServoCalibrator_Companion (v2 -- merged calibration + trajectory demo)

  Replaces the old ServoCalibrator_Companion (host-paced wizard executor)
  and TrajectoryDemo_Companion (compile-time-table trajectory demo) with
  one firmware that does both, in sequence: **characterize** the servo
  (find its real pulse range, build a 20-point calibration lookup table
  -- all on-device, one command, fully automated), then **drive/visualize**
  trajectories against that same characterization, live.

  Deliberately NOT a reimplementation of RCServoCalibration's installation
  wizard (horn install / direction test / fine trim / logical zero shift)
  -- this firmware is purely for characterization. It always works in the
  PHYSICAL frame, [0, maxAngleDeg], direction=+1, no logical remapping --
  "minimum is always 0, just like the table". Turning a measured
  range/table into an actual `RCServoMotorDriver`/`PCA9685MotorDriver`
  constructor (direction, logical zero, install offset) is left to
  whatever application code consumes this tool's output -- that's an
  application concern, not a characterization one.

  Protocol (115200 baud, one command per line, \n-terminated ASCII).
  Non-blocking reads (loop() must keep streaming telemetry at a fixed
  rate once calibrated, even between commands) EXCEPT during CALIBRATE
  itself, which is a deliberately blocking one-shot routine (like
  UMI's own examples/RCServoAutoCalibration) -- no other command can be
  serviced while it's running, same as a human would expect a physical
  range-finding sweep to occupy the board for its own duration.

    PING                               -> OK PONG

    CALIBRATE                          -> CALRESULT <maxAngleDeg> <minPulseUs> <maxPulseUs>
                                           <pulse0> <cdeg0> ... <pulse19> <cdeg19>
                                         | ERR CAL_FAILED <reason>
      Fully automated, on-device: stall-scans outward from center in both
      directions (sliding-window net-delta check, not consecutive-step --
      see stallScan()) to find the real safe pulse range without any
      hand-measurement, then sweeps that range twice (up, once down) to
      build a direction-averaged 20-point table -- same algorithm as
      UMI's examples/RCServoAutoCalibration, just triggered on-demand by
      this command instead of running once at boot. Takes 1-3 minutes.
      Progress is streamed throughout as "#"-prefixed log lines. On
      success, replaces whatever calibration (if any) was active before
      -- re-running this is how you recalibrate. <cdeg> = angle in
      hundredths of a degree, matching UMI's CalPoint.angleCentideg.

    GETTABLE                           -> CALRESULT ... (same shape as above)
                                         | ERR NOT_CALIBRATED
      Re-reports the currently active table/range without recalibrating
      -- lets the browser recover its state after a page reload/reconnect
      without a full physical recalibration, as long as the board itself
      hasn't been reset since.

    IMPORT <maxAngleDeg> <minPulseUs> <maxPulseUs>
           <pulse0> <cdeg0> ... <pulse19> <cdeg19>
                                       -> OK | ERR <msg>
      Loads a previously-exported table (see CALRESULT's shape -- the
      wire format is deliberately identical, so a saved CALRESULT line
      can be replayed verbatim as an IMPORT command) WITHOUT re-running
      the physical stall-scan/sweep. Still does one quick physical
      re-anchor (~2 small moves) to re-zero the AS5600 live reference for
      THIS session/mounting, since an imported table carries the pulse
      curve but not a live sensor zero reference. Validated the same way
      CALIBRATE's own result is (length/ordering/coverage/plausibility)
      before being accepted -- rejects and leaves any prior calibration
      untouched on failure.

    GO <targetDeg> <vMaxDegS> <aMaxDegS2>
                                       -> OK | ERR <msg>
    SQUARE <lowDeg> <highDeg> <periodS> <vMaxDegS> <aMaxDegS2>
                                       -> OK | ERR <msg>
    SINE <centerDeg> <amplitudeDeg> <freqHz>
                                       -> OK | ERR <msg>
    STOP                               -> OK
    MODEL LINEAR | TABLE               -> OK | ERR <msg>
      Same as the old TrajectoryDemo_Companion -- see that project's own
      history for the full design rationale (replanning-from-live-setpoint,
      post-traj square vs. no-traj sine, live model switching mid-motion).
      All of GO/SQUARE/SINE/MODEL require a calibration to exist first
      (CALIBRATE or IMPORT) -- ERR NOT_CALIBRATED otherwise, since
      targetDeg/model math is meaningless without a known range.

  Streamed continuously, ~50Hz, once calibrated (whether or not a move is
  active -- setpoint holds flat at rest):

    T,<t_ms>,<setpoint_deg>,<setpoint_vel_degs>,<actual_deg>,<mode>,<model>

  <mode> is IDLE/MOVE/SQUARE/SINE, <model> is LINEAR/TABLE -- both sent
  explicitly rather than inferred, same reasoning as before (a sine's
  velocity legitimately crosses zero without stopping; a live MODEL
  switch should show up in the stream itself, not require the host to
  remember its own last command).

  Lines starting with "#" are informational only -- boot banner,
  CALIBRATE progress -- not part of the command/response or telemetry
  protocol, safe to log-and-ignore.
*/

#include <Arduino.h>
#include <Wire.h>
#include <Servo.h>
#include <AS5600.h>
#include <ServoCalibrationTable.h>  // CalPoint, computeServoPulseUs() (linear branch only -- PROGMEM-safe), isPlausiblePulseUs()/roundClampToInt16()/CAL_TABLE_MIN_POINTS/MAX_POINTS (plain-value helpers, also PROGMEM-safe) -- see lookupPulseFromRamTable()/validateRamCalTable() below for why the TABLE-lookup path can't reuse this header's own table functions
#include <TrapezoidalProfile.h>
#include <string.h>

// Declared up front: arduino-cli's auto-generated function prototypes
// land right after the last #include, before this file's own struct
// definition further down -- same gotcha as UMI's own
// examples/RCServoAutoCalibration.
struct ScanResult {
  int pulseUs;
  float angleDeg;
  bool hitAbsBound;
};

const int SERVO_PIN = A3;
const unsigned long SAMPLE_MS = 20;  // 50Hz telemetry + control rate

// Hard safety bounds for CALIBRATE's stall-scan -- generous defaults
// covering the vast majority of hobby servos (same values UMI's
// examples/RCServoAutoCalibration uses). Never commanded outside this
// even if a stall is never detected.
const int ABS_FLOOR_US = 200;
const int ABS_CEIL_US  = 2900;
const int CENTER_US    = 1500;

const int STALL_STEP_US      = 10;
const int STALL_SETTLE_MS    = 200;
const uint8_t STALL_WINDOW   = 10;
const float STALL_PLATEAU_DEG = 0.5f;

const int SWEEP_STEP_US   = 15;
const int SWEEP_SETTLE_MS = 200;

const uint8_t CAL_TABLE_POINTS = 20;  // fixed size for both CALIBRATE's output and IMPORT's expected input -- the validated sweet spot from this project's own earlier characterization work, not an arbitrary choice

Servo servo;
AS5600 encoder;
TrapezoidalProfile profile;

// ------------------------------------------------------------------
// Calibration state. Nothing here is compile-time-fixed anymore (no
// GenTable.h) -- CALIBRATE or IMPORT populate all of it at runtime.
// ------------------------------------------------------------------
bool ready = false;        // true once a valid table+range exists
bool useTable = true;      // live-switchable via MODEL, once ready
float maxAngleDeg = 0.0f;
int minPulseUs = 0, maxPulseUs = 0;

// RAM-resident -- NOT PROGMEM. This table is built from live sensor
// sweeps at runtime, so (unlike every other CalPoint table in this
// project, e.g. the old GenTable.h) it can never be a compile-time
// PROGMEM array. UMI's own ServoCalibrationTable.h functions
// (lookupPulseUsFromTable()/validateCalTable()) always read through
// pgm_read_word(), which is real flash access on AVR's Harvard
// architecture -- pointing that at a RAM address would misread RAM as
// flash and return garbage (documented as a footgun in that header's own
// comments). lookupPulseFromRamTable()/validateRamCalTable() below are
// the minimal necessary re-implementation of that same algorithm against
// plain RAM instead -- everything else (the CalPoint type itself, the
// linear-formula path, the shared plausibility/rounding helpers) still
// comes straight from ServoCalibrationTable.h.
CalPoint calTable[CAL_TABLE_POINTS];
uint8_t calTableLen = 0;

// ------------------------------------------------------------------
// AS5600 ground truth, unwrapped running angle -- same wrap-safe
// small-delta accumulation technique as every earlier sketch in this
// project.
// ------------------------------------------------------------------
float lastRawDeg = 0.0f;
float runningAngle = 0.0f;
float zeroRefAngle = 0.0f;
float signConv = 1.0f;

float updateRunningAngle() {
  float raw = encoder.readAngle() * AS5600_RAW_TO_DEGREES;
  float delta = raw - lastRawDeg;
  while (delta > 180.0f) delta -= 360.0f;
  while (delta < -180.0f) delta += 360.0f;
  runningAngle += delta;
  lastRawDeg = raw;
  return runningAngle;
}
void resyncRunningAngle() {
  lastRawDeg = encoder.readAngle() * AS5600_RAW_TO_DEGREES;
}
float measuredDeg() {
  updateRunningAngle();
  return signConv * (runningAngle - zeroRefAngle);
}
float pollUntilSettled(unsigned long timeoutMs = 4000) {
  unsigned long start = millis();
  float prev = updateRunningAngle();
  int stableCount = 0;
  while (millis() - start < timeoutMs) {
    delay(40);
    float cur = updateRunningAngle();
    if (fabs(cur - prev) < 0.15f) {
      stableCount++;
      if (stableCount >= 3) return cur;
    } else {
      stableCount = 0;
    }
    prev = cur;
  }
  Serial.println(F("# WARNING: settle timed out"));
  return prev;
}

// ------------------------------------------------------------------
// RAM-safe table lookup + validation -- see the calTable[] comment above
// for why these can't just be UMI's own lookupPulseUsFromTable()/
// validateCalTable(). Otherwise identical algorithms (binary search +
// linear interpolation; length/ordering/coverage/plausibility checks),
// reusing UMI's own shared constants/helpers wherever those don't touch
// PROGMEM (isPlausiblePulseUs(), roundClampToInt16(),
// CAL_TABLE_MIN_POINTS/MAX_POINTS).
// ------------------------------------------------------------------
uint16_t lookupPulseFromRamTable(float angleDeg) {
  int16_t ac = (int16_t)(angleDeg * 100.0f + (angleDeg >= 0.0f ? 0.5f : -0.5f));
  if (ac <= calTable[0].angleCentideg) return calTable[0].pulseUs;
  if (ac >= calTable[calTableLen - 1].angleCentideg) return calTable[calTableLen - 1].pulseUs;

  uint8_t lo = 0, hi = calTableLen - 1;
  while (hi - lo > 1) {
    uint8_t mid = (lo + hi) / 2;
    if (calTable[mid].angleCentideg <= ac) lo = mid; else hi = mid;
  }
  // int32_t, not int/uint16_t -- same AVR 16-bit-int wraparound guard as
  // UMI's own lookupPulseUsFromTable(): a locally-decreasing pulseUs
  // across an angle-ascending step (permitted -- only angle order is
  // required strictly ascending) would otherwise wrap to a huge positive
  // value via unsigned promotion.
  int32_t aSpan = (int32_t)calTable[hi].angleCentideg - calTable[lo].angleCentideg;
  int32_t pSpan = (int32_t)calTable[hi].pulseUs - calTable[lo].pulseUs;
  float t = (aSpan != 0) ? (float)(ac - calTable[lo].angleCentideg) / (float)aSpan : 0.0f;
  float us = calTable[lo].pulseUs + t * (float)pSpan;
  if (us < 0.0f) us = 0.0f;
  if (us > 65535.0f) us = 65535.0f;
  return (uint16_t)(us + 0.5f);
}

bool validateRamCalTable(const CalPoint* table, uint8_t len, float maxAngleDegParam,
                          uint16_t& outMin, uint16_t& outMax) {
  if (len < CAL_TABLE_MIN_POINTS || len > CAL_TABLE_MAX_POINTS) return false;
  if (!isPlausiblePulseUs(table[0].pulseUs)) return false;
  if (table[0].angleCentideg > 0) return false;

  uint16_t minP = table[0].pulseUs, maxP = table[0].pulseUs;
  for (uint8_t i = 1; i < len; i++) {
    if (table[i].angleCentideg <= table[i - 1].angleCentideg) return false;
    if (!isPlausiblePulseUs(table[i].pulseUs)) return false;
    if (table[i].pulseUs < minP) minP = table[i].pulseUs;
    if (table[i].pulseUs > maxP) maxP = table[i].pulseUs;
  }
  int16_t maxAngleCentideg = roundClampToInt16(maxAngleDegParam * 100.0f);
  if (table[len - 1].angleCentideg < maxAngleCentideg) return false;

  outMin = minP; outMax = maxP;
  return true;
}

// Angle (deg, physical frame [0, maxAngleDeg]) -> pulse width (us).
// TABLE path: lookupPulseFromRamTable() above. LINEAR path: UMI's own
// computeServoPulseUs() with calTable=nullptr -- that branch never
// touches PROGMEM, so it's safe to call directly.
uint16_t pulseForDeg(float deg) {
  float us;
  if (useTable && calTableLen > 0) {
    us = (float)lookupPulseFromRamTable(deg);
  } else {
    us = (float)computeServoPulseUs(radians(deg), nullptr, 0, radians(maxAngleDeg), minPulseUs, maxPulseUs);
  }
  if (us < minPulseUs) us = (float)minPulseUs;
  if (us > maxPulseUs) us = (float)maxPulseUs;
  return (uint16_t)us;
}
void driveTo(float deg) { servo.writeMicroseconds(pulseForDeg(deg)); }

// ------------------------------------------------------------------
// Setpoint state machine -- unchanged from TrajectoryDemo_Companion.
// ------------------------------------------------------------------
enum Mode { MODE_IDLE, MODE_MOVE, MODE_SQUARE, MODE_SINE };
Mode mode = MODE_IDLE;
const char* modeName() {
  switch (mode) {
    case MODE_MOVE:   return "MOVE";
    case MODE_SQUARE: return "SQUARE";
    case MODE_SINE:   return "SINE";
    default:          return "IDLE";
  }
}

unsigned long moveStartMs = 0;
float holdSetpointDeg = 0.0f;
unsigned long streamStartMs = 0;
unsigned long nextSampleMs = 0;

float sqLowDeg = 0.0f, sqHighDeg = 0.0f, sqVMax = 0.0f, sqAMax = 0.0f;
unsigned long sqPeriodMs = 0;
unsigned long sqLegStartMs = 0;
bool sqGoingHigh = true;

float sinCenterDeg = 0.0f, sinAmplitudeDeg = 0.0f, sinFreqHz = 0.0f;
unsigned long sinStartMs = 0;
bool sinePending = false;

float currentSetpointDeg(float* velOut = nullptr) {
  if (mode == MODE_SINE) {
    float t = (millis() - sinStartMs) / 1000.0f;
    float w = 2.0f * PI * sinFreqHz;
    if (velOut) *velOut = sinAmplitudeDeg * w * cos(w * t);
    return sinCenterDeg + sinAmplitudeDeg * sin(w * t);
  }

  if (mode == MODE_SQUARE) {
    unsigned long now = millis();
    if (now - sqLegStartMs >= sqPeriodMs / 2) {
      sqLegStartMs = now;
      sqGoingHigh = !sqGoingHigh;
      float pos0, vel0, accel0;
      profile.evaluate((now - moveStartMs) / 1000.0f, pos0, vel0, accel0);
      TrajectoryLimits limits;
      limits.vMax = sqVMax; limits.aMax = sqAMax; limits.jMax = 0.0f;
      profile.plan(pos0, sqGoingHigh ? sqHighDeg : sqLowDeg, limits);
      moveStartMs = now;
    }
    float pos, vel, accel;
    profile.evaluate((now - moveStartMs) / 1000.0f, pos, vel, accel);
    if (velOut) *velOut = vel;
    return pos;
  }

  if (mode == MODE_MOVE) {
    float t = (millis() - moveStartMs) / 1000.0f;
    float pos, vel, accel;
    bool inMotion = profile.evaluate(t, pos, vel, accel);
    if (!inMotion) {
      holdSetpointDeg = pos;
      if (sinePending) {
        mode = MODE_SINE;
        sinStartMs = millis();
        sinePending = false;
        float w = 2.0f * PI * sinFreqHz;
        if (velOut) *velOut = sinAmplitudeDeg * w;
        return sinCenterDeg;
      }
      mode = MODE_IDLE;
      if (velOut) *velOut = 0.0f;
      return pos;
    }
    if (velOut) *velOut = vel;
    return pos;
  }

  if (velOut) *velOut = 0.0f;
  return holdSetpointDeg;
}

// ------------------------------------------------------------------
// Non-blocking line-based command reader.
// ------------------------------------------------------------------
const int LINE_BUF_LEN = 320;  // must fit IMPORT's full line: ~44 tokens worth of numbers
char lineBuf[LINE_BUF_LEN];
int lineLen = 0;

const int MAX_TOKENS = 48;  // IMPORT: 1(cmd) + 3(header) + 2*CAL_TABLE_POINTS(40) = 44, +headroom
char* tok[MAX_TOKENS];
int tokenize(char* line) {
  int n = 0;
  char* p = line;
  while (*p && n < MAX_TOKENS) {
    while (*p == ' ') p++;
    if (!*p) break;
    tok[n++] = p;
    while (*p && *p != ' ') p++;
    if (*p) { *p = '\0'; p++; }
  }
  return n;
}

// ------------------------------------------------------------------
// CALIBRATE -- adapted from UMI's examples/RCServoAutoCalibration,
// triggered on-demand instead of running once at boot.
// ------------------------------------------------------------------
float targetAngleRaw[CAL_TABLE_POINTS];
uint16_t upPulseAtTarget[CAL_TABLE_POINTS];
uint16_t downPulseAtTarget[CAL_TABLE_POINTS];

bool between(float target, float a, float b) {
  float lo = (a < b) ? a : b;
  float hi = (a < b) ? b : a;
  return target >= lo && target <= hi;
}

ScanResult stallScan(int startUs, int stepUs) {
  float window[STALL_WINDOW];
  int windowPulse[STALL_WINDOW];
  uint8_t windowLen = 0, windowHead = 0;

  int pulse = startUs;
  servo.writeMicroseconds(pulse);
  delay(400);
  float angle = pollUntilSettled();

  window[windowHead] = angle;
  windowPulse[windowHead] = pulse;
  windowHead = (windowHead + 1) % STALL_WINDOW;
  windowLen = 1;

  ScanResult result = { pulse, angle, false };

  while (true) {
    int nextPulse = pulse + stepUs;
    if (stepUs < 0 && nextPulse < ABS_FLOOR_US) { result.hitAbsBound = true; break; }
    if (stepUs > 0 && nextPulse > ABS_CEIL_US)  { result.hitAbsBound = true; break; }

    servo.writeMicroseconds(nextPulse);
    delay(STALL_SETTLE_MS);
    pulse = nextPulse;
    angle = updateRunningAngle();

    window[windowHead] = angle;
    windowPulse[windowHead] = pulse;
    windowHead = (windowHead + 1) % STALL_WINDOW;
    if (windowLen < STALL_WINDOW) windowLen++;

    if (windowLen == STALL_WINDOW) {
      uint8_t oldestIdx = windowHead;
      float net = fabs(angle - window[oldestIdx]);
      if (net < STALL_PLATEAU_DEG) {
        result.pulseUs = windowPulse[oldestIdx];
        result.angleDeg = window[oldestIdx];
        return result;
      }
    }
    result.pulseUs = pulse;
    result.angleDeg = angle;
  }
  return result;
}

void sweepUp(int fromUs, int toUs) {
  servo.writeMicroseconds(fromUs);
  delay(400);
  float angle = pollUntilSettled();

  upPulseAtTarget[0] = fromUs;
  uint8_t nextIdx = 1;
  int prevPulse = fromUs;
  float prevAngle = angle;

  for (int pulse = fromUs + SWEEP_STEP_US; pulse <= toUs; pulse += SWEEP_STEP_US) {
    servo.writeMicroseconds(pulse);
    delay(SWEEP_SETTLE_MS);
    float cur = updateRunningAngle();

    while (nextIdx < CAL_TABLE_POINTS - 1 && between(targetAngleRaw[nextIdx], prevAngle, cur)) {
      float span = cur - prevAngle;
      float t = (fabs(span) > 1e-4f) ? (targetAngleRaw[nextIdx] - prevAngle) / span : 0.0f;
      upPulseAtTarget[nextIdx] = (uint16_t)(prevPulse + t * (pulse - prevPulse) + 0.5f);
      nextIdx++;
    }
    prevPulse = pulse;
    prevAngle = cur;
  }

  servo.writeMicroseconds(toUs);
  delay(SWEEP_SETTLE_MS);
  updateRunningAngle();
  upPulseAtTarget[CAL_TABLE_POINTS - 1] = toUs;
}

void sweepDown(int fromUs, int toUs) {
  servo.writeMicroseconds(fromUs);
  delay(400);
  float angle = pollUntilSettled();

  downPulseAtTarget[CAL_TABLE_POINTS - 1] = fromUs;
  int nextIdx = CAL_TABLE_POINTS - 2;
  int prevPulse = fromUs;
  float prevAngle = angle;

  for (int pulse = fromUs - SWEEP_STEP_US; pulse >= toUs; pulse -= SWEEP_STEP_US) {
    servo.writeMicroseconds(pulse);
    delay(SWEEP_SETTLE_MS);
    float cur = updateRunningAngle();

    while (nextIdx >= 1 && between(targetAngleRaw[nextIdx], prevAngle, cur)) {
      float span = cur - prevAngle;
      float t = (fabs(span) > 1e-4f) ? (targetAngleRaw[nextIdx] - prevAngle) / span : 0.0f;
      downPulseAtTarget[nextIdx] = (uint16_t)(prevPulse + t * (pulse - prevPulse) + 0.5f);
      nextIdx--;
    }
    prevPulse = pulse;
    prevAngle = cur;
  }

  servo.writeMicroseconds(toUs);
  delay(SWEEP_SETTLE_MS);
  updateRunningAngle();
  downPulseAtTarget[0] = toUs;
}

void reportCalResult() {
  Serial.print(F("CALRESULT "));
  Serial.print(maxAngleDeg, 3); Serial.print(' ');
  Serial.print(minPulseUs); Serial.print(' ');
  Serial.print(maxPulseUs);
  for (uint8_t i = 0; i < calTableLen; i++) {
    Serial.print(' '); Serial.print(calTable[i].pulseUs);
    Serial.print(' '); Serial.print(calTable[i].angleCentideg);
  }
  Serial.println();
}

void handleCalibrate() {
  mode = MODE_IDLE;
  sinePending = false;

  Serial.println(F("# CAL: centering..."));
  servo.writeMicroseconds(CENTER_US);
  delay(400);
  resyncRunningAngle();
  runningAngle = 0.0f;
  pollUntilSettled();
  Serial.println(F("# CAL: centered."));

  Serial.println(F("# CAL: scanning low limit..."));
  ScanResult low = stallScan(CENTER_US, -STALL_STEP_US);
  if (low.hitAbsBound) {
    Serial.println(F("ERR CAL_FAILED hit absolute floor without detecting a stall -- check wiring/power"));
    return;
  }
  Serial.print(F("# CAL: low done, pulse=")); Serial.print(low.pulseUs);
  Serial.print(F(" angle=")); Serial.println(low.angleDeg, 2);

  servo.writeMicroseconds(CENTER_US);
  pollUntilSettled();

  Serial.println(F("# CAL: scanning high limit..."));
  ScanResult high = stallScan(CENTER_US, STALL_STEP_US);
  if (high.hitAbsBound) {
    Serial.println(F("ERR CAL_FAILED hit absolute ceiling without detecting a stall -- check wiring/power"));
    return;
  }
  Serial.print(F("# CAL: high done, pulse=")); Serial.print(high.pulseUs);
  Serial.print(F(" angle=")); Serial.println(high.angleDeg, 2);

  int trueMinUs = low.pulseUs, trueMaxUs = high.pulseUs;
  float strokeDeg = fabs(high.angleDeg - low.angleDeg);
  Serial.print(F("# CAL: range ")); Serial.print(trueMinUs); Serial.print('-'); Serial.print(trueMaxUs);
  Serial.print(F("us, stroke=")); Serial.print(strokeDeg, 1); Serial.println(F("deg"));

  for (uint8_t i = 0; i < CAL_TABLE_POINTS; i++) {
    float f = (float)i / (float)(CAL_TABLE_POINTS - 1);
    targetAngleRaw[i] = low.angleDeg + f * (high.angleDeg - low.angleDeg);
  }

  Serial.println(F("# CAL: sweeping up..."));
  sweepUp(trueMinUs, trueMaxUs);
  Serial.println(F("# CAL: sweeping down..."));
  sweepDown(trueMaxUs, trueMinUs);

  for (uint8_t i = 0; i < CAL_TABLE_POINTS; i++) {
    float physicalDeg = strokeDeg * (float)i / (float)(CAL_TABLE_POINTS - 1);
    calTable[i].angleCentideg = (int16_t)(physicalDeg * 100.0f + 0.5f);
    calTable[i].pulseUs = (uint16_t)(((uint32_t)upPulseAtTarget[i] + downPulseAtTarget[i] + 1) / 2);
  }
  calTableLen = CAL_TABLE_POINTS;
  minPulseUs = trueMinUs; maxPulseUs = trueMaxUs; maxAngleDeg = strokeDeg;

  // zeroRef/signConv come straight out of the scan data itself -- no
  // separate probe move needed, unlike the old TrajectoryDemo_Companion's
  // boot sequence, since the stall-scan already visited both endpoints.
  zeroRefAngle = low.angleDeg;
  signConv = (high.angleDeg - low.angleDeg >= 0.0f) ? 1.0f : -1.0f;

  useTable = true;
  holdSetpointDeg = 0.0f;
  ready = true;
  streamStartMs = millis();
  nextSampleMs = streamStartMs;

  Serial.println(F("# CAL: done."));
  reportCalResult();
}

void handleGetTable() {
  if (!ready) { Serial.println(F("ERR NOT_CALIBRATED")); return; }
  reportCalResult();
}

void handleImport(int n) {
  const int expectedTokens = 1 + 3 + 2 * CAL_TABLE_POINTS;
  if (n != expectedTokens) {
    Serial.print(F("ERR USAGE: IMPORT <maxAngleDeg> <minPulseUs> <maxPulseUs> + "));
    Serial.print(CAL_TABLE_POINTS);
    Serial.println(F(" <pulse> <cdeg> pairs"));
    return;
  }

  float newMaxAngleDeg = atof(tok[1]);
  int newMin = atoi(tok[2]);
  int newMax = atoi(tok[3]);

  CalPoint tmp[CAL_TABLE_POINTS];
  for (uint8_t i = 0; i < CAL_TABLE_POINTS; i++) {
    tmp[i].pulseUs = (uint16_t)atoi(tok[4 + 2 * i]);
    tmp[i].angleCentideg = (int16_t)atoi(tok[5 + 2 * i]);
  }

  uint16_t vMin, vMax;
  if (!validateRamCalTable(tmp, CAL_TABLE_POINTS, newMaxAngleDeg, vMin, vMax)) {
    Serial.println(F("ERR INVALID_TABLE"));
    return;
  }

  memcpy(calTable, tmp, sizeof(tmp));
  calTableLen = CAL_TABLE_POINTS;
  maxAngleDeg = newMaxAngleDeg;
  minPulseUs = newMin; maxPulseUs = newMax;
  useTable = true;
  mode = MODE_IDLE;
  sinePending = false;

  // An imported table has no live sensor zero-reference of its own --
  // physically re-anchor: move to the table's own minPulseUs, settle,
  // zero there, then a small probe move to (re-)establish signConv. Much
  // quicker than a full CALIBRATE (a couple of small moves, not a whole
  // stall-scan+sweep) since the range/table itself is already known.
  Serial.println(F("# IMPORT: re-anchoring live zero reference..."));
  servo.writeMicroseconds(minPulseUs);
  delay(400);
  resyncRunningAngle();
  runningAngle = 0.0f;
  zeroRefAngle = pollUntilSettled();
  signConv = 1.0f;

  servo.writeMicroseconds(minPulseUs + (maxPulseUs - minPulseUs) / 10);
  float probe = pollUntilSettled();
  signConv = ((probe - zeroRefAngle) >= 0.0f) ? 1.0f : -1.0f;

  servo.writeMicroseconds(minPulseUs);
  pollUntilSettled();

  holdSetpointDeg = 0.0f;
  ready = true;
  streamStartMs = millis();
  nextSampleMs = streamStartMs;

  Serial.println(F("OK"));
}

// ------------------------------------------------------------------
// GO / SQUARE / SINE / STOP / MODEL -- unchanged from TrajectoryDemo_
// Companion except gated on `ready` (targetDeg/model math is meaningless
// without a known range) and referencing runtime maxAngleDeg/minPulseUs/
// maxPulseUs instead of compile-time constants.
// ------------------------------------------------------------------
void handleGo(int n) {
  if (!ready) { Serial.println(F("ERR NOT_CALIBRATED")); return; }
  if (n != 4) { Serial.println(F("ERR USAGE: GO <targetDeg> <vMaxDegS> <aMaxDegS2>")); return; }
  float target = atof(tok[1]);
  float vMax = atof(tok[2]);
  float aMax = atof(tok[3]);

  if (target < 0.0f || target > maxAngleDeg) {
    Serial.print(F("ERR OUT_OF_RANGE 0..")); Serial.println(maxAngleDeg, 2);
    return;
  }
  if (vMax <= 0.0f || aMax <= 0.0f) { Serial.println(F("ERR LIMITS_MUST_BE_POSITIVE")); return; }

  float q0 = currentSetpointDeg();
  TrajectoryLimits limits;
  limits.vMax = vMax; limits.aMax = aMax; limits.jMax = 0.0f;
  if (!profile.plan(q0, target, limits)) { Serial.println(F("ERR PLAN_FAILED")); return; }
  moveStartMs = millis();
  mode = MODE_MOVE;
  sinePending = false;
  Serial.println(F("OK"));
}

void handleSquare(int n) {
  if (!ready) { Serial.println(F("ERR NOT_CALIBRATED")); return; }
  if (n != 6) { Serial.println(F("ERR USAGE: SQUARE <lowDeg> <highDeg> <periodS> <vMaxDegS> <aMaxDegS2>")); return; }
  float lo = atof(tok[1]);
  float hi = atof(tok[2]);
  float periodS = atof(tok[3]);
  float vMax = atof(tok[4]);
  float aMax = atof(tok[5]);

  if (lo < 0.0f || lo > maxAngleDeg || hi < 0.0f || hi > maxAngleDeg) {
    Serial.print(F("ERR OUT_OF_RANGE 0..")); Serial.println(maxAngleDeg, 2);
    return;
  }
  if (hi <= lo) { Serial.println(F("ERR HIGH_MUST_EXCEED_LOW")); return; }
  if (periodS < 0.1f || periodS > 60.0f) { Serial.println(F("ERR PERIOD_OUT_OF_RANGE 0.1..60")); return; }
  if (vMax <= 0.0f || aMax <= 0.0f) { Serial.println(F("ERR LIMITS_MUST_BE_POSITIVE")); return; }

  float q0 = currentSetpointDeg();
  sqLowDeg = lo; sqHighDeg = hi; sqVMax = vMax; sqAMax = aMax;
  sqPeriodMs = (unsigned long)(periodS * 1000.0f);
  sqGoingHigh = true;

  TrajectoryLimits limits;
  limits.vMax = vMax; limits.aMax = aMax; limits.jMax = 0.0f;
  if (!profile.plan(q0, hi, limits)) { Serial.println(F("ERR PLAN_FAILED")); return; }
  moveStartMs = millis();
  sqLegStartMs = moveStartMs;
  mode = MODE_SQUARE;
  sinePending = false;
  Serial.println(F("OK"));
}

void handleSine(int n) {
  if (!ready) { Serial.println(F("ERR NOT_CALIBRATED")); return; }
  if (n != 4) { Serial.println(F("ERR USAGE: SINE <centerDeg> <amplitudeDeg> <freqHz>")); return; }
  float center = atof(tok[1]);
  float amp = atof(tok[2]);
  float freq = atof(tok[3]);

  if (amp <= 0.0f) { Serial.println(F("ERR AMPLITUDE_MUST_BE_POSITIVE")); return; }
  if (center - amp < 0.0f || center + amp > maxAngleDeg) {
    Serial.print(F("ERR OUT_OF_RANGE center+-amplitude must fit within 0.."));
    Serial.println(maxAngleDeg, 2);
    return;
  }
  if (freq <= 0.0f || freq > 5.0f) { Serial.println(F("ERR FREQ_OUT_OF_RANGE 0..5")); return; }

  sinCenterDeg = center; sinAmplitudeDeg = amp; sinFreqHz = freq;

  float q0 = currentSetpointDeg();
  TrajectoryLimits limits;
  limits.vMax = 60.0f; limits.aMax = 120.0f; limits.jMax = 0.0f;
  if (!profile.plan(q0, center, limits)) { Serial.println(F("ERR PLAN_FAILED")); return; }
  moveStartMs = millis();
  mode = MODE_MOVE;
  sinePending = true;
  Serial.println(F("OK"));
}

void handleStop() {
  holdSetpointDeg = currentSetpointDeg();
  mode = MODE_IDLE;
  sinePending = false;
  Serial.println(F("OK"));
}

void handleModel(int n) {
  if (!ready) { Serial.println(F("ERR NOT_CALIBRATED")); return; }
  if (n != 2) { Serial.println(F("ERR USAGE: MODEL LINEAR|TABLE")); return; }
  if (strcmp(tok[1], "LINEAR") == 0) {
    useTable = false;
    Serial.println(F("OK"));
  } else if (strcmp(tok[1], "TABLE") == 0) {
    useTable = true;
    Serial.println(F("OK"));
  } else {
    Serial.println(F("ERR USAGE: MODEL LINEAR|TABLE"));
  }
}

void handleLine(char* line) {
  int n = tokenize(line);
  if (n == 0) return;
  if (strcmp(tok[0], "PING") == 0) {
    Serial.println(F("OK PONG"));
  } else if (strcmp(tok[0], "CALIBRATE") == 0) {
    handleCalibrate();
  } else if (strcmp(tok[0], "GETTABLE") == 0) {
    handleGetTable();
  } else if (strcmp(tok[0], "IMPORT") == 0) {
    handleImport(n);
  } else if (strcmp(tok[0], "GO") == 0) {
    handleGo(n);
  } else if (strcmp(tok[0], "SQUARE") == 0) {
    handleSquare(n);
  } else if (strcmp(tok[0], "SINE") == 0) {
    handleSine(n);
  } else if (strcmp(tok[0], "STOP") == 0) {
    handleStop();
  } else if (strcmp(tok[0], "MODEL") == 0) {
    handleModel(n);
  } else {
    Serial.println(F("ERR UNKNOWN_CMD"));
  }
}

void readSerialNonBlocking() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (lineLen > 0) {
        lineBuf[lineLen] = '\0';
        handleLine(lineBuf);
        lineLen = 0;
      }
    } else if (lineLen < LINE_BUF_LEN - 1) {
      lineBuf[lineLen++] = c;
    }
  }
}

void setup() {
  Serial.begin(115200);
  while (!Serial) {}
  delay(300);

  Serial.println(F("# ServoCalibrator_Companion booting..."));

  Wire.begin();
  encoder.begin();
  if (!encoder.isConnected()) {
    Serial.println(F("# FATAL: AS5600 not detected. Halting."));
    while (true) {}
  }

  servo.attach(SERVO_PIN, ABS_FLOOR_US, ABS_CEIL_US);
  servo.writeMicroseconds(CENTER_US);

  Serial.println(F("# READY -- not calibrated yet. Send CALIBRATE or IMPORT."));
}

void loop() {
  readSerialNonBlocking();
  if (!ready) return;

  unsigned long now = millis();
  if ((long)(now - nextSampleMs) < 0) return;
  nextSampleMs += SAMPLE_MS;

  float vel;
  float setpointDeg = currentSetpointDeg(&vel);
  driveTo(setpointDeg);
  float actualDeg = measuredDeg();

  Serial.print(F("T,"));
  Serial.print(now - streamStartMs);
  Serial.print(',');
  Serial.print(setpointDeg, 3);
  Serial.print(',');
  Serial.print(vel, 3);
  Serial.print(',');
  Serial.print(actualDeg, 3);
  Serial.print(',');
  Serial.print(modeName());
  Serial.print(',');
  Serial.println(useTable ? F("TABLE") : F("LINEAR"));
}
