/*
  TrajectoryDemo_Companion

  Generic-ish companion firmware for TrajectoryDemo_App (a WebSerial live
  trajectory-following demo -- "simplified ODrive GUI" for a single RC
  servo joint). Unlike ServoCalibrator_Companion (host-paced, one
  request/response per command), THIS firmware runs the trajectory itself:
  UTI's TrapezoidalProfile is planned/evaluated on-device and streamed to
  the host continuously at a fixed rate, regardless of whether a command
  just arrived -- the host never paces the motion, it only requests moves
  and receives telemetry.

  Position feedback: AS5600 direct (RobTillaart's AS5600 library), read
  independently of whatever pulse was just commanded -- same
  ground-truth-vs-model separation as every accuracy test in this project.

  Servo: drives a raw Servo object directly, but the angle->pulse math is
  still UMI's real production code -- ServoCalibrationTable.h's
  computeServoPulseUs()/validateCalTable(), the exact same free functions
  RCServoMotorDriver::angleToPulseUs() calls internally, just called
  directly instead of through that class. Deliberate: RCServoMotorDriver
  binds its calibration table at CONSTRUCTION time with no runtime
  swap, but this sketch needs to flip between the 2-point linear formula
  and the 20-point table live, mid-motion, on a MODEL command -- so it
  keeps its own `useTable` flag and passes calTable=genTable-or-nullptr
  into computeServoPulseUs() every tick instead. Same real math either
  way, just not locked in at construction.

  Protocol (115200 baud, one command per line, \n-terminated ASCII).
  Unlike ServoCalibrator_Companion, reading is NON-BLOCKING (loop() must
  keep streaming telemetry at a fixed rate even when no command has
  arrived) -- see readSerialNonBlocking().

    PING                              -> OK PONG
    GO <targetDeg> <vMaxDegS> <aMaxDegS2>
                                       -> OK | ERR <msg>
      Plans a new trapezoidal move from wherever the CURRENT setpoint is
      (not necessarily at rest -- a GO while already moving replans
      smoothly from the live setpoint, not the old target) to
      <targetDeg>, honoring vMax/aMax. targetDeg must be within
      [0, maxAngleDeg] (see the boot banner for this rig's actual range).
      Cancels SQUARE/SINE mode if either was active.
    SQUARE <lowDeg> <highDeg> <periodS> <vMaxDegS> <aMaxDegS2>
                                       -> OK | ERR <msg>
      Continuous repeating step: alternates the target between <lowDeg>
      and <highDeg> every periodS/2 seconds, forever, until GO/STOP/SINE.
      Each transition is still a full TrapezoidalProfile move at the
      given vMax/aMax ("post traj" -- the profile shapes every edge, this
      just auto-repeats GO back and forth instead of the host re-issuing
      it manually). If a transition hasn't finished when the next one is
      due, it replans smoothly from wherever the setpoint currently is,
      same as GO.
    SINE <centerDeg> <amplitudeDeg> <freqHz>
                                       -> OK | ERR <msg>
      Continuous sinusoidal setpoint: pos = centerDeg + amplitudeDeg *
      sin(2*pi*freqHz*t), vel = amplitudeDeg*2*pi*freqHz*cos(2*pi*freqHz*t)
      -- analytic, deliberately bypassing TrapezoidalProfile entirely
      ("without traj": no vMax/aMax clamp, the sine's own amplitude*2*pi*freq
      product IS the effective peak velocity -- keep it sane for the servo
      yourself). centerDeg+-amplitudeDeg must fit within [0, maxAngleDeg].
      Eases in with one normal trapezoidal move to centerDeg first (fixed
      60/120 deg/s/s^2), so starting the sine never jumps -- the sine
      itself begins (t=0, i.e. pos==centerDeg) only once that settles.
    STOP                               -> OK
      Freezes the setpoint at wherever it currently is (not a controlled
      decel -- an immediate hold). Safe: it only holds a position already
      being commanded, never jumps. Cancels SQUARE/SINE mode too.
    MODEL LINEAR | TABLE               -> OK | ERR <msg>
      Switches which angle->pulse mapping drives the servo, live, without
      touching the current setpoint/mode -- so the SAME trajectory (a
      running SQUARE or SINE, or just holding at rest) can be watched
      under both models back to back, isolating the mapping's effect on
      tracking error from everything else. Takes effect on the very next
      50Hz tick, whatever mode is currently active. TABLE is the boot
      default (the whole point of the earlier lookup-table investigation
      this project did); LINEAR uses the same measured 350-2630us pulse
      range as an apples-to-apples 2-point comparison, not the "typical"
      500-2500us guess. `MODEL TABLE` is rejected if the table failed
      validation at boot (shouldn't happen with GenTable.h as shipped).

  Streamed continuously, ~50Hz, whether or not a move is active (setpoint
  holds flat at rest so the actual-position trace stays live and
  comparable even between moves):

    T,<t_ms>,<setpoint_deg>,<setpoint_vel_degs>,<actual_deg>,<mode>,<model>

  <t_ms> is milliseconds since boot-time zero/settle finished -- a single
  continuously-increasing clock for the whole session, not per-move, so
  the browser can keep one unbroken rolling time axis. <mode> is one of
  IDLE/MOVE/SQUARE/SINE -- sent explicitly rather than left for the host
  to infer from setpoint velocity, since a sine's velocity legitimately
  passes through zero twice a cycle (at its position extremes) without
  the motion actually stopping. <model> is LINEAR or TABLE -- also sent
  explicitly (not left for the host to track from its own MODEL calls)
  so a live model switch shows up in the telemetry stream itself, at the
  exact sample it took effect.

  Lines starting with "#" are informational only (boot banner, sign-probe
  result) -- not part of the command/response or telemetry protocol,
  safe to log-and-ignore.
*/

#include <Arduino.h>
#include <Wire.h>
#include <Servo.h>
#include <AS5600.h>
#include <ServoCalibrationTable.h>  // computeServoPulseUs()/validateCalTable() -- see header comment above for why this is called directly instead of via RCServoMotorDriver
#include <TrapezoidalProfile.h>
#include "GenTable.h"

const int SERVO_PIN = A3;
const unsigned long SAMPLE_MS = 20;  // 50Hz telemetry + control rate

// Same measured range either way (the table's own scanned extremes ARE
// 350/2630 -- see GenTable.h) so LINEAR vs TABLE is a clean, apples-to-
// apples comparison of the mapping function alone, not confounded by a
// different pulse-width clamp.
const int PULSE_MIN_US = 350;
const int PULSE_MAX_US = 2630;

Servo servo;
AS5600 encoder;
TrapezoidalProfile profile;

float maxAngleDeg = 0.0f;
bool useTable = true;      // live-switchable via MODEL; TABLE is the boot default
bool tableValid = false;   // set once at boot by validateCalTable(); MODEL TABLE refuses to enable an invalid table

// Angle (deg, physical frame) -> pulse width (us), using UMI's real
// production math either way -- only the calTable argument changes.
uint16_t pulseForDeg(float deg) {
  return computeServoPulseUs(radians(deg),
                              useTable ? genTable : nullptr,
                              useTable ? GEN_TABLE_LEN : 0,
                              MAX_ANGLE_RAD, PULSE_MIN_US, PULSE_MAX_US);
}
void driveTo(float deg) { servo.writeMicroseconds(pulseForDeg(deg)); }

// ------------------------------------------------------------------
// AS5600 ground truth, unwrapped running angle -- same wrap-safe
// small-delta accumulation technique as RCServoAutoCalibration.ino /
// UMI_CalTable_HWTest.ino.
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
  Serial.println(F("# WARNING: settle timed out during boot zeroing"));
  return prev;
}

// ------------------------------------------------------------------
// Setpoint state machine. Setpoint is always well-defined regardless of
// mode -- currentSetpointDeg() is the one authoritative place that
// decides "where should we be right now", called both by loop() (to
// actually drive the servo) and by GO/SQUARE/SINE handlers (to know
// where to replan FROM, so switching modes never jumps).
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

unsigned long moveStartMs = 0;   // start-of-current-leg clock, shared by MODE_MOVE and MODE_SQUARE's current leg
float holdSetpointDeg = 0.0f;    // meaningful only in MODE_IDLE
unsigned long streamStartMs = 0;
unsigned long nextSampleMs = 0;

// SQUARE state
float sqLowDeg = 0.0f, sqHighDeg = 0.0f, sqVMax = 0.0f, sqAMax = 0.0f;
unsigned long sqPeriodMs = 0;
unsigned long sqLegStartMs = 0;
bool sqGoingHigh = true;

// SINE state, plus the "settle to center first" pending flag -- SINE
// doesn't switch mode immediately; it starts a normal MODE_MOVE to
// centerDeg and only flips to MODE_SINE once that move actually finishes
// (see the MODE_MOVE branch below), so entering sine mode never jumps.
float sinCenterDeg = 0.0f, sinAmplitudeDeg = 0.0f, sinFreqHz = 0.0f;
unsigned long sinStartMs = 0;
bool sinePending = false;

// Current setpoint position (deg) for whichever mode is active.
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
        // The centering move just settled -- hand off to MODE_SINE
        // starting exactly here (t=0 of the sine == pos == centerDeg),
        // in this same call, so no telemetry sample shows a gap or jump.
        mode = MODE_SINE;
        sinStartMs = millis();
        sinePending = false;
        float w = 2.0f * PI * sinFreqHz;
        if (velOut) *velOut = sinAmplitudeDeg * w;  // cos(0) == 1
        return sinCenterDeg;                         // sin(0) == 0
      }
      mode = MODE_IDLE;
      if (velOut) *velOut = 0.0f;
      return pos;
    }
    if (velOut) *velOut = vel;
    return pos;
  }

  // MODE_IDLE
  if (velOut) *velOut = 0.0f;
  return holdSetpointDeg;
}

// ------------------------------------------------------------------
// Non-blocking line-based command reader (NOT the blocking readLine()
// pattern ServoCalibrator_Companion uses -- loop() must keep streaming
// telemetry every SAMPLE_MS even while no full command line has arrived
// yet).
// ------------------------------------------------------------------
const int LINE_BUF_LEN = 48;
char lineBuf[LINE_BUF_LEN];
int lineLen = 0;

const int MAX_TOKENS = 6;  // SQUARE <lowDeg> <highDeg> <periodS> <vMax> <aMax>
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

void handleGo(int n) {
  if (n != 4) { Serial.println(F("ERR USAGE: GO <targetDeg> <vMaxDegS> <aMaxDegS2>")); return; }
  float target = atof(tok[1]);
  float vMax = atof(tok[2]);
  float aMax = atof(tok[3]);

  if (target < 0.0f || target > maxAngleDeg) {
    Serial.print(F("ERR OUT_OF_RANGE 0.."));
    Serial.println(maxAngleDeg, 2);
    return;
  }
  if (vMax <= 0.0f || aMax <= 0.0f) {
    Serial.println(F("ERR LIMITS_MUST_BE_POSITIVE"));
    return;
  }

  float q0 = currentSetpointDeg();  // replans smoothly from wherever we are NOW, cancels SQUARE/SINE

  TrajectoryLimits limits;
  limits.vMax = vMax;
  limits.aMax = aMax;
  limits.jMax = 0.0f;
  if (!profile.plan(q0, target, limits)) {
    Serial.println(F("ERR PLAN_FAILED"));
    return;
  }
  moveStartMs = millis();
  mode = MODE_MOVE;
  sinePending = false;
  Serial.println(F("OK"));
}

void handleSquare(int n) {
  if (n != 6) { Serial.println(F("ERR USAGE: SQUARE <lowDeg> <highDeg> <periodS> <vMaxDegS> <aMaxDegS2>")); return; }
  float lo = atof(tok[1]);
  float hi = atof(tok[2]);
  float periodS = atof(tok[3]);
  float vMax = atof(tok[4]);
  float aMax = atof(tok[5]);

  if (lo < 0.0f || lo > maxAngleDeg || hi < 0.0f || hi > maxAngleDeg) {
    Serial.print(F("ERR OUT_OF_RANGE 0.."));
    Serial.println(maxAngleDeg, 2);
    return;
  }
  if (hi <= lo) { Serial.println(F("ERR HIGH_MUST_EXCEED_LOW")); return; }
  if (periodS < 0.1f || periodS > 60.0f) { Serial.println(F("ERR PERIOD_OUT_OF_RANGE 0.1..60")); return; }
  if (vMax <= 0.0f || aMax <= 0.0f) { Serial.println(F("ERR LIMITS_MUST_BE_POSITIVE")); return; }

  float q0 = currentSetpointDeg();  // cancels any prior MOVE/SQUARE/SINE

  sqLowDeg = lo; sqHighDeg = hi; sqVMax = vMax; sqAMax = aMax;
  sqPeriodMs = (unsigned long)(periodS * 1000.0f);
  sqGoingHigh = true;

  TrajectoryLimits limits;
  limits.vMax = vMax; limits.aMax = aMax; limits.jMax = 0.0f;
  if (!profile.plan(q0, hi, limits)) {
    Serial.println(F("ERR PLAN_FAILED"));
    return;
  }
  moveStartMs = millis();
  sqLegStartMs = moveStartMs;
  mode = MODE_SQUARE;
  sinePending = false;
  Serial.println(F("OK"));
}

void handleSine(int n) {
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

  // Ease in via one ordinary trapezoidal move to centerDeg (fixed, gentle
  // limits -- these aren't user-tunable, just getting into position) --
  // MODE_SINE actually starts once currentSetpointDeg() sees that move
  // finish (see the MODE_MOVE/sinePending branch above), not immediately.
  float q0 = currentSetpointDeg();
  TrajectoryLimits limits;
  limits.vMax = 60.0f; limits.aMax = 120.0f; limits.jMax = 0.0f;
  if (!profile.plan(q0, center, limits)) {
    Serial.println(F("ERR PLAN_FAILED"));
    return;
  }
  moveStartMs = millis();
  mode = MODE_MOVE;
  sinePending = true;
  Serial.println(F("OK"));
}

void handleStop() {
  holdSetpointDeg = currentSetpointDeg();  // freeze wherever we currently are
  mode = MODE_IDLE;
  sinePending = false;
  Serial.println(F("OK"));
}

// Deliberately does NOT touch mode/holdSetpointDeg/profile/sq*/sin* state
// at all -- switching models mid-SQUARE or mid-SINE (or at rest) is the
// whole point, so the same commanded trajectory can be compared under
// both mappings back to back. Takes effect on the very next loop() tick.
void handleModel(int n) {
  if (n != 2) { Serial.println(F("ERR USAGE: MODEL LINEAR|TABLE")); return; }
  if (strcmp(tok[1], "LINEAR") == 0) {
    useTable = false;
    Serial.println(F("OK"));
  } else if (strcmp(tok[1], "TABLE") == 0) {
    if (!tableValid) { Serial.println(F("ERR TABLE_INVALID")); return; }
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

  Serial.println(F("# TrajectoryDemo_Companion booting..."));

  Wire.begin();
  encoder.begin();
  if (!encoder.isConnected()) {
    Serial.println(F("# FATAL: AS5600 not detected. Halting."));
    while (true) {}
  }

  uint16_t tMin, tMax;  // discarded -- PULSE_MIN_US/MAX_US are used directly (see above), this call is purely validation
  tableValid = validateCalTable(genTable, GEN_TABLE_LEN, MAX_ANGLE_RAD, tMin, tMax);
  if (!tableValid) {
    Serial.println(F("# WARNING: calibration table failed validation -- forcing linear model"));
    useTable = false;
  }

  servo.attach(SERVO_PIN, PULSE_MIN_US, PULSE_MAX_US);
  maxAngleDeg = MAX_ANGLE_RAD * RAD_TO_DEG;

  // Zero the AS5600 reference at physical angle 0, then auto-detect
  // sign convention with a small probe move -- same technique as
  // UMI_CalTable_HWTest, kept here rather than hardcoding the sign this
  // rig happened to have last time (test, don't assume).
  //
  // NOTE: zeroRefAngle must be the settled value pollUntilSettled()
  // actually returns, not a hardcoded 0.0f -- the servo's pre-boot
  // position (wherever the previously-flashed sketch left it) is
  // arbitrary, and delay(400) alone isn't reliably enough time to reach
  // physical 0 from far away (e.g. the far end of a ~214deg stroke).
  // pollUntilSettled() correctly waits out however long that actually
  // takes; discarding its return value and hardcoding 0.0f re-introduces
  // exactly the race it exists to avoid -- caught via a real ~151deg
  // offset on first boot after this sketch followed one that left the
  // servo at the opposite end of its range.
  driveTo(0.0f);
  delay(400);
  resyncRunningAngle();
  runningAngle = 0.0f;
  zeroRefAngle = pollUntilSettled();
  signConv = 1.0f;

  driveTo(maxAngleDeg * 0.3f);
  float probe = pollUntilSettled();
  // Compare the DELTA since leaving zeroRefAngle, not probe's raw value in
  // isolation -- the AS5600's own raw zero point is arbitrary, so a
  // nonzero zeroRefAngle could otherwise flip the apparent sign even
  // though only the direction of the delta actually matters here.
  signConv = ((probe - zeroRefAngle) >= 0.0f) ? 1.0f : -1.0f;
  Serial.print(F("# signConv=")); Serial.println(signConv);

  driveTo(0.0f);
  pollUntilSettled();
  holdSetpointDeg = 0.0f;

  streamStartMs = millis();
  nextSampleMs = streamStartMs;

  Serial.print(F("# READY maxAngleDeg="));
  Serial.println(maxAngleDeg, 2);
}

void loop() {
  readSerialNonBlocking();

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
