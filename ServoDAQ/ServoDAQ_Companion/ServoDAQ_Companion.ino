/*
  ServoDAQ_Companion

  Boots into a known state, then takes a raw pulse width and reports back
  the settled shaft position. No zero reference (position is relative to
  wherever the shaft happened to be at boot), no model -- that's the
  host's job. Unit conversion (raw AS5600 counts -> centidegrees) *is*
  done here now, and so is multi-turn unwrapping -- see below.

  Everything runs on one fixed-rate loop (200Hz): reading serial, reading
  the encoder, differentiating, writing the servo, and deciding "settled"
  all happen on the same tick, nothing in a nested blocking sub-loop. The
  fixed period is what makes a raw delta between ticks mean velocity,
  regardless of anything else going on -- and why no separate filtering
  step is needed; the reported value is just the average of the raw
  samples across the stable dwell window.

  Multi-turn position tracking: the AS5600 itself only ever reports a
  raw 12-bit angle, 0-4095, that wraps every revolution -- it has no
  concept of "which lap." updatePositionTracking() turns that into a
  signed, monotonic, unwrapped position by watching for a same-direction
  jump bigger than half a revolution between two *consecutive* samples
  and crediting it to a whole-turn count instead of a real move. That
  unwrap is Universal-Encoder-Interface's (AS5600EncoderDriver in
  continuous mode); this file used to implement it locally and no longer
  does. The tracked position is carried in radians, since that is the
  only form UEI exposes an unwrapped reading in -- converted to
  centidegrees at the same single boundary as before, so nothing on the
  wire changed. That's safe here because nothing this
  servo does can complete a full revolution between consecutive samples
  at this sample rate (tick() samples every 5ms; CAP, the fastest path,
  samples every I2C transaction, well under that) -- a real single-step
  move of more than 2048 counts between reads 5ms apart is not physically
  possible for a hobby servo, so any jump that big is unambiguously a
  wrap, never mistaken for real motion. This is the ONLY place in the
  firmware that reads the encoder, and it runs first, unconditionally,
  before any mode-specific logic (settling, streaming, anything) -- so
  every consumer (tick's settle detector, CAP's capture loop) sees the
  same always-continuous position, and nothing downstream -- not this
  firmware, not the host script, not a plot -- ever needs to unwrap a
  wraparound after the fact.

  Wire values are centidegrees (degrees x100), not raw counts. The
  AS5600's native resolution is 360/4096 = 0.087890625 deg/count (~8.79
  centidegrees/count); centidegrees give ~0.01 deg of resolution, about
  8.8x finer than the sensor's own quantization step, so nothing is lost
  by converting, and the wire format stays a plain signed integer with
  no float parsing needed on the host.

  Protocol (115200 baud, one command per line, \n-terminated ASCII).

    PING           -> OK PONG

    US <pulseUs>   -> OK <pulseUs> <centideg>
                     | ERR NOT_SETTLED <pulseUs> <centideg>
                     | ERR OUT_OF_RANGE <floor>..<ceil>
                     | ERR USAGE
                     | ERR BUSY
      One direct Servo.writeMicroseconds(pulseUs) -- no ramping. Starts
      settling and replies once the tick loop confirms it (or times out).
      A second US sent before that reply arrives gets ERR BUSY -- one in
      flight at a time, never queued or overlapped.
      centideg (OK case) is the settled shaft position in centidegrees,
      signed, relative to wherever the shaft was at boot (0) -- averaged
      over the stable dwell window. Monotonic across turns: never wraps.
      NOT_SETTLED means SETTLE_TIMEOUT_MS elapsed without
      SETTLE_DWELL_TICKS of consecutive <=SETTLE_DELTA_RAD deltas ever
      being observed -- centideg here is just the single last raw
      reading, not a genuinely stable value, and the shaft may still be
      moving when this reply arrives. Previously reported as a plain OK
      indistinguishable from a real settle; see the file's own history
      for why that was a real bug for a servo with nothing to settle
      against.

    CAP <pulseUs> <delayMs>
                   -> CAPSTART <pulseUs>
                      (streamed) CP <tMs> <centideg>  -- one per sample
                      CAPEND <count>
                     | ERR OUT_OF_RANGE <floor>..<ceil>
                     | ERR DELAY_OUT_OF_RANGE 0..1000
                     | ERR BUSY
      Diagnostic only -- not part of normal operation, a deliberate
      exception to "everything happens on the tick" the same way
      blocking was an accepted exception for CALIBRATE in the old
      firmware. tick()'s fixed period matters for making a delta mean
      velocity; CAP doesn't make any on-device decision at all, it just
      timestamps samples for the host to look at, so sampling as fast as
      I2C allows (delayMs=0) -- or slower, if delayMs>0, to trade
      resolution for a longer capture window against a fixed sample
      buffer -- is fine and is the point. One direct write, no settle
      wait; streams the whole burst only after it's done (printing
      mid-loop would slow the sampling rate and skew the timing being
      captured). Each sample still goes through the same single tracked
      read as tick(), so a CAP burst is monotonic too, same as US.

  Lines starting with "#" are informational only -- not part of the
  command/response protocol, safe to log-and-ignore.
*/

#include <Arduino.h>
#include <Wire.h>
#include <Servo.h>
// Universal-Encoder-Interface -- owns the multi-turn unwrap this file used
// to hand-roll. Still built on RobTillaart's AS5600 underneath (UEI wraps
// it), and pulls in Universal-Device-Interface, UEI's own dependency.
#include <AS5600EncoderDriver.h>
#include <string.h>

const int SERVO_PIN = A3;

// Hard safety fence only -- guards against an absurd pulse width
// regardless of what the host asks for.
//
// ABS_CEIL_US must also stay inside what Servo::attach() can actually
// encode: internally it stores (MAX_PULSE_WIDTH - max)/4 in a signed
// int8_t (Servo.h: MAX_PULSE_WIDTH 2400), so any ceiling above
// 2400 + 4*127 = 2908 (2912 rounds cleanly to the same 4us step Servo.cpp
// itself uses) silently overflows and wraps. The old value here, 3100,
// computed (2400-3100)/4 = -175, which doesn't fit in int8_t and wraps to
// 81, giving an ACTUAL enforced ceiling of 2400-81*4 = 2076us -- every
// commanded pulse above ~2076us was silently rewritten to 2076us the
// whole time, indistinguishable from the servo genuinely stalling right
// there. Found by comparing this project's own find_range() output
// against Servo_Test.ino (a sibling sketch on the same hardware that
// deliberately caps at 2912us for exactly this reason) and tracing the
// ~2076us "edge" both ServoDAQ and ServoCalibrator_Companion kept
// reporting back to this overflow -- not a real mechanical limit. See
// ServoCalibrator_Companion.ino's own ABS_CEIL_US comment and the
// 2026-08-25 CLAUDE.md entry for the full finding. This affects every
// max_pulse_us this project has ever recorded via find_range() on a
// servo whose true range exceeds ~2076us.
const int ABS_FLOOR_US = 80;
const int ABS_CEIL_US  = 2912;
const int CENTER_US    = 1500;

// Tick rate for the whole program -- long enough to comfortably contain
// one AS5600 read at 400kHz I2C, fast enough that a delta between ticks
// is a fine-grained velocity signal.
const unsigned long LOOP_PERIOD_MS = 5;   // 200Hz

// Settling thresholds, all keyed off that fixed tick period.
// Max |pos[n]-pos[n-1]| between ticks to count as stopped. Still "2 AS5600
// counts" -- the sensor's own quantization is what this has always meant --
// just expressed in radians now that the tracked position is radians (see
// updatePositionTracking()), written as the conversion rather than a
// pre-rounded decimal so it stays correct by construction.
const int   SETTLE_DELTA_COUNTS = 2;
const float SETTLE_DELTA_RAD    = SETTLE_DELTA_COUNTS * (2.0f * 3.14159265358979f / 4096.0f);
const int           SETTLE_DWELL_TICKS  = 40;   // 200ms sustained
const unsigned long SETTLE_TIMEOUT_MS   = 3000; // safety net -- slowest real move measured so far was under 1s

Servo servo;

// continuous = true: readPhysicalAngleRad() accumulates across revolutions
// rather than resetting at the AS5600's 4096-count wrap.
AS5600EncoderDriver encoder(Wire, /*continuous=*/true);

// ------------------------------------------------------------------
// Position tracking -- now owned by Universal-Encoder-Interface's
// AS5600EncoderDriver rather than hand-rolled here. This file used to keep
// its own signed lap counter and rebuild a continuous position as
// turnCount*4096 + raw; UEI's unwrapRawCounts() (EncoderMath.h) is that
// same algorithm with the same half-revolution-between-samples assumption,
// maintained and tested in one place, so the local copy was deleted rather
// than kept in parallel. The assumption is as safe here as it ever was --
// this file samples at 200Hz, and CAP faster still.
//
// The carried unit is RADIANS now, not counts: UEI exposes its unwrapped
// reading only as a float angle (readRawCounts() is deliberately always the
// bounded 0..4095 register and cannot carry lap information). Conversion to
// centidegrees still happens at exactly the same boundary as before, so the
// wire protocol is byte-for-byte unchanged.
//
// readPhysicalAngleRad(), not readAngleRad(): no software zero offset is
// applied, so the boot reading is the shaft's absolute angle -- exactly
// what this firmware reported before (it seeded totalCounts from the raw
// register, not from zero). The host works in relative terms and re-zeros
// on every connection anyway.
// ------------------------------------------------------------------
float totalRad = 0.0f;   // continuous, unwrapped position in radians

// The only place anywhere in this program that reads the encoder.
float updatePositionTracking() {
  totalRad = encoder.readPhysicalAngleRad();
  return totalRad;
}

// radians -> centidegrees (degrees x100). lroundf() rounds to nearest and
// is symmetric about zero, so a negative position (shaft below its boot
// reference) rounds the same way a positive one does -- the same property
// the previous integer implementation went out of its way to preserve, now
// for free. 18000/PI is the exact analogue of that version's 36000/4096
// counts-based ratio.
const float CENTIDEG_PER_RAD = 5729.577951308232f;

long radToCentideg(float rad) {
  return lroundf(rad * CENTIDEG_PER_RAD);
}

// ------------------------------------------------------------------
// Command line assembly. Non-blocking: only ever consumes what's
// currently sitting in the serial buffer.
// ------------------------------------------------------------------
const int LINE_BUF_LEN = 32;
char lineBuf[LINE_BUF_LEN];
int lineLen = 0;

const int MAX_TOKENS = 4;
char* tok[MAX_TOKENS];
int tokenize(char* line) {
  int n = 0;
  char* p = line;
  while (*p && n < MAX_TOKENS) {
    while (*p == ' ') p++;      // skip leading/repeated spaces
    if (!*p) break;
    tok[n++] = p;
    while (*p && *p != ' ') p++;
    if (*p) { *p = '\0'; p++; } // terminate this token, advance past it
  }
  return n;
}

// ------------------------------------------------------------------
// Runtime state -- persists across ticks, not local to a nested loop.
// mode tracks whether a US command is in flight; tick() advances it one
// step per call, whether that call lands during an idle period or one
// mid-command.
// ------------------------------------------------------------------
enum Mode { MODE_IDLE, MODE_SETTLING };
Mode mode = MODE_IDLE;

int targetPulseUs = CENTER_US;   // echoed back in the reply
float settleSum = 0.0f;          // sum of position samples (radians) across the stable dwell window
int settleStableCount = 0;
unsigned long settleStartMs = 0;
unsigned long nextTickMs = 0;
float prevTickTotal = 0.0f;      // position as of the previous tick, for settle-delta comparison

// Reports the settled value, or -- if SETTLE_TIMEOUT_MS elapsed without
// ever actually settling -- an ERR instead of a fake OK. lastTotal in
// the timeout case is just whatever the single last raw reading was,
// not a genuinely stable value, and the shaft may still be moving right
// now; reporting it as an error (not success) is the whole fix -- a
// caller that only ever checked reply[0]=="OK" now correctly sees a
// failure instead of silently treating an unsettled reading as real
// data (see this command's protocol comment for the full reasoning).
void reportSettled(float lastTotal, bool converged) {
  if (!converged) {
    Serial.print(F("ERR NOT_SETTLED "));
    Serial.print(targetPulseUs);
    Serial.print(' ');
    Serial.println(radToCentideg(lastTotal));
    mode = MODE_IDLE;
    return;
  }
  // Fine to print directly here: AVR's Serial has a 64-byte TX buffer
  // and this reply is ~16 bytes, so it won't block the tick loop.
  Serial.print(F("OK "));
  Serial.print(targetPulseUs);
  Serial.print(' ');
  Serial.println(radToCentideg(settleSum / (float)settleStableCount));
  mode = MODE_IDLE;
}

// US <pulseUs> -- validate, write, arm settling. Non-blocking: the
// actual detection happens tick by tick in tick() below.
void handleUs(int n) {
  if (n != 2) { Serial.println(F("ERR USAGE: US <pulseUs>")); return; }
  int pulseUs = atoi(tok[1]);
  if (pulseUs < ABS_FLOOR_US || pulseUs > ABS_CEIL_US) {
    Serial.print(F("ERR OUT_OF_RANGE ")); Serial.print(ABS_FLOOR_US); Serial.print(F(".."));
    Serial.println(ABS_CEIL_US);
    return;
  }
  servo.writeMicroseconds(pulseUs);
  targetPulseUs = pulseUs;
  float total = updatePositionTracking();   // fresh seed right as the move starts
  settleSum = total;
  settleStableCount = 1;
  settleStartMs = millis();
  mode = MODE_SETTLING;
}

// ------------------------------------------------------------------
// CAP -- raw step-response capture for diagnosing exactly what US's
// settle detector can't see (e.g. a single-sample discontinuity that
// two independently-settled readings just look like a big gap between).
// Deliberately blocking, deliberately not going through tick() -- see
// this command's protocol comment at the top of the file for why that's
// fine here specifically.
// ------------------------------------------------------------------
struct CapSample { uint16_t tMs; float rad; };
// Halved from 200 back when each sample went from a 2-byte raw reading to
// a 4-byte multi-turn total; a float position is the same 4 bytes, so the
// buffer's RAM footprint is unchanged again here (still RAM-bounded, see
// protocol comment above).
const uint16_t CAP_BUFFER_SIZE = 100;
CapSample capBuf[CAP_BUFFER_SIZE];

void handleCap(int n) {
  if (n != 3) { Serial.println(F("ERR USAGE: CAP <pulseUs> <delayMs>")); return; }
  int pulseUs = atoi(tok[1]);
  if (pulseUs < ABS_FLOOR_US || pulseUs > ABS_CEIL_US) {
    Serial.print(F("ERR OUT_OF_RANGE ")); Serial.print(ABS_FLOOR_US); Serial.print(F(".."));
    Serial.println(ABS_CEIL_US);
    return;
  }
  int delayMs = atoi(tok[2]);
  if (delayMs < 0 || delayMs > 1000) { Serial.println(F("ERR DELAY_OUT_OF_RANGE 0..1000")); return; }

  unsigned long startMs = millis();
  servo.writeMicroseconds(pulseUs);   // direct write -- this is the capture event, not a normal settle

  uint16_t count = 0;
  while (count < CAP_BUFFER_SIZE) {
    capBuf[count].tMs = (uint16_t)(millis() - startMs);
    capBuf[count].rad = updatePositionTracking();   // same single tracked read tick() uses
    count++;
    if (delayMs > 0) delay(delayMs);
  }

  // Converted to centidegrees only here, after capture -- keeps the tight
  // sampling loop above free of the conversion, same reasoning as the
  // original "stream only after it's done".
  Serial.print(F("CAPSTART ")); Serial.println(pulseUs);
  for (uint16_t i = 0; i < count; i++) {
    Serial.print(F("CP "));
    Serial.print(capBuf[i].tMs);
    Serial.print(' ');
    Serial.println(radToCentideg(capBuf[i].rad));
  }
  Serial.print(F("CAPEND ")); Serial.println(count);
}

// Dispatch on the first token. US/CAP while already settling get ERR
// BUSY instead of overlapping with the command in flight.
void handleLine(char* line) {
  int n = tokenize(line);
  if (n == 0) return;
  if (strcmp(tok[0], "PING") == 0) {
    Serial.println(F("OK PONG"));
  } else if (strcmp(tok[0], "US") == 0) {
    if (mode == MODE_SETTLING) { Serial.println(F("ERR BUSY")); return; }
    handleUs(n);
  } else if (strcmp(tok[0], "CAP") == 0) {
    if (mode == MODE_SETTLING) { Serial.println(F("ERR BUSY")); return; }
    handleCap(n);
  } else {
    Serial.println(F("ERR UNKNOWN_CMD"));
  }
}

void readSerialNonBlocking() {
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

// One tick: track position, then (only if a command is in flight)
// advance settling. Position tracking happens first, unconditionally,
// every single tick -- everything else is downstream of it. Called
// once per LOOP_PERIOD_MS from loop().
void tick() {
  float total = updatePositionTracking();
  float delta = total - prevTickTotal;   // plain subtraction -- total is already continuous, no wrap fixup needed
  prevTickTotal = total;

  if (mode != MODE_SETTLING) return;

  if (fabsf(delta) <= SETTLE_DELTA_RAD) {
    settleSum += total;
    settleStableCount++;
    if (settleStableCount >= SETTLE_DWELL_TICKS) {
      reportSettled(total, true);
      return;
    }
  } else {
    settleSum = total;
    settleStableCount = 1;
  }

  if (millis() - settleStartMs >= SETTLE_TIMEOUT_MS) {
    reportSettled(total, false);
  }
}

void setup() {
  Serial.begin(115200);
  while (!Serial) {}
  delay(300);   // let the USB-serial adapter settle before printing

  Serial.println(F("# ServoDAQ_Companion booting..."));

  Wire.begin();
  Wire.setClock(400000);  // Fast-mode I2C -- keeps one read comfortably inside a tick
  // UEI's begin() brings the chip up and probes it -- a false return is the
  // ERR_NOT_CONNECTED case the old separate isConnected() check covered.
  if (!encoder.begin()) {
    Serial.print(F("# FATAL: "));
    Serial.print(encoder.getErrorString(encoder.getError()));
    Serial.println(F(". Halting."));
    while (true) {}
  }

  // Magnet diagnostics this firmware had no access to before: an I2C ack
  // says nothing about whether a magnet is present and within AGC range,
  // which is the difference between real readings and plausible garbage.
  if (!encoder.isValid()) {
    Serial.print(F("# WARNING: "));
    Serial.println(encoder.getStatusString(encoder.getStatus()));
  }

  servo.attach(SERVO_PIN, ABS_FLOOR_US, ABS_CEIL_US);
  servo.writeMicroseconds(CENTER_US);   // known starting position, not wherever the last session left it
  totalRad = updatePositionTracking();  // seed UEI's unwrap state from the shaft's current angle
  prevTickTotal = totalRad;

  Serial.println(F("# READY"));
}

void loop() {
  readSerialNonBlocking();   // every pass, not just on-tick -- keeps command dispatch latency low

  if ((long)(millis() - nextTickMs) < 0) return;
  nextTickMs += LOOP_PERIOD_MS;

  tick();
}
