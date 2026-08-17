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
  signed, monotonic, unwrapped position (turnCount*4096 + raw) by
  watching for a same-direction jump bigger than half a revolution
  between two *consecutive* samples and crediting it to a whole-turn
  counter instead of a real move. That's safe here because nothing this
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
                     | ERR OUT_OF_RANGE <floor>..<ceil>
                     | ERR USAGE
                     | ERR BUSY
      One direct Servo.writeMicroseconds(pulseUs) -- no ramping. Starts
      settling and replies once the tick loop confirms it (or times out).
      A second US sent before that reply arrives gets ERR BUSY -- one in
      flight at a time, never queued or overlapped.
      centideg is the settled shaft position in centidegrees, signed,
      relative to wherever the shaft was at boot (0) -- averaged over the
      stable dwell window. Monotonic across turns: never wraps.

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
#include <AS5600.h>
#include <string.h>

const int SERVO_PIN = A3;

// Hard safety fence only -- guards against an absurd pulse width
// regardless of what the host asks for.
const int ABS_FLOOR_US = 80;
const int ABS_CEIL_US  = 3100;
const int CENTER_US    = 1500;

// Tick rate for the whole program -- long enough to comfortably contain
// one AS5600 read at 400kHz I2C, fast enough that a delta between ticks
// is a fine-grained velocity signal.
const unsigned long LOOP_PERIOD_MS = 5;   // 200Hz

// Settling thresholds, all keyed off that fixed tick period.
const int           SETTLE_DELTA_COUNTS = 2;    // max |total[n]-total[n-1]| between ticks to count as stopped
const int           SETTLE_DWELL_TICKS  = 40;   // 200ms sustained
const unsigned long SETTLE_TIMEOUT_MS   = 3000; // safety net -- slowest real move measured so far was under 1s

Servo servo;
AS5600 encoder;

// Raw 12-bit reading (0-4095 per revolution), nothing else.
uint16_t readEncoder() {
  return encoder.readAngle();
}

// ------------------------------------------------------------------
// Multi-turn position tracking -- see the file-level comment for the
// full reasoning. turnCount is the signed lap counter; totalCounts is
// the resulting always-continuous position (turnCount*4096 + raw).
// ------------------------------------------------------------------
const int WRAP_THRESHOLD = 2048;   // half a revolution -- a same-direction jump bigger than
                                    // this between two consecutive samples is a wrap, not a
                                    // real move (see file-level comment for why that's safe
                                    // at this sample rate)

int32_t turnCount = 0;
int prevRaw = 0;
long totalCounts = 0;

// The only place anywhere in this program that reads the encoder.
// Updates the lap counter and returns the new continuous total.
long updatePositionTracking() {
  int raw = readEncoder();
  int naiveDelta = raw - prevRaw;
  if (naiveDelta < -WRAP_THRESHOLD)      turnCount++;   // wrapped forward, 4095->0
  else if (naiveDelta > WRAP_THRESHOLD)  turnCount--;   // wrapped backward, 0->4095
  prevRaw = raw;
  totalCounts = (long)turnCount * 4096L + raw;
  return totalCounts;
}

// counts -> centidegrees (degrees x100). Rounds to nearest, symmetric
// around zero so a negative total (shaft has gone below its boot
// position) rounds the same way a positive one does. The multiply runs
// in 64 bits purely as cheap insurance against overflow -- this servo's
// real mechanical range is under one revolution, so counts never gets
// anywhere near where a 32-bit product would actually overflow.
const long CENTIDEG_NUM = 36000L;   // 360 deg * 100
const long CENTIDEG_DEN = 4096L;    // counts per revolution

long countsToCentideg(long counts) {
  int64_t num = (int64_t)counts * CENTIDEG_NUM;
  if (num >= 0) return (long)((num + CENTIDEG_DEN / 2) / CENTIDEG_DEN);
  return (long)((num - CENTIDEG_DEN / 2) / CENTIDEG_DEN);
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
long settleSum = 0;              // sum of totalCounts samples across the stable dwell window
int settleStableCount = 0;
unsigned long settleStartMs = 0;
unsigned long nextTickMs = 0;
long prevTickTotal = 0;          // totalCounts as of the previous tick, for settle-delta comparison

// Reports the settled value (or, on timeout, the last total) and
// returns to idle. Averaging is a plain integer mean of totalCounts --
// no wraparound correction needed, totalCounts is already continuous
// even across a stable window that straddles a lap boundary.
void reportSettled(long lastTotal, bool converged) {
  long total = converged ? (settleSum / settleStableCount) : lastTotal;
  // Fine to print directly here: AVR's Serial has a 64-byte TX buffer
  // and this reply is ~16 bytes, so it won't block the tick loop.
  Serial.print(F("OK "));
  Serial.print(targetPulseUs);
  Serial.print(' ');
  Serial.println(countsToCentideg(total));
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
  long total = updatePositionTracking();   // fresh seed right as the move starts
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
struct CapSample { uint16_t tMs; long counts; };
// Halved from 200: each sample now carries a 4-byte multi-turn total
// instead of a 2-byte raw reading, so this keeps the buffer's total RAM
// footprint the same as before it (still RAM-bounded, see protocol
// comment above).
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
    capBuf[count].counts = updatePositionTracking();   // same single tracked read tick() uses
    count++;
    if (delayMs > 0) delay(delayMs);
  }

  // Converted to centidegrees only here, after capture -- keeps the
  // tight sampling loop above free of the conversion's int64 multiply,
  // same reasoning as the original "stream only after it's done".
  Serial.print(F("CAPSTART ")); Serial.println(pulseUs);
  for (uint16_t i = 0; i < count; i++) {
    Serial.print(F("CP "));
    Serial.print(capBuf[i].tMs);
    Serial.print(' ');
    Serial.println(countsToCentideg(capBuf[i].counts));
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
  long total = updatePositionTracking();
  long delta = total - prevTickTotal;   // plain subtraction -- total is already continuous, no wrap fixup needed
  prevTickTotal = total;

  if (mode != MODE_SETTLING) return;

  if (labs(delta) <= SETTLE_DELTA_COUNTS) {
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
  encoder.begin();
  if (!encoder.isConnected()) {
    Serial.println(F("# FATAL: AS5600 not detected. Halting."));
    while (true) {}
  }

  servo.attach(SERVO_PIN, ABS_FLOOR_US, ABS_CEIL_US);
  servo.writeMicroseconds(CENTER_US);   // known starting position, not wherever the last session left it
  prevRaw = readEncoder();              // seed the tracker; boot position becomes 0 total counts (0.00 centideg)
  totalCounts = prevRaw;
  prevTickTotal = totalCounts;

  Serial.println(F("# READY"));
}

void loop() {
  readSerialNonBlocking();   // every pass, not just on-tick -- keeps command dispatch latency low

  if ((long)(millis() - nextTickMs) < 0) return;
  nextTickMs += LOOP_PERIOD_MS;

  tick();
}
