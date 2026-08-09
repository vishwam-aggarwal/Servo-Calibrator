/*
  ServoCalibrator_Companion

  Generic, reusable companion firmware for the Servo Calibrator desktop app
  (a WebSerial-based tool). This sketch runs NO test logic itself -- it's a
  thin, dumb command executor. All sweep/calibration logic (range-finding,
  direction detection, settle-until-stable, linearity/hysteresis sweeps)
  lives in the PC-side app and drives this firmware one command at a time
  over Serial. That keeps every threshold/algorithm tunable from the app
  without ever reflashing the board.

  Supports either an RC servo on a digital pin (raw Servo, no calibration
  layer) OR a channel on a PCA9685 I2C board (via Universal-Motor-Interface's
  PCA9685Backend -- also raw pulse writes, no calibration layer) -- exactly
  one at a time, chosen at runtime via CONFIG. Both are deliberately
  uncalibrated: this firmware is for *discovering* a servo's range/direction/
  zero point, which is the input to RCServoMotorDriver/PCA9685MotorDriver,
  not a consumer of it.

  Position feedback is an AS5600 magnetic encoder via
  Universal-Encoder-Interface, continuous mode (matches Servo_Auto_Calibrator's
  usage elsewhere in this Arduino sketchbook).

  Protocol (115200 baud, one command per line, \n-terminated ASCII):

    PING                                 -> OK PONG
    CONFIG SERVO <pin> <minUs> <maxUs>    -> OK | ERR <msg>
    CONFIG PCA9685 <addrHex> <ch> <hz>    -> OK | ERR <msg>
    CONFIG ENCODER AS5600                 -> OK | ERR <msg>
    MOVE <us>                             -> OK | ERR NOT_CONFIGURED
    READ                                  -> DATA <angle_deg> <valid 0|1> <status>
    ZERO <angle_deg>                      -> OK | ERR NOT_CONFIGURED
    STOP                                  -> OK

  <status> (from AS5600EncoderDriver::magnetStatusCode(), added alongside
  the plain 0/1 rather than replacing it -- <valid> stays a stable
  always-boolean field, <status> is the "why" for anything that reads it):
    OK             magnet detected, field strength in range
    NO_MAGNET      chip responding, but no magnet detected at all
    TOO_WEAK       magnet detected but field too weak -- too far / off-axis
    TOO_STRONG     magnet detected but field too strong -- too close
    NOT_RESPONDING I2C communication with the AS5600 itself is failing

  Lines starting with "#" are informational/boot banner only -- not part of
  the command/response protocol, safe for the app to ignore.

  Pin/channel/address are runtime-configurable via CONFIG, not fixed at
  compile time -- upload this once, reconfigure per-project from the app.
  AS5600 is assumed on the board's default I2C bus (A4/A5 on a Nano).
*/

#include <Servo.h>
#include <Wire.h>
#include <PCA9685Backend.h>
#include <AS5600EncoderDriver.h>
#include <math.h>

enum MotorType { MOTOR_NONE, MOTOR_SERVO, MOTOR_PCA9685 };

Servo rcServo;
PCA9685Backend* pcaBackend = nullptr;
int pcaChannel = -1;
MotorType motorType = MOTOR_NONE;

AS5600EncoderDriver encoder(Wire, /*continuous=*/true);
bool encoderConfigured = false;

const int LINE_BUF_LEN = 48;
char lineBuf[LINE_BUF_LEN];

const int MAX_TOKENS = 6;
char* tok[MAX_TOKENS];

int readLine() {
  int idx = 0;
  while (true) {
    while (!Serial.available()) { }
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (idx == 0) continue;  // drop stray leading CR/LF
      break;
    }
    if (idx < LINE_BUF_LEN - 1) lineBuf[idx++] = c;
  }
  lineBuf[idx] = '\0';
  return idx;
}

// Splits lineBuf in place on spaces; fills tok[] with pointers into it.
// No String class, no dynamic allocation per line.
int tokenize() {
  int n = 0;
  char* p = lineBuf;
  while (*p && n < MAX_TOKENS) {
    while (*p == ' ') p++;
    if (!*p) break;
    tok[n++] = p;
    while (*p && *p != ' ') p++;
    if (*p) { *p = '\0'; p++; }
  }
  return n;
}

void detachMotor() {
  if (motorType == MOTOR_SERVO) rcServo.detach();
  if (motorType == MOTOR_PCA9685 && pcaBackend) pcaBackend->setChannelOff(pcaChannel);
  motorType = MOTOR_NONE;
}

// tok[0]="CONFIG" tok[1]="SERVO" tok[2]=pin tok[3]=minUs tok[4]=maxUs, n=5
void cmdConfigServo(int n) {
  if (n != 5) { Serial.println(F("ERR USAGE: CONFIG SERVO <pin> <minUs> <maxUs>")); return; }
  detachMotor();
  int pin = atoi(tok[2]);
  int minUs = atoi(tok[3]);
  int maxUs = atoi(tok[4]);
  rcServo.attach(pin, minUs, maxUs);
  motorType = MOTOR_SERVO;
  Serial.println(F("OK"));
}

// tok[0]="CONFIG" tok[1]="PCA9685" tok[2]=addrHex tok[3]=channel tok[4]=freqHz, n=5
void cmdConfigPCA9685(int n) {
  if (n != 5) { Serial.println(F("ERR USAGE: CONFIG PCA9685 <addrHex> <channel> <freqHz>")); return; }
  detachMotor();
  uint8_t addr = (uint8_t)strtol(tok[2], NULL, 16);
  int channel = atoi(tok[3]);
  float freq = atof(tok[4]);

  if (pcaBackend) { delete pcaBackend; pcaBackend = nullptr; }
  pcaBackend = new PCA9685Backend(Wire, addr, freq);
  if (!pcaBackend->begin()) {
    Serial.println(F("ERR PCA9685_BEGIN_FAILED"));
    delete pcaBackend;
    pcaBackend = nullptr;
    return;
  }
  pcaChannel = channel;
  motorType = MOTOR_PCA9685;
  Serial.println(F("OK"));
}

// tok[0]="CONFIG" tok[1]="ENCODER" tok[2]="AS5600", n=3
void cmdConfigEncoder(int n) {
  if (n != 3 || strcmp(tok[2], "AS5600") != 0) {
    Serial.println(F("ERR USAGE: CONFIG ENCODER AS5600"));
    return;
  }
  Wire.begin();
  if (!encoder.begin()) {
    Serial.println(F("ERR AS5600_BEGIN_FAILED"));
    encoderConfigured = false;
    return;
  }
  encoderConfigured = true;
  Serial.println(F("OK"));
}

void cmdMove(int n) {
  if (n != 2) { Serial.println(F("ERR USAGE: MOVE <us>")); return; }
  int us = atoi(tok[1]);
  if (motorType == MOTOR_SERVO) {
    rcServo.writeMicroseconds(us);
    Serial.println(F("OK"));
  } else if (motorType == MOTOR_PCA9685 && pcaBackend) {
    pcaBackend->setChannelPulseUs(pcaChannel, (uint16_t)us);
    Serial.println(F("OK"));
  } else {
    Serial.println(F("ERR NOT_CONFIGURED"));
  }
}

void cmdRead() {
  if (!encoderConfigured) { Serial.println(F("ERR NOT_CONFIGURED")); return; }
  float angle = encoder.readAngle(RotaryUnit::DEG);
  bool valid = encoder.isValid();
  // Magnet status is a live I2C probe on top of the angle read (isConnected()
  // inside magnetStatusCode()) -- a second-or-so of extra I2C traffic per
  // READ, worth it so a bad reading always comes with a reason, not just a
  // 0/1. Matters most once the servo's installed in its real application,
  // where the magnet/sensor air gap may no longer match the bench setup
  // used to calibrate it.
  const char* status = encoder.magnetStatusCode();
  Serial.print(F("DATA "));
  Serial.print(angle, 4);
  Serial.print(' ');
  Serial.print(valid ? 1 : 0);
  Serial.print(' ');
  Serial.println(status);
}

void cmdZero(int n) {
  if (n != 2) { Serial.println(F("ERR USAGE: ZERO <angle_deg>")); return; }
  if (!encoderConfigured) { Serial.println(F("ERR NOT_CONFIGURED")); return; }
  float deg = atof(tok[1]);
  encoder.setCurrentPosition(deg * DEG_TO_RAD);
  Serial.println(F("OK"));
}

void cmdStop() {
  detachMotor();
  Serial.println(F("OK"));
}

void setup() {
  Serial.begin(115200);
  while (!Serial) { }
  Serial.println(F("# ServoCalibrator_Companion ready"));
}

void loop() {
  readLine();
  int n = tokenize();
  if (n == 0) return;

  if (strcmp(tok[0], "PING") == 0) {
    Serial.println(F("OK PONG"));
  } else if (strcmp(tok[0], "CONFIG") == 0 && n >= 2) {
    if (strcmp(tok[1], "SERVO") == 0) {
      cmdConfigServo(n);
    } else if (strcmp(tok[1], "PCA9685") == 0) {
      cmdConfigPCA9685(n);
    } else if (strcmp(tok[1], "ENCODER") == 0) {
      cmdConfigEncoder(n);
    } else {
      Serial.println(F("ERR UNKNOWN_CONFIG_TARGET"));
    }
  } else if (strcmp(tok[0], "MOVE") == 0) {
    cmdMove(n);
  } else if (strcmp(tok[0], "READ") == 0) {
    cmdRead();
  } else if (strcmp(tok[0], "ZERO") == 0) {
    cmdZero(n);
  } else if (strcmp(tok[0], "STOP") == 0) {
    cmdStop();
  } else {
    Serial.println(F("ERR UNKNOWN_CMD"));
  }
}
