#pragma once
#include <ServoCalibrationTable.h>

// Same table as UMI_CalTable_HWTest -- generated live on this rig by
// Universal-Motor-Interface's examples/RCServoAutoCalibration
// (Nano + servo on A3 + AS5600), 350-2630us / 214.01deg stroke.
// See ../UMI_CalTable_HWTest/GenTable.h for the original capture.

const float MAX_ANGLE_RAD = 3.735243f;  // radians(214.01f)

static const CalPoint genTable[] PROGMEM = {
  { 350, 0 },       // 0.0 deg
  { 466, 1126 },    // 11.3 deg
  { 571, 2253 },    // 22.5 deg
  { 679, 3379 },    // 33.8 deg
  { 785, 4506 },    // 45.1 deg
  { 898, 5632 },    // 56.3 deg
  { 1015, 6758 },   // 67.6 deg
  { 1135, 7885 },   // 78.8 deg
  { 1262, 9011 },   // 90.1 deg
  { 1400, 10137 },  // 101.4 deg
  { 1527, 11264 },  // 112.6 deg
  { 1671, 12390 },  // 123.9 deg
  { 1798, 13517 },  // 135.2 deg
  { 1914, 14643 },  // 146.4 deg
  { 2034, 15769 },  // 157.7 deg
  { 2159, 16896 },  // 169.0 deg
  { 2283, 18022 },  // 180.2 deg
  { 2397, 19149 },  // 191.5 deg
  { 2507, 20275 },  // 202.7 deg
  { 2630, 21401 },  // 214.0 deg
};
const uint8_t GEN_TABLE_LEN = sizeof(genTable) / sizeof(genTable[0]);
