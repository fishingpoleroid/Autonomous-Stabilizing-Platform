// Rotary Stewart platform stabilizer (ESP32)
// Same PID structure as the PyBullet sim: tilt error -> PID -> tilt command -> IK -> servo ticks.
// Hardware, geometry and servo settings live in stewart_common.h.

#include "stewart_common.h"

// ------------------------------------------------------------------ controller
float Kp = 1.0f;
float Ki = 0.0f;
float Kd = 0.0f;
const float INTEGRAL_LIMIT = 2.0f;
float MAX_ANGLE = 20.0f;     // PID output clamp; the reachable range is smaller (printed at boot)

const uint32_t LOOP_HZ = 100;
const uint32_t LOOP_US = 1000000UL / LOOP_HZ;

// ------------------------------------------------------------------ state
float prevErrRoll = 0, prevErrPitch = 0, sumErrRoll = 0, sumErrPitch = 0;
bool  enabled = true;
bool  showTelemetry = true;
int16_t lastPos[6];

uint32_t imuFails = 0, scaledCycles = 0, lateMaxUs = 0;
uint8_t  consecFails = 0;

void goHome() {
  for (int i = 0; i < 6; i++) lastPos[i] = homeTick(i);
  writeServos(lastPos);
}

// ------------------------------------------------------------------ serial tuning (non-blocking)
void processCommand(String s) {
  s.trim(); s.toLowerCase();
  if (s.length() == 0) return;
  if (s.startsWith("kp "))      Kp = s.substring(3).toFloat();
  else if (s.startsWith("ki ")) Ki = s.substring(3).toFloat();
  else if (s.startsWith("kd ")) Kd = s.substring(3).toFloat();
  else if (s == "z") { goHome(); delay(1000); captureBaseline(); }
  else if (s == "e") { enabled = !enabled; if (!enabled) goHome(); }
  else if (s == "r") { prevErrRoll = prevErrPitch = sumErrRoll = sumErrPitch = 0; }
  else if (s.startsWith("mup ")) HORN_MAX_UP_DEG = s.substring(4).toFloat();     // horn travel limits, degrees
  else if (s.startsWith("mdn ")) HORN_MAX_DOWN_DEG = s.substring(4).toFloat();
  else if (s.startsWith("mxa ")) MAX_ANGLE = s.substring(4).toFloat();
  else if (s.startsWith("show")) showTelemetry = !showTelemetry;
  Serial.printf("Kp=%.2f Ki=%.2f Kd=%.2f MAX_ANGLE=%.1f horn limits +%.0f/-%.0f enabled=%d\n",
                Kp, Ki, Kd, MAX_ANGLE, HORN_MAX_UP_DEG, HORN_MAX_DOWN_DEG, enabled);
}

void handleSerial() {
  static String buf;
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\r') continue;
    if (c == '\n') { processCommand(buf); buf = ""; }
    else if (buf.length() < 40) buf += c;
  }
}

// ------------------------------------------------------------------ setup / loop
void setup() {
  Serial.begin(115200);
  delay(500);

  initHardware();
  buildGeometry();
  printReachable();
  printSensitivity();
  goHome();
  delay(2000);
  captureBaseline();

  Serial.println("Ready. Commands: kp/ki/kd <val>, mup/mdn <deg>, mxa <deg>, z, e, r, show");
}

void loop() {
  static uint32_t nextTick = micros();
  static uint32_t count = 0;
  static float scale = 1.0f;

  handleSerial();

  uint32_t now = micros();
  int32_t late = (int32_t)(now - nextTick);
  if (late < 0) return;
  if ((uint32_t)late > lateMaxUs) lateMaxUs = late;
  nextTick += LOOP_US;
  if ((int32_t)(now - nextTick) > (int32_t)LOOP_US * 5) nextTick = now;

  if (!enabled) return;

  float roll, pitch;
  if (!readTilt(roll, pitch)) {            // bad I2C transfer: keep the last command, count it, recover the bus
    imuFails++;
    if (++consecFails >= 5) { recoverI2C(); consecFails = 0; }
    return;
  }
  consecFails = 0;

  float errRoll  = 0.0f - roll;
  float errPitch = 0.0f - pitch;

  float newSumRoll  = constrain(sumErrRoll  + errRoll,  -INTEGRAL_LIMIT, INTEGRAL_LIMIT);
  float newSumPitch = constrain(sumErrPitch + errPitch, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);

  float rollCmd  = Kp * errRoll  + Ki * newSumRoll  + Kd * (errRoll  - prevErrRoll);
  float pitchCmd = Kp * errPitch + Ki * newSumPitch + Kd * (errPitch - prevErrPitch);

  rollCmd  = constrain(rollCmd,  -MAX_ANGLE, MAX_ANGLE);
  pitchCmd = constrain(pitchCmd, -MAX_ANGLE, MAX_ANGLE);

  prevErrRoll = errRoll; prevErrPitch = errPitch;

  int16_t pos[6];
  scale = solveTicksScaled(rollCmd, pitchCmd, pos);
  memcpy(lastPos, pos, sizeof(pos));
  writeServos(lastPos);

  if (scale >= 1.0f) { sumErrRoll = newSumRoll; sumErrPitch = newSumPitch; }   // anti-windup
  else scaledCycles++;

  if (++count % 10 == 0) {
    if (showTelemetry) {
      Serial.printf("roll %6.2f pitch %6.2f | cmd %6.2f %6.2f | scale %.2f | imuFail %lu scaled %lu lateMax %lu us | ticks %d %d %d %d %d %d\n",
                    roll, pitch, rollCmd, pitchCmd, scale,
                    (unsigned long)imuFails, (unsigned long)scaledCycles, (unsigned long)lateMaxUs,
                    lastPos[0], lastPos[1], lastPos[2], lastPos[3], lastPos[4], lastPos[5]);
    }
    lateMaxUs = 0;
  }
}