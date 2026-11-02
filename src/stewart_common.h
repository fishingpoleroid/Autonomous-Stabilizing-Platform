// Shared configuration, kinematics and IMU code for main.cpp and test_pid_single.cpp.
// Edit hardware / geometry / servo settings HERE ONLY, so both firmwares always agree.
#pragma once

#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <SMS_STS.h>

// ================================================================== hardware
#define BNO_ADDR 0x28
#define I2C_SDA  22
#define I2C_SCL  23
#define BNO_EULER_REG 0x1A          // EUL_Heading_LSB; heading, roll, pitch follow (int16, 16 LSB/deg)
Adafruit_BNO055 bno(55, BNO_ADDR, &Wire);

#define S_RXD 18
#define S_TXD 19
SMS_STS st;

const int NUM_SERVOS = 6;
// Servo ID of each IK leg (leg 0..5 = the sim's phi arrays, see the table printed at boot).
uint8_t SERVO_IDS[NUM_SERVOS] = {1, 2, 3, 4, 5, 6};

#define HORN_ZERO_TICK 2048         // servo tick where the horn is exactly HORIZONTAL
#define SERVO_SPEED    3073
#define SERVO_ACC      150
const float TICKS_PER_DEG = 4096.0f / 360.0f;

// +1: a higher tick value moves that horn UP. -1: a higher tick value moves it DOWN.
// Verify each servo with test_pid_single's "horn 20" command (the horn must rise).
const int SERVO_DIR[NUM_SERVOS] = {+1, -1, +1, -1, +1, -1};

float HORN_MAX_UP_DEG   = 90.0f;
float HORN_MAX_DOWN_DEG = 90.0f;

// Horn angle above horizontal when the platform is at its home pose. Raising it centres the
// useful travel (up 85 / down 20 around horizontal is very lopsided). Set 0 for horizontal horns.
// The servos go to this angle at boot, so the platform sits higher than with horizontal horns.
const float HOME_HORN_DEG = 30.0f;

// ================================================================== geometry (metres / degrees)
const float R_SERVO   = 0.113f;
const float R_PLAT    = 0.125f;
const float GAMMA_DEG = 12.0f;
const float HORN_LEN  = 0.020f;
const float ROD_LEN   = 0.156f;

// inv_kinematics.py effectively uses phi_P as the base (servo) joints and phi_B as the platform joints.
const bool SERVOS_ON_PHI_P = true;

// Servos sit in pairs on the long sides of the base, and the two horns of a pair point toward
// each other (toward the middle of that side). Set false if yours point away from each other.
const bool HORN_TOWARD_PAIR_CENTER = true;

// ================================================================== IMU axis mapping
// Use the guided test in test.cpp to find these.
const bool  SWAP_ROLL_PITCH = true;
const float ROLL_SIGN  = -1.0f;
const float PITCH_SIGN = +1.0f;

// ================================================================== derived geometry state
float BASE_ANGLE_DEG[6];
float PLAT_ANGLE_DEG[6];
float baseX[6], baseY[6];
float platX[6], platY[6];
float hornDir[6];                   // horizontal direction the horn points at, radians
float homeHeight = 0.0f;
float rollOffset = 0.0f, pitchOffset = 0.0f;

static float angDiffDeg(float a, float b) { return fmodf(a - b + 540.0f, 360.0f) - 180.0f; }

int legOfId(int id) {
  for (int i = 0; i < NUM_SERVOS; i++) if (SERVO_IDS[i] == id) return i;
  return -1;
}

// Servo tick for a horn angle (degrees above horizontal) on a given leg.
int tickForHornDeg(int leg, float hornDeg) {
  return HORN_ZERO_TICK + SERVO_DIR[leg] * (int)lroundf(hornDeg * TICKS_PER_DEG);
}
int homeTick(int leg) { return tickForHornDeg(leg, HOME_HORN_DEG); }

// ================================================================== kinematics
void rotationMatrix(float rollDeg, float pitchDeg, float yawDeg, float R[3][3]) {
  float r = radians(rollDeg), p = radians(pitchDeg), y = radians(yawDeg);
  float cr = cosf(r), sr = sinf(r), cp = cosf(p), sp = sinf(p), cy = cosf(y), sy = sinf(y);
  float M[3][3] = {{cp, 0, sp}, {sr * sp, cr, -sr * cp}, {-cr * sp, sr, cr * cp}};   // Rx * Ry
  for (int i = 0; i < 3; i++) {
    R[i][0] = M[i][0] * cy + M[i][1] * sy;
    R[i][1] = -M[i][0] * sy + M[i][1] * cy;
    R[i][2] = M[i][2];
  }
}

// Horn angle (rad, above horizontal) for leg i at the given platform pose. false = unreachable.
bool legAngle(int i, const float R[3][3], const float T[3], float &alpha) {
  float px = R[0][0] * platX[i] + R[0][1] * platY[i] + T[0];
  float py = R[1][0] * platX[i] + R[1][1] * platY[i] + T[1];
  float pz = R[2][0] * platX[i] + R[2][1] * platY[i] + T[2];

  float lx = px - baseX[i], ly = py - baseY[i], lz = pz;
  float L = lx * lx + ly * ly + lz * lz - (ROD_LEN * ROD_LEN - HORN_LEN * HORN_LEN);
  float M = 2.0f * HORN_LEN * lz;
  float N = 2.0f * HORN_LEN * (lx * cosf(hornDir[i]) + ly * sinf(hornDir[i]));
  float d = sqrtf(M * M + N * N);
  if (d < 1e-9f || fabsf(L / d) > 1.0f) return false;
  alpha = asinf(L / d) - atan2f(N, M);
  return true;
}

void buildGeometry() {
  const float g = GAMMA_DEG;
  const float phiB[6]  = {210 + g, 210 - g, 90 + g, 90 - g, 330 + g, 330 - g};
  const float phiP[6]  = {270 - g, 150 + g, 150 - g, 30 + g, 30 - g, 270 + g};
  const float cenB[6]  = {210, 210, 90, 90, 330, 330};     // centre of each pair
  const float cenP[6]  = {270, 150, 150, 30, 30, 270};

  for (int i = 0; i < 6; i++) {
    BASE_ANGLE_DEG[i] = SERVOS_ON_PHI_P ? phiP[i] : phiB[i];
    PLAT_ANGLE_DEG[i] = SERVOS_ON_PHI_P ? phiB[i] : phiP[i];
    float b = radians(BASE_ANGLE_DEG[i]), p = radians(PLAT_ANGLE_DEG[i]);
    baseX[i] = R_SERVO * cosf(b); baseY[i] = R_SERVO * sinf(b);
    platX[i] = R_PLAT * cosf(p);  platY[i] = R_PLAT * sinf(p);

    // Horn direction at home: tangential, toward (or away from) the middle of this servo's pair.
    float centre = SERVOS_ON_PHI_P ? cenP[i] : cenB[i];
    float side = angDiffDeg(centre, BASE_ANGLE_DEG[i]) >= 0 ? 1.0f : -1.0f;   // +1 = CCW tangent
    if (!HORN_TOWARD_PAIR_CENTER) side = -side;
    hornDir[i] = radians(BASE_ANGLE_DEG[i] + 90.0f * side);
  }

  Serial.println("leg | servo ID | shaft angle | platform joint | horn points at");
  for (int i = 0; i < 6; i++)
    Serial.printf(" %d  |    %d     |   %6.1f    |     %6.1f     |   %6.1f deg\n",
                  i, SERVO_IDS[i], BASE_ANGLE_DEG[i], PLAT_ANGLE_DEG[i], degrees(hornDir[i]));

  // Home height: horn at HOME_HORN_DEG above horizontal, rod must reach the platform joint.
  float a0 = radians(HOME_HORN_DEG), hSum = 0, hMin = 1e9f, hMax = -1e9f;
  for (int i = 0; i < 6; i++) {
    float hx = baseX[i] + HORN_LEN * cosf(a0) * cosf(hornDir[i]);
    float hy = baseY[i] + HORN_LEN * cosf(a0) * sinf(hornDir[i]);
    float hz = HORN_LEN * sinf(a0);
    float dx = platX[i] - hx, dy = platY[i] - hy;
    float h2 = ROD_LEN * ROD_LEN - dx * dx - dy * dy;
    float h = (h2 > 0 ? sqrtf(h2) : 0) + hz;
    hSum += h; hMin = min(hMin, h); hMax = max(hMax, h);
  }
  homeHeight = hSum / 6.0f;
  Serial.printf("home height: %.2f cm (shaft plane to joint plane), horn at %.0f deg above horizontal\n",
                homeHeight * 100.0f, HOME_HORN_DEG);

  float R[3][3]; rotationMatrix(0, 0, 0, R);
  float T[3] = {0, 0, homeHeight};
  for (int i = 0; i < 6; i++) {
    float a;
    if (!legAngle(i, R, T, a) || fabsf(degrees(a) - HOME_HORN_DEG) > 0.5f)
      Serial.printf("WARNING: leg %d home pose check failed, check geometry\n", i);
  }
}

// Servo ticks for a platform tilt. false if a leg can't reach it or needs more horn travel than allowed.
bool solveTicks(float rollDeg, float pitchDeg, int16_t outPos[6]) {
  float R[3][3]; rotationMatrix(rollDeg, pitchDeg, 0.0f, R);
  float T[3] = {0, 0, homeHeight};
  int16_t tmp[6];
  for (int i = 0; i < 6; i++) {
    float a;
    if (!legAngle(i, R, T, a)) return false;
    float deg = degrees(a);
    if (deg > HORN_MAX_UP_DEG || deg < -HORN_MAX_DOWN_DEG) return false;
    tmp[i] = tickForHornDeg(i, deg);
  }
  memcpy(outPos, tmp, sizeof(tmp));
  return true;
}

// Always returns a usable pose: an out-of-reach tilt is shrunk along the same direction to the
// largest reachable one. Returns the scale used (1 = full command).
float solveTicksScaled(float rollDeg, float pitchDeg, int16_t outPos[6]) {
  if (solveTicks(rollDeg, pitchDeg, outPos)) return 1.0f;
  float lo = 0.0f, hi = 1.0f;
  int16_t best[6], tmp[6];
  for (int i = 0; i < 6; i++) best[i] = homeTick(i);
  for (int k = 0; k < 10; k++) {
    float mid = 0.5f * (lo + hi);
    if (solveTicks(rollDeg * mid, pitchDeg * mid, tmp)) { lo = mid; memcpy(best, tmp, sizeof(best)); }
    else hi = mid;
  }
  memcpy(outPos, best, sizeof(best));
  return lo;
}

void printReachable() {
  float minT = 1e9f;
  for (int d = 0; d < 360; d += 15) {
    float cx = cosf(radians((float)d)), cy = sinf(radians((float)d));
    int16_t tmp[6];
    for (float t = 0.25f; t <= 45.0f; t += 0.25f) {
      if (!solveTicks(t * cx, t * cy, tmp)) { minT = min(minT, t); break; }
    }
  }
  Serial.printf("guaranteed tilt range: ~%.2f deg in every direction (bigger commands get scaled down)\n", minT);
}

// Expected servo response, handy for unit tests: ticks of servo movement per degree of COMMANDED platform tilt.
void printSensitivity() {
  Serial.println("expected response, ticks per degree of commanded tilt (PID commands the opposite of the measured tilt):");
  Serial.println("servo ID | leg | roll  | pitch");
  float T[3] = {0, 0, homeHeight};
  for (int i = 0; i < 6; i++) {
    float R[3][3], r1, r2, p1, p2;
    rotationMatrix(+1, 0, 0, R); bool ok1 = legAngle(i, R, T, r1);
    rotationMatrix(-1, 0, 0, R); bool ok2 = legAngle(i, R, T, r2);
    rotationMatrix(0, +1, 0, R); bool ok3 = legAngle(i, R, T, p1);
    rotationMatrix(0, -1, 0, R); bool ok4 = legAngle(i, R, T, p2);
    if (!(ok1 && ok2 && ok3 && ok4)) continue;
    float sr = degrees(r1 - r2) / 2.0f * SERVO_DIR[i] * TICKS_PER_DEG;
    float sp = degrees(p1 - p2) / 2.0f * SERVO_DIR[i] * TICKS_PER_DEG;
    Serial.printf("   %d     |  %d  | %+6.1f | %+6.1f\n", SERVO_IDS[i], i, sr, sp);
  }
}

// ================================================================== servo output
void writeServos(const int16_t pos[6]) {
  uint16_t spd[6]; uint8_t acc[6];
  for (int i = 0; i < 6; i++) { spd[i] = SERVO_SPEED; acc[i] = SERVO_ACC; }
  st.SyncWritePosEx(SERVO_IDS, NUM_SERVOS, (int16_t *)pos, spd, acc);
}

// ================================================================== IMU
// Direct register read so a failed I2C transfer is detected (Adafruit's getEvent() returns zeros on failure).
bool readEuler(float &heading, float &roll, float &pitch) {
  Wire.beginTransmission(BNO_ADDR);
  Wire.write((uint8_t)BNO_EULER_REG);
  if (Wire.endTransmission(true) != 0) return false;
  if (Wire.requestFrom((uint8_t)BNO_ADDR, (uint8_t)6, (uint8_t)1) != 6) return false;
  uint8_t b[6];
  for (int i = 0; i < 6; i++) b[i] = Wire.read();
  heading = (int16_t)((b[1] << 8) | b[0]) / 16.0f;
  roll    = (int16_t)((b[3] << 8) | b[2]) / 16.0f;
  pitch   = (int16_t)((b[5] << 8) | b[4]) / 16.0f;
  return true;
}

void recoverI2C() {
  Wire.end();
  delay(2);
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(100000);
  Wire.setTimeOut(10);
}

bool readTilt(float &rollDeg, float &pitchDeg) {
  float h, ry, pz;
  if (!readEuler(h, ry, pz)) return false;
  float r = ry - rollOffset;
  float p = pz - pitchOffset;
  if (SWAP_ROLL_PITCH) { float t = r; r = p; p = t; }
  rollDeg  = ROLL_SIGN  * r;
  pitchDeg = PITCH_SIGN * p;
  return true;
}

void captureBaseline() {
  float sr = 0, sp = 0; int got = 0;
  for (int i = 0; i < 100; i++) {
    float h, r, p;
    if (readEuler(h, r, p)) { sr += r; sp += p; got++; }
    delay(10);
  }
  if (got < 50) Serial.printf("WARNING: only %d/100 IMU reads succeeded during baseline\n", got);
  if (got > 0) { rollOffset = sr / got; pitchOffset = sp / got; }
  Serial.printf("baseline roll/pitch (deg): %.2f %.2f\n", rollOffset, pitchOffset);
}

// Proves which firmware is running and which leg each servo ID is assigned to.
void printBanner() {
  Serial.printf("\n=== firmware built %s %s ===\n", __DATE__, __TIME__);
  Serial.print("SERVO_IDS by leg:");
  for (int i = 0; i < NUM_SERVOS; i++) Serial.printf("  leg%d=ID%d", i, SERVO_IDS[i]);
  Serial.println();
}

// Common start-up: IMU, servo bus, ping. Call from setup().
void initHardware() {
  printBanner();
  Wire.begin(I2C_SDA, I2C_SCL);
  if (!bno.begin(OPERATION_MODE_IMUPLUS)) {
    Serial.println("No BNO055 detected.");
    while (1) delay(10);
  }
  delay(1000);
  Wire.setClock(100000);      // BNO055 clock-stretches; 400 kHz is unreliable on the ESP32
  Wire.setTimeOut(10);

  Serial1.begin(1000000, SERIAL_8N1, S_RXD, S_TXD);
  st.pSerial = &Serial1;
  delay(500);

  for (int i = 0; i < NUM_SERVOS; i++) {
    if (st.Ping(SERVO_IDS[i]) == -1)
      Serial.printf("WARNING: servo ID %d did not answer ping.\n", SERVO_IDS[i]);
  }
}