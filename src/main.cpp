#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <string.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <SMS_STS.h>

static const int  SERVO_RX_PIN = 18;
static const int  SERVO_TX_PIN = 19;
static const long SERVO_BAUD   = 1000000;
static const int  IMU_SDA_PIN  = 22;  
static const int  IMU_SCL_PIN  = 23;
#define BNO_ADDR 0x28                 

static u8   SERVO_IDS[6] = {1, 2, 3, 4, 5, 6};
static int8_t SERVO_DIR[6] = {+1, +1, +1, +1, +1, +1};   // +1: positive count raises the horn tip. Change live: "dir <1-6> <+1|-1>"
static const int    SERVO_CENTER = 2048;                 // horn must be at its "home" angle here
static const float  COUNTS_PER_DEG = 4096.0f / 360.0f;   // ST3215: 4096 counts / rev
static const float  SERVO_LIMIT_DEG = 60.0f;             // max horn travel from home (+/-)
static const u16    SERVO_SPEED = 3000;                  // steps/s (0 = max)
static const u8     SERVO_ACC   = 0;                     // 0 = no accel ramp (fast response)

// ---- Geometry (metres / degrees) ----
// Base anchor = servo shaft axis point. Platform anchor = rod end on the top plate.
static const float R_BASE   = 0.110f;   // MEASURE: radius of servo shaft circle
static const float R_PLAT   = 0.120f;   // MEASURE: radius of rod-end circle on top plate
static const float GAMMA_BASE_DEG = 30.0f;  // MEASURE: half the angle between a servo pair
static const float GAMMA_PLAT_DEG = 35.0f;  // MEASURE: half the angle between a rod-end pair
static const float HORN_LEN = 0.020f;   // MEASURE: servo horn length (shaft centre -> rod pivot)
static const float ROD_LEN  = 0.160f;   // MEASURE: pushrod length (ball-joint centre to centre)

// Home height of the top plate above the servo-shaft plane.
// 0 = auto-compute so that every horn is exactly horizontal at home
// (best choice: equal travel up and down). Otherwise set metres.
static const float HOME_Z_OVERRIDE = 0.0f;

// Horn swing direction (azimuth of the horn when horizontal, degrees from +X).
// false = auto: horn points along the horizontal direction of its own leg at home.
// true  = use BETA_MANUAL_DEG below (MEASURE on your build).
static const bool  USE_MANUAL_BETA = false;
static const float BETA_MANUAL_DEG[6] = {0, 0, 0, 0, 0, 0};

// ---- Control ----
static const float LOOP_HZ = 100.0f;           // BNO055 fusion output is 100 Hz

// PID, position form, one identical loop for roll and pitch. Same structure as the sim's loop():
//     err = 0 - measured tilt
//     cmd = Kp*err + I + Kd*d(err)/dt          (cmd = tilt of the top plate RELATIVE TO THE BASE)
//   Kp [-]   degrees of tilt command per degree of tilt error
//   Ki [1/s] degrees of tilt command per second, per degree of error
//   Kd [s]   degrees of tilt command per (degree/second) of tilt rate
//
// IMPORTANT: the IMU is on the top plate, so  measured = base tilt + commanded tilt.
// P alone therefore only corrects a fraction Kp/(1+Kp) of the base tilt (Kp=0.1 -> 9 %, Kp=1 -> 50 %,
// Kp=2 -> 67 %); the I term removes the rest. The sim uses Kp=15 (94 % from P alone) because its
// actuators have no latency. On the real robot (IMU filter + 100 Hz loop + servo lag) a big Kp
// oscillates, so keep Kp around 1..2 and raise Ki for stiffness.
// Tuning: Kd = 0. Start Kp = 1, Ki = 10. Raise Ki (15, 20 ...) for a faster return to level; if it rings
// after a knock go back to ~70 %. Live: "kp 1.5", "ki 15" ...
static float Kp = 1.0f, Ki = 10.0f, Kd = 0.0f;

// Largest tilt command [deg]. The mechanism may reach less (see the "reach" value in the telemetry);
// solveSafe() cuts the command back to what is reachable anyway.
static const float MAX_TILT_CMD_DEG = 20.0f;
// Largest change of the tilt command per second [deg/s]. The servos cannot tilt the platform faster
// than about (SERVO_SPEED / COUNTS_PER_DEG) * HORN_LEN / R_PLAT = ~53 deg/s with these numbers.
static const float MAX_TILT_RATE_DPS = 50.0f;
static const float MEAS_LPF_HZ = 20.0f;        // low-pass on measured tilt, 0 = off
static const float D_LPF_HZ    = 10.0f;        // low-pass on the rate for the D term, 0 = off

// ---- IMU link health ----
static const int IMU_RECOVER_AFTER_FAILS = 20;   // consecutive failed reads (= 0.2 s) before restarting I2C

// ============================================================================
//  Globals
// ============================================================================
static const float DEG2RAD = (float)M_PI / 180.0f;
static const float RAD2DEG = 180.0f / (float)M_PI;

SMS_STS st;
Adafruit_BNO055 bno = Adafruit_BNO055(55, BNO_ADDR, &Wire);

struct Mat3 { float m[3][3]; };

static float bx[6], by[6];        // base anchors (servo shaft points), z = 0
static float px[6], py[6];        // platform anchors in platform frame, z = 0
static float beta[6];             // horn swing azimuth (rad)
static float homeZ = 0.0f;
static float alphaHome[6];        // horn angle at home (rad)

static Mat3 baselineR = {{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}};   // IMU orientation at tare
static bool pidEnabled = true;
static float manualRoll = 0, manualPitch = 0;

struct AxisCtl {
  float m = 0;         // filtered measured tilt                      [deg]
  float mPrev = 0;
  float rate = 0;      // filtered tilt rate                          [deg/s]
  float iTerm = 0;     // integral term, already multiplied by Ki     [deg]
  float cmd = 0;       // last applied tilt command (relative to base)[deg]
  bool  primed = false;
};
static AxisCtl ctlRoll, ctlPitch;

static uint32_t lastLoopUs = 0, lastCtlUs = 0;
static uint32_t loopCount  = 0;
static uint32_t imuFailTotal = 0;
static int      imuFailStreak = 0;
static const uint32_t LOOP_PERIOD_US = (uint32_t)(1000000.0f / LOOP_HZ);

// ============================================================================
//  Small math helpers
// ============================================================================
static float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }

static Mat3 matMul(const Mat3 &a, const Mat3 &b) {
  Mat3 r;
  for (int i = 0; i < 3; i++)
    for (int j = 0; j < 3; j++) {
      r.m[i][j] = 0;
      for (int k = 0; k < 3; k++) r.m[i][j] += a.m[i][k] * b.m[k][j];
    }
  return r;
}

static Mat3 transpose(const Mat3 &a) {
  Mat3 r;
  for (int i = 0; i < 3; i++)
    for (int j = 0; j < 3; j++) r.m[i][j] = a.m[j][i];
  return r;
}

static Mat3 rotX(float deg) {
  float c = cosf(deg * DEG2RAD), s = sinf(deg * DEG2RAD);
  return Mat3{{{1, 0, 0}, {0, c, -s}, {0, s, c}}};
}
static Mat3 rotY(float deg) {
  float c = cosf(deg * DEG2RAD), s = sinf(deg * DEG2RAD);
  return Mat3{{{c, 0, s}, {0, 1, 0}, {-s, 0, c}}};
}
static Mat3 rotZ(float deg) {
  float c = cosf(deg * DEG2RAD), s = sinf(deg * DEG2RAD);
  return Mat3{{{c, -s, 0}, {s, c, 0}, {0, 0, 1}}};
}

// Same convention as the sim IK: R = Rx * Ry * Rz
static Mat3 buildR(float rollDeg, float pitchDeg, float yawDeg) {
  return matMul(matMul(rotX(rollDeg), rotY(pitchDeg)), rotZ(yawDeg));
}

// Same extraction as matrix_to_xyz_euler_deg() in the sim
static void matToEulerDeg(const Mat3 &R, float &roll, float &pitch, float &yaw) {
  pitch = asinf(clampf(R.m[0][2], -1.0f, 1.0f)) * RAD2DEG;
  roll  = atan2f(-R.m[1][2], R.m[2][2]) * RAD2DEG;
  yaw   = atan2f(-R.m[0][1], R.m[0][0]) * RAD2DEG;
}

static Mat3 quatToMat(float w, float x, float y, float z) {
  float n = sqrtf(w * w + x * x + y * y + z * z);
  if (n < 1e-6f) { w = 1; x = y = z = 0; n = 1; }
  w /= n; x /= n; y /= n; z /= n;
  Mat3 R;
  R.m[0][0] = 1 - 2 * (y * y + z * z); R.m[0][1] = 2 * (x * y - w * z);     R.m[0][2] = 2 * (x * z + w * y);
  R.m[1][0] = 2 * (x * y + w * z);     R.m[1][1] = 1 - 2 * (x * x + z * z); R.m[1][2] = 2 * (y * z - w * x);
  R.m[2][0] = 2 * (x * z - w * y);     R.m[2][1] = 2 * (y * z + w * x);     R.m[2][2] = 1 - 2 * (x * x + y * y);
  return R;
}

// ============================================================================
//  Geometry + rotary inverse kinematics
// ============================================================================
/*
 * Anchor layout (same angles as the sim; the sim's frame() returned (P,B) but
 * the caller unpacked them as (B,P), so its *effective* base used the "phi_P"
 * angles and its platform the "phi_B" angles. That effective layout is used here
 * and named correctly):
 *
 *   base anchors     : 270-g, 150+g, 150-g, 30+g, 30-g, 270+g
 *   platform anchors : 210+g, 210-g,  90+g, 90-g, 330+g, 330-g
 *
 * Replace with your measured angles if your build differs.
 */
static bool initGeometry() {
  const float gB = GAMMA_BASE_DEG, gP = GAMMA_PLAT_DEG;
  const float baseAng[6] = {270 - gB, 150 + gB, 150 - gB, 30 + gB, 30 - gB, 270 + gB};
  const float platAng[6] = {210 + gP, 210 - gP, 90 + gP, 90 - gP, 330 + gP, 330 - gP};

  float zSum = 0;
  for (int i = 0; i < 6; i++) {
    bx[i] = R_BASE * cosf(baseAng[i] * DEG2RAD);
    by[i] = R_BASE * sinf(baseAng[i] * DEG2RAD);
    px[i] = R_PLAT * cosf(platAng[i] * DEG2RAD);
    py[i] = R_PLAT * sinf(platAng[i] * DEG2RAD);

    float dx = px[i] - bx[i], dy = py[i] - by[i];
    beta[i] = USE_MANUAL_BETA ? BETA_MANUAL_DEG[i] * DEG2RAD : atan2f(dy, dx);

    // height at which the horn is horizontal and the rod just reaches the plate anchor
    float hx = dx - HORN_LEN * cosf(beta[i]);
    float hy = dy - HORN_LEN * sinf(beta[i]);
    float z2 = ROD_LEN * ROD_LEN - hx * hx - hy * hy;
    if (z2 <= 0) {
      Serial.printf("Geometry error: leg %d rod too short for this layout\n", i);
      return false;
    }
    zSum += sqrtf(z2);
  }
  homeZ = (HOME_Z_OVERRIDE > 0.0f) ? HOME_Z_OVERRIDE : zSum / 6.0f;
  return true;
}

/*
 * Rotary IK. For leg i the horn tip is
 *     h = a * ( cos(alpha)cos(beta), cos(alpha)sin(beta), sin(alpha) )
 * and the rod must satisfy |l - h| = s, where l = (T + R*p_i) - b_i.
 * That reduces to   L = N cos(alpha) + M sin(alpha)   with
 *     L = |l|^2 - (s^2 - a^2)
 *     M = 2a * l_z
 *     N = 2a * (l_x cos(beta) + l_y sin(beta))
 *  =>  alpha = asin(L / sqrt(M^2 + N^2)) - atan2(N, M)
 * alpha > 0 raises the horn tip. Returns false if any leg is out of reach.
 */
static bool solveIK(const float trans[3], const float rotDeg[3], float alpha[6]) {
  Mat3 R = buildR(rotDeg[0], rotDeg[1], rotDeg[2]);
  const float a = HORN_LEN, s = ROD_LEN;
  bool ok = true;

  for (int i = 0; i < 6; i++) {
    float qx = trans[0] + R.m[0][0] * px[i] + R.m[0][1] * py[i];
    float qy = trans[1] + R.m[1][0] * px[i] + R.m[1][1] * py[i];
    float qz = trans[2] + homeZ + R.m[2][0] * px[i] + R.m[2][1] * py[i];

    float lx = qx - bx[i], ly = qy - by[i], lz = qz;   // base anchor z = 0

    float L = lx * lx + ly * ly + lz * lz - (s * s - a * a);
    float M = 2 * a * lz;
    float N = 2 * a * (lx * cosf(beta[i]) + ly * sinf(beta[i]));
    float d = sqrtf(M * M + N * N);
    if (d < 1e-6f) { ok = false; alpha[i] = 0; continue; }

    float ratio = L / d;
    if (ratio > 1.0f || ratio < -1.0f) { ok = false; ratio = clampf(ratio, -1.0f, 1.0f); }
    alpha[i] = asinf(ratio) - atan2f(N, M);
  }
  return ok;
}

// A tilt is reachable when the IK has a solution AND every horn stays within its travel limit.
static bool poseReachable(float rollDeg, float pitchDeg, float alpha[6]) {
  const float trans[3] = {0, 0, 0};
  const float rot[3]   = {rollDeg, pitchDeg, 0};
  if (!solveIK(trans, rot, alpha)) return false;
  for (int i = 0; i < 6; i++)
    if (fabsf(alpha[i] - alphaHome[i]) * RAD2DEG > SERVO_LIMIT_DEG) return false;
  return true;
}

/*
 * Solves the tilt, or the largest scaled-down version of it that is reachable. ALWAYS fills alpha[]
 * with something that can be sent. Returns the scale that was applied (1 = as asked, 0 = only level).
 * The scale is found by bisection (8 steps), so a command at the edge of the workspace is cut back by
 * < 0.5 %. The controller uses the returned scale to keep its state inside the reachable region.
 */
static float solveSafe(float rollDeg, float pitchDeg, float alpha[6]) {
  if (poseReachable(rollDeg, pitchDeg, alpha)) return 1.0f;
  if (!poseReachable(0.0f, 0.0f, alpha)) {          // even level fails: fall back to the home angles
    for (int i = 0; i < 6; i++) alpha[i] = alphaHome[i];
    return 0.0f;
  }
  float lo = 0.0f, hi = 1.0f;                        // lo: reachable, hi: not reachable
  float tmp[6];
  for (int k = 0; k < 8; k++) {
    float mid = 0.5f * (lo + hi);
    if (poseReachable(rollDeg * mid, pitchDeg * mid, tmp)) { lo = mid; memcpy(alpha, tmp, sizeof(tmp)); }
    else hi = mid;
  }
  return lo;
}

// ============================================================================
//  Servo output
// ============================================================================
static void sendHornAngles(const float alpha[6]) {
  s16 pos[6];
  u16 spd[6];
  u8  acc[6];
  for (int i = 0; i < 6; i++) {
    float deltaDeg = (alpha[i] - alphaHome[i]) * RAD2DEG;       // same idea as piston_extensions in sim
    deltaDeg = clampf(deltaDeg, -SERVO_LIMIT_DEG, SERVO_LIMIT_DEG);
    int p = SERVO_CENTER + SERVO_DIR[i] * (int)lroundf(deltaDeg * COUNTS_PER_DEG);
    pos[i] = (s16)constrain(p, 0, 4095);
    spd[i] = SERVO_SPEED;
    acc[i] = SERVO_ACC;
  }
  st.SyncWritePosEx(SERVO_IDS, 6, pos, spd, acc);
}

static void goToPose(float rollDeg, float pitchDeg) {
  float alpha[6];
  float scale = solveSafe(rollDeg, pitchDeg, alpha);
  if (scale < 1.0f) Serial.printf("pose out of reach - using %.0f %% of it\n", scale * 100.0f);
  sendHornAngles(alpha);
}

// ---------------------------------------------------------------------------
//  Manual roll / pitch test (no IMU, no PID) - verifies the IK and the servo directions
// ---------------------------------------------------------------------------
// Largest |rod length - ROD_LEN| over the 6 legs for horn angles alpha[] and tilt (roll, pitch).
// ~0 means the IK solution really satisfies the rod constraint (checks the IK maths itself).
static float ikRodError(float rollDeg, float pitchDeg, const float alpha[6]) {
  Mat3 R = buildR(rollDeg, pitchDeg, 0);
  float maxErr = 0;
  for (int i = 0; i < 6; i++) {
    float qx = R.m[0][0] * px[i] + R.m[0][1] * py[i];
    float qy = R.m[1][0] * px[i] + R.m[1][1] * py[i];
    float qz = homeZ + R.m[2][0] * px[i] + R.m[2][1] * py[i];
    float hx = HORN_LEN * cosf(alpha[i]) * cosf(beta[i]);
    float hy = HORN_LEN * cosf(alpha[i]) * sinf(beta[i]);
    float hz = HORN_LEN * sinf(alpha[i]);
    float dx = qx - bx[i] - hx, dy = qy - by[i] - hy, dz = qz - hz;
    float err = fabsf(sqrtf(dx * dx + dy * dy + dz * dz) - ROD_LEN);
    if (err > maxErr) maxErr = err;
  }
  return maxErr;
}

// Moves the top plate to a fixed roll/pitch relative to the base and prints what every leg should do.
// Check on the real robot: a leg whose plate anchor should RISE must have its horn go UP, one that
// should DROP must go DOWN. Any servo where the horn moves the other way -> flip its SERVO_DIR ("dir i -1").
static void manualRollPitch(float rollDeg, float pitchDeg) {
  float alpha[6];
  float scale = solveSafe(rollDeg, pitchDeg, alpha);
  float r = rollDeg * scale, p = pitchDeg * scale;
  if (scale < 1.0f) Serial.printf("pose out of reach - using %.0f %% of it (roll %.2f pitch %.2f)\n", scale * 100.0f, r, p);

  Mat3 R = buildR(r, p, 0);
  Serial.printf("manual roll=%.2f pitch=%.2f | IK rod-length error %.6f m (should be ~0)\n", r, p, ikRodError(r, p, alpha));
  for (int i = 0; i < 6; i++) {
    float plateDz = (R.m[2][0] * px[i] + R.m[2][1] * py[i]) * 1000.0f;       // plate anchor height change [mm]
    float deltaDeg = clampf((alpha[i] - alphaHome[i]) * RAD2DEG, -SERVO_LIMIT_DEG, SERVO_LIMIT_DEG);
    int cnt = SERVO_DIR[i] * (int)lroundf(deltaDeg * COUNTS_PER_DEG);
    Serial.printf("  servo %u: plate anchor %+6.1f mm | horn %s %+6.2f deg | counts %+5d -> %d | dir %+d\n",
                  SERVO_IDS[i], plateDz, deltaDeg >= 0 ? "UP  " : "DOWN", deltaDeg, cnt,
                  (int)constrain(SERVO_CENTER + cnt, 0, 4095), SERVO_DIR[i]);
  }
  sendHornAngles(alpha);
}

// Moves ONE horn by deltaDeg (+ = tip up) with all others at home - checks the direction of each servo.
static void manualHorn(int idx, float deltaDeg) {
  float alpha[6];
  for (int i = 0; i < 6; i++) alpha[i] = alphaHome[i];
  alpha[idx] += deltaDeg * DEG2RAD;
  sendHornAngles(alpha);
  Serial.printf("servo %u horn %+.1f deg (should go %s), dir %+d\n", SERVO_IDS[idx], deltaDeg,
                deltaDeg >= 0 ? "UP" : "DOWN", SERVO_DIR[idx]);
}

// ============================================================================
//  IMU
// ============================================================================
// Quaternion straight from the BNO055 registers (QUA_Data_w_LSB = 0x20, 8 bytes, 2^14 LSB per unit),
// so a failed I2C transfer is detected. Returns false on any bus error or an implausible quaternion.
static bool readQuat(float &w, float &x, float &y, float &z) {
  Wire.beginTransmission(BNO_ADDR);
  Wire.write((uint8_t)0x20);
  if (Wire.endTransmission(true) != 0) return false;
  if (Wire.requestFrom((uint8_t)BNO_ADDR, (uint8_t)8, (uint8_t)1) != 8) return false;
  uint8_t b[8];
  for (int i = 0; i < 8; i++) b[i] = Wire.read();
  w = (int16_t)((b[1] << 8) | b[0]) / 16384.0f;
  x = (int16_t)((b[3] << 8) | b[2]) / 16384.0f;
  y = (int16_t)((b[5] << 8) | b[4]) / 16384.0f;
  z = (int16_t)((b[7] << 8) | b[6]) / 16384.0f;
  float n = sqrtf(w * w + x * x + y * y + z * z);
  return n > 0.9f && n < 1.1f;                      // the fused quaternion is a unit quaternion
}

static bool readIMU(Mat3 &C) {
  float w, x, y, z;
  if (!readQuat(w, x, y, z)) return false;
  C = quatToMat(w, x, y, z);
  return true;
}

static void configureI2C() {
  Wire.setClock(100000);      // the BNO055 stretches the clock, 400 kHz is unreliable on the ESP32
  Wire.setTimeOut(10);        // ms: a stuck transfer must not stall the control loop for long
}

// Clears a bus that a slave is holding low (9 clock pulses on SCL), then restarts the I2C driver.
static void recoverI2C() {
  Wire.end();
  pinMode(IMU_SDA_PIN, INPUT_PULLUP);
  pinMode(IMU_SCL_PIN, OUTPUT);
  for (int i = 0; i < 9; i++) {
    digitalWrite(IMU_SCL_PIN, LOW);  delayMicroseconds(5);
    digitalWrite(IMU_SCL_PIN, HIGH); delayMicroseconds(5);
  }
  Wire.begin(IMU_SDA_PIN, IMU_SCL_PIN);
  configureI2C();
  Serial.println("I2C restarted after repeated IMU read failures");
}

// Average a few quaternions, platform must be level and still.
static void tareIMU() {
  float w = 0, x = 0, y = 0, z = 0;
  int cnt = 0;
  for (int i = 0; i < 30; i++) {
    float qw, qx, qy, qz;
    if (readQuat(qw, qx, qy, qz)) {
      float sign = (cnt > 0 && (w * qw + x * qx + y * qy + z * qz) < 0) ? -1.0f : 1.0f;
      w += sign * qw; x += sign * qx; y += sign * qy; z += sign * qz;
      cnt++;
    }
    delay(10);
  }
  if (cnt == 0) { Serial.println("tare failed: no valid IMU reads, baseline unchanged"); return; }
  baselineR = quatToMat(w, x, y, z);
  ctlRoll = AxisCtl(); ctlPitch = AxisCtl();
  float r, p, yw;
  matToEulerDeg(baselineR, r, p, yw);
  Serial.printf("IMU tared (%d samples). baseline roll/pitch/yaw (deg): %.2f %.2f %.2f\n", cnt, r, p, yw);
}

// ============================================================================
//  PID (position form, as in the sim)
// ============================================================================
// First-order low-pass coefficient for a cutoff in Hz and a sample time dt (1 = no filtering).
static float lpfAlpha(float cutoffHz, float dt) {
  if (cutoffHz <= 0.0f) return 1.0f;
  float rc = 1.0f / (2.0f * (float)M_PI * cutoffHz);
  return dt / (rc + dt);
}

// One control step for one axis; the target tilt is 0. Returns the new tilt command [deg] of the top
// plate relative to the base:   cmd = Kp*err + iTerm - Kd*rate,   err = -measured tilt.
// (With a zero target, D on the error and D on the measurement are the same thing.)
// iTerm is stored in degrees, so changing Ki on the fly does not make the command jump.
// Anti-windup by back-calculation: whenever the output is cut (tilt limit or rate limit), iTerm is
// pulled back by the same amount, so the integrator never runs away from what was actually applied.
static float axisUpdate(AxisCtl &a, float meas, float dt) {
  if (!a.primed) {                                   // bumpless start: output begins at the current cmd
    a.m = a.mPrev = meas;
    a.rate = 0.0f;
    a.iTerm = clampf(a.cmd - Kp * (-a.m), -MAX_TILT_CMD_DEG, MAX_TILT_CMD_DEG);
    a.primed = true;
  }
  a.mPrev = a.m;
  a.m += lpfAlpha(MEAS_LPF_HZ, dt) * (meas - a.m);
  a.rate += lpfAlpha(D_LPF_HZ, dt) * ((a.m - a.mPrev) / dt - a.rate);

  float err = -a.m;
  a.iTerm = clampf(a.iTerm + Ki * err * dt, -MAX_TILT_CMD_DEG, MAX_TILT_CMD_DEG);
  float unsat = Kp * err + a.iTerm - Kd * a.rate;

  float maxStep = MAX_TILT_RATE_DPS * dt;
  float out = clampf(unsat, a.cmd - maxStep, a.cmd + maxStep);   // slew-rate limit
  out = clampf(out, -MAX_TILT_CMD_DEG, MAX_TILT_CMD_DEG);        // tilt limit

  a.iTerm = clampf(a.iTerm + (out - unsat), -MAX_TILT_CMD_DEG, MAX_TILT_CMD_DEG);   // back-calculation
  a.cmd = out;
  return out;
}

// Used when the mechanism cannot reach the command: shrink the command and keep the integrator consistent.
static void axisForceCmd(AxisCtl &a, float newCmd) {
  a.iTerm = clampf(a.iTerm + (newCmd - a.cmd), -MAX_TILT_CMD_DEG, MAX_TILT_CMD_DEG);
  a.cmd = newCmd;
}

// ============================================================================
//  Serial commands
// ============================================================================
static void handleLine(char *line) {
  float a = 0, b = 0;
  if (sscanf(line, "kp %f", &a) == 1)        { Kp = a; Serial.printf("Kp=%.3f\n", Kp); }
  else if (sscanf(line, "ki %f", &a) == 1)   { Ki = a; Serial.printf("Ki=%.3f\n", Ki); }
  else if (sscanf(line, "kd %f", &a) == 1)   { Kd = a; Serial.printf("Kd=%.3f\n", Kd); }
  else if (sscanf(line, "pid %f", &a) == 1)  {
    pidEnabled = (a != 0);
    ctlRoll = AxisCtl(); ctlPitch = AxisCtl();
    if (pidEnabled) { ctlRoll.cmd = manualRoll; ctlPitch.cmd = manualPitch; }   // start from the pose the platform is in
    else goToPose(manualRoll, manualPitch);
    Serial.printf("PID %s\n", pidEnabled ? "ON" : "OFF");
  }
  else if (sscanf(line, "manual %f %f", &a, &b) == 2 || sscanf(line, "pose %f %f", &a, &b) == 2) {
    pidEnabled = false;
    manualRoll = a; manualPitch = b;
    manualRollPitch(a, b);
  }
  else if (sscanf(line, "roll %f", &a) == 1) {          // roll only, keeps the last manual pitch
    pidEnabled = false;
    manualRoll = a;
    manualRollPitch(manualRoll, manualPitch);
  }
  else if (sscanf(line, "pitch %f", &a) == 1) {         // pitch only, keeps the last manual roll
    pidEnabled = false;
    manualPitch = a;
    manualRollPitch(manualRoll, manualPitch);
  }
  else if (sscanf(line, "horn %f %f", &a, &b) == 2 && a >= 1 && a <= 6) {
    pidEnabled = false;
    manualHorn((int)a - 1, b);
  }
  else if (sscanf(line, "dir %f %f", &a, &b) == 2 && a >= 1 && a <= 6 && b != 0) {
    SERVO_DIR[(int)a - 1] = (b > 0) ? +1 : -1;
    Serial.printf("servo %d dir=%+d\n", (int)a, SERVO_DIR[(int)a - 1]);
  }
  else if (strncmp(line, "home", 4) == 0) {
    pidEnabled = false; manualRoll = manualPitch = 0;
    goToPose(0, 0);
    Serial.println("home (PID off)");
  }
  else if (strncmp(line, "tare", 4) == 0) { tareIMU(); }
  else Serial.println("cmds: kp|ki|kd <v>, pid 1|0, manual|pose <roll> <pitch>, roll <deg>, pitch <deg>, horn <1-6> <deg>, dir <1-6> <+1|-1>, home, tare");
}

static void pollSerial() {
  static char buf[48];
  static uint8_t n = 0;
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (n > 0) { buf[n] = 0; handleLine(buf); n = 0; }
    } else if (n < sizeof(buf) - 1) {
      buf[n++] = c;
    }
  }
}

// ============================================================================
//  Arduino entry points
// ============================================================================
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\nStewart platform (rotary) starting...");

  // Servo bus
  Serial1.begin(SERVO_BAUD, SERIAL_8N1, SERVO_RX_PIN, SERVO_TX_PIN);
  st.pSerial = &Serial1;
  delay(300);

  // IMU (same order as the earlier working code: begin first, slow the bus down afterwards)
  Wire.begin(IMU_SDA_PIN, IMU_SCL_PIN);
  if (!bno.begin(OPERATION_MODE_IMUPLUS)) {   // gyro+accel only: no magnetometer trouble from servo motors
    Serial.println("BNO055 not found - check wiring / address");
    while (true) delay(1000);
  }
  delay(1000);
  configureI2C();

  // Geometry + home
  if (!initGeometry()) { while (true) delay(1000); }

  const float zero3[3] = {0, 0, 0};
  if (!solveIK(zero3, zero3, alphaHome)) {
    Serial.println("Home pose unreachable - check geometry/HORN_LEN/ROD_LEN");
    while (true) delay(1000);
  }
  Serial.printf("home height = %.4f m\n", homeZ);
  for (int i = 0; i < 6; i++)
    Serial.printf("leg %d: beta=%.1f deg, horn angle at home=%.1f deg\n",
                  i, beta[i] * RAD2DEG, alphaHome[i] * RAD2DEG);

  // How far can the platform actually tilt? (printed so MAX_TILT_CMD_DEG can be set sensibly)
  for (int dir = 0; dir < 360; dir += 90) {
    float lo = 0, hi = 45, al[6];
    for (int k = 0; k < 20; k++) {
      float mid = 0.5f * (lo + hi);
      if (poseReachable(mid * cosf(dir * DEG2RAD), mid * sinf(dir * DEG2RAD), al)) lo = mid; else hi = mid;
    }
    Serial.printf("max reachable tilt toward %3d deg (roll=0, pitch=90): %.1f deg\n", dir, lo);
  }

  // Torque on, go to home (all servos at centre count)
  for (int i = 0; i < 6; i++) st.EnableTorque(SERVO_IDS[i], 1);
  goToPose(0, 0);
  Serial.println("Moving to home, hold the platform level...");
  delay(2500);

  tareIMU();
  lastLoopUs = lastCtlUs = micros();
  Serial.println("Running. Type 'help' for commands.");
}

void loop() {
  pollSerial();

  uint32_t now = micros();
  if ((uint32_t)(now - lastLoopUs) < LOOP_PERIOD_US) return;
  lastLoopUs = now;

  // The IMU is read every cycle, also with the PID off, so the measured tilt can be checked
  // (e.g. "pose 5 0" -> measured roll should settle near +5; if not, the IMU axes/signs do not match).
  Mat3 C;
  if (!readIMU(C)) {                                  // bad read: keep the last servo command
    imuFailTotal++;
    if (++imuFailStreak >= IMU_RECOVER_AFTER_FAILS) { recoverI2C(); imuFailStreak = 0; }
    return;
  }
  imuFailStreak = 0;

  // real time step (longer after failed reads), clamped so one hiccup cannot cause a jump
  now = micros();
  float dt = clampf((now - lastCtlUs) * 1e-6f, 0.5f / LOOP_HZ, 3.0f / LOOP_HZ);
  lastCtlUs = now;

  // Orientation relative to baseline, as in the sim: R_rel = baseline^T * current
  Mat3 Rrel = matMul(transpose(baselineR), C);
  float rollDeg, pitchDeg, yawDeg;
  matToEulerDeg(Rrel, rollDeg, pitchDeg, yawDeg);

  if (!pidEnabled) {
    if (++loopCount % 10 == 0)
      Serial.printf("PID off | measured roll %.2f pitch %.2f | imu fails %lu\n",
                    rollDeg, pitchDeg, (unsigned long)imuFailTotal);
    return;
  }

  float rollCmd  = axisUpdate(ctlRoll,  rollDeg,  dt);
  float pitchCmd = axisUpdate(ctlPitch, pitchDeg, dt);

  // inverse kinematics -> horn angles. Never skipped: an unreachable command is scaled back.
  float alpha[6];
  float reach = solveSafe(rollCmd, pitchCmd, alpha);
  if (reach < 1.0f) {                                 // keep the controller state at what the platform can do
    rollCmd  *= reach;  pitchCmd *= reach;
    axisForceCmd(ctlRoll, rollCmd);
    axisForceCmd(ctlPitch, pitchCmd);
  }
  sendHornAngles(alpha);

  if (++loopCount % 10 == 0) {   // ~10 Hz telemetry
    Serial.printf("roll %.2f pitch %.2f | cmd %.2f %.2f | reach %.2f | imu fails %lu\n",
                  rollDeg, pitchDeg, rollCmd, pitchCmd, reach, (unsigned long)imuFailTotal);
  }
}