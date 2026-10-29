// Rotary Stewart platform stabilizer (ESP32)
// Ported from the PyBullet simulation: same PID structure, same "tilt command -> IK -> actuators" flow.
// Everything marked <<< SET must match your physical build.

#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <SMS_STS.h>

// ------------------------------------------------------------------ hardware
#define BNO_ADDR 0x28
#define I2C_SDA  21
#define I2C_SCL  22
Adafruit_BNO055 bno(55, BNO_ADDR, &Wire);

#define S_RXD 18
#define S_TXD 19
SMS_STS st;

const int NUM_SERVOS = 6;
// Indexed by IK leg number (leg i = i-th entry of the sim's phi arrays, see the table printed at boot).
// Reorder so each ID is the servo that physically sits at that leg's shaft angle.   <<< SET
uint8_t SERVO_IDS[NUM_SERVOS] = {1, 2, 3, 4, 5, 6};

#define HOME_POS      2048
#define SERVO_SPEED   3073
#define SERVO_ACC     0        // 0 = no ramp, needed for a fast control loop (was 50 in the test code)
const float TICKS_PER_DEG = 4096.0f / 360.0f;
const int   MAX_DELTA_TICKS = 500;   // safety clamp around HOME_POS (~44 deg of horn travel) <<< SET

// +1 / -1 per servo: which way a positive tick change moves the horn UP. <<< SET
const int SERVO_DIR[NUM_SERVOS] = {+1, -1, +1, -1, +1, -1};

// ------------------------------------------------------------------ geometry (metres / degrees)
// Joint layout copied from inv_kinematics.py (same angles, same leg ordering).
// Radii and half-angle are the sim's values: confirm they match the real platform.
const float R_SERVO   = 0.200f;  // radius of the circle the servo shafts sit on   <<< verify
const float R_PLAT    = 0.200f;  // radius of the platform ball-joint circle        <<< verify
const float GAMMA_DEG = 12.0f;   // half the angle between the two joints of a pair <<< verify
const float HORN_LEN  = 0.025f;  // servo horn length (shaft to rod pivot)          <<< SET
const float ROD_LEN   = 0.120f;  // rod length (pivot to pivot)                     <<< SET

// inv_kinematics.py builds two angle sets, phi_B and phi_P. Because solve() does
// "self.B, self.P = self.frame()" while frame() returns (P, B), the leg vectors it computes
// use phi_P as the BASE joints and phi_B as the PLATFORM joints. true = replicate what the sim
// actually does; set false if your real servos sit on the phi_B set instead.
const bool SERVOS_ON_PHI_P = true;

// Filled in buildGeometry(): angle (deg from +X, CCW) of each leg's servo shaft / platform joint.
float BASE_ANGLE_DEG[6];
float PLAT_ANGLE_DEG[6];

// ------------------------------------------------------------------ controller (same as sim)
float Kp = 1.0f;     // sim used 15 / 3 / 13, start MUCH lower on hardware and tune via serial
float Ki = 0.0f;
float Kd = 0.0f;
const float INTEGRAL_LIMIT = 2.0f;
const float MAX_ANGLE      = 20.0f;    // sim used 45, reduce until you trust the setup  <<< SET

// IMU axis conventions: flip a sign if the platform tilts the wrong way.             <<< SET
const float ROLL_SIGN  = +1.0f;
const float PITCH_SIGN = +1.0f;

const uint32_t LOOP_HZ = 100;
const uint32_t LOOP_US = 1000000UL / LOOP_HZ;

// ------------------------------------------------------------------ state
float baseX[6], baseY[6];        // servo shaft positions (z = 0)
float platX[6], platY[6];        // platform joints in the platform frame
float hornDir[6];                // horn direction in the horizontal plane at HOME_POS (rad)
float homeHeight = 0.0f;         // platform height at home
float alphaHome[6];              // horn angle at home (rad)

float rollOffset = 0.0f, pitchOffset = 0.0f;
float prevErrRoll = 0, prevErrPitch = 0, sumErrRoll = 0, sumErrPitch = 0;
bool  enabled = true;
int16_t lastPos[6];

// ------------------------------------------------------------------ geometry / IK
static float angDiffDeg(float a, float b) { return fmodf(a - b + 540.0f, 360.0f) - 180.0f; }

// Rotation matrix with the same convention as the sim (R = Rx(roll) * Ry(pitch) * Rz(yaw)).
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

// Horn angle (rad) for leg i given the platform pose. Returns false if the pose is unreachable.
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
  const float phiB[6] = {210 + g, 210 - g, 90 + g, 90 - g, 330 + g, 330 - g};   // 7pi/6, pi/2, 11pi/6
  const float phiP[6] = {270 - g, 150 + g, 150 - g, 30 + g, 30 - g, 270 + g};   // 3pi/2, 5pi/6, pi/6
  for (int i = 0; i < 6; i++) {
    BASE_ANGLE_DEG[i] = SERVOS_ON_PHI_P ? phiP[i] : phiB[i];
    PLAT_ANGLE_DEG[i] = SERVOS_ON_PHI_P ? phiB[i] : phiP[i];
  }

  Serial.println("leg | servo ID | servo shaft angle | platform joint angle");
  for (int i = 0; i < 6; i++)
    Serial.printf(" %d  |    %d     |      %6.1f       |      %6.1f\n",
                  i, SERVO_IDS[i], BASE_ANGLE_DEG[i], PLAT_ANGLE_DEG[i]);

  for (int i = 0; i < 6; i++) {
    float b = radians(BASE_ANGLE_DEG[i]), p = radians(PLAT_ANGLE_DEG[i]);
    baseX[i] = R_SERVO * cosf(b); baseY[i] = R_SERVO * sinf(b);
    platX[i] = R_PLAT * cosf(p);  platY[i] = R_PLAT * sinf(p);
    // horn swings tangentially toward the platform joint it connects to  <<< check against your build
    float side = angDiffDeg(PLAT_ANGLE_DEG[i], BASE_ANGLE_DEG[i]) >= 0 ? 1.0f : -1.0f;
    hornDir[i] = radians(BASE_ANGLE_DEG[i] + 90.0f * side);
  }

  // Home height: horn horizontal at HOME_POS, rod length fixes the platform height.
  float hSum = 0, hMin = 1e9f, hMax = -1e9f;
  for (int i = 0; i < 6; i++) {
    float hx = baseX[i] + HORN_LEN * cosf(hornDir[i]);
    float hy = baseY[i] + HORN_LEN * sinf(hornDir[i]);
    float dx = platX[i] - hx, dy = platY[i] - hy;
    float h2 = ROD_LEN * ROD_LEN - dx * dx - dy * dy;
    float h = h2 > 0 ? sqrtf(h2) : 0;
    hSum += h; hMin = min(hMin, h); hMax = max(hMax, h);
  }
  homeHeight = hSum / 6.0f;
  Serial.printf("home height: %.4f m (spread between legs: %.2f mm, should be ~0)\n",
                homeHeight, (hMax - hMin) * 1000.0f);

  float R[3][3]; rotationMatrix(0, 0, 0, R);
  float T[3] = {0, 0, homeHeight};
  for (int i = 0; i < 6; i++) {
    if (!legAngle(i, R, T, alphaHome[i])) {
      Serial.printf("WARNING: home pose unreachable for leg %d, check geometry\n", i);
      alphaHome[i] = 0;
    }
  }
}

// Same role as ik_solver.solve() + (target - home) in the sim: returns servo ticks.
bool solveTicks(float rollDeg, float pitchDeg, int16_t outPos[6]) {
  float R[3][3]; rotationMatrix(rollDeg, pitchDeg, 0.0f, R);
  float T[3] = {0, 0, homeHeight};
  int16_t tmp[6];
  for (int i = 0; i < 6; i++) {
    float a;
    if (!legAngle(i, R, T, a)) return false;
    float deltaDeg = degrees(a - alphaHome[i]);
    int delta = (int)lroundf(SERVO_DIR[i] * deltaDeg * TICKS_PER_DEG);
    delta = constrain(delta, -MAX_DELTA_TICKS, MAX_DELTA_TICKS);
    tmp[i] = HOME_POS + delta;
  }
  memcpy(outPos, tmp, sizeof(tmp));
  return true;
}

void writeServos(const int16_t pos[6]) {
  uint16_t spd[6]; uint8_t acc[6];
  for (int i = 0; i < 6; i++) { spd[i] = SERVO_SPEED; acc[i] = SERVO_ACC; }
  st.SyncWritePosEx(SERVO_IDS, NUM_SERVOS, (int16_t *)pos, spd, acc);
}

void goHome() {
  for (int i = 0; i < 6; i++) lastPos[i] = HOME_POS;
  writeServos(lastPos);
}

// ------------------------------------------------------------------ IMU
bool readTilt(float &rollDeg, float &pitchDeg) {
  sensors_event_t ev;
  if (!bno.getEvent(&ev)) return false;
  // Adafruit: orientation.x = heading, .y = roll, .z = pitch (heading is irrelevant here)
  rollDeg  = ROLL_SIGN  * (ev.orientation.y - rollOffset);
  pitchDeg = PITCH_SIGN * (ev.orientation.z - pitchOffset);
  return true;
}

void captureBaseline() {
  // Equivalent of baseline_R in the sim. Keep the BASE LEVEL while this runs.
  float sr = 0, sp = 0; const int N = 100;
  for (int i = 0; i < N; i++) {
    sensors_event_t ev; bno.getEvent(&ev);
    sr += ev.orientation.y; sp += ev.orientation.z;
    delay(10);
  }
  rollOffset = sr / N; pitchOffset = sp / N;
  Serial.printf("baseline roll/pitch (deg): %.2f %.2f\n", rollOffset, pitchOffset);
}

// ------------------------------------------------------------------ serial tuning (replaces the sliders)
// kp 5 | ki 1 | kd 2 | z (re-zero baseline) | e (enable/disable) | r (reset integrators)
void handleSerial() {
  if (!Serial.available()) return;
  String s = Serial.readStringUntil('\n'); s.trim(); s.toLowerCase();
  if (s.startsWith("kp "))      Kp = s.substring(3).toFloat();
  else if (s.startsWith("ki ")) Ki = s.substring(3).toFloat();
  else if (s.startsWith("kd ")) Kd = s.substring(3).toFloat();
  else if (s == "z") { goHome(); delay(1000); captureBaseline(); }
  else if (s == "e") { enabled = !enabled; if (!enabled) goHome(); }
  else if (s == "r") { prevErrRoll = prevErrPitch = sumErrRoll = sumErrPitch = 0; }
  Serial.printf("Kp=%.2f Ki=%.2f Kd=%.2f enabled=%d\n", Kp, Ki, Kd, enabled);
}

// ------------------------------------------------------------------ setup / loop
void setup() {
  Serial.begin(115200);
  delay(500);

  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);
  if (!bno.begin(OPERATION_MODE_IMUPLUS)) {   // gyro+accel fusion, no magnetometer needed for roll/pitch
    Serial.println("No BNO055 detected.");
    while (1) delay(10);
  }
  delay(1000);

  Serial1.begin(1000000, SERIAL_8N1, S_RXD, S_TXD);
  st.pSerial = &Serial1;
  delay(500);

  for (int i = 0; i < NUM_SERVOS; i++) {
    if (st.Ping(SERVO_IDS[i]) == -1)
      Serial.printf("WARNING: servo ID %d did not answer ping.\n", SERVO_IDS[i]);
  }

  buildGeometry();
  goHome();
  delay(2000);            // let the platform settle (sim: 120 steps)
  captureBaseline();

  Serial.println("Ready. Commands: kp/ki/kd <val>, z, e, r");
}

void loop() {
  static uint32_t nextTick = micros();
  static uint32_t count = 0;

  handleSerial();

  uint32_t now = micros();
  if ((int32_t)(now - nextTick) < 0) return;
  nextTick += LOOP_US;
  if ((int32_t)(now - nextTick) > (int32_t)LOOP_US * 5) nextTick = now;   // recover after a stall

  if (!enabled) return;

  float roll, pitch;
  if (!readTilt(roll, pitch)) return;

  // ---- PID (identical form to the sim, per-sample, not time-scaled) ----
  float errRoll = 0.0f - roll;
  float errPitch = 0.0f - pitch;

  sumErrRoll  = constrain(sumErrRoll  + errRoll,  -INTEGRAL_LIMIT, INTEGRAL_LIMIT);
  sumErrPitch = constrain(sumErrPitch + errPitch, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);

  float rollCmd  = Kp * errRoll  + Ki * sumErrRoll  + Kd * (errRoll  - prevErrRoll);
  float pitchCmd = Kp * errPitch + Ki * sumErrPitch + Kd * (errPitch - prevErrPitch);

  rollCmd  = constrain(rollCmd,  -MAX_ANGLE, MAX_ANGLE);
  pitchCmd = constrain(pitchCmd, -MAX_ANGLE, MAX_ANGLE);

  prevErrRoll = errRoll; prevErrPitch = errPitch;

  // ---- IK -> servo ticks -> bus ----
  int16_t pos[6];
  if (solveTicks(rollCmd, pitchCmd, pos)) {
    memcpy(lastPos, pos, sizeof(pos));
  }                                    // unreachable pose: hold the last valid command
  writeServos(lastPos);

  if (++count % 10 == 0) {             // print at 10 Hz, not every loop
    Serial.printf("roll %6.2f pitch %6.2f | cmd %6.2f %6.2f | ticks %d %d %d %d %d %d\n",
                  roll, pitch, rollCmd, pitchCmd,
                  lastPos[0], lastPos[1], lastPos[2], lastPos[3], lastPos[4], lastPos[5]);
  }
}