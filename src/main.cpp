#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <stdarg.h>
#include <string.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <SMS_STS.h>

static const char*    WIFI_SSID = "dkg";
static const char*    WIFI_PASS = "c4di3qit";
static const uint16_t TELNET_PORT = 23;
static const uint32_t WIFI_CONNECT_TIMEOUT_MS = 20000;
static const bool     MIRROR_TO_SERIAL = true;

static const size_t OUT_BUF_SIZE     = 4096;   
static const size_t CMD_LEN          = 48;     
static const int    CMD_QUEUE_DEPTH  = 8;      

static const int  SERVO_RX_PIN = 18;
static const int  SERVO_TX_PIN = 19;
static const long SERVO_BAUD   = 1000000;
static const int  IMU_SDA_PIN  = 22;
static const int  IMU_SCL_PIN  = 23;
#define BNO_ADDR 0x28

static u8   SERVO_IDS[6] = {1, 2, 3, 4, 5, 6};
static int8_t SERVO_DIR[6] = {+1, -1, +1, -1, +1, -1};
static const int    SERVO_CENTER = 2048;
static const float  COUNTS_PER_DEG = 4096.0f / 360.0f;
static const float  SERVO_LIMIT_DEG = 90.0f;
static const u16    SERVO_SPEED = 3073;
static const u8     SERVO_ACC   = 0;

static const float R_BASE   = 0.110f;
static const float R_PLAT   = 0.120f;
static const float GAMMA_BASE_DEG = 30.0f;
static const float GAMMA_PLAT_DEG = 35.0f;
static const float HORN_LEN = 0.020f;
static const float ROD_LEN  = 0.160f;  

static const float HOME_Z_OVERRIDE = 0.0f;

static const bool  USE_MANUAL_BETA = true;
static const float BETA_MANUAL_DEG[6] = {120, 120, 0, 0, 240, 240};

static const float LOOP_HZ = 100.0f;
static float Kp = 0.5f, Ki = 3.0f, Kd = 0.025f;

static const float MAX_TILT_CMD_DEG = 40.0f;
static const float MAX_TILT_RATE_DPS = 50.0f;
static const float MEAS_LPF_HZ = 20.0f;
static const float D_LPF_HZ = 10.0f;

static const int IMU_RECOVER_AFTER_FAILS = 20;

static const float DEG2RAD = (float)M_PI / 180.0f;
static const float RAD2DEG = 180.0f / (float)M_PI;

SMS_STS st;
Adafruit_BNO055 bno = Adafruit_BNO055(55, BNO_ADDR, &Wire);

WiFiServer telnetServer(TELNET_PORT);
WiFiClient telnetClient;

struct Mat3 { float m[3][3]; };

static float bx[6], by[6];
static float px[6], py[6];
static float beta[6];
static float homeZ = 0.0f;
static float alphaHome[6];

static Mat3 baselineR = {{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}};

// servo Stats
static bool SHOW_SERVO = false;
static float SERVO_LOAD[6];
static float SERVO_TEMP[6];

static bool SHOW_IMU = true;
static bool SWAP_ROLL_PITCH = false;
static float MEAS_ROLL_SIGN  = -1.0f;
static float MEAS_PITCH_SIGN = 1.0f;

static bool pidEnabled = true;
static float manualRoll = 0, manualPitch = 0;

struct AxisCtl {
  float m = 0;       
  float mPrev = 0;
  float rate = 0;
  float ratePrev = 0;
  float cmd = 0;     
  bool  primed = false;
};
static AxisCtl ctlRoll, ctlPitch;

static uint32_t lastLoopUs = 0, lastCtlUs = 0;
static uint32_t loopCount  = 0;
static uint32_t imuFailTotal = 0;
static int      imuFailStreak = 0;
static const uint32_t LOOP_PERIOD_US = (uint32_t)(1000000.0f / LOOP_HZ);

static char    telnetLine[CMD_LEN];
static uint8_t telnetLineLen = 0;
static uint8_t telnetIacState = 0;

static char              outBuf[OUT_BUF_SIZE];
static size_t            outHead = 0, outTail = 0;     
static volatile uint32_t outDropped = 0;
static portMUX_TYPE      outMux = portMUX_INITIALIZER_UNLOCKED;
static QueueHandle_t     cmdQueue = nullptr;

static float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }

static void outPush(const char *d, size_t n) {
  portENTER_CRITICAL(&outMux);
  size_t used = (outHead + OUT_BUF_SIZE - outTail) % OUT_BUF_SIZE;
  if (n <= OUT_BUF_SIZE - 1 - used) {
    for (size_t i = 0; i < n; i++) {
      outBuf[outHead] = d[i];
      if (++outHead == OUT_BUF_SIZE) outHead = 0;
    }
  } else {
    outDropped++;
  }
  portEXIT_CRITICAL(&outMux);
}

static size_t outPop(char *dst, size_t maxN) {
  size_t n = 0;
  portENTER_CRITICAL(&outMux);
  while (n < maxN && outTail != outHead) {
    dst[n++] = outBuf[outTail];
    if (++outTail == OUT_BUF_SIZE) outTail = 0;
  }
  portEXIT_CRITICAL(&outMux);
  return n;
}

static void tprintf(const char *fmt, ...) {
  char buf[256];
  va_list ap;
  va_start(ap, fmt);
  int n = vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);
  if (n <= 0) return;
  if (n >= (int)sizeof(buf)) n = sizeof(buf) - 1;
  outPush(buf, (size_t)n);
}

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

static Mat3 buildR(float rollDeg, float pitchDeg, float yawDeg) {
  return matMul(matMul(rotX(rollDeg), rotY(pitchDeg)), rotZ(yawDeg));
}

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

static bool initGeometry() {
  const float gB = GAMMA_BASE_DEG, gP = GAMMA_PLAT_DEG;
  const float baseAng[6] = {360 - gB,   0 + gB, 120 - gB, 120 + gB, 240 - gB, 240 + gB};
  const float platAng[6] = {300 + gP,  60 - gP,  60 + gP, 180 - gP, 180 + gP, 300 - gP};

  float zSum = 0;
  for (int i = 0; i < 6; i++) {
    bx[i] = R_BASE * cosf(baseAng[i] * DEG2RAD);
    by[i] = R_BASE * sinf(baseAng[i] * DEG2RAD);
    px[i] = R_PLAT * cosf(platAng[i] * DEG2RAD);
    py[i] = R_PLAT * sinf(platAng[i] * DEG2RAD);

    float dx = px[i] - bx[i], dy = py[i] - by[i];
    beta[i] = USE_MANUAL_BETA ? BETA_MANUAL_DEG[i] * DEG2RAD : atan2f(dy, dx);

    float hx = dx - HORN_LEN * cosf(beta[i]);
    float hy = dy - HORN_LEN * sinf(beta[i]);
    float z2 = ROD_LEN * ROD_LEN - hx * hx - hy * hy;
    zSum += sqrtf(z2);
  }
  homeZ = (HOME_Z_OVERRIDE > 0.0f) ? HOME_Z_OVERRIDE : zSum / 6.0f;
  return true;
}

static bool solveIK(const float trans[3], const float rotDeg[3], float alpha[6]) {
  Mat3 R = buildR(rotDeg[0], rotDeg[1], rotDeg[2]);
  const float a = HORN_LEN, s = ROD_LEN;
  bool ok = true;

  for (int i = 0; i < 6; i++) {
    float qx = trans[0] + R.m[0][0] * px[i] + R.m[0][1] * py[i];
    float qy = trans[1] + R.m[1][0] * px[i] + R.m[1][1] * py[i];
    float qz = trans[2] + homeZ + R.m[2][0] * px[i] + R.m[2][1] * py[i];

    float lx = qx - bx[i], ly = qy - by[i], lz = qz;

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

static bool poseReachable(float rollDeg, float pitchDeg, float alpha[6]) {
  const float trans[3] = {0, 0, 0};
  const float rot[3]   = {rollDeg, pitchDeg, 0};
  if (!solveIK(trans, rot, alpha)) return false;
  for (int i = 0; i < 6; i++)
    if (fabsf(alpha[i] - alphaHome[i]) * RAD2DEG > SERVO_LIMIT_DEG) return false;
  return true;
}

static float solveSafe(float rollDeg, float pitchDeg, float alpha[6]) {
  if (poseReachable(rollDeg, pitchDeg, alpha)) return 1.0f;
  if (!poseReachable(0.0f, 0.0f, alpha)) {
    for (int i = 0; i < 6; i++) alpha[i] = alphaHome[i];
    return 0.0f;
  }
  float lo = 0.0f, hi = 1.0f;
  float tmp[6];
  for (int k = 0; k < 8; k++) {
    float mid = 0.5f * (lo + hi);
    if (poseReachable(rollDeg * mid, pitchDeg * mid, tmp)) { lo = mid; memcpy(alpha, tmp, sizeof(tmp)); }
    else hi = mid;
  }
  return lo;
}

static void setHorn(const float alpha[6]) {
  s16 pos[6];
  u16 spd[6];
  u8  acc[6];
  for (int i = 0; i < 6; i++) {
    float deltaDeg = (alpha[i] - alphaHome[i]) * RAD2DEG;
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
  if (scale < 1.0f) tprintf("pose out of reach - using %.0f %% of it\n", scale * 100.0f);
  setHorn(alpha);
}

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

static void manualRollPitch(float rollDeg, float pitchDeg) {
  float alpha[6];
  float scale = solveSafe(rollDeg, pitchDeg, alpha);
  float r = rollDeg * scale, p = pitchDeg * scale;
  if (scale < 1.0f) tprintf("Out of reach. Moving to (roll %.2f pitch %.2f)\n", scale * 100.0f, r, p);

  Mat3 R = buildR(r, p, 0);
  tprintf("manual roll=%.2f pitch=%.2f, r, p");
  for (int i = 0; i < 6; i++) {
    float plateDz = (R.m[2][0] * px[i] + R.m[2][1] * py[i]) * 1000.0f;       
    float deltaDeg = clampf((alpha[i] - alphaHome[i]) * RAD2DEG, -SERVO_LIMIT_DEG, SERVO_LIMIT_DEG);
    int cnt = SERVO_DIR[i] * (int)lroundf(deltaDeg * COUNTS_PER_DEG);
  }
  setHorn(alpha);
}

static void manualHorn(int idx, float deltaDeg) {
  float alpha[6];
  for (int i = 0; i < 6; i++) alpha[i] = alphaHome[i];
  alpha[idx] += deltaDeg * DEG2RAD;
  setHorn(alpha);
  tprintf("servo %u horn %+.1f deg (should go %s), dir %+d\n", SERVO_IDS[idx], deltaDeg,
          deltaDeg >= 0 ? "UP" : "DOWN", SERVO_DIR[idx]);
}

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
  return n > 0.9f && n < 1.1f;
}

static bool readIMU(Mat3 &C) {
  float w, x, y, z;
  if (!readQuat(w, x, y, z)) return false;
  C = quatToMat(w, x, y, z);
  return true;
}

static void configureI2C() {
  Wire.setClock(100000);
  Wire.setTimeOut(10);
}

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
  tprintf("I2C restarted after repeated IMU read failures\n");
}

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
  if (cnt == 0) { tprintf("tare failed: no valid IMU reads, baseline unchanged\n"); return; }
  baselineR = quatToMat(w, x, y, z);
  ctlRoll = AxisCtl(); ctlPitch = AxisCtl();
  float r, p, yw;
  matToEulerDeg(baselineR, r, p, yw);
  tprintf("IMU tared.");
}

static float lpfAlpha(float cutoffHz, float dt) {
  if (cutoffHz <= 0.0f) return 1.0f;
  float rc = 1.0f / (2.0f * (float)M_PI * cutoffHz);
  return dt / (rc + dt);
}

static float axisUpdate(AxisCtl &a, float meas, float dt) {
  if (!a.primed) {                                  
    a.m = a.mPrev = meas;
    a.rate = a.ratePrev = 0.0f;
    a.primed = true;
  }
  a.mPrev = a.m;
  a.ratePrev = a.rate;
  
  a.m += lpfAlpha(MEAS_LPF_HZ, dt) * (meas - a.m);
  a.rate += lpfAlpha(D_LPF_HZ, dt) * ((a.m - a.mPrev) / dt - a.rate);
  
  float dCmd = -Ki * a.m * dt - Kp * (a.m - a.mPrev) - Kd * (a.rate - a.ratePrev);
  float maxStep = MAX_TILT_RATE_DPS * dt;
  dCmd = clampf(dCmd, -maxStep, maxStep);                       
  a.cmd = clampf(a.cmd + dCmd, -MAX_TILT_CMD_DEG, MAX_TILT_CMD_DEG);   
  return a.cmd;
}

static float controlStep(float rollMeas, float pitchMeas, float dt, float alpha[6], float &rollCmd, float &pitchCmd) {
  rollCmd  = axisUpdate(ctlRoll,  rollMeas,  dt);
  pitchCmd = axisUpdate(ctlPitch, pitchMeas, dt);
  float reach = solveSafe(rollCmd, pitchCmd, alpha);
  if (reach < 1.0f) {
    rollCmd  *= reach;  pitchCmd *= reach;
    ctlRoll.cmd  = rollCmd;
    ctlPitch.cmd = pitchCmd;
  }
  return reach;
}

static void getServoStats() {
  for (int i = 0; i < 6; i++) {
    SERVO_LOAD[i] = st.ReadLoad(i+1);
    SERVO_TEMP[i] = st.ReadTemper(i+1);
  }
}

static void handleLine(char *line) {
  float a = 0, b = 0;
  if (sscanf(line, "kp %f", &a) == 1)        { Kp = a; tprintf("Kp=%.3f\n", Kp); }
  else if (sscanf(line, "ki %f", &a) == 1)   { Ki = a; tprintf("Ki=%.3f\n", Ki); }
  else if (sscanf(line, "kd %f", &a) == 1)   { Kd = a; tprintf("Kd=%.3f\n", Kd); }
  else if (sscanf(line, "pid %f", &a) == 1)  {
    pidEnabled = (a != 0);
    ctlRoll = AxisCtl(); ctlPitch = AxisCtl();
    if (pidEnabled) { ctlRoll.cmd = manualRoll; ctlPitch.cmd = manualPitch; }   
    else goToPose(manualRoll, manualPitch);
    tprintf("PID %s\n", pidEnabled ? "ON" : "OFF");
  }
  else if (strncmp(line, "swaprp", 4) == 0) {
    SWAP_ROLL_PITCH = !SWAP_ROLL_PITCH;
  }
  else if (strncmp(line, "fliproll", 8) == 0) {
    MEAS_ROLL_SIGN *= -1;
  }
  else if (strncmp(line, "flippitch", 9) == 0) {
    MEAS_PITCH_SIGN *= -1;
  }
  else if (sscanf(line, "manual %f %f", &a, &b) == 2 || sscanf(line, "pose %f %f", &a, &b) == 2) {
    pidEnabled = false;
    manualRoll = a; manualPitch = b;
    manualRollPitch(a, b);
  }
  else if (sscanf(line, "roll %f", &a) == 1) {          
    pidEnabled = false;
    manualRoll = a;
    manualRollPitch(manualRoll, manualPitch);
  }
  else if (sscanf(line, "pitch %f", &a) == 1) {         
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
    tprintf("servo %d dir=%+d\n", (int)a, SERVO_DIR[(int)a - 1]);
  }
  else if (strncmp(line, "home", 4) == 0) {
    pidEnabled = false; manualRoll = manualPitch = 0;
    goToPose(0, 0);
    tprintf("home (PID off)\n");
  }
  else if (strncmp(line, "tare", 4) == 0) { tareIMU(); }
  else if (strncmp(line, "imu", 3) == 0) { SHOW_IMU = !SHOW_IMU; }
  else if (strncmp(line, "servo", 5) == 0) { SHOW_SERVO = !SHOW_SERVO; }
  else tprintf("cmds: kp|ki <v>, pid 1|0, manual|pose <roll> <pitch>, roll <deg>, pitch <deg>, horn <1-6> <deg>, dir <1-6> <+1|-1>, home, tare\n");
}

static void pollCommands() {
  char line[CMD_LEN];
  if (xQueueReceive(cmdQueue, line, 0) == pdTRUE) {
    line[CMD_LEN - 1] = 0;
    handleLine(line);
  }
}

static void flushOutput() {
  char chunk[128];
  size_t n;
  while ((n = outPop(chunk, sizeof(chunk))) > 0) {
    if (MIRROR_TO_SERIAL) Serial.write((const uint8_t *)chunk, n);

    if (telnetClient && telnetClient.connected()) {
      char out[2 * sizeof(chunk)];
      size_t m = 0;
      for (size_t i = 0; i < n; i++) {
        if (chunk[i] == '\n') out[m++] = '\r';
        out[m++] = chunk[i];
      }
      if (telnetClient.write((const uint8_t *)out, m) < m) telnetClient.stop();   
    }
  }
}

static void acceptTelnetClient() {
  if (!telnetServer.hasClient()) return;
  WiFiClient nc = telnetServer.available();
  if (!nc) return;

  telnetClient.stop();                
  telnetClient = nc;
  telnetClient.setNoDelay(true);
  telnetLineLen = 0;
  telnetIacState = 0;
  tprintf("Telnet Connected.\n");
}

static void pollTelnetInput() {
  if (!telnetClient.connected()) { telnetClient.stop(); return; }

  while (telnetClient.available()) {
    int ci = telnetClient.read();
    if (ci < 0) break;
    uint8_t c = (uint8_t)ci;

    if (telnetIacState == 1) {                       
      if (c >= 251 && c <= 254) telnetIacState = 2;  
      else if (c == 250)        telnetIacState = 3;  
      else                      telnetIacState = 0;  
      continue;
    }
    if (telnetIacState == 2) { telnetIacState = 0; continue; }
    if (telnetIacState == 3) { if (c == 240) telnetIacState = 0; continue; }
    if (c == 255) { telnetIacState = 1; continue; }
    if (c == 0) continue;                            

    char ch = (char)c;
    if (ch == '\n' || ch == '\r') {
      if (telnetLineLen > 0) {
        telnetLine[telnetLineLen] = 0;
        if (xQueueSend(cmdQueue, telnetLine, 0) != pdTRUE) tprintf("command dropped: queue full\n");
        telnetLineLen = 0;
      }
    } else if (telnetLineLen < sizeof(telnetLine) - 1) {
      telnetLine[telnetLineLen++] = ch;
    }
  }
}

static void netTask(void *) {
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);       
  WiFi.setAutoReconnect(true);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  tprintf("Connecting to WiFi '%s' in the background...\n", WIFI_SSID);

  const uint32_t t0 = millis();
  bool serverStarted = false, timeoutWarned = false;
  uint32_t reportedDrops = 0;

  for (;;) {
    if (WiFi.status() == WL_CONNECTED && !serverStarted) {
      telnetServer.begin();
      telnetServer.setNoDelay(true);
      serverStarted = true;
      tprintf("WiFi connected. Telnet to %s port %u\n", WiFi.localIP().toString().c_str(), TELNET_PORT);
    } else if (!serverStarted && !timeoutWarned && millis() - t0 > WIFI_CONNECT_TIMEOUT_MS) {
      tprintf("WiFi not connected after %lu s\n",
              (unsigned long)(WIFI_CONNECT_TIMEOUT_MS / 1000));
      timeoutWarned = true;
    }

    if (serverStarted) {
      acceptTelnetClient();
      pollTelnetInput();
    }

    flushOutput();

    uint32_t d = outDropped;
    if (d != reportedDrops) {
      reportedDrops = d;
      tprintf("[%lu output messages dropped so far - output buffer was full]\n", (unsigned long)d);
    }

    vTaskDelay(pdMS_TO_TICKS(5));
  }
}

void setup() {
  Serial.begin(115200);
  delay(500);

  cmdQueue = xQueueCreate(CMD_QUEUE_DEPTH, CMD_LEN);
  xTaskCreatePinnedToCore(netTask, "net", 8192, nullptr, 1, nullptr, 0);

  Serial1.begin(SERVO_BAUD, SERIAL_8N1, SERVO_RX_PIN, SERVO_TX_PIN);
  st.pSerial = &Serial1;
  delay(300);

  Wire.begin(IMU_SDA_PIN, IMU_SCL_PIN);
  if (!bno.begin(OPERATION_MODE_IMUPLUS)) {   
    tprintf("BNO055 not found.\n");
    while (true) delay(1000);
  }
  delay(1000);
  configureI2C();

  if (!initGeometry()) { while (true) delay(1000); }

  const float zero3[3] = {0, 0, 0};
  if (!solveIK(zero3, zero3, alphaHome)) {
    tprintf("Home pose unreachable.\n");
    while (true) delay(1000);
  }
  tprintf("home height = %.4f m\n", homeZ);
  for (int i = 0; i < 6; i++)
    tprintf("leg %d: beta=%.1f deg, horn angle at home=%.1f deg\n",
            i, beta[i] * RAD2DEG, alphaHome[i] * RAD2DEG);

  for (int dir = 0; dir < 360; dir += 90) {
    float lo = 0, hi = 45, al[6];
    for (int k = 0; k < 20; k++) {
      float mid = 0.5f * (lo + hi);
      if (poseReachable(mid * cosf(dir * DEG2RAD), mid * sinf(dir * DEG2RAD), al)) lo = mid; else hi = mid;
    }
    tprintf("max reachable tilt, direction %3d deg (0 = +roll, 90 = +pitch): %.1f deg\n", dir, lo);
  }

  for (int i = 0; i < 6; i++) st.EnableTorque(SERVO_IDS[i], 1);
  goToPose(0, 0);
  tprintf("Moving to home.\n");
  delay(2500);

  lastLoopUs = lastCtlUs = micros();
}

void loop() {
  pollCommands();

  uint32_t now = micros();
  if ((uint32_t)(now - lastLoopUs) < LOOP_PERIOD_US) return;
  lastLoopUs = now;

  Mat3 C;
  getServoStats();
  if (!readIMU(C)) {                                  
    imuFailTotal++;
    if (++imuFailStreak >= IMU_RECOVER_AFTER_FAILS) { recoverI2C(); imuFailStreak = 0; }
    return;
  }
  imuFailStreak = 0;

  now = micros();
  float dt = clampf((now - lastCtlUs) * 1e-6f, 0.5f / LOOP_HZ, 3.0f / LOOP_HZ);
  lastCtlUs = now;

  Mat3 Rrel = matMul(transpose(baselineR), C);
  float rollDeg, pitchDeg, yawDeg;
  matToEulerDeg(Rrel, rollDeg, pitchDeg, yawDeg);

  if (SWAP_ROLL_PITCH) {
    float temp = rollDeg;
    rollDeg = pitchDeg;
    pitchDeg = temp;
  }

  rollDeg  *= MEAS_ROLL_SIGN;
  pitchDeg *= MEAS_PITCH_SIGN;


  if (!pidEnabled) {
    if (++loopCount % 10 == 0)
      tprintf("PID off | measured roll %.2f pitch %.2f | imu fails %lu\n",
              rollDeg, pitchDeg, (unsigned long)imuFailTotal);
    return;
  }

  float alpha[6], rollCmd, pitchCmd;
  float reach = controlStep(rollDeg, pitchDeg, dt, alpha, rollCmd, pitchCmd);
  setHorn(alpha);

  if (SHOW_IMU && ++loopCount % 10 == 0) {   
    tprintf("roll %.2f pitch %.2f | cmd %.2f %.2f | reach %.2f | imu fails %lu\n",
            rollDeg, pitchDeg, rollCmd, pitchCmd, reach, (unsigned long)imuFailTotal);
  }

  if (SHOW_SERVO && ++loopCount % 10 == 0) {   
    tprintf("Servo Loads:  %.2f  |  %.2f  |  %.2f  |  %.2f  |  %.2f    Servo Temperatures:  %.2f  |  %.2f  |  %.2f  |  %.2f  |  %.2f\n", 
      SERVO_LOAD[0], SERVO_LOAD[1], SERVO_LOAD[2], SERVO_LOAD[3], SERVO_LOAD[4], SERVO_LOAD[5],
      SERVO_TEMP[1], SERVO_TEMP[1], SERVO_TEMP[2], SERVO_TEMP[3], SERVO_TEMP[4], SERVO_TEMP[5]);
  }
}