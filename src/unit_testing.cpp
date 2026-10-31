// Single-servo PID unit test (ESP32)
//
// Same pipeline as main.cpp (IMU -> PID -> tilt command -> IK -> servo ticks), but only the servo you
// select by ID receives its target. The others hold their home pose (or have torque off).
// Roll and pitch can each be switched on/off. "horn <deg>" drives the selected servo to a fixed
// horn angle instead, to verify its direction and travel limits without the IMU.
//
// Serial monitor: 115200 baud, line ending = Newline. Type "help".
// Hardware, geometry and servo settings live in stewart_common.h (shared with main.cpp).

#include "stewart_common.h"

// ------------------------------------------------------------------ controller (same as main.cpp)
float Kp = 1.0f;
float Ki = 0.0f;
float Kd = 0.0f;
const float INTEGRAL_LIMIT = 2.0f;
float MAX_ANGLE = 20.0f;

const uint32_t LOOP_HZ = 100;
const uint32_t LOOP_US = 1000000UL / LOOP_HZ;

// ------------------------------------------------------------------ test configuration (serial-controlled)
int   selectedId    = 0;      // the one servo that gets the output (set to SERVO_IDS[0] in setup)
bool  useRoll       = true;
bool  usePitch      = true;
bool  othersFree    = false;  // false: others hold home. true: others have torque OFF
int   limitTicks    = 300;    // PID target clamp: home tick +- limitTicks for the selected servo
bool  manualHorn    = false;  // true: ignore the PID, hold manualHornDeg
float manualHornDeg = 0.0f;
bool  showTelemetry = true;

// ------------------------------------------------------------------ state
float prevErrRoll = 0, prevErrPitch = 0, sumErrRoll = 0, sumErrPitch = 0;
bool  enabled = true;
uint32_t imuFails = 0, lateMaxUs = 0;
uint8_t  consecFails = 0;

void resetPid() { prevErrRoll = prevErrPitch = sumErrRoll = sumErrPitch = 0; }

void selectedHome() {
  st.WritePosEx(SERVO_IDS[selectedId], homeTick(legOfId(SERVO_IDS[selectedId])), SERVO_SPEED, SERVO_ACC);
}

void applyOthersMode() {
  for (int i = 0; i < NUM_SERVOS; i++) {
    int id = SERVO_IDS[i];
    if (id == selectedId) { st.EnableTorque(id, 1); continue; }
    if (othersFree) st.EnableTorque(id, 0);
    else { st.EnableTorque(id, 1); st.WritePosEx(id, homeTick(i), SERVO_SPEED, SERVO_ACC); }
  }
}

void printStatus() {
  Serial.printf("servo ID %d (leg %d) | roll %s | pitch %s | %s | others %s | lim +-%d ticks | "
                "Kp=%.2f Ki=%.2f Kd=%.2f | MAX_ANGLE=%.1f | horn limits +%.0f/-%.0f | %s\n",
                SERVO_IDS[selectedId], legOfId(SERVO_IDS[selectedId]), useRoll ? "ON" : "off", usePitch ? "ON" : "off",
                manualHorn ? "MANUAL horn" : "PID", othersFree ? "FREE (torque off)" : "hold home", limitTicks,
                Kp, Ki, Kd, MAX_ANGLE, HORN_MAX_UP_DEG, HORN_MAX_DOWN_DEG, enabled ? "RUNNING" : "STOPPED");
}

void printHelp() {
  Serial.println(
    "\n  id <n>          select the servo that gets the output (e.g. id 5)\n"
    "  roll [0|1]      roll error into the PID on/off (no argument = toggle)\n"
    "  pitch [0|1]     pitch error into the PID on/off (no argument = toggle)\n"
    "  kp|ki|kd <v>    PID gains\n"
    "  horn <deg>      MANUAL: hold the selected horn at <deg> above horizontal (e.g. horn 20, horn -15)\n"
    "  horn off        back to the PID\n"
    "  lim <ticks>     clamp the PID target to home +- ticks (default 300)\n"
    "  others hold|free   other servos hold home / have torque off\n"
    "  mup|mdn <deg>   horn travel limits up / down (default 85 / 20)\n"
    "  mxa <deg>       PID output clamp\n"
    "  z               re-zero the IMU baseline (keep the platform level)\n"
    "  r               reset PID integrators\n"
    "  e               start/stop (stop returns the selected servo home)\n"
    "  show            toggle telemetry\n"
    "  status          print the configuration\n");
}

void processCommand(String s) {
  s.trim(); s.toLowerCase();
  if (s.length() == 0) return;
  int a;
  float f;

  if (s == "help" || s == "?") { printHelp(); }
  else if (sscanf(s.c_str(), "id %d", &a) == 1) {
    if (legOfId(a) < 0) { Serial.printf("unknown servo ID %d\n", a); return; }
    selectedHome();
    selectedId = a-1;
    manualHorn = false;
    resetPid();
    applyOthersMode();
  }
  else if (s == "roll")  { useRoll = !useRoll; resetPid(); }
  else if (sscanf(s.c_str(), "roll %d", &a) == 1)  { useRoll = (a != 0); resetPid(); }
  else if (s == "pitch") { usePitch = !usePitch; resetPid(); }
  else if (sscanf(s.c_str(), "pitch %d", &a) == 1) { usePitch = (a != 0); resetPid(); }
  else if (s == "horn off") { manualHorn = false; resetPid(); }
  else if (sscanf(s.c_str(), "horn %f", &f) == 1) { manualHornDeg = f; manualHorn = true; }
  else if (sscanf(s.c_str(), "kp %f", &f) == 1) Kp = f;
  else if (sscanf(s.c_str(), "ki %f", &f) == 1) Ki = f;
  else if (sscanf(s.c_str(), "kd %f", &f) == 1) Kd = f;
  else if (sscanf(s.c_str(), "lim %d", &a) == 1) limitTicks = constrain(a, 0, 1100);
  else if (sscanf(s.c_str(), "mup %f", &f) == 1) HORN_MAX_UP_DEG = f;
  else if (sscanf(s.c_str(), "mdn %f", &f) == 1) HORN_MAX_DOWN_DEG = f;
  else if (sscanf(s.c_str(), "mxa %f", &f) == 1) MAX_ANGLE = f;
  else if (s == "others hold") { othersFree = false; applyOthersMode(); }
  else if (s == "others free") { othersFree = true;  applyOthersMode(); }
  else if (s == "z")    { selectedHome(); delay(1000); captureBaseline(); resetPid(); }
  else if (s == "r")    { resetPid(); }
  else if (s == "e")    { enabled = !enabled; if (!enabled) selectedHome(); resetPid(); }
  else if (s == "show") { showTelemetry = !showTelemetry; }
  else if (s != "status") { Serial.println("unknown command, type 'help'"); }
  printStatus();
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

  selectedId = SERVO_IDS[0];
  for (int i = 0; i < NUM_SERVOS; i++) st.WritePosEx(SERVO_IDS[i], homeTick(i), SERVO_SPEED, SERVO_ACC);
  delay(2000);
  applyOthersMode();
  captureBaseline();

  Serial.println("Single-servo PID test ready. Type 'help'.");
  printStatus();
}

void loop() {
  static uint32_t nextTick = micros();
  static uint32_t count = 0;
 

  handleSerial();

  uint32_t now = micros();
  int32_t late = (int32_t)(now - nextTick);
  if (late < 0) return;
  if ((uint32_t)late > lateMaxUs) lateMaxUs = late;
  nextTick += LOOP_US;
  if ((int32_t)(now - nextTick) > (int32_t)LOOP_US * 5) nextTick = now;

  if (!enabled) return;

  float roll, pitch;
  if (!readTilt(roll, pitch)) {
    imuFails++;
    if (++consecFails >= 5) { recoverI2C(); consecFails = 0; }
    return;
  }
  consecFails = 0;

  int leg = legOfId(SERVO_IDS[selectedId]);
  float rollCmd = 0, pitchCmd = 0, scale = 1.0f;
  int target;

  if (manualHorn) {
    float deg = constrain(manualHornDeg, -HORN_MAX_DOWN_DEG, HORN_MAX_UP_DEG);
    target = tickForHornDeg(leg, deg);
  } else {
    // An axis that is switched off contributes zero error (and no integral / derivative history).
    float errRoll  = useRoll  ? (0.0f - roll)  : 0.0f;
    float errPitch = usePitch ? (0.0f - pitch) : 0.0f;
    if (!useRoll)  { sumErrRoll = 0;  prevErrRoll = 0; }
    if (!usePitch) { sumErrPitch = 0; prevErrPitch = 0; }

    float newSumRoll  = constrain(sumErrRoll  + errRoll,  -INTEGRAL_LIMIT, INTEGRAL_LIMIT);
    float newSumPitch = constrain(sumErrPitch + errPitch, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);

    rollCmd  = Kp * errRoll  + Ki * newSumRoll  + Kd * (errRoll  - prevErrRoll);
    pitchCmd = Kp * errPitch + Ki * newSumPitch + Kd * (errPitch - prevErrPitch);
    rollCmd  = constrain(rollCmd,  -MAX_ANGLE, MAX_ANGLE);
    pitchCmd = constrain(pitchCmd, -MAX_ANGLE, MAX_ANGLE);
    prevErrRoll = errRoll; prevErrPitch = errPitch;

    // Full IK for the commanded tilt, but only the selected leg's target is sent.
    int16_t pos[6];
    scale = solveTicksScaled(rollCmd, pitchCmd, pos);
    if (scale >= 1.0f) { sumErrRoll = newSumRoll; sumErrPitch = newSumPitch; }

    int h = homeTick(leg);
    target = constrain((int)pos[leg], h - limitTicks, h + limitTicks);
  }

  st.WritePosEx(SERVO_IDS[selectedId], target, SERVO_SPEED, SERVO_ACC);

  if (++count % 10 == 0) {
    if (showTelemetry) {
      int actual = st.ReadPos(SERVO_IDS[selectedId]);
      Serial.printf("ID %d (leg %d) %s | meas roll %6.2f%s pitch %6.2f%s | cmd %6.2f %6.2f | scale %.2f | "
                    "target %d (%+d from home) actual %d | imuFail %lu lateMax %lu us\n",
                    SERVO_IDS[selectedId], leg, manualHorn ? "MAN" : "PID", roll, useRoll ? "*" : " ", pitch, usePitch ? "*" : " ",
                    rollCmd, pitchCmd, scale, target, target - homeTick(leg), actual,
                    (unsigned long)imuFails, (unsigned long)lateMaxUs);
    }
    lateMaxUs = 0;
  }
}