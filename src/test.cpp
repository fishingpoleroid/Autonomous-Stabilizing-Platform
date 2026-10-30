#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <SMS_STS.h>
#include <WiFi.h>

const char* ssid = "dkg";
const char* password = "c4di3qit";

WiFiServer telnetServer(23);
WiFiClient telnetClient;

#define BNO_ADDR 0x28
#define I2C_SDA  22
#define I2C_SCL  23
Adafruit_BNO055 bno(55, BNO_ADDR, &Wire);

#define S_RXD 18
#define S_TXD 19
SMS_STS st;

const int NUM_SERVOS = 6;
const uint8_t SERVO_IDS[NUM_SERVOS] = {1, 2, 3, 4, 5, 6};  

const float SHAFT_ANGLE_DEG[NUM_SERVOS] = {258.0f, 162.0f, 138.0f, 42.0f, 18.0f, 282.0f};

#define HOME_POS    2048
#define TEST_SPEED  1500
#define TEST_ACC    50
#define SWEEP_DELTA 300          

int  limitTicks = 400;           
bool imuOk = false;
bool imuStream = false;
float rollOffset = 0, pitchOffset = 0;

int legOfId(int id) {
  for (int i = 0; i < NUM_SERVOS; i++) if (SERVO_IDS[i] == id) return i;
  return -1;
}

int clampPos(int pos) { return constrain(pos, HOME_POS - limitTicks, HOME_POS + limitTicks); }

void moveServo(int id, int pos) {
  if (legOfId(id) < 0) { Serial.printf("Unknown servo ID %d\n", id); return; }
  int c = clampPos(pos);
  if (c != pos) Serial.printf("(clamped %d -> %d)\n", pos, c);
  st.WritePosEx(id, c, TEST_SPEED, TEST_ACC);
  Serial.printf("servo %d -> %d (%+d from home)\n", id, c, c - HOME_POS);
}

void moveAll(int pos) {
  for (int i = 0; i < NUM_SERVOS; i++) moveServo(SERVO_IDS[i], pos);
}

void pingAll() {
  for (int i = 0; i < NUM_SERVOS; i++) {
    int r = st.Ping(SERVO_IDS[i]);
    if (r == -1) Serial.printf("servo %d: NO REPLY\n", SERVO_IDS[i]);
    else         Serial.printf("servo %d: ok\n", r);
  }
}

void readAll() {
  for (int i = 0; i < NUM_SERVOS; i++) {
    int id = SERVO_IDS[i];
    int pos = st.ReadPos(id);
    if (pos == -1) { Serial.printf("servo %d: no reply\n", id); continue; }
    int volt = st.ReadVoltage(id);
    int temp = st.ReadTemper(id);
    Serial.printf("servo %d (leg %d): pos %d (%+d from home) | %.1f V | %d C\n",
                  id, i, pos, pos - HOME_POS, volt / 10.0f, temp);
  }
}

void sweepServo(int id) {
  int leg = legOfId(id);
  if (leg < 0) { Serial.printf("Unknown servo ID %d\n", id); return; }
  Serial.printf("--- servo %d = leg %d, expected shaft angle %.0f deg ---\n", id, leg, SHAFT_ANGLE_DEG[leg]);
  int d = min(SWEEP_DELTA, limitTicks);
  moveServo(id, HOME_POS + d); delay(900);
  Serial.printf("   reads %d\n", st.ReadPos(id));
  moveServo(id, HOME_POS - d); delay(900);
  Serial.printf("   reads %d\n", st.ReadPos(id));
  moveServo(id, HOME_POS);     delay(900);
}

void readRawTilt(float &roll, float &pitch, float &heading) {
  sensors_event_t ev;
  bno.getEvent(&ev);
  heading = ev.orientation.x;
  roll    = ev.orientation.y;
  pitch   = ev.orientation.z;
}

void averageTilt(float &roll, float &pitch, int n = 50) {
  float sr = 0, sp = 0, h;
  for (int i = 0; i < n; i++) {
    float r, p;
    readRawTilt(r, p, h);
    sr += r; sp += p;
    delay(10);
  }
  roll = sr / n; pitch = sp / n;
}

void printImu() {
  float r, p, h;
  readRawTilt(r, p, h);
  imu::Vector<3> g = bno.getVector(Adafruit_BNO055::VECTOR_GRAVITY);
  uint8_t sys, gyro, accel, mag;
  bno.getCalibration(&sys, &gyro, &accel, &mag);
  Serial.printf("heading %7.2f | roll %7.2f pitch %7.2f | rel roll %7.2f pitch %7.2f | "
                "gravity xyz %6.2f %6.2f %6.2f | cal S%d G%d A%d M%d\n",
                h, r, p, r - rollOffset, p - pitchOffset, g.x(), g.y(), g.z(), sys, gyro, accel, mag);
}

void zeroBaseline() {
  averageTilt(rollOffset, pitchOffset);
  Serial.printf("baseline set: roll %.2f pitch %.2f\n", rollOffset, pitchOffset);
}

void waitEnter() {
  while (Serial.available()) Serial.read();
  while (true) {
    if (Serial.available() && Serial.read() == '\n') break;
    delay(5);
  }
}

void tiltAfterPrompt(const char *msg, float &rollOut, float &pitchOut) {
  Serial.println(msg);
  Serial.println("   (press Enter in the serial monitor once the platform is held still)");
  waitEnter();
  averageTilt(rollOut, pitchOut);
}

void guidedAxisTest() {
  float r0, p0, r1, p1, rA, pA, rB, pB;
  Serial.println("\n=== Guided IMU axis test ===");
  Serial.println("Tilt the WHOLE platform (base and top plate together) by hand, about 15-20 deg.");
  Serial.println("+Y = side between the servos of legs 2 and 3 (default servo IDs 3 and 4).");
  Serial.println("+X = 90 deg clockwise from +Y, seen from above.");

  tiltAfterPrompt("\nStep 1: place the platform LEVEL.", r0, p0);
  tiltAfterPrompt("\nStep 2: tilt so the +Y side goes UP (the +Y edge rises, the -Y edge drops).", rA, pA);
  tiltAfterPrompt("\nStep 3: put it back LEVEL.", r1, p1);
  tiltAfterPrompt("\nStep 4: tilt so the +X side goes DOWN (the +X edge drops, the -X edge rises).", rB, pB);

  float dAr = rA - r0, dAp = pA - p0;   // tilt A: expect roll > 0
  float dBr = rB - r1, dBp = pB - p1;   // tilt B: expect pitch > 0
  Serial.printf("\nTilt A (+Y up)   : raw roll %+7.2f   raw pitch %+7.2f\n", dAr, dAp);
  Serial.printf("Tilt B (+X down) : raw roll %+7.2f   raw pitch %+7.2f\n", dBr, dBp);

  if (max(fabsf(dAr), fabsf(dAp)) < 5.0f || max(fabsf(dBr), fabsf(dBp)) < 5.0f) {
    Serial.println("\nRESULT: tilts too small to judge (<5 deg). Tilt more and repeat.");
    return;
  }

  bool normal  = fabsf(dAr) >= fabsf(dAp) && fabsf(dBp) >= fabsf(dBr);
  bool swapped = fabsf(dAp) >  fabsf(dAr) && fabsf(dBr) >  fabsf(dBp);

  if (normal) {
    float rs = dAr > 0 ? +1.0f : -1.0f;
    float ps = dBp > 0 ? +1.0f : -1.0f;
    Serial.println("\nRESULT: IMU roll/pitch axes line up with the platform axes.");
    Serial.println("Put these in main.cpp:");
    Serial.println("  const bool  SWAP_ROLL_PITCH = false;");
    Serial.printf ("  const float ROLL_SIGN  = %+.1ff;\n", rs);
    Serial.printf ("  const float PITCH_SIGN = %+.1ff;\n", ps);
    float cr = min(fabsf(dAp), fabsf(dBr)), mr = min(fabsf(dAr), fabsf(dBp));
    if (cr > 0.3f * mr) Serial.println("Note: noticeable cross-talk between roll and pitch, so the tilts may not have been clean, or the IMU is mounted at an angle. Repeat with cleaner tilts.");
  } else if (swapped) {
    float rs = dAp > 0 ? +1.0f : -1.0f;   // platform roll comes from IMU pitch
    float ps = dBr > 0 ? +1.0f : -1.0f;   // platform pitch comes from IMU roll
    Serial.println("\nRESULT: IMU axes are swapped (IMU is rotated ~90 deg about vertical vs the platform frame).");
    Serial.println("Put these in main.cpp:");
    Serial.println("  const bool  SWAP_ROLL_PITCH = true;");
    Serial.printf ("  const float ROLL_SIGN  = %+.1ff;\n", rs);
    Serial.printf ("  const float PITCH_SIGN = %+.1ff;\n", ps);
  } else {
    Serial.println("\nRESULT: ambiguous, both tilts moved both axes. Check how the IMU is mounted (flat? screwed down?),");
    Serial.println("and look at the 'imu' stream (gravity vector) with the platform level: gravity z should be ~ +9.8.");
  }
  Serial.println("Heading (yaw) is ignored by the controller, so it does not matter for stabilization.");
}

void printHelp() {
  Serial.println(
    "\nServos\n"
    "  ping            ping all servos\n"
    "  read            position / voltage / temperature of each servo\n"
    "  home            all servos to 2048\n"
    "  m <id> <pos>    move one servo to an absolute position (e.g. m 1 2300)\n"
    "  d <id> <delta>  move one servo to home + delta (e.g. d 3 -200)\n"
    "  all <delta>     all servos to home + delta\n"
    "  sweep <id>      home -> +300 -> -300 -> home for one servo\n"
    "  sweepall        sweep each servo in turn (watch which one moves vs the shaft angle printed)\n"
    "  limit <ticks>   change the clamp around home (now +-400)\n"
    "  off / on        torque off (move by hand) / on\n"
    "IMU\n"
    "  imu             toggle 10 Hz euler/gravity stream\n"
    "  imu1            print one reading\n"
    "  zero            set the 'level' reference (keep the platform level)\n"
    "  guide           guided axis/sign test\n");
}

void handleCommand(String line) {
  line.trim(); line.toLowerCase();
  if (line.length() == 0) return;
  int a, b;

  if (line == "help" || line == "?")      printHelp();
  else if (line == "ping")                pingAll();
  else if (line == "read")                readAll();
  else if (line == "home")                moveAll(HOME_POS);
  else if (sscanf(line.c_str(), "m %d %d", &a, &b) == 2)   moveServo(a, b);
  else if (sscanf(line.c_str(), "d %d %d", &a, &b) == 2)   moveServo(a, HOME_POS + b);
  else if (sscanf(line.c_str(), "all %d", &a) == 1)        moveAll(HOME_POS + a);
  else if (sscanf(line.c_str(), "sweep %d", &a) == 1)      sweepServo(a);
  else if (line == "sweepall")            { for (int i = 0; i < NUM_SERVOS; i++) sweepServo(SERVO_IDS[i]); }
  else if (sscanf(line.c_str(), "limit %d", &a) == 1)      { limitTicks = constrain(a, 0, 1000); Serial.printf("limit = +-%d ticks\n", limitTicks); }
  else if (line == "off")                 { for (int i = 0; i < NUM_SERVOS; i++) st.EnableTorque(SERVO_IDS[i], 0); Serial.println("torque OFF"); }
  else if (line == "on")                  { for (int i = 0; i < NUM_SERVOS; i++) st.EnableTorque(SERVO_IDS[i], 1); Serial.println("torque ON"); }
  else if (line == "imu")                 { imuStream = !imuStream; Serial.printf("imu stream %s\n", imuStream ? "ON" : "OFF"); }
  else if (line == "imu1")                { if (imuOk) printImu(); else Serial.println("IMU not available"); }
  else if (line == "zero")                { if (imuOk) zeroBaseline(); else Serial.println("IMU not available"); }
  else if (line == "guide")               { if (imuOk) guidedAxisTest(); else Serial.println("IMU not available"); }
  else Serial.println("unknown command, type 'help'");
}

void setup() {
  Serial.begin(115200);
  delay(500);

  WiFi.begin(ssid, password);

  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  
  Serial.println("\nWiFi connected.");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());

  telnetServer.begin();
  telnetServer.setNoDelay(true);
  Serial.println("Telnet server started on port 23");

  Wire.begin(I2C_SDA, I2C_SCL);
  if (!bno.begin(OPERATION_MODE_IMUPLUS)) {
    Serial.println("No BNO055 detected (check wiring and address 0x28). Servo tests still work.");
  } else {
    imuOk = true;
    delay(1000);
  }

  Serial1.begin(1000000, SERIAL_8N1, S_RXD, S_TXD);
  st.pSerial = &Serial1;
  delay(500);

  Serial.println("\nServo ID -> leg -> expected shaft angle (deg from +X, CCW, seen from above)");
  for (int i = 0; i < NUM_SERVOS; i++)
    Serial.printf("  servo %d -> leg %d -> %.0f deg\n", SERVO_IDS[i], i, SHAFT_ANGLE_DEG[i]);

  pingAll();
  Serial.println("Servos are NOT moved at boot. Type 'help'.");
}

void loop() {
  static uint32_t lastPrint = 0;

  if (Serial.available()) handleCommand(Serial.readStringUntil('\n'));

  if (telnetServer.hasClient()) {
    if (!telnetClient || !telnetClient.connected()) {
      if (telnetClient) telnetClient.stop();
      telnetClient = telnetServer.available();
      telnetClient.println("Welcome to ESP32 Telnet Server!");
      Serial.println("New Telnet client connected.");
    } else {
      telnetServer.available().stop();
    }
  }

  if (telnetClient && telnetClient.connected() && telnetClient.available()) {
    while (telnetClient.available()) {
      Serial.write(telnetClient.read());
    }
  }

  if (Serial.available() && telnetClient && telnetClient.connected()) {
    telnetClient.write(Serial.read());
  }

  if (imuStream && imuOk && millis() - lastPrint >= 100) {
    lastPrint = millis();
    printImu();
  }
}