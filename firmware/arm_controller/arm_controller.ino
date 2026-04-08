#include <Dynamixel2Arduino.h>

#define DXL_SERIAL Serial1
#define DIR_PIN    -1  // OpenRB-150: automatic

// ── Serial ports ─────────────────────────────────────────────
// Serial  = USB to Raspberry Pi   (sends <ARM:b,s,e,w>)
// Serial2 = UART to second Arduino (sends <HAND:wRot,grip>)
#define PI_SERIAL   Serial
#define HAND_SERIAL Serial2
#define HAND_BAUD   115200

// ── All 6 servos on one Dynamixel bus ────────────────────────
#define NUM_ARM   4
#define NUM_HAND  2
#define NUM_ALL   6

static const uint8_t ids[NUM_ALL] = { 1, 2, 3, 4, 5, 6 };
static const char *names[NUM_ALL] = { "base", "shldr", "elbow", "wrist", "wRot", "grip" };

// ── Calibration offsets ──────────────────────────────────────
static float offsets[NUM_ALL] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };

// ── Per-joint hard limits (degrees, AFTER offset) ────────────
static float lim_lo[NUM_ALL] = {   0.0,  48.0, 100.0,  75.0,   0.0,   0.0 };
static float lim_hi[NUM_ALL] = { 360.0, 190.0, 280.0, 280.0, 360.0, 360.0 };

// ── Motion profile ───────────────────────────────────────────
#define PROFILE_VEL  60
#define PROFILE_ACC  20

#define COLLISION_MARGIN 15.0

Dynamixel2Arduino dxl(DXL_SERIAL, DIR_PIN);
using namespace ControlTableItem;

static bool  alive[NUM_ALL];
static float lastCmd[NUM_ALL];
static bool  calMode = false;

// ── Per-port receive buffers ─────────────────────────────────
struct RxBuf {
  char  buf[128];
  int   idx;
  bool  active;
};
static RxBuf piRx   = { {}, 0, false };
static RxBuf handRx = { {}, 0, false };

// ──────────────────────────────────────────────────────────────
//  Collision check (arm joints only, user-space angles)
// ──────────────────────────────────────────────────────────────
bool poseIsSafe(float *a) {
  float shoulder = a[1] + offsets[1];
  float elbow    = a[2] + offsets[2];
  float wrist    = a[3] + offsets[3];

  if (shoulder > 160.0 && elbow > (360.0 - shoulder + COLLISION_MARGIN)) {
    PI_SERIAL.print("COLLISION: shldr="); PI_SERIAL.print(shoulder, 1);
    PI_SERIAL.print(" elb="); PI_SERIAL.println(elbow, 1);
    return false;
  }
  if ((elbow + wrist) > 420.0) {
    PI_SERIAL.print("COLLISION: elb+wrist="); PI_SERIAL.println(elbow + wrist, 1);
    return false;
  }
  if (shoulder > 180.0 && elbow > 200.0) {
    PI_SERIAL.print("COLLISION: full tuck shldr="); PI_SERIAL.print(shoulder, 1);
    PI_SERIAL.print(" elb="); PI_SERIAL.println(elbow, 1);
    return false;
  }
  return true;
}

// ── Servo helpers ────────────────────────────────────────────
bool initServo(int j) {
  if (!dxl.ping(ids[j])) return false;

  uint8_t hwErr = dxl.readControlTableItem(HARDWARE_ERROR_STATUS, ids[j]);
  if (hwErr != 0) {
    PI_SERIAL.print("ID "); PI_SERIAL.print(ids[j]);
    PI_SERIAL.print(" hw error 0x"); PI_SERIAL.print(hwErr, HEX);
    PI_SERIAL.println(" – rebooting");
    dxl.reboot(ids[j]);
    delay(500);
    if (!dxl.ping(ids[j])) return false;
  }

  dxl.torqueOff(ids[j]);
  dxl.setOperatingMode(ids[j], OP_POSITION);
  dxl.writeControlTableItem(PROFILE_VELOCITY,    ids[j], PROFILE_VEL);
  dxl.writeControlTableItem(PROFILE_ACCELERATION, ids[j], PROFILE_ACC);
  dxl.torqueOn(ids[j]);
  return true;
}

bool drive(int j, float deg) {
  if (!alive[j]) return false;
  float a = deg + offsets[j];
  if (a < lim_lo[j]) a = lim_lo[j];
  if (a > lim_hi[j]) a = lim_hi[j];
  lastCmd[j] = a;
  return dxl.setGoalPosition(ids[j], a, UNIT_DEGREE);
}

// ── Generic float parser ─────────────────────────────────────
bool parseFloats(const char *msg, float *out, int count) {
  char buf[128];
  strncpy(buf, msg, 127); buf[127] = '\0';
  char *t = strtok(buf, ",");
  for (int i = 0; i < count; i++) {
    if (!t) return false;
    char *end;
    float v = strtof(t, &end);
    if (end == t || isnan(v) || isinf(v)) return false;
    out[i] = v;
    t = strtok(NULL, ",");
  }
  return true;
}

// ── Calibration mode (all 6 joints) ─────────────────────────
void enterCalMode() {
  calMode = true;
  for (int j = 0; j < NUM_ALL; j++)
    if (alive[j]) dxl.torqueOff(ids[j]);
  PI_SERIAL.println("=== CAL MODE === torque OFF – move joints by hand");
  PI_SERIAL.println("  <ARM:SNAP>  labelled snapshot");
  PI_SERIAL.println("  <ARM:RUN>   exit cal mode");
}

void exitCalMode() {
  calMode = false;
  for (int j = 0; j < NUM_ALL; j++) {
    if (alive[j]) {
      float cur = dxl.getPresentPosition(ids[j], UNIT_DEGREE);
      lastCmd[j] = cur;
      dxl.torqueOn(ids[j]);
      dxl.setGoalPosition(ids[j], cur, UNIT_DEGREE);
    }
  }
  PI_SERIAL.println("=== RUN MODE === torque ON");
}

void printAngles(bool labelled) {
  for (int j = 0; j < NUM_ALL; j++) {
    if (!alive[j]) {
      if (labelled) { PI_SERIAL.print(names[j]); PI_SERIAL.print(":---"); }
      else PI_SERIAL.print("---");
    } else {
      float deg = dxl.getPresentPosition(ids[j], UNIT_DEGREE);
      if (labelled) { PI_SERIAL.print(names[j]); PI_SERIAL.print(":"); }
      PI_SERIAL.print(deg, 1);
    }
    if (j < NUM_ALL - 1) PI_SERIAL.print("  ");
  }
  PI_SERIAL.println();
}

void calLoop() {
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint < 200) return;
  lastPrint = millis();
  printAngles(false);
}

// ── Health check ─────────────────────────────────────────────
void healthCheck() {
  static unsigned long lastCheck = 0;
  if (millis() - lastCheck < 500) return;
  lastCheck = millis();

  for (int j = 0; j < NUM_ALL; j++) {
    if (!alive[j]) {
      alive[j] = initServo(j);
      if (alive[j]) {
        PI_SERIAL.print("ID "); PI_SERIAL.print(ids[j]); PI_SERIAL.println(" RECOVERED");
        dxl.setGoalPosition(ids[j], lastCmd[j], UNIT_DEGREE);
      }
      continue;
    }
    uint8_t torque = dxl.readControlTableItem(TORQUE_ENABLE, ids[j]);
    if (torque == 0) {
      PI_SERIAL.print("ID "); PI_SERIAL.print(ids[j]); PI_SERIAL.println(" torque lost – recovering");
      if (initServo(j))
        dxl.setGoalPosition(ids[j], lastCmd[j], UNIT_DEGREE);
      else
        alive[j] = false;
    }
  }
}

// ──────────────────────────────────────────────────────────────
//  Handle a complete <TAG:payload> message
// ──────────────────────────────────────────────────────────────
void handleMessage(const char *msg) {
  const char *colon = strchr(msg, ':');
  if (!colon) return;

  int tagLen = colon - msg;
  const char *payload = colon + 1;

  // ── ARM:  4 angles from Raspberry Pi (joints 0-3) ─────────
  if (tagLen == 3 && strncmp(msg, "ARM", 3) == 0) {

    if (strcmp(payload, "CAL")  == 0) { enterCalMode(); return; }
    if (strcmp(payload, "RUN")  == 0) { exitCalMode();  return; }
    if (strcmp(payload, "SNAP") == 0) { printAngles(true); return; }

    if (calMode) {
      PI_SERIAL.println("In CAL mode – send <ARM:RUN> first");
      return;
    }

    float angles[NUM_ARM];
    if (!parseFloats(payload, angles, NUM_ARM)) {
      PI_SERIAL.println("ARM PARSE ERROR");
      return;
    }

    if (!poseIsSafe(angles)) {
      PI_SERIAL.println("ARM REJECTED – collision risk");
      return;
    }

    PI_SERIAL.print("ARM CMD: ");
    for (int i = 0; i < NUM_ARM; i++) {
      PI_SERIAL.print(angles[i], 1);
      if (i < NUM_ARM - 1) PI_SERIAL.print(", ");
    }
    PI_SERIAL.println();

    for (int i = 0; i < NUM_ARM; i++) {
      if (!drive(i, angles[i]))
        PI_SERIAL.print("ID "), PI_SERIAL.print(ids[i]), PI_SERIAL.println(" FAILED");
    }
    return;
  }

  // ── HAND:  2 angles from second Arduino (joints 4-5) ──────
  if (tagLen == 4 && strncmp(msg, "HAND", 4) == 0) {

    if (calMode) return;

    float angles[NUM_HAND];
    if (!parseFloats(payload, angles, NUM_HAND)) {
      PI_SERIAL.println("HAND PARSE ERROR");
      return;
    }

    PI_SERIAL.print("HAND CMD: ");
    for (int i = 0; i < NUM_HAND; i++) {
      PI_SERIAL.print(angles[i], 1);
      if (i < NUM_HAND - 1) PI_SERIAL.print(", ");
    }
    PI_SERIAL.println();

    for (int i = 0; i < NUM_HAND; i++) {
      int j = NUM_ARM + i;   // maps to joint index 4, 5
      if (!drive(j, angles[i]))
        PI_SERIAL.print("ID "), PI_SERIAL.print(ids[j]), PI_SERIAL.println(" FAILED");
    }
    return;
  }
}

// ── Read from a serial port into its buffer ──────────────────
void readPort(Stream &port, RxBuf &rb) {
  while (port.available()) {
    char c = port.read();
    if (c == '<') { rb.idx = 0; rb.active = true; }
    else if (c == '>' && rb.active) {
      rb.buf[rb.idx] = '\0';
      rb.active = false;
      handleMessage(rb.buf);
    }
    else if (rb.active && rb.idx < 127) { rb.buf[rb.idx++] = c; }
  }
}

// ──────────────────────────────────────────────────────────────
void setup() {
  PI_SERIAL.begin(115200);
  while (!PI_SERIAL && millis() < 2000);

  HAND_SERIAL.begin(HAND_BAUD);

  dxl.begin(57600);
  dxl.setPortProtocolVersion(2.0);

  for (int j = 0; j < NUM_ALL; j++) {
    lastCmd[j] = (lim_lo[j] + lim_hi[j]) / 2.0;
    alive[j] = initServo(j);
    if (alive[j])
      dxl.setGoalPosition(ids[j], lastCmd[j], UNIT_DEGREE);
    PI_SERIAL.print("ID "); PI_SERIAL.print(ids[j]);
    PI_SERIAL.println(alive[j] ? " OK" : " MISSING");
  }

  PI_SERIAL.println("Ready");
  PI_SERIAL.println("  Pi  -> <ARM:base,shldr,elbow,wrist>");
  PI_SERIAL.println("  Aux -> <HAND:wRot,grip>");
  PI_SERIAL.println("  <ARM:CAL>  <ARM:RUN>  <ARM:SNAP>");
}

void loop() {
  readPort(PI_SERIAL,   piRx);     // USB from Raspberry Pi
  readPort(HAND_SERIAL, handRx);   // UART from second Arduino

  if (calMode) calLoop();
  else         healthCheck();
}