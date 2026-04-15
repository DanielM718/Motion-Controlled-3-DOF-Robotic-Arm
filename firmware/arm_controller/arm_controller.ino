#include <Dynamixel2Arduino.h>

#define DXL_SERIAL Serial1
#define DIR_PIN    -1  // OpenRB-150: automatic

// ── Joint mapping ────────────────────────────────────────────
//  Index:  0      1       2      3       4       5
//  Name:   base   shldr   elbow  xWrist  yWrist  grip
//
//  <ARM:base,shldr,elbow>     → joints 0-2
//  <HAND:xWrist,yWrist>       → joints 3-4
//  <GRIP:grip>                → joint  5
//  <ARM:CAL> <ARM:RUN> <ARM:SNAP>

#define NUM_ARM   3
#define NUM_HAND  2
#define NUM_GRIP  1
#define NUM_ALL   6

#define ARM_START   0
#define HAND_START  3
#define GRIP_START  5

static const uint8_t ids[NUM_ALL] = { 1, 2, 3, 4, 5, 6 };
static const char *names[NUM_ALL] = { "base", "shldr", "elbow", "xWrist", "yWrist", "grip" };

// ── Calibration offsets ──────────────────────────────────────
static float offsets[NUM_ALL] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };

// ── Per-joint hard limits (degrees, AFTER offset) ────────────
static float lim_lo[NUM_ALL] = {   80.0,  52.0, 100.0,   155.0,   0.0,   86 };
static float lim_hi[NUM_ALL] = { 260.0, 150.0, 288.0, 360.0, 360.0, 150 };

// ── Motion profile ───────────────────────────────────────────
#define PROFILE_VEL  80
#define PROFILE_ACC  45

#define COLLISION_MARGIN 15.0

Dynamixel2Arduino dxl(DXL_SERIAL, DIR_PIN);
using namespace ControlTableItem;

static bool  alive[NUM_ALL];
static float lastCmd[NUM_ALL];
static bool  calMode = false;

static char  rx[128];
static int   ri = 0;
static bool  rxActive = false;

enum PARTS {
  BASE,
  SHOULDER,
  ELBOW,
  XWRIST,
  YWRIST,
  GRIP
};

// ──────────────────────────────────────────────────────────────
//  Collision check (arm joints, user-space angles)
// ──────────────────────────────────────────────────────────────
bool poseIsSafe(float *a) {
  float shoulder = a[1] + offsets[1];
  float elbow    = a[2] + offsets[2];

  if (shoulder > 160.0 && elbow > (360.0 - shoulder + COLLISION_MARGIN)) {
    Serial.print("COLLISION: shldr="); Serial.print(shoulder, 1);
    Serial.print(" elb="); Serial.println(elbow, 1);
    return false;
  }
  if (shoulder > 180.0 && elbow > 200.0) {
    Serial.print("COLLISION: full tuck shldr="); Serial.print(shoulder, 1);
    Serial.print(" elb="); Serial.println(elbow, 1);
    return false;
  }
  return true; 
}


//Joy stick graba
void gripper_control(float pressed, float* out) {
    out[0] = (pressed == 0) ? lim_hi[GRIP] : lim_lo[GRIP];
}

void wrist_control(float x, float y, float* out) {
    out[0] = dxl.getPresentPosition(ids[XWRIST], UNIT_DEGREE);
    out[1] = dxl.getPresentPosition(ids[YWRIST], UNIT_DEGREE);
    if (x > 900) out[0] -= 15.0;
    if (x < 100) out[0] += 15.0;
    if (y > 900) out[1] += 15.0;
    if (y < 100) out[1] -= 15.0;
}

// ── Servo helpers ────────────────────────────────────────────
bool initServo(int j) {
  if (!dxl.ping(ids[j])) return false;

  uint8_t hwErr = dxl.readControlTableItem(HARDWARE_ERROR_STATUS, ids[j]);
  if (hwErr != 0) {
    Serial.print("ID "); Serial.print(ids[j]);
    Serial.print(" hw error 0x"); Serial.print(hwErr, HEX);
    Serial.println(" – rebooting");
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
  Serial.println("=== CAL MODE === torque OFF – move joints by hand");
  Serial.println("  <ARM:SNAP>  labelled snapshot");
  Serial.println("  <ARM:RUN>   exit cal mode");
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
  Serial.println("=== RUN MODE === torque ON");
}

void printAngles(bool labelled) {
  for (int j = 0; j < NUM_ALL; j++) {
    if (!alive[j]) {
      if (labelled) { Serial.print(names[j]); Serial.print(":---"); }
      else Serial.print("---");
    } else {
      float deg = dxl.getPresentPosition(ids[j], UNIT_DEGREE);
      if (labelled) { Serial.print(names[j]); Serial.print(":"); }
      Serial.print(deg, 1);
    }
    if (j < NUM_ALL - 1) Serial.print("  ");
  }
  Serial.println();
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
        Serial.print("ID "); Serial.print(ids[j]); Serial.println(" RECOVERED");
        dxl.setGoalPosition(ids[j], lastCmd[j], UNIT_DEGREE);
      }
      continue;
    }
    uint8_t torque = dxl.readControlTableItem(TORQUE_ENABLE, ids[j]);
    if (torque == 0) {
      Serial.print("ID "); Serial.print(ids[j]); Serial.println(" torque lost – recovering");
      if (initServo(j))
        dxl.setGoalPosition(ids[j], lastCmd[j], UNIT_DEGREE);
      else
        alive[j] = false;
    }
  }
}

// ── Drive a group of joints ──────────────────────────────────
void driveGroup(const char *label, float *angles, int count, int jointStart) {
  Serial.print(label); Serial.print(" CMD: ");
  for (int i = 0; i < count; i++) {
    Serial.print(angles[i], 1);
    if (i < count - 1) Serial.print(", ");
  }
  Serial.println();

  for (int i = 0; i < count; i++) {
    int j = jointStart + i;
    if (!drive(j, angles[i]))
      Serial.print("ID "), Serial.print(ids[j]), Serial.println(" FAILED");
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

  // ── ARM: 3 angles (base, shoulder, elbow) ──────────────────
  if (tagLen == 3 && strncmp(msg, "ARM", 3) == 0) {

    if (strcmp(payload, "CAL")  == 0) { enterCalMode(); return; }
    if (strcmp(payload, "RUN")  == 0) { exitCalMode();  return; }
    if (strcmp(payload, "SNAP") == 0) { printAngles(true); return; }

    if (calMode) {
      Serial.println("In CAL mode – send <ARM:RUN> first");
      return;
    }

    float angles[NUM_ARM];
    if (!parseFloats(payload, angles, NUM_ARM)) {
      Serial.println("ARM PARSE ERROR");
      return;
    }

    if (!poseIsSafe(angles)) {
      Serial.println("ARM REJECTED – collision risk");
      return;
    }

    driveGroup("ARM", angles, NUM_ARM, ARM_START);
    return;
  }

  // ── HAND: 2 angles (xWrist, yWrist) ───────────────────────
  if (tagLen == 4 && strncmp(msg, "HAND", 4) == 0) {

    if (calMode) return;

    float angles[NUM_HAND];
    if (!parseFloats(payload, angles, NUM_HAND)) {
      Serial.println("HAND PARSE ERROR");
      return;
    }
    wrist_control(angles[0], angles[1], angles);
    driveGroup("HAND", angles, NUM_HAND, HAND_START);
    return;
  }

  // ── GRIP: 1 angle (gripper) ────────────────────────────────
  if (tagLen == 4 && strncmp(msg, "GRIP", 4) == 0) {

    if (calMode) return;

    float angles[NUM_GRIP];
    if (!parseFloats(payload, angles, NUM_GRIP)) {
      Serial.println("GRIP PARSE ERROR");
      return;
    }
    gripper_control(angles[0], angles);
    driveGroup("GRIP", angles, NUM_GRIP, GRIP_START);
    return;
  }
}

// ──────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 2000);

  dxl.begin(57600);
  dxl.setPortProtocolVersion(2.0);

  for (int j = 0; j < NUM_ALL; j++) {
    lastCmd[j] = (lim_lo[j] + lim_hi[j]) / 2.0;
    alive[j] = initServo(j);
    if (alive[j])
      dxl.setGoalPosition(ids[j], lastCmd[j], UNIT_DEGREE);
    Serial.print("ID "); Serial.print(ids[j]);
    Serial.println(alive[j] ? " OK" : " MISSING");
  }

  Serial.println("Ready");
  Serial.println("  <ARM:base,shldr,elbow>       joints 1-3");
  Serial.println("  <HAND:xWrist,yWrist>         joints 4-5");
  Serial.println("  <GRIP:grip>                  joint  6");
  Serial.println("  <ARM:CAL>  <ARM:RUN>  <ARM:SNAP>");
}

void loop() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '<') { ri = 0; rxActive = true; }
    else if (c == '>' && rxActive) {
      rx[ri] = '\0';
      rxActive = false;
      handleMessage(rx);
    }
    else if (rxActive && ri < 127) { rx[ri++] = c; }
  }

  if (calMode) calLoop();
  else         healthCheck();
}