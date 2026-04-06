#include <Dynamixel2Arduino.h>

#define DXL_SERIAL Serial1
#define DIR_PIN    -1  // OpenRB-150: automatic

#define NUM_JOINTS 3//5

static const uint8_t ids[NUM_JOINTS] = { 1, 2, 3, 4, 5 };

// ── Calibration offsets (degrees added before sending to servo) ──
// Measure these: command 0° to each joint, note how far off mechanical
// zero it is, put the correction here.
static float offsets[NUM_JOINTS] = { 0.0, 0.0, 0.0, 0.0, 0.0 };

// ── Per-joint hard limits (in degrees, AFTER offset applied) ────
// Joints: 0=base, 1=shoulder, 2=elbow, 3=wrist, 4=gripper
// Tune these with torque off: manually move each joint to its
// mechanical stops and read the angle.
static float lim_lo[NUM_JOINTS] = {   0.0,  48.0,  100.0,  75.0,  0.0 };
static float lim_hi[NUM_JOINTS] = { 360.0, 190.0, 280.0, 280.0, 360.0 };

// ── Motion profile (servos handle the smoothing internally) ─────
// PROFILE_VEL: 0 = max speed, units = 0.229 rev/min
// PROFILE_ACC: 0 = max accel, units = 214.577 rev/min²
// These prevent overload by limiting how aggressively the servo
// tries to reach the goal — the TARGET stays exact.
#define PROFILE_VEL  60
#define PROFILE_ACC  20

// ── Collision thresholds (tune to your geometry) ────────────────
#define COLLISION_MARGIN 15.0

Dynamixel2Arduino dxl(DXL_SERIAL, DIR_PIN);
using namespace ControlTableItem;

static char  rx[128];
static int   ri = 0;
static bool  active = false;
static bool  alive[NUM_JOINTS];
static float lastCmd[NUM_JOINTS];
static bool  calMode = false;          // calibration mode: torque off, stream angles

// ──────────────────────────────────────────────────────────────
//  Inter-joint collision check
//  Returns true if the requested pose is SAFE.
//  All angles here are in USER space (before offsets).
// ──────────────────────────────────────────────────────────────
bool poseIsSafe(float *a) {
  float shoulder = a[1] + offsets[1];
  float elbow    = a[2] + offsets[2];
  float wrist    = a[3] + offsets[3];

  // Rule 1: shoulder raised + elbow folded back = forearm hits base
  if (shoulder > 160.0 && elbow > (360.0 - shoulder + COLLISION_MARGIN)) {
    Serial.print("COLLISION: shldr="); Serial.print(shoulder, 1);
    Serial.print(" elb="); Serial.println(elbow, 1);
    return false;
  }

  // Rule 2: elbow + wrist fold = gripper hits forearm
  if ((elbow + wrist) > 420.0) {
    Serial.print("COLLISION: elb+wrist="); Serial.println(elbow + wrist, 1);
    return false;
  }

  // Rule 3: full tuck
  if (shoulder > 180.0 && elbow > 200.0) {
    Serial.print("COLLISION: full tuck shldr="); Serial.print(shoulder, 1);
    Serial.print(" elb="); Serial.println(elbow, 1);
    return false;
  }

  return true;
}

// ── Initialise one servo ─────────────────────────────────────
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
  dxl.writeControlTableItem(PROFILE_VELOCITY,     ids[j], PROFILE_VEL);
  dxl.writeControlTableItem(PROFILE_ACCELERATION,  ids[j], PROFILE_ACC);
  dxl.torqueOn(ids[j]);
  return true;
}

// ── Drive one joint to an ABSOLUTE angle ─────────────────────
// The servo's profile velocity/acceleration smooths the motion.
// The target is sent exactly as requested (after offset + clamp).
bool drive(int j, float deg) {
  if (!alive[j]) return false;
  float a = deg + offsets[j];
  if (a < lim_lo[j]) a = lim_lo[j];
  if (a > lim_hi[j]) a = lim_hi[j];
  lastCmd[j] = a;
  return dxl.setGoalPosition(ids[j], a, UNIT_DEGREE);
}

// ── Parse "<v,v,v,v,v>" payload ──────────────────────────────
bool parse(const char *msg, float *angles) {
  char buf[128];
  strncpy(buf, msg, 127); buf[127] = '\0';
  char *t = strtok(buf, ",");
  for (int i = 0; i < NUM_JOINTS; i++) {
    if (!t) return false;
    char *end;
    float v = strtof(t, &end);
    if (end == t)              return false;   // no digits
    if (isnan(v) || isinf(v)) return false;   // garbage
    angles[i] = v;
    t = strtok(NULL, ",");
  }
  return true;
}

// ── Calibration mode ─────────────────────────────────────────
//    Send <CAL> to enter: torque off, angles printed every 200ms
//    Send <RUN> to exit:  torque back on, normal operation
//    Send <SNAP> while in CAL to print one labelled snapshot

void enterCalMode() {
  calMode = true;
  for (int j = 0; j < NUM_JOINTS; j++) {
    if (alive[j]) dxl.torqueOff(ids[j]);
  }
  Serial.println("=== CAL MODE === torque OFF – move joints by hand");
  Serial.println("  <SNAP>  print a labelled snapshot");
  Serial.println("  <RUN>   exit cal mode");
}

void exitCalMode() {
  calMode = false;
  for (int j = 0; j < NUM_JOINTS; j++) {
    if (alive[j]) {
      // Read wherever the user left each joint and adopt that as goal
      float cur = dxl.getPresentPosition(ids[j], UNIT_DEGREE);
      lastCmd[j] = cur;
      dxl.torqueOn(ids[j]);
      dxl.setGoalPosition(ids[j], cur, UNIT_DEGREE);
    }
  }
  Serial.println("=== RUN MODE === torque ON");
}

void printAngles(bool labelled) {
  const char *names[NUM_JOINTS] = { "base", "shldr", "elbow", "wrist", "grip" };
  for (int j = 0; j < NUM_JOINTS; j++) {
    if (!alive[j]) {
      if (labelled) { Serial.print(names[j]); Serial.print(":---"); }
      else Serial.print("---");
    } else {
      float deg = dxl.getPresentPosition(ids[j], UNIT_DEGREE);
      if (labelled) { Serial.print(names[j]); Serial.print(":"); }
      Serial.print(deg, 1);
    }
    if (j < NUM_JOINTS - 1) Serial.print("  ");
  }
  Serial.println();
}

void calLoop() {
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint < 200) return;
  lastPrint = millis();
  printAngles(false);
}

// ── Health check (runs every 500 ms) ─────────────────────────
void healthCheck() {
  static unsigned long lastCheck = 0;
  if (millis() - lastCheck < 500) return;
  lastCheck = millis();

  for (int j = 0; j < NUM_JOINTS; j++) {
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
      if (initServo(j)) {
        dxl.setGoalPosition(ids[j], lastCmd[j], UNIT_DEGREE);
      } else {
        alive[j] = false;
      }
    }
  }
}

// ──────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 2000);
  dxl.begin(57600);
  dxl.setPortProtocolVersion(2.0);

  for (int j = 0; j < NUM_JOINTS; j++) {
    lastCmd[j] = (lim_lo[j] + lim_hi[j]) / 2.0;
    alive[j] = initServo(j);
    if (alive[j]) {
      dxl.setGoalPosition(ids[j], lastCmd[j], UNIT_DEGREE);
    }
    Serial.print("ID "); Serial.print(ids[j]);
    Serial.println(alive[j] ? " OK" : " MISSING");
  }
  Serial.println("Ready - send <base,shoulder,elbow,wrist,gripper>");
  Serial.println("  <CAL> = calibration mode (torque off, read angles)");
}

void loop() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '<') { ri = 0; active = true; }
    else if (c == '>' && active) {
      rx[ri] = '\0'; active = false;

      // ── Special commands ──
      if (strcmp(rx, "CAL") == 0)  { enterCalMode(); continue; }
      if (strcmp(rx, "RUN") == 0)  { exitCalMode();  continue; }
      if (strcmp(rx, "SNAP") == 0) { printAngles(true); continue; }

      // ── Ignore motion commands in cal mode ──
      if (calMode) {
        Serial.println("In CAL mode – send <RUN> first");
        continue;
      }

      float angles[NUM_JOINTS];

      if (!parse(rx, angles)) {
        Serial.println("PARSE ERROR");
        continue;
      }

      // Reject unsafe poses — do NOT modify the angles
      if (!poseIsSafe(angles)) {
        Serial.println("REJECTED – collision risk");
        continue;
      }

      // Send exact IK targets to servos
      Serial.print("CMD: ");
      for (int i = 0; i < NUM_JOINTS; i++) {
        Serial.print(angles[i], 1);
        if (i < NUM_JOINTS - 1) Serial.print(", ");
      }
      Serial.println();

      for (int i = 0; i < NUM_JOINTS; i++) {
        if (!drive(i, angles[i]))
          Serial.print("ID "), Serial.print(ids[i]), Serial.println(" FAILED");
      }
    }
    else if (active && ri < 127) { rx[ri++] = c; }
  }

  // ── Mode-dependent background tasks ──
  if (calMode) {
    calLoop();         // stream angles every 200ms
  } else {
    healthCheck();     // keep servos alive
  }
}