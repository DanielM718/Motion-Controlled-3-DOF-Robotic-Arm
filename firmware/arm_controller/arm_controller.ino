#include <Dynamixel2Arduino.h>

#define DXL_SERIAL Serial1
#define DIR_PIN    -1  // OpenRB-150: automatic

#define NUM_JOINTS 5

static const uint8_t ids[NUM_JOINTS] = { 1, 2, 3, 4, 5 };

static float offsets[NUM_JOINTS] = { 0.0, 0.0, 0.0, 0.0, 0.0 };
static float lim_lo[NUM_JOINTS]  = {   0.0,  30.0,  30.0,   0.0,   0.0 };
static float lim_hi[NUM_JOINTS]  = { 360.0, 210.0, 210.0, 360.0, 360.0 };

#define PROFILE_VEL 100

Dynamixel2Arduino dxl(DXL_SERIAL, DIR_PIN);
using namespace ControlTableItem;

static char  rx[128];
static int   ri = 0;
static bool  active = false;
static bool  alive[NUM_JOINTS];          // true if servo answered ping
static float lastCmd[NUM_JOINTS];        // last commanded angle per joint

// ── Try to (re-)initialise one servo ──────────────────────────
bool initServo(int j) {
  if (!dxl.ping(ids[j])) return false;

  dxl.torqueOff(ids[j]);

  // Clear any hardware error (overload / overheat) so torque can re-enable
  dxl.writeControlTableItem(HARDWARE_ERROR_STATUS, ids[j], 0);

  dxl.setOperatingMode(ids[j], OP_POSITION);
  dxl.writeControlTableItem(PROFILE_VELOCITY, ids[j], PROFILE_VEL);
  dxl.torqueOn(ids[j]);
  return true;
}

// ── Drive one joint ──────────────────────────────────────────
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
    angles[i] = atof(t);
    t = strtok(NULL, ",");
  }
  return true;
}

// ── Periodic health check ────────────────────────────────────
//    Re-pings missing servos, clears errors, re-sends last goal
void healthCheck() {
  static unsigned long lastCheck = 0;
  if (millis() - lastCheck < 500) return;   // run every 500 ms
  lastCheck = millis();

  for (int j = 0; j < NUM_JOINTS; j++) {
    // Retry any servo that was missing at boot
    if (!alive[j]) {
      alive[j] = initServo(j);
      if (alive[j]) {
        Serial.print("ID "); Serial.print(ids[j]); Serial.println(" RECOVERED");
        dxl.setGoalPosition(ids[j], lastCmd[j], UNIT_DEGREE);
      }
      continue;
    }

    // Check if torque is still on (it drops on hardware error)
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
    lastCmd[j] = 180.0;                 // safe default (mid-range)
    alive[j] = initServo(j);
    Serial.print("ID "); Serial.print(ids[j]);
    Serial.println(alive[j] ? " OK" : " MISSING");
  }
  Serial.println("Ready - send <base,shoulder,elbow,wrist,gripper>");
}

void loop() {
  // ── Serial command parser ──
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '<') { ri = 0; active = true; }
    else if (c == '>' && active) {
      rx[ri] = '\0'; active = false;
      float angles[NUM_JOINTS];
      if (parse(rx, angles)) {
        for (int i = 0; i < NUM_JOINTS; i++) {
          if (!drive(i, angles[i]))
            Serial.print("ID "), Serial.print(ids[i]), Serial.println(" FAILED");
        }
      } else {
        Serial.println("PARSE ERROR");
      }
    }
    else if (active && ri < 127) { rx[ri++] = c; }
  }

  // ── Keep servos alive ──
  healthCheck();
}