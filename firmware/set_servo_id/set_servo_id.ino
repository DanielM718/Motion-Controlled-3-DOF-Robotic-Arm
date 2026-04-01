#include <Dynamixel2Arduino.h>

#define DXL_SERIAL Serial1
#define DIR_PIN    -1

// Change FROM_ID if the servo isn't at factory default.
// Assign IDs one servo at a time:
//   Base     -> 1
//   Shoulder -> 2
//   Elbow    -> 3
//   Wrist    -> 4
//   Gripper  -> 5

#define FROM_ID 1

Dynamixel2Arduino dxl(DXL_SERIAL, DIR_PIN);

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 2000);
  dxl.begin(57600);
  dxl.setPortProtocolVersion(2.0);

  Serial.println("--- Dynamixel ID Tool ---");
  Serial.print("Ping ID ");
  Serial.print(FROM_ID);
  Serial.println(dxl.ping(FROM_ID) ? " -> FOUND" : " -> NOT FOUND");
  Serial.println("Type new ID (1-252):");
}

void loop() {
  if (!Serial.available()) return;
  int id = Serial.readStringUntil('\n').toInt();
  if (id < 1 || id > 252) { Serial.println("Bad ID"); return; }

  dxl.torqueOff(FROM_ID);
  delay(20);
  dxl.setID(FROM_ID, id);
  delay(100);

  if (dxl.ping(id)) {
    Serial.print("OK -> ID ");
    Serial.println(id);
    for (int i = 0; i < 3; i++) {
      dxl.ledOn(id); delay(150); dxl.ledOff(id); delay(150);
    }
    Serial.println("Disconnect this servo, connect the next, reset board.");
  } else {
    Serial.println("FAILED - check wiring/power");
  }
}