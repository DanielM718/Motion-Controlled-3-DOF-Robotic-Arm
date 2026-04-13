#include <Dynamixel2Arduino.h>

#define DXL_SERIAL Serial1
#define DIR_PIN    -1

Dynamixel2Arduino dxl(DXL_SERIAL, DIR_PIN);

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 2000);

  Serial.println("--- Dynamixel ID Scanner ---");
  Serial.println("Scanning all IDs (1-252) at common baud rates...\n");

  // Try each common baud rate the servos might be set to
  long baudRates[] = {100000, 57600};
  int numBauds = 4;

  for (int b = 0; b < numBauds; b++) {
    dxl.begin(baudRates[b]);
    dxl.setPortProtocolVersion(2.0);

    Serial.print("Baud rate: ");
    Serial.println(baudRates[b]);

    bool found = false;
    for (int id = 1; id <= 252; id++) {
      if (dxl.ping(id)) {
        found = true;
        Serial.print("  FOUND servo at ID ");
        Serial.print(id);
        Serial.print("  | Model: ");
        Serial.print(dxl.getModelNumber(id));
        Serial.println();

        // Blink LED so you can physically identify which servo this is
        for (int i = 0; i < id; i++) {
          dxl.ledOn(id);  delay(200);
          dxl.ledOff(id); delay(200);
        }
      }
    }

    if (!found) {
      Serial.println("  No servos found at this baud rate.");
    }
    Serial.println();
  }

  Serial.println("--- Scan complete ---");
  Serial.println("Each servo blinked a number of times equal to its ID.");
  Serial.println("Type a new ID assignment as: OLD_ID NEW_ID  (e.g. '1 3')");

  dxl.begin(57600);
  dxl.setPortProtocolVersion(2.0);
}

void loop() {
  if (!Serial.available()) return;

  String input = Serial.readStringUntil('\n');
  input.trim();

  int spaceIdx = input.indexOf(' ');
  if (spaceIdx == -1) {
    Serial.println("Format: OLD_ID NEW_ID  (e.g. '1 3')");
    return;
  }

  int oldId = input.substring(0, spaceIdx).toInt();
  int newId = input.substring(spaceIdx + 1).toInt();

  if (oldId < 1 || oldId > 252 || newId < 1 || newId > 252) {
    Serial.println("IDs must be 1-252");
    return;
  }

  dxl.torqueOff(oldId);
  delay(20);
  dxl.setID(oldId, newId);
  delay(100);

  if (dxl.ping(newId)) {
    Serial.print("OK: ID ");
    Serial.print(oldId);
    Serial.print(" -> ");
    Serial.println(newId);
    for (int i = 0; i < 3; i++) {
      dxl.ledOn(newId); delay(150); dxl.ledOff(newId); delay(150);
    }
  } else {
    Serial.println("FAILED - check wiring/power");
  }
}