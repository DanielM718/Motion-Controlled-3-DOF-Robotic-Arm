#include <Arduino.h>

const int VRX_PIN = A1;
const int VRY_PIN = A2;
const int SEL_PIN = 7;

void setup() {
  Serial.begin(9600);
  pinMode(SEL_PIN, INPUT_PULLUP); // PULLUP so unpressed = HIGH, pressed = LOW
}

void loop() {
  int xVal = analogRead(VRX_PIN);  // 0–1023
  int yVal = analogRead(VRY_PIN);  // 0–1023
  bool pressed = !digitalRead(SEL_PIN); // LOW when pressed, so we flip it

  Serial.print("Rotation:");
  Serial.print(xVal + ','); // X coordinate
  Serial.print(yVal + ','); // Y coordinate
  Serial.println(pressed ? "PRESSED" : "NOT_PRESSED");
}

// Right & Up: 516 < X <= 1023 , 516 < Y <= 1023
// Right & Down: 516 < X <= 1023 , 0 <= Y <= 516
// Left & Down: 0 <= X <= 516, 0 <= Y <= 516
// Left & Up: 0 <= X <= 516, 516 < Y <= 1023
