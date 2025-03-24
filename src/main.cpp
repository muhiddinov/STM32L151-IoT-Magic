#include <Arduino.h>

void setup() {
  pinMode(PB5, OUTPUT);
  pinMode(PB4, OUTPUT);
  digitalWrite(PB5, 1);
  digitalWrite(PB4, 1);
  Serial.begin(115200);
}

void loop() {
  digitalWrite(PB5, 0);
  digitalWrite(PB4, 0);
  Serial.println("Led on");
  delay(5000);
  digitalWrite(PB5, 1);
  digitalWrite(PB4, 1);
  Serial.println("Led off");
  delay(5000);
}