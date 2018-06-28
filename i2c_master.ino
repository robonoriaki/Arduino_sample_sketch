#include <Wire.h>

void setup() {
  Wire.begin();
  Serial.begin(9600);
}

void loop() {
  Wire.requestFrom(8, 1);// request 1 bytes from Slave ID #8

  while (Wire.available()) {
    byte b = Wire.read();
    Serial.println(b);
  }

  delay(500);
}
