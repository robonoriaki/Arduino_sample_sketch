#include <Wire.h>

byte b=0;

void setup() {
  Wire.begin(8);// Slave ID #8
  Wire.onRequest(requestEvent);
}

void loop() {
}

void requestEvent() {
  Wire.write(b++);
}
