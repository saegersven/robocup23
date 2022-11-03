#include <Wire.h>

void setup() {
  Serial.begin(115200);
  Serial.println("Upload done");
  Wire.begin(0x2a);
  Wire.onReceive(onI2CReceive);
}

void onI2CReceive(int n) {
  while (Wire.available()) {
    Serial.print(Wire.read());
  }
}

void loop() {
}
