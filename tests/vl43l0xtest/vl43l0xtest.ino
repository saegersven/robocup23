#include <Wire.h>
#include <VL53L0X.h>
#define NUM_VL53L0X 3
VL53L0X dist_sensors[NUM_VL53L0X];
int dist_xshut_pins[NUM_VL53L0X] = {12, 11, 10};
int dist_addresses[NUM_VL53L0X] = {0x8A, 0x8B, 0x8C};

void setup() {
  Serial.begin(115200);
  Wire.begin();
  
  // Configure distance sensors
  for (int i = 0; i < NUM_VL53L0X; ++i) {
    pinMode(dist_xshut_pins[i], OUTPUT);
    delay(10);
    digitalWrite(dist_xshut_pins[i], LOW);
  }
  delay(10);

  for (int i = 0; i < NUM_VL53L0X; ++i) {
    digitalWrite(dist_xshut_pins[i], HIGH);
    delay(10);

    if (!dist_sensors[i].init()) {
      Serial.print("Failed to initialize dist sensor ");
      Serial.println(i);
    }

    dist_sensors[i].setAddress(dist_addresses[i]);
    dist_sensors[i].setMeasurementTimingBudget(20000);
    dist_sensors[i].startContinuous();
    dist_sensors[i].setTimeout(200);
    delay(10);
  }
}

void loop() {
  long long starttime = millis();
  Serial.print(dist_sensors[0].readRangeContinuousMillimeters());
  Serial.print("  ");
  Serial.print(dist_sensors[1].readRangeContinuousMillimeters());
  Serial.print("  ");
  Serial.print(dist_sensors[2].readRangeContinuousMillimeters());
  long long endtime = millis();
  Serial.print("   | Took: ");
  Serial.print((int)(endtime - starttime));
  Serial.println("ms");
}
