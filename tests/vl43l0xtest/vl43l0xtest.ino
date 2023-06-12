#include <Wire.h>
#include <VL53L0X.h>

#define NUM_VL53L0X 2
int dist_xshut_pins[NUM_VL53L0X] = {12, 11};
int dist_addresses[NUM_VL53L0X] = {0x8A, 0x8B};
VL53L0X dist_sensors[NUM_VL53L0X];

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

    Serial.println("3.14");
    digitalWrite(dist_xshut_pins[i], HIGH);
    delay(10);

    if (!dist_sensors[i].init()) exit(-1);

    dist_sensors[i].setAddress(dist_addresses[i]);
    dist_sensors[i].setMeasurementTimingBudget(20000);
    dist_sensors[i].startContinuous();
    dist_sensors[i].setTimeout(100);
    delay(10);
  }
}

void loop() {
  Serial.print(dist_sensors[0].readRangeContinuousMillimeters());
  Serial.print("  |  ");
  Serial.println(dist_sensors[1].readRangeContinuousMillimeters());
}
