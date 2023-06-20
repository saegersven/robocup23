#include <VL53L0X.h>
#include <Wire.h>
#include <Servo.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <EEPROM.h> // only used for debugging


#include "defines.h"

char message[5];
unsigned int message_pos = 0;
bool led_on = false; // global variable to store current debug led state. Led state can be toggled via RasPi

Servo servos[NUM_SERVOS];

VL53L0X dist_sensors[NUM_VL53L0X];
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

void setup() {
  Serial.begin(115200);
  Wire.begin();

  // Configure pins
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(PIN_BTN, INPUT);

  pinMode(M_LEFT_A, OUTPUT);
  pinMode(M_LEFT_B, OUTPUT);
  pinMode(M_LEFT_FRONT_EN, OUTPUT);
  pinMode(M_LEFT_REAR_EN, OUTPUT);

  pinMode(M_RIGHT_A, OUTPUT);
  pinMode(M_RIGHT_B, OUTPUT);
  pinMode(M_RIGHT_FRONT_EN, OUTPUT);
  pinMode(M_RIGHT_REAR_EN, OUTPUT);

  pinMode(M_GRIPPER_A, OUTPUT);
  pinMode(M_GRIPPER_B, OUTPUT);

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

    if (!dist_sensors[i].init()) error(3); // freezes here

    dist_sensors[i].setAddress(dist_addresses[i]);
    dist_sensors[i].setMeasurementTimingBudget(20000);
    dist_sensors[i].startContinuous();
    dist_sensors[i].setTimeout(100);
    delay(10);
  }

  if (!bno.begin()) error(2);

  if (get_battery_voltage() < 6.5) error(1);

  m(0, 0, 0);
}

void loop() {
  // Delete message after 50ms of silence
  long long start_time = millis();
  while (!Serial.available()) {
    if (millis() - start_time > 50) {
      message_pos = 0;
    }
  }

  while (Serial.available() > 0)  {
    message[message_pos] = Serial.read();
    message_pos++;

    if (message_pos == message_lengths[message[0] - 1]) {
      parse_message();
      message_pos = 0;
    }
  }
}
