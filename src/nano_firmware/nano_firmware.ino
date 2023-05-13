#include <Wire.h>
#include <VL53L0X.h>
#include <Servo.h>

#include "defines.h"

const unsigned int MESSAGE_LENGTH = 3;

char message[MESSAGE_LENGTH];
unsigned int message_pos = 0;

Servo servos[NUM_SERVOS];

VL53L0X dist_sensors[NUM_VL53L0X];

void setup() {
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);

  Wire.begin();

  // Configure distance sensors
  for(int i = 0; i < NUM_VL53L0X; ++i) {
    pinMode(dist_xshut_pins[i], OUTPUT);
    delay(10);
    digitalWrite(dist_xshut_pins[i], LOW);
  }
  delay(10);

  for(int i = 0; i < NUM_VL53L0X; ++i) {
    digitalWrite(dist_xshut_pins[i], HIGH);
    delay(10);

    if(!dist_sensors[i].init()) {
      Serial.println("Failed to detect and initialize sensor DIST 2");
    }

    dist_sensors[i].setAddress(dist_addresses[i]);

    dist_sensors[i].setTimeout(500);
    delay(10);
  }
}

void parse_message() {
  switch(message[0]) {
  case CMD_SERVO_ATTACH_DETACH:
  {
    uint8_t servo_id = message[1];

    if(servos[servo_id].attached()) {
      servos[servo_id].detach();
    } else {
      servos[servo_id].attach(servo_pins[servo_id]);
    }
    break;    
  }
  case CMD_SERVO_WRITE:
  {
    uint8_t servo_id = message[1];
    uint8_t angle = message[2];

    bool was_attached = servos[servo_id].attached();
    if(!was_attached) servos[servo_id].attach(servo_pins[servo_id]);

    servos[servo_id].write(angle);
    delay(1000);

    if(!was_attached) servos[servo_id].detach();
    break;
  }
  case CMD_SENSOR:
    uint8_t sensor_id = message[1];

    uint16_t value = 0;

    switch(sensor_id) {
    case SENSOR_DIST_1:
    case SENSOR_DIST_2:
      value = dist_sensors[sensor_id].readRangeSingleMillimeters();
      if(dist_sensors[sensor_id].timeoutOccurred()) { Serial.print("DIST TIMEOUT"); }
      break;
    }

    char msg[2];
    memcpy(msg, &value, 2);

    Serial.write(msg, 2);
    break;
  }
}

void loop() {
  // Delete message after 50ms of silence
  long long start_time = millis();
  while(!Serial.available()) {
    if(millis() - start_time > 50) {
      message_pos = 0;
    }
  }

  while (Serial.available() > 0)  {
    message[message_pos] = Serial.read();
    message_pos++;

    if(message_pos == MESSAGE_LENGTH) {
      parse_message();
      message_pos = 0;
    }
  }
}