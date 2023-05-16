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

  // Configure pins
  pinMode(LED_BUILTIN, OUTPUT);

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
      Serial.print("Failed to detect and initialize sensor DIST ");
      Serial.println(i);
    }

    dist_sensors[i].setAddress(dist_addresses[i]);

    dist_sensors[i].setTimeout(200);
    delay(10);
  }
}

void m(int8_t left_speed, int8_t right_speed) {
  // Left motors
  // Motors will be shorted to ground when speed is zero
  digitalWrite(M_LEFT_A, left_speed > 0);
  digitalWrite(M_LEFT_B, left_speed < 0);

  if(left_speed < 0) left_speed = -left_speed;
  uint8_t left_front_pulse_width = left_speed / 128 * 255;
  uint8_t left_rear_pulse_width = left_front_pulse_width * REAR_WHEEL_FACTOR;

  // Set pulse-width to 100% for full stopping power
  if(left_speed == 0) {
    left_front_pulse_width = 255;
    left_rear_pulse_width = 255;
  }
  
  analogWrite(M_LEFT_FRONT_EN, left_front_pulse_width);
  analogWrite(M_LEFT_REAR_EN, left_rear_pulse_width);

  // Right motors
  digitalWrite(M_RIGHT_A, right_speed > 0);
  digitalWrite(M_RIGHT_B, right_speed < 0);

  if(right_speed < 0) right_speed = -right_speed;
  uint8_t right_front_pulse_width = left_speed / 128 * 255;
  uint8_t right_rear_pulse_width = right_front_pulse_width * REAR_WHEEL_FACTOR;

  if(right_speed == 0) {
    right_front_pulse_width = 255;
    right_rear_pulse_width = 255;
  }

  analogWrite(M_RIGHT_FRONT_EN, right_front_pulse_width);
  analogWrite(M_RIGHT_REAR_EN, right_rear_pulse_width);
}

// Dir: -1 for open, 0 for short (both LOW), 1 for close
void gripper(int8_t dir) {
  digitalWrite(M_GRIPPER_A, dir > 0);
  digitalWrite(M_GRIPPER_B, dir < 0);
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
  case CMD_MOTOR:
    int8_t left_speed = message[1];
    int8_t right_speed = message[2];

    m(left_speed, right_speed);
    break;
  case CMD_STOP:
    m(0, 0);
    break;
  case CMD_GRIPPER:
    int8_t gripper_direction = message[1];
    gripper(gripper_direction);
    break;
  case CMD_SENSOR:
    uint8_t sensor_id = message[1];

    uint16_t value = 0;

    switch(sensor_id) {
    case SENSOR_DIST_1:
    case SENSOR_DIST_2:
      // Relies on distance sensors having ids 0, 1, etc.
      // This is the only type of sensor where we need to index an array, so that is fine.
      value = dist_sensors[sensor_id].readRangeSingleMillimeters();
      if(dist_sensors[sensor_id].timeoutOccurred()) { Serial.print("DIST TIMEOUT"); }
      break;
    case SENSOR_BTN:
      value = digitalRead(BTN_PIN);
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