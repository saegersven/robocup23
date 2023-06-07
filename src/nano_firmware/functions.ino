void m(int8_t left_speed, int8_t right_speed, uint16_t duration) {
  // Left motors
  // Motors will be shorted to ground when speed is zero
  digitalWrite(M_LEFT_A, left_speed < 0);
  digitalWrite(M_LEFT_B, left_speed > 0);

  if (left_speed < 0) {
    if (left_speed == -128) left_speed = -127;
    left_speed = -left_speed;
  }

  uint8_t left_front_pulse_width = left_speed * 2;
  uint16_t left_rear_pulse_width = left_front_pulse_width * REAR_WHEEL_FACTOR;
  if (left_rear_pulse_width > 255) left_rear_pulse_width = 255;

  // Set pulse-width to 100% for full stopping power
  if (left_speed == 0) {
    left_front_pulse_width = 255;
    left_rear_pulse_width = 255;
  }

  // Left motors
  analogWrite(M_LEFT_FRONT_EN, left_front_pulse_width);
  analogWrite(M_LEFT_REAR_EN, left_rear_pulse_width);

  // Right Motors
  digitalWrite(M_RIGHT_A, right_speed < 0);
  digitalWrite(M_RIGHT_B, right_speed > 0);

  if (right_speed < 0) {
    if (right_speed == -128) right_speed = -127;
    right_speed = -right_speed;
  }
  uint8_t right_front_pulse_width = right_speed * 2;
  uint16_t right_rear_pulse_width = right_front_pulse_width * REAR_WHEEL_FACTOR;
  if (right_rear_pulse_width > 255) right_rear_pulse_width = 255;

  if (right_speed == 0) {
    right_front_pulse_width = 255;
    right_rear_pulse_width = 255;
  }

  analogWrite(M_RIGHT_FRONT_EN, right_front_pulse_width);
  analogWrite(M_RIGHT_REAR_EN, right_rear_pulse_width);

  /*Serial.print("Left front: ");
    Serial.print(left_front_pulse_width);
    Serial.print("   Right front: ");
    Serial.print(right_front_pulse_width);

    Serial.print("   Left rear: ");
    Serial.print(left_rear_pulse_width);
    Serial.print("   Right rear: ");
    Serial.print(right_rear_pulse_width);

    Serial.println("");*/

  if (duration > 0) {
    delay(duration);
    m(0, 0, 0);
  }
}

float get_heading() {
  return 0.0f;
}

int sgn(float x) {
  if (x < 0) return -1;
  if (x == 0) return 0;
  if (x > 0) return 1;
}

void turn(int16_t angle) {
  if (angle == 0) return;
  uint16_t duration = abs((float)angle) / 1000.0f / RAD360 * 360.0f * MS_PER_DEGREE;
  if (angle > 0) {
    m(70, -70, duration);
  } else {
    m(-70, 70, duration);
  }

  return;
  // Angle is in milliradians, convert to radians
  float angle_rad = (float)angle / 1000.0f;
  int min_duration = MIN_TIME_PER_RAD * angle_rad;
  int max_duration = MAX_TIME_PER_RAD * angle_rad;

  float final_heading = get_heading() + angle_rad;
  if (final_heading > RAD360) final_heading -= RAD360;
  if (final_heading < 0) final_heading += RAD360;

  m(70 * sgn(angle_rad), -70 * sgn(angle_rad), 0);

  uint16_t start_time = millis();

  while (millis() - start_time < min_duration);
  while (millis() - start_time < max_duration) {
    // TODO: Read heading from sensor
    float heading = get_heading();
    if (abs(heading - final_heading) < TURN_TOLERANCE) {
      break;
    }
  }
  m(0, 0, 0);
}

// Dir: -1 for open, 0 for short (both LOW), 1 for close
void gripper(int8_t dir) {
  digitalWrite(M_GRIPPER_A, dir > 0);
  digitalWrite(M_GRIPPER_B, dir < 0);
}

uint16_t distance(int sensor_id) {
  uint16_t value = dist_sensors[sensor_id].readRangeSingleMillimeters();
  //if(dist_sensors[sensor_id].timeoutOccurred()) { Serial.print("DIST TIMEOUT"); }
  return 2000;
}

// returns battery voltage in V
float get_battery_voltage() {
  return ((5.0f / 1023.0f) * analogRead(PIN_BATTERY_VOLTAGE)) * 2.0f;
}

void toggle_led() {
  if (led_on) {
    digitalWrite(PIN_LED, LOW);
    led_on = false;
  } else {
    digitalWrite(PIN_LED, HIGH);
    led_on = true;
  }
}

void parse_message() {
  if (message[0] == CMD_SERVO_ATTACH_DETACH) {
    uint8_t servo_id = message[1];

    if (servos[servo_id].attached()) {
      servos[servo_id].detach();
    } else {
      servos[servo_id].attach(servo_pins[servo_id]);
    }
  } else if (message[0] == CMD_SERVO_WRITE) {
    uint8_t servo_id = message[1];
    uint8_t angle = message[2];

    bool was_attached = servos[servo_id].attached();
    if (!was_attached) servos[servo_id].attach(servo_pins[servo_id]);

    servos[servo_id].write(angle);

    if (!was_attached) {
      delay(1000);
      servos[servo_id].detach();
    }
  } else if (message[0] == CMD_MOTOR) {
    int8_t left_speed = message[1];
    int8_t right_speed = message[2];
    uint16_t duration = *((uint16_t*)&message[3]);

    m(left_speed, right_speed, duration);
  } else if (message[0] == CMD_STOP) {
    m(0, 0, 0);
  } else if (message[0] == CMD_GRIPPER) {
    int8_t gripper_direction = message[1];
    gripper(gripper_direction);
  } else if (message[0] == CMD_TURN) {
    int16_t angle = *((int16_t*)&message[1]);

    turn(angle);
  } else if (message[0] == CMD_SENSOR) {
    uint8_t sensor_id = message[1];

    uint16_t value = 0;

    switch (sensor_id) {
      case SENSOR_ID_DIST_1:
      case SENSOR_ID_DIST_2:
      case SENSOR_ID_DIST_3:
        // Relies on distance sensors having ids 0, 1, etc.
        // This is the only type of sensor where we need to index an array, so that is fine.
        value = distance(sensor_id);
        break;
      case SENSOR_ID_BTN:
        value = analogRead(PIN_BTN) > 400;
        break;
    }

    char msg[2];
    memcpy(msg, &value, 2);

    Serial.write(msg, 2);
  } else if (message[0] == CMD_M_BTN_OBSTACLE) {
    int8_t left_speed = message[1];
    int8_t right_speed = message[2];

    m(left_speed, right_speed, 0);
    uint16_t dist = distance(SENSOR_ID_DIST_1) / 10;
    if (dist > 127) dist = 127;
    uint8_t flags = dist;
    flags |= (analogRead(PIN_BTN) > 400) << 7;

    Serial.write(&flags, 1);
  } else if (message[0] == CMD_TOGGLE_LED) {
    toggle_led();
  }
}
