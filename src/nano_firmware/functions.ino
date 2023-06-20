void m(int8_t left_speed, int8_t right_speed, uint16_t duration) {
  Serial.print(left_speed);
  Serial.print(right_speed);
  Serial.println(duration);
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

  if (duration > 0) {
    delay(duration);
    m(0, 0, 0);
  }
}

int sgn(float x) {
  if (x < 0) return -1;
  if (x == 0) return 0;
  if (x > 0) return 1;
}

// angle in mrad
void turn(int16_t mrad) {
  int16_t angle = (int)((float)mrad / 1000.0f * 180.0f / 3.141592f);
  // TODO: fix
  // if (abs(angle) > 30) angle -= 3;
  angle -= 0.14 * angle; // robot overturns slightly, probably because motors don't stop immediately. This is the quick fix
  if (angle == 0) return;

  int min_duration = MIN_TIME_PER_DEG * abs(angle);
  int max_duration = MAX_TIME_PER_DEG * abs(angle);

  float cur_heading = get_heading();
  float final_heading = cur_heading + angle;

  if (final_heading > 360.0f) final_heading -= 360.0f;
  if (final_heading < 0) final_heading += 360.0f;

  m(50 * sgn(angle), -50 * sgn(angle), 0);

  long long start_time = millis();

  while (millis() - start_time < min_duration);
  while (millis() - start_time < max_duration) {
    float heading = get_heading();
    //m((heading + 15)* sgn(angle), -(heading + 15) * sgn(angle), 0); // as robot reaches goal heading, decrease motor speed
    if (abs(heading - final_heading) < TURN_TOLERANCE) {
      break;
    }
  }
  m(0, 0, 50);
}

// Dir: -1 for open, 0 for short (both LOW), 1 for close
void gripper(int8_t dir) {
  digitalWrite(M_GRIPPER_A, dir > 0);
  digitalWrite(M_GRIPPER_B, dir < 0);
}

uint16_t distance(int sensor_id) {
  if (sensor_id > NUM_VL53L0X) return -1;
  return dist_sensors[sensor_id].readRangeContinuousMillimeters();
}

// returns heading of BNO055 (z rotation). Used for accurate turning
float get_heading() {
  sensors_event_t orientationData;
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  return orientationData.orientation.x;
}

// returns pitch of BNO055 (x rotation). Used to detect ramps
// !!! sensor is not mounted flat. Pitch of -5 is normal
float get_pitch() {
  sensors_event_t orientationData;
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  return  orientationData.orientation.z;
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
        // Relies on distance sensors having ids 0, 1, etc.
        // This is the only type of sensor where we need to index an array, so that is fine.
        value = distance(sensor_id);
        break;
      case SENSOR_ID_BTN:
        value = analogRead(PIN_BTN) > 350;
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

// emergency function that is only being called when there is something really really wrong
// currently the function is called when:
// - the battery voltage is critically low (error_code 1)
// - the gyro sensor could not be initialized (error_code 2)
// - one of the distance sensors could not be initialized (error_code 3)
void error(int error_code) {
  while (1) {
    Serial.print("--- ERROR: ");
    Serial.print(error_code);
    Serial.println(" ---");
    digitalWrite(13, HIGH);
    delay(42);
    digitalWrite(13, LOW);
    delay(42);
  }
}
