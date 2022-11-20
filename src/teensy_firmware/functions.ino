void init() {
  Serial.begin(115200);
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  pinMode(LED3, OUTPUT);

  pinMode(lf1, OUTPUT);
  pinMode(lf2, OUTPUT);
  pinMode(lf_pwm, OUTPUT);
  pinMode(lb1, OUTPUT);
  pinMode(lb2, OUTPUT);
  pinMode(lb_pwm, OUTPUT);
  pinMode(rf1, OUTPUT);
  pinMode(rf2, OUTPUT);
  pinMode(rf_pwm, OUTPUT);
  pinMode(rb1, OUTPUT);
  pinMode(rb2, OUTPUT);
  pinMode(rb_pwm, OUTPUT);

  servo_cam.attach(servo_cam_pin);
  servo_arm.attach(servo_arm_pin);
  servo_gripper1.attach(servo_gripper1_pin);
  servo_gripper2.attach(servo_gripper2_pin);
  servo_gate.attach(servo_gate_pin);
  
  servo_cam.write(cam_lower_pos);
  servo_gripper1.write(gripper1_closed);
  servo_gripper2.write(gripper2_closed);
  servo_arm.write(arm_higher_pos);
  servo_gate.write(gate_closed);
  displayBatVoltage();
  // wait for servos to finish turning
  for (int i = 0; i < 3; ++i) {
    digitalWrite(LED3, HIGH);
    delay(200);
    digitalWrite(LED3, LOW);
    delay(200);
  }
  debugln("Setup completed");
}

// returns current battery voltage in Volts
float batteryVoltage(int iterations) {
  float sum = 0.0;
  for (int i = 0; i < iterations; i++) {
    sum = sum + analogRead(BAT_VOLTAGE);
  }
  return (sum / iterations) * 0.00955;
}

// displays battery voltage through two LEDs
void displayBatVoltage() {
  float batVoltage = batteryVoltage(1);
  debugln();
  debug("Battery voltage: ");
  debug(batVoltage);
  debugln("V");

  if (batVoltage > 8.05) leds(255, 0);       // > 80%
  else if (batVoltage > 7.91) leds(230, 1);  // > 70%
  else if (batVoltage > 7.75) leds(200, 3);  // > 60%
  else if (batVoltage > 7.67) leds(150, 5);  // > 50%
  else if (batVoltage > 7.59) leds(123, 13); // > 40%
  else if (batVoltage > 7.53) leds(80, 25);  // > 30%
  else if (batVoltage > 7.45) leds(40, 80);  // > 20%
  else if (batVoltage > 7.37) leds(20, 130); // > 10%
  else leds(0, 255);                         // 0%
}

void leds(byte b1, byte b2) {
  analogWrite(LED1, b1);
  analogWrite(LED2, b2);
}
// controls single motor
void single_m(int inA_pin, int inB_pin, int pwm_pin, int pwm) {
  if (pwm > 0) {
    digitalWrite(inA_pin, HIGH);
    digitalWrite(inB_pin, LOW);
  }
  else if (pwm < 0) {
    digitalWrite(inA_pin, LOW);
    digitalWrite(inB_pin, HIGH);
  }
  else {
    digitalWrite(inA_pin, LOW);
    digitalWrite(inB_pin, LOW);
  }
  pwm = abs(pwm) * 2;
  if (pwm > 255) pwm = 255;
  //debugln(pwm); // TODO: Test
  analogWrite(pwm_pin, pwm);
}

// controls all four motors
void m(int left_speed, int right_speed, int duration) {
  single_m(lf1, lf2, lf_pwm, left_speed);
  single_m(rf2, rf1, rf_pwm, right_speed);

  int s = (left_speed + right_speed) / 2;
  int d = right_speed - left_speed;
  single_m(lb1, lb2, lb_pwm, (s - d) * backwheel_factor);
  single_m(rb1, rb2, rb_pwm, (s + d) * backwheel_factor);

  delay(duration);

  // stop all motors after movement
  if (duration != 0) stop();
}

void stop() {
  digitalWrite(lf1, LOW);
  digitalWrite(lf2, LOW);
  digitalWrite(lb1, LOW);
  digitalWrite(lb2, LOW);

  digitalWrite(rf1, LOW);
  digitalWrite(rf2, LOW);
  digitalWrite(rb1, LOW);
  digitalWrite(rb2, LOW);
}

void m(int speed) {
  m(speed, speed, 0); // does not stop after motor movement
}

void pick_up_victim() {
  servo_arm.write((int) arm_higher_pos / 3);
  delay(450);
  servo_gripper1.write(gripper1_open);
  servo_gripper2.write(gripper2_open);
  servo_arm.write(arm_lower_pos);
  delay(300);
  m(100, 100, 300);
  servo_gripper1.write(gripper1_closed);
  servo_gripper2.write(gripper2_closed);
  delay(1000);
  servo_arm.write(arm_higher_pos);
  delay(800);
  servo_gripper1.write(gripper1_open);
  servo_gripper2.write(gripper2_open);
  m(-100, -100, 300);
  servo_gripper1.write(gripper1_closed);
  servo_gripper2.write(gripper2_closed);
}
