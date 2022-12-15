void init() {
  Serial.begin(115200);
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  pinMode(LED3, OUTPUT);

  pinMode(LF1_PIN, OUTPUT);
  pinMode(LF2_PIN, OUTPUT);
  pinMode(LF_PWM_PIN, OUTPUT);
  pinMode(LB1_PIN, OUTPUT);
  pinMode(LB2_PIN, OUTPUT);
  pinMode(LB_PWM_PIN, OUTPUT);
  pinMode(RF1_PIN, OUTPUT);
  pinMode(RF2_PIN, OUTPUT);
  pinMode(RF_PWM_PIN, OUTPUT);
  pinMode(RB1_PIN, OUTPUT);
  pinMode(RB2_PIN, OUTPUT);
  pinMode(RB_PWM_PIN, OUTPUT);

  pinMode(CS_PIN, INPUT);
  pinMode(MOSI_PIN, INPUT);
  pinMode(SCK_PIN, INPUT);
  
  attachInterrupt(SCK_PIN, sck_rising_interrupt, RISING);
  
  displayBatVoltage();
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
  //debugln();
  //debug("Battery voltage: ");
  debugln(batVoltage);
  //debugln("V");

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
  single_m(LF1_PIN, LF2_PIN, LF_PWM_PIN, left_speed);
  single_m(RF2_PIN, RF1_PIN, RF_PWM_PIN, right_speed);

  int s = (left_speed + right_speed) / 2;
  int d = right_speed - left_speed;
  single_m(LB1_PIN, LB2_PIN, LB_PWM_PIN, (s - d) * BACKWHEEL_FACTOR);
  single_m(RB1_PIN, RB2_PIN, RB_PWM_PIN, (s + d) * BACKWHEEL_FACTOR);

  //delay(duration);

  // stop all motors after movement
  //if (duration != 0) stop();
}

void stop() {
  digitalWrite(LF1_PIN, LOW);
  digitalWrite(LF2_PIN, LOW);
  digitalWrite(LB1_PIN, LOW);
  digitalWrite(LB2_PIN, LOW);

  digitalWrite(RF1_PIN, LOW);
  digitalWrite(RF2_PIN, LOW);
  digitalWrite(RB1_PIN, LOW);
  digitalWrite(RB2_PIN, LOW);
}

void m(int speed) {
  m(speed, speed, 0); // does not stop after motor movement
}

void pick_up_victim() {
  servo_arm.write((int) ARM_HIGHER_POS / 3);
  delay(450);
  servo_gripper1.write(GRIPPER1_OPEN);
  servo_gripper2.write(GRIPPER2_OPEN);
  servo_arm.write(ARM_LOWER_POS);
  delay(300);
  m(100, 100, 300);
  servo_gripper1.write(GRIPPER1_CLOSED);
  servo_gripper2.write(GRIPPER2_CLOSED);
  delay(1000);
  servo_arm.write(ARM_HIGHER_POS);
  delay(800);
  servo_gripper1.write(GRIPPER1_OPEN);
  servo_gripper2.write(GRIPPER2_OPEN);
  m(-100, -100, 300);
  servo_gripper1.write(GRIPPER1_CLOSED);
  servo_gripper2.write(GRIPPER2_CLOSED);
}
