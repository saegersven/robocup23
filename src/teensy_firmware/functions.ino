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
  pinMode(GRIPPER_MOTOR_PIN1, OUTPUT);
  pinMode(GRIPPER_MOTOR_PIN2, OUTPUT);

  pinMode(CS_PIN, INPUT);
  pinMode(MOSI_PIN, INPUT);
  pinMode(SCK_PIN, INPUT);

  attachInterrupt(SCK_PIN, sck_rising_interrupt, RISING);
  attachInterrupt(MISO_PIN, stop, RISING);

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

  single_m(LB1_PIN, LB2_PIN, LB_PWM_PIN, left_speed * BACKWHEEL_FACTOR);
  single_m(RB1_PIN, RB2_PIN, RB_PWM_PIN, right_speed * BACKWHEEL_FACTOR);

  delay(abs(duration));

  // stop all motors after movement
  if (duration != 0) stop();
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

  analogWrite(LF_PWM_PIN, 255);
  analogWrite(RF_PWM_PIN, 255);
  analogWrite(LB_PWM_PIN, 255);
  analogWrite(RB_PWM_PIN, 255);
}

void m(int speed) {
  m(speed, speed, 0); // does not stop after motor movement
}

// picks up rescue kit
void pick_up_rescue_kit() {
  long start = millis();
  stop();
  close_gripper(0);
  servo_arm.attach(SERVO_ARM_PIN);
  servo_arm.write(ARM_LOWER_POS);
  delay(150);
  m(-65, -65, 150);
  close_gripper(1);
  delay(20);
  open_gripper(0);
  delay(420);
  m(60, 60, 260);
  close_gripper(0);
  delay(50);
  m(30, 30, 0);
  delay(320);
  open_gripper(0);
  delay(50);
  close_gripper(0);
  delay(150);
  stop();
  m(-65, -65, 100);

  servo_arm.write(ARM_HIGHER_POS);
  delay(10);
  m(-50, -50, 350);
  delay(700);
  open_gripper(50);
  delay(100);
  close_gripper(200);
  servo_arm.detach();
  Serial.println(millis() - start);
}

void open_gripper(int duration) {
  digitalWrite(GRIPPER_MOTOR_PIN1, LOW);
  digitalWrite(GRIPPER_MOTOR_PIN2, HIGH);
  if (duration != 0) {
    delay(duration);
    digitalWrite(GRIPPER_MOTOR_PIN1, LOW);
    digitalWrite(GRIPPER_MOTOR_PIN2, LOW);
  }
}

void close_gripper(int duration) {
  digitalWrite(GRIPPER_MOTOR_PIN1, HIGH);
  digitalWrite(GRIPPER_MOTOR_PIN2, LOW);
  if (duration != 0) {
    delay(duration);
    digitalWrite(GRIPPER_MOTOR_PIN1, LOW);
    digitalWrite(GRIPPER_MOTOR_PIN2, LOW);
  }
}

void pick_up_victim() {
  close_gripper(150);
  servo_arm.attach(SERVO_ARM_PIN);
  servo_arm.write(ARM_LOWER_POS);
  delay(350);
  open_gripper(0);
  delay(500);
  m(50, 50, 0);
  delay(210);
  close_gripper(0);
  delay(100);
  m(-50, -50, 80);
  delay(300);
  m(-35, -35, 630);
  delay(500);
  open_gripper(0);
  m(127, 127, 0);
  delay(40);
  close_gripper(0);
  delay(20);
  stop();
  delay(350);

  // move arm up
  servo_arm.attach(SERVO_ARM_PIN);
  servo_arm.write(ARM_HIGHER_POS);
  delay(900);
  // detach servos
  servo_arm.detach();

  open_gripper(50);
  delay(50);
  close_gripper(125);  
}
// opens gripper and moves arm down
void arm_down() {
  // move arm down a bit
  servo_arm.attach(SERVO_ARM_PIN);
  servo_arm.write(ARM_LOWER_POS);
  delay(500);

  // open arm
  servo_gripper1.attach(SERVO_GRIPPER1_PIN);
  servo_gripper2.attach(SERVO_GRIPPER2_PIN);
  servo_gripper1.write(GRIPPER1_OPEN);
  servo_gripper2.write(GRIPPER2_OPEN);
  delay(450);

  // detach servos
  //servo_arm.detach();
  //servo_gripper1.detach();
  //servo_gripper2.detach();
}

// closes gripper and unloads into victim container
void arm_up() {
  // close arm
  servo_gripper1.attach(SERVO_GRIPPER1_PIN);
  servo_gripper2.attach(SERVO_GRIPPER2_PIN);
  servo_gripper1.write(GRIPPER1_CLOSED);
  servo_gripper2.write(GRIPPER2_CLOSED);
  delay(500);

  // move arm up
  servo_arm.attach(SERVO_ARM_PIN);
  servo_arm.write(ARM_HIGHER_POS);
  delay(900);

  // open arm (only one side)
  servo_gripper2.write(GRIPPER2_OPEN);
  delay(500);

  // close arm
  servo_gripper2.write(GRIPPER2_CLOSED);
  delay(500);

  // detach servos
  servo_arm.detach();
  servo_gripper1.detach();
  servo_gripper2.detach();
}

// closes gripper and moves arm half up
void arm_half_up() {
  // close arm
  servo_gripper1.attach(SERVO_GRIPPER1_PIN);
  servo_gripper2.attach(SERVO_GRIPPER2_PIN);
  servo_gripper1.write(GRIPPER1_CLOSED);
  servo_gripper2.write(GRIPPER2_CLOSED);
  delay(500);

  // move arm up a bit
  servo_arm.attach(SERVO_ARM_PIN);
  servo_arm.write((int)ARM_HIGHER_POS / 1.5);
  delay(500);

  // servos aren't detached so they stay in their positons
}

// unloads all victims from victim container and the dead one from within the gripper
void unload_victims() {
  servo_gate.attach(SERVO_GATE_PIN);
  servo_gate.write(GATE_OPEN);
  delay(400);
  servo_gate.detach();
  m(50, 50, 80);
  m(-90, -90, 80);
  delay(100);
  m(50, 50, 80);
  m(-90, -90, 80);
  delay(100);
  m(50, 50, 80);
  m(-90, -90, 80);
  delay(600);
  servo_gate.attach(SERVO_GATE_PIN);
  servo_gate.write(GATE_CLOSED);
  delay(400);
  servo_gate.detach();
}
