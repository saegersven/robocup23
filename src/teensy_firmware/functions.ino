void init() {
  Serial.begin(115200);
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  
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
	if (pwm < -255) pwm = 255;
	if (pwm > 255) pwm = 255;
	pwm = abs(pwm);
	debugln(pwm); // TODO: Test
	//analogWrite(pwm_pin, pwm)
}

// controls all four motors
void m(int left_speed, right_speed, int duration) {
	single_m(lf1, lf2, lf_pwm, left_speed);
	single_m(rf1, rf2, rf_pwm, right_speed);

	single_m(lb1, lb2, lb_pwm, left_speed*backwheel_factor);
	single_m(rb1, rb2, rb_pwm, right_speed*backwheel_factor);

	// stop all motors after movement
	if (duration != 0) stop();
}

void stop() m(0, 0); // stops all four motors
void m(int speed) m(speed, 0); // does not stop after motor movement
