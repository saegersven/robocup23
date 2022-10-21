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

int clamp(int val, int min, int max) {
  return val < min ? min : (val > max ? max : val);
}

// sets lf motor speed to lf
void m_lf(int lf) {
  // limit motor speed to [-255;255]
  lf = clamp(lf, -255, 255);
  Serial.println(lf);

  // stop lf motor
  if (lf == 0) {
    Serial.println("1");
    digitalWrite(lf1, LOW);
    digitalWrite(lf2, LOW);
  }
  // turn lf motor forwards
  else if (lf > 0) {
    Serial.println("2");
    digitalWrite(lf1, HIGH);
    digitalWrite(lf2, LOW);
  }
  // turn lf motor backwards
  else if (lf < 0) {
    Serial.println("3");
    digitalWrite(lf1, LOW);
    digitalWrite(lf2, HIGH);
  }
  Serial.println("4");
  Serial.println(abs(lf));
  analogWrite(lf_pwm, 128);
  Serial.println("5");
}
