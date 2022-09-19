void init() {
  Serial.begin(11520);
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  //displayBatVoltage();
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

  // TODO: change to proper voltages
  if (batVoltage > 8.0) leds(255, 0);       // 100%
  else if (batVoltage > 8.0) leds(230, 1);  // 88%
  else if (batVoltage > 8.0) leds(200, 3);  // 75%
  else if (batVoltage > 8.0) leds(150, 5);  // 63%
  else if (batVoltage > 8.0) leds(123, 13); // 50%
  else if (batVoltage > 8.0) leds(80, 25);  // 38%
  else if (batVoltage > 8.0) leds(40, 80);  // 25%
  else if (batVoltage > 8.0) leds(20, 130); // 13%
  else if (batVoltage > 8.0) leds(0, 255);  // 0%
}

void leds(byte b1, byte b2) {
  analogWrite(LED1, b1);
  analogWrite(LED2, b2);
}
