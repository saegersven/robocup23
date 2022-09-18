void init() {
  Serial.begin(11520);
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
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
  /*
  // Battery has over 70%
  if (batVoltage > 7.91) {
    digitalWrite(LED1, HIGH);
    digitalWrite(LED2, LOW);
  }

  // over 35%
  else if (batVoltage > 7.57) {
    digitalWrite(LED1, HIGH);  
    digitalWrite(LED2, HIGH);  
  }

  // over 10%
  else if (batVoltage > 7.37) {   
    digitalWrite(LED1, LOW); 
    digitalWrite(LED2, HIGH);
  }

  // nearly empty
  else {
    digitalWrite(LED2, HIGH);
    delay(50);
    digitalWrite(LED2, LOW);
    delay(50);
    digitalWrite(LED2, HIGH);
    delay(50);
    digitalWrite(LED2, LOW);
    delay(50);
    digitalWrite(LED2, HIGH);
    delay(50);
    digitalWrite(LED2, LOW);
    delay(50);
  }*/
  // change brightness of red and green led relative to battery voltage
  int brightness_red_led = -120 * batVoltage + 970; 
  int brightness_green_led = 120 * batVoltage - 700;
  brightness_red_led = max(0, brightness_red_led);
  brightness_red_led = min(255, brightness_red_led);
  brightness_green_led = max(0, brightness_green_led);
  brightness_green_led = min(255, brightness_green_led);
  analogWrite(LED1, brightness_green_led);
  analogWrite(LED2, brightness_red_led);
}
