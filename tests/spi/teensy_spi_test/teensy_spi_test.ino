#include <Wire.h>
 
// LED on pin 13
const int ledPin = 13; 
 
void setup() {
  Serial.begin(115200);
  // Join I2C bus as slave with address 8
  Wire.begin(0x2a);

  Wire.onReceive(onI2CReceive);
  
  // Setup pin 13 as output and turn LED off
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, LOW);
}

int counter = 0;
long t = 0;

void onI2CReceive(int n) {
  while (Wire.available()) { // loop through all but the last
    char c = Wire.read(); // receive byte as a character
    //Serial.print(c);
    counter++;

    Serial.print(c);

    if(c == 0x01) {
      Wire.write(42);
    }
  }
}

void loop() {
  delay(1000);
  Serial.println();
  Serial.println(counter);
}
