#include <Servo.h>
#include <Wire.h>

#include "defines.h"

Servo servo_cam;
Servo servo_arm;


#define DEBUG 1
// only print when in debug mode:
#if DEBUG == 1
#define debug(x) Serial.print(x)
#define debugln(x) Serial.println(x)
#else
#define debug(x)
#define debugln(x)
#endif

void setup() {
  init();

  Wire.begin(0x2a);

  Wire.onReceive(onI2CReceive);
}

void onI2CReceive(int n) {
  int counter = 0;
  char data[5];

  while(Wire.available()) {
    char c = Wire.read();
    data[counter] = c;

    switch(data[0]) {
    case CMD_MOTOR:
      // Set motor speeds [left, right]
      if(counter == 2) {
        int left = (int)data[1];
        int right = (int)data[2];

        m(left, right, 0);
      }
      break;
    case CMD_STOP:
      // Stop
      if(counter == 1) {
        stop();
      }
      break;
    }

    ++counter;
  }
}

void loop() {
  
}
