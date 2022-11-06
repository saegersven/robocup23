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
        int8_t left = (int8_t)data[1];
        int8_t right = (int8_t)data[2];

        debug(left);
        debug(" ");
        debugln(right);
        m(left, right, 0);
      }
      break;
    case CMD_STOP:
      // Stop
      if(counter == 0) {
        debugln("Stop");
        stop();
      }
      break;
    }

    ++counter;
  }
}

void loop() {
  while (1) {
    m(-128, 128, 200);
    m(128, -128, 200);
  }
}
