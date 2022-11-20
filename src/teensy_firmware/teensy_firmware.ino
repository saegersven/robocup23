#include <Servo.h>
#include <Wire.h>

#include "defines.h"

Servo servo_cam;
Servo servo_arm;
Servo servo_gripper1;
Servo servo_gripper2;
Servo servo_gate;

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
      // RasPi program has been stopped
      if(counter == 0) {
        debugln("Stop");
        stop();
        digitalWrite(LED3, HIGH);
        displayBatVoltage();
      }
      break;
    case CMD_START:
       digitalWrite(LED3, LOW);
    }
    ++counter;
  }
}

void loop() {
  /*
  pick_up_victim();
  delay(1000);
  pick_up_victim();
  delay(1000);
  pick_up_victim();
  servo_gate.write(gate_open);
  m(100, 100, 50);
  m(-100, -100, 50);
  m(100, 100, 50);
  m(-100, -100, 50);
  m(100, 100, 50);
  m(-100, -100, 50);
  m(100, 100, 50);
  m(-100, -100, 50);
  m(100, 100, 50);
  m(-100, -100, 50);
  m(100, 100, 50);
  m(-100, -100, 50);
  delay(1000);
  servo_gate.write(gate_closed);
  
  delay(1000000);
  */
}
