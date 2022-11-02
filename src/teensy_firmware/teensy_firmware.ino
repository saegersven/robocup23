#include "defines.h"
#include <Servo.h>
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
}

void loop() {
  servo_cam.write(30);
  servo_arm.write(180);
  delay(2000);
  servo_cam.write(90);
  servo_arm.write(0);
  delay(2000);
}
