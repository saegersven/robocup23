#include <Servo.h>
#include <SPI.h>

#include "defines.h"

Servo servo_cam;
Servo servo_arm;
Servo servo_gripper1;
Servo servo_gripper2;
Servo servo_gate;


// GLOBAL VARIABLES
uint8_t buffer[128]; // stores received spi data
uint8_t num_bits = 0; // number of received bits

const Servo servos[5] = {
  servo_cam,
  servo_arm,
  servo_gripper1,
  servo_gripper2,
  servo_gate
};

const int servo_pins[5] = {
  servo_cam_pin,
  servo_arm_pin,
  servo_gripper1_pin,
  servo_gripper2_pin,
  servo_gate_pin
};

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

// interrupt is called on rising spi clock
void sck_rising_interrupt() {
  buffer[num_bits / 8] |= digitalRead(MOSI_PIN) << (7 - num_bits % 8); // write received data to buffer
  ++num_bits;
}

int time_since_last_bat_update = 0;
float minvol = 10.0;
void loop() {
  if (millis() - time_since_last_bat_update >= 500) {
    time_since_last_bat_update = millis();
    displayBatVoltage();
  }
  while (digitalRead(CS_PIN) != LOW);
  while (digitalRead(CS_PIN) == LOW);
  debugln("RECEIVED COMMAND");
  int len = num_bits / 8;
  num_bits = 0;
  
  int cmd = (int)buffer[0]; // received command
  
  if (cmd == CMD_MOTOR) {
    // [CMD_MOTOR, left, right]
    int8_t left = (int8_t)buffer[1];
    int8_t right = (int8_t)buffer[2];

    m(left, right, 0);

    debug("M: ");
    debug(left);
    debug(" ");
    debugln(right);
  } else if(cmd == CMD_STOP) {
    stop();
  } else if(cmd == CMD_SERVO) {
    // [CMD_SERVO, servo_id, angle]
    uint8_t servo_id = buffer[1];
    uint8_t angle = buffer[2];

    if (!servos[servo_id].attached()) servos[servo_id].attach(servo_pins[servo_id]);
    servos[servo_id].write(angle);
    //servos[servo_id].detach();

    /*
      debug("S: ");
      debug((int)data[1]);
      debug(" to ");
      debugln((int)data[2]);
    */
  } else if (cmd == CMD_READY) {
    digitalWrite(LED3, HIGH);
    delay(200);
    digitalWrite(LED3, LOW);
  }

  // clear buffer
  for (int i = 0; i < len; ++i) {
    buffer[i] = 0;
  }
}
