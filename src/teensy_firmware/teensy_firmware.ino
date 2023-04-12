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
  SERVO_CAM_PIN,
  SERVO_ARM_PIN,
  SERVO_GRIPPER1_PIN,
  SERVO_GRIPPER2_PIN,
  SERVO_GATE_PIN
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

float minvol = 10.0;
void loop() {
  while (digitalRead(CS_PIN) != LOW);
  while (digitalRead(CS_PIN) == LOW);
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
  } else if (cmd == CMD_STOP) {
    stop();
  } else if (cmd == CMD_SERVO) {
    // [CMD_SERVO, servo_id, angle]
    uint8_t servo_id = buffer[1];
    uint8_t angle = buffer[2];

    servos[servo_id].attach(servo_pins[servo_id]);
    debug("Attached Servo: ");
    debugln(servo_id);
    servos[servo_id].write(angle);
    debug("Written Angle: ");
    debugln(angle);
    if (servo_id != 0) {
      delay(800); // wait for servo to finish turning
      servos[servo_id].detach(); // detach servo after turning (despite camera servo)
    }

    /*
      debug("S: ");
      debug((int)data[1]);
      debug(" to ");
      debugln((int)data[2]);
    */
  } else if (cmd == CMD_READY) {
    displayBatVoltage();
    digitalWrite(LED3, HIGH);
    delay(30);
    digitalWrite(LED3, LOW);
  } else if (cmd == CMD_SERVOS_HOME_POS) {

    servo_cam.attach(SERVO_CAM_PIN);
    servo_arm.attach(SERVO_ARM_PIN);
    servo_gripper1.attach(SERVO_GRIPPER1_PIN);
    servo_gripper2.attach(SERVO_GRIPPER2_PIN);
    servo_gate.attach(SERVO_GATE_PIN);

    servo_cam.write(CAM_LOWER_POS);
    servo_gripper1.write(GRIPPER1_CLOSED);
    servo_gripper2.write(GRIPPER2_CLOSED);
    servo_arm.write(ARM_HIGHER_POS);
    servo_gate.write(GATE_CLOSED);

    // wait for servos to finish turning
    delay(800);

    //servo_cam.detach(); // cam servo stays attached
    servo_gripper1.detach();
    servo_gripper2.detach();
    servo_arm.detach();
    servo_gate.detach();

  } else if (cmd == CMD_ARM_DOWN) {
    arm_down();
    Serial.println("Received CMD_ARM DOWN");
  } else if (cmd == CMD_ARM_UP) {
    arm_up();
  } else if (cmd == CMD_ARM_HALF_UP) {
    arm_half_up();
  } else if (cmd == CMD_UNLOAD) {
    unload_victims();
  } else if (cmd == CMD_PICK_UP_RESCUE_KIT) {
    pick_up_rescue_kit();
  } else if (cmd == CMD_PICK_UP_VICTIM) {
    pick_up_victim();
  }

  // clear buffer
  for (int i = 0; i < len; ++i) {
    buffer[i] = 0;
  }
}
