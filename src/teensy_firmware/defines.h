// general pins
#define LED1 37 // green LED on PCB
#define LED2 38 // red LED on PCB
#define LED3 32 // LED ring from restart btn
#define BAT_VOLTAGE A20

// motors
#define lf1 3
#define lf2 4
#define lf_pwm 2

#define lb1 9
#define lb2 8
#define lb_pwm 10

#define rf1 16
#define rf2 17
#define rf_pwm 14

#define rb1 6
#define rb2 5
#define rb_pwm 7

#define backwheel_factor 1.1 // diameter of front wheels is roughtly 10% greater than diameter of back wheels

// servo pins
#define servo_cam_pin 21 // servo to change cam position
#define servo_arm_pin 22 // servo to lift up gripper
#define servo_gripper1_pin 20 // servo to grab victims
#define servo_gripper2_pin 36 // 2th servo to grab victims
#define servo_gate_pin 23 // servo to empty victim container

// servo position
#define cam_lower_pos 75
#define cam_higher_pos 90
#define arm_lower_pos 20
#define arm_higher_pos 180
#define gripper1_open 60
#define gripper1_closed 125
#define gripper2_open 95
#define gripper2_closed 15
#define gate_open 100
#define gate_closed 12

// SPI Pins
#define MISO_PIN 12
#define MOSI_PIN 11
#define SCK_PIN 13
#define CS_PIN 15

// SPI commands
#define CMD_MOTOR 0x01
#define CMD_STOP 0x02
#define CMD_SERVO 0x03
#define CMD_READY 0x05
