// general pins
#define LED1 37 // green LED on PCB
#define LED2 38 // red LED on PCB
#define LED3 32 // LED ring from restart btn
#define BAT_VOLTAGE A20

// motors
#define LF1_PIN 3
#define LF2_PIN 4
#define LF_PWM_PIN 2

#define LB1_PIN 9
#define LB2_PIN 8
#define LB_PWM_PIN 10

#define RF1_PIN 16
#define RF2_PIN 17
#define RF_PWM_PIN 14

#define RB1_PIN 6
#define RB2_PIN 5
#define RB_PWM_PIN 7

#define BACKWHEEL_FACTOR 1.1 // diameter of front wheels is roughtly 10% greater than diameter of back wheels

// servo pins
#define SERVO_CAM_PIN 21 // servo to change cam position
#define SERVO_ARM_PIN 22 // servo to lift up gripper
#define SERVO_GRIPPER1_PIN 20 // servo to grab victims
#define SERVO_GRIPPER2_PIN 36 // 2th servo to grab victims
#define SERVO_GATE_PIN 23 // servo to empty victim container

// servo position (UPDATE IN BOTH FILES!!!)
#define CAM_LOWER_POS 73
#define CAM_HIGHER_POS 120
#define ARM_LOWER_POS 3
#define ARM_HIGHER_POS 180

#define GRIPPER1_OPEN 160
#define GRIPPER1_CLOSED 60

#define GRIPPER2_OPEN 70
#define GRIPPER2_CLOSED 160

#define GATE_OPEN 110
#define GATE_CLOSED 17

// SPI Pins
#define MISO_PIN 12
#define MOSI_PIN 11
#define SCK_PIN 13
#define CS_PIN 15

// SPI commands (abitrary numbers)
#define CMD_MOTOR 11
#define CMD_STOP 21
#define CMD_SERVO 31
#define CMD_READY 41
#define CMD_SERVOS_HOME_POS 51
#define CMD_ARM_DOWN 61
#define CMD_ARM_UP 71
#define CMD_ARM_HALF_UP 81
#define CMD_UNLOAD 91
#define CMD_PICK_UP_RESCUE_KIT 101
#define CMD_PICK_UP_VICTIM 111
