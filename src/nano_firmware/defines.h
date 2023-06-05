// CONSTANTS
#define RAD360 6.28318531f

// MOTORS
// A and B are pins to control the direction
// EN are PWM control pins
#define M_LEFT_A          2
#define M_LEFT_B          4
#define M_LEFT_FRONT_EN   3
#define M_LEFT_REAR_EN    5

#define M_RIGHT_A         7
#define M_RIGHT_B         8
#define M_RIGHT_FRONT_EN  6
#define M_RIGHT_REAR_EN   9

// Gripper has no PWM control, only directional
#define M_GRIPPER_A       A3
#define M_GRIPPER_B       A2

#define PIN_BTN A6
#define PIN_LED 13
#define PIN_BATTERY_VOLTAGE A7

// Rear wheels have smaller circumference, therefore must turn faster
#define REAR_WHEEL_FACTOR 1.1

// Only serves to cancel turning after a *long* time so that the robot will not turn indefinitely (when stuck, for example).
#define MAX_TIME_PER_RAD  900
#define MIN_TIME_PER_RAD 200

#define TURN_TOLERANCE 0.0872 // 5°

#define MS_PER_DEGREE ((4.2f - 0.15f) * 1.08f) // milliseconds a one degree turn takes

// SERVOS
#define NUM_SERVOS 2
int servo_pins[NUM_SERVOS] = {A0, A1};

// SENSOR IDS
#define SENSOR_ID_DIST_1 0
#define SENSOR_ID_DIST_2 1
#define SENSOR_ID_DIST_3 2
#define SENSOR_ID_BTN 3

#define NUM_VL53L0X 3
int dist_xshut_pins[NUM_VL53L0X] = {12, 11, 10};
int dist_addresses[NUM_VL53L0X] = {0x8A, 0x8B, 0x8C};

// PROTOCOL
#define CMD_MOTOR                 0x01
#define CMD_STOP                  0x02
#define CMD_SERVO_ATTACH_DETACH   0x03
#define CMD_SERVO_WRITE           0x04
#define CMD_SENSOR                0x05
#define CMD_GRIPPER               0x06
#define CMD_TURN                  0x07
#define CMD_M_BTN_OBSTACLE        0x08
#define CMD_TOGGLE_LED            0x09

int message_lengths[9] = {5, 1, 2, 3, 2, 2, 3, 3, 1};   
