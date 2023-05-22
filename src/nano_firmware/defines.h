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

// Rear wheels have smaller circumference, therefore must turn faster
#define REAR_WHEEL_FACTOR 1.1

// SERVOS
#define NUM_SERVOS 1
//int servo_pins[NUM_SERVOS] = {A0, A1};

// SENSORS
#define SENSOR_DIST_1 0
#define SENSOR_DIST_2 1

#define NUM_VL53L0X 2
int dist_xshut_pins[NUM_VL53L0X] = {2, 3};
int dist_addresses[NUM_VL53L0X] = {0x8A, 0x8B};

#define SENSOR_BTN 2
#define BTN_PIN A6

// PROTOCOL
#define CMD_MOTOR                 0x01
#define CMD_STOP                  0x02
#define CMD_SERVO_ATTACH_DETACH   0x03
#define CMD_SERVO_WRITE           0x04
#define CMD_SENSOR                0x05
#define CMD_GRIPPER               0x06