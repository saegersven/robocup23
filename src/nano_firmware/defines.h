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

// XSHUT pins of VL52L0X
#define PIN_XSHUT_DIST1 12
#define PIN_XSHUT_DIST2 11

// Rear wheels have smaller circumference, therefore must turn faster
#define REAR_WHEEL_FACTOR 1.1

// Only serves to cancel turning after a *long* time so that the robot will not turn indefinitely (when stuck, for example).
#define MIN_TIME_PER_DEG  5 // around 10% less then using gyro. So even if gyro fails robot might still be able to work
#define MAX_TIME_PER_DEG  MIN_TIME_PER_DEG * 5 // always multiply Min * 5!!! Makes sense, trust me :)

#define TURN_TOLERANCE 11 // in deg

#define MS_PER_DEGREE ((4.2f - 0.15f) * 1.08f) // milliseconds a one degree turn takes

// SERVOS
#define NUM_SERVOS 3
int servo_pins[NUM_SERVOS] = {A0, A1, 10};

// SENSOR IDS
#define SENSOR_ID_DIST_1 0
#define SENSOR_ID_DIST_2 1
#define SENSOR_ID_BTN 2
#define SENSOR_RAMP 3

#define NUM_VL53L0X 2
int dist_xshut_pins[NUM_VL53L0X] = {12, 11};
int dist_addresses[NUM_VL53L0X] = {0x8A, 0x8B};

// PROTOCOL
#define CMD_MOTOR                 0x01 // used to drive motors
#define CMD_STOP                  0x02 // stops robot
#define CMD_SERVO_ATTACH_DETACH   0x03 // attach/detach servo
#define CMD_SERVO_WRITE           0x04 // control servo
#define CMD_SENSOR                0x05 // general command to access sensors
#define CMD_GRIPPER               0x06 // opens and closes gripper
#define CMD_TURN                  0x07 // turns robot using gyro
#define CMD_M_BTN_OBSTACLE        0x08 // ???
#define CMD_READY                 0x09 // RasPi compilation complete. Indicates user that robot can be started
#define CMD_TURN_DONE             0x0A // finish signal after robot is done turning

int message_lengths[10] = {5, 1, 2, 3, 2, 2, 3, 3, 1, 1};   
