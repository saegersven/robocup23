#define CMD_MOTOR    0x01
#define CMD_STOP     0x02
#define CMD_SERVO_ATTACH_DETACH 0x03
#define CMD_SERVO_WRITE    0x04
#define CMD_SENSOR   0x05

const int NUM_SERVOS = 1;
int servo_pins[NUM_SERVOS] = {A0};

#define SENSOR_DIST_1 0
#define SENSOR_DIST_2 1

const int NUM_VL53L0X = 2;
int dist_xshut_pins[NUM_VL53L0X] = {2, 3};
int dist_addresses[NUM_VL53L0X] = {0x8A, 0x8B};