// general pins:
#define LED1 37
#define LED2 38
#define BAT_VOLTAGE A20

// motors:
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

// servos:
#define servo_cam_pin 36
#define servo_arm_pin 20

// I2C commands
#define CMD_MOTOR 0x01
#define CMD_STOP 0x02