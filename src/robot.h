#pragma once

#include <cstdlib>
#include <cstdint>
#include <vector>
#include <atomic>

#include "lccv.hpp"

extern "C" {
#include "BNO055_driver/bno055.h"
}

//#include "vl53l0x-linux/VL53L0X.hpp"

#define SPI_MODE 0
#define SPI_SPEED 500'000 // Hz
#define SPI_BITS_PER_WORD 8

#define TEENSY_I2C_ADDR 0x2a
#define BNO055_I2C_ADDR 0x28

#define VL53L0X_FORWARD_XSHUT -1
#define VL53L0X_FORWARD_ADDR 0x29

#define HCSR04_TRIGGER 22
#define HCSR04_ECHO 17

#define DIST_FORWARD 0

#define DEV_TEENSY 0
#define DEV_BNO055 1
#define DEV_VL53L0X 2

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

#define SERVO_CAM 0
#define SERVO_ARM 1
#define SERVO_GRIPPER1 2
#define SERVO_GRIPPER2 3
#define SERVO_GATE 4

// servo position (UPDATE IN BOTH FILES!!!)
#define CAM_LOWER_POS 73
#define CAM_HIGHER_POS 140
#define CAM_MID_POS ((CAM_LOWER_POS + CAM_HIGHER_POS) / 2)
#define CAM_EXIT_POS 90

#define ARM_LOWER_POS 3
#define ARM_HIGHER_POS 180

#define GRIPPER1_OPEN 160
#define GRIPPER1_CLOSED 60

#define GRIPPER2_OPEN 70
#define GRIPPER2_CLOSED 160

#define GATE_OPEN 110
#define GATE_CLOSED 17

#define BTN_RESTART 4

#define STOP_PIN 9

#define DISTANCE_FACTOR (4.2f + 2 * 0.42f)

#define CM_TO_MS_FULL_SPEED 18.0f

#define MS_PER_DEGREE (4.2f - 0.15f) * 1.06f // milliseconds a one degree turn takes

#define T180_ERR (-DTOR(6.0f))

class Robot {
private:
	// Linux file handles
	int serial_fd;

	std::atomic<bool> blocked;

	lccv::PiCamera camera;

public:
	Robot();

	bool camera_running;
	void start_camera(int width, int height, int framerate);
	void stop_camera();
	cv::Mat grab_frame();

	void set_blocked(bool blocked);

	bool button();

	/**
	 * Sends left and right motor speeds to the motor controller.
	 * Sends stop command after specified duration if not zero.
	 */
	void m(int8_t left, int8_t right, int32_t duration = 0);

	/**
	 * Stop all four drive motors
	 */
	void stop();

	/**
	 * Turn by a specified angle (in radians). Positive angles result in clockwise
	 * rotation when viewed from above. Uses BNO055 sensor.
	 */
	void turn(float angle);

	/**
	 * Send a single byte to the Teensy.
	 */
	void send_byte(char b);

	/**
	 * Send Servo command with servo id and angle.
	 */
	void servo(uint8_t servo_id, uint8_t angle, uint16_t delay_ms = 650);

	/**
	 * Read orientation values (euler angles) from BNO055.
	 * Returns angle in radians.
	 * read_heading() acts like a compass.
	 * read_pitch() is used for detecting ramps.
	 */
	float read_heading();
	float read_pitch();

	/**
	 * Read distance from forward HCSR04 ultrasonic sensor in millimeters
	 */
	int distance();
	int distance_avg(uint8_t num_measurements, float remove_percentage);
};
