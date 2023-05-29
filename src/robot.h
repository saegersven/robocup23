#pragma once

#include <cstdlib>
#include <cstdint>
#include <vector>
#include <atomic>

#include "lccv.hpp"

// PROTOCOL
#define CMD_MOTOR                 0x01
#define CMD_STOP                  0x02
#define CMD_SERVO_ATTACH_DETACH   0x03
#define CMD_SERVO_WRITE           0x04
#define CMD_SENSOR                0x05
#define CMD_GRIPPER               0x06
#define CMD_TURN                  0x07
#define CMD_M_BTN_OBSTACLE        0x08

#define SERVO_ARM 0
#define SERVO_GATE 1

#define ARM_LOWER_POS 3
#define ARM_HIGHER_POS 180
#define GATE_OPEN 110
#define GATE_CLOSED 17

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

	void init_serial();

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
	 * Send Servo command with servo id and angle.
	 */
	void attach_detach_servo(uint8_t servo_id);
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
	 * Read distance
	 */
	int distance(uint8_t sensor_id = 0);
	int distance_avg(uint8_t num_measurements, float remove_percentage);
};
