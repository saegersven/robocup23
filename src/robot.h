#pragma once

#include <cstdlib>
#include <cstdint>
#include <vector>
#include <atomic>
#include <mutex>

#include <opencv2/opencv.hpp>

// PROTOCOL
#define CMD_MOTOR                 0x01
#define CMD_STOP                  0x02
#define CMD_SERVO_ATTACH_DETACH   0x03
#define CMD_SERVO_WRITE           0x04
#define CMD_SENSOR                0x05
#define CMD_GRIPPER               0x06
#define CMD_TURN                  0x07
#define CMD_M_BTN_OBSTACLE        0x08
#define CMD_READY				  0x09
#define CMD_TURN_DONE             0x0A

#define SERVO_ARM 0
#define SERVO_GATE 1
#define SERVO_CAM 2

#define CAM_LOWER_POS 33
#define CAM_HIGHER_POS 100
#define ARM_LOWER_POS 3
#define ARM_HIGHER_POS 180
#define GATE_OPEN 110
#define GATE_CLOSED 0

#define GRIPPER_OPEN 1
#define GRIPPER_CLOSE -1
#define GRIPPER_OFF 0


// Servo parameters
#define SERVO_MIN_PULSE 1000
#define SERVO_MAX_PULSE 2000
#define SERVO_MIN_ANGLE -45
#define SERVO_MAX_ANGLE 45

#define DISTANCE_FACTOR (4.2f + 2 * 0.42f)

#define CM_TO_MS_FULL_SPEED 18.0f

#define MS_PER_DEGREE 5.5f // milliseconds a one degree turn takes

#define T180_ERR (-DTOR(6.0f))


class Robot {
private:
	// Linux file handles
	int serial_fd;

	std::atomic<bool> blocked;

	std::atomic<bool> has_frame;
	cv::VideoCapture cap;
	std::mutex frame_lock;
	cv::Mat curr_frame;

public:
	Robot();

	void init_serial();

	// Camera thread running?
	std::atomic<bool> camera_running;
	void start_camera();
	void camera_thread();
	void stop_camera();
	void set_cam_angle(uint8_t angle);
	cv::Mat grab_frame();

	void set_blocked(bool blocked);

	int serial_available();

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
	 * Nano turns on green led for 50ms, indicating that robot can be started via button
	 */
	void send_ready();

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
	 * Command gripper
	*/
	void gripper(int8_t gripper_direction, uint16_t delay_ms = 0);

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
	int distance_avg(uint8_t sensor_id, uint8_t num_measurements, float remove_percentage);

	int ramp();

	void servo_cam(int16_t angle, uint16_t d);
};
