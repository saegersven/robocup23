#pragma once

#include <cstdlib>
#include <cstdint>
#include <vector>

extern "C" {
#include "BNO055_driver/bno055.h"
}

#include "vl53l0x-linux/VL53L0X.hpp"

#define TEENSY_I2C_ADDR 0x2a
#define BNO055_I2C_ADDR 0x28

#define VL53L0X_FORWARD_XSHUT -1
#define VL53L0X_FORWARD_ADDR 0x29

#define DIST_FORWARD 0

#define DEV_TEENSY 0
#define DEV_BNO055 1
#define DEV_VL53L0X 2

#define CMD_MOTOR 0x01
#define CMD_STOP 0x02
#define CMD_SERVO 0x03
#define CMD_BEGIN 0x05
#define CMD_END 0x06

#define SERVO_CAM 0
#define SERVO_ARM 1
#define SERVO_GRIPPER1 2
#define SERVO_GRIPPER2 3
#define SERVO_GATE 4


#define CAM_LOWER_POS 75
#define CAM_HIGHER_POS 110
#define ARM_LOWER_POS 20
#define ARM_HIGHER_POS 180
#define GRIPPER1_OPEN 60
#define GRIPPER1_CLOSED 125
#define GRIPPER2_OPEN 95
#define GRIPPER2_CLOSED 15
#define GATE_OPEN 100
#define GATE_CLOSED 12

#define BTN_RESTART 4

#define DISTANCE_FACTOR (4.2f + 3 * 0.42f)

/*
 * I2C interface functions. These are passed to the bno055 driver and used for communication
 * with the Teensy.
 */
int8_t i2c_write_byte_single(uint8_t dev_addr, uint8_t byte);
int8_t i2c_write_byte(uint8_t dev_addr, uint8_t reg_addr, uint8_t byte);
int8_t i2c_write(uint8_t dev_addr, uint8_t reg_addr, uint8_t* reg_data, uint8_t cnt);
int8_t i2c_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t* reg_data, uint8_t cnt);
void i2c_delay_msec(uint32_t ms);

class Robot {
private:
	// Linux I2C file handle
	int i2c_fd;
	std::vector<uint8_t> device_addresses;
	int selected_device;

	// Orientation sensor
	bno055_t bno055;

	// Distance sensor
	VL53L0X vl53l0x;
public:
	Robot();

	/**
	 * Simple digital read
	 */
	bool button(uint8_t pin);

	/**
	 * Select an I2C device
	 */
	void select_device(uint8_t device_id);

	/**
	 * Sends left and right motor speeds to the motor controller.
	 * Sends stop command after specified duration if not zero.
	 */
	void m(int8_t left, int8_t right, uint16_t duration = 0);

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
	 */
	float read_heading();
	float read_pitch();

	/**
	 * Read distance from forward VL53L0X in millimeters
	 */
	uint16_t read_distance();
};
