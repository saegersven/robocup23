#pragma once

#include <cstdlib>
#include <cstdint>
#include <vector>
#include <atomic>

extern "C" {
#include "BNO055_driver/bno055.h"
}

//#include "vl53l0x-linux/VL53L0X.hpp"

#define SPI_MODE 0
#define SPI_SPEED 1000000 // Hz
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
#define CMD_PICK_UP 101

#define SERVO_CAM 0
#define SERVO_ARM 1
#define SERVO_GRIPPER1 2
#define SERVO_GRIPPER2 3
#define SERVO_GATE 4

// servo position (UPDATE IN BOTH FILES!!!)
#define CAM_LOWER_POS 71
#define CAM_HIGHER_POS 135
#define ARM_LOWER_POS 3
#define ARM_HIGHER_POS 180
#define GRIPPER1_OPEN 60
#define GRIPPER1_CLOSED 125
#define GRIPPER2_OPEN 95
#define GRIPPER2_CLOSED 15
#define GATE_OPEN 100
#define GATE_CLOSED 12

#define BTN_RESTART 4

#define STOP_PIN 9

#define DISTANCE_FACTOR (4.2f + 2 * 0.42f)

#define MS_PER_DEGREE (4.2f - 0.15f) // milliseconds a one degree turn takes

#define T180_ERR (-DTOR(6.0f))

//#define ENABLE_VL53L0X
//#define ENABLE_BNO055

/*
 * I2C interface functions. These are passed to the bno055 and vl53l0x drivers
 * and used for communication with the Teensy.
 */
int8_t i2c_write_byte_single(uint8_t dev_addr, uint8_t byte);
int8_t i2c_write_byte(uint8_t dev_addr, uint8_t reg_addr, uint8_t byte);
int8_t i2c_write(uint8_t dev_addr, uint8_t reg_addr, uint8_t* reg_data, uint8_t cnt);
int8_t i2c_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t* reg_data, uint8_t cnt);
void i2c_delay_msec(uint32_t ms);

class Robot {
private:
	// Linux file handles
	int i2c_fd;
	int spi_fd;

	std::atomic<bool> blocked;

	uint8_t spi_mode;
	uint32_t spi_speed;
	uint8_t spi_bits_per_word;

	// Orientation sensor
	bno055_t bno055;

#ifdef ENABLE_VL53L0X
	// Distance sensor
	VL53L0X vl53l0x;
#endif // ENABLE_VL53L0X
public:
	Robot();

	void set_blocked(bool blocked);

	/**
	 * Initialize spi
	 */
	void spi_init();

	/**
	 * Write bytes to Teensy over SPI
	 */
	void spi_write(uint8_t* data, uint8_t len);

	/**
	 * Simple digital read
	 */
	bool button(uint8_t pin);

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
