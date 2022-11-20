#pragma once

#include <cstdlib>
#include <cstdint>

extern "C" {
#include "BNO055_driver/bno055.h"
}

#define TEENSY_I2C_ADDR 0x2a
#define BNO055_I2C_ADDR 0x28

#define DEV_TEENSY 1
#define DEV_BNO055 2

#define CMD_MOTOR 0x01
#define CMD_STOP 0x02
#define CMD_TURN 0x03
#define CMD_START 0x04

#define BTN_RESTART 4

#define DISTANCE_FACTOR (4.2f + 3 * 0.42f)

/*
 * I2C interface functions
 */
int8_t i2c_write_byte(uint8_t dev_addr, uint8_t byte);
int8_t i2c_write(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint8_t cnt);
int8_t i2c_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint8_t cnt);
void i2c_delay_msec(uint32_t ms);

class Robot {
private:
	int i2c_fd;
	int selected_device;

	// Orientation sensor
	bno055_t bno055;
public:
	Robot();

	bool button(uint8_t pin);

	void select_device(uint8_t device_id);

	void m(int8_t left, int8_t right, uint16_t duration = 0);
	void stop();

	void turn(float angle);

	void send_byte(char b);

	float read_heading();
	float read_pitch();
};
