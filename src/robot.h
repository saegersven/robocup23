#pragma once

#include <cstdlib>
#include <cstdint>

#define TEENSY_I2C_ADDR 0x2a

#define CMD_MOTOR 0x01
#define CMD_STOP 0x02
#define CMD_TURN 0x03

#define BTN_RESTART 4

class Robot {
private:
	int teensy_fd;
public:
	Robot();

	bool button(uint8_t pin);

	void i2c_write(char* data, int len);

	void m(int8_t left, int8_t right, uint16_t duration = 0);
	void stop();

	void turn(float angle);
};