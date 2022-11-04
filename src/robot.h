#pragma once

#define TEENSY_I2C_ADDR 0x2a

#define CMD_MOTOR 0x01
#define CMD_STOP 0x02

#define BTN_RESTART 19 // TODO

class Robot {
private:
	int teensy_fd;
public:
	Robot();

	bool button(uint8_t pin);

	void i2c_write(char* data, int len);

	void m(int8_t left, int8_t right, uint16_t duration = 0);
	void stop(uint8_t brake = 0xFF);
};