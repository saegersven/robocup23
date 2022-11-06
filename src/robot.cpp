#include "robot.h"

extern "C" {
#include <linux/i2c-dev.h>
#include <i2c/smbus.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
}

#include <wiringPi.h>

#include <cstdlib>
#include <iostream>
#include <chrono>
#include <thread>

#include "defines.h"
#include "utils.h"

Robot::Robot() {
	wiringPiSetupGpio();

	pinMode(BTN_RESTART, INPUT);

	teensy_fd = open("/dev/i2c-1", O_RDWR);
	if(teensy_fd < 0) {
		std::cerr << "I2C init failed" << std::endl;
		exit(ERRCODE_I2C);
	}

	if(ioctl(teensy_fd, I2C_SLAVE, TEENSY_I2C_ADDR) < 0) {
		std::cerr << "I2C device init failed" << std::endl;
		exit(ERRCODE_I2C);
	}
}

bool Robot::button(uint8_t pin) {
	return digitalRead(pin) == HIGH;
}

void Robot::i2c_write(char* data, int len) {
	if(write(teensy_fd, data, len) != len) {
		std::cerr << "I2C transaction failed" << std::endl;
	}
}

void Robot::m(int8_t left, int8_t right, uint16_t duration) {
	char msg[3] = {CMD_MOTOR, (char)left, (char)right};
	i2c_write(msg, 3);

	if(duration > 0) {
		delay(duration);
		stop();
	}
}

void Robot::stop() {
	char msg[1] = {CMD_STOP};
	i2c_write(msg, 1);
}

void Robot::turn(float angle) {
	/*char* angle_bytes = (char*)&angle;
	char msg[5] = {CMD_TURN, angle_bytes[0], angle_bytes[1], angle_bytes[2], angle_bytes[3]};
	i2c_write(msg, 5);*/
	uint16_t duration = (uint16_t)std::abs(angle) * 200.0f;
	if(angle > 0) {
		m(80, -80, duration);
	} else {
		m(-80, 80, duration);
	}
}

void Robot::send_byte(char b) {
	char msg[1] = {b};
	i2c_write(msg, 1);
}