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

int8_t i2c_write_byte(uint8_t dev_addr, uint8_t byte) {
	if(i2c_smbus_write_byte(dev_addr, byte) < 0) {
		std::cerr << "I2C write byte failed" << std::endl;
		return 1;
	}
	return 0;
}

int8_t i2c_write(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint8_t cnt) {
	if(i2c_smbus_write_block_data(dev_addr, reg_addr, cnt, reg_data) < 0) {
		std::cerr << "I2C write failed" << std::endl;
		return 1;
	}
	return 0;
}

int8_t i2c_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint8_t cnt) {
	if(i2c_smbus_read_block_data(dev_addr, reg_addr, cnt, reg_data)) {
		std::cerr << "I2C read failed" << std::endl;
		return 1;
	}
	return 0;
}

void i2c_delay_msec(uint32_t ms) {
	delay(ms);
}

Robot::Robot() {
	wiringPiSetupGpio();

	pinMode(BTN_RESTART, INPUT);

	// Init I2C
	i2c_fd = open("/dev/i2c-1", O_RDWR);
	if(i2c_fd < 0) {
		std::cerr << "I2C init failed" << std::endl;
		exit(ERRCODE_I2C);
	}

	// Init Teensy (Just select to see if it works)
	select_device(DEV_TEENSY);

	// Init BNO055
	select_device(DEV_BNO055);

	bno055.bus_write = i2c_write;
	bno055.bus_read = i2c_read;
	bno055.delay_msec = i2c_delay_msec;
	bno055.dev_addr = i2c_fd;

	uint32_t comres = 0;
	comres += bno055_init(&bno055);
	comres += bno055_set_power_mode(BNO055_POWER_MODE_NORMAL);
	comres += bno055_set_operation_mode(BNO055_OPERATION_MODE_NDOF);
	
	if(comres > 0) {
		std::cerr << "BNO055 init failed" << std::endl;
		exit(ERRCODE_BNO055);
	}
}

void Robot::select_device(uint8_t device_id) {
	if(selected_device == device_id) return;

	uint8_t addr = 0;
	switch(device_id) {
	case DEV_TEENSY: addr = TEENSY_I2C_ADDR; break;
	case DEV_BNO055: addr = BNO055_I2C_ADDR; break;
	}

	if(ioctl(i2c_fd, I2C_SLAVE, addr) < 0) {
		std::cerr << "I2C device init failed (" << std::to_string(device_id) + ")" << std::endl;
		exit(ERRCODE_I2C);
	}
	selected_device = device_id;
}

bool Robot::button(uint8_t pin) {
	return digitalRead(pin) == HIGH;
}

void Robot::m(int8_t left, int8_t right, uint16_t duration) {
	select_device(DEV_TEENSY);
	char msg[3] = {(char)left, (char)right};
	i2c_write(i2c_fd, CMD_MOTOR, msg, 2);

	if(duration > 0) {
		delay(duration);
		stop();
	}
}

void Robot::stop() {
	select_device(DEV_TEENSY);
	send_byte(CMD_STOP);
}

void Robot::turn(float angle) {
	select_device(DEV_TEENSY);
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
	select_device(DEV_TEENSY);
	i2c_write_byte(i2c_fd, b);
}

float Robot::read_heading() {
	select_device(DEV_BNO055);
	double data = 0.0;

	if(bno055_read_euler_data_h(&data) != 0) {
		std::cerr << "Error reading euler data" << std::endl;
		exit(ERRCODE_BNO055);
	}
	return DTOR((float)data / 16.0f);
}

float Robot::read_pitch() {
	select_device(DEV_BNO055);
	double data = 0.0;

	if(bno055_read_euler_data_r(&data) != 0) {
		std::cerr << "Error reading euler data" << std::endl;
		exit(ERRCODE_BNO055);
	}
	return DTOR((float)data / 16.0f);
}