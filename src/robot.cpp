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
#include <unistd.h>

#include "defines.h"
#include "utils.h"

int8_t i2c_write_byte_single(uint8_t dev_addr, uint8_t byte) {
	return i2c_write(dev_addr, byte, nullptr, 0);
}

int8_t i2c_write_byte(uint8_t dev_addr, uint8_t reg_addr, uint8_t byte) {
	/*if(i2c_smbus_write_byte(dev_addr, byte) < 0) {
		std::cerr << "I2C write byte failed" << std::endl;
		return 1;
	}
	return 0;*/
	return i2c_write(dev_addr, reg_addr, &byte, 1);
}

// VL53L0X needs this for some reason
int8_t i2c_write_word(uint8_t dev_addr, uint8_t reg_addr, uint16_t word) {
	uint8_t buf[2];
	buf[0] = word >> 8;
	buf[1] = word;

	return i2c_write(dev_addr, reg_addr, buf, 2);
}

int8_t i2c_write(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint8_t cnt) {
	/*if(i2c_smbus_write_block_data(dev_addr, reg_addr, cnt, reg_data) < 0) {
		std::cerr << "I2C write failed" << std::endl;
		return 1;
	}
	return 0;*/
	/*char msg[cnt + 1] = {reg_addr};
	for(int i = 0; i < cnt; i++) {
		msg[i + 1] = reg_data[i];
	}
	if(write(dev_addr, msg, cnt + 1) != cnt + 1) {
		std::cerr << "I2C transaction failed" << std::endl;
	}*/
	uint8_t buf[128];
	buf[0] = reg_addr;
	if(reg_data != nullptr) memcpy(buf + 1, reg_data, cnt);
	int8_t count = write(dev_addr, buf, cnt + 1);
	if(count < 0) {
		std::cerr << "I2C write failed" << std::endl;
		close(dev_addr);
		exit(ERRCODE_I2C);
	} else if(count != cnt + 1) {
		std::cerr << "Short write to device" << std::endl;
		close(dev_addr);
		exit(ERRCODE_I2C);
	}
	return 0;
}

int8_t i2c_read_byte(uint8_t dev_addr, uint8_t reg_addr, uint8_t* data) {
	return i2c_read(dev_addr, reg_addr, data, 1);
}

int8_t i2c_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint8_t cnt) {
	/*if(i2c_smbus_read_block_data(dev_addr, reg_addr, reg_data) < 0) {
		std::cerr << "I2C read failed" << std::endl;
		return 1;
	}*/
	if(write(dev_addr, &reg_addr, 1) != 1) {
		std::cerr << "I2C read failed (reg addr write)" << std::endl;
		close(dev_addr);
		exit(ERRCODE_I2C);
	}
	int8_t count = read(dev_addr, data, cnt);
	if(count < 0) {
		std::cerr << "I2C read failed" << std::endl;
		close(dev_addr);
		exit(ERRCODE_I2C);
	} else if(count != cnt + 1) {
		std::cerr << "Short read from device" << std::endl;
		close(dev_addr);
		exit(ERRCODE_I2C);
	}

	return 0;
}

// bno055 needs this
void i2c_delay_msec(uint32_t ms) {
	usleep(ms * 1000);
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

	device_addresses.push_back(TEENSY_I2C_ADDR);
	device_addresses.push_back(BNO055_I2C_ADDR);
	device_addresses.push_back(VL53L0X_FORWARD_ADDR);
	device_addresses.push_back(VL53L0X_SIDE_ADDR);
	// Init Teensy (Just select to see if it works)
	select_device(DEV_TEENSY);

	// Init BNO055
	/*select_device(DEV_BNO055);

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
	}*/

	const uint8_t vl53l0x_xshuts[2] = {
		VL53L0X_FORWARD_XSHUT,
		VL53L0X_SIDE_XSHUT
	};

	const uint8_t vl53l0x_addresses[2] = {
		VL53L0X_FORWARD_ADDR,
		VL53L0X_SIDE_ADDR
	};

	// Init VL53L0Xs
	for(int i = 0; i < 2; ++i) {
		vl53l0x_vec.push_back(std::make_unique<VL53L0X>(vl53l0x_xshuts[i]));
		// Set I2C interface functions
		vl53l0x_vec[i]->i2c_readByte = &i2c_read_byte;
		vl53l0x_vec[i]->i2c_readBytes = &i2c_read;
		vl53l0x_vec[i]->i2c_writeByte = &i2c_write_byte;
		vl53l0x_vec[i]->i2c_writeBytes = &i2c_write;
		vl53l0x_vec[i]->i2c_writeWord = &i2c_write_word;

		select_device(DEV_VL53L0X + i);

		// Power each sensor off to configure and be able to set unique addresses
		vl53l0x_vec[i]->powerOff();
	}

	for(int i = 0; i < 2; ++i) {
		delay(20);
		select_device(DEV_VL53L0X + i);
		delay(20);
		vl53l0x_vec[i]->initialize();
		vl53l0x_vec[i]->setTimeout(200);
		vl53l0x_vec[i]->setMeasurementTimingBudget(40000);
		vl53l0x_vec[i]->setAddress(vl53l0x_addresses[i]);
	}

	std::cout << "VL53L0Xs initialized" << std::endl;
}

void Robot::select_device(uint8_t device_id) {
	if(selected_device == device_id) return;

	if(ioctl(i2c_fd, I2C_SLAVE, device_addresses[device_id]) < 0) {
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
	uint8_t msg[2] = {*(uint8_t*)(&left), *(uint8_t*)(&right)};
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
	int16_t data = 0.0;

	if(bno055_read_euler_h(&data) != 0) {
		std::cerr << "Error reading euler data" << std::endl;
		exit(ERRCODE_BNO055);
	}
	return DTOR((float)data / 16.0f);
}

float Robot::read_pitch() {
	select_device(DEV_BNO055);
	int16_t data = 0.0;

	if(bno055_read_euler_r(&data) != 0) {
		std::cerr << "Error reading euler data" << std::endl;
		exit(ERRCODE_BNO055);
	}
	return DTOR((float)data / 16.0f);
}