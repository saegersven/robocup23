#include "robot.h"

extern "C" {
#include <linux/i2c-dev.h>
#include <linux/spi/spidev.h>
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
	int i2c_fd = open("/dev/i2c-1", O_RDWR);
	if(i2c_fd < 0) {
		std::cerr << "I2C init failed" << std::endl;
		exit(ERRCODE_I2C);
	}
	if(ioctl(i2c_fd, I2C_SLAVE, dev_addr) < 0) {
		std::cerr << "I2C device init failed (" << std::to_string(dev_addr) + ")" << std::endl;
		close(i2c_fd);
		exit(ERRCODE_I2C);
	}

	uint8_t buf[128];
	buf[0] = reg_addr;
	if(reg_data != nullptr) memcpy(buf + 1, reg_data, cnt);
	int8_t count = write(i2c_fd, buf, cnt + 1);
	if(count < 0) {
		std::cerr << "I2C write failed" << std::endl;
		close(i2c_fd);
		exit(ERRCODE_I2C);
	} else if(count != cnt + 1) {
		std::cerr << "Short write to device" << std::endl;
		close(i2c_fd);
		exit(ERRCODE_I2C);
	}
	close(i2c_fd);

	return 0;
}

int8_t i2c_read_byte(uint8_t dev_addr, uint8_t reg_addr, uint8_t* data) {
	return i2c_read(dev_addr, reg_addr, data, 1);
}

int8_t i2c_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint8_t cnt) {
	// Init I2C
	int i2c_fd = open("/dev/i2c-1", O_RDWR);
	if(i2c_fd < 0) {
		std::cerr << "I2C init failed" << std::endl;
		exit(ERRCODE_I2C);
	}

	// Select device
	if(ioctl(i2c_fd, I2C_SLAVE, dev_addr) < 0) {
		std::cerr << "I2C device init failed (" << std::to_string(dev_addr) + ")" << std::endl;
		close(i2c_fd);
		exit(ERRCODE_I2C);
	}

	// Write register address/command
	if(write(i2c_fd, &reg_addr, 1) != 1) {
		std::cerr << "I2C read failed (reg addr write)" << std::endl;
		std::cerr << std::to_string(reg_addr) << std::endl;
		close(i2c_fd);
		exit(ERRCODE_I2C);
	}

	// Read response
	int8_t count = read(i2c_fd, data, cnt);
	if(count < 0) {
		std::cerr << "I2C read failed" << std::endl;
		close(i2c_fd);
		exit(ERRCODE_I2C);
	} else if(count != cnt) {
		std::cerr << "Short read from device" << std::endl;
		close(i2c_fd);
		exit(ERRCODE_I2C);
	}
	close(i2c_fd);

	return 0;
}

// bno055 needs this
void i2c_delay_msec(uint32_t ms) {
	usleep(ms * 1000);
}

Robot::Robot() : vl53l0x(VL53L0X_FORWARD_XSHUT) {
	wiringPiSetupGpio();

	pinMode(BTN_RESTART, INPUT);
	pinMode(HCSR04_TRIGGER, OUTPUT);
	pinMode(HCSR04_ECHO, INPUT);
	digitalWrite(HCSR04_TRIGGER, LOW);

	spi_init();

#ifdef ENABLE_BNO055
	bno055.bus_write = i2c_write;
	bno055.bus_read = i2c_read;
	bno055.delay_msec = i2c_delay_msec;
	bno055.dev_addr = BNO055_I2C_ADDR;

	uint32_t comres = 0;
	comres += bno055_init(&bno055);
	comres += bno055_set_power_mode(BNO055_POWER_MODE_NORMAL);
	comres += bno055_set_operation_mode(BNO055_OPERATION_MODE_NDOF);

	if(comres > 0) {
		std::cerr << "BNO055 init failed" << std::endl;
		exit(ERRCODE_BNO055);
	}

	uint8_t c_id = 0;
	if(bno055_read_chip_id(&c_id) != 0) std::cout << "OH OH" << std::endl;
	std::cout << std::to_string(c_id) << std::endl;

	std::cout << "BNO055 initialized" << std::endl;
#endif // ENABLE_BNO055

#ifdef ENABLE_VL53L0X
	vl53l0x.i2c_readByte = &i2c_read_byte;
	vl53l0x.i2c_readBytes = &i2c_read;
	vl53l0x.i2c_writeByte = &i2c_write_byte;
	vl53l0x.i2c_writeBytes = &i2c_write;
	vl53l0x.i2c_writeWord = &i2c_write_word;

	vl53l0x.initialize();
	vl53l0x.setTimeout(200);
	vl53l0x.setMeasurementTimingBudget(40000);

	std::cout << "VL53L0X initialized" << std::endl;
#endif // ENABLE_VL53L0X
}

void Robot::spi_init() {
	// Open device
	if((spi_fd = open("/dev/spidev0.0", O_RDWR)) < 0) {
		std::cerr << "Could not open SPI device" << std::endl;
		exit(ERRCODE_SPI);
	}

	// Set mode
	spi_mode = SPI_MODE;
	if(ioctl(spi_fd, SPI_IOC_WR_MODE, &spi_mode) < 0) {
		std::cerr << "Could not set SPI mode" << std::endl;
	}

	if(ioctl(spi_fd, SPI_IOC_RD_MODE, &spi_mode) < 0) {
		std::cout << "Could not get SPI mode" << std::endl;
	}

	// Set bits per word
	spi_bits_per_word = SPI_BITS_PER_WORD;
	if(ioctl(spi_fd, SPI_IOC_WR_BITS_PER_WORD, &spi_bits_per_word) < 0) {
		std::cerr << "Could not set SPI bits per word" << std::endl;
	}

	if(ioctl(spi_fd, SPI_IOC_RD_BITS_PER_WORD, &spi_bits_per_word) < 0) {
		std::cerr << "Could not get SPI bits per word" << std::endl;
	}

	// Set maximum speed
	spi_speed = SPI_SPEED;
	if(ioctl(spi_fd, SPI_IOC_WR_MAX_SPEED_HZ, &spi_speed) < 0) {
		std::cerr << "Could not set SPI maximum speed" << std::endl;
	}

	if(ioctl(spi_fd, SPI_IOC_RD_MAX_SPEED_HZ, &spi_speed) < 0) {
		std::cout << "Could not get SPI maximum speed" << std::endl;
	}
}

void Robot::spi_write(uint8_t* data, uint8_t len) {
	spi_ioc_transfer transfer;
	memset(&transfer, 0, sizeof(transfer));
	transfer.tx_buf = (unsigned long)data;
	transfer.len = len;
	transfer.delay_usecs = 0;
	transfer.speed_hz = spi_speed;
	transfer.bits_per_word = spi_bits_per_word;
	transfer.cs_change = 0;

	int8_t ret = ioctl(spi_fd, SPI_IOC_MESSAGE(1), &transfer);

	if(ret < 0) {
		std::cerr << "SPI transfer failed" << std::endl;
	}
}

bool Robot::button(uint8_t pin) {
	return digitalRead(pin) == HIGH;
}

void Robot::m(int8_t left, int8_t right, uint16_t duration) {
	uint8_t msg[3] = {CMD_MOTOR, *(uint8_t*)(&left), *(uint8_t*)(&right)};
	//i2c_write(TEENSY_I2C_ADDR, CMD_MOTOR, msg, 2);
	spi_write(msg, 3);

	if(duration > 0) {
		delay(duration);
		stop();
	}
}

void Robot::stop() {
	send_byte(CMD_STOP);
}

void Robot::turn(int angle) {
	uint16_t duration = (uint16_t)std::abs(angle) * MS_PER_DEGREE;
	if(angle > 0) {
		m(127, -127, duration);
	} else {
		m(-127, 127, duration);
	}
}

void Robot::send_byte(char b) {
	//i2c_write_byte_single(TEENSY_I2C_ADDR, b);
	uint8_t msg[1] = {b};
	spi_write(msg, 1);
}

void Robot::servo(uint8_t servo_id, uint8_t angle, uint16_t delay_ms) {
	uint8_t msg[3] = {CMD_SERVO, servo_id, angle};
	//i2c_write(TEENSY_I2C_ADDR, CMD_SERVO, msg, 2);
	spi_write(msg, 3);
	delay(delay_ms);
}

float Robot::read_heading() {
	int16_t data = 0.0;

	if(bno055_read_euler_h(&data) != 0) {
		std::cerr << "Error reading euler data" << std::endl;
		exit(ERRCODE_BNO055);
	}
	return DTOR((float)data / 16.0f);
}

float Robot::read_pitch() {
	int16_t data = 0.0;

	if(bno055_read_euler_r(&data) != 0) {
		std::cerr << "Error reading euler data" << std::endl;
		exit(ERRCODE_BNO055);
	}
	return DTOR((float)data / 16.0f);
}

// returns front distance in cm
int Robot::distance() {
	digitalWrite(HCSR04_TRIGGER, HIGH);
    delayMicroseconds(10);
    digitalWrite(HCSR04_TRIGGER, LOW);

	auto starttime = std::chrono::high_resolution_clock::now();
	auto endtime = std::chrono::high_resolution_clock::now();

	while (digitalRead(HCSR04_ECHO) == LOW) starttime = std::chrono::high_resolution_clock::now();
	while (digitalRead(HCSR04_ECHO) == HIGH) endtime = std::chrono::high_resolution_clock::now();

	auto travel_time = endtime - starttime;

	// convert travel time of sound signal to cm (TODO: check for overflow)
	return (int)(0.000000001 * 0.5 * travel_time.count() * 34300);
}