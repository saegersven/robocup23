#include <iostream>

#include <linux/i2c-dev.h>
#include <i2c/smbus.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>

#include <chrono>
#include <thread>

#define delay(x) std::this_thread::sleep_for(std::chrono::milliseconds(x))

// Compile with
// g++ -o pi_spi_test pi_spi_test.cpp -lwiringPi
// Run with
// ./pi_spi_test

#define SPI_CHANNEL 0
#define SPI_CLOCK_SPEED 1000000

int main() {
	int fd;
	int adapter_nr = 1;
	char filename[20];

	snprintf(filename, 19, "/dev/i2c-%d", adapter_nr);
	fd = open(filename, O_RDWR);
	if(fd < 0) {
		std::cout << "i2c init failed" << std::endl;
		exit(1);
	}

	int addr = 0x2a;

	if(ioctl(fd, I2C_SLAVE, addr) < 0) {
		std::cout << "device init failed" << std::endl;
		exit(1);
	}

	char buf[1] = {0x01};
	if(write(fd, buf, 1) != 1) {
		std::cout << "i2c transaction failed" << std::endl;
	}

	char res[1];
	while(read(fd, res, 1) != 1) {
		std::cout << ".";
	}
	std::cout << " Read " << std::to_string(res[0]) << std::endl;

	/*auto start = std::chrono::high_resolution_clock::now();

	for(int i = 0; i < 1000; i++) {
		if(write(fd, buf, NUM) != NUM) {
			std::cout << "i2c transaction failed" << std::endl;
		}
	}
	auto end = std::chrono::high_resolution_clock::now();

	std::cout << std::chrono::duration_cast<std::chrono::microseconds>(end - start).count() / 8000 << std::endl;*/



	return 0;
}