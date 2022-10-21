#include <iostream>
#include <wiringPiSPI.h>

// Compile with
// g++ -o pi_spi_test pi_spi_test.cpp -lwiringPi
// Run with
// ./pi_spi_test

#define SPI_CHANNEL 0
#define SPI_CLOCK_SPEED 1000000

int main() {
	int fd = wiringPiSPISetup(SPI_CHANNEL, SPI_CLOCK_SPEED);
	if(fd == -1) {
		std::cout << "Failed to init SPI communication.\n";
		return -1;
	}
	std::cout << "SPI init successful.\n";

	unsigned char buf1[3] = { 1, 32, 0 };
	unsigned char buf2[3] = { 2, 21, 0 };

	unsigned char buf[1] = {1};
	//while(true) {
		wiringPiSPIDataRW(SPI_CHANNEL, buf, 1);
	//}
	return 0;

	std::cout << "Data returned: " << std::to_string(buf1[2]) << "\n";

	wiringPiSPIDataRW(SPI_CHANNEL, buf2, 3);

	std::cout << "Data returned: " << std::to_string(buf2[2]) << "\n";

	return 0;
}