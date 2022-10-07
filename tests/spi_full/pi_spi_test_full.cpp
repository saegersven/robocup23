#include "pi_spi_test_full.h"

#include <iostream>
#include <wiringPiSPI.h>

// Compile with
// g++ -o pi_spi_test pi_spi_test.cpp -lwiringPi
// Run with
// ./pi_spi_test

#define SPI_CHANNEL 0
#define SPI_CLOCK_SPEED 1000000

uint8_t teensy_acknowledge() {
	uint8_t buf[1] = {PACKET_ID_ACKNOWLEDGE, 0};
	wiringPiSPIDataRW()
}

int main() {

	unsigned char buf1[3] = { 1, 32, 0 };
	unsigned char buf2[3] = { 2, 21, 0 };

	wiringPiSPIDataRW(SPI_CHANNEL, buf1, 3);

	std::cout << "Data returned: " << std::to_string(buf1[2]) << "\n";

	wiringPiSPIDataRW(SPI_CHANNEL, buf2, 3);

	std::cout << "Data returned: " << std::to_string(buf2[2]) << "\n";

	return 0;
}