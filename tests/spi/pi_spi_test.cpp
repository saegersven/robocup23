#include <iostream>
#include <wiringPiSPI.h>
#include <wiringPi.h>
#include <unistd.h>

// Compile with
// g++ -o pi_spi_test pi_spi_test.cpp -lwiringPi
// Run with
// ./pi_spi_test

#define SPI_CHANNEL 0
#define SPI_CLOCK_SPEED 50000

int main() {
	wiringPiSetupGpio();
	pinMode(8, OUTPUT);
	digitalWrite(8, HIGH);

	int fd = wiringPiSPISetup(SPI_CHANNEL, SPI_CLOCK_SPEED);
	if(fd == -1) {
		std::cout << "Failed to init SPI communication.\n";
		return -1;
	}
	std::cout << "SPI init successful.\n";

	unsigned char buf1[3] = { 42, 13, 42};

	digitalWrite(8, LOW);	
	wiringPiSPIDataRW(SPI_CHANNEL, buf1, 3);
	digitalWrite(8, HIGH);	
	usleep(100000);

	return 0;
}