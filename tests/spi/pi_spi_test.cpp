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

	unsigned char buf1[3] = { 1, 32, 0 };
	unsigned char buf2[3] = { 2, 21, 0 };

	for(int i = 0; i < 10; ++i) {

		digitalWrite(8, LOW);	
		wiringPiSPIDataRW(SPI_CHANNEL, buf1, 3);
		digitalWrite(8, HIGH);	
		usleep(100000);
	}
	return 0;

	std::cout << "Data returned: " << std::to_string(buf1[2]) << "\n";

	wiringPiSPIDataRW(SPI_CHANNEL, buf2, 3);

	std::cout << "Data returned: " << std::to_string(buf2[2]) << "\n";

	return 0;
}