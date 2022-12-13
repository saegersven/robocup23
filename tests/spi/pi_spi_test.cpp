#include <iostream>
#include <wiringPiSPI.h>
#include <wiringPi.h>
#include <unistd.h>
#include <cstring>

extern "C" {
	#include <fcntl.h>
	#include <sys/ioctl.h>
	#include <linux/spi/spidev.h>
}

static const char* device = "/dev/spidev0.0";
static uint8_t mode = 0;
static uint8_t bits = 8;
static uint32_t speed = 500000;
int ret, fd;

// Compile with
// g++ -o pi_spi_test pi_spi_test.cpp -lwiringPi
// Run with
// ./pi_spi_test

int main() {
	/* Device oeffen */
	if ((fd = open(device, O_RDWR)) < 0)
	{
		std::cout << "Could not open device" << std::endl;
	}
	/* Mode setzen */
	ret = ioctl(fd, SPI_IOC_WR_MODE, &mode);
	if (ret < 0)
	{
		std::cout << "Could not set mode" << std::endl;
	}

	/* Mode abfragen */
	ret = ioctl(fd, SPI_IOC_RD_MODE, &mode);
	if (ret < 0)
	{
		std::cout << "Could not get mode" << std::endl;
	}

	/* Wortlaenge setzen */
	ret = ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bits);
	if (ret < 0)
	{
		std::cout << "Could not set bits" << std::endl;
	}

	/* Wortlaenge abfragen */
	ret = ioctl(fd, SPI_IOC_RD_BITS_PER_WORD, &bits);
	if (ret < 0)
	{
		std::cout << "Could not get bits" << std::endl;
	}

	/* Datenrate setzen */
	ret = ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed);
	if (ret < 0)
	{
		std::cout << "Could not set speed" << std::endl;
	}
	
	/* Datenrate abfragen */
	ret = ioctl(fd, SPI_IOC_RD_MAX_SPEED_HZ, &speed);
	if (ret < 0)
	{
		std::cout << "Could not get speed" << std::endl;
	}

	srand(time(NULL));
	unsigned char buf[3] = {rand() % 128, 123, 2};
	std::cout << (int)buf[0] << std::endl;
	spi_ioc_transfer spi;
	spi.tx_buf = (unsigned long)buf;
	//spi.rx_buf = (unsigned long)buf;
	spi.len = 3;
	spi.delay_usecs = 0;
	spi.speed_hz = speed;
	spi.bits_per_word = bits;
	spi.cs_change = 0;

	ret = ioctl(fd, SPI_IOC_MESSAGE(1), &spi);

	if(ret < 0) {
		std::cout << "Could not transfer" << std::endl;
	}

	return ret;

	/*wiringPiSetupGpio();
	pinMode(8, OUTPUT);
	digitalWrite(8, HIGH);

	int fd = wiringPiSPISetup(SPI_CHANNEL, SPI_CLOCK_SPEED);
	if(fd == -1) {
		std::cout << "Failed to init SPI communication.\n";
		return -1;
	}
	std::cout << "SPI init successful.\n";

	unsigned char buf1[3] = { 42, 24, 21};

	for(int i = -128; i < 128; ++i) {
		unsigned char buf[3] = {2, 21, 43};
		wiringPiSPIDataRW(SPI_CHANNEL, buf, 3);
		usleep(1000000);
	}*/

	/*digitalWrite(8, LOW);	
	wiringPiSPIDataRW(SPI_CHANNEL, buf1, 3);
	digitalWrite(8, HIGH);	
	usleep(100000);*/

	return 0;
}