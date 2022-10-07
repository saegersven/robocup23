#include "protocol.h"

Protocol::Protocol() {
	fd = wiringPiSPISetupMode(SPI_CHANNEL, SPI_CLOCK_SPEED, 0);
	if(fd == -1) {
		std::cout << "Failed to init SPI communication.\n";
	} else {
		std::cout << "SPI init successful.\n";
	}
}

bool Protocol::transfer(uint8_t* buf, uint32_t len) {
	uint8_t command = buf[0];
	wiringPiSPIDataRW(fd, buf, len);

	if(buf[packet_lengths[command] - 1] != command) {
		// No acknowledgement from Teensy
		return false;
	}
	return true;
}

bool Protocol::confirm() {
	uint8_t buf[3] = {PACKET_ID_CONFIRM, 0, 0};
	transfer(buf, 3);
	if(buf[1] == CONFIRM_POSITIVE) {
		buf[0] = PACKET_ID_CONFIRM;
		buf[1] = CONFIRM_POSITIVE;
		buf[2] = 0;
		// Send Teensy packet with CONFIRM_POSITIVE as acknowledgement
		transfer(buf, 3);
		return true;
	}
	return false;
}

void Protocol::set_speed(int8_t left, int8_t right) {
	uint8_t buf[4] = {PACKET_ID_SET_SPEED, (uint8_t)left, (uint8_t)right, 0};
	transfer(buf, 4);
}

void Protocol::turn(float angle) {
	uint8_t buf[5] = {PACKET_ID_TURN, 0,0,0,0, 0};
	memcpy(&buf[1], &angle, sizeof(float));
	transfer(buf, 5);

	// Wait until Teensy acknowledges again
	while(!confirm());
}