#include <SPI.h>
//test
// TEST PACKET
// Pi:
// 1, n, 0
// 2, n, 0
// Teensy:
// -, -, n + 10
// -, -, n * 2

void setup() {
	pinMode(MISO, OUTPUT);

	SPCR |= _BV(SPE);

	SPI.attachInterrupt();
}

int packet_pos = 0;
int command = 0;
int number = 0;

ISR (SPI_STC_vect) {

	switch(packet_pos) {
	case 0:
		command = b;
		break;
	case 1:
		number = b;

		if(command == 1) {
			SPDR = number + 10;
		} else {
			SPDR = number * 2;
		}
		break;
	case 2:
		command = 0;
		number = 0;
		packet_pos = 0;
		break;
	}
	++packet_pos;
}

void loop() {}