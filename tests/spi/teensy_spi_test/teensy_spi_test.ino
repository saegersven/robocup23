#include "TSPISlave.h"
//test
// TEST PACKET
// Pi:
// 1, n, 0
// 2, n, 0
// Teensy:
// -, -, n + 10
// -, -, n * 2
volatile uint8_t payload;
uint8_t i = 0;

TSPISlave mySPI = TSPISlave(SPI, 12, 11, 13, 10, 16);
void setup() {
  Serial.begin(115200);
  Serial.println("lol1");
  mySPI.onReceive(myFunc);
}

void loop() {
  delay(4000);
  Serial.println(i);
  cli();
  payload=i;
  sei();
  i++;
}

int packet_pos = 0;
int cmd = 0;
int number = 0;

void myFunc() {
  Serial.println("START: ");
  uint8_t i = 0;
//  while ( mySPI.active() ) {
    if (mySPI.available()) {
      mySPI.pushr(payload);
      Serial.print("VALUE: 0x");
      Serial.println(mySPI.popr(), HEX);
    }
//  }
  Serial.println("END");
}

void on_receive_SPI() {
  if(mySPI.available()) {
    byte b = mySPI.popr();
    Serial.println("Lol");
  }
	/*switch(packet_pos) {
	case 0:
		cmd = b;
		break;
	case 1:
		number = b;

		if(cmd == 1) {
			spi.pushr(number + 10);
		} else {
			spi.pushr(number * 2);
		}
		break;
	case 2:
		cmd = 0;
		number = 0;
		packet_pos = 0;
		break;
	}
	++packet_pos;*/
}
