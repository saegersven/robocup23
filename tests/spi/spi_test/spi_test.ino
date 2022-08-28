#include<SPI.h>
volatile boolean received;
volatile byte Slavereceived 
volatile byte Slavesend;

void setup() {
  Serial.begin(115200);
  pinMode(MISO, OUTPUT);
  SPCR |= _BV(SPE);
  received = false;
  SPI.attachInterrupt();
}

void loop() {
  if (received) {
    if (Slavereceived == 1) {
      Serial.println("Slave LED ON");
    } else {
      Serial.println("Slave LED OFF");
    }
    SPDR = 42;
    delay(1000);
  }
}

ISR (SPI_STC_vect) {
  Slavereceived = SPDR;
  received = true;
}
