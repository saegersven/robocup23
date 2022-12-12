//#include <interrupts.h>
#include <TSPISlave.h>

uint8_t miso = 12;
uint8_t mosi = 11;
uint8_t sck = 13;
uint8_t cs = 15;
uint8_t spimode = 8;

byte buffer[128];
uint8_t num_bits = 0;

TSPISlave spi = TSPISlave(SPI, miso, mosi, sck, cs, spimode);

void setup() {
  Serial.begin(115200);

  pinMode(cs, INPUT);
  pinMode(miso, INPUT);
  pinMode(sck, INPUT);
  attachInterrupt(sck, sck_rising_interrupt, RISING);

  Serial.println("START!");
}

void sck_rising_interrupt() {
    buffer[num_bits / 8] |= digitalRead(miso) << (7 - num_bits % 8);
    ++num_bits; 
    Serial.println(num_bits);
}


void loop() {
    while(digitalRead(cs) != LOW);
    while(digitalRead(cs) == LOW);
    int len = num_bits / 8;
    for(int i = 0; i < len; ++i) {
      Serial.print((int)buffer[i]);
      if(i != len - 1) Serial.print("\t");
    }
    num_bits = 0;

}
