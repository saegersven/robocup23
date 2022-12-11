#include <TSPISlave.h>

uint8_t miso = 12;
uint8_t mosi = 11;
uint8_t sck = 13;
uint8_t cs = 15;
uint8_t spimode = 8;

TSPISlave spi = TSPISlave(SPI, miso, mosi, sck, cs, spimode);

void setup() {
  Serial.begin(115200);

  pinMode(cs, INPUT);
  pinMode(miso, INPUT);
  pinMode(sck, INPUT);
  //spi.onReceive(spi_receive);

  Serial.println("START!");
}

void read_spi(byte* data, int* len) {
    uint8_t counter = 0;
    while (digitalRead(sck) == LOW) {
      if(digitalRead(cs) == HIGH) return;
    }
    data[*len] |= digitalRead(miso) << counter;
    ++counter;
    if (counter == 8) {
      ++len;
      counter = 0;
    }
    while (sck == HIGH) {
      if(digitalRead(sck) == HIGH) return;
    }
}

void loop() {
  if (digitalRead(cs) == LOW) {
    byte data[128];
    int len = 0;
    Serial.println("DATA OGHHHHJ");
    read_spi(data, &len);
    Serial.print(len);
    Serial.println("WORKED VERY GOODLY YES YES YES!!11!");
    for(int i = 0; i < len; ++i) {
      Serial.print((int)data[i]);
      if(i != len - 1) Serial.print("\t");
    }
    Serial.println();
  }
}
