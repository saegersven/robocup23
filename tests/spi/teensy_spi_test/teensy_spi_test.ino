//#include <interrupts.h>

uint8_t miso = 12;
uint8_t mosi = 11;
uint8_t sck = 13;
uint8_t cs = 15;
uint8_t spimode = 8;

uint8_t buffer[128];
uint8_t num_bits = 0;

void setup() {
  Serial.begin(115200);

  pinMode(cs, INPUT);
  pinMode(mosi, INPUT);
  pinMode(sck, INPUT);
  attachInterrupt(sck, sck_rising_interrupt, RISING);

  Serial.println("START!");
}

void sck_rising_interrupt() {
    Serial.print(digitalRead(mosi));
    buffer[num_bits / 8] |= digitalRead(mosi) << (7 - num_bits % 8);
    ++num_bits;
}


void loop() {
    while(digitalRead(cs) != LOW);
    while(digitalRead(cs) == LOW);
    int len = num_bits / 8;
    num_bits = 0;
    
    Serial.println();
    for(int i = 0; i < len; ++i) {
      Serial.print((uint8_t)buffer[i]);
      if(i != len - 1) Serial.print("\t");
    }
    Serial.println();
    
    for(int i = 0; i < len; ++i) {
      buffer[i] = 0;
    }
}
