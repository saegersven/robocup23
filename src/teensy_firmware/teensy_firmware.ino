#include "defines.h"
#define DEBUG 1

// only print when in debug mode:
#if DEBUG == 1
#define debug(x) Serial.print(x)
#define debugln(x) Serial.println(x)
#else
#define debug(x)
#define debugln(x)
#endif

void setup() {
  pinMode(6, OUTPUT);
}

void loop() {
  analogWrite(6, 255);
}
