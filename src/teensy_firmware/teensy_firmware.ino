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
  init();
}

void loop() {
  m(255, 255, 2000);
  m(-255, -255, 2000); 
  m(255, -255, 2000);
  m(-255, 255, 2000);
  m(150, 150, 2000);
  m(-150, -150, 2000);
  m(150, -150, 2000);
  m(-150, 150, 2000);
}
