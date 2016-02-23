//
// Sketch: Timer1Interrupt.ino
//
//   This sketch demonstrates the usage of the ARC Timer-1.  It
// uses timer-1 to blink the onboard LED, pin 13, at different
// intervals (speed).
//

#include "CurieTimerOne.h"

// Uncomment the following statement to enable logging on serial port.
// #define SERIAL_PORT_LOG_ENABLE 1

const unsigned int oneSecInUsec = 1000000;  // A second in mirco second unit.
unsigned int toggle = 0;


void timedBlinkIsr()
{
  digitalWrite(13, toggle ? HIGH : LOW);
  toggle = (toggle + 1) & 0x01;
}

void setup() {

#ifdef SERIAL_PORT_LOG_ENABLE
  Serial.begin(115200);
  while(!Serial);
#endif

  // Initialize pin 13 as an output - onboard LED.
  pinMode(13, OUTPUT);
}

void loop() {
  unsigned int i, time = oneSecInUsec;

  CurieTimerOne.start(time, &timedBlinkIsr);

  for(i=0; i < 4; i++, time >>= 1)
  {

#ifdef SERIAL_PORT_LOG_ENABLE
    Serial.print("The blink period: ");
    Serial.println(time);
#endif

    delay(10000);  // 10 seconds

#ifdef SERIAL_PORT_LOG_ENABLE
    Serial.print("Total number of ticks in 10 seconds: ");
    Serial.println(CurieTimerOne.rdRstTickCount());
    Serial.println("----");
#endif

    CurieTimerOne.restart(time);
  }
}

