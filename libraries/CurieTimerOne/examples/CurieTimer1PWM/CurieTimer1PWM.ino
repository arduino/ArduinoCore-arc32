#include "CurieTimerOne.h"

void setup() {
  // Setup a PWM signal on pin 13, onboard LED, with a 25% duty cycle
  // of a 1 second period (1000000 usec), as follow (please note the
  // decimal point to indicate double):

  CurieTimerOne.pwmStart(13, 25.0, 1000000);

  // Or, use discrete number range, 0-1023, to define the duty period,
  // 255 is 24.9%, as follow:

  // CurieTimerOne.pwmStart(13, 255, 1000000);
}

void loop() {
  // put your main code here, to run repeatedly:

  delay( 10000 );
}
