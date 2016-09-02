#include <CurieTimerOne.h>

const int blinkPin = 13;

void setup(void)
{
  CurieTimerOne.initialize(50000);
  Serial.begin(9600); // initialize Serial communication
  while(!Serial) ;    // wait for serial port to connect.
}

void loop(void)
{
  unsigned int range;

  Serial.println("PWM blink: low -> high duty cycle");

  for (float dutyCycle = 5.0; dutyCycle <= 100.0; dutyCycle++) {
    range = (dutyCycle / 100) * 1023;
    CurieTimerOne.pwm(blinkPin, range);
    delay(50);
  }

  Serial.println("PWM blink: high -> low duty cycle");

  for (float dutyCycle = 100.0; dutyCycle > 5.0; dutyCycle--) {
    range = (dutyCycle / 100) * 1023;
    CurieTimerOne.setPwmDuty(blinkPin, range);
    delay(50);
  }
}
