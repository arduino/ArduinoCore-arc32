#include <Power.h>
#include "CurieIMU.h"


void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  CurieIMU.begin();
  CurieIMU.attachInterrupt(wakeup);
  CurieIMU.setDetectionThreshold(CURIE_IMU_MOTION, 20);      // 100mg
  CurieIMU.setDetectionDuration(CURIE_IMU_MOTION, 10);       // trigger times of consecutive slope data points
  CurieIMU.interrupts(CURIE_IMU_MOTION);
  
}

void loop() {
  PM.sleep();
  digitalWrite(LED_BUILTIN, HIGH);
  delay(3000);
  digitalWrite(LED_BUILTIN, LOW);
}

void wakeup()
{
  PM.wakeFromDoze();
  // This function will be called once on device wakeup
  // You can do some little operations here (like changing variables which will be used in the loop)
  // Remember to avoid calling delay() and long running functions since this functions executes in interrupt context
}
