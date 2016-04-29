/*
 *   Copyright (c) 2015 Intel Corporation.  All rights reserved.
 */

/*
   This sketch example demonstrates how the BMI160 accelerometer on the
   Intel(R) Curie(TM) module can be used as a Step Counter (pedometer)
*/

#include "CurieIMU.h"

/* To get an interrupt notification for every step detected,
    set stepEventsEnabeled to true. Otherwise, the main loop will
    poll for the current step count.

   By design, the step counter does not immediately update on every step detected.
   Please refer to Section 2.7 of the BMI160 IMU SensorData Sheet
   for more information on this feature.
*/
const int ledPin = 13;

boolean stepEventsEnabeled = true;   // whether you're polling or using events
long lastStepCount = 0;              // step count on previous polling check
boolean blinkState = false;          // state of the LED

void setup() {
  Serial.begin(9600);
  // pinMode(13, OUTPUT);
  // intialize the sensor:
  CurieIMU.begin();
  // turn on step detection mode:
  CurieIMU.setStepDetectionMode(CURIE_IMU_STEP_MODE_NORMAL);
  // enable step counting:
  CurieIMU.setStepCountEnabled(true);

  if (stepEventsEnabeled) {
    // attach the eventCallback function as the
    // step event handler:
    CurieIMU.attachInterrupt(eventCallback);
    CurieIMU.interrupts(CURIE_IMU_STEP);  // turn on step detection

    Serial.println("IMU initialisation complete, waiting for events...");
  }
}

void loop() {
  /* Instead of using step detection event notifications,
     we can check the step count periodically */
  if (!stepEventsEnabeled) {
    updateStepCount();
  }
  digitalWrite(13, blinkState);
  blinkState = !blinkState;
  delay(1000);
}

static void updateStepCount() {
  // get the step count:
  int stepCount = CurieIMU.getStepCount();

  // if the step count has changed, print it:
  if (stepCount != lastStepCount) {
    Serial.print("Step count: ");
    Serial.println(stepCount);
    // save the current count for comparison next check:
    lastStepCount = stepCount;
  }
}

static void eventCallback(void) {
  if (CurieIMU.stepsDetected())
    updateStepCount();
}
