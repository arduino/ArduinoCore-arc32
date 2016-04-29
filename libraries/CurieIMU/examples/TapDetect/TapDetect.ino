/*
 *   Copyright (c) 2015 Intel Corporation.  All rights reserved.
 */

/*
   This sketch example demonstrates how the BMI160 accelerometer on the
   Intel(R) Curie(TM) module can be used to detect tap events
*/

#include "CurieIMU.h"

void setup() {
  Serial.begin(9600);

  // Initialise the IMU
  CurieIMU.begin();
  CurieIMU.attachInterrupt(eventCallback);

  // Increase Accelerometer range to allow detection of stronger taps (< 4g)
  CurieIMU.setAccelerometerRange(4);

  // Reduce threshold to allow detection of weaker taps (>= 750mg)
  CurieIMU.setDetectionThreshold(CURIE_IMU_TAP, 750); // (750mg)

  // Set the time window for 2 taps to be registered as a double-tap (<= 250 milliseconds)
  CurieIMU.setDetectionDuration(CURIE_IMU_DOUBLE_TAP, 250);

  // Enable Double-Tap detection
  CurieIMU.interrupts(CURIE_IMU_DOUBLE_TAP);

  Serial.println("IMU initialisation complete, waiting for events...");
}

void loop() {
  // nothing happens in the loop because all the action happens
  // in the callback function.
}

static void eventCallback()
{
  if (CurieIMU.getInterruptStatus(CURIE_IMU_DOUBLE_TAP)) {
    if (CurieIMU.tapDetected(X_AXIS, NEGATIVE))
      Serial.println("Double Tap detected on negative X-axis");
    if (CurieIMU.tapDetected(X_AXIS, POSITIVE))
      Serial.println("Double Tap detected on positive X-axis");
    if (CurieIMU.tapDetected(Y_AXIS, NEGATIVE))
      Serial.println("Double Tap detected on negative Y-axis");
    if (CurieIMU.tapDetected(Y_AXIS, POSITIVE))
      Serial.println("Double Tap detected on positive Y-axis");
    if (CurieIMU.tapDetected(Z_AXIS, NEGATIVE))
      Serial.println("Double Tap detected on negative Z-axis");
    if (CurieIMU.tapDetected(Z_AXIS, POSITIVE))
      Serial.println("Double Tap detected on positive Z-axis");
  }
}
