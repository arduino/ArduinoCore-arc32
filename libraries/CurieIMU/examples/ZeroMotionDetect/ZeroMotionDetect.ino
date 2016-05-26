/*
   Copyright (c) 2016 Intel Corporation.  All rights reserved.

   This library is free software; you can redistribute it and/or
   modify it under the terms of the GNU Lesser General Public
   License as published by the Free Software Foundation; either
   version 2.1 of the License, or (at your option) any later version.

   This library is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
   Lesser General Public License for more details.

   You should have received a copy of the GNU Lesser General Public
   License along with this library; if not, write to the Free Software
   Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA

*/

/*
   This sketch example demonstrates how the BMI160 accelerometer on the
   Intel(R) Curie(TM) module can be used to detect zero motion events
*/
#include "CurieIMU.h"

boolean blinkState = false;          // state of the LED
unsigned long loopTime = 0;          // get the time since program started
unsigned long interruptsTime = 0;    // get the time when zero motion event is detected
void setup() {
  Serial.begin(9600);
  while(!Serial);   // wait for the serial port to open

  /* Initialise the IMU */
  blinkState = CurieIMU.begin();
  CurieIMU.attachInterrupt(eventCallback);

  /* Enable Zero Motion Detection */
  CurieIMU.setDetectionThreshold(CURIE_IMU_ZERO_MOTION, 1500); // 1.5g=1500mg
  CurieIMU.setDetectionDuration(CURIE_IMU_ZERO_MOTION, 25);    // 25s
  CurieIMU.interrupts(CURIE_IMU_ZERO_MOTION);

  Serial.println("IMU initialisation complete, waiting for events...");
}

void loop() {
  //if zero motion is detected in 1500ms, LED will be turned up
  loopTime = millis();
  if(abs(loopTime -interruptsTime) < 1500)     
    blinkState = true;
  else
    blinkState = false;
  digitalWrite(13, blinkState);
}

static void eventCallback(void){
  if (CurieIMU.getInterruptStatus(CURIE_IMU_ZERO_MOTION)) {
    interruptsTime = millis(); 
    Serial.println("zero motion detected...");
  }  
}
