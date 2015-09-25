/*
 * Copyright (c) 2015 Intel Corporation.  All rights reserved.
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.

 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.

 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
 *
 */

/*
 * This sketch example demonstrates how the BMI160 accelerometer on the
 * Intel(R) Curie(TM) module can be used as a Step Counter (pedometer)
 */

#include "CurieImu.h"

/* To get an interrupt notification for every step detected, uncomment the line below.
 * Note that, by design, the step counter does not immediately update on every step detected.
 * Please refer to Section 2.7 of the BMI160 Data Sheet for more information on this feature
 */
//#define ENABLE_STEP_DETECTION_EVENTS

uint16_t lastStepCount = 0;

static void updateStepCount()
{
    uint16_t stepCount = CurieImu.getStepCount();
    if (stepCount != lastStepCount) {
        Serial.print("Step count: "); Serial.println(stepCount);
        lastStepCount = stepCount;
    }
}

static void eventCallback(void)
{
    if (CurieImu.getIntStepStatus())
        updateStepCount();
}

void setup() {
    Serial.begin(115200);

    CurieImu.initialize();

    CurieImu.setStepDetectionMode(BMI160_STEP_MODE_NORMAL);
    CurieImu.setStepCountEnabled(true);

#ifdef ENABLE_STEP_DETECTION_EVENTS
    CurieImu.attachInterrupt(eventCallback);
    CurieImu.setIntStepEnabled(true);
    CurieImu.setIntEnabled(true);
#endif

    Serial.println("IMU initialisation complete, waiting for events...");
}

void loop() {
#ifndef ENABLE_STEP_DETECTION_EVENTS
    /* Instead of using step detection event notifications, 
     * we can check the step count periodically */
    updateStepCount();
    delay(1000);
#endif
}
