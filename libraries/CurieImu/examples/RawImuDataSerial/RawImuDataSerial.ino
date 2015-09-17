/*
===============================================
Example sketch for CurieImu library for Intel(R) Curie(TM) devices.
Copyright (c) 2015 Intel Corporation.  All rights reserved.

Based on I2C device class (I2Cdev) demonstration Arduino sketch for MPU6050
class by Jeff Rowberg: https://github.com/jrowberg/i2cdevlib

===============================================
I2Cdev device library code is placed under the MIT license
Copyright (c) 2011 Jeff Rowberg

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
===============================================
*/

#include "CurieImu.h"

int16_t ax, ay, az;
int16_t gx, gy, gz;

// uncomment "OUTPUT_READABLE_ACCELGYRO" if you want to see a tab-separated
// list of the accel X/Y/Z and then gyro X/Y/Z values in decimal. Easy to read,
// not so easy to parse, and slow(er) over UART.
#define OUTPUT_READABLE_ACCELGYRO

// uncomment "OUTPUT_BINARY_ACCELGYRO" to send all 6 axes of data as 16-bit
// binary, one right after the other. This is very fast (as fast as possible
// without compression or data loss), and easy to parse, but impossible to read
// for a human.
//#define OUTPUT_BINARY_ACCELGYRO

// uncomment "CALIBRATE_ACCELGRYO_OFFSETS" to perform auto-calibration of all 6 axes during start-up
// This requires the device to be resting in a horizontal position during the start-up phase
//#define CALIBRATE_ACCELGRYO_OFFSETS


#define LED_PIN 13
bool blinkState = false;

void setup() {
    // initialize Serial communication
    Serial.begin(115200);

    // initialize device
    Serial.println("Initializing IMU device...");
    CurieImu.initialize();

    // verify connection
    Serial.println("Testing device connections...");
    Serial.println(CurieImu.testConnection() ? "CurieImu connection successful" : "CurieImu connection failed");

#if CALIBRATE_ACCELGRYO_OFFSETS
// use the code below to calibrate accel/gyro offset values
    Serial.println("Internal sensor offsets BEFORE calibration...");
    Serial.print(CurieImu.getXAccelOffset()); Serial.print("\t"); // -76
    Serial.print(CurieImu.getYAccelOffset()); Serial.print("\t"); // -235
    Serial.print(CurieImu.getZAccelOffset()); Serial.print("\t"); // 168
    Serial.print(CurieImu.getXGyroOffset()); Serial.print("\t"); // 0
    Serial.print(CurieImu.getYGyroOffset()); Serial.print("\t"); // 0
    Serial.print(CurieImu.getZGyroOffset()); Serial.print("\t"); // 0
    Serial.println("");

// To manually configure offset compensation values, use the following methods instead of the autoCalibrate...() methods below
//    CurieImu.setXGyroOffset(220);
//    CurieImu.setYGyroOffset(76);
//    CurieImu.setZGyroOffset(-85);
//    CurieImu.setXAccelOffset(-76);
//    CurieImu.setYAccelOffset(--235);
//    CurieImu.setZAccelOffset(168);

    // IMU device must be resting in a horizontal position for the following calibration procedure to work correctly!
    Serial.print("Starting Gyroscope calibration...");
    CurieImu.autoCalibrateGyroOffset();
    Serial.println(" Done");
    Serial.print("Starting Acceleration calibration...");
    CurieImu.autoCalibrateXAccelOffset(0);
    CurieImu.autoCalibrateYAccelOffset(0);
    CurieImu.autoCalibrateZAccelOffset(1);
    Serial.println(" Done");

    Serial.println("Internal sensor offsets AFTER calibration...");
    Serial.print(CurieImu.getXAccelOffset()); Serial.print("\t"); // -76
    Serial.print(CurieImu.getYAccelOffset()); Serial.print("\t"); // -2359
    Serial.print(CurieImu.getZAccelOffset()); Serial.print("\t"); // 1688
    Serial.print(CurieImu.getXGyroOffset()); Serial.print("\t"); // 0
    Serial.print(CurieImu.getYGyroOffset()); Serial.print("\t"); // 0
    Serial.print(CurieImu.getZGyroOffset()); Serial.print("\t"); // 0
    Serial.println("");
    
    Serial.println("Enabling Gyroscope/Acceleration offset compensation");
    CurieImu.setGyroOffsetEnabled(true);
    CurieImu.setAccelOffsetEnabled(true);
#endif

    // configure Arduino LED for
    pinMode(LED_PIN, OUTPUT);
}

void loop() {
    // read raw accel/gyro measurements from device
    CurieImu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    // these methods (and a few others) are also available
    //CurieImu.getAcceleration(&ax, &ay, &az);
    //CurieImu.getRotation(&gx, &gy, &gz);

    #ifdef OUTPUT_READABLE_ACCELGYRO
        // display tab-separated accel/gyro x/y/z values
        Serial.print("a/g:\t");
        Serial.print(ax); Serial.print("\t");
        Serial.print(ay); Serial.print("\t");
        Serial.print(az); Serial.print("\t");
        Serial.print(gx); Serial.print("\t");
        Serial.print(gy); Serial.print("\t");
        Serial.println(gz);
    #endif

    #ifdef OUTPUT_BINARY_ACCELGYRO
        Serial.write((uint8_t)(ax >> 8)); Serial.write((uint8_t)(ax & 0xFF));
        Serial.write((uint8_t)(ay >> 8)); Serial.write((uint8_t)(ay & 0xFF));
        Serial.write((uint8_t)(az >> 8)); Serial.write((uint8_t)(az & 0xFF));
        Serial.write((uint8_t)(gx >> 8)); Serial.write((uint8_t)(gx & 0xFF));
        Serial.write((uint8_t)(gy >> 8)); Serial.write((uint8_t)(gy & 0xFF));
        Serial.write((uint8_t)(gz >> 8)); Serial.write((uint8_t)(gz & 0xFF));
    #endif

    // blink LED to indicate activity
    blinkState = !blinkState;
    digitalWrite(LED_PIN, blinkState);
}