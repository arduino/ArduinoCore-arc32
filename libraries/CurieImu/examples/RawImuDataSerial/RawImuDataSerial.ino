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

  Genuino 101 CurieIMU Orientation Visualiser
  Hardware Required:
  *Arduino/Genuino 101
  
  http://arduino.cc/en/Tutorial/Genuino101CurieIMUOrientationVisualiser
*/

#include "CurieImu.h"

/*  Library can be found at:
    http://www.x-io.co.uk/open-source-imu-and-ahrs-algorithms/
    Note: change .c file extension to .cpp
*/
#include "MadgwickAHRS.h"

int ax, ay, az;
int gx, gy, gz;
float yaw;
float pitch;
float roll;
int factor = 800; // variable by which to divide gyroscope values, used to control sensitivity
// note that an increased baud rate requires an increase in value of factor

int calibrateOffsets = 0; // int to determine whether calibration takes place or not


void setup() {
  // initialize Serial communication
  Serial.begin(9600);

  // initialize device
  CurieImu.initialize();

  // verify connection
  if (!CurieImu.testConnection()) {
    Serial.println("CurieImu connection failed");
  }


  if (calibrateOffsets == 1) {
    // use the code below to calibrate accel/gyro offset values
    Serial.println("Internal sensor offsets BEFORE calibration...");
    Serial.print(CurieImu.getXAccelOffset()); Serial.print("\t");
    Serial.print(CurieImu.getYAccelOffset()); Serial.print("\t");
    Serial.print(CurieImu.getZAccelOffset()); Serial.print("\t");
    Serial.print(CurieImu.getXGyroOffset()); Serial.print("\t");
    Serial.print(CurieImu.getYGyroOffset()); Serial.print("\t");
    Serial.print(CurieImu.getZGyroOffset()); Serial.print("\t");
    Serial.println("");

    // To manually configure offset compensation values, use the following methods instead of the autoCalibrate...() methods below
    //    CurieImu.setXGyroOffset(220);
    //    CurieImu.setYGyroOffset(76);
    //    CurieImu.setZGyroOffset(-85);
    //    CurieImu.setXAccelOffset(-76);
    //    CurieImu.setYAccelOffset(--235);
    //    CurieImu.setZAccelOffset(168);

    // IMU device must be resting in a horizontal position for the following calibration procedure to work correctly!

    // Serial.print("Starting Gyroscope calibration...");
    CurieImu.autoCalibrateGyroOffset();
    // Serial.println(" Done");
    // Serial.print("Starting Acceleration calibration...");
    CurieImu.autoCalibrateXAccelOffset(0);
    CurieImu.autoCalibrateYAccelOffset(0);
    CurieImu.autoCalibrateZAccelOffset(1);
    // Serial.println(" Done");

    Serial.println("Internal sensor offsets AFTER calibration...");
    Serial.print(CurieImu.getXAccelOffset()); Serial.print("\t");
    Serial.print(CurieImu.getYAccelOffset()); Serial.print("\t");
    Serial.print(CurieImu.getZAccelOffset()); Serial.print("\t");
    Serial.print(CurieImu.getXGyroOffset()); Serial.print("\t");
    Serial.print(CurieImu.getYGyroOffset()); Serial.print("\t");
    Serial.print(CurieImu.getZGyroOffset()); Serial.print("\t");
    Serial.println("");
    Serial.println("Enabling Gyroscope/Acceleration offset compensation");
    CurieImu.setGyroOffsetEnabled(true);
    CurieImu.setAccelOffsetEnabled(true);
  }
}

void loop() {
  // read raw accel/gyro measurements from device
  ax = CurieImu.getAccelerationX();
  ay = CurieImu.getAccelerationY();
  az = CurieImu.getAccelerationZ();
  gx = CurieImu.getRotationX();
  gy = CurieImu.getRotationY();
  gz = CurieImu.getRotationZ();

  // use function from MagdwickAHRS.h to return quaternions
  MadgwickAHRSupdateIMU(gx / factor, gy / factor, gz / factor, ax, ay, az);

  // equations to find yaw roll and pitch from quaternions
  yaw = atan2(2 * q1 * q2 - 2 * q0 * q3, 2 * q0 * q0 + 2 * q1 * q1 - 1);
  roll = -1 * asin(2 * q1 * q3 + 2 * q0 * q2);
  pitch = atan2(2 * q2 * q3 - 2 * q0 * q1, 2 * q0 * q0 + 2 * q3 * q3 - 1);

  // print gyro and accel values for debugging only, comment out when running Processing
  /*
    Serial.print(ax); Serial.print("\t");
    Serial.print(ay); Serial.print("\t");
    Serial.print(az); Serial.print("\t");
    Serial.print(gx); Serial.print("\t");
    Serial.print(gy); Serial.print("\t");
    Serial.print(gz); Serial.print("\t");
    Serial.println("");
  */

  if (Serial.available() > 0) {
    int val = Serial.read();
    if (val == 's') { // if incoming serial is "s"
      Serial.print(yaw);
      Serial.print(","); // print comma so values can be parsed
      Serial.print(pitch);
      Serial.print(","); // print comma so values can be parsed
      Serial.println(roll);
    }
  }
}

/* Processing code for this example

// This example code is in the public domain.

import processing.serial.*;
Serial myPort;

int newLine = 13; // new line character in ASCII
float yaw;
float pitch;
float roll;
String message;
String [] ypr = new String [3];

void setup()
{
  size(600, 500, P3D);
  // Set my serial port to same as Arduino, baud rate 9600
  myPort = new Serial(this, "/dev/cu.usbmodemfa131", 9600); 
  textSize(16); // set text size
  textMode(SHAPE); // set text mode to shape
}

void draw()
{
  serialEvent();  // read and parse incoming serial message
  background(255); // set background to white

  translate(width/2, height/2); // set position to centre
  
  pushMatrix(); // begin object
  
  rotateX(pitch); // RotateX pitch value
  rotateY(-yaw); // yaw
  rotateZ(-roll); // roll
  
  drawArduino(); // function to draw rough Arduino shape
  
  popMatrix(); // end of object

  // Print values to console
  print(pitch);
  print("\t");
  print(roll); 
  print("\t");
  print(-yaw);   
  println("\t");

  myPort.write("s"); // write an "s" to receive more data from Arduino
} 

void serialEvent()
{
  message = myPort.readStringUntil(newLine); // read from port until new line (ASCII code 13)
  if (message != null) {
    ypr = split(message, ","); // split message by commas and store in String array 
    yaw = float(ypr[0]); // convert to float yaw
    pitch = float(ypr[1]); // convert to float pitch
    roll = float(ypr[2]); // convert to float roll
  }
}
void drawArduino() {
  // function contains shape(s) that are rotated with the IMU
  stroke(0, 90, 90); // set outline colour to darker teal
  fill(0, 130, 130); // set fill colour to lighter teal
  box(300, 10, 200); // draw Arduino board base shape

  stroke(0); // set outline colour to black
  fill(80); // set fill colour to dark grey

  translate(60, -10, 90); // set position to edge of Arduino box
  box(170, 20, 10); // draw pin header as box

  translate(-20, 0, -180); // set position to other edge of Arduino box
  box(210, 20, 10); // draw other pin header as box
}

*/
