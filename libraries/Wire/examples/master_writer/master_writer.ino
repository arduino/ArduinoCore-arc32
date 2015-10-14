// Wire Master Writer
// by Nicholas Zambetti <http://www.zambetti.com>

// Demonstrates use of the Wire library
// Writes data to an I2C/TWI slave device
// Refer to the "Wire Slave Receiver" example for use with this

// Created 29 March 2006

// This example code is in the public domain.


#include <Wire.h>

void setup()
{
  Wire.begin(); // join i2c bus (address optional for master)
  Serial1.begin(9600);
}

byte x = 1;
byte rdata;
void loop()
{
  Wire.beginTransmission(8); // transmit to device #8
  //Wire.write("x is ");        // sends five bytes
  Wire.write(x);              // sends one byte
  int result = Wire.endTransmission();    // stop transmitting
  Serial1.println();
  Serial1.print("x =  ");
  Serial1.print(x);
  x++;
  delay(500);
}
