// Wire Master Reader
// by Nicholas Zambetti <http://www.zambetti.com>

// Demonstrates use of the Wire library
// Reads data from an I2C/TWI slave device
// Refer to the "Wire Slave Sender" example for use with this

// Created 29 March 2006

// This example code is in the public domain.


#include <Wire.h>

void setup()
{
  Serial.begin(9600);  // start serial for output
  while(!Serial);
  Wire.begin();        // join i2c bus (address optional for master)
}

void loop()
{
  Wire.requestFrom(8, 6, true);    // request 6 bytes from slave device #8

  while (Wire.available())   // slave may send less than requested
  {
    char c = Wire.read(); // receive a byte as character
    Serial.print(c, HEX); // print the character
    Serial.println();
  }

  delay(500);
}
