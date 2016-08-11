// Wire Master Reader
// by Nicholas Zambetti <http://www.zambetti.com>

// Demonstrates use of the Wire library
// Reads data from an I2C/TWI slave device
// Refer to the "Wire Slave Sender" example for use with this

// Created 29 March 2006

// This example code is in the public domain.


#include <Wire2.h>

void setup()
{
  Serial.begin(115200);  // start serial for output
  while(Serial);

  Wire2.begin();        // join i2c bus (address optional for master)
}

void loop()
{

  int ret = Wire2.requestFrom(8, 20, 1);    // request 20 bytes from slave device #8
  if (ret == 0)
  {
    Serial.println("read from slave device failed");   
  }
    
  while (Wire2.available())   // slave may send less than requested
  {
    char c = Wire2.read(); // receive a byte as character
    Serial.println(c, HEX);
  }
  delay(1000);
}
