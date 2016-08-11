// Wire Master Writer
// by Nicholas Zambetti <http://www.zambetti.com>

// Demonstrates use of the Wire library
// Writes data to an I2C/TWI slave device
// Refer to the "Wire Slave Receiver" example for use with this

// Created 29 March 2006

// This example code is in the public domain.


#include <Wire2.h>

void setup()
{
  Serial.begin(115200);
  while(Serial);

  Wire2.begin(); // join i2c bus (address optional for master)
}

byte x = 1;
byte rdata;
void loop()
{
  Wire2.beginTransmission(8); // transmit to device #8
  Wire2.write(x);              // sends one byte
  int result = Wire2.endTransmission();    // stop transmitting
  if (result == 0)
  {
    Serial.print("x =  ");
    Serial.println(x);
  }
  else
  {
    Serial.print("transmit failed with error code ");
    Serial.println(result);
  }
  x++;
  delay(500);
}
