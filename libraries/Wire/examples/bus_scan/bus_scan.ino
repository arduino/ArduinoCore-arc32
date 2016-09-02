/*
  Sketch: I2CBusScan.ino

  This sketch demonstrates the usage of the Curie Wire Library.
  It scan the I2C bus to find slave devices

  You can see the found I2C slave device shown in serial output
  with the following message:
     address:xxx  found

  created by Intel
  Modified 18 May 2016

  This example code is in the public domain.
*/


#include <Wire.h>

byte startAddress = 1; // skip reserved address from 0 to 7
byte endAddress   = 127;

void I2CBusScan(byte startAddress, byte endAddress)
{
  byte retval;
  char temp[64];
  for( byte address = startAddress; address <= endAddress; address++ ) {
    Wire.beginTransmission(address);
    retval = Wire.endTransmission();
    sprintf(temp, "address: %-4d%-5s", address, (retval == 0 || retval == 3) ? "found" : "");
    Serial.print(temp);
    Serial.print((address % 4) ? '\t' : '\n');
  }
}

void setup()
{
  // Initialize pin 13 as an output - onboard LED.
  pinMode(13, OUTPUT);

  // join i2c bus (address optional for master)
  Wire.begin();

  Serial.begin(115200); // initialize Serial communication
  while(!Serial) ;      // wait for serial port to connect.
}

boolean toggle = false;          // state of the LED
void loop()
{
  toggle = !toggle;
  digitalWrite(13, toggle);
  delay(5000);

  Serial.print("Start I2C Bus Scan from ");
  Serial.print(startAddress);
  Serial.print(" to ");
  Serial.print(endAddress);
  Serial.println(".....");

  I2CBusScan( startAddress, endAddress);

  Serial.println("\ndone");
}
