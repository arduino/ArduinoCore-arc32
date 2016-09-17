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

byte startAddress = 1;
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
  pinMode(LED_BUILTIN, OUTPUT);

  // join i2c bus (address optional for master)
  Wire.begin();

  // initialize Serial communication
  Serial.begin(115200);
  // wait for the Serial port to connect. Open the Serial Monitor to continue executing the sketch
  while(!Serial);
}

boolean toggle = false;          // state of the LED
void loop()
{
  toggle = !toggle;
  digitalWrite(LED_BUILTIN, toggle);
  delay(5000);

  Serial.print("Start I2C Bus Scan from ");
  Serial.print(startAddress);
  Serial.print(" to ");
  Serial.print(endAddress);
  Serial.println(".....");

  I2CBusScan(startAddress, endAddress);

  Serial.println("\ndone");
}
