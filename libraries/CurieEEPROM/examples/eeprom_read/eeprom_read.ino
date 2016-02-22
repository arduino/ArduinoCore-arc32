/*
 * EEPROM Read
 *
 * Reads the value of each DWORD of the EEPROM and prints it
 * to the computer.
 * This example code is in the public domain.
 * 01/05/2016 - Modified for Arduino 101 - Dino Tinitigan <dino.tinitigan@intel.com>
 */

#include <CurieEEPROM.h>

// start reading from the first byte (address 0) of the EEPROM
int address = 0;
unsigned long value;

void setup() {
  // initialize serial and wait for port to open:
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
}

void loop() {
  // read a dword from the current address of the EEPROM
  value = EEPROM.read(address);

  Serial.print(address);
  Serial.print("\t");
  Serial.print(value, DEC);
  Serial.println();

  //increment address
  address++;
  if (address == EEPROM.length()) {
    address = 0;
  }

  delay(500);
}
