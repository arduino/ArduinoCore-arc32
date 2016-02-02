/*
 * EEPROM Write
 *
 * Stores values read from analog input 0 into the EEPROM.
 * These values will stay in the EEPROM when the board is
 * turned off and may be retrieved later by another sketch.
 * 01/05/2016 - Modified for Arduino 101 - Dino Tinitigan <dino.tinitigan@intel.com>
 */

#include <CurieEEPROM.h>

/** the current address in the EEPROM (i.e. which byte we're going to write to next) **/
int addr = 0;

void setup() {
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  //use write for the first half of the EEPROM area
  Serial.println("Writing with write()");
  for(int i = 0; i < EEPROM.length()/8; i++)
  {
    unsigned long  val = analogRead(0);
    Serial.print("Addr:\t");
    Serial.print(addr);
    Serial.print("\tWriting: ");
    Serial.println(val);
    EEPROM.write(addr, val); 
    addr +=4; //increment address by 4 since we are using DWORDs
    delay(100);
  }
  
  //use write8 for the second half of the EEPROM area
  Serial.println("Writing with write8()");
  for(int i = EEPROM.length()/2; i < EEPROM.length(); i++)
  {
    byte val8 = analogRead(0)/4;
    Serial.print("Addr:\t");
    Serial.print(addr);
    Serial.print("\tWriting: ");
    Serial.println(val8);
    EEPROM.write(addr, val8);
    addr++;
    delay(100);
  }
  
  Serial.println("done writing");
}

void loop() {

}