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

  for(int i = 0; i < 512; i++)
  {
    unsigned long  val = analogRead(0);
    Serial.print("Addr:\t");
    Serial.print(addr);
    Serial.print("\tWriting: ");
    Serial.println(val);
    EEPROM.write(addr, val); 
    addr++;
    delay(100);
  }
    
  Serial.println("done writing");
}

void loop() {

}