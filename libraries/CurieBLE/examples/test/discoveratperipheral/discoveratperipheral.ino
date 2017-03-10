/*
 * Copyright (c) 2016 Intel Corporation.  All rights reserved.
 * See the bottom of this file for the license terms.
 */

/*
 *  Sketch: discoveratperipheral.ino.
 *
 *  Description:
 *    This is a BLE Central sketch that looks for a particular
 *  Characteristic in a connected Peripheral to write to.  The
 *  Peripheral with the special Characteristic will blink its
 *  LED.
 *
 *  Notes:
 *    - This sketch is Arduino BLE Peripheral LED example.
 *      Please see licensing at the bottom of this file.
 *    - Expected Peripheral Characteristic: 19b10000e8f2537e4f6cd104768a1214
 */

#include <CurieBLE.h>

// LED pin
#define LED_PIN   13

// create service
BLEService               ledService("19b10000e8f2537e4f6cd104768a1214");

void setup() {
  Serial.begin(9600);
  
  // set LED pin to output mode
  pinMode(LED_PIN, OUTPUT);

  // begin initialization
  BLE.begin();
  Serial.println(BLE.address());

  // set advertised local name and service UUID
  BLE.setLocalName("LED");

  BLE.advertise();

  Serial.println(F("BLE LED Peripheral"));
}

void loop() {
  BLEDevice central = BLE.central();

  if (central) {
    // central connected to peripheral
    Serial.print(F("Connected to central: "));
    Serial.println(central.address());

    controlLed(central);
    // central disconnected
    Serial.print(F("Disconnected from central: "));
    Serial.println(central.address());
  }
}

void controlLed(BLEDevice &central)
{
    if (central.discoverAttributes() == false)
    {
        Serial.println("Discover failed, Disconnecting...");
        central.disconnect();
        return;
    }
    
    BLECharacteristic ledCharacteristic = central.characteristic("19b10101-e8f2-537e-4f6c-d104768a1214");
    
    if (!ledCharacteristic)
    {
        central.disconnect();
        //while(1)
        {
        Serial.println("Central does not have LED characteristic!");
        delay(5000);
        }
        return;
    }
 
    ledCharacteristic.subscribe();
      
    unsigned char ledstate = 0;

    while (central.connected())
    {
        if (ledstate == 1)
        {
            ledstate = 0;
        }
        else
        {
            ledstate = 1;
        }
        ledCharacteristic.write(&ledstate, sizeof(ledstate));
        delay(5000);
    }
    Serial.print("Disconnected");
    Serial.println(central.address());
}


/*
  Arduino BLE Peripheral LED example
  Copyright (c) 2016 Arduino LLC. All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301 USA
*/

