/*
 * Copyright (c) 2017 Intel Corporation.  All rights reserved.
 * See the bottom of this file for the license terms.
 */

/*
 * Sketch: profileatcentral.ino
 *
 * Description:
 *   This is a BLE Central sketch that demostrates the setting of the
 *   BLE descriptor.
 *
 * Notes:
 *   - This sketch is based on the Arduino BLE Peripheral LED example.
 *     Please refer to licensing agreement at the bottom of this file.
 */

#include "CurieBLE.h"
#include <errno.h>
// LED pin
#define LED_PIN   13
BLEService          ledService("19b10100e8f2537e4f6cd104768a1214");

BLECharacteristic   switchCharacteristic("19b10101e8f2537e4f6cd104768a1214", BLERead | BLEWrite | BLENotify, 1);

BLEDescriptor       switchDescriptor("2901", "switch");

void setup() {
    Serial.begin(115200);
    
    // set LED pin to output mode
    pinMode(LED_PIN, OUTPUT);

    // begin initialization
    BLE.begin();
    Serial.println(BLE.address());
    ledService.addCharacteristic(switchCharacteristic);
    switchCharacteristic.addDescriptor(switchDescriptor);
    BLE.addService(ledService);
	
    BLE.scanForName("LED");
}


void loop() {
    BLEDevice peripheral = BLE.available();
    if (peripheral) 
    {
        Serial.println(peripheral.address());
		
        BLE.stopScan();
		
		if (peripheral.connect())
		{
			Serial.print("Connected: ");
			Serial.println(peripheral.address());
			while (peripheral.connected())
			{
				delay (1000);
			}
		}
		else
		{
			Serial.println("Failed to connect!");
		}
        delay (4000);
        BLE.scanForName("LED");
    }
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


