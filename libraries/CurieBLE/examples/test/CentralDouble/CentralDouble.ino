/*
 * Copyright (c) 2016 Intel Corporation.  All rights reserved.
 * See the bottom of this file for the license terms.
 */

/*
 *  Sketch:  CentralDouble.ino.
 *
 *  Description:
 *    This is a simple BLE sketch that initiates the
 *  Arduino platform as a Central.  It reads a double value from
 *  a connected Peripheral.  The sketch exercises:  Scanning for
 *  a specific Peripheral, connecting to it, discover its Attributes,
 *  and exercise a specific Characteristic to read a double value
 *  from the Peripheral.
 *
 *  Notes:
 *  Expected Peripheral name: DataTest
 *  Expected Peripheral Characteristic: 19b20001e8f2537e4f6cd104768a1214
 *  Expected Characteristic read value:  double.
 */

#include "CurieBLE.h"


// LED pin
#define LED_PIN   13

void setup() {
    Serial.begin(9600);
    
    // set LED pin to output mode
    pinMode(LED_PIN, OUTPUT);

    // begin initialization
    BLE.begin();
    Serial.println(BLE.address());
    
    BLE.scanForName("DataTest");
}

void loop() {
    BLEDevice peripheral = BLE.available();
    if (peripheral) 
    {
        Serial.println(peripheral.address());
        BLE.stopScan();
        // central connected to peripheral
        controlLogic(peripheral);
        BLE.scanForName("DataTest");
    }
}


void controlLogic(BLEDevice &peripheral)
{
    // connect to the peripheral
    Serial.print("Connecting ... ");
    Serial.println(peripheral.address());

    if (peripheral.connect())
    {
        Serial.print("Connected: ");
        Serial.println(peripheral.address());
    }
    else
    {
        Serial.println("Failed to connect!");
        return;
    }
    
    if (peripheral.discoverAttributes() == false)
    {
        Serial.println("Discover failed, Disconnecting...");
        peripheral.disconnect();
        return;
    }
    
    BLECharacteristic doubleCharacteristic = peripheral.characteristic("19b20001e8f2537e4f6cd104768a1214");
    
    if (!doubleCharacteristic)
    {
        peripheral.disconnect();
        Serial.println("Peripheral does not have test double characteristic!");
        delay(5000);
        return;
    } 
    doubleCharacteristic.subscribe();
    
    while (peripheral.connected())
    {
        doubleCharacteristic.read();
        delay(1000);
        if (doubleCharacteristic.valueUpdated())
        {
            Serial.print("Double characteristic value: ");
            Serial.println(doubleCharacteristic.doubleValue());
        }
        delay(1000);
    }
    Serial.print("Disconnected");
    Serial.println(peripheral.address());
}



/*
  Arduino BLE Peripheral Double read/write test example
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


