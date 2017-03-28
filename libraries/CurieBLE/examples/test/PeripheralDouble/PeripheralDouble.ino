/*
 * Copyright (c) 2016 Intel Corporation.  All rights reserved.
 * See the bottom of this file for the license terms.
 */

/*
 * Sketch: PeripheralDouble.ino
 *
 * Description:
 *   This is a Peripheral sketch that has a Characteristic of double.
 *   It demonstrates the usage of double in a BLE exchange.
 *
 * Notes:
 *
 *  - Peripheral Characteristic:  19b20001e8f2537e4f6cd104768a1214
 */

#include <CurieBLE.h>

// LED pin
#define LED_PIN   13

// create service
BLEService               dataService("19b20000e8f2537e4f6cd104768a1214");

// create data characteristic
BLEDoubleCharacteristic    doubleCharacteristic("19b20001e8f2537e4f6cd104768a1214", BLERead | BLEWrite);

void setup() {
    Serial.begin(9600);

    // set LED pin to output mode
    pinMode(LED_PIN, OUTPUT);

    // begin initialization
    BLE.begin();
    Serial.println(BLE.address());

    // set advertised local name and service UUID
    BLE.setLocalName("DataTest");

    dataService.addCharacteristic(doubleCharacteristic);

    // add service and characteristic
    BLE.addService(dataService);

    BLE.advertise();

    Serial.println(F("BLE Test Double Peripheral"));
}

void loop() {
  BLEDevice central = BLE.central();
  double test_value = 0.1;

  if (central) {
    // central connected to peripheral
    Serial.print(F("Connected to central: "));
    Serial.println(central.address());

    while (central.connected())
    {
        // central still connected to peripheral
        doubleCharacteristic.writeDouble(test_value);
        test_value += 0.3;
        delay(2000);
    }

    // central disconnected
    Serial.print(F("Disconnected from central: "));
    Serial.println(central.address());
  }
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


