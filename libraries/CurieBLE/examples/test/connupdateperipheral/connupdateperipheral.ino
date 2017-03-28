/*
 * Copyright (c) 2017 Intel Corporation.  All rights reserved.
 * See the bottom of this file for the license terms.
 */

/*
 * Sketch: connupdateperipheral.ino
 *
 * Description:
 *     This is a Peripheral sketch that is based on the Arduino LED
 *   control sketch.  The purpose of this sketch is to exercise
 *   the changing of Connection Interval as a Peripheral.  The Central
 *   writes to the Characteristic to turn the LED on and off similar
 *   to the original sketch.  However, when the LED is turned off,
 *   this sketch will change the connection interval to a new
 *   value and, thus, forcing a re-connection.
 *
 * Notes:
 *
 *   - This sketch is based on the Arduino BLE Peripheral LED example.
 *     Please refer to licensing info at the bottom of this file.
 */

#include <CurieBLE.h>

// LED pin
#define LED_PIN   13

// create service
BLEService               ledService("19b10000e8f2537e4f6cd104768a1214");

// create switch characteristic
BLECharCharacteristic    switchCharacteristic("19b10001e8f2537e4f6cd104768a1214", BLERead | BLEWrite);

BLEDescriptor            switchDescriptor("2901", "switch");


void bleCentralConnectionParameterUpdateHandler(BLEDevice central) {
  Serial1.println("Updated connection parameter");
  Serial1.println("-----------------------");

  // print address
  Serial1.print("Address: ");
  Serial1.println(central.address());

  Serial1.print("Interval: ");
  Serial1.println(central.getConnectionInterval());
  Serial1.print("Timeout: ");
  Serial1.println(central.getConnectionTimeout());
  Serial1.print("Latency: ");
  Serial1.println(central.getConnectionLatency());

  //Serial1.println();
}

void setup() {
  Serial.begin(9600);
  Serial1.begin(115200);

  // set LED pin to output mode
  pinMode(LED_PIN, OUTPUT);

  // begin initialization
  BLE.begin();
  Serial1.println(BLE.address());

  // set advertised local name and service UUID
  BLE.setLocalName("LED");
  BLE.setAdvertisedServiceUuid(ledService.uuid());

  switchCharacteristic.addDescriptor(switchDescriptor);
  ledService.addCharacteristic(switchCharacteristic);

  // add service and characteristic
  BLE.addService(ledService);
  
  //Register callbacks
  BLE.setEventHandler(BLEConParamUpdate, bleCentralConnectionParameterUpdateHandler);

  BLE.advertise();

  Serial1.println(F("BLE LED Peripheral"));
}

void loop() {
  BLEDevice central = BLE.central();

  if (central) {
    // central connected to peripheral
    Serial1.print(F("Connected to central: "));
    Serial1.println(central.address());
    static int connection_interval = 15;

    while (central.connected()) {
      // central still connected to peripheral
      if (switchCharacteristic.written()) {
        // central wrote new value to characteristic, update LED
        if (switchCharacteristic.value()) {
          Serial1.println(F("LED on"));
          digitalWrite(LED_PIN, HIGH);
          connection_interval++;
        } else {
          Serial1.println(F("LED off"));
          digitalWrite(LED_PIN, LOW);
          delay(100);
          // The peripheral update the connection interval
          //  If want central update the connection interval
          //  comment the below line
          central.setConnectionInterval(connection_interval, connection_interval);
        }
      }
    }

    // central disconnected
    Serial1.print(F("Disconnected from central: "));
    Serial1.println(central.address());
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

