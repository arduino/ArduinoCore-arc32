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

#include <CurieBLE.h>

// LED pin
#define LED_PIN   13

// create service
BLEService               ledService("19b10000e8f2537e4f6cd104768a1214");

// create switch characteristic
BLECharCharacteristic    switchCharacteristic("19b10001e8f2537e4f6cd104768a1214", BLERead | BLEWrite);

BLEDescriptor            switchDescriptor("2901", "switch");

void setup() {
  Serial.begin(9600);

  // set LED pin to output mode
  pinMode(LED_PIN, OUTPUT);

  // begin initialization
  BLE.begin();
  Serial.println(BLE.address());

  // set advertised local name and service UUID
  BLE.setLocalName("LED");
  BLE.setAdvertisedServiceUuid(ledService.uuid());

  switchCharacteristic.addDescriptor(switchDescriptor);
  ledService.addCharacteristic(switchCharacteristic);

  // add service and characteristic
  BLE.addService(ledService);

  BLE.advertise();

  Serial.println(F("BLE LED Peripheral"));
}

void loop() {
  BLEDevice central = BLE.central();

  if (central) {
    // central connected to peripheral
    Serial.print(F("Connected to central: "));
    Serial.println(central.address());

    while (central.connected()) {
      // central still connected to peripheral
      if (switchCharacteristic.written()) {
        // central wrote new value to characteristic, update LED
        if (switchCharacteristic.value()) {
          Serial.println(F("LED on"));
          digitalWrite(LED_PIN, HIGH);
        } else {
          Serial.println(F("LED off"));
          digitalWrite(LED_PIN, LOW);
        }
      }
    }

    // central disconnected
    Serial.print(F("Disconnected from central: "));
    Serial.println(central.address());
  }
}
