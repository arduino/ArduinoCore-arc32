/*
  Arduino BLE Central scan callback example
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

#include <ArduinoBLE.h>

char addr_buf[BT_ADDR_STR_LEN];
char name_buf[BLE_MAX_ADV_SIZE];
char uuid_buf[70];

void setup() {
  Serial.begin(9600);

  // initialize the BLE hardware
  BLE.begin();

  Serial.println("BLE Central scan callback");

  // set the discovered event handle
  BLE.setEventHandler(BLEDiscovered, bleCentralDiscoverHandler);

  // start scanning for peripherals with duplicates
  BLE.startScanningWithDuplicates();
}

void loop() {
  // poll the central for events
  BLE.poll();
}

void bleCentralDiscoverHandler(BLEDevice peripheral) {
  // discovered a peripheral
  Serial.println("Discovered a peripheral");
  Serial.println("-----------------------");

  peripheral.address(addr_buf);

  // print address
  Serial.println("Address: " + String(addr_buf));

  // print the local name, if present
  if (peripheral.hasLocalName()) {
    peripheral.localName(name_buf);
    Serial.println("Local Name: " + String(name_buf));
  }

    // print the advertised service UUID's, if present
    if (peripheral.hasAdvertisedServiceUuid()) {
    Serial.print("Service UUID's: ");
    for (int i = 0; i < peripheral.advertisedServiceUuidCount(); i++) {
      peripheral.advertisedServiceUuid(i, uuid_buf);
      Serial.print(String(uuid_buf) + " ");
    }
    Serial.println();
  }

  // print the RSSI
  Serial.print("RSSI: ");
  Serial.println(peripheral.rssi());

  Serial.println();
}
