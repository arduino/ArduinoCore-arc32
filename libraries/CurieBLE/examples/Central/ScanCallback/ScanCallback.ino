/*
 * Copyright (c) 2016 Intel Corporation.  All rights reserved.
 * See the bottom of this file for the license terms.
 */

/*
 * Sketch: ScanCallback.ino
 *
 * Description:
 *   This is a Central Sketch that scan for Peripherals without
 *   performing connection.  It displays the Adv info for each
 *   Peripheral scanned.  Notice that this sketch makes use of
 *   the Event Handler, a call back routine, to display the
 *   info onto the serial output.
 *
 * Notes:
 *
 *   - It is highly recommended not to use the Event Handler to
 *     dump information to the serial monitor.  Please note
 *     that Event Handler executes in interrupt context and it
 *     is expected to perform short execution task.  It is due
 *     to the fact that the entire system timing is impact by the
 *     execution time of the Event Handler.  Accessing the serial
 *     monitor is relatively time consuming, it is highly desirable
 *     to perform that in the background.
 *
 */

#include <CurieBLE.h>

void setup() {
  Serial.begin(9600);

  // initialize the BLE hardware
  BLE.begin();

  Serial.println("BLE Central scan callback");

  // set the discovered event handle
  BLE.setEventHandler(BLEDiscovered, bleCentralDiscoverHandler);

  // start scanning for peripherals with duplicates
  BLE.scan(true);
}

void loop() {
  // poll the central for events
  BLE.poll();
}

void bleCentralDiscoverHandler(BLEDevice peripheral) {
  // discovered a peripheral
  Serial.println("Discovered a peripheral");
  Serial.println("-----------------------");

  // print address
  Serial.print("Address: ");
  Serial.println(peripheral.address());

  // print the local name, if present
  if (peripheral.hasLocalName()) {
    Serial.print("Local Name: ");
    Serial.println(peripheral.localName());
  }

  // print the advertised service UUID's, if present
    if (peripheral.hasAdvertisedServiceUuid()) {
    Serial.print("Service UUID's: ");
    for (int i = 0; i < peripheral.advertisedServiceUuidCount(); i++) {
      Serial.print(peripheral.advertisedServiceUuid(i));
      Serial.print(" ");
    }
    Serial.println();
  }

  // print the RSSI
  Serial.print("RSSI: ");
  Serial.println(peripheral.rssi());

  Serial.println();
}



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

