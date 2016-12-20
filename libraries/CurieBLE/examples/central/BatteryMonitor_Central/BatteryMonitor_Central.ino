/*
 * Copyright (c) 2016 Intel Corporation.  All rights reserved.
 * See the bottom of this file for the license terms.
 */

/*
 * Sketch: BatteryMonitor_Central.ino
 *
 * Description:
 *   This sketch will receive the notifications and output the received
 *   data in the serial monitor.  It also illustrates using a non-typed
 *   characteristic.
 *
 * Notes:
 *
 *  - Set the baud rate to 115200 on the serial monitor to accomodate
 *    the speed of constant data updates
 *  - Expected Peripheral name: BatteryMonitorSketch
 *  - Expected Peripheral Characteristic: 2A19
 *  - Expected Peripheral sketch: BatteryMonitor_Notification.ino
 *
 */

#include <CurieBLE.h>

#define LED_PIN   13


void setup()
{
  // This is set to higher baud rate because accelerometer data changes very quickly
  Serial.begin(9600);    // initialize serial communication
  while (!Serial);
  pinMode(LED_PIN, OUTPUT);    // initialize the LED on pin 13 to indicate when a central is connected

  /* Now activate the BLE device.  It will start continuously transmitting BLE
    advertising packets and will be visible to remote BLE central devices
    until it receives a new connection */
  BLE.begin();
  Serial.print("My Address is: ");
  Serial.println(BLE.address());

  BLE.scanForName("BatteryMonitorSketch");
  Serial.println("Scan Started");
}


void loop()
{
  BLEDevice peripheral = BLE.available();

  if (peripheral)
  {
    Serial.print("Found ");
    Serial.print(peripheral.address());
    Serial.print(" '");
    Serial.print(peripheral.localName());
    Serial.print("' ");
    Serial.print(peripheral.advertisedServiceUuid());
    Serial.println();
    if ( peripheral.localName() == "BatteryMonitorSketch") {
      BLE.stopScan();
      processNotification(peripheral);

      delay (1000);
    }

    // central connected to peripheral

    //delay (4000);
    // BLE.scan();//("BatteryMonitorSketch");
  }
}

void processNotification(BLEDevice peripheral) {
  if (peripheral.connect()) {
    Serial.print("Connected: ");
    Serial.println(peripheral.address());
    // light pin to indicate connection
    digitalWrite(LED_PIN, HIGH);
  } else {
    Serial.println("Failed to connect!");
    return;
  }

  // discover peripheral attributes
  Serial.println("Discovering attributes ...");
  if (peripheral.discoverAttributes()) {
    Serial.println("Attributes discovered");
  } else {
    Serial.println("Attribute discovery failed!");
    peripheral.disconnect();
    digitalWrite(LED_PIN, LOW);
    return;
  }
  BLECharacteristic batteryLevelChar = peripheral.characteristic("2A19");
  if (!batteryLevelChar) {
    peripheral.disconnect();
    Serial.println("Peripheral does not have battery level characteristic!");
    digitalWrite(LED_PIN, LOW);
    delay(1000);
    return;
  }

  if (!batteryLevelChar.subscribe()) {
    Serial.println("subscription failed!");
    peripheral.disconnect();
    return;
  } else {
    Serial.println("Subscribed");
  }

  while (peripheral.connected())
  {
    if (batteryLevelChar.valueUpdated()) {

      printData(batteryLevelChar.value(), batteryLevelChar.valueLength());
      Serial.println("%");

    }

  }
  Serial.print("Disconnected");
  Serial.println(peripheral.address());
  digitalWrite(LED_PIN, LOW);
}


void printData(const unsigned char data[], int length) {
  for (int i = 0; i < length; i++) {
    unsigned char b = data[i];

    if (b < 16) {
      Serial.print("0");
    }

    Serial.print(b);
  }
}

/*
   Copyright (c) 2016 Intel Corporation.  All rights reserved.

   This library is free software; you can redistribute it and/or
   modify it under the terms of the GNU Lesser General Public
   License as published by the Free Software Foundation; either
   version 2.1 of the License, or (at your option) any later version.

   This library is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
   Lesser General Public License for more details.

   You should have received a copy of the GNU Lesser General Public
   License along with this library; if not, write to the Free Software
   Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
*/





