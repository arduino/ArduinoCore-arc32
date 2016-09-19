/*
   Copyright (c) 2016 Intel Corporation.  All rights reserved.
   See the bottom of this file for the license terms.
*/

/*
  This example can work with LEDCentral.

  You should see the LED blink on and off.
  This example demonstrates the use of Callback or event Handlers responding to events.
  BLEConnected, BLEDisconnected and BLEWritten are events.
  To test interactively, use a Phone app like nrf Controller (Android) or Light Blue (iOS).
  Connect to BLE device named LEDCB and explore characteristic with UUID 19B10001-E8F2-537E-4F6C-D104768A1214.
  Writing a byte value such as 0x40 should turn on the LED.
  Writing a byte value of 0x00 should turn off the LED.
*/

#include <CurieBLE.h>

const int ledPin = 13; // set ledPin to use on-board LED
BLEPeripheral blePeripheral; // create peripheral instance
BLECentralHelper *bleCentral1 = NULL; // peer central device

BLEService ledService("19B10000-E8F2-537E-4F6C-D104768A1214"); // create service with a 128-bit UUID (32 characters exclusive of dashes).
// Long UUID denote custom user created UUID

// create switch characteristic and allow remote device to read and write
BLECharCharacteristic switchChar("19B10001-E8F2-537E-4F6C-D104768A1214", BLERead | BLEWrite);

void setup() {
  Serial.begin(9600);
  // wait for the Serial port to connect. Open the Serial Monitor to continue executing the sketch
  while (!Serial) {
    ;
  }
  pinMode(ledPin, OUTPUT); // use the LED on pin 13 as an output

  // set the local name peripheral advertises
  blePeripheral.setLocalName("LEDCB");
  // set the UUID for the service this peripheral advertises
  blePeripheral.setAdvertisedServiceUuid(ledService.uuid());

  // add service and characteristic
  blePeripheral.addAttribute(ledService);
  blePeripheral.addAttribute(switchChar);

  // assign event handlers for connected, disconnected to peripheral
  blePeripheral.setEventHandler(BLEConnected, blePeripheralConnectHandler);
  blePeripheral.setEventHandler(BLEDisconnected, blePeripheralDisconnectHandler);

  // assign event handlers for characteristic
  switchChar.setEventHandler(BLEWritten, switchCharacteristicWritten);
  // set an initial value for the characteristic
  switchChar.setValue(0);

  // advertise the service
  blePeripheral.begin();
  Serial.println(("Bluetooth device active, waiting for connections..."));
}

void loop() {
  // poll peripheral
  blePeripheral.poll();
}

// The function parameter (BLEHelper& central) is for peripheral devices
// This enable us to have access to the central's data like its bluetooth address

void blePeripheralConnectHandler(BLEHelper& central) {
  // central connected event handler
  bleCentral1 = blePeripheral.getPeerCentralBLE(central);
  Serial.print("Connected event, central: ");
  Serial.println(bleCentral1->address());
}

void blePeripheralDisconnectHandler(BLEHelper& central) {
  // central disconnected event handler
  Serial.print("Disconnected event, central: ");
  Serial.println(central.address());
}

// In addtion to the BLECentral& central parameter, we also have to have to BLECharacteristic& characteristic parameter

void switchCharacteristicWritten(BLEHelper& central, BLECharacteristic& characteristic) {
  // central wrote new value to characteristic, update LED
  Serial.print("Characteristic event, written: ");

  if (switchChar.value()) {
    Serial.println("LED on");
    digitalWrite(ledPin, HIGH);
  } else {
    Serial.println("LED off");
    digitalWrite(ledPin, LOW);
  }
}

/*
  Copyright (c) 2016 Intel Corporation. All rights reserved.

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
  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-
  1301 USA
*/
