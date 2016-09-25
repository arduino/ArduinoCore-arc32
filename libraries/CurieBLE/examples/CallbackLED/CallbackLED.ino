/*
 * Copyright (c) 2016 Intel Corporation.  All rights reserved.
 * See the bottom of this file for the license terms.
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

// create service with a 128-bit UUID (32 characters exclusive of dashes).
// Long UUID denote custom user created UUID.
BLEService ledService("19B10000-E8F2-537E-4F6C-D104768A1214");

// create switch characteristic with a custom 128-bit UUID with Read and Write properties
// to allow remote device to read and write this characteristic value
BLECharCharacteristic switchChar("19B10001-E8F2-537E-4F6C-D104768A1214", BLERead | BLEWrite);

void setup() {
  // initialize serial communication
  Serial.begin(9600);
  // wait for the serial port to connect. Open the Serial Monitor to continue executing the sketch
  // If you don't care to see text messages sent to the Serial Monitor during board initialization, 
  // remove or comment out the next line
  while(!Serial) ;
  // set the pin 13 of the on-board LED as output
  pinMode(ledPin, OUTPUT);

  // Set the local name for the BLE device.
  // This name will appear in advertising packets
  // and can be used by remote devices to identify this BLE device
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
  // set the initial value of switchChar characteristic
  switchChar.setValue(0);

  // start advertising ledService
  blePeripheral.begin();
  Serial.println("Bluetooth device active, waiting for connections...");
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

// In addtion to the BLEHelper& central parameter, we also have to have to BLECharacteristic& characteristic parameter

void switchCharacteristicWritten(BLEHelper& central, BLECharacteristic& characteristic) {
  // when central writes a new value to switchChar characteristic, update the LED state
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
