/*
 * Copyright (c) 2016 Intel Corporation.  All rights reserved.
 * See the bottom of this file for the license terms.
 */
 
 /*
  This example can work with LEDCentral

  This example is similar to CallbackLED example in functionality.
  It does not use callbacks. In the loop it interogates the connection state with central.
  Checks if the characteristic is written and turns the LED on or off accordingly.
  To test interactively, use a phone app like nrf Controller (Android) or Light Blue (iOS).
  Connect to BLE device named LED and explore characteristic with UUID 19B10001-E8F2-537E-4F6C-D104768A1214.
  Writing a byte value such as 0x40 should turn on the LED.
  Writing a byte value of 0x00 should turn off the LED.
 */
 
#include <CurieBLE.h>

BLEPeripheral blePeripheral;  // BLE Peripheral Device (the board you're programming)

// BLE LED Service - custom 128-bit UUID
BLEService ledService("19B10000-E8F2-537E-4F6C-D104768A1214");

// BLE LED Switch Characteristic - custom 128-bit UUID
// This characteristic has Read and Write properties that allow
// remote clients to read and write the characteristic value
BLEUnsignedCharCharacteristic switchCharacteristic("19B10001-E8F2-537E-4F6C-D104768A1214", BLERead | BLEWrite);

const int ledPin = 13; // pin to use for the LED

void setup() {
  // initialize serial communication
  Serial.begin(9600);
  // wait for the serial port to connect. Open the Serial Monitor to continue executing the sketch
  // If you don't care to see text messages sent to the Serial Monitor during board initialization, 
  // remove or comment out the next line
  while(!Serial) ;

  // set LED pin to output mode
  pinMode(ledPin, OUTPUT);

  // set the local name of this BLE peripheral device
  // This name will appear in advertising packets
  // and can be used by remote devices to identify this BLE device
  blePeripheral.setLocalName("LED");
  // set the UUID of the ledService that is advertised by the BLE peripheral device
  blePeripheral.setAdvertisedServiceUuid(ledService.uuid());

  // add service and characteristic:
  blePeripheral.addAttribute(ledService);
  blePeripheral.addAttribute(switchCharacteristic);

  // set the initial value for the characteristic:
  switchCharacteristic.setValue(0);

  // begin advertising BLE ledService:
  blePeripheral.begin();

  Serial.println("BLE LED Peripheral");
}

void loop() {
  // listen for BLE peripherals to connect:
  BLECentralHelper central = blePeripheral.central();

  // if a central is connected to peripheral:
  if (central) {
    Serial.print("Connected to central: ");
    // print the central's MAC address:
    Serial.println(central.address());

    // while the central is still connected to peripheral:
    while (central.connected()) {
      // if the remote device wrote to the characteristic,
      // use the value to control the LED:
      if (switchCharacteristic.written()) {
        if (switchCharacteristic.value()) {   // any value other than 0
          Serial.println("LED on");
          digitalWrite(ledPin, HIGH);         // will turn the LED on
        } else {                              // a 0 value
          Serial.println("LED off");
          digitalWrite(ledPin, LOW);          // will turn the LED off
        }
      }
    }

    // when the central disconnects, print it out:
    Serial.print("Disconnected from central: ");
    Serial.println(central.address());
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
