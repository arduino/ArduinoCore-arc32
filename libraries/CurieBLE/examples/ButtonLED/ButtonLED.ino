/*
   Copyright (c) 2016 Intel Corporation.  All rights reserved.
   See the bottom of this file for the license terms.
*/

/*
  This example can work with phone BLE app.

  This examples needs a button connected similarly as described here https://www.arduino.cc/en/Tutorial/Button
  The only difference is that instead of connecting to pin 2, it connects to pin 4
  After the sketch starts connect to a BLE app on a phone and set notification to the Characteristic and you should see it update
  whenever the button is pressed. This sketch is not written to pair with any of the central examples.
*/

#include <CurieBLE.h>

const int ledPin = 13; // set ledPin to on-board LED
const int buttonPin = 4; // set buttonPin to digital pin 4

BLEPeripheral blePeripheral; // create peripheral instance
BLEService ledService("19B10010-E8F2-537E-4F6C-D104768A1214"); // create service with a 128-bit UUID (32 characters exclusive of dashes).
// Long UUID denote custom user created UUID


// create switch characteristic and allow remote device to read and write
BLECharCharacteristic ledCharacteristic("19B10011-E8F2-537E-4F6C-D104768A1214", BLERead | BLEWrite);
// create button characteristic and allow remote device to get notifications
BLECharCharacteristic buttonCharacteristic("19B10012-E8F2-537E-4F6C-D104768A1214", BLERead | BLENotify); // allows remote device to get notifications
// Note use of Typed Characteristics. These previous 2  characeristics are of the type char

void setup() {
  Serial.begin(9600);
  // wait for the Serial port to connect. Open the Serial Monitor to continue executing the sketch
  while (!Serial) {
    ;
  }
  pinMode(ledPin, OUTPUT); // use the LED on pin 13 as an output
  pinMode(buttonPin, INPUT); // use button pin 4 as an input

  // set the local name peripheral advertises
  blePeripheral.setLocalName("ButtonLED");
  // set the UUID for the service this peripheral advertises:
  blePeripheral.setAdvertisedServiceUuid(ledService.uuid());

  // add service and characteristics
  blePeripheral.addAttribute(ledService);
  blePeripheral.addAttribute(ledCharacteristic);
  blePeripheral.addAttribute(buttonCharacteristic);

  // set initial values for led and button characteristic
  ledCharacteristic.setValue(0);
  buttonCharacteristic.setValue(0);

  // advertise the service
  blePeripheral.begin();

  Serial.println("Bluetooth device active, waiting for connections...");
}

void loop() {
  // poll peripheral
  blePeripheral.poll();

  // read the current button pin state
  char buttonValue = digitalRead(buttonPin);

  // has the value changed since the last read
  boolean buttonChanged = (buttonCharacteristic.value() != buttonValue);

  if (buttonChanged) {
    // button state changed, update characteristics
    ledCharacteristic.setValue(buttonValue);
    buttonCharacteristic.setValue(buttonValue);
  }

  if (ledCharacteristic.written() || buttonChanged) {
    // update LED, either central has written to characteristic or button state has changed.
    // If you are using a phone or a BLE  central device that is aware of this characteristic,
    // writing a value of 0x40 for example will be interpreted as written
    if (ledCharacteristic.value()) {
      Serial.println("LED on");
      digitalWrite(ledPin, HIGH);
    } else {
      // If central writes a 0 value (0x00) then it is interpreted as no value and turns off the LED
      Serial.println("LED off");
      digitalWrite(ledPin, LOW);
    }
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
