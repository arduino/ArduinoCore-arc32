/*
  Arduino BLE Peripheral LED callback example
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

// Import libraries
#include <CurieBLE.h>

// LED pin
#define LED_PIN   13

// create service
BLEService              ledService("19b10000e8f2537e4f6cd104768a1214");

// create switch characteristic
BLECharCharacteristic   switchCharacteristic("19b10001e8f2537e4f6cd104768a1214", BLERead | BLEWrite);

void setup() {
  Serial.begin(9600);

  // set LED pin to output mode
  pinMode(LED_PIN, OUTPUT);

  // begin initialization
  BLE.begin();

  // set advertised local name and service UUID
  BLE.setLocalName("LED");
  BLE.setAdvertisedServiceUuid(ledService.uuid());

  ledService.addCharacteristic(switchCharacteristic);

  // add service
  BLE.addService(ledService);

  // assign event handlers for connected, disconnected to peripheral
  BLE.setEventHandler(BLEConnected, bleDeviceConnectHandler);
  BLE.setEventHandler(BLEDisconnected, bleDeviceDisconnectHandler);

  // assign event handlers for characteristic
  switchCharacteristic.setEventHandler(BLEWritten, switchCharacteristicWritten);

  BLE.advertise();

  Serial.println(F("BLE LED Peripheral"));
}

void loop() {
  // poll peripheral
  BLE.poll();
}

void bleDeviceConnectHandler(BLEDevice central) {
  // central connected event handler
  Serial.print(F("Connected event, central: "));
  Serial.println(central.address());
}

void bleDeviceDisconnectHandler(BLEDevice central) {
  // central disconnected event handler
  Serial.print(F("Disconnected event, central: "));
  Serial.println(central.address());
}

void switchCharacteristicWritten(BLEDevice central, BLECharacteristic characteristic) {
  // central wrote new value to characteristic, update LED
  Serial.print(F("Characteristic event, writen: "));

  if (switchCharacteristic.value()) {
    Serial.println(F("LED on"));
    digitalWrite(LED_PIN, HIGH);
  } else {
    Serial.println(F("LED off"));
    digitalWrite(LED_PIN, LOW);
  }
}
