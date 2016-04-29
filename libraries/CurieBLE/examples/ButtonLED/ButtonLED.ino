/*
 *  Copyright (c) 2015 Intel Corporation. All rights reserved.
 */
#include <CurieBLE.h>

const int ledPin = 13; // set ledPin to on-board LED
const int buttonPin = 4; // set buttonPin to digital pin 4

BLEPeripheral blePeripheral; // create peripheral instance
BLEService ledService("19B10010-E8F2-537E-4F6C-D104768A1214"); // create service


// create switch characteristic and allow remote device to read and write
BLECharCharacteristic ledCharacteristic("19B10011-E8F2-537E-4F6C-D104768A1214", BLERead | BLEWrite);
// create button characteristic and allow remote device to get notifications
BLECharCharacteristic buttonCharacteristic("19B10012-E8F2-537E-4F6C-D104768A1214", BLERead | BLENotify); // allows remote device to get notifications

void setup() {
  Serial.begin(9600);
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
    // update LED, either central has written to characteristic or button state has changed
    if (ledCharacteristic.value()) {
      Serial.println("LED on");
      digitalWrite(ledPin, HIGH);
    } else {
      Serial.println("LED off");
      digitalWrite(ledPin, LOW);
    }
  }
}
