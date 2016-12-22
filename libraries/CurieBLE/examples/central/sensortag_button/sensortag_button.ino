/*
  Arduino BLE Central SensorTag button example
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

const int NUM_OF_SERVICE = 10;

char *serviceUUIDArray[NUM_OF_SERVICE] =

  // These are the various services that are included in the CC2650 Sensor Tag
  // If you uncomment them you can see the various services
{ //"f000aa00-0451-4000-b000-000000000000",
  //    "f000aa20-0451-4000-b000-000000000000",
  //    "f000aa40-0451-4000-b000-000000000000",
  //    "f000aa70-0451-4000-b000-000000000000",
  //    "f000aa80-0451-4000-b000-000000000000",
  //    "f000aa64-0451-4000-b000-000000000000",
  //    "f000ac00-0451-4000-b000-000000000000",
  //    "f000ccc0-0451-4000-b000-000000000000",
  //    "f000ffc0-0451-4000-b000-000000000000",
  "0000ffe0-0000-1000-8000-00805f9b34fb"
};

void setup() {
  Serial.begin(9600);
  while (!Serial);

  // initialize the BLE hardware
  BLE.begin();

  Serial.println("BLE Central - SensorTag button");
  Serial.println("Make sure to turn on the device.");

  // start scanning for peripheral
  BLE.scan();
}

void loop() {
  // check if a peripheral has been discovered
  BLEDevice peripheral = BLE.available();

  if (peripheral) {
    // discovered a peripheral, print out address, local name, and advertised service
    Serial.print("Found ");
    Serial.print(peripheral.address());
    Serial.print(" '");
    Serial.print(peripheral.localName());
    Serial.print("' ");
    Serial.print(peripheral.advertisedServiceUuid());
    Serial.println();

    /*see if peripheral is a SensorTag
      The localName SensorTag is in the Scan Response data packet
      In this release we do not have the feature that gets the scan response data and hence
      the local name in the scan is blank
      We have to explicitly find the BLE mac address
      Please use another deviice like nrfConnect app to discover the Bluetooth Address
    */
    //if (peripheral.localName() == "SensorTag") {


    /******************************************************
    * ATTENTION:
    * Change to the mac address according to your device!
    * Use a central app that can display the BT MAC address
    * ******************************************************
    */
    
    if (peripheral.address() == "24:71:89:07:27:80")

    {
      // stop scanning
      BLE.stopScan();

      monitorSensorTagButtons(peripheral);

      // peripheral disconnected, start scanning again
      BLE.scan();
    }
  }
}

void monitorSensorTagButtons(BLEDevice peripheral)
{
  static bool getAllServices = true;
  static int serviceIndx = 0;

  // connect to the peripheral
  Serial.println("Connecting ...");
  if (peripheral.connect()) {
    Serial.println("Connected");
  } else {
    Serial.println("Failed to connect!");
    return;
  }

  if (getAllServices) {
    // discover peripheral attributes
    Serial.println("Discovering attributes ...");
    if (peripheral.discoverAttributes()) {
      Serial.println("Attributes discovered");
    } else {
      getAllServices = false;
      Serial.println("Attribute discovery failed.");
      peripheral.disconnect();
      return;
    }
  } else {
    int tmp = serviceIndx;
    Serial.print("Discovering Service: ");
    Serial.println(serviceUUIDArray[tmp]);
    if (++serviceIndx >= NUM_OF_SERVICE)
      serviceIndx = 0;
    if (peripheral.discoverAttributesByService(serviceUUIDArray[tmp]) == false) {
      Serial.println("Can't find the Service.");
      peripheral.disconnect();
      return;
    } else {
      Serial.println("Service discovered.");
    }
  }

  // retrieve the simple key characteristic
  BLECharacteristic simpleKeyCharacteristic = peripheral.characteristic("0000ffe1-0000-1000-8000-00805f9b34fb");

  // subscribe to the simple key characteristic
  Serial.println("Subscribing to simple key characteristic ...");
  if (!simpleKeyCharacteristic) {
    Serial.println("no simple key characteristic found!");
    peripheral.disconnect();
    return;
  } else if (!simpleKeyCharacteristic.canSubscribe()) {
    Serial.println("simple key characteristic is not subscribable!");
    peripheral.disconnect();
    return;
  } else if (!simpleKeyCharacteristic.subscribe()) {
    Serial.println("subscription failed!");
    peripheral.disconnect();
    return;
  } else {
    Serial.println("Subscribed");
    Serial.println("Press the right and left buttons on your Sensor Tag.");
  }

  while (peripheral.connected()) {
    // while the peripheral is connected

    // check if the value of the simple key characteristic has been updated
    if (simpleKeyCharacteristic.valueUpdated()) {
      // yes, get the value, characteristic is 1 byte so use char value
      int value = simpleKeyCharacteristic.charValue();

      if (value & 0x01) {
        // first bit corresponds to the right button
        Serial.println("Right button pressed");
      }

      if (value & 0x02) {
        // second bit corresponds to the left button
        Serial.println("Left button pressed");
      }
    }
  }

}
