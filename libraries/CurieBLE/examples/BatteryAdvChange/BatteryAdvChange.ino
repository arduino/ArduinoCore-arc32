/* Please see code cpyright at the bottom of this example code */
/*
   This sketch illustrates how to change the advertising data so that it is visible but not
   connectable. Then after 10 seconds it changes to being connectable
   This sketch example partially implements the standard Bluetooth Low-Energy Battery service.
   
   This sketch is not paired with a specific central example sketch, 
   but to see how it works you need to use a BLE APP on your phone or central device 
    and try connecting when it is either a connectable or not connectable state 
    as displayed in the serial monitor.
*/

#include <CurieBLE.h>

BLEPeripheral blePeripheral;       // BLE Peripheral Device (the board you're programming)
BLEService batteryService("180F"); // BLE Battery Service
int count = 0;
// BLE Battery Level Characteristic"
BLEUnsignedCharCharacteristic batteryLevelChar("2A19",  // standard 16-bit characteristic UUID
    BLERead | BLENotify);     // remote clients will be able to
// get notifications if this characteristic changes

void setup() {
  Serial.begin(9600);    // initialize serial communication
  pinMode(13, OUTPUT);   // initialize the LED on pin 13 to indicate when a central is connected
  while (!Serial) {
    //wait for Serial to connect
  }
  /* Set a local name for the BLE device
     This name will appear in advertising packets
     and can be used by remote devices to identify this BLE device
     The name can be changed but maybe be truncated based on space left in advertisement packet */
  blePeripheral.setLocalName("BatteryAdvChangeSketch");
  blePeripheral.setAdvertisedServiceUuid(batteryService.uuid());  // add the service UUID
  blePeripheral.addAttribute(batteryService);   // Add the BLE Battery service
  blePeripheral.addAttribute(batteryLevelChar); // add the battery level characteristic

  /* Now activate the BLE device.  It will start continuously transmitting BLE
     advertising packets and will be visible to remote BLE central devices
     until it receives a new connection */

  blePeripheral.begin();
  Serial.println("Bluetooth device active, waiting for connections...");
  Serial.println("Starts in Connectable mode");
}

void loop() {
  // listen for BLE peripherals to connect:
  BLECentralHelper central = blePeripheral.central();
  // wait
  Serial.print(". ");
  if (count == 10) {
    Serial.print("\nReached count ");
    Serial.println(count);

  }
  delay (1000);
  count++;
  // Switch from Connectable to Non Connectable and vice versa
  if (count > 10 ) {
    static bool change_discover = false;
    Serial.println("Stop Adv and pausing for 10 seconds. Device should be invisible");
    // Some central devices (phones included) may cache previous scan inofrmation
    // restart your central and it should not see this peripheral once stopAdvertising() is called
    blePeripheral.stopAdvertising();
    delay(10000);

    if (change_discover)
    {

      // Using the function setConnectable we specify that it now NOT connectable
      // The loop is for 10 seconds. Your central device may timeout later than that
      // and may eventually connect when we set it back to connectable mode below
      blePeripheral.setConnectable(false);
      Serial.println("In Non Connectable mode");

    }
    else
    {

      //using the function setConnectable we specify that it now  connectable
      blePeripheral.setConnectable(true);
      Serial.println("In Connectable mode");
    }
    Serial.println("Start Adv");
    blePeripheral.startAdvertising();
    if (change_discover) {
      Serial.println("Adding 5 second delay in Non Connect Mode");
      delay(5000);
    }
    change_discover = !change_discover;
    count = 0;
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

