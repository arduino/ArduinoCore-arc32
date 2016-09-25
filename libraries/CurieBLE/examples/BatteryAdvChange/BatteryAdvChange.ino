/* Please see code copyright at the bottom of this example code */

/*
  This example can work with phone BLE app.

  This sketch illustrates how to change the advertising data so that it is visible but not
  connectable. Then after 10 seconds it changes to being connectable.
  This sketch example partially implements the standard Bluetooth Low-Energy Battery service.

  This sketch is not paired with a specific central example sketch,
  but to see how it works you need to use a BLE APP on your phone or central device
  and try connecting when it is either a connectable or not connectable state
  as displayed in the serial monitor.
*/

#include <CurieBLE.h>

BLEPeripheral blePeripheral;       // BLE Peripheral Device (the board you're programming)
BLEService batteryService("180F"); // BLE Battery Service

// BLE Battery Level Characteristic with standard 16-bit characteristic UUID.
// This characteristic has Read and Notify properties that allow remote clients 
// to get notifications when this characteristic changes
BLEUnsignedCharCharacteristic batteryLevelChar("2A19", BLERead | BLENotify);

int count = 0;
bool change_discover = false;

void setup() {
   // initialize serial communication
  Serial.begin(9600);
  // wait for the serial port to connect. Open the Serial Monitor to continue executing the sketch
  // If you don't care to see text messages sent to the Serial Monitor during board initialization, 
  // remove or comment out the next line
  while(!Serial) ;
  // initialize the LED on pin 13. When the BLE device will switch to connectable mode 
  // the on-board LED will be turned on, otherwise turn the on-board LED off 
  pinMode(LED_BUILTIN, OUTPUT);
  
  /* Set a local name for the BLE device
     This name will appear in advertising packets
     and can be used by remote devices to identify this BLE device
     The name can be changed but maybe be truncated based on space left in advertising packets */
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
    Serial.println("Stop Advertising and wait for 10 seconds. Device should be invisible");
    // Some central devices (phones included) may cache previous scan informations.
    // Restart your central device and it should not see this peripheral once stopAdvertising() is called
    blePeripheral.stopAdvertising();
    delay(10000);

    if (change_discover)
    {

      // Using the method setConnectable() we specify that it is now in NON connectable mode
      // The loop is for 10 seconds. Your central device may timeout later than that
      // and may eventually connect when we set it back to connectable mode below
      blePeripheral.setConnectable(false);
      Serial.println("In Non Connectable mode");
      // turn the on-board LED off
      digitalWrite(LED_BUILTIN, LOW);
    }
    else
    {

      // Switch to connectable mode by calling the setConnectable() method
      blePeripheral.setConnectable(true);
      Serial.println("In Connectable mode");
      // turn the on-board LED on
      digitalWrite(LED_BUILTIN, HIGH);
    }
    Serial.println("Start Advertising...");
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

