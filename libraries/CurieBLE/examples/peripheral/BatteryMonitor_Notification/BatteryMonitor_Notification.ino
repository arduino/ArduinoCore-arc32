/*
   Copyright (c) 2016 Intel Corporation.  All rights reserved.
   See the bottom of this file for the license terms.
*/

#include <CurieBLE.h>

/*
  This sketch can work with BatteryMonitor_Central.

  You can also use an android or IOS app that supports notifications.
  This sketch example partially implements the standard Bluetooth Low-Energy Battery service
  and connection interval paramater update.
  For more information: https://developer.bluetooth.org/gatt/services/Pages/ServicesHome.aspx
*/



BLEService batteryService("180F"); // BLE Battery Service

// BLE Battery Level Characteristic"
BLEUnsignedCharCharacteristic batteryLevelChar("2A19", BLERead | BLENotify);   // standard 16-bit characteristic UUID  defined in the URL above
                                                                               // remote clients will be able to get notifications if this characteristic changes

int oldBatteryLevel = 0;  // last battery level reading from analog input
long previousMillis = 0;  // last time the battery level was checked, in ms

void setup() {
  BLE.begin();
  Serial.begin(9600);    // initialize serial communication
  pinMode(13, OUTPUT);   // initialize the LED on pin 13 to indicate when a central is connected

  /* Set a local name for the BLE device
     This name will appear in advertising packets
     and can be used by remote devices to identify this BLE device
     The name can be changed but maybe be truncated based on space left in advertisement packet
     If you want to make this work with the BatteryMonitor_Central sketch, do not modufy the name.
  */
  BLE.setLocalName("BatteryMonitorSketch");
  BLE.setAdvertisedServiceUuid(batteryService.uuid());  // add the service UUID
  BLE.addService(batteryService);   // Add the BLE Battery service
  batteryService.addCharacteristic(batteryLevelChar); // add the battery level characteristic
  batteryLevelChar.setValue(oldBatteryLevel);   // initial value for this characteristic

  /* Now activate the BLE device.  It will start continuously transmitting BLE
     advertising packets and will be visible to remote BLE central devices
     until it receives a new connection
  */

  BLE.advertise();
  Serial.println("Bluetooth device active, waiting for connections...");
}

void loop() {
  // listen for BLE peripherals to connect:
  BLEDevice central = BLE.central();

  // if a central is connected to peripheral:
  if (central) {
    Serial.print("Connected to central: ");
    // print the central's MAC address:
    Serial.println(central.address());
    // turn on the LED to indicate the connection:
    digitalWrite(13, HIGH);

    // check the battery level every 200ms
    // as long as the central is still connected:
    while (central.connected()) {
      long currentMillis = millis();
      // if 200ms have passed, check the battery level:
      if (currentMillis - previousMillis >= 200) {
        previousMillis = currentMillis;
        updateBatteryLevel();

        static unsigned short count = 0;
        count++;
        // update the connection interval
        if (count % 5 == 0) {
          delay(1000);
          updateIntervalParams(central);
        }
      }
    }
    // when the central disconnects, turn off the LED:
    digitalWrite(13, LOW);
    Serial.print("Disconnected from central: ");
    Serial.println(central.address());
  }
}

void updateBatteryLevel() {
  /* Read the current voltage level on the A0 analog input pin.
     This is used here to simulate the charge level of a battery.
  */
  int battery = analogRead(A0);
  int batteryLevel = map(battery, 0, 1023, 0, 100);

  if (batteryLevel != oldBatteryLevel) {      // if the battery level has changed
    Serial.print("Battery Level % is now: "); // print it
    Serial.println(batteryLevel);
    batteryLevelChar.writeUnsignedChar(batteryLevel);  // and update the battery level characteristic
    oldBatteryLevel = batteryLevel;           // save the level for next comparison
  }
}

void updateIntervalParams(BLEDevice central) {
  // read and update the connection interval that peer central device
  static unsigned short interval = 0x60;
  ble_conn_param_t m_conn_param;
  // Get connection interval that peer central device wanted
  //central.getConnParams(m_conn_param);
  Serial.print("min interval = " );
  Serial.println(m_conn_param.interval_min );
  Serial.print("max interval = " );
  Serial.println(m_conn_param.interval_max );
  Serial.print("latency = " );
  Serial.println(m_conn_param.latency );
  Serial.print("timeout = " );
  Serial.println(m_conn_param.timeout );

  //Update connection interval
  Serial.println("set Connection Interval");
  central.setConnectionInterval(interval, interval);

  interval++;
  if (interval < 0x06)
    interval = 0x06;
  if (interval > 0x100)
    interval = 0x06;
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
