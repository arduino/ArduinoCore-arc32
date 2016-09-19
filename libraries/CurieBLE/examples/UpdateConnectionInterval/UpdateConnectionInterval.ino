/*
   Copyright (c) 2016 Intel Corporation.  All rights reserved.
   See the bottom of this file for the license terms.
*/

#include <CurieBLE.h>

/*
    This example can work with BatteryMonitor.

    Show how to control and response the connection interval request.
*/

// set up connection params

ble_conn_param_t conn_param = {30.0,    // minimum interval in ms 7.5 - 4000
                               50.0,    // maximum interval in ms 7.5 -
                               0,       // latency
                               4000     // timeout in ms 100 - 32000ms
                              };

const int ledPin = 13; // set ledPin to use on-board LED
BLECentral bleCentral; // create central instance
BLEPeripheralHelper *blePeripheral1 = NULL; // peer peripheral device

BLEService batteryService("180F"); // create service with a 16-bit UUID
BLECharCharacteristic batteryLevelChar("2A19", BLERead | BLENotify); // create switch characteristic
//and allow remote device to read and notify

bool adv_found(uint8_t type,
               const uint8_t *dataPtr,
               uint8_t data_len,
               const bt_addr_le_t *addrPtr);

void setup()
{
  Serial.begin(9600);
  // wait for the Serial port to connect. Open the Serial Monitor to continue executing the sketch
  while (!Serial) {
    ;
  }

  // add service and characteristic
  bleCentral.addAttribute(batteryService);
  bleCentral.addAttribute(batteryLevelChar);

  // assign event handlers for connected, disconnected to central
  bleCentral.setEventHandler(BLEConnected, bleCentralConnectHandler);
  bleCentral.setEventHandler(BLEDisconnected, bleCentralDisconnectHandler);
  bleCentral.setEventHandler(BLEUpdateParam, bleCentralUpdateParam);

  // advertise the service
  bleCentral.setAdvertiseHandler(adv_found);

  // assign event handlers for characteristic
  batteryLevelChar.setEventHandler(BLEWritten, switchCharacteristicWritten);

  bleCentral.begin();
  Serial.println("Bluetooth device active, waiting for connections...");
}

void loop()
{
  static unsigned int counter = 0;
  static char ledstate = 0;
  delay(2000);
  if (blePeripheral1)
  {
    counter++;

    if (counter % 3)
    {
      batteryLevelChar.read(*blePeripheral1);
    }
    else
    {
      ledstate = !ledstate;
      batteryLevelChar.write(*blePeripheral1, ledstate);
    }
  }

}

void bleCentralConnectHandler(BLEHelper& peripheral)
{
  // peripheral connected event handler
  blePeripheral1 = bleCentral.getPeerPeripheralBLE(peripheral);
  Serial.print("Connected event, peripheral: ");
  Serial.println(peripheral.address());
  // Start discovery the profiles in peripheral device
  blePeripheral1->discover();
}

void bleCentralDisconnectHandler(BLEHelper& peripheral)
{
  // peripheral disconnected event handler
  blePeripheral1 = NULL;
  Serial.print("Disconnected event, peripheral: ");
  Serial.println(peripheral.address());
  bleCentral.startScan();
}

void bleCentralUpdateParam(BLEHelper& peripheral)
{
  // peripheral update the connection interval event handler
  Serial.print("UpdateParam event, peripheral: ");
  blePeripheral1 = bleCentral.getPeerPeripheralBLE(peripheral);;
  Serial.println(peripheral.address());

  // Get connection interval that peer peripheral device wanted
  ble_conn_param_t m_conn_param;
  blePeripheral1->getConnParams(m_conn_param);
  Serial.print("min interval = " );
  Serial.println(m_conn_param.interval_min );
  Serial.print("max interval = " );
  Serial.println(m_conn_param.interval_max );
  Serial.print("latency = " );
  Serial.println(m_conn_param.latency );
  Serial.print("timeout = " );
  Serial.println(m_conn_param.timeout );

  //Update the connection interval
  blePeripheral1->setConnectionInterval(m_conn_param.interval_min, m_conn_param.interval_max);
}

void switchCharacteristicWritten(BLEHelper& peripheral, BLECharacteristic& characteristic)
{
  // Read response/Notification wrote new value to characteristic, update LED
  Serial.print("Characteristic event, notify: ");

  int battery = batteryLevelChar.value();
  if (battery)
  {
    Serial.print("Battery Level % is now: "); // print it
    Serial.println(battery);
    delay(100);

    Serial.println("LED on");
    digitalWrite(ledPin, HIGH);
  }
  else
  {
    Serial.println("LED off");
    digitalWrite(ledPin, LOW);
  }
}

bool adv_found(uint8_t type,
               const uint8_t *dataPtr,
               uint8_t data_len,
               const bt_addr_le_t *addrPtr)
{
  int i;

  Serial.print("[AD]:");
  Serial.print(type);
  Serial.print(" data_len ");
  Serial.println(data_len);

  switch (type)
  {
    case BT_DATA_UUID16_SOME:
    case BT_DATA_UUID16_ALL:
      {
        if (data_len % UUID_SIZE_16 != 0)
        {
          Serial.println("AD malformed");
          return true;
        }
        for (i = 0; i < data_len; i += UUID_SIZE_16)
        {
          if (batteryService.uuidCompare(dataPtr + i, UUID_SIZE_16) == false)
          {
            continue;
          }

          // Accept the advertisement
          if (!bleCentral.stopScan())
          {
            Serial.println("Stop LE scan failed");
            continue;
          }
          Serial.println("Connecting");
          // Connect to peripheral
          bleCentral.connect(addrPtr, &conn_param);
          return false;
        }
      }
  }

  return true;
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
