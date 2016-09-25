/*
 * Copyright (c) 2016 Intel Corporation.  All rights reserved.
 * See the bottom of this file for the license terms.
 */

#include <CurieBLE.h>

/*
  This sketch example works with IMUBleNotification.ino

  IMUBleNotification.ino will send notifications to this central sketch.
  This sketch will receive the notifications and output the received data in the serial monitor.
  It also illustrates using a non-typed characteristic.
  Set the baud rate to 115200 on the serial monitor to accomodate the speed of constant data updates from IMU subsystem.
*/

#define MAX_IMU_RECORD 1

ble_conn_param_t conn_param = {30.0,    // minimum interval in ms 7.5 - 4000
                               50.0,    // maximum interval in ms 7.5 -
                               0,       // latency 
                               4000     // timeout in ms 100 - 32000ms
                               };
// define a structure that will serve as buffer for holding IMU data

typedef struct {
    int index;
    unsigned int slot[3];
} imuFrameType;

imuFrameType imuBuf[MAX_IMU_RECORD];
BLECentral bleCentral;      // BLE Central Device (the board you're programming)

// create a new service with a custom 128-bit UUID
BLEService bleImuService("F7580001-153E-D4F6-F26D-43D8D98EEB13");

// standard 128-bit characteristic UUID with Read and Notify properties that allow
// remote clients to get notifications when this characteristic changes
// We have a third parameter which is the size of imyBuffer. This is because it is a non-typed characteristic.
// If we are only writing to this characteristic we can set this buffer to 512 bytes,
// but because of the limitation of the Nordic FW, please do not set this to more than 128 if you intend to read it.
// MAX_IMU_RECORD value is 1 so we are safe
BLECharacteristic bleImuChar("F7580003-153E-D4F6-F26D-43D8D98EEB13",
                             BLERead | BLENotify, sizeof(imuBuf));

// function prototype for function that determines if the advertising data is found
bool adv_found(uint8_t type,
               const uint8_t *dataPtr,
               uint8_t data_len,
               const bt_addr_le_t *addrPtr);

void setup()
{
    // initialize serial communication and set the baud rate to 115200 bps.
    // This is set to higher baud rate because accelerometer data changes very quickly
	  Serial.begin(115200);
    // wait for the Serial port to connect. Open the Serial Monitor to continue executing the sketch
    // If you don't care to see text messages sent to the Serial Monitor during board initialization, 
    // remove or comment out the next line
    while(!Serial) ;
    // initialize the LED on pin 13 to indicate when a central is connected
    pinMode(LED_BUILTIN, OUTPUT);
    
    // set the event handeler function for the bleImuChar characteristic
    bleImuChar.setEventHandler(BLEWritten, bleImuCharacteristicWritten);
    
    bleCentral.addAttribute(bleImuService); // Add the BLE IMU service
    bleCentral.addAttribute(bleImuChar);    // Add the BLE IMU characteristic

    // Setup callback whenever a Peripheral advertising data is found
    bleCentral.setAdvertiseHandler(adv_found);
    bleCentral.setEventHandler(BLEConnected, ble_connected);
    bleCentral.setEventHandler(BLEDisconnected, ble_disconnected);
    
    /* Now activate the BLE device.  It will start continuously transmitting BLE
     advertising packets and will be visible to remote BLE central devices
     until it receives a new connection */
    bleCentral.begin();
}


void loop()
{
    // we put a  2 second delay
    // Even though this looks empty, since we setup 2 callbacks  by setting the advertising handler adv_found
    // and event handler for BLEConnected, we basically are lsitening for advertising data and connected events.
    
    delay(2000);
}

void ble_connected(BLEHelper &role)
{
    // since we are a central device we create a BLEPeripheralHelper peripheral
    BLEPeripheralHelper *peripheral = bleCentral.getPeerPeripheralBLE(role);
    Serial.print("Connected to peripheral ");
    // print MAC Address of peripheral device
    Serial.println(peripheral->address());
    // the BLE central device is connected to peripheral so turn the on-board LED on 
    digitalWrite(LED_BUILTIN, HIGH);
    
    // Start discovery the profiles in peripheral device
    peripheral->discover();
}

void ble_disconnected(BLEHelper &peripheral)
{
    Serial.print("Disconnected form peripheral ");
    // print MAC Address of peripheral device
    Serial.println(peripheral.address());
    // if peripheral disconnects from the BLE central device turn the on-board LED off
    digitalWrite(LED_BUILTIN, LOW);
}

void bleImuCharacteristicWritten(BLEHelper& peripheral, BLECharacteristic& characteristic)
{
    // Peripheral wrote new value to characteristic by Notification/Indication
    // We have to use pointers because we are NOT using a type characteristic
    // In other examples our charcteristics are typed so we not have to use pointers and can access the value directly
    // The parent non typde characteristic class, the value method gives a pointer to the  characteristic value 
	
    const unsigned char *cvalue = characteristic.value();
    const imuFrameType *value = (const imuFrameType *)cvalue;
    Serial.print("\r\nCharacteristic event, written: ");
    Serial.print(value->index);
    Serial.print("\t");
    Serial.print(value->slot[0]);
    Serial.print("\t");
    Serial.print(value->slot[1]);
    Serial.print("\t");
    Serial.println(value->slot[2]);
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
    // Please see https://www.bluetooth.org/en-us/specification/assigned-numbers/generic-access-profile
    //  To decode the data the central device cares.
    //  This example use UUID as identity.
    switch (type)
    {
        case BT_DATA_UUID128_SOME:
        case BT_DATA_UUID128_ALL:
        {
            if (data_len % UUID_SIZE_128 != 0) 
            {
                Serial.println("AD malformed");
                return true;
            }
            for (i = 0; i < data_len; i += UUID_SIZE_128)
            {
                if (bleImuService.uuidCompare(dataPtr + i, UUID_SIZE_128) == false)
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
