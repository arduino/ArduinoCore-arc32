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

#include <CurieBLE.h>

/*
   This sketch example partially implements the standard Bluetooth Low-Energy Battery service.
   For more information: https://developer.bluetooth.org/gatt/services/Pages/ServicesHome.aspx
*/

#define MAX_IMU_RECORD 1

struct bt_le_conn_param conn_param = {0x18, 0x28, 0, 400};
typedef struct {
    int index;
    unsigned int slot[3];
} imuFrameType;

imuFrameType imuBuf[MAX_IMU_RECORD];
BLECentral bleCentral;      // BLE Central Device (the board you're programming)

BLEService bleImuService("F7580001-153E-D4F6-F26D-43D8D98EEB13");
BLECharacteristic bleImuChar("F7580003-153E-D4F6-F26D-43D8D98EEB13",    // standard 128-bit characteristic UUID
                             BLERead | BLENotify, sizeof(imuBuf));      // remote clients will be able to
                                                                        // get notifications if this characteristic changes

void ble_connected(BLEHelper &role)
{
    BLEPeripheralHelper&peripheral = *(BLEPeripheralHelper*)(&role);
    Serial.println("Connected");
    
    // Start discovery the profiles in peripheral device
    peripheral.discover();
}

void bleImuCharacteristicWritten(BLEHelper& central, BLECharacteristic& characteristic)
{
    // Peripheral wrote new value to characteristic by Notification/Indication
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
               const uint8_t *data, 
               uint8_t data_len,
               void *user_data)
{
    bt_addr_le_t *addr = (bt_addr_le_t *)user_data;
    int i;

    Serial.print("[AD]:");
    Serial.print(type);
    Serial.print(" data_len ");
    Serial.println(data_len);

    switch (type)
    {
        case BT_DATA_UUID128_SOME:
        case BT_DATA_UUID128_ALL:
        {
            if (data_len % MAX_UUID_SIZE != 0) 
            {
                Serial.println("AD malformed");
                return true;
            }
            struct bt_uuid * serviceuuid = bleImuService.uuid();
            for (i = 0; i < data_len; i += MAX_UUID_SIZE)
            {
                if (memcmp (((struct bt_uuid_128*)serviceuuid)->val, &data[i], MAX_UUID_SIZE) != 0) 
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
                bleCentral.connect(addr, &conn_param);
                return false;
            }
        }
    }

    return true;
}

void setup() {    
    Serial.begin(115200);    // initialize serial communication
    pinMode(13, OUTPUT);   // initialize the LED on pin 13 to indicate when a central is connected

    bleImuChar.setEventHandler(BLEWritten, bleImuCharacteristicWritten);
    
    /* Set a local name for the BLE device
     This name will appear in advertising packets
     and can be used by remote devices to identify this BLE device
     The name can be changed but maybe be truncated based on space 
     left in advertisement packet */
    bleCentral.addAttribute(bleImuService); // Add the BLE IMU service
    bleCentral.addAttribute(bleImuChar);    // Add the BLE IMU characteristic

    /* Setup callback */
    bleCentral.setAdvertiseHandler(adv_found);
    bleCentral.setEventHandler(BLEConnected, ble_connected);
    
    /* Now activate the BLE device.  It will start continuously transmitting BLE
     advertising packets and will be visible to remote BLE central devices
     until it receives a new connection */
    bleCentral.begin();
}


void loop()
{
    delay(2000);
}

