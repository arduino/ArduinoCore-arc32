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

#define BLE_SCANING_DEVICE_MAX_CNT  5

typedef struct{
    char macaddr[32];       // BLE MAC address.
    char loacalname[22];    // Device's name
}ble_device_info_t;

ble_device_info_t device_list[BLE_SCANING_DEVICE_MAX_CNT];
uint8_t list_index = 0;

BLECentral bleCentral;       // BLE Central Device (the board you're programming)

bool adv_list_add(ble_device_info_t &device)
{
    if (list_index >= BLE_SCANING_DEVICE_MAX_CNT)
    {
        return false;
    }
    for (int i = 0; i < list_index; i++)
    {
        if (0 == memcmp(device.macaddr, device_list[i].macaddr, sizeof (device.macaddr)))
        {
            // Found and update the item
            return false;
        }
    }
    // Add the device
    memcpy(&device_list[list_index], &device, sizeof (ble_device_info_t));
    list_index++;
    return true;
}


bool adv_list_update(ble_device_info_t &device)
{
    for (int i = 0; i < list_index; i++)
    {
        if (0 == memcmp(device.macaddr, device_list[i].macaddr, sizeof (device.macaddr)))
        {
            // Found and update the item
            memcpy(device_list[i].loacalname, device.loacalname, sizeof(device.loacalname));
            return true;
        }
    }
    return false;
}

void adv_list_clear()
{
    list_index = 0;
    memset(device_list, 0x00, sizeof(device_list));
}

// Process the Advertisement data
bool adv_found(uint8_t type, 
               const uint8_t *data, 
               uint8_t data_len,
               void *user_data)
{
    bt_addr_le_t *addr = (bt_addr_le_t *)user_data;
    ble_device_info_t device;
    bt_addr_le_to_str (addr, device.macaddr, sizeof (device.macaddr));
    memcpy(device.loacalname, " -NA-", sizeof(" -NA-"));
    
    switch (type) {
    case BT_DATA_NAME_SHORTENED:
    case BT_DATA_NAME_COMPLETE:
        memcpy(device.loacalname, data, data_len);
        device.loacalname[data_len] = '\0';
        adv_list_update(device);
        break;
    }
    adv_list_add(device);
    return true;
}

void setup() {
    Serial.begin(115200);    // initialize serial communication
    
    /* Setup callback */
    bleCentral.setAdvertiseHandler(adv_found);
    
    /* Now activate the BLE device.
       It will start continuously scanning BLE advertising
     */
    bleCentral.begin();
    Serial.println("Bluetooth device active, start scanning...");
}

void loop() {
    // Output the scanned device per 3s
    delay(3000);
    Serial.print("\r\n\r\n\t\t\tScaning result\r\n \tMAC\t\t\t\tLocal Name\r\n");
    Serial.print("-------------------------------------------------------------\r\n");
    
    for (int i = 0; i < list_index; i++)
    {
        
        Serial.print(device_list[i].macaddr);
        Serial.print(" | ");
        Serial.println(device_list[i].loacalname);
    }
    if (list_index == 0)
    {
        Serial.print("No device found\r\n");
    }
    Serial.print("-------------------------------------------------------------\r\n");
    adv_list_clear();
}

