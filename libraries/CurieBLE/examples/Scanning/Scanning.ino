/*
 *  Copyright (c) 2016 Intel Corporation.  All rights reserved.
 *  See the bottom of this file for the license terms.
 */

#include <CurieBLE.h>

/*
  This sketch is meaningful if one or more BLE peripheral devices (any of the peripheral examples will do)
  are present.

  This sketch try to show the scan function.
  The sketch will list the device's MAC address and device name to the console.
  The list will refresh every 3s.
*/


const int bleScanMaxCnt = 5;

typedef struct {
  char macaddr[32];       // BLE MAC address.
  char loacalname[22];    // Device's name
} ble_device_info_t;

ble_device_info_t device_list[bleScanMaxCnt];
uint8_t list_index = 0;

BLECentral bleCentral;  // BLE Central Device (the board you're programming)

bool adv_found(uint8_t type,
               const uint8_t *dataPtr,
               uint8_t data_len,
               const bt_addr_le_t *addrPtr);

void setup() {
  // initialize serial communication
  Serial.begin(115200);
  // wait for the serial port to connect. Open the Serial Monitor to continue executing the sketch
  // If you don't care to see text messages sent to the Serial Monitor during board initialization,
  // remove or comment out the next line
  while (!Serial) ;

  // Setup adv_found() callback function for advertising packets
  bleCentral.setAdvertiseHandler(adv_found);

  /* Now activate the BLE central device.
     It will start continuously scanning BLE advertising packets
     sent by BLE peripheral devices
  */
  bleCentral.begin();
  Serial.println("Bluetooth device active, start scanning...");
}

void loop() {
  // Output the scanned device per 3s
  delay(3000);
  Serial.print("\r\n\r\n\t\t\tScaning result\r\n \tMAC\t\t\t\tLocal Name\r\n");
  Serial.print("-------------------------------------------------------------\r\n");

  // print MAC address and local name of new scanned devices
  for (int i = 0; i < list_index; i++) {
    Serial.print(device_list[i].macaddr);
    Serial.print(" | ");
    Serial.println(device_list[i].loacalname);
  }
  if (list_index == 0) {
    Serial.print("No device found\r\n");
  }
  Serial.print("-------------------------------------------------------------\r\n");
  adv_list_clear();
}

// Add the scanned BLE device into the global variables.
bool adv_list_add(ble_device_info_t &device) {
  if (list_index >= bleScanMaxCnt) {
    return false;
  }
  for (int i = 0; i < list_index; i++) {
    if (0 == memcmp(device.macaddr, device_list[i].macaddr, sizeof (device.macaddr))) {
      // Found and update the item
      return false;
    }
  }
  // Add the device
  memcpy(&device_list[list_index], &device, sizeof (ble_device_info_t));
  list_index++;
  return true;
}


bool adv_list_update(ble_device_info_t &device) {
  for (int i = 0; i < list_index; i++) {
    if (0 == memcmp(device.macaddr, device_list[i].macaddr, sizeof (device.macaddr))) {
      // Found and update the item
      memcpy(device_list[i].loacalname, device.loacalname, sizeof(device.loacalname));
      return true;
    }
  }
  return false;
}

void adv_list_clear() {
  list_index = 0;
  memset(device_list, 0x00, sizeof(device_list));
}

// Process the Advertisement data
bool adv_found(uint8_t type,
               const uint8_t *dataPtr,
               uint8_t data_len,
               const bt_addr_le_t *addrPtr)
{
  ble_device_info_t device;
  bt_addr_le_to_str (addrPtr, device.macaddr, sizeof (device.macaddr));
  memcpy(device.loacalname, " -NA-", sizeof(" -NA-"));
  // Please see https://www.bluetooth.org/en-us/specification/assigned-numbers/generic-access-profile
  switch (type) {
    case BT_DATA_NAME_SHORTENED:
    case BT_DATA_NAME_COMPLETE:
      memcpy(device.loacalname, dataPtr, data_len);
      device.loacalname[data_len] = '\0';
      adv_list_update(device);
      break;
  }
  adv_list_add(device);
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
