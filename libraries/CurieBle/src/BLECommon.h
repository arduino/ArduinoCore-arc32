/*
 * Copyright (c) 2015 Intel Corporation.  All rights reserved.
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.

 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.

 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
 *
 */

#ifndef _BLE_COMMON_H_INCLUDED
#define _BLE_COMMON_H_INCLUDED

#include "Arduino.h"

#include "../src/services/ble/ble_protocol.h"
#include "services/ble/ble_service_gatt.h"
#include "services/ble/ble_service_gatts_api.h"

/* Theoretically we should be able to support attribute lengths up to 512 bytes
 * but this involves splitting it across multiple packets.  For simplicity,
 * we will just limit this to 20 bytes for now, which will fit in a single packet
 */
#define BLE_MAX_ATTR_DATA_LEN 20

/* Default device name prefix, applied only if user does not provide a name
 * If a factory-configured MAC address is defined, the last 2 bytes of the
 * address will be appended to the device name */
#define BLE_DEVICE_NAME_DEFAULT_PREFIX "Arduino101"

/* Invalid BLE Address type */
#define BLE_DEVICE_ADDR_INVALID 0xFF

typedef ble_status_t BleStatus;

#endif // _BLE_COMMON_H_INCLUDED
