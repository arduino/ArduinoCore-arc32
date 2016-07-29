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

#include "../src/services/ble_service/ble_protocol.h"


#include "infra/log.h"


#include <bluetooth/bluetooth.h>
#include <bluetooth/gatt.h>
#include <bluetooth/conn.h>
#include <bluetooth/hci.h>

#define BLE_ADDR_LEN 6

#define MAX_UUID_SIZE              16


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

/** BLE response/event status codes. */
enum BLE_STATUS {
	BLE_STATUS_SUCCESS = 0, /**< General BLE Success code */
	BLE_STATUS_PENDING, /**< Request received and execution started, response pending */
	BLE_STATUS_TIMEOUT, /**< Request timed out */
	BLE_STATUS_NOT_SUPPORTED, /**< Request/feature/parameter not supported */
	BLE_STATUS_NOT_ALLOWED, /**< Request not allowed */
	BLE_STATUS_LINK_TIMEOUT, /**< Link timeout (link loss) */
	BLE_STATUS_NOT_ENABLED, /**< BLE not enabled, @ref ble_enable */
	BLE_STATUS_ERROR,	/**< Generic Error */
	BLE_STATUS_ALREADY_REGISTERED, /**< BLE service already registered */
	BLE_STATUS_WRONG_STATE, /**< Wrong state for request */
	BLE_STATUS_ERROR_PARAMETER, /**< Parameter in request is wrong */
	BLE_STATUS_GAP_BASE = 0x100, /**< GAP specific error base */
	BLE_STATUS_GATT_BASE = 0x200, /**< GATT specific Error base */
};

typedef uint16_t ble_status_t; /**< Response and event BLE service status type @ref BLE_STATUS */

typedef ble_status_t BleStatus;

#define BLE_MAX_CONN_CFG    2

typedef bool (*ble_advertise_handle_cb_t)(uint8_t type, const uint8_t *data,
				                          uint8_t data_len, void *user_data);

#endif // _BLE_COMMON_H_INCLUDED
