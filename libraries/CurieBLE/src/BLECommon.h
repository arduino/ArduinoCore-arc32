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
//#include "CurieBLE.h"

#include "../src/services/ble_service/ble_protocol.h"


#include "infra/log.h"


#include <bluetooth/bluetooth.h>
#include <bluetooth/gatt.h>
#include <bluetooth/conn.h>
#include <bluetooth/hci.h>
//#include <bluetooth/uuid.h>

#define BLE_ADDR_LEN 6

#define UUID_SIZE_128   16
#define UUID_SIZE_16    2
#define MAX_UUID_SIZE   UUID_SIZE_128

/* Theoretically we should be able to support attribute lengths up to 512 bytes
 * but this involves splitting it across multiple packets.  For simplicity,
 * we will just limit this to 20 bytes for now, which will fit in a single packet
 */
#define BLE_MAX_ATTR_DATA_LEN       20
#define BLE_MAX_ATTR_LONGDATA_LEN   512

/* Default device name prefix, applied only if user does not provide a name
 * If a factory-configured MAC address is defined, the last 2 bytes of the
 * address will be appended to the device name */
#define BLE_DEVICE_NAME_DEFAULT_PREFIX "Arduino101"

/* Invalid BLE Address type */
#define BLE_DEVICE_ADDR_INVALID 0xFF

#if 0
/** BLE response/event status codes. */
enum BLE_STATUS {
    BLE_STATUS_SUCCESS = 0, /**< General BLE Success code */
    BLE_STATUS_PENDING, /**< Request received and execution started, response pending */
    BLE_STATUS_TIMEOUT, /**< Request timed out */
    BLE_STATUS_NOT_SUPPORTED, /**< Request/feature/parameter not supported */
    BLE_STATUS_NOT_ALLOWED, /**< Request not allowed */
    BLE_STATUS_LINK_TIMEOUT, /**< Link timeout (link loss) */
    BLE_STATUS_NOT_ENABLED, /**< BLE not enabled, @ref ble_enable */
    BLE_STATUS_ERROR,   /**< Generic Error */
    BLE_STATUS_ALREADY_REGISTERED, /**< BLE service already registered */
    BLE_STATUS_WRONG_STATE, /**< Wrong state for request */
    BLE_STATUS_ERROR_PARAMETER, /**< Parameter in request is wrong */
    BLE_STATUS_NO_MEMORY, /**< System doesn't have memory */
    BLE_STATUS_GAP_BASE = 0x100, /**< GAP specific error base */
    BLE_STATUS_GATT_BASE = 0x200, /**< GATT specific Error base */
};
#endif

typedef enum 
{
    BLE_STATUS_SUCCESS = 0,
    BLE_STATUS_FORBIDDEN, /**< The operation is forbidden. Central mode call peripheral API and vice versa */
    BLE_STATUS_PENDING, /**< Request received and execution started, response pending */
    BLE_STATUS_TIMEOUT, /**< Request timed out */
    BLE_STATUS_NOT_SUPPORTED, /**< Request/feature/parameter not supported */
    BLE_STATUS_NOT_FOUND,
    BLE_STATUS_NOT_ALLOWED, /**< Request not allowed */
    BLE_STATUS_LINK_TIMEOUT, /**< Link timeout (link loss) */
    BLE_STATUS_NOT_ENABLED, /**< BLE not enabled, @ref ble_enable */
    BLE_STATUS_ERROR,   /**< Generic Error */
    BLE_STATUS_ALREADY_REGISTERED, /**< BLE service already registered */
    BLE_STATUS_WRONG_STATE, /**< Wrong state for request */
    BLE_STATUS_ERROR_PARAMETER, /**< Parameter in request is wrong */
    BLE_STATUS_NO_MEMORY, /**< System doesn't have memory */
    BLE_STATUS_NO_SERVICE, /**< System doesn't have service */
}BLE_STATUS_T;

typedef uint16_t ble_status_t; /**< Response and event BLE service status type @ref BLE_STATUS */

typedef ble_status_t BleStatus;

#define BLE_LIB_ASSERT(cond) ((cond) ? (void)0 : __assert_fail())

#define BLE_MAX_CONN_CFG            2
#define BLE_MAX_ADV_BUFFER_CFG      3

typedef bool (*ble_advertise_handle_cb_t)(uint8_t type, const uint8_t *dataPtr,
                                          uint8_t data_len, const bt_addr_le_t *addrPtr);


typedef struct ble_conn_param {
    float interval_min; // millisecond 7.5 - 4000ms
    float interval_max; // millisecond 7.5 - 4000ms
    uint16_t latency;   // 0x0000 - 0x01F4
    uint16_t timeout;   // millisecond 100 - 32000ms
}ble_conn_param_t;
#ifdef __cplusplus
extern "C" {
#endif

#include "os/os.h"

extern void __assert_fail(void);

/// Define the structure for app
typedef struct bt_uuid      bt_uuid_t;
typedef struct bt_uuid_16   bt_uuid_16_t;
typedef struct bt_uuid_128  bt_uuid_128_t;
typedef struct bt_conn  bt_conn_t;
typedef struct bt_gatt_attr  bt_gatt_attr_t;
typedef struct bt_gatt_discover_params  bt_gatt_discover_params_t;
typedef struct bt_le_scan_param  bt_le_scan_param_t;
typedef struct bt_le_conn_param bt_le_conn_param_t;
typedef struct bt_gatt_subscribe_params bt_gatt_subscribe_params_t;
typedef struct bt_gatt_read_params bt_gatt_read_params_t;
typedef struct _bt_gatt_ccc _bt_gatt_ccc_t;
typedef struct bt_gatt_chrc bt_gatt_chrc_t;
typedef struct bt_gatt_ccc_cfg bt_gatt_ccc_cfg_t;
typedef struct bt_data bt_data_t;

#ifdef __cplusplus
}
#endif
#endif // _BLE_COMMON_H_INCLUDED
