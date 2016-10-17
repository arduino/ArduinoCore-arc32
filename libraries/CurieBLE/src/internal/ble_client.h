/*
 * Copyright (c) 2015, Intel Corporation. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef _BLE_CLIENT_H_INCLUDED
#define _BLE_CLIENT_H_INCLUDED

#include "BLECommon.h"

enum {
    UNIT_0_625_MS = 625,                            /**< Number of microseconds in 0.625 milliseconds. */
    UNIT_1_25_MS = 1250,                            /**< Number of microseconds in 1.25 milliseconds. */
    UNIT_10_MS = 10000                              /**< Number of microseconds in 10 milliseconds. */
};

#define MSEC_TO_UNITS(TIME, RESOLUTION) (((TIME) * 1000) / (RESOLUTION))
#define UNITS_TO_MSEC(TIME, RESOLUTION) (((TIME) * RESOLUTION) / 1000)

/* Connection parameters used for Peripheral Preferred Connection Parameterss (PPCP) and update request */
#define DEFAULT_MIN_CONN_INTERVAL MSEC_TO_UNITS(80, UNIT_1_25_MS)
#define DEFAULT_MAX_CONN_INTERVAL MSEC_TO_UNITS(150, UNIT_1_25_MS)
#define MIN_CONN_INTERVAL 0x0006
#define MAX_CONN_INTERVAL 0x0C80
#define SLAVE_LATENCY 0
#define CONN_SUP_TIMEOUT MSEC_TO_UNITS(6000, UNIT_10_MS)

/* Borrowed from ble_service_utils.h */
#define UINT8_TO_LESTREAM(p, i) \
    do { *(p)++ = (uint8_t)(i); } \
    while (0)
#define UINT16_TO_LESTREAM(p, i) UINT8_TO_LESTREAM(p, i); UINT8_TO_LESTREAM(p, (i)>>8)
#define UINT32_TO_LESTREAM(p, i) UINT16_TO_LESTREAM(p, i); UINT16_TO_LESTREAM(p, (i)>>16)

#define INT8_TO_LESTREAM(p, i) UINT8_TO_LESTREAM(p, (uint8_t)(i))
#define INT16_TO_LESTREAM(p, i) UINT16_TO_LESTREAM(p, (uint16_t)(i))
#define INT32_TO_LESTREAM(p, i) UINT32_TO_LESTREAM(p, (uint32_t)(i))

#define LESTREAM_TO_UINT8(p, i) \
    do { i = *p; p++; } \
    while (0)
#define LESTREAM_TO_UINT16(p, i) \
    do { uint16_t temp16; LESTREAM_TO_UINT8(p, i); LESTREAM_TO_UINT8(p, temp16); i |= (temp16 << 8); } \
    while (0)
#define LESTREAM_TO_UINT32(p, i) \
    do { uint32_t temp32; LESTREAM_TO_UINT16(p, i); LESTREAM_TO_UINT16(p, temp32); i |= (temp32 << 16); } \
    while (0)

#define LESTREAM_TO_INT8(p, i) \
    do {uint8_t __i; LESTREAM_TO_UINT8(p, __i); i = (int8_t)__i; } while (0)
#define LESTREAM_TO_INT16(p, i) \
    do {uint16_t __i; LESTREAM_TO_UINT16(p, __i); i = (int16_t)__i; } while (0)
#define LESTREAM_TO_INT32(p, i) \
    do {uint32_t __i; LESTREAM_TO_UINT32(p, __i); i = (int32_t)__i; } while (0)

#define BLE_BASE_UUID_OCTET_OFFSET 12
#define BLE_UUID16_TO_UUID128(uuid, base)                               \
    do {                                                                \
        uint16_t uuid16 = uuid.uuid16;                                  \
        memcpy(uuid.uuid128, base.uuid128, sizeof(uuid.uuid128));       \
        uint8_t *p = &uuid.uuid128[BLE_BASE_UUID_OCTET_OFFSET];         \
        UINT16_TO_LESTREAM(p, uuid16);                                  \
        uuid.type = BT_UUID128;                                         \
    } while(0)


typedef void (*ble_client_connect_event_cb_t)(struct bt_conn *conn, uint8_t err, void *param);
typedef void (*ble_client_disconnect_event_cb_t)(struct bt_conn *conn, uint8_t reason, void *param);
typedef void (*ble_client_update_param_event_cb_t)(struct bt_conn *conn, 
                                                   uint16_t interval,
                                                   uint16_t latency, 
                                                   uint16_t timeout, 
                                                   void *param);


#ifdef __cplusplus
extern "C" {
#endif

void ble_client_init(ble_client_connect_event_cb_t connect_cb, void* connect_param,
                     ble_client_disconnect_event_cb_t disconnect_cb, void* disconnect_param,
                     ble_client_update_param_event_cb_t update_param_cb, void* update_param_param);
void ble_client_get_factory_config(bt_addr_le_t *bda, char *name);
void ble_gap_set_tx_power(int8_t tx_power);
BLE_STATUS_T errorno_to_ble_status(int err);

void ble_client_get_mac_address(bt_addr_le_t *bda);

#ifdef __cplusplus
}
#endif


#endif // _BLE_CLIENT_H_INCLUDED
