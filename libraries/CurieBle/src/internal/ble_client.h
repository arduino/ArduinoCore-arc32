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

typedef enum {
    BLE_CLIENT_GAP_EVENT_CONNECTED = 0,
    BLE_CLIENT_GAP_EVENT_DISCONNECTED,
    BLE_CLIENT_GAP_EVENT_ADV_TIMEOUT,
    BLE_CLIENT_GAP_EVENT_CONN_TIMEOUT,
    BLE_CLIENT_GAP_EVENT_RSSI,
} ble_client_gap_event_t;

typedef enum {
    BLE_CLIENT_GATTS_EVENT_WRITE = 0,
} ble_client_gatts_event_t;

typedef void (*ble_client_gap_event_cb_t)(ble_client_gap_event_t event, struct ble_gap_event *event_data, void *param);
typedef void (*ble_client_gatts_event_cb_t)(ble_client_gatts_event_t event, struct ble_gatts_evt_msg *event_data, void *param);

#ifdef __cplusplus
extern "C" {
#endif

void ble_client_get_factory_config(ble_addr_t *bda, char *name);
BleStatus ble_client_init(ble_client_gap_event_cb_t gap_event_cb,
                          void *gap_event_param,
                          ble_client_gatts_event_cb_t gatts_event_cb,
                          void *gatts_event_param);
BleStatus ble_client_gap_set_enable_config(const char *name,
                                           const ble_addr_t *bda,
                                           const uint16_t appearance,
                                           const int8_t tx_power);
BleStatus ble_client_gap_get_bda(ble_addr_t *p_bda);
BleStatus ble_client_gap_wr_adv_data(uint8_t *adv_data,
                                     const uint8_t adv_data_len);
BleStatus ble_client_gap_start_advertise(uint16_t timeout);
BleStatus ble_client_gap_stop_advertise(void);
BleStatus ble_client_gatts_add_service(const struct bt_uuid *uuid, const uint8_t type, uint16_t *svc_handle);
BleStatus ble_client_gatts_include_service(const uint16_t primary_svc_handle, uint16_t included_svc_handle);
BleStatus ble_client_gatts_add_characteristic(const uint16_t svc_handle,
                                              struct ble_gatts_characteristic *char_data,
                                              struct ble_gatts_char_handles *handles);
BleStatus ble_client_gatts_add_descriptor(const uint16_t svc_handle,
                                          struct ble_gatts_descriptor *desc,
                                          uint16_t *handle);
BleStatus ble_client_gatts_set_attribute_value(const uint16_t value_handle,
                                               const uint16_t len, const uint8_t *value,
                                               const uint16_t offset);
BleStatus ble_client_gatts_send_notif_ind(const uint16_t value_handle,
                                          const uint16_t len, uint8_t * p_value,
                                          const uint16_t offset,
                                          const bool indication);
BleStatus ble_client_gap_disconnect(const uint8_t reason);
BleStatus ble_client_gap_set_rssi_report(boolean_t enable);

/* Direct Test Mode (DTM) API - for internal use only */
BleStatus ble_client_dtm_init(void);
BleStatus ble_client_dtm_cmd(const struct ble_test_cmd *test_cmd,
                             struct ble_dtm_test_result *test_result);

#ifdef __cplusplus
}
#endif


#endif // _BLE_CLIENT_H_INCLUDED
