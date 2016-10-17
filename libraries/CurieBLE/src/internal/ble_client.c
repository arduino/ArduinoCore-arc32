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
     
#include <errno.h>

#include <string.h>
#include "cfw/cfw.h"
#include "cfw/cfw_debug.h"
#include "cfw/cfw_messages.h"
#include "cfw/cfw_internal.h"
#include "cfw/cfw_service.h"
#include "cfw_platform.h"
#include "infra/time.h"
#include "infra/factory_data.h"
#include "infra/version.h"
#include "curie_factory_data.h"
#include "portable.h"

#include "uart.h"
#include "ipc_uart_ns16550.h"
#include "infra/ipc_uart.h"

#include "ble_client.h"
#include "platform.h"

#include "infra/log.h"

// APP callback
static ble_client_connect_event_cb_t ble_client_connect_event_cb = NULL;
static void *ble_client_connect_event_param;

static ble_client_disconnect_event_cb_t ble_client_disconnect_event_cb = NULL;
static void *ble_client_disconnect_event_param;

static ble_client_update_param_event_cb_t ble_client_update_param_event_cb = NULL;
static void *ble_client_update_param_event_param;


#define NIBBLE_TO_CHAR(n) \
    ((n) >= 0xA ? ('A' + (n) - 0xA) : ('0' + (n)))

#define BYTE_TO_STR(s, byte)               \
    do {                                   \
        *s++ = NIBBLE_TO_CHAR(byte >> 4);  \
        *s++ = NIBBLE_TO_CHAR(byte & 0xF); \
    }while(0)


#ifdef __cplusplus
extern "C" {
#endif

static void on_connected(bt_conn_t *conn, uint8_t err)
{
    if (ble_client_connect_event_cb)
    {
        ble_client_connect_event_cb(conn, err, ble_client_connect_event_param);
    }
}

static void on_disconnected(bt_conn_t *conn, uint8_t reason)
{
    if (ble_client_disconnect_event_cb)
    {
        ble_client_disconnect_event_cb(conn, reason, ble_client_disconnect_event_param);
    }
}

static void on_le_param_updated(bt_conn_t *conn, uint16_t interval,
                uint16_t latency, uint16_t timeout)
{
    if (ble_client_update_param_event_cb)
    {
        ble_client_update_param_event_cb (conn, 
                                          interval,
                                          latency, 
                                          timeout, 
                                          ble_client_update_param_event_param);
    }
}

static struct bt_conn_cb conn_callbacks = {
    .connected = on_connected,
    .disconnected = on_disconnected,
    .le_param_updated = on_le_param_updated
};

void ble_client_get_mac_address(bt_addr_le_t *bda)
{
    struct curie_oem_data *p_oem = NULL;
    unsigned i;

    /* Set the MAC address defined in Factory Data (if provided)
     * Otherwise, the device will default to a static random address */
    if (bda) {
        bda->type = BLE_DEVICE_ADDR_INVALID;
        if (!strncmp((char*)global_factory_data->oem_data.magic, FACTORY_DATA_MAGIC, 4)) {
            p_oem = (struct curie_oem_data *) &global_factory_data->oem_data.project_data;
            if (p_oem->bt_mac_address_type < 2) {
                bda->type = p_oem->bt_mac_address_type;
                for (i = 0; i < BLE_ADDR_LEN; i++)
                    bda->val[i] = p_oem->bt_address[BLE_ADDR_LEN - 1 - i];
            }
        }
    }
}

void ble_client_get_factory_config(bt_addr_le_t *bda, char *name)
{
    struct curie_oem_data *p_oem = NULL;
    
    ble_client_get_mac_address(bda);

    /* Set a default name if one has not been specified */
    if (name) {

        // Need to check in the OTP if there is some board name set
        // If yes, let's read it, otherwise let's keep the default
        // name set in BLE_DEVICE_NAME_DEFAULT_PREFIX
        const struct customer_data* otp_data_ptr = (struct customer_data*)(FACTORY_DATA_ADDR + 0x200);
        char *suffix;

        // checking the presence of key patterns
        if ((otp_data_ptr->patternKeyStart == PATTERN_KEY_START) &&
            (otp_data_ptr->patternKeyEnd == PATTERN_KEY_END))
        {
           // The board name is with OTP ar programmed
           uint8_t len = otp_data_ptr->board_name_len;

           // We need to reserve 5 bytes for '-' and 4 last MAC address in ASCII 
           if (len > BLE_MAX_DEVICE_NAME - 5) len = BLE_MAX_DEVICE_NAME - 5;
           strncpy(name, (const char *)otp_data_ptr->board_name, len);
           suffix = name + len;
        }
        else
        {
            // There is no board name in the OTP area
            suffix = name + strlen(BLE_DEVICE_NAME_DEFAULT_PREFIX);
            strcpy(name, BLE_DEVICE_NAME_DEFAULT_PREFIX);
        }

        // Adding the four last digits of MAC address separated by '-' sufix
        if (bda && bda->type != BLE_DEVICE_ADDR_INVALID) 
        {
            *suffix++ = '-';
            p_oem = (struct curie_oem_data *) &global_factory_data->oem_data.project_data;
            BYTE_TO_STR(suffix, p_oem->bt_address[4]);
            BYTE_TO_STR(suffix, p_oem->bt_address[5]);
            *suffix = 0; /* NULL-terminate the string. Note the macro BYTE_TO_STR 
                             automatically move the pointer */
        }
        else
        {
         /* This code segment will be only reached if Curie module was not 
                provisioned properly with a BLE MAC address*/
             *suffix++ = 0; /* NULL-terminate the string */
        }
    }
}

void ble_client_init(ble_client_connect_event_cb_t connect_cb, void* connect_param,
                     ble_client_disconnect_event_cb_t disconnect_cb, void* disconnect_param,
                     ble_client_update_param_event_cb_t update_param_cb, void* update_param_param)
{
    //uint32_t delay_until;
    pr_info(LOG_MODULE_BLE, "%s", __FUNCTION__);
    ble_client_connect_event_cb = connect_cb;
    ble_client_connect_event_param = connect_param;
    
    ble_client_disconnect_event_cb = disconnect_cb;
    ble_client_disconnect_event_param = disconnect_param;
    
    ble_client_update_param_event_cb = update_param_cb;
    ble_client_update_param_event_param = update_param_param;
    
    bt_conn_cb_register(&conn_callbacks);
    return;
}

BLE_STATUS_T errorno_to_ble_status(int err)
{
    BLE_STATUS_T err_code;
    err = 0 - err;
    
    switch(err) {
    case 0:
        err_code = BLE_STATUS_SUCCESS;
        break;
    case EIO:
        err_code = BLE_STATUS_WRONG_STATE;
        break;
    case EBUSY:
        err_code = BLE_STATUS_TIMEOUT;
        break;
    case EFBIG:
    case ENOTSUP:
        err_code = BLE_STATUS_NOT_SUPPORTED;
        break;
    case EPERM:
    case EACCES:
        err_code = BLE_STATUS_NOT_ALLOWED;
        break;
    case ENOMEM: // No memeory
        err_code = BLE_STATUS_NO_MEMORY;
        break;
    default:
        err_code = BLE_STATUS_ERROR;
        break;
    }
    return err_code;
}


#ifdef __cplusplus
}
#endif
