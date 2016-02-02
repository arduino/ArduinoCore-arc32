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

enum {
    UNIT_0_625_MS = 625,                            /**< Number of microseconds in 0.625 milliseconds. */
    UNIT_1_25_MS = 1250,                            /**< Number of microseconds in 1.25 milliseconds. */
    UNIT_10_MS = 10000                              /**< Number of microseconds in 10 milliseconds. */
};

#define MSEC_TO_UNITS(TIME, RESOLUTION) (((TIME) * 1000) / (RESOLUTION))

/* Connection parameters used for Peripheral Preferred Connection Parameterss (PPCP) and update request */
#define MIN_CONN_INTERVAL MSEC_TO_UNITS(80, UNIT_1_25_MS)
#define MAX_CONN_INTERVAL MSEC_TO_UNITS(150, UNIT_1_25_MS)
#define SLAVE_LATENCY 0
#define CONN_SUP_TIMEOUT MSEC_TO_UNITS(6000, UNIT_10_MS)

/* Advertising parameters */
#define BLE_GAP_ADV_TYPE_ADV_IND          0x00   /**< Connectable undirected. */
#define BLE_GAP_ADV_FP_ANY                0x00   /**< Allow scan requests and connect requests from any device. */
/** options see \ref BLE_ADV_OPTIONS */
/* options: BLE_NO_ADV_OPT */
#define APP_ULTRA_FAST_ADV_INTERVAL             32
#define APP_ULTRA_FAST_ADV_TIMEOUT_IN_SECONDS   180
/* options: BLE_SLOW_ADV */
#define APP_DISC_ADV_INTERVAL           160
#define APP_DISC_ADV_TIMEOUT_IN_SECONDS 180
/* options: BLE_NON_DISC_ADV */
#define APP_NON_DISC_ADV_FAST_INTERVAL              160
#define APP_NON_DISC_ADV_FAST_TIMEOUT_IN_SECONDS    30
/* options: BLE_SLOW_ADV | BLE_NON_DISC_ADV */
#define APP_NON_DISC_ADV_SLOW_INTERVAL              2056
#define APP_NON_DISC_ADV_SLOW_TIMEOUT_IN_SECONDS    0

struct cfw_msg_rsp_sync {
    volatile unsigned response;
    volatile ble_status_t status;
    void *param;
};

#define TIMEOUT_TICKS_1SEC 32768 /* ~1 second in RTC timer ticks */
#define TIMEOUT_TICKS_1MS  32    /* ~1 millisecond in RTC timer ticks */
#define wait_for_condition(cond, status) \
do { \
    unsigned timeout = get_uptime_32k() + TIMEOUT_TICKS_1SEC;  \
    status = BLE_STATUS_SUCCESS; \
    while (!(cond)) { \
        if (get_uptime_32k() > timeout) { \
            status = BLE_STATUS_TIMEOUT; \
            break; \
        } \
    } \
} while(0)

static cfw_handle_t        client_handle;
static svc_client_handle_t *service_handle;
static uint16_t            conn_handle;
static bool                connected;

static ble_client_gap_event_cb_t ble_client_gap_event_cb;
static void *ble_client_gap_event_param;

static ble_client_gatts_event_cb_t ble_client_gatts_event_cb;
static void *ble_client_gatts_event_param;

volatile struct cfw_msg_rsp_sync sync;


static void handle_msg_id_cfw_svc_avail_evt(cfw_svc_available_evt_msg_t *evt, void *param)
{
    if (evt->service_id == BLE_CORE_SERVICE_ID) {
        sync.status = BLE_STATUS_SUCCESS;
        sync.response = 1;
    }
}

static void handle_msg_id_cfw_open_svc(cfw_open_conn_rsp_msg_t *rsp, void *param)
{
    service_handle = (svc_client_handle_t *)(rsp->client_handle);

    sync.status = BLE_STATUS_SUCCESS;
    sync.response = 1;
}

static void handle_msg_id_ble_gap_wr_conf_rsp(struct ble_rsp *rsp, void *param)
{
    sync.status = rsp->status;
    sync.response = 1;
}

static void handle_msg_id_ble_gap_rd_bda_rsp(ble_bda_rd_rsp_t *rsp, void *param)
{
    ble_addr_t *p_bda = (ble_addr_t *)sync.param;

    if (p_bda && BLE_STATUS_SUCCESS == rsp->status)
        memcpy(p_bda, &rsp->bd, sizeof(*p_bda));

    sync.status = rsp->status;
    sync.response = 1;
}

static void handle_msg_id_ble_gap_sm_config_rsp(struct ble_rsp *rsp, void *param)
{
    sync.status = rsp->status;
    sync.response = 1;
}

static void handle_msg_id_ble_gap_wr_adv_data_rsp(struct ble_rsp *rsp, void *param)
{
    sync.status = rsp->status;
    sync.response = 1;
}

static void handle_msg_id_ble_gap_enable_adv_rsp(struct ble_rsp *rsp, void *param)
{
    /* No waiting for this response, so nothing to do here */
}

static void handle_msg_id_ble_gap_disable_adv_rsp(struct ble_rsp *rsp, void *param)
{
    /* No waiting for this response, so nothing to do here */
}

static void handle_msg_id_gatts_add_service_rsp(struct ble_gatts_add_svc_rsp *rsp, void *param)
{
    uint16_t *p_svc_handle = (uint16_t *)sync.param;

    if (p_svc_handle && BLE_STATUS_SUCCESS == rsp->status)
        *p_svc_handle = rsp->svc_handle;

    sync.status = rsp->status;
    sync.response = 1;
}

static void handle_msg_id_gatts_add_characteristic_rsp(struct ble_gatts_add_char_rsp *rsp, void *param)
{
    struct ble_gatts_char_handles *p_handles = (struct ble_gatts_char_handles *)sync.param;

    if (p_handles && BLE_STATUS_SUCCESS == rsp->status)
        memcpy(p_handles, &rsp->char_h, sizeof(*p_handles));

    sync.status = rsp->status;
    sync.response = 1;
}

static void handle_msg_id_gatts_add_desc_rsp(struct ble_gatts_add_desc_rsp *rsp, void *param)
{
    uint16_t *p_handle = (uint16_t *)sync.param;

    if (p_handle && BLE_STATUS_SUCCESS == rsp->status)
        *p_handle = rsp->handle;

    sync.status = rsp->status;
    sync.response = 1;
}

static void handle_msg_id_ble_gatts_set_attribute_value_rsp(struct ble_gatts_set_attr_rsp_msg *rsp, void *param)
{
    sync.status = rsp->status;
    sync.response = 1;
}

static void handle_msg_id_ble_gap_connect_evt_msg(struct ble_gap_event *evt, void *param)
{
    conn_handle = evt->conn_handle;
    connected = true;

    if (ble_client_gap_event_cb)
        ble_client_gap_event_cb(BLE_CLIENT_GAP_EVENT_CONNECTED, evt, ble_client_gap_event_param);
}

static void handle_msg_id_ble_gap_disconnect_evt_msg(struct ble_gap_event *evt, void *param)
{
    connected = false;

    if (ble_client_gap_event_cb)
        ble_client_gap_event_cb(BLE_CLIENT_GAP_EVENT_DISCONNECTED, evt, ble_client_gap_event_param);
}

static void handle_msg_id_ble_gap_timeout_evt_msg(struct ble_gap_event *evt, void *param)
{
    connected = false;

    if (!ble_client_gap_event_cb)
        return;

    switch (evt->timeout.reason) {
    case BLE_SVC_GAP_TO_ADV:
        ble_client_gap_event_cb(BLE_CLIENT_GAP_EVENT_ADV_TIMEOUT, evt, ble_client_gap_event_param);
        break;
    case BLE_SVC_GAP_TO_CONN:
        ble_client_gap_event_cb(BLE_CLIENT_GAP_EVENT_CONN_TIMEOUT, evt, ble_client_gap_event_param);
        break;
    };
}

static void handle_msg_id_ble_gap_rssi_evt_msg(struct ble_gap_event *evt, void *param)
{
    if (ble_client_gap_event_cb)
        ble_client_gap_event_cb(BLE_CLIENT_GAP_EVENT_RSSI, evt, ble_client_gap_event_param);
}

static void handle_msg_id_ble_gatts_write_evt_msg(struct ble_gatts_evt_msg *evt, void *param)
{
    if (ble_client_gatts_event_cb)
        ble_client_gatts_event_cb(BLE_CLIENT_GATTS_EVENT_WRITE, evt, ble_client_gatts_event_param);
}

static void handle_msg_id_ble_gatts_send_notif_ind_rsp(ble_gatts_rsp_t *rsp, void *param)
{
    sync.status = rsp->status;
    sync.response = 1;
}

static void handle_msg_id_ble_gap_disconnect_rsp(struct ble_rsp *rsp, void *param)
{
    sync.status = rsp->status;
    sync.response = 1;
}

static void handle_msg_id_ble_gap_set_rssi_report_rsp(struct ble_rsp *rsp, void *param)
{
    sync.status = rsp->status;
    sync.response = 1;
}

static void handle_msg_id_ble_gap_dtm_init_rsp(struct ble_generic_msg *rsp, void *param)
{
    sync.status = rsp->status;
    sync.response = 1;
}

static void ble_core_client_handle_message(struct cfw_message *msg, void *param)
{
    switch (CFW_MESSAGE_ID(msg)) {

    case MSG_ID_CFW_SVC_AVAIL_EVT:
        handle_msg_id_cfw_svc_avail_evt((cfw_svc_available_evt_msg_t *)msg, param);
        break;

    case MSG_ID_CFW_OPEN_SERVICE:
        handle_msg_id_cfw_open_svc((cfw_open_conn_rsp_msg_t *)msg, param);
        break;

    case MSG_ID_BLE_GAP_WR_CONF_RSP:
        handle_msg_id_ble_gap_wr_conf_rsp((struct ble_rsp *)msg, param);
        break;

    case MSG_ID_BLE_GAP_RD_BDA_RSP:
        handle_msg_id_ble_gap_rd_bda_rsp((ble_bda_rd_rsp_t *)msg, param);
        break;

    case MSG_ID_BLE_GAP_SM_CONFIG_RSP:
        handle_msg_id_ble_gap_sm_config_rsp((struct ble_rsp *)msg, param);
        break;

    case MSG_ID_BLE_GAP_WR_ADV_DATA_RSP:
        handle_msg_id_ble_gap_wr_adv_data_rsp((struct ble_rsp *)msg, param);
        break;

    case MSG_ID_BLE_GAP_ENABLE_ADV_RSP:
        handle_msg_id_ble_gap_enable_adv_rsp((struct ble_rsp *)msg, param);
        break;

    case MSG_ID_BLE_GAP_DISABLE_ADV_RSP:
        handle_msg_id_ble_gap_disable_adv_rsp((struct ble_rsp *)msg, param);
        break;

    case MSG_ID_BLE_GATTS_ADD_SERVICE_RSP:
        handle_msg_id_gatts_add_service_rsp((struct ble_gatts_add_svc_rsp *)msg, param);
        break;

    case MSG_ID_BLE_GATTS_ADD_CHARACTERISTIC_RSP:
        handle_msg_id_gatts_add_characteristic_rsp((struct ble_gatts_add_char_rsp *)msg, param);
        break;

    case MSG_ID_BLE_GATTS_ADD_DESCRIPTOR_RSP:
        handle_msg_id_gatts_add_desc_rsp((struct ble_gatts_add_desc_rsp *)msg, param);
        break;

    case MSG_ID_BLE_GATTS_SET_ATTRIBUTE_VALUE_RSP:
        handle_msg_id_ble_gatts_set_attribute_value_rsp((struct ble_gatts_set_attr_rsp_msg *)msg, param);
        break;

    case MSG_ID_BLE_GATTS_SEND_NOTIF_RSP:
    case MSG_ID_BLE_GATTS_SEND_IND_RSP:
        handle_msg_id_ble_gatts_send_notif_ind_rsp((ble_gatts_rsp_t *)msg, param);
        break;

    case MSG_ID_BLE_GAP_CONNECT_EVT:
        handle_msg_id_ble_gap_connect_evt_msg((struct ble_gap_event *)msg, param);
        break;

    case MSG_ID_BLE_GAP_DISCONNECT_EVT:
        handle_msg_id_ble_gap_disconnect_evt_msg((struct ble_gap_event *)msg, param);
        break;

    case MSG_ID_BLE_GAP_TO_EVT:
        handle_msg_id_ble_gap_timeout_evt_msg((struct ble_gap_event *)msg, param);
        break;

    case MSG_ID_BLE_GAP_RSSI_EVT:
        handle_msg_id_ble_gap_rssi_evt_msg((struct ble_gap_event *)msg, param);
        break;

    case MSG_ID_BLE_GATTS_WRITE_EVT:
        handle_msg_id_ble_gatts_write_evt_msg((struct ble_gatts_evt_msg *)msg, param);
        break;

    case MSG_ID_BLE_GAP_DISCONNECT_RSP:
        handle_msg_id_ble_gap_disconnect_rsp((struct ble_rsp *)msg, param);
        break;

    case MSG_ID_BLE_GAP_SET_RSSI_REPORT_RSP:
        handle_msg_id_ble_gap_set_rssi_report_rsp((struct ble_rsp *)msg, param);
        break;

    case MSG_ID_BLE_GAP_DTM_INIT_RSP:
        handle_msg_id_ble_gap_dtm_init_rsp((struct ble_generic_msg *)msg, param);
        break;
    }
    cfw_msg_free(msg);
}

#ifdef __cplusplus
extern "C" {
#endif

#define NIBBLE_TO_CHAR(n) \
    ((n) >= 0xA ? ('A' + (n) - 0xA) : ('0' + (n)))

#define BYTE_TO_STR(s, byte)               \
    do {                                   \
        *s++ = NIBBLE_TO_CHAR(byte >> 4);  \
        *s++ = NIBBLE_TO_CHAR(byte & 0xF); \
    }while(0)

void ble_client_get_factory_config(ble_addr_t *bda, char *name)
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
                    bda->addr[i] = p_oem->bt_address[BLE_ADDR_LEN - 1 - i];
            }
        }
    }

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

BleStatus ble_client_init(ble_client_gap_event_cb_t gap_event_cb, void *gap_event_param,
                          ble_client_gatts_event_cb_t gatts_event_cb, void *gatts_event_param)
{
    BleStatus status;
    uint32_t delay_until;

    cfw_platform_nordic_init();

    client_handle = cfw_init(cfw_get_service_queue(),
                             ble_core_client_handle_message,
                             NULL);

    sync.response = 0;
    if (cfw_register_svc_available(client_handle,
                                   BLE_CORE_SERVICE_ID,
                                   NULL))
        return BLE_STATUS_ERROR;

    /* Wait for response messages */
    wait_for_condition(sync.response, status);
    if (status != BLE_STATUS_SUCCESS)
        return status;

    /* We need to wait for ~1 ms before continuing */
    delay_until = get_uptime_32k() + TIMEOUT_TICKS_1MS;
    while (get_uptime_32k() < delay_until);

    sync.response = 0;
    cfw_open_service(client_handle,
                     BLE_CORE_SERVICE_ID,
                     NULL);

    /* Wait for response messages */
    wait_for_condition(sync.response, status);
    if (status != BLE_STATUS_SUCCESS)
        return status;

    ble_client_gap_event_cb = gap_event_cb;
    ble_client_gap_event_param = gap_event_param;

    ble_client_gatts_event_cb = gatts_event_cb;
    ble_client_gatts_event_param = gatts_event_param;

    return sync.status;
}

BleStatus ble_client_gap_set_enable_config(const char *name,
                                           const ble_addr_t *bda,
                                           const uint16_t appearance,
                                           const int8_t tx_power)
{
    struct ble_wr_config config;
    BleStatus status;

    config.p_bda = (bda && bda->type != BLE_DEVICE_ADDR_INVALID) ? (ble_addr_t *)bda : NULL;
    config.p_name = (uint8_t *)name;
    config.appearance = appearance;
    config.tx_power = tx_power;
    config.peripheral_conn_params.interval_min = MIN_CONN_INTERVAL;
    config.peripheral_conn_params.interval_max = MAX_CONN_INTERVAL;
    config.peripheral_conn_params.slave_latency = SLAVE_LATENCY;
    config.peripheral_conn_params.link_sup_to = CONN_SUP_TIMEOUT;
    config.central_conn_params.interval_min = MIN_CONN_INTERVAL;
    config.central_conn_params.interval_max = MAX_CONN_INTERVAL;
    config.central_conn_params.slave_latency = SLAVE_LATENCY;
    config.central_conn_params.link_sup_to = CONN_SUP_TIMEOUT;

    sync.response = 0;
    if (ble_gap_set_enable_config(service_handle, &config, NULL))
        return BLE_STATUS_ERROR;
    /* Wait for response message */
    wait_for_condition(sync.response, status);
    if (status != BLE_STATUS_SUCCESS)
        return status;
    if (sync.status)
        return sync.status;

    struct ble_gap_sm_config_params sm_params = {
        .options = BLE_GAP_BONDING,
        .io_caps = BLE_GAP_IO_NO_INPUT_NO_OUTPUT,
        .key_size = 16,
    };
    sync.response = 0;
    if (ble_gap_sm_config(service_handle, &sm_params, NULL))
        return BLE_STATUS_ERROR;
    /* Wait for response message */
    wait_for_condition(sync.response, status);
    if (status != BLE_STATUS_SUCCESS)
        return status;

    return sync.status;
}

BleStatus ble_client_gap_get_bda(ble_addr_t *p_bda)
{
    BleStatus status;

    sync.response = 0;
    sync.param = (void *)p_bda;
    if (ble_gap_read_bda(service_handle, NULL))
        return BLE_STATUS_ERROR;
    /* Wait for response message */
    wait_for_condition(sync.response, status);
    if (status != BLE_STATUS_SUCCESS)
        return status;

    return sync.status;
}

BleStatus ble_client_gap_wr_adv_data(uint8_t *adv_data, const uint8_t adv_data_len)
{
    BleStatus status;

    struct ble_gap_adv_rsp_data adv_rsp_data = {
        .p_data = adv_data,
        .len = adv_data_len,
    };

    /* write advertisement data */
    sync.response = 0;
    if (ble_gap_wr_adv_data(service_handle, &adv_rsp_data, NULL, NULL))
        return BLE_STATUS_ERROR;

    /* Wait for response messages */
    wait_for_condition(sync.response, status);
    if (status != BLE_STATUS_SUCCESS)
        return status;

    return sync.status;
}

BleStatus ble_client_gap_start_advertise(uint16_t timeout)
{
    /* Hard-coding these advertising parameters for now
     * Could be changed to support advanced features such as:
     * - slow advertising
     * - directed advertising
     * - whitelist filtering
     * - etc.
     */
    ble_gap_adv_param_t adv_params = {
        .timeout = timeout,
        .interval_min = APP_ULTRA_FAST_ADV_INTERVAL,
        .interval_max = APP_ULTRA_FAST_ADV_INTERVAL,
        .type = BLE_GAP_ADV_TYPE_ADV_IND,
        .filter_policy = BLE_GAP_ADV_FP_ANY,
        .p_peer_bda = NULL,
        .options = BLE_GAP_OPT_ADV_DEFAULT,
    };

    /* For this message, we don't wait for the response, just fire
     * and forget.  This allows us to invoke it within the
     * disconnect event handler to restart the connection
     */
    return ble_gap_start_advertise(service_handle, &adv_params, NULL);
}

BleStatus ble_client_gap_stop_advertise(void)
{
    /* For this message, we don't wait for the response, just fire
     * and forget.
     */
    return ble_gap_stop_advertise(service_handle, NULL);
}

BleStatus ble_client_gatts_add_service(const struct bt_uuid *uuid,
				       const uint8_t type,
				       uint16_t *svc_handle)
{
    BleStatus status;

    sync.response = 0;
    sync.param = (void *)svc_handle;
    if (ble_gatts_add_service(service_handle, uuid, type, NULL, NULL))
        return BLE_STATUS_ERROR;

    /* Wait for response messages */
    wait_for_condition(sync.response, status);
    if (status != BLE_STATUS_SUCCESS)
        return status;

    return sync.status;
}

BleStatus ble_client_gatts_include_service(const uint16_t primary_svc_handle,
					   const uint16_t included_svc_handle)
{
    BleStatus status;

    sync.response = 0;
    if (ble_gatts_add_included_svc(service_handle,
				   primary_svc_handle,
				   included_svc_handle,
				   NULL))
        return BLE_STATUS_ERROR;

    /* Wait for response messages */
    wait_for_condition(sync.response, status);
    if (status != BLE_STATUS_SUCCESS)
        return status;

    return sync.status;
}

BleStatus ble_client_gatts_add_characteristic(const uint16_t svc_handle,
                                              struct ble_gatts_characteristic *char_data,
                                              struct ble_gatts_char_handles *handles)
{
    BleStatus status;

    sync.response = 0;
    sync.param = (void *)handles;

    if (ble_gatts_add_characteristic(service_handle, svc_handle, char_data,
                                     NULL, NULL))
        return BLE_STATUS_ERROR;

    /* Wait for response messages */
    wait_for_condition(sync.response, status);
    if (status != BLE_STATUS_SUCCESS)
        return status;

    return sync.status;
}

BleStatus ble_client_gatts_add_descriptor(const uint16_t svc_handle,
                                          struct ble_gatts_descriptor *desc,
                                          uint16_t *handle)
{
    BleStatus status;

    sync.response = 0;
    sync.param = (void *)handle;

    if (ble_gatts_add_descriptor(service_handle, desc, NULL, NULL))
        return BLE_STATUS_ERROR;

    /* Wait for response messages */
    wait_for_condition(sync.response, status);
    if (status != BLE_STATUS_SUCCESS)
        return status;

    return sync.status;
}

BleStatus ble_client_gatts_set_attribute_value(const uint16_t value_handle,
					       const uint16_t len, const uint8_t * p_value,
					       const uint16_t offset)
{
    BleStatus status;

    sync.response = 0;
    if (ble_gatts_set_attribute_value(service_handle, value_handle,
				      len, p_value, offset, NULL))
        return BLE_STATUS_ERROR;

    /* Wait for response messages */
    wait_for_condition(sync.response, status);
    if (status != BLE_STATUS_SUCCESS)
        return status;

    return sync.status;
}

BleStatus ble_client_gatts_send_notif_ind(const uint16_t value_handle,
                                          const uint16_t len, uint8_t * p_value,
                                          const uint16_t offset,
                                          const bool indication)
{
    BleStatus status;

    ble_gatts_ind_params_t ind_params = {
        .val_handle = value_handle,
        .len = len,
        .p_data = p_value,
        .offset = offset,
    };

    sync.response = 0;
    if (indication)
        status = ble_gatts_send_ind(service_handle, conn_handle, &ind_params, NULL, NULL);
    else
        status = ble_gatts_send_notif(service_handle, conn_handle, &ind_params, NULL, NULL);

    if (status)
        return BLE_STATUS_ERROR;

    /* Wait for response messages */
    wait_for_condition(sync.response, status);
    if (status != BLE_STATUS_SUCCESS)
        return status;

    return sync.status;
}

BleStatus ble_client_gap_disconnect(const uint8_t reason)
{
    BleStatus status;

    if (!connected)
        return BLE_STATUS_WRONG_STATE;

    sync.response = 0;
    if (ble_gap_disconnect(service_handle, conn_handle, reason, NULL))
        return BLE_STATUS_ERROR;

    /* Wait for response messages */
    wait_for_condition(sync.response, status);
    if (status != BLE_STATUS_SUCCESS)
        return status;

    return sync.status;
}

BleStatus ble_client_gap_set_rssi_report(boolean_t enable)
{
    BleStatus status;
    struct rssi_report_params params;

    if (!connected)
        return BLE_STATUS_WRONG_STATE;

    params.conn_hdl = conn_handle;
    params.op = enable ? BLE_GAP_RSSI_ENABLE_REPORT : BLE_GAP_RSSI_DISABLE_REPORT;
    /* TODO - pick sensible defaults for these and/or allow user to specify */
    params.delta_dBm = 5;
    params.min_count = 3;

    sync.response = 0;
    if (ble_gap_set_rssi_report(service_handle, &params, NULL))
        return BLE_STATUS_ERROR;

    /* Wait for response messages */
    wait_for_condition(sync.response, status);
    if (status != BLE_STATUS_SUCCESS)
        return status;

    return sync.status;
}

BleStatus ble_client_dtm_init(void)
{
    BleStatus status;

    /* Ensure that the ble_client_init() has been called already */
    if (!service_handle)
        return BLE_STATUS_WRONG_STATE;

    /* Instruct the Nordic to enter Direct Test Mode */
    sync.response = 0;
    ble_gap_dtm_init_req(service_handle, NULL);

    /* Wait for response messages */
    wait_for_condition(sync.response, status);
    if (status != BLE_STATUS_SUCCESS)
        return status;

    /* DTM is active. Detach UART IPC driver to allow direct access */
	if (BLE_STATUS_SUCCESS == sync.status)
        uart_ipc_disable(IPC_UART);

    return sync.status;
}

static int uart_raw_ble_core_tx_rx(uint8_t * send_data, uint8_t send_no,
			       uint8_t * rcv_data, uint8_t rcv_no)
{
	int i;
	uint8_t rx_byte;
	int res;
	/* send command */
	for (i = 0; i < send_no; i++)
		uart_poll_out(IPC_UART, send_data[i]);
	/* answer */
	i = 0;
	do {
		res = uart_poll_in(IPC_UART, &rx_byte);
		if (res == 0) {
			rcv_data[i++] = rx_byte;
		}
	} while (i < rcv_no);
	return i;
}

BleStatus ble_client_dtm_cmd(const struct ble_test_cmd *test_cmd,
                             struct ble_dtm_test_result *test_result)
{
    BleStatus status;

	uint8_t send_data[7];
	uint8_t rcv_data[9] = {};
	int send_no;
	int rcv_no;

	send_data[0] = DTM_HCI_CMD;
	send_data[1] = test_cmd->mode;
	send_data[2] = DTM_HCI_OPCODE2;

	switch (test_cmd->mode) {
	case BLE_TEST_START_DTM_RX:
		send_data[3] = 1;	/* length */
		send_data[4] = test_cmd->rx.freq;
		send_no = 5;
		rcv_no = 7;
		break;
	case BLE_TEST_START_DTM_TX:
		send_data[3] = 3;	/* length */
		send_data[4] = test_cmd->tx.freq;
		send_data[5] = test_cmd->tx.len;
		send_data[6] = test_cmd->tx.pattern;
		send_no = 7;
		rcv_no = 7;
		break;
	case BLE_TEST_SET_TXPOWER:
		send_data[3] = 1;	/* length */
		send_data[4] = test_cmd->tx_pwr.dbm;
		send_no = 5;
		rcv_no = 7;
		break;
	case BLE_TEST_END_DTM:
		send_data[3] = 0;	/* length */
		send_no = 4;
		rcv_no = 9;
		break;
	default:
		return BLE_STATUS_NOT_SUPPORTED;
	}

	uart_raw_ble_core_tx_rx(send_data, send_no, rcv_data, rcv_no);

	status = rcv_data[DTM_HCI_STATUS_IDX];

    test_result->mode = test_cmd->mode;

	uint8_t *p;
	switch (test_cmd->mode) {
	case BLE_TEST_END_DTM:
		p = &rcv_data[DTM_HCI_LE_END_IDX];
		LESTREAM_TO_UINT16(p, test_result->nb);
		break;
	}

    return status;
}

#ifdef __cplusplus
}
#endif
