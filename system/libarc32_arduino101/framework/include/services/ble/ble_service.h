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

#ifndef __BLE_SERVICE_H__
#define __BLE_SERVICE_H__

#include <stdint.h>

/* For MSG_ID_BLE_SERVICE_BASE */
#include "services/services_ids.h"

/* For bt_uuid  */
#include "bluetooth/gatt.h"

#include "bluetooth/bluetooth.h"

#include "cfw/cfw.h"

/* Forward declarations */
struct _ble_service_cb;
struct bt_conn;

/**
 * @cond
 * @defgroup ble_service BLE Service
 *
 * Bluetooth Low Energy (BLE) application level service.
 *
 * This service provides BLE service, abstracting most of the complexity of the underlying BLE services/profiles.
 *
 * @ingroup services
 *
 * @{
 */

/*
 * CFW Message ID base definitions for BLE services.
 */

/* BLE Service Message ID definitions. */
#define MSG_ID_BLE_SERVICE_RSP                      (MSG_ID_BLE_SERVICE_BASE + 0x40)
#define MSG_ID_BLE_SERVICE_EVT                      (MSG_ID_BLE_SERVICE_BASE + 0x80)

/** BLE High level Message IDs used for request, response, events. */
enum BLE_MSG_ID {
	MSG_ID_BLE_ENABLE_RSP = MSG_ID_BLE_SERVICE_RSP, /**< Message ID for <i>enable</i> response, of type @ref ble_enable_rsp */
	MSG_ID_BLE_INIT_SVC_RSP,                /**< Message ID for <i>init service</i> response, of type @ref ble_init_svc_rsp */

	/* BLE direct test mode command */
	MSG_ID_BLE_DBG_RSP,                     /**< Message ID for <i>DTM command</i> response, of type @ref ble_dbg_req_rsp */

	MSG_ID_BLE_SERVICE_RSP_LAST,

	/* events */
	MSG_ID_BLE_ADV_TO_EVT = MSG_ID_BLE_SERVICE_EVT,           /**< Message ID for struct @ref ble_adv_to_evt */
	MSG_ID_BLE_SERVICE_EVT_LAST
};

/** Macro to convert milliseconds to a specific unit */
#define MSEC_TO_1_25_MS_UNITS(TIME) (((TIME) * 1000) / 1250)
#define MSEC_TO_10_MS_UNITS(TIME) ((TIME) / 10)

#define BLE_GAP_SEC_RAND_LEN    8		/**< Random Security number length (64 bits) */
#define BLE_GAP_SEC_MAX_KEY_LEN 16		/**< Maximum security key len (LTK, CSRK) */

/**
 * Advertisement options.
 */
enum BLE_GAP_ADV_OPTIONS {
	BLE_GAP_OPT_ADV_DEFAULT = 0,		/**< no specific option */
	BLE_GAP_OPT_ADV_WHITE_LISTED = 0x02	/**< use white list and only report whitelisted devices */
};

/**
 * LE security modes.
 *
 * see BT spec  PART C, 10.2
 *
 * - Security mode 1
 *   - Level 1: No security at all (service may use data signing)
 *   - Level 2: Unauthenticated (no MITM protection pairing with encryption
 *   - Level 3: Authenticated (MITM protection) pairing with encryption
 *   - Level 4: Authenticated (MITM protection) LE Secure Connection wi
 *
 * - Security mode 2 (data signing)
 *   - Level 1: Unauthenticated pairing with data signing
 *   - Level 2: Authenticated (MITM protection) with data signing
 */
enum BLE_GAP_SEC_MODES {
	GAP_SEC_NO_PERMISSION = 0,	/**< No access permitted. */
	GAP_SEC_LEVEL_1,
	GAP_SEC_LEVEL_2,
	GAP_SEC_LEVEL_3,
	GAP_SEC_LEVEL_4,
	GAP_SEC_MODE_1 = 0x10,
	GAP_SEC_MODE_2 = 0x20		/**< only used for data signing, level 1 or 2 */
};

/**
 * Security manager passkey type.
 */
enum BLE_GAP_SM_PASSKEY_TYPE {
	BLE_GAP_SM_PK_NONE = 0, /**< No key (may be used to reject). */
	BLE_GAP_SM_PK_PASSKEY,  /**< Security data is a 6-digit passkey. */
	BLE_GAP_SM_PK_OOB,         /**< Security data is 16 bytes of OOB data */
};

/**
 * Connection Parameter update request event.
 */
struct ble_gap_conn_param_update_req_evt {
	struct bt_le_conn_param param;
};

/* -  BLE_SERVICE_GAP_API.H */


/** Generic BLE status response message. */
struct ble_rsp {
	struct cfw_message header;	/**< Component framework message header (@ref cfw), MUST be first element of structure */
	int status;			/**< Response status */
};

/** Generic BLE response with connection reference and status. */
struct ble_conn_rsp {
	struct cfw_message header; /**< Component framework message header (@ref cfw), MUST be first element of structure */
	struct bt_conn *conn;      /**< Connection reference */
	int status;                /**< Status */
};

/** BLE Enable configuration options. */
struct ble_enable_config {
	bt_addr_le_t * p_bda;  /**< Optional BT device address. If NULL, internal unique static random will be used */
	struct bt_le_conn_param central_conn_params;    /**< Central supported range */
};

/** Parameters of MSG_ID_BLE_ENABLE_RSP. */
struct ble_enable_rsp {
	struct cfw_message header; /**< Component framework message header (@ref cfw), MUST be first element of structure */
	int status;                /**< Response status */
	uint8_t enable;            /**< Enable state: 0:Disabled, 1:Enabled */
	bt_addr_le_t bd_addr;
};

/**
 * Attribute handle range definition.
 */
struct ble_gatt_handle_range {
	uint16_t start_handle;
	uint16_t end_handle;
};

/** Parameters of the current connection. */
struct ble_connection_values {
	uint16_t interval;		/**< Connection interval (unit 1.25 ms) */
	uint16_t latency;		/**< Connection latency (unit interval) */
	uint16_t supervision_to;	/**< Connection supervision timeout (unit 10ms)*/
};

/** Parameters for @ref MSG_ID_BLE_INIT_SVC_RSP. */
struct ble_init_svc_rsp {
	struct cfw_message header; /**< Component framework message header (@ref cfw), MUST be first element of structure */
	int status;
};

/** Authentication data. */
struct ble_auth_data {
	union {
		uint8_t passkey[6];   /**< 6 digit key (000000 - 999999) */
		uint8_t obb_data[16]; /**< 16 byte of OBB data */
	};
	uint8_t type;                 /**< @ref BLE_GAP_SM_PASSKEY_TYPE */
};

/** Parameters for @ref MSG_ID_BLE_ADV_TO_EVT. */
struct ble_adv_to_evt {
	struct cfw_message header; /**< Component framework message header (@ref cfw), MUST be first element of structure */
};

/**
 * BLE debug rsp message.
 */

/*
 * BLE debug req message.
 */
struct ble_dbg_req_rsp {
	struct cfw_message header;
	uint32_t u0;
	uint32_t u1;
};

/** Enable/Disable BLE stack. To be called before any BLE service related call.
 *
 * @param p_service_conn client service connection (cfw service connection)
 * @param enable 1: enable BLE stack 0: disable BLE stack
 * @param p_config configuration parameters when enabling BLE. shall be null in case of BLE disable. @ref ble_enable_config
 * @param p_priv pointer to private structure returned in a response
 *
 * @return @ref OS_ERR_TYPE
 * @note Expected notification:
 *       - Message with @ref MSG_ID_BLE_ENABLE_RSP and type @ref ble_enable_rsp.
 */
int ble_service_enable(cfw_service_conn_t * p_service_conn, uint8_t enable,
			const struct ble_enable_config * p_config,
			void *p_priv);

/** @endcond */
/** @}*/
#endif
