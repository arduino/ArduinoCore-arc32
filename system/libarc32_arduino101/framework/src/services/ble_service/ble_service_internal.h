/*
 * Copyright (c) 2016, Intel Corporation. All rights reserved.
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

#ifndef __BLE_SERVICE_INTERNAL_H__
#define __BLE_SERVICE_INTERNAL_H__

// For MSG_ID_BLE_SERVICE_BASE
#include "services/services_ids.h"

/* BLE High level Message IDs used for request, response, events. */
enum BLE_MSG_ID_REQ {
	MSG_ID_BLE_ENABLE_REQ = MSG_ID_BLE_SERVICE_BASE,  /* Message ID for enable request */
	MSG_ID_BLE_SET_NAME_REQ,                  /* Message ID for set name request */
	MSG_ID_BLE_START_SCAN_REQ,            /* Message ID for start scan request */
	MSG_ID_BLE_STOP_SCAN_REQ,             /* Message ID for stop scan request */
	MSG_ID_BLE_INIT_SVC_REQ,                  /* Message ID for init service request */

	/* BLE debug command */
	MSG_ID_BLE_DBG_REQ,                       /* Message ID for BLE debug command request */

	MSG_ID_BLE_REQ_LAST,
};

/* Parameters of MSG_ID_BLE_ENABLE_REQ. */
struct ble_enable_req {
	struct cfw_message header; /* Component framework message header, MUST be first element of structure */
	struct bt_le_conn_param central_conn_params;
	uint8_t enable;
	uint8_t bda_present;
	bt_addr_le_t bda;
};

struct ble_disconnect_req {
	struct cfw_message header; /* Component framework message header, MUST be first element of structure */
	struct bt_conn *conn;      /* Connection reference */
	uint8_t reason;            /* Reason of the disconnect*/
};

struct ble_connect_req {
	struct cfw_message header; /* Component framework message header, MUST be first element of structure */
	struct bt_le_conn_param conn_params;
	bt_addr_le_t bd_addr;
};

struct ble_init_svc_req {
	struct cfw_message header; /* Component framework message header, MUST be first element of structure */
	int (*init_svc)(struct ble_init_svc_req *msg, struct _ble_service_cb *p_cb);
	void (*init_svc_complete)(struct ble_init_svc_req *req);
	uint8_t status;
};

#endif /* __BLE_SERVICE_INTERNAL_H__ */
