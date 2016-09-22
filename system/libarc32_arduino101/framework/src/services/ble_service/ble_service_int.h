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

#ifndef BLE_SERVICE_INT_H_
#define BLE_SERVICE_INT_H_

#include <stdint.h>

/* For cfw_service_conn_t, cfw_client_t, T_QUEUE */
#include "cfw/cfw.h"

/* Forward declarations */
struct ble_init_svc_req;
struct ble_init_svc_rsp;
struct bt_conn_cb;
struct bt_gatt_attr;

enum BLE_STATE {
	BLE_ST_NOT_READY = 0,
	BLE_ST_DISABLED,
	BLE_ST_ENABLED,
	BLE_ST_DTM
};

struct _ble_service_cb {
	T_QUEUE queue; /* Queue for the messages */
	uint8_t ble_state;
};

extern struct _ble_service_cb _ble_cb;

/** Send advertisement timeout event to application */
void ble_gap_advertisement_timeout(void);

#ifdef CONFIG_BLE_CORE_TEST
void test_ble_service_init(void);
T_QUEUE get_service_queue(void);
#endif

#endif /* BLE_SERVICE_INT_H_ */
