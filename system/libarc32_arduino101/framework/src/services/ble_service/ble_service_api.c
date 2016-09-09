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

#include "ble_service.h"

#include <errno.h>
#include <string.h>
//#include <atomic.h>
//#include "util/assert.h"

#include "ble_service_int.h"
#include "ble_service_internal.h"
#include "cfw/cfw_service.h"
#include "cfw/cfw_client.h"

#include <bluetooth/bluetooth.h>
#include <bluetooth/hci.h>

#include "ble_protocol.h"
#include "ble_service_utils.h"

int ble_service_enable(cfw_service_conn_t * p_service_conn, uint8_t enable,
			const struct ble_enable_config * p_config,
			void *p_priv)
{
	struct ble_enable_req * msg =
			(void *) cfw_alloc_message_for_service(p_service_conn,
				    MSG_ID_BLE_ENABLE_REQ,
				    sizeof(*msg), p_priv);
	msg->central_conn_params = p_config->central_conn_params;
	msg->enable = enable;

	if (p_config->p_bda) {
		msg->bda_present = 1;
		msg->bda = *p_config->p_bda;
	}

	return cfw_send_message(msg);
}
