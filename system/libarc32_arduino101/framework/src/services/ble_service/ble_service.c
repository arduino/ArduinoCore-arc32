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
#include "os/os.h"

//#include "util/assert.h"
#include <errno.h>
//#include <atomic.h>
#include "cfw/cfw_service.h"
#include "ble_protocol.h"
#include "ble_service_int.h"
#include "ble_service_internal.h"
#include "infra/log.h"

#include <bluetooth/bluetooth.h>
#include <bluetooth/gatt.h>
#include <bluetooth/conn.h>
#include <bluetooth/hci.h>
#include "gap_internal.h"

#include "nble_driver.h"

#include "rpc.h"

#include "util/misc.h"

struct _ble_service_cb _ble_cb = { 0 };
volatile uint8_t ble_inited = false;


static void ble_client_connected(conn_handle_t *instance);
static void ble_client_disconnected(conn_handle_t *instance);

#if defined(CONFIG_BLUETOOTH_SMP)
static const struct bt_conn_auth_cb auth_callbacks;
#endif

static service_t ble_service = {
	.service_id = BLE_SERVICE_ID,
	.client_connected = ble_client_connected,
	.client_disconnected = ble_client_disconnected,
};

static void ble_is_not_enabled_rsp(struct cfw_message *msg, int status)
{
	struct ble_enable_rsp *resp =
	    (struct ble_enable_rsp *)cfw_alloc_rsp_msg(msg,
			    /* translate msg from req to rsp */
						(CFW_MESSAGE_ID(msg) ^ MSG_ID_BLE_SERVICE_BASE)
						| MSG_ID_BLE_SERVICE_RSP,
						sizeof(*resp));
	resp->status = status;
	cfw_send_message(resp);
}

#ifdef CONFIG_TCMD_BLE_DEBUG
static void handle_msg_id_ble_dbg(struct cfw_message *msg)
{
	struct nble_debug_params params;
	struct ble_dbg_req_rsp *resp = (void *)
		cfw_alloc_rsp_msg(msg, MSG_ID_BLE_DBG_RSP, sizeof(*resp));
	struct ble_dbg_req_rsp *req = (struct ble_dbg_req_rsp *) msg;

	params.u0 = req->u0;
	params.u1 = req->u1;

	nble_gap_dbg_req(&params, resp);
}
#endif /* CONFIG_TCMD_BLE_DEBUG */

void on_nble_gap_dbg_rsp(const struct nble_debug_resp *params)
{
#ifdef CONFIG_TCMD_BLE_DEBUG
	struct ble_dbg_req_rsp *resp = params->user_data;
	if (!resp)
		return;
	resp->u0 = params->u0;
	resp->u1 = params->u1;
	cfw_send_message(resp);
#endif /* CONFIG_TCMD_BLE_DEBUG */
}


static void handle_msg_id_ble_rpc_callin(struct message *msg, void *priv)
{
	struct ble_rpc_callin *rpc = container_of(msg, struct ble_rpc_callin, msg);
	/* handle incoming message */
    //pr_debug(LOG_MODULE_BLE, "%s-%d", __FUNCTION__, __LINE__);
	rpc_deserialize(rpc->p_data, rpc->len);
	bfree(rpc->p_data);
	message_free(msg);
    //pr_debug(LOG_MODULE_BLE, "%s-%d", __FUNCTION__, __LINE__);
}

static void ble_set_bda_cb(int status, void *user_data)
{
	struct ble_enable_req *req = user_data;

	if (!req)
		return;

	struct ble_enable_rsp *resp = (void *)cfw_alloc_rsp_msg(&req->header,
			    MSG_ID_BLE_ENABLE_RSP, sizeof(*resp));
	resp->status = status;

	if (status == 0) {
		resp->enable = 1;

		nble_gap_read_bda_req(resp);
	} else {
		/* error case */
		resp->enable = 0;
		cfw_send_message(resp);
	}
	bfree(req);
}


void on_nble_gap_read_bda_rsp(const struct nble_service_read_bda_response *params)
{
	struct cfw_message *rsp = params->user_data;

	if (rsp) {
		struct ble_enable_rsp *r = container_of(rsp, struct ble_enable_rsp, header);
		r->bd_addr = params->bd;
		cfw_send_message(rsp);
	}
}

static void handle_ble_enable(struct ble_enable_req *req,
		struct _ble_service_cb *p_cb)
{
	pr_info(LOG_MODULE_BLE, "ble_enable: state %d", p_cb->ble_state);

	p_cb->ble_state = BLE_ST_ENABLED;

	if (req->bda_present) {
		struct nble_set_bda_params params;

		params.cb = ble_set_bda_cb;
		params.user_data = req;
		params.bda = req->bda;

		nble_set_bda_req(&params);
	} else {
		ble_set_bda_cb(0, req);
	}
}

void on_nble_set_bda_rsp(const struct nble_set_bda_rsp *params)
{
	if (params->cb) {
		params->cb(params->status, params->user_data);
	}
}

static void handle_ble_disable(struct ble_enable_req *req, struct _ble_service_cb *p_cb)
{
	struct ble_enable_rsp *resp;

	pr_debug(LOG_MODULE_BLE, "ble_disable");
	p_cb->ble_state = BLE_ST_DISABLED;

	bt_le_adv_stop();

	resp = (void *)cfw_alloc_rsp_msg(&req->header,
				MSG_ID_BLE_ENABLE_RSP,
				sizeof(*resp));
	cfw_send_message(resp);  // Sid. KW warning ack.

}

static void ble_service_message_handler(struct cfw_message *msg, void *param)
{
	bool free_msg = true;
	struct _ble_service_cb *p_cb = param;
	uint16_t msg_id = CFW_MESSAGE_ID(msg);

	if (p_cb->ble_state < BLE_ST_ENABLED &&
		msg_id != MSG_ID_BLE_ENABLE_REQ) {
		ble_is_not_enabled_rsp(msg, -ENODEV);
		goto out;
	}

	switch (msg_id) {
	case MSG_ID_BLE_ENABLE_REQ: {
		struct ble_enable_req *req =
			container_of(msg, struct ble_enable_req, header);
		if (p_cb->ble_state) {
			if (req->enable) {
				handle_ble_enable(req, p_cb);
				free_msg = false;
			} else
				handle_ble_disable(req, p_cb);
		} else {
			pr_debug(LOG_MODULE_BLE, "ble_hdl_msg: core service not opened!");
			/* core service is not yet up */
			struct ble_enable_rsp *resp = (void *)cfw_alloc_rsp_msg(msg,
				MSG_ID_BLE_ENABLE_RSP, sizeof(*resp));
			resp->status = -EINPROGRESS;
			resp->enable = 0;
			cfw_send_message(resp);  // Sid. KW warning ack.
		}
	}
		break;
#ifdef CONFIG_TCMD_BLE_DEBUG
	case MSG_ID_BLE_DBG_REQ:
		handle_msg_id_ble_dbg(msg);
		break;
#endif
	default:
		pr_warning(LOG_MODULE_BLE, "unsupported %d", msg_id);
		break;
	}
out:
	if (free_msg)
		cfw_msg_free(msg);
}

static void ble_client_connected(conn_handle_t *instance)
{
	if (_ble_cb.ble_state == BLE_ST_NOT_READY)
		pr_warning(LOG_MODULE_BLE, "BLE_CORE service is not registered");
}

static void ble_client_disconnected(conn_handle_t *instance)
{
}

void ble_bt_rdy(int err)
{
	_ble_cb.ble_state = BLE_ST_DISABLED;
    ble_inited = true;

	/* register BLE service */
	if (cfw_register_service(_ble_cb.queue, &ble_service,
			ble_service_message_handler, &_ble_cb) == -1) {
		panic(0xb1eb1e);
	}
}

void ble_cfw_service_init(int service_id, T_QUEUE queue)
{
	_ble_cb.queue = queue;
	_ble_cb.ble_state = BLE_ST_NOT_READY;

#ifdef CONFIG_IPC_UART_NS16550
	nble_driver_configure(queue, handle_msg_id_ble_rpc_callin);
#endif

    ble_inited = false;

    bt_enable(ble_bt_rdy);
    do{}
    while (ble_inited == false);
}

void nble_log(const struct nble_log_s *param, char *buf, uint8_t buflen)
{
	pr_info(LOG_MODULE_BLE, buf, param->param0, param->param1, param->param2, param->param3);
}

void on_nble_common_rsp(const struct nble_response *params)
{
	struct ble_rsp *resp = params->user_data;

	if (!resp)
		return;
	resp->status = params->status;
	cfw_send_message(resp);
}
