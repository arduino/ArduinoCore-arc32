/*
 * Copyright (c) 2016 Intel Corporation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <errno.h>

#include <bluetooth/hci.h>
#include <bluetooth/bluetooth.h>
#include <bluetooth/conn.h>

#include "smp.h"

/* nble internal APIs */
#include "gap_internal.h"

/* #define BT_GATT_DEBUG 1 */

extern void __assert_fail(void);
#ifdef BT_GATT_DEBUG
#define BT_DBG(fmt, ...) nble_curie_log_hook(fmt, ##__VA_ARGS__)
#define BT_ERR(fmt, ...) nble_curie_log_hook(fmt, ##__VA_ARGS__)
#define BT_WARN(fmt, ...) nble_curie_log_hook(fmt, ##__VA_ARGS__)
#define BT_INFO(fmt, ...) nble_curie_log_hook(fmt, ##__VA_ARGS__)
#define BT_ASSERT(cond) ((cond) ? (void)0 : __assert_fail())
#else
#define BT_DBG(fmt, ...) do {} while (0)
#define BT_ERR(fmt, ...) nble_curie_log_hook(fmt, ##__VA_ARGS__)
#define BT_WARN(fmt, ...) nble_curie_log_hook(fmt, ##__VA_ARGS__)
#define BT_INFO(fmt, ...) nble_curie_log_hook(fmt, ##__VA_ARGS__)
#define BT_ASSERT(cond) ((cond) ? (void)0 : __assert_fail())
#endif

/**
 * Compile time IO capabilities and OOB support settings for nble.
 *
 * This compile options must match the application registered callbacks
 * (bt_conn_auth_cb_register)
 */
#define NBLE_SMP_IO_CAPS BT_SMP_IO_NO_INPUT_OUTPUT
#define NBLE_SMP_AUTH_OPTIONS 0
#define NBLE_SMP_OBB_PRESENT BT_SMP_OOB_NOT_PRESENT


int bt_smp_init(void)
{
	return 0;
}

void on_nble_sm_bond_info_rsp(const struct nble_sm_bond_info_rsp *par,
			      const bt_addr_le_t *peer_addr, uint16_t len)
{
	/* stub */
}

void on_nble_sm_passkey_req_evt(const struct nble_sm_passkey_req_evt *evt)
{
	/* stub */
}

void on_nble_sm_passkey_disp_evt(const struct nble_sm_passkey_disp_evt *evt)
{
	/* stub */
}

void on_nble_sm_status_evt(const struct nble_sm_status_evt *evt)
{
	/* stub */
}

void on_nble_sm_common_rsp(const struct nble_sm_common_rsp *rsp)
{
	if (rsp->status) {
		BT_WARN("gap sm request failed: %d", rsp->status);
	}
}

void on_nble_sm_config_rsp(struct nble_sm_config_rsp *rsp)
{
	if (rsp->status) {
		BT_ERR("sm_config failed: %d", rsp->status);
	}
}

void on_nble_sm_pairing_request_evt(const struct nble_sm_pairing_request_evt *evt)
{
    /* stub */
	(void)evt;
}

void on_nble_sm_security_request_evt(const struct nble_sm_security_request_evt *evt)
{
    /* stub */
	(void)evt;
}

