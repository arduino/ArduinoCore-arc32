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
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>

#include <bluetooth/dtm.h>
#include "dtm_internal.h"

extern void __assert_fail(void);
#define BT_ASSERT(cond) ((cond) ? (void)0 : __assert_fail())

void on_nble_dtm_rsp(const struct nble_dtm_rsp *rsp) {
	struct ble_dtm_rsp ret;

	ret.status = rsp->status;
	ret.nb = rsp->nb;

	rsp->func(&ret, rsp->user_data);
}

int ble_dtm_cmd(struct dtm_cmd_params *params) {

	struct nble_dtm_cmd_req req;

	if (!params || !params->func) {
		return -EINVAL;
	}

    memset(&req, 0, sizeof(req));
	switch(params->cmd_type) {
	case DTM_START_RX:
		req.tx_rx_freq = params->rx.freq;
		break;
	case DTM_START_TX:
		req.tx_rx_freq = params->tx.freq;
		req.tx_len = params->tx.len;
		req.tx_pattern = params->tx.pattern;
		break;
	case DTM_SET_TXPOWER:
		req.pwr_dbm = params->tx_pwr.dbm;
		break;
	case DTM_START_TX_CARRIER:
		req.tx_rx_freq = params->tx.freq;
		break;
	case DTM_END:
		break;
	default:
		return -EINVAL;
	}

	req.cmd_type = params->cmd_type;
	req.func = params->func;
	req.user_data = params->user_data;

	nble_dtm_cmd_req(&req);

	return 0;
}
