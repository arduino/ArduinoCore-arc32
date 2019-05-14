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

#ifndef DTM_INTERNAL_H_
#define DTM_INTERNAL_H_

#include <bluetooth/dtm.h>

struct nble_dtm_cmd_req {
	uint8_t cmd_type;
	uint8_t tx_rx_freq;
	uint8_t tx_len;
	uint8_t tx_pattern;
	int8_t pwr_dbm;
	dtm_rsp_func_t func;
	void *user_data;
};

void nble_dtm_cmd_req(const struct nble_dtm_cmd_req *req);

struct nble_dtm_rsp {
	int status;
	uint16_t nb;
	dtm_rsp_func_t func;
	void *user_data;
};

void on_nble_dtm_rsp(const struct nble_dtm_rsp *rsp);
#endif //DTM_INTERNAL_H_
