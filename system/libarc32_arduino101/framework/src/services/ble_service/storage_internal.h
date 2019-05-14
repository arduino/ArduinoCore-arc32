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

#ifndef STORAGE_INTERNAL_H_
#define STORAGE_INTERNAL_H_

struct nble_storage_read_evt {
	bt_addr_le_t            addr;
	/* valid only for BT_STORAGE_ADDRESSES */
	uint8_t                 max_num_keys;
	uint16_t                key;
};

enum {
	NBLE_STORAGE_UPDATE     = BIT(0), /* key update */
};

struct nble_storage_write_evt {
	uint8_t                 flags;
	bt_addr_le_t            addr;
	uint16_t                key;
};

struct nble_storage_read_rsp_req {
	int                     status; /* posix status code */
	bt_addr_le_t            addr;
	uint16_t                key;
};

void on_nble_storage_read_evt(const struct nble_storage_read_evt *evt);

void on_nble_storage_write_evt(const struct nble_storage_write_evt *evt,
			       const uint8_t *data, uint16_t len);

void nble_storage_read_rsp_req(const struct nble_storage_read_rsp_req *req,
			       const uint8_t *data, uint16_t len);

#endif //STORAGE_INTERNAL_H_
