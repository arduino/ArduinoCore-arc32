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

#ifndef GATT_INTERNAL_H_
#define GATT_INTERNAL_H_

#include <stdint.h>
#include <bluetooth/gatt.h>


#define BLE_GATT_MTU_SIZE 23

/*
 * Internal APIs used between host and BLE controller
 * Typically they are required if gatt.h APIs can not be mapped 1:1 onto controller API
 */

/*
 * GATT Attribute stream structure.
 */
struct nble_gatts_attr {
	/* Attribute pointer */
	struct bt_gatt_attr *attr;
	/* Attribute permissions */
	uint16_t perm;
	/* Attribute variable data size */
	uint16_t data_size;
	/* Attribute variable data: always starts with the UUID and data follows */
	uint8_t data[];
};

struct nble_gatts_register_req {
	/* Base address of the attribute table in the Quark mem space */
	struct bt_gatt_attr *attr_base;
	/* Number of of attributes in this service */
	uint8_t attr_count;
};

void nble_gatts_register_req(const struct nble_gatts_register_req *req,
			     uint8_t *data, uint16_t len);

struct nble_gatts_register_rsp {
	int status;
	struct bt_gatt_attr *attr_base;
	/* Number of attributes successfully added */
	uint8_t attr_count;
};

struct nble_gatt_attr_handles {
	uint16_t handle;
};

void on_nble_gatts_register_rsp(const struct nble_gatts_register_rsp *rsp,
				const struct nble_gatt_attr_handles *attrs,
				uint8_t len);

enum nble_gatt_wr_flag {
	NBLE_GATT_WR_FLAG_REPLY	= 1,
	NBLE_GATT_WR_FLAG_PREP	= 2,
};

struct nble_gatts_write_evt {
	struct bt_gatt_attr *attr;
	uint16_t conn_handle;
	uint16_t offset;
	/* see nble_gatt_wr_flag */
	uint8_t flag;
};

void on_nble_gatts_write_evt(const struct nble_gatts_write_evt *evt,
			     const uint8_t *data, uint8_t len);

struct nble_gatts_write_reply_req {
	uint16_t conn_handle;
	uint16_t offset;
	int32_t status;
};

void nble_gatts_write_reply_req(const struct nble_gatts_write_reply_req *req,
				const uint8_t *data, uint8_t len);

struct nble_gatts_write_exec_evt {
	uint16_t conn_handle;
	uint8_t flag;
};

void on_nble_gatts_write_exec_evt(const struct nble_gatts_write_exec_evt *evt);

struct nble_gatts_read_evt {
	struct bt_gatt_attr *attr;
	uint16_t conn_handle;
	uint16_t offset;
};

void on_nble_gatts_read_evt(const struct nble_gatts_read_evt *evt);

struct nble_gatts_read_reply_req {
	uint16_t conn_handle;
	uint16_t offset;
	int32_t status;
};

void nble_gatts_read_reply_req(const struct nble_gatts_read_reply_req *req,
			       uint8_t *data, uint16_t len);

struct nble_gatts_value_change_param {
	const struct bt_gatt_attr *attr;
	uint16_t conn_handle;
	uint16_t offset;
};

struct nble_gatts_notify_req {
	/* Function to be invoked when buffer is freed */
	bt_gatt_notify_sent_func_t cback;
	struct nble_gatts_value_change_param params;
};

void nble_gatts_notify_req(const struct nble_gatts_notify_req *req,
			   const uint8_t *data, uint16_t len);

struct nble_gatts_notify_tx_evt {
	bt_gatt_notify_sent_func_t cback;
	uint8_t status __attribute__ ((aligned (4)));
	uint16_t conn_handle __attribute__ ((aligned (4)));
	struct bt_gatt_attr *attr;
};

void on_nble_gatts_notify_tx_evt(const struct nble_gatts_notify_tx_evt *evt);

struct nble_gatts_indicate_req {
	/* Function to be invoked when buffer is freed */
	bt_gatt_indicate_func_t cback;
	struct nble_gatts_value_change_param params;
};

void nble_gatts_indicate_req(const struct nble_gatts_indicate_req *req,
			     const uint8_t *data, uint8_t len);

struct nble_gatts_indicate_rsp {
	bt_gatt_indicate_func_t cback;
	struct bt_gatt_attr *attr;
	uint8_t status;
	uint16_t conn_handle;
};

void on_nble_gatts_indicate_rsp(const struct nble_gatts_indicate_rsp *rsp);

#define DISCOVER_FLAGS_UUID_PRESENT 1

struct nble_gatt_handle_range {
	uint16_t start_handle;
	uint16_t end_handle;
};

struct nble_gattc_discover_req {
	void *user_data;
	struct bt_uuid_128 uuid;
	struct nble_gatt_handle_range handle_range;
	uint16_t conn_handle;
	uint8_t type;
	uint8_t flags;
};

void nble_gattc_discover_req(const struct nble_gattc_discover_req *req);

struct nble_gattc_primary {
	uint16_t handle;
	struct nble_gatt_handle_range range;
	struct bt_uuid_128 uuid;
};

struct nble_gattc_included {
	uint16_t handle;
	struct nble_gatt_handle_range range;
	struct bt_uuid_128 uuid;
};

struct nble_gattc_characteristic {
	uint16_t handle;
	uint8_t prop;
	uint16_t value_handle;
	struct bt_uuid_128 uuid;
};

struct nble_gattc_descriptor {
	uint16_t handle;
	struct bt_uuid_128 uuid;
};

struct nble_gattc_discover_rsp {
	uint8_t status;
	void *user_data;
	uint16_t conn_handle;
	uint8_t type;
};

void on_nble_gattc_discover_rsp(const struct nble_gattc_discover_rsp *rsp,
				const uint8_t *data, uint8_t len);

struct nble_gattc_read_req {
	void *user_data;
	uint16_t conn_handle;
	uint16_t handle;
	uint16_t offset;
};

void nble_gattc_read_req(const struct nble_gattc_read_req *req);

struct nble_gattc_read_rsp {
	uint8_t status;
	void *user_data;
	uint16_t conn_handle;
	uint16_t handle;
	uint16_t offset;
};

void on_nble_gattc_read_rsp(const struct nble_gattc_read_rsp *rsp,
			    uint8_t *data, uint8_t len);

struct nble_gattc_read_multi_req {
	void *user_data;
	uint16_t conn_handle;
};

void nble_gattc_read_multi_req(const struct nble_gattc_read_multi_req *req,
			       const uint16_t *handles, uint16_t len);

void on_nble_gattc_read_multi_rsp(const struct nble_gattc_read_rsp *rsp,
				  uint8_t *data, uint8_t len);

struct nble_gattc_write_param;

typedef void (*bt_att_func_t)(struct bt_conn *conn, uint8_t err,
				  const struct nble_gattc_write_param *par);

struct nble_gattc_write_param {
	/* Function invoked upon write response */
	bt_att_func_t func;
	/* User specific data */
	void *user_data[2];
};

struct nble_gattc_write_req {
	uint16_t conn_handle;
	uint16_t handle;
	uint16_t offset;
	/* different than 0 if response required */
	uint8_t with_resp;
	struct nble_gattc_write_param wr_params;
};

void nble_gattc_write_req(const struct nble_gattc_write_req *req,
			  const uint8_t *data, uint16_t len);

struct nble_gattc_write_rsp {
	uint8_t status;
	uint16_t conn_handle;
	uint16_t handle;
	struct nble_gattc_write_param wr_params;
};

void on_nble_gattc_write_rsp(const struct nble_gattc_write_rsp *rsp);

void bt_gatt_connected(struct bt_conn *conn);
void bt_gatt_disconnected(struct bt_conn *conn);

enum NBLE_GATTC_EVT {
	NBLE_GATTC_EVT_NOTIFICATION,
	NBLE_GATTC_EVT_INDICATION,
};

struct nble_gattc_value_evt {
	int status;
	uint16_t conn_handle;
	uint16_t handle;
	/* see NBLE_GATTC_VALUE_EVT */
	uint8_t type;
};

void on_nble_gattc_value_evt(const struct nble_gattc_value_evt *evt,
			     uint8_t *data, uint8_t len);

void *nble_curie_alloc_hook(uint32_t size);
void nble_curie_free_hook(void *buffer);
#endif /* GATT_INTERNAL_H_ */
