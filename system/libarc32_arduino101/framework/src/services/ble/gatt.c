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

#include <limits.h>
#include <string.h>
#include <errno.h>
#include <atomic.h>

#include <misc/byteorder.h>

#include <bluetooth/gatt.h>
#include "gatt_internal.h"
#include "hci_core.h"
#include "conn_internal.h"

/* #define BT_GATT_DEBUG 1 */

extern void on_nble_curie_log(char *fmt, ...);
extern void __assert_fail(void);
#ifdef BT_GATT_DEBUG
#define BT_DBG(fmt, ...) on_nble_curie_log(fmt, ##__VA_ARGS__)
#define BT_ERR(fmt, ...) on_nble_curie_log(fmt, ##__VA_ARGS__)
#define BT_WARN(fmt, ...) on_nble_curie_log(fmt, ##__VA_ARGS__)
#define BT_INFO(fmt, ...) on_nble_curie_log(fmt, ##__VA_ARGS__)
#define BT_ASSERT(cond) ((cond) ? (void)0 : __assert_fail())
#else
#define BT_DBG(fmt, ...) do {} while (0)
#define BT_ERR(fmt, ...) on_nble_curie_log(fmt, ##__VA_ARGS__)
#define BT_WARN(fmt, ...) on_nble_curie_log(fmt, ##__VA_ARGS__)
#define BT_INFO(fmt, ...) on_nble_curie_log(fmt, ##__VA_ARGS__)
#define BT_ASSERT(cond) ((cond) ? (void)0 : __assert_fail())
#endif

#define N_BLE_BUF_SIZE 512

struct ble_gatt_service {
	struct bt_gatt_attr *attrs; /* Pointer to the array of attributes */
	uint16_t attr_count; /* Number of attributes in the array */
};

struct ble_gatts_flush_all {
	struct bt_conn *conn;
	int status;
	uint8_t flag;
};

static struct ble_gatt_service db[CONFIG_BT_GATT_BLE_MAX_SERVICES];

static uint8_t db_cnt;

#if defined(CONFIG_BLUETOOTH_GATT_CLIENT)
static struct bt_gatt_subscribe_params *subscriptions;
#endif

/**
 * Copy a UUID in a buffer using the smallest memory length
 * @param buf Pointer to the memory where the UUID shall be copied
 * @param uuid Pointer to the UUID to copy
 * @return The length required to store the UUID in the memory
 */
static uint8_t bt_gatt_uuid_memcpy(uint8_t *buf,
				   const struct bt_uuid *uuid)
{
	uint8_t *ptr = buf;

	/* Store the type of the UUID */
	*ptr = uuid->type;
	ptr++;

	/* Store the UUID data */
	if (uuid->type == BT_UUID_TYPE_16) {
		uint16_t le16;
        
        memcpy(&le16, &BT_UUID_16(uuid)->val, sizeof(le16));
		le16 = sys_cpu_to_le16(le16);
		memcpy(ptr, &le16, sizeof(le16));
		ptr += sizeof(le16);
	} else {
		memcpy(ptr, BT_UUID_128(uuid)->val,
		       sizeof(BT_UUID_128(uuid)->val));
		ptr += sizeof(BT_UUID_128(uuid)->val);
	}
	return ptr - buf;
}

/* These attributes need the value to be read */
static struct bt_uuid *whitelist[] = {
	BT_UUID_GATT_PRIMARY,
	BT_UUID_GATT_SECONDARY,
	BT_UUID_GATT_INCLUDE,
	BT_UUID_GATT_CHRC,
	BT_UUID_GATT_CEP,
	BT_UUID_GATT_CUD,
	BT_UUID_GATT_CPF,
	BT_UUID_GAP_DEVICE_NAME,
	BT_UUID_GAP_APPEARANCE,
	BT_UUID_GAP_PPCP
};

static int attr_read(struct bt_gatt_attr *attr, uint8_t *data, size_t len)
{
	uint8_t i;
	int data_size;

	if (!data || len < 0) {
		return -ENOMEM;
	}

	data_size = bt_gatt_uuid_memcpy(data, attr->uuid);

	for (i = 0; i < ARRAY_SIZE(whitelist); i++) {
		if (!bt_uuid_cmp(attr->uuid, whitelist[i])) {
			int read;

			read = attr->read(NULL, attr, data + data_size, len,
					  0);
			if (read < 0) {
				return read;
			}

			data_size += read;
			break;
		}
	}

	return data_size;
}

int bt_gatt_register(struct bt_gatt_attr *attrs, size_t count)
{
	size_t attr_table_size, i;
	struct nble_gatt_register_req param;
	/* TODO: Replace the following with net_buf */
	uint8_t attr_table[N_BLE_BUF_SIZE];

	if (!attrs || !count) {
		return -EINVAL;
	}
	BT_ASSERT(db_cnt < ARRAY_SIZE(db));

	db[db_cnt].attrs = attrs;
	db[db_cnt].attr_count = count;
	db_cnt++;
	param.attr_base = attrs;
	param.attr_count = count;

	attr_table_size = 0;

	for (i = 0; i < count; i++) {
		struct bt_gatt_attr *attr = &attrs[i];
		struct ble_gatt_attr *att;
		int data_size;

		if (attr_table_size + sizeof(*att) > sizeof(attr_table)) {
			return -ENOMEM;
		}

		att = (void *)&attr_table[attr_table_size];
		att->perm = attr->perm;

		/* Read attribute data */
		data_size = attr_read(attr, att->data,
				      sizeof(attr_table) -
				      (attr_table_size + sizeof(*att)));
		if (data_size < 0) {
			return data_size;
		}
		att->data_size = data_size;

		BT_DBG("table size = %u attr data_size = %u", attr_table_size,
		       att->data_size);

		/* Compute the new element size and align it on upper 4 bytes
		 * boundary.
		 */
		attr_table_size += (sizeof(*att) + att->data_size + 3) & ~3;
	}

	nble_gatt_register_req(&param, attr_table, attr_table_size);
	return 0;
}

void on_nble_gatt_register_rsp(const struct nble_gatt_register_rsp *rsp,
		const struct nble_gatt_attr_handles *handles, uint8_t len)
{

	if (rsp->status != 0) {
		BT_ERR("failure registering table: %d - %u - %p", rsp->status,
		       rsp->attr_count, rsp->attr_base);
	}
#ifdef BT_GATT_DEBUG
	BT_DBG("register rsp : s=%d - b=%p - c=%u", rsp->status,
	       rsp->attr_base, rsp->attr_count);
	{
		int i;

		for (i = 0; i < rsp->attr_count; i++) {
			if (handles[i].handle != 0) {
                uint16_t le16;
                memcpy(&le16, &BT_UUID_16(rsp->attr_base[i].uuid)->val, sizeof(le16));
				BT_DBG("gatt: i %d, h %d, type %d, u16 0x%x",
				       i, handles[i].handle,
				       rsp->attr_base[i].uuid->type,
				       le16);
			}
		}
	}
#endif
}

ssize_t bt_gatt_attr_read(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			  void *buf, uint16_t buf_len, uint16_t offset,
			  const void *value, uint16_t value_len)
{
	uint16_t len;

	if (offset > value_len) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
	}

	len = min(buf_len, value_len - offset);

	BT_DBG("handle 0x%04x offset %u length %u", attr->handle, offset,
	       len);

	memcpy(buf, value + offset, len);

	return len;
}

ssize_t bt_gatt_attr_read_service(struct bt_conn *conn,
				  const struct bt_gatt_attr *attr,
				  void *buf, uint16_t len, uint16_t offset)
{
	struct bt_uuid *uuid = attr->user_data;

	if (uuid->type == BT_UUID_TYPE_16) {
		uint16_t uuid16;

        memcpy(&uuid16, &BT_UUID_16(uuid)->val, sizeof(uuid16));
        uuid16 = sys_cpu_to_le16(uuid16);

		return bt_gatt_attr_read(conn, attr, buf, len, offset,
					 &uuid16, 2);
	}

	return bt_gatt_attr_read(conn, attr, buf, len, offset,
				 BT_UUID_128(uuid)->val, 16);
}

struct gatt_incl {
	uint16_t start_handle;
	uint16_t end_handle;
	uint16_t uuid16;
} __packed;

ssize_t bt_gatt_attr_read_included(struct bt_conn *conn,
				   const struct bt_gatt_attr *attr,
				   void *buf, uint16_t len, uint16_t offset)
{
	struct bt_gatt_include *incl = attr->user_data;
	struct gatt_incl pdu;
	uint8_t value_len;

	pdu.start_handle = sys_cpu_to_le16(incl->start_handle);
	pdu.end_handle = sys_cpu_to_le16(incl->end_handle);
	value_len = sizeof(pdu.start_handle) + sizeof(pdu.end_handle);

	/*
	 * Core 4.2, Vol 3, Part G, 3.2,
	 * The Service UUID shall only be present when the UUID is a 16-bit
	 * Bluetooth UUID.
	 */
	if (incl->uuid->type == BT_UUID_TYPE_16) {
        memcpy(&pdu.uuid16, &BT_UUID_16(incl->uuid)->val, sizeof(pdu.uuid16));
		pdu.uuid16 = sys_cpu_to_le16(pdu.uuid16);
		value_len += sizeof(pdu.uuid16);
	}

	return bt_gatt_attr_read(conn, attr, buf, len, offset, &pdu, value_len);
}

struct gatt_chrc {
	uint8_t properties;
	uint16_t value_handle;
	union {
		uint16_t uuid16;
		uint8_t  uuid[16];
	};
} __packed;

ssize_t bt_gatt_attr_read_chrc(struct bt_conn *conn,
			       const struct bt_gatt_attr *attr, void *buf,
			       uint16_t len, uint16_t offset)
{
	struct bt_gatt_chrc *chrc = attr->user_data;
	struct gatt_chrc pdu;
	uint8_t value_len;

	pdu.properties = chrc->properties;
	/* BLUETOOTH SPECIFICATION Version 4.2 [Vol 3, Part G] page 534:
	 * 3.3.2 Characteristic Value Declaration
	 * The Characteristic Value declaration contains the value of the
	 * characteristic. It is the first Attribute after the characteristic
	 * declaration. All characteristic definitions shall have a
	 * Characteristic Value declaration.
	 */
#if 0
	next = bt_gatt_attr_next(attr);
	if (!next) {
		BT_WARN("No value for characteristic at 0x%04x", attr->handle);
		pdu.value_handle = 0x0000;
	} else {
		pdu.value_handle = sys_cpu_to_le16(next->handle);
	}
#else
	pdu.value_handle = 0x0000;
#endif
	value_len = sizeof(pdu.properties) + sizeof(pdu.value_handle);

	if (chrc->uuid->type == BT_UUID_TYPE_16) {
        memcpy(&pdu.uuid16, &BT_UUID_16(chrc->uuid)->val, sizeof(pdu.uuid16));
		pdu.uuid16 = sys_cpu_to_le16(pdu.uuid16);
		value_len += 2;
	} else {
		memcpy(pdu.uuid, BT_UUID_128(chrc->uuid)->val, 16);
		value_len += 16;
	}

	return bt_gatt_attr_read(conn, attr, buf, len, offset, &pdu, value_len);
}

void bt_gatt_foreach_attr(uint16_t start_handle, uint16_t end_handle,
			  bt_gatt_attr_func_t func, void *user_data)
{

	const struct bt_gatt_attr *attr;

	BT_ASSERT(start_handle == 1 && end_handle == 0xFFFF);

	for (attr = db[0].attrs; attr; attr = bt_gatt_attr_next(attr)) {
#if 0
		/* Check if attribute handle is within range */
		if (attr->handle < start_handle || attr->handle > end_handle) {
			continue;
		}
#endif

		if (func(attr, user_data) == BT_GATT_ITER_STOP) {
			break;
		}
	}
}

struct bt_gatt_attr *bt_gatt_attr_next(const struct bt_gatt_attr *attr)
{
	struct ble_gatt_service *svc, *svc_last;

	svc_last = &db[db_cnt];

	for (svc = db; svc < svc_last; svc++) {
		if (attr >= svc->attrs && attr < &svc->attrs[svc->attr_count]) {
			int index = attr - &svc->attrs[0];

			if (index < (svc->attr_count - 1)) {
				return (struct bt_gatt_attr *)&attr[1];
			} else if ((svc + 1) < svc_last) {
				return (svc + 1)->attrs;
			} else {
				return NULL;
			}
		}
	}
	/* Normally, we should not reach here */
	return NULL;
}

ssize_t bt_gatt_attr_read_ccc(struct bt_conn *conn,
			      const struct bt_gatt_attr *attr, void *buf,
			      uint16_t len, uint16_t offset)
{
	return BT_GATT_ERR(BT_ATT_ERR_NOT_SUPPORTED);

#if 0
	struct _bt_gatt_ccc *ccc = attr->user_data;
	uint16_t value;
	size_t i;

	for (i = 0; i < ccc->cfg_len; i++) {
		if (bt_addr_le_cmp(&ccc->cfg[i].peer, &conn->le.dst)) {
			continue;
		}

		value = sys_cpu_to_le16(ccc->cfg[i].value);
		break;
	}

	/* Default to disable if there is no cfg for the peer */
	if (i == ccc->cfg_len) {
		value = 0x0000;
	}

	return bt_gatt_attr_read(conn, attr, buf, len, offset, &value,
				 sizeof(value));
#endif
}

static void gatt_ccc_changed(struct _bt_gatt_ccc *ccc)
{
	int i;
	uint16_t value = 0x0000;

	for (i = 0; i < ccc->cfg_len; i++) {
		if (ccc->cfg[i].value > value) {
			value = ccc->cfg[i].value;
		}
	}

	BT_DBG("ccc %p value 0x%04x", ccc, value);

	if (value != ccc->value) {
		ccc->value = value;
		if (ccc->cfg_changed)
			ccc->cfg_changed(value);
	}
}

#if defined(CONFIG_BLUETOOTH_GATT_CLIENT)
static bool is_bonded(const bt_addr_le_t *addr)
{
#if defined(CONFIG_BLUETOOTH_SMP)
	struct bt_conn *conn = bt_conn_lookup_addr_le(addr);

	/*
	 *  this is a temporary workaround. if encrypt is set, we know we are
	 *  paired. nble does not yet report if device is bonded.
	 */
	if (conn) {
		uint8_t encrypt = conn->encrypt;

		bt_conn_unref(conn);

		return encrypt;
	}
	return false;
#else
	return false;
#endif /* defined(CONFIG_BLUETOOTH_SMP) */
}
#endif

ssize_t bt_gatt_attr_write_ccc(struct bt_conn *conn,
			       const struct bt_gatt_attr *attr, const void *buf,
			       uint16_t len, uint16_t offset)
{
	struct _bt_gatt_ccc *ccc = attr->user_data;
	const uint16_t *data = buf;
	size_t i;

	//BT_DBG("%s", __FUNCTION__);

	if (offset > sizeof(*data)) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
	}

	//BT_DBG("%s1", __FUNCTION__);

	if (offset + len > sizeof(*data)) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
	}

	//BT_DBG("%s2", __FUNCTION__);

	for (i = 0; i < ccc->cfg_len; i++) {
		/* Check for existing configuration */
		if (!bt_addr_le_cmp(&ccc->cfg[i].peer, &conn->le.dst)) {
			break;
		}
	}

	//BT_DBG("%s3", __FUNCTION__);

	if (i == ccc->cfg_len) {
		for (i = 0; i < ccc->cfg_len; i++) {
			/* Check for unused configuration */
			if (!ccc->cfg[i].valid) {
				bt_addr_le_copy(&ccc->cfg[i].peer, &conn->le.dst);
#if NOT_USED_FOR_THE_TIME_BEING
				/* Only set valid if bonded */
				ccc->cfg[i].valid = is_bonded(&conn->le.dst);
#endif
				break;
			}
		}

		if ((i == ccc->cfg_len) && (ccc->cfg_len)) {
			BT_WARN("No space to store CCC cfg");
			return -ENOMEM;
		}
	}

	//BT_DBG("%s len-%p", __FUNCTION__, ccc);
    
	ccc->cfg[i].value = sys_le16_to_cpu(*data);
	//BT_DBG("%s 5len-%d", __FUNCTION__, len);

	//BT_DBG("handle 0x%04x value %u", attr->handle, *data);

	/* Update cfg if don't match */
	if (ccc->value != *data) {
		gatt_ccc_changed(ccc);
	}

	return len;
}

ssize_t bt_gatt_attr_read_cep(struct bt_conn *conn,
			      const struct bt_gatt_attr *attr, void *buf,
			      uint16_t len, uint16_t offset)
{
	struct bt_gatt_cep *value = attr->user_data;
	uint16_t props = sys_cpu_to_le16(value->properties);

	return bt_gatt_attr_read(conn, attr, buf, len, offset, &props,
				 sizeof(props));
}

ssize_t bt_gatt_attr_read_cud(struct bt_conn *conn,
			      const struct bt_gatt_attr *attr, void *buf,
			      uint16_t len, uint16_t offset)
{
	char *value = attr->user_data;

	return bt_gatt_attr_read(conn, attr, buf, len, offset, value, strlen(value));
}

ssize_t bt_gatt_attr_read_cpf(struct bt_conn *conn,
			      const struct bt_gatt_attr *attr, void *buf,
			      uint16_t len, uint16_t offset)
{
	struct bt_gatt_cpf *value = attr->user_data;

	return bt_gatt_attr_read(conn, attr, buf, len, offset, value,
				 sizeof(*value));
}

struct notify_data {
	uint16_t state;
	uint16_t type;
	const struct bt_gatt_attr *attr;
	const void *data;
	uint16_t len;
	bt_gatt_notify_sent_func_t notify_cb;
	struct bt_gatt_indicate_params *params;
};

static int att_notify(struct bt_conn *conn, const struct bt_gatt_attr *attr,
		      const void *data, size_t len,
		      bt_gatt_notify_sent_func_t cb)
{
	struct nble_gatt_send_notif_params notif;

	notif.conn_handle = conn->handle;
	notif.params.attr = attr;
	notif.params.offset = 0;
	notif.cback = cb;

	nble_gatt_send_notif_req(&notif, data, len);

	return 0;
}

void on_nble_gatts_send_notif_rsp(const struct nble_gatt_notif_rsp *rsp)
{
	struct bt_conn *conn;

	conn = bt_conn_lookup_handle(rsp->conn_handle);

	if (conn) {
		if (rsp->cback) {
			rsp->cback(conn, rsp->attr, rsp->status);
		}
		bt_conn_unref(conn);
	}
}

void on_nble_gatts_send_ind_rsp(const struct nble_gatt_ind_rsp *rsp)
{
	struct bt_conn *conn;

	conn = bt_conn_lookup_handle(rsp->conn_handle);

	if (conn) {
		if (rsp->cback) {
			rsp->cback(conn, rsp->attr, rsp->status);
		}
		bt_conn_unref(conn);
	}
}

static int att_indicate(struct bt_conn *conn,
			struct bt_gatt_indicate_params *params)
{
	struct nble_gatt_send_ind_params ind;

	ind.conn_handle = conn->handle;
	ind.cback = params->func;
	ind.params.attr = params->attr;
	ind.params.offset = 0;

	nble_gatt_send_ind_req(&ind, params->data, params->len);

	return 0;
}

static uint8_t notify_cb(const struct bt_gatt_attr *attr, void *user_data)
{
	struct notify_data *data = user_data;
	struct _bt_gatt_ccc *ccc;
	size_t i;

	/* Check if the attribute was reached */
	if (data->state == 0) {
		if (attr == data->attr)
			data->state = 1;
		return BT_GATT_ITER_CONTINUE;
	}

	if (bt_uuid_cmp(attr->uuid, BT_UUID_GATT_CCC)) {
		/* Stop if we reach the next characteristic */
		if (!bt_uuid_cmp(attr->uuid, BT_UUID_GATT_CHRC)) {
			return BT_GATT_ITER_STOP;
		}
		return BT_GATT_ITER_CONTINUE;
	}

	/* Check attribute user_data must be of type struct _bt_gatt_ccc */
	if (attr->write != bt_gatt_attr_write_ccc) {
		return BT_GATT_ITER_CONTINUE;
	}

	ccc = attr->user_data;

	/* Notify all peers configured */
	for (i = 0; i < ccc->cfg_len; i++) {
		struct bt_conn *conn;
		int err;

		if (ccc->value != data->type) {
			continue;
		}

		conn = bt_conn_lookup_addr_le(&ccc->cfg[i].peer);
		if (!conn || conn->state != BT_CONN_CONNECTED) {//Bug here
			continue;
		}

		if (data->type == BT_GATT_CCC_INDICATE) {
			err = att_indicate(conn, data->params);

		} else {
			err = att_notify(conn, data->attr, data->data,
					 data->len, data->notify_cb);
		}

		bt_conn_unref(conn);

		if (err < 0) {
			return BT_GATT_ITER_STOP;
		}
	}

	return BT_GATT_ITER_CONTINUE;
}

int bt_gatt_notify(struct bt_conn *conn, const struct bt_gatt_attr *attr,
		   const void *data, uint16_t len,
		   bt_gatt_notify_sent_func_t cb)
{
	struct notify_data nfy;

	if (!attr) {
		return -EINVAL;
	}

	if (conn) {
		return att_notify(conn, attr, data, len, cb);
	}

	nfy.state = 0;
	nfy.attr = attr;
	nfy.type = BT_GATT_CCC_NOTIFY;
	nfy.data = data;
	nfy.len = len;
	nfy.notify_cb = cb;

	bt_gatt_foreach_attr(1, 0xffff, notify_cb, &nfy);

	return 0;
}

int bt_gatt_indicate(struct bt_conn *conn,
		     struct bt_gatt_indicate_params *params)
{
	struct notify_data nfy;

	if (!params || !params->attr) {
		return -EINVAL;
	}

	if (conn) {
		return att_indicate(conn, params);
	}

	nfy.state = 0;
	nfy.type = BT_GATT_CCC_INDICATE;
	nfy.params = params;

	bt_gatt_foreach_attr(1, 0xffff, notify_cb, &nfy);

	return 0;
}

static uint8_t connected_cb(const struct bt_gatt_attr *attr, void *user_data)
{
	struct bt_conn *conn = user_data;
	struct _bt_gatt_ccc *ccc;
	size_t i;

	/* Check attribute user_data must be of type struct _bt_gatt_ccc */
	if (attr->write != bt_gatt_attr_write_ccc) {
		return BT_GATT_ITER_CONTINUE;
	}

	ccc = attr->user_data;

	/* If already enabled skip */
	if (ccc->value) {
		return BT_GATT_ITER_CONTINUE;
	}

	for (i = 0; i < ccc->cfg_len; i++) {
		/* Ignore configuration for different peer */
		if (bt_addr_le_cmp(&conn->le.dst, &ccc->cfg[i].peer)) {
			continue;
		}

		if (ccc->cfg[i].value) {
			gatt_ccc_changed(ccc);
			return BT_GATT_ITER_CONTINUE;
		}
	}

	return BT_GATT_ITER_CONTINUE;
}

static uint8_t disconnected_cb(const struct bt_gatt_attr *attr, void *user_data)
{
	struct bt_conn *conn = user_data;
	struct _bt_gatt_ccc *ccc;
	size_t i;

	/* Check attribute user_data must be of type struct _bt_gatt_ccc */
	if (attr->write != bt_gatt_attr_write_ccc) {
		return BT_GATT_ITER_CONTINUE;
	}

	ccc = attr->user_data;

	/* If already disabled skip */
	if (!ccc->value) {
		return BT_GATT_ITER_CONTINUE;
	}

	for (i = 0; i < ccc->cfg_len; i++) {
		/* Ignore configurations with disabled value */
		if (!ccc->cfg[i].value) {
			continue;
		}

		if (bt_addr_le_cmp(&conn->le.dst, &ccc->cfg[i].peer)) {
			struct bt_conn *tmp;

			/* Skip if there is another peer connected */
			tmp = bt_conn_lookup_addr_le(&ccc->cfg[i].peer);
			if (tmp) {
				if (tmp->state == BT_CONN_CONNECTED) {
					bt_conn_unref(tmp);
					return BT_GATT_ITER_CONTINUE;
				}

				bt_conn_unref(tmp);
			}
		}
	}

	/* Reset value while disconnected */
	memset(&ccc->value, 0, sizeof(ccc->value));

	if (ccc->cfg_changed) {
		ccc->cfg_changed(ccc->value);
	}

	BT_DBG("ccc %p reseted", ccc);

	return BT_GATT_ITER_CONTINUE;
}

void on_nble_gatts_write_evt(const struct nble_gatt_wr_evt *evt,
			    const uint8_t *buf, uint8_t buflen)
{
	const struct bt_gatt_attr *attr = evt->attr;
	struct bt_conn *conn = bt_conn_lookup_handle(evt->conn_handle);
	struct nble_gatts_wr_reply_params reply_data;

	//BT_DBG("write_evt %p", attr);

	/* Check for write support and flush support in case of prepare */
	if (!attr->write ||
	    ((evt->flag & NBLE_GATT_WR_FLAG_PREP) && !attr->flush)) {
		reply_data.status = BT_GATT_ERR(BT_ATT_ERR_WRITE_NOT_PERMITTED);

		goto reply;
	}
	//BT_DBG("%s", __FUNCTION__);

	reply_data.status = attr->write(conn, attr, buf, buflen, evt->offset);
	if (reply_data.status < 0) {
        
        //BT_DBG("%s1-1", __FUNCTION__);

		goto reply;
	}

	//BT_DBG("%s1", __FUNCTION__);

	/* Return an error if not all data has been written */
	if (reply_data.status != buflen) {
		reply_data.status = BT_GATT_ERR(BT_ATT_ERR_UNLIKELY);

		goto reply;
	}

	//BT_DBG("%s2", __FUNCTION__);

	if (attr->flush && !(evt->flag & NBLE_GATT_WR_FLAG_PREP))
		reply_data.status = attr->flush(conn, attr, BT_GATT_FLUSH_SYNC);

	//BT_DBG("%s3", __FUNCTION__);

reply:
	//BT_DBG("%s4", __FUNCTION__);
	if (evt->flag & NBLE_GATT_WR_FLAG_REPLY) {
		reply_data.conn_handle = evt->conn_handle;

		nble_gatts_wr_reply_req(&reply_data);
	}

	if (conn)
		bt_conn_unref(conn);
}

static uint8_t flush_all(const struct bt_gatt_attr *attr, void *user_data)
{
	struct ble_gatts_flush_all *flush_data = user_data;

	if (attr->flush) {
		int status = attr->flush(flush_data->conn, attr, flush_data->flag);

		if (status < 0 && flush_data->status == 0)
			flush_data->status = status;
	}

	return BT_GATT_ITER_CONTINUE;
}

void on_nble_gatts_write_exec_evt(const struct nble_gatt_wr_exec_evt *evt)
{
	BT_DBG("write_exec_evt");

	struct ble_gatts_flush_all flush_data = {
		.conn = bt_conn_lookup_handle(evt->conn_handle),
		.flag = evt->flag,
		.status = 0,
	};

	bt_gatt_foreach_attr(0x0001, 0xFFFF, flush_all, &flush_data);

	struct nble_gatts_wr_reply_params reply_data = {
		.conn_handle = evt->conn_handle,
		.status = flush_data.status,
	};
	nble_gatts_wr_reply_req(&reply_data);

	bt_conn_unref(flush_data.conn);
}

void on_nble_gatts_read_evt(const struct nble_gatt_rd_evt *evt)
{
	struct nble_gatts_rd_reply_params reply_data;
	const struct bt_gatt_attr *attr;
	/* The length of the value sent back in the response is unknown because
	 * of NRF API limitation, so we use the max possible one: ATT_MTU-1 */
	uint8_t data[BLE_GATT_MTU_SIZE - 1] = { 0 };
	int len;

	attr = evt->attr;

	BT_DBG("read_evt %p", attr);

	memset(&reply_data, 0, sizeof(reply_data));

	if (attr->read) {
		struct bt_conn *conn = bt_conn_lookup_handle(evt->conn_handle);

		len = attr->read(conn, attr, data, sizeof(data), evt->offset);

		if (conn)
			bt_conn_unref(conn);

	} else {
		len = BT_GATT_ERR(BT_ATT_ERR_READ_NOT_PERMITTED);
	}

	/* status >= 0 is considered as success by nble */
	reply_data.status = len;

	if (len < 0) {
		len = 0;
	}

	reply_data.conn_handle = evt->conn_handle;

	/* offset is needed by nble even in error case */
	reply_data.offset = evt->offset;

	nble_gatts_rd_reply_req(&reply_data, data, len);
}

#if defined(CONFIG_BLUETOOTH_GATT_CLIENT)
void on_nble_gattc_value_evt(const struct ble_gattc_value_evt *evt,
		uint8_t *data, uint8_t length)
{
	struct bt_gatt_subscribe_params *params;
	struct bt_conn *conn;

	conn = bt_conn_lookup_handle(evt->conn_handle);

	//BT_DBG("FUNC %s", __FUNCTION__);

	if (conn) {
		for (params = subscriptions; params; params = params->_next) {
			if (evt->handle != params->value_handle) {
				continue;
			}

			if (params->notify(conn, params, data, length) ==
			    BT_GATT_ITER_STOP) {
				bt_gatt_unsubscribe(conn, params);
			}
		}
		bt_conn_unref(conn);
	}
}

static void gatt_subscription_remove(struct bt_conn *conn,
				     struct bt_gatt_subscribe_params *prev,
				     struct bt_gatt_subscribe_params *params)
{
	/* Remove subscription from the list*/
	if (!prev) {
		subscriptions = params->_next;
	} else {
		prev->_next = params->_next;
	}

	params->notify(conn, params, NULL, 0);
}

static void remove_subscribtions(struct bt_conn *conn)
{
	struct bt_gatt_subscribe_params *params, *prev;

	/* Lookup existing subscriptions */
	for (params = subscriptions, prev = NULL; params;
	     prev = params, params = params->_next) {
		if (bt_addr_le_cmp(&params->_peer, &conn->le.dst)) {
			continue;
		}

		/* Remove subscription */
		gatt_subscription_remove(conn, prev, params);
	}
}

int bt_gatt_exchange_mtu(struct bt_conn *conn, bt_gatt_rsp_func_t func)
{
	return -EINVAL;
}

void on_nble_gattc_discover_rsp(const struct nble_gattc_discover_rsp *rsp,
		const uint8_t *data, uint8_t data_len)
{
	size_t i;
	uint8_t attr_count;
	uint16_t last_handle;
	int status = BT_GATT_ITER_STOP;
	struct bt_gatt_discover_params *params;
	struct bt_gatt_service svc_value;
	struct bt_gatt_include inc_value;
	struct bt_conn *conn = bt_conn_lookup_handle(rsp->conn_handle);

	BT_ASSERT(conn);

	params = rsp->user_data;

	BT_DBG("disc_rsp: s=%d", rsp->status);

	if (rsp->status)
		goto complete;

	if (rsp->type == BT_GATT_DISCOVER_PRIMARY) {
		attr_count = (data_len / sizeof(struct nble_gattc_primary));
	} else if (rsp->type == BT_GATT_DISCOVER_INCLUDE) {
		attr_count = (data_len / sizeof(struct nble_gattc_included));
	} else if (rsp->type == BT_GATT_DISCOVER_CHARACTERISTIC) {
		attr_count = (data_len / sizeof(struct nble_gattc_characteristic));
	} else if (rsp->type == BT_GATT_DISCOVER_DESCRIPTOR) {
		attr_count = (data_len / sizeof(struct nble_gattc_descriptor));
	} else
		goto complete;
	BT_DBG("disc_rsp: c=%d", attr_count);
	last_handle = params->end_handle;
	for (i = 0; i < attr_count; i++) {
		struct bt_gatt_attr *attr = NULL;

		if (rsp->type == BT_GATT_DISCOVER_PRIMARY) {
    //BT_DBG("%s-%d", __FUNCTION__, __LINE__);
			const struct nble_gattc_primary *gattr =
					(void *)&data[i * sizeof(*gattr)];
			if ((gattr->range.start_handle < params->start_handle) &&
			    (gattr->range.end_handle > params->end_handle)) {
				/*
				 * Only the attributes with attribute handles
				 * between and including the Starting
				 * Handle and the Ending Handle is returned
				 */
				goto complete;
			}
			svc_value.end_handle = gattr->range.end_handle;
			svc_value.uuid = (struct bt_uuid*)(&(gattr->uuid));//params->uuid;
			attr = (&(struct bt_gatt_attr)BT_GATT_PRIMARY_SERVICE(&svc_value));
			attr->handle = gattr->handle;
			last_handle = svc_value.end_handle;
		} else if (rsp->type == BT_GATT_DISCOVER_INCLUDE) {
			const struct nble_gattc_included *gattr =
					(void *)&data[i * sizeof(*gattr)];

			inc_value.start_handle = gattr->range.start_handle;
			inc_value.end_handle = gattr->range.end_handle;
			/*
			 * 4.5.1 If the service UUID is a 16-bit Bluetooth UUID
			 *  it is also returned in the response.
			 */
			switch (gattr->uuid.uuid.type) {
			case BT_UUID_TYPE_16:
				inc_value.uuid = &gattr->uuid.uuid;
				break;
			case BT_UUID_TYPE_128:
				/* Data is not available at this point */
				break;
			}
			attr = (&(struct bt_gatt_attr)
					BT_GATT_INCLUDE_SERVICE(&inc_value));
			attr->handle = gattr->handle;
			last_handle = gattr->handle;
		} else if (rsp->type == BT_GATT_DISCOVER_CHARACTERISTIC) {
			const struct nble_gattc_characteristic *gattr =
					(void *)&data[i * sizeof(*gattr)];
			attr = (&(struct bt_gatt_attr)
				BT_GATT_CHARACTERISTIC(&gattr->uuid.uuid,
				gattr->prop));
			attr->handle = gattr->handle;
			last_handle = gattr->handle;
			/* Skip if UUID is set but doesn't match */
			if (params->uuid && bt_uuid_cmp(&gattr->uuid.uuid, params->uuid)) {
				continue;
			}
		} else if (rsp->type == BT_GATT_DISCOVER_DESCRIPTOR) {
			const struct nble_gattc_descriptor *gattr =
					(void *)&data[i * sizeof(*gattr)];

			attr = (&(struct bt_gatt_attr)
				BT_GATT_DESCRIPTOR(&gattr->uuid.uuid, 0, NULL, NULL, NULL));
			attr->handle = gattr->handle;
			last_handle = gattr->handle;
		} else {
			/* Error case */
			goto complete;
		}
		status = params->func(conn, attr, params);
		if (status == BT_GATT_ITER_STOP) {
			/* Not required to call complete */
			goto done;
		}
	}
	if (last_handle < UINT16_MAX) {
		last_handle++;
	}
	BT_DBG("disc_rsp: l=%d", last_handle);
	params->start_handle = last_handle;
	if (params->start_handle < params->end_handle) {
		if (!bt_gatt_discover(conn, params))
			goto not_done;
	}

complete:
	/* Indicate that there are no more attributes found */
	params->func(conn, NULL, params);

done:
	BT_DBG("disc_rsp: done");

not_done:
	bt_conn_unref(conn);
}

int bt_gatt_discover(struct bt_conn *conn,
		     struct bt_gatt_discover_params *params)
{
	struct nble_discover_params discover_params;

	if (!conn || !params || !params->func || !params->start_handle ||
	    !params->end_handle || params->start_handle > params->end_handle) {
		return -EINVAL;
	}

	if (conn->state != BT_CONN_CONNECTED) {
		return -EINVAL;
	}

	BT_DBG("disc: %d", params->start_handle);

	memset(&discover_params, 0, sizeof(discover_params));

	switch (params->type) {
	case BT_GATT_DISCOVER_PRIMARY:
	case BT_GATT_DISCOVER_CHARACTERISTIC:
		if (params->uuid) {
			/* Always copy a full 128 bit UUID */
			discover_params.uuid = *BT_UUID_128(params->uuid);
			discover_params.flags = DISCOVER_FLAGS_UUID_PRESENT;
		}
		break;

	case BT_GATT_DISCOVER_INCLUDE:
	case BT_GATT_DISCOVER_DESCRIPTOR:
		break;
	default:
		return -EINVAL;
	}

	discover_params.conn_handle = conn->handle;
	discover_params.type = params->type;
	discover_params.handle_range.start_handle = params->start_handle;
	discover_params.handle_range.end_handle = params->end_handle;

	discover_params.user_data = params;

	nble_gattc_discover_req(&discover_params);

	return 0;
}

void on_nble_gattc_read_multiple_rsp(const struct ble_gattc_read_rsp *rsp,
		uint8_t *pdu, uint8_t length)
{
	struct bt_gatt_read_params *params;
	struct bt_conn *conn = bt_conn_lookup_handle(rsp->conn_handle);

	BT_ASSERT(conn);

	params = rsp->user_data;

	BT_DBG("err 0x%02x", rsp->status);

	if (rsp->status) {
		params->func(conn, rsp->status, params, NULL, 0);
		bt_conn_unref(conn);
		return;
	}

	params->func(conn, 0, params, pdu, length);

	/* mark read as complete since read multiple is single response */
	params->func(conn, 0, params, NULL, 0);

	bt_conn_unref(conn);
}

void on_nble_gattc_read_rsp(const struct ble_gattc_read_rsp *rsp,
		uint8_t *pdu, uint8_t length)
{
	struct bt_gatt_read_params *params;
	struct bt_conn *conn = bt_conn_lookup_handle(rsp->conn_handle);

	BT_ASSERT(conn);

	params = rsp->user_data;

	if (rsp->status) {
		params->func(conn, rsp->status, params, NULL, 0);
		bt_conn_unref(conn);
		return;
	}

	if (params->func(conn, 0, params, pdu, length) == BT_GATT_ITER_STOP) {
		bt_conn_unref(conn);
		return;
	}

	/*
	 * Core Spec 4.2, Vol. 3, Part G, 4.8.1
	 * If the Characteristic Value is greater than (ATT_MTU – 1) octets
	 * in length, the Read Long Characteristic Value procedure may be used
	 * if the rest of the Characteristic Value is required.
	 */
	if (length < BLE_GATT_MTU_SIZE - 1) {
		params->func(conn, 0, params, NULL, 0);
		bt_conn_unref(conn);
		return;
	}

	params->single.offset += length;

	/* Continue reading the attribute */
	if (bt_gatt_read(conn, params)) {
		params->func(conn, BT_ATT_ERR_UNLIKELY, params, NULL, 0);
	}
	bt_conn_unref(conn);
}

int bt_gatt_read(struct bt_conn *conn, struct bt_gatt_read_params *params)
{

	struct ble_gattc_read_params single_req;
	struct ble_gattc_read_multiple_params mutiple_req;

	if (!conn || conn->state != BT_CONN_CONNECTED || !params ||
	    params->handle_count == 0 || !params->func) {
		return -EINVAL;
	}

	single_req.conn_handle = conn->handle;

	if (1 == params->handle_count) {
		single_req.handle = params->single.handle;
		single_req.offset = params->single.offset;
		single_req.user_data = params;

		nble_gattc_read_req(&single_req);
	} else {
		mutiple_req.conn_handle = conn->handle;
		mutiple_req.user_data = params;

		nble_gattc_read_multiple_req(&mutiple_req, params->handles, 2 * params->handle_count);
	}
	return 0;
}

static void on_write_no_rsp_complete(struct bt_conn *conn, uint8_t err,
				     const void *data)
{
}

static void on_write_complete(struct bt_conn *conn, uint8_t err,
			      const struct bt_gatt_write_params *wr_params)
{
	bt_gatt_write_rsp_func_t func = wr_params->user_data[0];
	const void *data = wr_params->user_data[1];

	BT_ASSERT(func);
	func(conn, err, data);
}

static int _bt_gatt_write(struct bt_conn *conn, uint16_t handle, bool with_resp,
			  uint16_t offset, const void *data, uint16_t length,
			  struct bt_gatt_write_params *wr_params)
{
	struct ble_gattc_write_params req;

	req.conn_handle = conn->handle;
	req.handle = handle;
	req.offset = offset;
	req.with_resp = with_resp;
	req.wr_params = *wr_params;

	nble_gattc_write_req(&req, data, length);

	return 0;
}

void on_nble_gattc_write_rsp(const struct ble_gattc_write_rsp *rsp)
{

	struct bt_conn *conn = bt_conn_lookup_handle(rsp->conn_handle);

	BT_ASSERT(conn);

	if (rsp->wr_params.func) {
		rsp->wr_params.func(conn, rsp->status, &rsp->wr_params);
	}

	bt_conn_unref(conn);
}


int bt_gatt_write_without_response(struct bt_conn *conn, uint16_t handle,
				   const void *data, uint16_t length, bool sign)
{
	return bt_gatt_write(conn, handle, 0, data, length,
			     on_write_no_rsp_complete);
}

int bt_gatt_write(struct bt_conn *conn, uint16_t handle, uint16_t offset,
		  const void *data, uint16_t length,
		  bt_gatt_write_rsp_func_t func)
{
	struct bt_gatt_write_params wr_params;

	if (!conn || conn->state != BT_CONN_CONNECTED  || !handle || !func) {
		return -EINVAL;
	}

	wr_params.func = on_write_complete;
	wr_params.user_data[0] = func;
	wr_params.user_data[1] = (void *)data;

	return _bt_gatt_write(conn, handle,
			      (func == on_write_no_rsp_complete)
			      ? false : true,
			      offset, data, length, &wr_params);
}

static void gatt_subscription_add(struct bt_conn *conn,
				  struct bt_gatt_subscribe_params *params)
{
	bt_addr_le_copy(&params->_peer, &conn->le.dst);

	/* Prepend subscription */
	params->_next = subscriptions;
	subscriptions = params;
}

static void att_write_ccc_rsp(struct bt_conn *conn, uint8_t err,
			      const struct bt_gatt_write_params *wr_params)
{
	struct bt_gatt_subscribe_params *params = wr_params->user_data[0];

	/* if write to CCC failed we remove subscription and notify app */
	if (err) {
		struct bt_gatt_subscribe_params *cur, *prev;

		for (cur = subscriptions, prev = NULL; cur;
		     prev = cur, cur = cur->_next) {

			if (cur == params) {
				gatt_subscription_remove(conn, prev, params);
				break;
			}
		}
	}
}

static int gatt_write_ccc(struct bt_conn *conn, uint16_t handle, uint16_t value,
			  bt_att_func_t func,
			  struct bt_gatt_subscribe_params *params)
{
	struct bt_gatt_write_params wr_params;

	wr_params.func = func;
	wr_params.user_data[0] = params;

	BT_DBG("handle 0x%04x value 0x%04x", handle, value);

	return _bt_gatt_write(conn, handle, true, 0, &value, sizeof(value),
			     &wr_params);
}

int bt_gatt_subscribe(struct bt_conn *conn,
		      struct bt_gatt_subscribe_params *params)
{
	struct bt_gatt_subscribe_params *tmp;
	bool has_subscription = false;

	if (!conn || conn->state != BT_CONN_CONNECTED) {
		return -ENOTCONN;
	}

	if (!params || !params->notify ||
	    !params->value || !params->ccc_handle) {
		return -EINVAL;
	}

	/* Lookup existing subscriptions */
	for (tmp = subscriptions; tmp; tmp = tmp->_next) {
		/* Fail if entry already exists */
		if (tmp == params) {
			return -EALREADY;
		}

		/* Check if another subscription exists */
		if (!bt_addr_le_cmp(&tmp->_peer, &conn->le.dst) &&
		    tmp->value_handle == params->value_handle &&
		    tmp->value >= params->value) {
			has_subscription = true;
		}
	}

	/* Skip write if already subscribed */
	if (!has_subscription) {
		int err;

		err = gatt_write_ccc(conn, params->ccc_handle, params->value,
				     att_write_ccc_rsp, params);
		if (err) {
			return err;
		}
	}

	/*
	 * Add subscription before write complete as some implementation were
	 * reported to send notification before reply to CCC write.
	 */
	gatt_subscription_add(conn, params);

	return 0;
}

int bt_gatt_unsubscribe(struct bt_conn *conn,
			struct bt_gatt_subscribe_params *params)
{
	struct bt_gatt_subscribe_params *tmp;
	bool has_subscription = false, found = false;

	if (!conn || conn->state != BT_CONN_CONNECTED) {
		return -ENOTCONN;
	}

	if (!params) {
		return -EINVAL;
	}

	/* Check head */
	if (subscriptions == params) {
		subscriptions = params->_next;
		found = true;
	}

	/* Lookup existing subscriptions */
	for (tmp = subscriptions; tmp; tmp = tmp->_next) {
		/* Remove subscription */
		if (tmp->_next == params) {
			tmp->_next = params->_next;
			found = true;
		}

		/* Check if there still remains any other subscription */
		if (!bt_addr_le_cmp(&tmp->_peer, &conn->le.dst) &&
		    tmp->value_handle == params->value_handle) {
			has_subscription = true;
		}
	}

	if (!found) {
		return -EINVAL;
	}

	if (has_subscription) {
		return 0;
	}

	params->value = 0;

	return gatt_write_ccc(conn, params->ccc_handle, params->value, NULL,
			      NULL);
}

static void add_subscriptions(struct bt_conn *conn)
{
	struct bt_gatt_subscribe_params *params, *prev;

	/* Lookup existing subscriptions */
	for (params = subscriptions, prev = NULL; params;
	     prev = params, params = params->_next) {
		if (bt_addr_le_cmp(&params->_peer, &conn->le.dst)) {
			continue;
		}

		/* Force write to CCC to workaround devices that don't track
		 * it properly.
		 */
		gatt_write_ccc(conn, params->ccc_handle, params->value,
			       att_write_ccc_rsp, params);
	}
}
#else

void on_nble_gattc_discover_rsp(const struct nble_gattc_discover_rsp *rsp,
		const uint8_t *data, uint8_t data_len)
{

}
void on_nble_gattc_write_rsp(const struct ble_gattc_write_rsp *rsp)
{
}

void on_nble_gattc_value_evt(const struct ble_gattc_value_evt *evt,
		uint8_t *buf, uint8_t buflen)
{
}

void on_nble_gattc_read_rsp(const struct ble_gattc_read_rsp *rsp,
		uint8_t *data, uint8_t data_len)
{
}

void on_nble_gattc_read_multiple_rsp(const struct ble_gattc_read_rsp *rsp,
		uint8_t *data, uint8_t data_len)
{
}

#endif

void bt_gatt_connected(struct bt_conn *conn)
{
	BT_DBG("conn %p", conn);
	bt_gatt_foreach_attr(0x0001, 0xffff, connected_cb, conn);
#if defined(CONFIG_BLUETOOTH_GATT_CLIENT)
	add_subscriptions(conn);
#endif /* CONFIG_BLUETOOTH_GATT_CLIENT */
}

void bt_gatt_disconnected(struct bt_conn *conn)
{
	BT_DBG("conn %p", conn);
	bt_gatt_foreach_attr(0x0001, 0xffff, disconnected_cb, conn);

#if defined(CONFIG_BLUETOOTH_GATT_CLIENT)
	/* If bonded don't remove subscriptions */
	if (is_bonded(&conn->le.dst)) {
		return;
	}

	remove_subscribtions(conn);
#endif /* CONFIG_BLUETOOTH_GATT_CLIENT */
}
