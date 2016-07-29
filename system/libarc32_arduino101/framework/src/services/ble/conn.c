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
#include <atomic.h>

#include <bluetooth/hci.h>
#include <bluetooth/bluetooth.h>
#include <bluetooth/gatt.h>
#include <misc/byteorder.h>

#include "hci_core.h"
#include "conn_internal.h"
#include "gap_internal.h"
#include "l2cap_internal.h"
#include "smp.h"

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

#if defined(CONFIG_BLUETOOTH_SMP) || defined(CONFIG_BLUETOOTH_BREDR)
const struct bt_conn_auth_cb *bt_auth;
#endif /* CONFIG_BLUETOOTH_SMP || CONFIG_BLUETOOTH_BREDR */

static struct bt_conn conns[CONFIG_BLUETOOTH_MAX_CONN];
static struct bt_conn_cb *callback_list;

static void notify_connected(struct bt_conn *conn)
{
	struct bt_conn_cb *cb;

	for (cb = callback_list; cb; cb = cb->_next) {
		if (cb->connected) {
			cb->connected(conn, conn->err);
		}
	}
}

static void notify_disconnected(struct bt_conn *conn)
{
	struct bt_conn_cb *cb;

	for (cb = callback_list; cb; cb = cb->_next) {
		if (cb->disconnected) {
			cb->disconnected(conn, conn->err);
		}
	}
}

void notify_le_param_updated(struct bt_conn *conn)
{
	struct bt_conn_cb *cb;

	for (cb = callback_list; cb; cb = cb->_next) {
		if (cb->le_param_updated) {
			cb->le_param_updated(conn, conn->le.interval,
					     conn->le.latency,
					     conn->le.timeout);
		}
	}
}

#if defined(CONFIG_BLUETOOTH_SMP)

void bt_conn_security_changed(struct bt_conn *conn)
{
	struct bt_conn_cb *cb;

	for (cb = callback_list; cb; cb = cb->_next) {
		if (cb->security_changed) {
			cb->security_changed(conn, conn->sec_level);
		}
	}
}

static int start_security(struct bt_conn *conn)
{
	switch (conn->role) {
#if defined(CONFIG_BLUETOOTH_CENTRAL)
	case BT_HCI_ROLE_MASTER:
	{
#ifdef NOT_APPLICABLE_NBLE
		if (!conn->keys) {
			conn->keys = bt_keys_find(BT_KEYS_LTK_P256,
						  &conn->le.dst);
			if (!conn->keys) {
				conn->keys = bt_keys_find(BT_KEYS_LTK,
							  &conn->le.dst);
			}
		}

		if (!conn->keys ||
		    !(conn->keys->keys & (BT_KEYS_LTK | BT_KEYS_LTK_P256))) {
			return bt_smp_send_pairing_req(conn);
		}

		if (conn->required_sec_level > BT_SECURITY_MEDIUM &&
		    !atomic_test_bit(&conn->keys->flags,
				     BT_KEYS_AUTHENTICATED)) {
			return bt_smp_send_pairing_req(conn);
		}

		if (conn->required_sec_level > BT_SECURITY_HIGH &&
		    !atomic_test_bit(&conn->keys->flags,
				     BT_KEYS_AUTHENTICATED) &&
		    !(conn->keys->keys & BT_KEYS_LTK_P256)) {
			return bt_smp_send_pairing_req(conn);
		}

		/* LE SC LTK and legacy master LTK are stored in same place */
		return bt_conn_le_start_encryption(conn, conn->keys->ltk.rand,
						   conn->keys->ltk.ediv,
						   conn->keys->ltk.val,
						   conn->keys->enc_size);
#endif
		return bt_smp_send_pairing_req(conn);
	}
#endif /* CONFIG_BLUETOOTH_CENTRAL */
#if defined(CONFIG_BLUETOOTH_PERIPHERAL)
	case BT_HCI_ROLE_SLAVE:
		return bt_smp_send_security_req(conn);
#endif /* CONFIG_BLUETOOTH_PERIPHERAL */
	default:
		return -EINVAL;
	}
}

int bt_conn_security(struct bt_conn *conn, bt_security_t sec)
{
	int err;

	if (conn->state != BT_CONN_CONNECTED) {
		return -ENOTCONN;
	}

#if defined(CONFIG_BLUETOOTH_SMP_SC_ONLY)
	if (sec < BT_SECURITY_FIPS) {
		return -EOPNOTSUPP;
	}
#endif/* CONFIG_BLUETOOTH_SMP_SC_ONLY */

	/* nothing to do */
	if (conn->sec_level >= sec || conn->required_sec_level >= sec) {
		return 0;
	}

	conn->required_sec_level = sec;

	err = start_security(conn);

	/* reset required security level in case of error */
	if (err) {
		conn->required_sec_level = conn->sec_level;
	}
	return err;
}
#endif /* CONFIG_BLUETOOTH_SMP */

void bt_conn_cb_register(struct bt_conn_cb *cb)
{
	cb->_next = callback_list;
	callback_list = cb;
}

static struct bt_conn *conn_new(void)
{
	struct bt_conn *conn = NULL;
	int i;

	for (i = 0; i < ARRAY_SIZE(conns); i++) {
		if (!atomic_get(&conns[i].ref)) {
			conn = &conns[i];
			break;
		}
	}

	if (!conn) {
		return NULL;
	}

	memset(conn, 0, sizeof(*conn));

	atomic_set(&conn->ref, 1);

	return conn;
}

struct bt_conn *bt_conn_add_le(const bt_addr_le_t *peer)
{
	struct bt_conn *conn = conn_new();

	if (!conn) {
		return NULL;
	}

	bt_addr_le_copy(&conn->le.dst, peer);
#if defined(CONFIG_BLUETOOTH_SMP)
	conn->sec_level = BT_SECURITY_LOW;
	conn->required_sec_level = BT_SECURITY_LOW;
#endif /* CONFIG_BLUETOOTH_SMP */
	conn->type = BT_CONN_TYPE_LE;
	conn->le.interval_min = BT_GAP_INIT_CONN_INT_MIN;
	conn->le.interval_max = BT_GAP_INIT_CONN_INT_MAX;

	return conn;
}

#if defined(CONFIG_BLUETOOTH_BREDR)
struct bt_conn *bt_conn_lookup_addr_br(const bt_addr_t *peer)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(conns); i++) {
		if (!atomic_get(&conns[i].ref)) {
			continue;
		}

		if (conns[i].type != BT_CONN_TYPE_BR) {
			continue;
		}

		if (!bt_addr_cmp(peer, &conns[i].br.dst)) {
			return bt_conn_ref(&conns[i]);
		}
	}

	return NULL;
}

struct bt_conn *bt_conn_add_br(const bt_addr_t *peer)
{
	struct bt_conn *conn = conn_new();

	if (!conn) {
		return NULL;
	}

	bt_addr_copy(&conn->br.dst, peer);
	conn->type = BT_CONN_TYPE_BR;

	return conn;
}
#endif

void bt_conn_set_state(struct bt_conn *conn, bt_conn_state_t state)
{
	bt_conn_state_t old_state;

	BT_DBG("conn state %d -> %d, err: %d", conn->state, state, conn->err);

	if (conn->state == state) {
		BT_DBG("no transition");
		return;
	}

	old_state = conn->state;
	conn->state = state;

	/* Actions needed for exiting the old state */
	switch (old_state) {
	case BT_CONN_DISCONNECTED:
		/* Take a reference for the first state transition after
		 * bt_conn_add_le() and keep it until reaching DISCONNECTED
		 * again.
		 */
		bt_conn_ref(conn);
		break;
	case BT_CONN_CONNECT:
#if 0
		if (conn->timeout) {
			fiber_delayed_start_cancel(conn->timeout);
			conn->timeout = NULL;

			/* Drop the reference taken by timeout fiber */
			bt_conn_unref(conn);
		}
#endif
		break;
	default:
		break;
	}

	/* Actions needed for entering the new state */
	switch (conn->state) {
	case BT_CONN_CONNECTED:
		bt_l2cap_connected(conn);
		notify_connected(conn);
		break;
	case BT_CONN_DISCONNECTED:
		/* Notify disconnection and queue a dummy buffer to wake
		 * up and stop the tx fiber for states where it was
		 * running.
		 */
		if (old_state == BT_CONN_CONNECTED ||
		    old_state == BT_CONN_DISCONNECT) {
			bt_l2cap_disconnected(conn);
			notify_disconnected(conn);
		} else if (old_state == BT_CONN_CONNECT) {
			/* conn->err will be set in this case */
			notify_connected(conn);
		}

		/* Release the reference we took for the very first
		 * state transition.
		 */
		bt_conn_unref(conn);

		break;
	case BT_CONN_CONNECT_SCAN:
		break;
	case BT_CONN_CONNECT:
		break;
	case BT_CONN_DISCONNECT:
		break;
	default:

		break;
	}
}

struct bt_conn *bt_conn_lookup_handle(uint16_t handle)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(conns); i++) {
		if (!atomic_get(&conns[i].ref)) {
			continue;
		}
		/* We only care about connections with a valid handle */
		if (conns[i].state != BT_CONN_CONNECTED &&
		    conns[i].state != BT_CONN_DISCONNECT) {
			continue;
		}

		if (conns[i].handle == handle) {
			return bt_conn_ref(&conns[i]);
		}
	}

	return NULL;
}

struct bt_conn *bt_conn_lookup_addr_le(const bt_addr_le_t *peer)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(conns); i++) {
		if (!atomic_get(&conns[i].ref)) {
			continue;
		}

		if (conns[i].type != BT_CONN_TYPE_LE) {
			continue;
		}

		if (!bt_addr_le_cmp(peer, &conns[i].le.dst)) {
			return bt_conn_ref(&conns[i]);
		}
	}

	return NULL;
}

struct bt_conn *bt_conn_lookup_state_le(const bt_addr_le_t *peer,
					const bt_conn_state_t state)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(conns); i++) {
		if (!atomic_get(&conns[i].ref)) {
			continue;
		}

		if (conns[i].type != BT_CONN_TYPE_LE) {
			continue;
		}

		if (bt_addr_le_cmp(peer, BT_ADDR_LE_ANY) &&
		    bt_addr_le_cmp(peer, &conns[i].le.dst)) {
			continue;
		}

		if (conns[i].state == state) {
			return bt_conn_ref(&conns[i]);
		}
	}

	return NULL;
}

struct bt_conn *bt_conn_ref(struct bt_conn *conn)
{
	atomic_inc(&conn->ref);

	return conn;
}

void bt_conn_unref(struct bt_conn *conn)
{
	atomic_dec(&conn->ref);
}

const bt_addr_le_t *bt_conn_get_dst(const struct bt_conn *conn)
{
	return &conn->le.dst;
}

int bt_conn_get_info(const struct bt_conn *conn, struct bt_conn_info *info)
{
	info->type = conn->type;
	info->role = conn->role;

	switch (conn->type) {
	case BT_CONN_TYPE_LE:
		if (conn->role == BT_HCI_ROLE_MASTER) {
#if 0
			info->le.src = &conn->le.init_addr;
			info->le.dst = &conn->le.resp_addr;
#else
			info->le.dst = &conn->le.dst;
#endif
		} else {
#if 0
			info->le.src = &conn->le.resp_addr;
			info->le.dst = &conn->le.init_addr;
#else
			info->le.src = &conn->le.dst;
#endif
		}
		info->le.interval = conn->le.interval;
		info->le.latency = conn->le.latency;
		info->le.timeout = conn->le.timeout;

		return 0;
#if defined(CONFIG_BLUETOOTH_BREDR)
	case BT_CONN_TYPE_BR:
		info->br.dst = &conn->br.dst;
		return 0;
#endif
	}

	return -EINVAL;
}

static int bt_hci_disconnect(struct bt_conn *conn, uint8_t reason)
{
	struct nble_gap_disconnect_req_params ble_gap_disconnect;

	ble_gap_disconnect.conn_handle = conn->handle;
	ble_gap_disconnect.reason = reason;
	nble_gap_disconnect_req(&ble_gap_disconnect);

	bt_conn_set_state(conn, BT_CONN_DISCONNECT);
	return 0;
}

static int bt_hci_connect_le_cancel(struct bt_conn *conn)
{
	nble_gap_cancel_connect_req(conn);
	return 0;
}

void on_nble_gap_cancel_connect_rsp(const struct nble_response *params)
{
	struct bt_conn *conn = params->user_data;

	conn->err = BT_HCI_ERR_INSUFFICIENT_RESOURCES;
	bt_conn_set_state(conn, BT_CONN_DISCONNECTED);
}

int bt_conn_disconnect(struct bt_conn *conn, uint8_t reason)
{
#if defined(CONFIG_BLUETOOTH_CENTRAL)
	/* Disconnection is initiated by us, so auto connection shall
	 * be disabled. Otherwise the passive scan would be enabled
	 * and we could send LE Create Connection as soon as the remote
	 * starts advertising.
	 */
	if (conn->type == BT_CONN_TYPE_LE) {
		bt_le_set_auto_conn(&conn->le.dst, NULL);
	}
#endif

	switch (conn->state) {
	case BT_CONN_CONNECT_SCAN:
		conn->err = BT_HCI_ERR_INSUFFICIENT_RESOURCES;
		bt_conn_set_state(conn, BT_CONN_DISCONNECTED);
		/* scan update not yet implemented */
		return 0;
	case BT_CONN_CONNECT:
		return bt_hci_connect_le_cancel(conn);
	case BT_CONN_CONNECTED:
		return bt_hci_disconnect(conn, reason);
	case BT_CONN_DISCONNECT:
		return 0;
	case BT_CONN_DISCONNECTED:
	default:
		return -ENOTCONN;
	}
}

static bool valid_adv_params(const struct nble_gap_adv_params *params)
{
	if (params->type == BT_LE_ADV_DIRECT_IND) {
		/* If high duty, ensure interval is 0 */
		if (params->interval_max != 0)
			return false;

		if (params->timeout != 0)
			return false;
	} else if (params->type == BT_LE_ADV_DIRECT_IND_LOW_DUTY) {
		if (params->interval_min < 0x20)
			return false;
	} else {
		return false;
	}

	if (params->interval_min > params->interval_max)
		return false;

	if (params->interval_max > 0x4000)
		return false;

	return true;
}

struct bt_conn *bt_conn_create_slave_le(const bt_addr_le_t *peer,
				  const struct bt_le_adv_param *param)
{
	struct bt_conn *conn;
	/* Timeout is handled by application timer */
	/* forced to none currently (no whitelist support) */
	struct nble_gap_adv_params params = {
		.interval_max = param->interval_max,
		.interval_min = param->interval_min,
		.type = param->type,
		.timeout = 0,
		.filter_policy = 0
	};

	bt_addr_le_copy(&params.peer_bda, peer);

	if (!valid_adv_params(&params)) {
		return NULL;
	}

	if (param->type == BT_LE_ADV_DIRECT_IND_LOW_DUTY) {
		params.type = BT_LE_ADV_DIRECT_IND;
	}

	if (atomic_test_bit(bt_dev.flags, BT_DEV_ADVERTISING)) {
		return NULL;
	}

	conn = bt_conn_add_le(&params.peer_bda);

	if (!conn) {
		return NULL;
	}

	bt_conn_set_state(conn, BT_CONN_CONNECT);

	nble_gap_set_adv_params_req(&params);
	nble_gap_start_adv_req();

	return conn;
}

#if defined(CONFIG_BLUETOOTH_CENTRAL)
static int hci_le_create_conn(struct bt_conn *conn)
{
	struct nble_gap_connect_req_params conn_params;

	conn_params.bda = conn->le.dst;
	conn_params.conn_params.interval_min = conn->le.interval_min;
	conn_params.conn_params.interval_max = conn->le.interval_max;
	conn_params.conn_params.slave_latency = conn->le.latency;
	conn_params.conn_params.link_sup_to = conn->le.timeout;

	conn_params.scan_params.interval = sys_cpu_to_le16(BT_GAP_SCAN_FAST_INTERVAL);
	conn_params.scan_params.window = conn_params.scan_params.interval;
	conn_params.scan_params.selective = 0;
	conn_params.scan_params.active = 1;
	conn_params.scan_params.timeout = 0;

	nble_gap_connect_req(&conn_params, conn);

	return 0;
}

void on_nble_gap_connect_rsp(const struct nble_response *params)
{
	struct bt_conn *conn = params->user_data;

	/* If the connection request was not issued successfully */
	if (params->status) {
		conn->err = BT_HCI_ERR_UNACCEPT_CONN_PARAMS;
		bt_conn_set_state(conn, BT_CONN_DISCONNECTED);
	}
}

struct bt_conn *bt_conn_create_le(const bt_addr_le_t *peer,
				const struct bt_le_conn_param *param)
{
	struct bt_conn *conn;

	if (!bt_le_conn_params_valid(param->interval_min, param->interval_max,
				param->latency, param->timeout)) {
		return NULL;
	}

	/* if (atomic_test_bit(bt_dev.flags, BT_DEV_EXPLICIT_SCAN)) */
	/*	return NULL; */

	conn = bt_conn_lookup_addr_le(peer);
	if (conn) {
		switch (conn->state) {
		case BT_CONN_CONNECT_SCAN:
			bt_conn_set_param_le(conn, param);
			return conn;
		case BT_CONN_CONNECT:
		case BT_CONN_CONNECTED:
			return conn;
		default:
			bt_conn_unref(conn);
			return NULL;
		}
	}

	conn = bt_conn_add_le(peer);
	if (!conn) {
		return NULL;
	}
#if 0
	bt_conn_set_state(conn, BT_CONN_CONNECT_SCAN);

	bt_le_scan_update(true);
#endif

	bt_addr_le_copy(&conn->le.dst, peer);

	bt_conn_set_param_le(conn, param);

	/* for the time being, the implementation bypassed the scan procedure */
	if (hci_le_create_conn(conn)) {
		goto done;
	}

	bt_conn_set_state(conn, BT_CONN_CONNECT);

done:
	return conn;
}
#else

void on_nble_gap_connect_rsp(const struct nble_response *params)
{
}

#endif /* CONFIG_BLUETOOTH_CENTRAL */

int bt_conn_le_param_update(struct bt_conn *conn, const struct bt_le_conn_param *param)
{
	return bt_conn_update_param_le(conn, param);
}

#if defined(CONFIG_BLUETOOTH_SMP) || defined(CONFIG_BLUETOOTH_BREDR)
uint8_t bt_conn_enc_key_size(struct bt_conn *conn)
{
	return 0;
}

int bt_conn_auth_cb_register(const struct bt_conn_auth_cb *cb)
{
	if (!cb) {
		bt_auth = NULL;
		return 0;
	}

	/* cancel callback should always be provided */
	if (!cb->cancel) {
		return -EINVAL;
	}

	if (bt_auth) {
		return -EALREADY;
	}

	bt_auth = cb;
	return 0;
}

#if defined(CONFIG_BLUETOOTH_BREDR)
static int pin_code_neg_reply(const bt_addr_t *bdaddr)
{
	struct bt_hci_cp_pin_code_neg_reply *cp;
	struct net_buf *buf;

	BT_DBG("");

	buf = bt_hci_cmd_create(BT_HCI_OP_PIN_CODE_NEG_REPLY, sizeof(*cp));
	if (!buf) {
		return -ENOBUFS;
	}

	cp = net_buf_add(buf, sizeof(*cp));
	bt_addr_copy(&cp->bdaddr, bdaddr);

	return bt_hci_cmd_send_sync(BT_HCI_OP_PIN_CODE_NEG_REPLY, buf, NULL);
}

static int pin_code_reply(struct bt_conn *conn, const char *pin, uint8_t len)
{
	struct bt_hci_cp_pin_code_reply *cp;
	struct net_buf *buf;

	BT_DBG("");

	buf = bt_hci_cmd_create(BT_HCI_OP_PIN_CODE_REPLY, sizeof(*cp));
	if (!buf) {
		return -ENOBUFS;
	}

	cp = net_buf_add(buf, sizeof(*cp));

	bt_addr_copy(&cp->bdaddr, &conn->br.dst);
	cp->pin_len = len;
	strncpy(cp->pin_code, pin, sizeof(cp->pin_code));

	return bt_hci_cmd_send_sync(BT_HCI_OP_PIN_CODE_REPLY, buf, NULL);
}

int bt_conn_auth_pincode_entry(struct bt_conn *conn, const char *pin)
{
	size_t len;

	if (!bt_auth) {
		return -EINVAL;
	}

	if (conn->type != BT_CONN_TYPE_BR) {
		return -EINVAL;
	}

	len = strlen(pin);
	if (len > 16) {
		return -EINVAL;
	}

	if (conn->required_sec_level == BT_SECURITY_HIGH && len < 16) {
		BT_WARN("PIN code for %s is not 16 bytes wide",
			bt_addr_str(&conn->br.dst));
		return -EPERM;
	}

	return pin_code_reply(conn, pin, len);
}

void bt_conn_pin_code_req(struct bt_conn *conn)
{
	if (bt_auth && bt_auth->pincode_entry) {
		bool secure = false;

		if (conn->required_sec_level == BT_SECURITY_HIGH) {
			secure = true;
		}

		bt_auth->pincode_entry(conn, secure);
	} else {
		pin_code_neg_reply(&conn->br.dst);
	}

}
#endif /* CONFIG_BLUETOOTH_BREDR */

int bt_conn_auth_passkey_entry(struct bt_conn *conn, unsigned int passkey)
{
	if (!bt_auth) {
		return -EINVAL;
	}
#if defined(CONFIG_BLUETOOTH_SMP)
	if (conn->type == BT_CONN_TYPE_LE) {
		return bt_smp_auth_passkey_entry(conn, passkey);
	}
#endif /* CONFIG_BLUETOOTH_SMP */

	return -EINVAL;
}

int bt_conn_auth_passkey_confirm(struct bt_conn *conn, bool match)
{
	if (!bt_auth) {
		return -EINVAL;
	};
#if defined(CONFIG_BLUETOOTH_SMP)
	if (conn->type == BT_CONN_TYPE_LE) {
		return bt_smp_auth_passkey_confirm(conn, match);
	}
#endif /* CONFIG_BLUETOOTH_SMP */

	return -EINVAL;
}

int bt_conn_auth_cancel(struct bt_conn *conn)
{
	if (!bt_auth) {
		return -EINVAL;
	}
#if defined(CONFIG_BLUETOOTH_SMP)
	if (conn->type == BT_CONN_TYPE_LE) {
		return bt_smp_auth_cancel(conn);
	}
#endif /* CONFIG_BLUETOOTH_SMP */
#if defined(CONFIG_BLUETOOTH_BREDR)
	if (conn->type == BT_CONN_TYPE_BR) {
		return pin_code_neg_reply(&conn->br.dst);
	}
#endif /* CONFIG_BLUETOOTH_BREDR */

	return -EINVAL;
}

int bt_conn_remove_info(const bt_addr_le_t *addr)
{
	struct bt_conn *conn;

	/* TODO: implement address specific removal */
	if (bt_addr_le_cmp(addr, BT_ADDR_LE_ANY))
		return -EINVAL;

	do {
		conn = bt_conn_lookup_state_le(addr, BT_CONN_CONNECTED);
		if (conn) {
			bt_conn_unref(conn);
			bt_conn_disconnect(conn,
					   BT_HCI_ERR_REMOTE_USER_TERM_CONN);
		}
	} while(conn);

	return bt_smp_remove_info(addr);
}
#endif /* CONFIG_BLUETOOTH_SMP || CONFIG_BLUETOOTH_BREDR */

int bt_conn_init(void)
{
	int err;
#if NOT_USED_FOR_NOW

	net_buf_pool_init(frag_pool);
	net_buf_pool_init(dummy_pool);

	bt_att_init();
#endif

	err = bt_smp_init();
	if (err) {
		return err;
	}

#if NOT_USED_FOR_NOW
	bt_l2cap_init();

	background_scan_init();
#endif
	return 0;
}

int bt_conn_le_conn_update(struct bt_conn *conn,
			   const struct bt_le_conn_param *param)
{
	struct nble_gap_connect_update_params ble_gap_connect_update;
#if 0
	struct hci_cp_le_conn_update *conn_update;
	struct net_buf *buf;

	buf = bt_hci_cmd_create(BT_HCI_OP_LE_CONN_UPDATE,
				sizeof(*conn_update));
	if (!buf) {
		return -ENOBUFS;
	}

	conn_update = net_buf_add(buf, sizeof(*conn_update));
	memset(conn_update, 0, sizeof(*conn_update));
	conn_update->handle = sys_cpu_to_le16(conn->handle);
	conn_update->conn_interval_min = sys_cpu_to_le16(param->interval_min);
	conn_update->conn_interval_max = sys_cpu_to_le16(param->interval_max);
	conn_update->conn_latency = sys_cpu_to_le16(param->latency);
	conn_update->supervision_timeout = sys_cpu_to_le16(param->timeout);
#endif
	ble_gap_connect_update.conn_handle = conn->handle;
	ble_gap_connect_update.params.interval_min = param->interval_min;
	ble_gap_connect_update.params.interval_max = param->interval_max;
	ble_gap_connect_update.params.slave_latency = param->latency;
	ble_gap_connect_update.params.link_sup_to = param->timeout;

	nble_gap_conn_update_req(&ble_gap_connect_update);

	return 0;
}
