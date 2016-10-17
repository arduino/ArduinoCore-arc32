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
#include <stddef.h>
#include <atomic.h>

#include <bluetooth/gatt.h>
#include <bluetooth/bluetooth.h>
#include "gap_internal.h"
#include "conn_internal.h"

#include "hci_core.h"

#if defined(CONFIG_BLUETOOTH_SMP)
#include "smp.h"
#endif /* CONFIG_BLUETOOTH_SMP */

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

static bt_ready_cb_t bt_ready_cb;
static bt_le_scan_cb_t *scan_dev_found_cb;
static rssi_report_t rssi_report_cb;

struct bt_dev bt_dev;

static int set_advertise_enable(void)
{
#if 0
	struct net_buf *buf;
	int err;
#endif
	if (atomic_test_bit(bt_dev.flags, BT_DEV_ADVERTISING)) {
		return 0;
	}
#if 0
	buf = bt_hci_cmd_create(BT_HCI_OP_LE_SET_ADV_ENABLE, 1);
	if (!buf) {
		return -ENOBUFS;
	}

	net_buf_add_u8(buf, BT_HCI_LE_ADV_ENABLE);
	err = bt_hci_cmd_send_sync(BT_HCI_OP_LE_SET_ADV_ENABLE, buf, NULL);
	if (err) {
		return err;
	}
#endif
	nble_gap_start_adv_req();

	return 0;
}

static int set_advertise_disable(void)
{
#if 0
	struct net_buf *buf;
	int err;
#endif
	if (!atomic_test_bit(bt_dev.flags, BT_DEV_ADVERTISING)) {
		return 0;
	}
#if 0
	buf = bt_hci_cmd_create(BT_HCI_OP_LE_SET_ADV_ENABLE, 1);
	if (!buf) {
		return -ENOBUFS;
	}

	net_buf_add_u8(buf, BT_HCI_LE_ADV_DISABLE);
	err = bt_hci_cmd_send_sync(BT_HCI_OP_LE_SET_ADV_ENABLE, buf, NULL);
	if (err) {
		return err;
	}
#endif
	nble_gap_stop_adv_req(NULL);

	atomic_clear_bit(bt_dev.flags, BT_DEV_ADVERTISING);
	return 0;
}

void ble_gap_get_bonding_info(ble_bond_info_cb_t func, void *user_data,
			      bool include_bonded_addrs)
{
	struct nble_gap_sm_bond_info_param params;

	params.cb = func;
	params.user_data = user_data;
	params.include_bonded_addrs = include_bonded_addrs;

	nble_gap_sm_bond_info_req(&params);
}

void on_nble_gap_start_advertise_rsp(const struct nble_response *params)
{
	if (params->status == 0)
		atomic_set_bit(bt_dev.flags, BT_DEV_ADVERTISING);
	else
		BT_WARN("start advertise failed with %d", params->status);
}

void on_nble_gap_disconnect_evt(const struct nble_gap_disconnect_evt *evt)
{
	struct bt_conn *conn;

#if 0
	/* Nordic has no disconnection error */
	if (evt->status) {
		return;
	}
#endif

	conn = bt_conn_lookup_handle(evt->conn_handle);
	if (!conn) {
		BT_DBG("Unable to look up conn with handle %u",
		       evt->conn_handle);
		return;
	}
#if 0
	/* Check stacks usage (no-ops if not enabled) */
	stack_analyze("rx stack", rx_fiber_stack, sizeof(rx_fiber_stack));
	stack_analyze("cmd rx stack", rx_prio_fiber_stack,
		      sizeof(rx_prio_fiber_stack));
	stack_analyze("cmd tx stack", cmd_tx_fiber_stack,
		      sizeof(cmd_tx_fiber_stack));
	stack_analyze("conn tx stack", conn->stack, sizeof(conn->stack));

#endif

	conn->err = evt->hci_reason;
	bt_conn_set_state(conn, BT_CONN_DISCONNECTED);
	conn->handle = 0;

#if 0
	/* Only LE supported */
	if (conn->type != BT_CONN_TYPE_LE) {
		bt_conn_unref(conn);
		return;
	}
	/* TODO enabled when autoconn is supported */
	if (atomic_test_bit(conn->flags, BT_CONN_AUTO_CONNECT)) {
		bt_conn_set_state(conn, BT_CONN_CONNECT_SCAN);
		bt_le_scan_update(false);
	}
#endif

	bt_conn_unref(conn);
	if (atomic_test_bit(bt_dev.flags, BT_DEV_KEEP_ADVERTISING)) {
		set_advertise_enable();
	}
}

void on_nble_gap_connect_evt(const struct nble_gap_connect_evt *evt)
{
	struct bt_conn *conn;

	/* Make lookup to check if there's a connection object in CONNECT state
	 * associated with passed peer LE address.
	 */
	conn = bt_conn_lookup_state_le(&evt->peer_bda, BT_CONN_CONNECT);

#if 0
	/* Nordic has no connection error */
	if (evt->status) {
		if (!conn) {
			return;
		}

		conn->err = BT_HCI_ERR_UNACCEPT_CONN_PARAMS;
		bt_conn_set_state(conn, BT_CONN_DISCONNECTED);

		/* Drop the reference got by lookup call in CONNECT state.
		 * We are now in DISCONNECTED state since no successful LE
		 * link been made.
		 */
		bt_conn_unref(conn);

		return;
	}
#endif
	/*
	 * clear advertising even if we are not able to add connection object
	 * to keep host in sync with controller state
	 */
	if (evt->role_slave == BT_CONN_ROLE_SLAVE) {
		atomic_clear_bit(bt_dev.flags, BT_DEV_ADVERTISING);
	}

	if (!conn) {
		conn = bt_conn_add_le(&evt->peer_bda);
	}

	if (!conn) {
		BT_DBG("Unable to add new conn for handle %u",
		       evt->conn_handle);
		return;
	}

	conn->handle = evt->conn_handle;
	bt_addr_le_copy(&conn->le.dst, &evt->peer_bda);
	conn->le.interval = evt->conn_values.interval;
	conn->le.latency = evt->conn_values.latency;
	conn->le.timeout = evt->conn_values.supervision_to;
	conn->role = evt->role_slave;

#if 0
	src.type = BT_ADDR_LE_PUBLIC;
	memcpy(src.val, bt_dev.bdaddr.val, sizeof(bt_dev.bdaddr.val));

	/* use connection address (instead of identity address) as initiator
	 * or responder address
	 */
	if (conn->role == BT_HCI_ROLE_MASTER) {
		bt_addr_le_copy(&conn->le.init_addr, &src);
		bt_addr_le_copy(&conn->le.resp_addr, &evt->peer_addr);
	} else {
		bt_addr_le_copy(&conn->le.init_addr, &evt->peer_addr);
		bt_addr_le_copy(&conn->le.resp_addr, &src);
	}
#endif
	bt_conn_set_state(conn, BT_CONN_CONNECTED);

	/* Note: Connection update removed because Windows interop and BT spec recommendations */

	bt_conn_unref(conn);
#if 0
	bt_le_scan_update(false);
#endif

}

void on_nble_gap_adv_report_evt(const struct nble_gap_adv_report_evt *evt,
				const uint8_t *buf, uint8_t len)
{
#if TODO_IMPLEMENT_CONNECTION
	uint8_t num_reports = buf->data[0];
	struct bt_hci_ev_le_advertising_info *info;

	BT_DBG("Adv number of reports %u",  num_reports);

	info = net_buf_pull(buf, sizeof(num_reports));

	while (num_reports--) {
		int8_t rssi = info->data[info->length];
		const bt_addr_le_t *addr;

		BT_DBG("%s event %u, len %u, rssi %d dBm",
		       bt_addr_le_str(&info->addr),
		       info->evt_type, info->length, rssi);

		addr = find_id_addr(&info->addr);
#endif

		BT_DBG("nble gap: event:%u, len %u", evt->adv_type, len);

		if (scan_dev_found_cb) {
			scan_dev_found_cb(&evt->addr, evt->rssi, evt->adv_type,
					  buf, len);
		}
#if TODO_IMPLEMENT_CONNECTION
#if defined(CONFIG_BLUETOOTH_CONN)
		check_pending_conn(addr, &info->addr, info->evt_type);
#endif /* CONFIG_BLUETOOTH_CONN */
		/* Get next report iteration by moving pointer to right offset
		 * in buf according to spec 4.2, Vol 2, Part E, 7.7.65.2.
		 */
		info = net_buf_pull(buf, sizeof(*info) + info->length +
				    sizeof(rssi));
	}
#endif
}

void on_nble_gap_conn_update_evt(const struct nble_gap_conn_update_evt *evt)
{
	struct bt_conn *conn;
	uint16_t handle, interval;

	handle = evt->conn_handle;
	interval = evt->conn_values.interval;

/*	BT_DBG("status %u, handle %u", evt->status, handle); */

	conn = bt_conn_lookup_handle(handle);
	if (!conn) {
/*		BT_ERR("Unable to lookup conn for handle %u", handle); */
		return;
	}

/*	if (!evt->status) { */
		conn->le.interval = interval;
		conn->le.latency = evt->conn_values.latency;
		conn->le.timeout = evt->conn_values.supervision_to;
		notify_le_param_updated(conn);
/*	} */


	bt_conn_unref(conn);
}

void bt_conn_set_param_le(struct bt_conn *conn,
			  const struct bt_le_conn_param *param)
{
	conn->le.interval_min = param->interval_min;
	conn->le.interval_max = param->interval_max;
	conn->le.latency = param->latency;
	conn->le.timeout = param->timeout;
}
int bt_conn_update_param_le(struct bt_conn *conn,
			    const struct bt_le_conn_param *param)
{
#if 0
	BT_DBG("conn %p features 0x%x params (%d-%d %d %d)", conn,
	       conn->le.features[0], param->interval_min, param->interval_max,
	       param->latency, param->timeout);
#endif
	/* Check if there's a need to update conn params */
	if (conn->le.interval >= param->interval_min &&
	    conn->le.interval <= param->interval_max) {
		return -EALREADY;
	}
#if 0
	if ((conn->role == BT_HCI_ROLE_SLAVE) &&
	    !(bt_dev.le.features[0] & BT_HCI_LE_CONN_PARAM_REQ_PROC)) {
		return bt_l2cap_update_conn_param(conn, param);
	}

	if ((conn->le.features[0] & BT_HCI_LE_CONN_PARAM_REQ_PROC) &&
	    (bt_dev.le.features[0] & BT_HCI_LE_CONN_PARAM_REQ_PROC)) {
#endif
		return bt_conn_le_conn_update(conn, param);
#if 0
	}
	return -EBUSY;
#endif
}

void on_nble_gap_scan_start_stop_rsp(const struct nble_response *rsp)
{
	if (rsp->status)
		BT_INFO("scan start/stop failed: %d", rsp->status);
	/* TODO: clear scanning bit atomic_clear_bit(bt_dev.flags, BT_DEV_SCANNING) */
}

static int bt_hci_stop_scanning(void)
{
#ifdef NOT_USED_FOR_NOW
	struct net_buf *buf, *rsp;
	struct bt_hci_cp_le_set_scan_enable *scan_enable;
	int err;

	if (!atomic_test_bit(bt_dev.flags, BT_DEV_SCANNING)) {
		return -EALREADY;
	}

	buf = bt_hci_cmd_create(BT_HCI_OP_LE_SET_SCAN_ENABLE,
				sizeof(*scan_enable));
	if (!buf) {
		return -ENOBUFS;
	}

	scan_enable = net_buf_add(buf, sizeof(*scan_enable));
	memset(scan_enable, 0, sizeof(*scan_enable));
	scan_enable->filter_dup = BT_HCI_LE_SCAN_FILTER_DUP_DISABLE;
	scan_enable->enable = BT_HCI_LE_SCAN_DISABLE;

	err = bt_hci_cmd_send_sync(BT_HCI_OP_LE_SET_SCAN_ENABLE, buf, &rsp);
	if (err) {
		return err;
	}

	/* Update scan state in case of success (0) status */
	err = rsp->data[0];
	if (!err) {
		atomic_clear_bit(bt_dev.flags, BT_DEV_SCANNING);
	}

	net_buf_unref(rsp);

	return err;
#endif

	nble_gap_stop_scan_req();

	return 0;
}

#if defined(CONFIG_BLUETOOTH_CENTRAL)
int bt_le_set_auto_conn(bt_addr_le_t *addr,
			const struct bt_le_conn_param *param)
{
	return -EINVAL;
}
#endif /* CONFIG_BLUETOOTH_CENTRAL */


static int start_le_scan(uint8_t scan_type, uint16_t interval, uint16_t window,
			 uint8_t filter_dup)
{
	struct nble_gap_scan_params params = {
			.interval = interval,
			.window = window,
			.scan_type = scan_type,
	};

#ifdef NOT_USED_FOR_NOW
	struct net_buf *buf, *rsp;
	struct bt_hci_cp_le_set_scan_params *set_param;
	struct bt_hci_cp_le_set_scan_enable *scan_enable;
	int err;

	buf = bt_hci_cmd_create(BT_HCI_OP_LE_SET_SCAN_PARAMS,
				sizeof(*set_param));
	if (!buf) {
		return -ENOBUFS;
	}


	set_param = net_buf_add(buf, sizeof(*set_param));
	memset(set_param, 0, sizeof(*set_param));
	set_param->scan_type = scan_type;

	/* for the rest parameters apply default values according to
	 *  spec 4.2, vol2, part E, 7.8.10
	 */
	set_param->interval = sys_cpu_to_le16(interval);
	set_param->window = sys_cpu_to_le16(window);
	set_param->filter_policy = 0x00;

	if (scan_type == BT_HCI_LE_SCAN_ACTIVE) {
		err = le_set_nrpa();
		if (err) {
			net_buf_unref(buf);
			return err;
		}

		set_param->addr_type = BT_ADDR_LE_RANDOM;
	} else {
		set_param->addr_type = BT_ADDR_LE_PUBLIC;
	}

	bt_hci_cmd_send(BT_HCI_OP_LE_SET_SCAN_PARAMS, buf);
	buf = bt_hci_cmd_create(BT_HCI_OP_LE_SET_SCAN_ENABLE,
				sizeof(*scan_enable));
	if (!buf) {
		return -ENOBUFS;
	}

	scan_enable = net_buf_add(buf, sizeof(*scan_enable));
	memset(scan_enable, 0, sizeof(*scan_enable));
	scan_enable->filter_dup = filter_dup;
	scan_enable->enable = BT_HCI_LE_SCAN_ENABLE;

	err = bt_hci_cmd_send_sync(BT_HCI_OP_LE_SET_SCAN_ENABLE, buf, &rsp);
	if (err) {
		return err;
	}
	/* Update scan state in case of success (0) status */
	err = rsp->data[0];
	if (!err) {
		atomic_set_bit(bt_dev.flags, BT_DEV_SCANNING);
	}

	net_buf_unref(rsp);
#endif

	nble_gap_start_scan_req(&params);

	return 0;
}

#if NOT_USED_FOR_NOW
/* Used to determine whether to start scan and which scan type should be used */
int bt_le_scan_update(bool fast_scan)
{
#if defined(CONFIG_BLUETOOTH_CENTRAL)
	uint16_t interval, window;
	struct bt_conn *conn;
#endif /* CONFIG_BLUETOOTH_CENTRAL */

	if (atomic_test_bit(bt_dev.flags, BT_DEV_EXPLICIT_SCAN)) {
		return 0;
	}

	if (atomic_test_bit(bt_dev.flags, BT_DEV_SCANNING)) {
		int err;

		err = bt_hci_stop_scanning();
		if (err) {
			return err;
		}
	}

#if defined(CONFIG_BLUETOOTH_CENTRAL)
	conn = bt_conn_lookup_state_le(NULL, BT_CONN_CONNECT_SCAN);
	if (!conn) {
		return 0;
	}

	bt_conn_unref(conn);

	if (fast_scan) {
		interval = BT_GAP_SCAN_FAST_INTERVAL;
		window = BT_GAP_SCAN_FAST_WINDOW;
	} else {
		interval = BT_GAP_SCAN_SLOW_INTERVAL_1;
		window = BT_GAP_SCAN_SLOW_WINDOW_1;
	}

	return start_le_scan(BT_HCI_LE_SCAN_PASSIVE, interval, window, 0x01);
#else
	return 0;
#endif /* CONFIG_BLUETOOTH_CENTRAL */
}
#endif

static int bt_init(void)
{
#if NOT_USED_FOR_NOW
	struct bt_driver *drv = bt_dev.drv;
#endif
	int err = 0;

#if NOT_USED_FOR_NOW
	err = drv->open();
	if (err) {
		BT_ERR("HCI driver open failed (%d)", err);
		return err;
	}

	err = hci_init();
#endif

	if (!err) {
		err = bt_conn_init();
	}

	scan_dev_found_cb = NULL;
	if (!err) {
		atomic_set_bit(bt_dev.flags, BT_DEV_READY);
#if 0
		bt_le_scan_update(false);
#endif
	}

	return err;
}

void on_nble_up(void)
{
    BT_DBG("%s", __FUNCTION__);
	if (bt_ready_cb)
		bt_ready_cb(bt_init());
}

extern void on_nble_curie_init(void);

int bt_enable(bt_ready_cb_t cb)
{
	bt_ready_cb = cb;

	on_nble_curie_init();

	if (!cb) {
		return bt_init();
	}

	return 0;
}


static bool valid_adv_param(const struct bt_le_adv_param *param)
{
	switch (param->type) {
	case BT_LE_ADV_IND:
	case BT_LE_ADV_SCAN_IND:
	case BT_LE_ADV_NONCONN_IND:
		break;
	default:
		return false;
	}

#if 0
	/* checks done in Nordic */
	switch (param->addr_type) {
	case BT_LE_ADV_ADDR_IDENTITY:
	case BT_LE_ADV_ADDR_NRPA:
		break;
	default:
		return false;
	}

	if (param->interval_min > param->interval_max ||
	    param->interval_min < 0x0020 || param->interval_max > 0x4000) {
		return false;
	}
#endif

	return true;
}

static int set_ad(struct bt_eir_data *p_ad_data,
		const struct bt_data *ad, size_t ad_len)
{
	int i;

	for (i = 0; i < ad_len; i++) {
		/* Check if ad fit in the remaining buffer */
		if (p_ad_data->len + ad[i].data_len + 2 > 31) {
			return -EINVAL;
		}

		p_ad_data->data[p_ad_data->len++] = ad[i].data_len + 1;
		p_ad_data->data[p_ad_data->len++] = ad[i].type;

		memcpy(&p_ad_data->data[p_ad_data->len], ad[i].data,
				ad[i].data_len);
		p_ad_data->len += ad[i].data_len;
	}

	return 0;
}

int bt_le_adv_start(const struct bt_le_adv_param *param,
			const struct bt_data *ad, size_t ad_len,
			const struct bt_data *sd, size_t sd_len)
{
	int err;
	struct nble_gap_adv_params set_param = {0};
	struct nble_gap_ad_data_params data;

	if (!valid_adv_param(param)) {
		return -EINVAL;
	}

	memset(&data, 0, sizeof(data));

	if (atomic_test_bit(bt_dev.flags, BT_DEV_KEEP_ADVERTISING)) {
		return -EALREADY;
	}

	err = set_advertise_disable();
	if (err) {
		return err;
	}
	err = set_ad(&data.ad, ad, ad_len);
	if (err) {
		return err;
	}

	/*
	 * Don't bother with scan response if the advertising type isn't
	 * a scannable one.
	 */
	if (param->type == BT_LE_ADV_IND || param->type == BT_LE_ADV_SCAN_IND) {
		err = set_ad(&data.sd, sd, sd_len);
		if (err) {
			return err;
		}
	}
	nble_gap_set_adv_data_req(&data);

	/* Timeout is handled by application timer */
	set_param.timeout = 0;
	/* forced to none currently (no whitelist support) */
	set_param.filter_policy = 0;
	set_param.interval_max = param->interval_max;
	set_param.interval_min = param->interval_min;
	set_param.type = param->type;
	nble_gap_set_adv_params_req(&set_param);

#if 0
	if (param->addr_type == BT_LE_ADV_ADDR_NRPA) {
		err = le_set_nrpa();
		if (err) {
			net_buf_unref(buf);
			return err;
		}

		set_param->own_addr_type = BT_ADDR_LE_RANDOM;
	} else {
		set_param->own_addr_type = BT_ADDR_LE_PUBLIC;
	}

	bt_hci_cmd_send(BT_HCI_OP_LE_SET_ADV_PARAMETERS, buf);
#endif

	err = set_advertise_enable();
	if (err) {
		return err;
	}

	atomic_set_bit(bt_dev.flags, BT_DEV_KEEP_ADVERTISING);

	return 0;
}

void on_nble_gap_dir_adv_timeout_evt(const struct nble_gap_dir_adv_timeout_evt *p_evt)
{
	struct bt_conn *conn = bt_conn_lookup_state_le(BT_ADDR_LE_ANY, BT_CONN_CONNECT);

	if (conn) {
		atomic_clear_bit(bt_dev.flags, BT_DEV_ADVERTISING);
		conn->err = p_evt->error;
		bt_conn_set_state(conn, BT_CONN_DISCONNECTED);
		bt_conn_unref(conn);
	}
}

int bt_le_adv_stop(void)
{
	int err;

	if (!atomic_test_bit(bt_dev.flags, BT_DEV_KEEP_ADVERTISING)) {
		return -EALREADY;
	}

	err = set_advertise_disable();
	if (err) {
		return err;
	}
	atomic_clear_bit(bt_dev.flags, BT_DEV_KEEP_ADVERTISING);

	return 0;
}

static bool valid_le_scan_param(const struct bt_le_scan_param *param)
{
	if (param->type != BT_HCI_LE_SCAN_PASSIVE &&
	    param->type != BT_HCI_LE_SCAN_ACTIVE) {
		return false;
	}

	if (/* param->filter_dup != BT_HCI_LE_SCAN_FILTER_DUP_DISABLE */
	    /* && nble always filters duplicates */
	    param->filter_dup != BT_HCI_LE_SCAN_FILTER_DUP_ENABLE) {
		return false;
	}

	if (param->interval < 0x0004 || param->interval > 0x4000) {
		return false;
	}

	if (param->window < 0x0004 || param->window > 0x4000) {
		return false;
	}

	if (param->window > param->interval) {
		return false;
	}

	return true;
}

int bt_le_scan_start(const struct bt_le_scan_param *param, bt_le_scan_cb_t cb)
{

	int err;

	/* Check that the parameters have valid values */
	if (!valid_le_scan_param(param)) {
		return -EINVAL;
	}
#if NOT_USED_FOR_NOW
	/* Return if active scan is already enabled */
	if (atomic_test_and_set_bit(bt_dev.flags, BT_DEV_EXPLICIT_SCAN)) {
		return -EALREADY;
	}

	if (atomic_test_bit(bt_dev.flags, BT_DEV_SCANNING)) {
		err = bt_hci_stop_scanning();
		if (err) {
			atomic_clear_bit(bt_dev.flags, BT_DEV_EXPLICIT_SCAN);
			return err;
		}
	}
#endif

	err = start_le_scan(param->type, param->interval, param->window,
			param->filter_dup);


	if (err) {
#if NOT_USED_FOR_NOW
		atomic_clear_bit(bt_dev.flags, BT_DEV_EXPLICIT_SCAN);
#endif
		return err;
	}

	scan_dev_found_cb = cb;

	return 0;
}

int bt_le_scan_stop(void)
{
#if NOT_USED_FOR_NOW
	/* Return if active scanning is already disabled */
	if (!atomic_test_and_clear_bit(bt_dev.flags, BT_DEV_EXPLICIT_SCAN)) {
		return -EALREADY;
	}
#endif
	scan_dev_found_cb = NULL;

#if NOT_USED_FOR_NOW
	return bt_le_scan_update(false);
#else
	return bt_hci_stop_scanning();
#endif
}

/* Temporary RSSI patch for UAS: RPC need definition if UAS not compiled */
__attribute__((weak))
void on_nble_uas_bucket_change(const struct nble_uas_bucket_change *p_params)
{
}

void ble_gap_set_rssi_report(struct nble_rssi_report_params *params,
				struct bt_conn *conn,
				rssi_report_resp_t resp_cb, rssi_report_t evt_cb)
{
	rssi_report_cb = evt_cb;

	params->conn_handle = conn->handle;

	nble_gap_set_rssi_report_req(params, resp_cb);
}

void on_nble_gap_set_rssi_report_rsp(const struct nble_response *params)
{
	rssi_report_resp_t resp_cb = params->user_data;

	if (resp_cb)
		resp_cb(params->status);
}

void on_nble_gap_rssi_evt(const struct nble_gap_rssi_evt *event)
{
	if (rssi_report_cb)
		rssi_report_cb(event->rssi_data);
}

void ble_gap_set_tx_power(int8_t tx_power)
{
	struct nble_gap_tx_power_params params = {
		.tx_power = tx_power,
	};
	nble_gap_tx_power_req(&params);
}

void on_nble_gap_tx_power_rsp(const struct nble_response *params)
{
}

void ble_gap_get_version(ble_get_version_cb_t func)
{
	struct nble_gap_get_version_param params;

	params.cb = func;

	nble_get_version_req(&params);
}

void on_nble_get_version_rsp(const struct nble_version_response *par)
{
	struct nble_gap_get_version_param param = par->params;
	ble_get_version_cb_t cb = param.cb;

	if (cb) {
		cb(&par->ver);
	}
}

void bt_le_set_device_name(char *device_name, int len)
{
    struct nble_gap_service_write_params gap_service_params;
    if (len > 20)
        len = 20;
    memset(&gap_service_params, 0, sizeof(gap_service_params));
    gap_service_params.attr_type = NBLE_GAP_SVC_ATTR_NAME;
    gap_service_params.name.len = len;
    gap_service_params.name.sec_mode = 0x11;// GAP_SEC_LEVEL_1 | GAP_SEC_MODE_1;
    memcpy(gap_service_params.name.name_array, device_name, len);
    nble_gap_service_write_req(&gap_service_params);
}

void bt_le_set_mac_address(bt_addr_le_t bda)
{
    // Update the MAC addr
    struct nble_set_bda_params params;
    params.cb = NULL;
    params.user_data = NULL;
    params.bda = bda;

    nble_set_bda_req(&params);
}

