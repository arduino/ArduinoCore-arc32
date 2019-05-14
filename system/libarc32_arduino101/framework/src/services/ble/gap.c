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
#include <bluetooth/storage.h>
#include "storage_internal.h"
#include "gap_internal.h"
#include "conn_internal.h"

#include "hci_core.h"

#if defined(CONFIG_BLUETOOTH_SMP)
#include "smp.h"
#endif /* CONFIG_BLUETOOTH_SMP */

/* #define BT_GATT_DEBUG 1 */

#ifndef __weak
#define __weak __attribute__((weak))
#endif

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

static bt_ready_cb_t bt_ready_cb;
static bt_le_scan_cb_t *scan_dev_found_cb;
static rssi_report_t rssi_report_cb;
static const struct bt_storage *bt_storage;

struct bt_dev bt_dev;

int set_advertise_enable(void)
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

void ble_gap_get_bda_info(struct bt_local_addr *addr)
{
	bt_addr_le_copy(&addr->id_addr, &bt_dev.id_addr);
#if defined(CONFIG_BLUETOOTH_PRIVACY)
	bt_addr_le_copy(&addr->rpa, &bt_dev.random_addr);
#endif
}

void on_nble_gap_start_adv_rsp(const struct nble_common_rsp *par)
{
	if (par->status == 0)
		atomic_set_bit(bt_dev.flags, BT_DEV_ADVERTISING);
	else
		BT_WARN("start advertise failed with %d", par->status);
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

	/* use connection address (instead of identity address) as initiator
	 * or responder address
	 */

	if (conn->role == BT_HCI_ROLE_MASTER) {
		bt_addr_le_copy(&conn->le.resp_addr, &evt->peer_bda);

		/* init_addr doesn't need updating here since it was
		* already set during previous steps.
		*/
	} else {
		bt_addr_le_copy(&conn->le.init_addr, &evt->peer_bda);

#if defined(CONFIG_BLUETOOTH_PRIVACY)
		bt_addr_le_copy(&conn->le.resp_addr, &bt_dev.random_addr);
#else
		/* id_addr is equal with peer_bda */
		bt_addr_le_copy(&conn->le.resp_addr,  &evt->peer_bda);
#endif /* CONFIG_BLUETOOTH_PRIVACY */
	}

	bt_conn_set_state(conn, BT_CONN_CONNECTED);

	/* Note: Connection update removed because Windows interop and BT spec recommendations */

	bt_conn_unref(conn);
#if 0
	bt_le_scan_update(false);
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

void on_nble_gap_scan_start_stop_rsp(const struct nble_common_rsp *rsp)
{
	if (rsp->status) {
		BT_INFO("scan start/stop failed: %d", rsp->status);
	    atomic_clear_bit(bt_dev.flags, BT_DEV_SCANNING);
	}
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
	atomic_clear_bit(bt_dev.flags, BT_DEV_SCANNING);

	nble_gap_stop_scan_req();

	return 0;
}

#if defined(CONFIG_BLUETOOTH_CENTRAL)
static int hci_le_create_conn(struct bt_conn *conn)
{
	struct nble_gap_connect_req conn_params;

    memset(&conn_params, 0, sizeof(conn_params));
    
	conn_params.bda = conn->le.dst;
	conn_params.conn_params.interval_min = conn->le.interval_min;
	conn_params.conn_params.interval_max = conn->le.interval_max;
	conn_params.conn_params.slave_latency = conn->le.latency;
	conn_params.conn_params.link_sup_to = conn->le.timeout;

	conn_params.scan_params.interval = BT_GAP_SCAN_FAST_INTERVAL;
	conn_params.scan_params.window = conn_params.scan_params.interval;

	nble_gap_connect_req(&conn_params, conn);

	return 0;
}

static void check_pending_conn(const bt_addr_le_t *id_addr,
			       const bt_addr_le_t *addr, uint8_t evtype)
{
	struct bt_conn *conn;

	/* No connections are allowed during explicit scanning */
	if (atomic_test_bit(bt_dev.flags, BT_DEV_EXPLICIT_SCAN)) {
		return;
	}

	/* Return if event is not connectable */
	if (evtype != BT_LE_ADV_IND && evtype != BT_LE_ADV_DIRECT_IND) {
		return;
	}

	conn = bt_conn_lookup_state_le(id_addr, BT_CONN_CONNECT_SCAN);
	if (!conn) {
		return;
	}

	if (bt_hci_stop_scanning()) {
		goto failed;
	}

#if defined(CONFIG_BLUETOOTH_PRIVACY)
	if (le_set_rpa()) {
		return;
	}
	conn->le.init_addr.type = BT_ADDR_LE_RANDOM;
#else
	bt_addr_le_copy(&conn->le.init_addr, &bt_dev.id_addr);
#endif /* CONFIG_BLUETOOTH_PRIVACY */

	bt_addr_le_copy(&conn->le.resp_addr, addr);

	if (hci_le_create_conn(conn)) {
		goto failed;
	}

	bt_conn_set_state(conn, BT_CONN_CONNECT);
	bt_conn_unref(conn);
	return;

failed:
	conn->err = BT_HCI_ERR_UNSPECIFIED;
	bt_conn_set_state(conn, BT_CONN_DISCONNECTED);
	bt_conn_unref(conn);
	bt_le_scan_update(false);
}
#endif

void on_nble_gap_adv_report_evt(const struct nble_gap_adv_report_evt *evt,
				const uint8_t *buf, uint8_t len)
{
	BT_DBG("nble gap: event:%u, len %u", evt->adv_type, len);

	if (scan_dev_found_cb) {
		scan_dev_found_cb(&evt->addr, evt->rssi, evt->adv_type,
				  buf, len);
	}
#if defined(CONFIG_BLUETOOTH_CENTRAL)
	check_pending_conn(&evt->addr, &evt->addr, evt->adv_type);
#endif /* CONFIG_BLUETOOTH_CENTRAL */
}
#if 0
static int set_random_address(const bt_addr_t *addr)
{
	struct nble_set_bda_req req = {0};

	memcpy(&req.bda.a, addr, sizeof(bt_addr_t));
	req.bda.type = BT_ADDR_LE_RANDOM;
	req.cb = NULL;

	nble_set_bda_req(&req);
	return 0;
}

static int le_set_nrpa(void)
{
	bt_addr_t nrpa = {{0}};

	nrpa.val[5] &= 0x3f;

	return set_random_address(&nrpa);
}
#endif

#if defined(CONFIG_BLUETOOTH_PRIVACY)
int le_set_rpa(void)
{
	bt_addr_t rpa = {0};

	/* Set the two most significant bits to 01 (indicating an RPA) */
	rpa.val[5] &= 0x3f;
	rpa.val[5] |= 0x40;

	return set_random_address(&rpa);
}
#endif

static int start_le_scan(uint8_t scan_type, uint16_t interval, uint16_t window,
			 uint8_t filter_dup)
{
	struct nble_gap_start_scan_req params;
#if 0
	int err;

#if defined(CONFIG_BLUETOOTH_PRIVACY)
	err = le_set_rpa();
	if (err) {
		return err;
	}
#else
	if (scan_type == BT_HCI_LE_SCAN_ACTIVE) {
		/* only set NRPA if there is no advertising ongoing */
		if (!atomic_test_bit(bt_dev.flags, BT_DEV_ADVERTISING)) {
			err = le_set_nrpa();
			if (err) {
				return err;
			}
		}
	}
#endif
#endif
	params.scan_params.interval  = interval;
	params.scan_params.window    = window;
	params.scan_params.scan_type = scan_type;
	params.scan_params.use_whitelist = 0;

	atomic_set_bit(bt_dev.flags, BT_DEV_SCANNING);

	nble_gap_start_scan_req(&params);

	return 0;
}

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

static void update_le_oob_local(const bt_addr_le_t *addr)
{
	if (bt_addr_le_is_identity(addr)) {
		bt_addr_le_copy(&bt_dev.id_addr, addr);
	} else if (bt_addr_le_is_rpa(addr)) {
		bt_addr_le_copy(&bt_dev.random_addr, addr);
	}
}

static void nble_get_bda_cb_init(const bt_addr_le_t *bda, void *user_data)
{
	update_le_oob_local(bda);
}

static int common_init(void)
{
#if 0
	struct nble_get_bda_req req;

	req.cb = nble_get_bda_cb_init;
	req.user_data =NULL;

	/* read nble identity address */
	nble_get_bda_req(&req);
#endif
	return 0;
}

static int hci_init(void)
{
	if (bt_storage) {
		int ret;
		struct nble_set_bda_req params;

		ret = bt_storage->read(NULL, BT_STORAGE_ID_ADDR, &params.bda,
				       sizeof(params.bda));

		if (!ret) {
			params.cb = NULL;
			params.user_data = NULL;

			nble_set_bda_req(&params);
			/* nble_get_bda_req() returns the set address but this
			 * ensures that bt_le_oob_get_local() reads a valid
			 * address independent of nble async response speed */
			update_le_oob_local(&params.bda);
			return 0;
		}
		/* in no provisioned bda available -> use nble one */
	}
	BT_DBG("no id addr provisioned");

	return common_init();
}

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
#endif
	err = hci_init();


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

void version_at_init_cb(const struct nble_version *ver)
{
	if (bt_ready_cb)
		bt_ready_cb(bt_init());
}

void rpc_init_cb(uint32_t version, bool compatible)
{
	/* Retrieve the Nordic version */
	ble_gap_get_version(version_at_init_cb);
}

__weak
void nble_curie_unreset_hook(void)
{
}

int bt_enable(bt_ready_cb_t cb)
{
	bt_ready_cb = cb;

	nble_curie_unreset_hook();

	if (!cb) {
		return bt_init();
	}

	return 0;
}


static bool valid_adv_param(const struct bt_le_adv_param *param)
{
	if (!(param->options & BT_LE_ADV_OPT_CONNECTABLE)) {
		/*
		 * BT Core 4.2 [Vol 2, Part E, 7.8.5]
		 * The Advertising_Interval_Min and Advertising_Interval_Max
		 * shall not be set to less than 0x00A0 (100 ms) if the
		 * Advertising_Type is set to ADV_SCAN_IND or ADV_NONCONN_IND.
		 */
		if (param->interval_min < 0x00a0) {
			return false;
		}
	}

	if (param->interval_min > param->interval_max ||
	    param->interval_min < 0x0020 || param->interval_max > 0x4000) {
		return false;
	}

	return true;
}

static int set_ad(struct nble_eir_data *p_ad_data,
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
	struct nble_gap_set_adv_params_req set_param = {0};
	struct nble_gap_set_adv_data_req data = {{0}, {0}};

	if (!valid_adv_param(param)) {
		return -EINVAL;
	}

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
	 * We need to set SCAN_RSP when enabling advertising type that allows
	 * for Scan Requests.
	 *
	 * If sd was not provided but we enable connectable undirected
	 * advertising sd needs to be cleared from values set by previous calls.
	 * Clearing sd is done by calling set_ad() with NULL data and zero len.
	 * So following condition check is unusual but correct.
	 */
	if (sd || (param->options & BT_LE_ADV_OPT_CONNECTABLE)) {
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
	
	if (param->options & BT_LE_ADV_OPT_CONNECTABLE) {
		set_param.type = BT_LE_ADV_IND;
	} else {
		if (sd) {
			set_param.type = BT_LE_ADV_SCAN_IND;
		} else {
			set_param.type = BT_LE_ADV_NONCONN_IND;
		}
	}

	nble_gap_set_adv_params_req(&set_param);

#if defined(CONFIG_BLUETOOTH_PRIVACY)
	if (set_param.type == BT_LE_ADV_IND) {
		err = le_set_rpa();
		if (err)
			return err;
	} else {
		err = le_set_nrpa();
		if (err)
			return err;
	}
#endif

	err = set_advertise_enable();
	if (err) {
		return err;
	}

	atomic_set_bit(bt_dev.flags, BT_DEV_KEEP_ADVERTISING);

	return 0;
}

void on_nble_gap_dir_adv_timeout_evt(const struct nble_gap_dir_adv_timeout_evt *evt)
{
	struct bt_conn *conn = bt_conn_lookup_state_le(BT_ADDR_LE_ANY, BT_CONN_CONNECT);

	if (conn) {
		atomic_clear_bit(bt_dev.flags, BT_DEV_ADVERTISING);
		if (atomic_test_and_clear_bit(conn->flags, BT_CONN_DIR_ADV_CONNECT)) {
			atomic_clear_bit(bt_dev.flags, BT_DEV_KEEP_ADVERTISING);
		}
		conn->err = evt->error;
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

	err = start_le_scan(param->type, param->interval, param->window,
			param->filter_dup);


	if (err) {
		atomic_clear_bit(bt_dev.flags, BT_DEV_EXPLICIT_SCAN);
		return err;
	}

	scan_dev_found_cb = cb;

	return 0;
}

int bt_le_scan_stop(void)
{
	/* Return if active scanning is already disabled */
	if (!atomic_test_and_clear_bit(bt_dev.flags, BT_DEV_EXPLICIT_SCAN)) {
		return -EALREADY;
	}

	scan_dev_found_cb = NULL;

	return bt_le_scan_update(false);
}

/* Temporary RSSI patch for UAS: RPC need definition if UAS not compiled */
__weak
void on_nble_uas_bucket_change(const struct nble_uas_bucket_change *p_params)
{
}

void ble_gap_set_rssi_report(struct ble_rssi_report_params *par,
				struct bt_conn *conn,
				rssi_report_resp_t resp_cb, rssi_report_t evt_cb)
{
	struct nble_gap_set_rssi_report_req req;
	rssi_report_cb = evt_cb;

	req.conn_handle = conn->handle;
	req.op          = par->op;
	req.channel     = par->channel;
	req.delta_dBm   = par->delta_dBm;
	req.min_count   = par->min_count;

	nble_gap_set_rssi_report_req(&req, resp_cb);
}

void on_nble_gap_set_rssi_report_rsp(const struct nble_common_rsp *rsp)
{
	rssi_report_resp_t resp_cb = rsp->user_data;

	if (resp_cb)
		resp_cb(rsp->status);
}

void on_nble_gap_rssi_evt(const struct nble_gap_rssi_evt *evt)
{
	if (rssi_report_cb)
		rssi_report_cb(evt->rssi_data);
}

void ble_gap_set_tx_power(int8_t tx_power)
{
	struct nble_gap_set_tx_power_req params = {
		.tx_power = tx_power,
	};
	nble_gap_set_tx_power_req(&params);
}

void on_nble_gap_tx_power_rsp(const struct nble_common_rsp *rsp)
{
}

void ble_gap_get_version(ble_get_version_cb_t func)
{
	nble_get_version_req(func);
}

void on_nble_get_version_rsp(const struct nble_get_version_rsp *rsp)
{
	ble_get_version_cb_t cb = rsp->cb;

	if (cb) {
		cb(&rsp->ver);
	}
}

void on_nble_set_bda_rsp(const struct nble_set_bda_rsp *rsp)
{
	update_le_oob_local(&rsp->bda);

	if (rsp->cb) {
		rsp->cb(rsp->status, rsp->user_data, &rsp->bda);
	}
}

void on_nble_get_bda_rsp(const struct nble_get_bda_rsp *rsp)
{
	if (rsp->cb) {
		rsp->cb(&rsp->bda, rsp->user_data);
	}
}

__weak
void on_nble_uart_test_evt(const struct nble_uart_test_evt *evt,
	     const uint8_t *data, uint8_t len)
{
}

int bt_le_oob_get_local(struct bt_le_oob *oob)
{
#if defined(CONFIG_BLUETOOTH_PRIVACY)
	bt_addr_le_copy(&oob->addr, &bt_dev.random_addr);
#else
	bt_addr_le_copy(&oob->addr, &bt_dev.id_addr);
#endif /* CONFIG_BLUETOOTH_PRIVACY */

	return 0;
}

void on_nble_gap_rpa_update_evt(const struct nble_gap_rpa_update_evt *evt)
{
#if defined(CONFIG_BLUETOOTH_PRIVACY)
	/* Update the RPA address */
	bt_addr_le_copy(&bt_dev.random_addr, &evt->addr);
#endif /* CONFIG_BLUETOOTH_PRIVACY */
}

/* storage interface */
void bt_storage_register(const struct bt_storage *storage)
{
	bt_storage = storage;
}

void on_nble_storage_read_evt(const struct nble_storage_read_evt *evt)
{
	struct nble_storage_read_rsp_req req;
	int len;

	req.status = -ENOTSUP;
	req.addr = evt->addr;
	req.key = evt->key;

	switch (evt->key) {
	case BT_STORAGE_ID_ADDR:
		len = sizeof(bt_addr_le_t);
		break;

	case BT_STORAGE_ADDRESSES:
		len = sizeof(bt_addr_le_t) * evt->max_num_keys;
		break;

	case BT_STORAGE_SLAVE_LTK:
	case BT_STORAGE_LTK:
		len = sizeof(struct bt_storage_ltk);
		break;

	case BT_STORAGE_IRK:
		len = 16; /* irk[16] */
		break;

	default:
		len = 64; /* P256 key */
		break;
	}

	BT_DBG("nble_storage_rd_evt(key:0x%x) len:%d", evt->key, len);

	if (bt_storage) {
		uint8_t data[len];

		req.status = bt_storage->read(&evt->addr, evt->key, data, len);

		if (req.status < 0) {
			BT_DBG("storage rd failed ret:%d", req.status);
			len = 0;
		} else {
			if (evt->key != BT_STORAGE_ADDRESSES) {
				BT_ASSERT(req.status == len);
			}

			len = req.status;
			req.status = 0;
		}

		nble_storage_read_rsp_req(&req, data, len);
	} else {
		BT_DBG("No storage driver!");
		nble_storage_read_rsp_req(&req, NULL, 0);
	}
}

void on_nble_storage_write_evt(const struct nble_storage_write_evt *evt,
			       const uint8_t *data, uint16_t len)
{
	int ret;

	if (!bt_storage) {
		BT_DBG("No storage driver!");
		return;
	}

	ret = bt_storage->write(&evt->addr, evt->key, data, len);
	if (0 > ret) {
		BT_ERR("storage write failed ret:%d", ret);
	}
	BT_DBG("storage wr(key:0x%x len:%d)", evt->key, len);
}

int bt_storage_clear(bt_addr_le_t *addr)
{
#if defined(CONFIG_BLUETOOTH_SMP)
	struct nble_sm_clear_bonds_req params;

	memset(&params, 0, sizeof(params));
#endif
	if (addr) {
		struct bt_conn *conn;

		conn = bt_conn_lookup_addr_le(addr);
		if (conn) {
			bt_conn_disconnect(conn,
					   BT_HCI_ERR_REMOTE_USER_TERM_CONN);
			bt_conn_unref(conn);
		}

#if defined(CONFIG_BLUETOOTH_SMP)
		bt_addr_le_copy(&params.addr, addr);
		nble_sm_clear_bonds_req(&params);
#endif

		if (bt_storage) {
			return bt_storage->clear(addr);
		}

		return 0;
	}

	bt_conn_disconnect_all();

#if defined(CONFIG_BLUETOOTH_SMP)
	/* BT_ADDR_LE_ANY clears all */
	nble_sm_clear_bonds_req(&params);
#endif

	if (bt_storage) {
		return bt_storage->clear(NULL);
	}

	return 0;
}

void bt_le_set_device_name(char *device_name, int len)
{
    struct nble_gap_service_req gap_service_params;
    if (len > 20)
        len = 20;
    memset(&gap_service_params, 0, sizeof(gap_service_params));
    gap_service_params.attr_type = NBLE_GAP_SVC_ATTR_NAME;
    gap_service_params.name.len = len;
    gap_service_params.name.sec_mode = 0x11;// GAP_SEC_LEVEL_1 | GAP_SEC_MODE_1;
    memcpy(gap_service_params.name.name_array, device_name, len);
    nble_gap_service_req(&gap_service_params);
}

void bt_le_set_mac_address(bt_addr_le_t bda)
{
    // Update the MAC addr
    struct nble_set_bda_req params;
    params.cb = NULL;
    params.user_data = NULL;
    params.bda = bda;

    nble_set_bda_req(&params);
    nble_get_bda_cb_init(&bda, NULL);
}

void on_nble_common_rsp(const struct nble_common_rsp *rsp)
{
        if (rsp->status) {
                BT_ERR("Last request failed, error %d", rsp->status);
                return;
        }

        BT_DBG("status %d", rsp->status);
}

void on_nble_gap_set_tx_power_rsp(const struct nble_common_rsp *rsp)
{
    /* stub */
}


