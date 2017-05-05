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

#ifndef GAP_INTERNAL_H_
#define GAP_INTERNAL_H_

#include <stdint.h>
/* For bt_addr_le_t */
#include <bluetooth/hci.h>
/* For bt_security_t */
#include <bluetooth/conn.h>

/* Directed advertisement timeout error after 1.28s */
#define ERR_DIRECTED_ADVERTISING_TIMEOUT 0x3C

enum NBLE_TEST_OPCODES {
	NBLE_TEST_INIT_DTM = 0x01,
	NBLE_TEST_START_DTM_RX = 0x1d,
	NBLE_TEST_START_DTM_TX = 0x1e,
	NBLE_TEST_END_DTM = 0x1f,
	/* vendor specific commands start at 0x80 */
	/* Set Tx power. To be called before start of tx test */
	NBLE_TEST_SET_TXPOWER = 0x80,
	NBLE_TEST_START_TX_CARRIER,
};

/*  DTM commands, opcodes, indexes. */
#define H4_CMD              0x01
#define HCI_OGF_LE_CMD      0x20

#define DTM_HCI_STATUS_IDX    6
#define DTM_HCI_LE_END_IDX    (DTM_HCI_STATUS_IDX + 1)



/* bt_dev flags: the flags defined here represent BT controller state */
enum {
	BT_DEV_READY,

	BT_DEV_ADVERTISING,
	BT_DEV_KEEP_ADVERTISING,
	BT_DEV_SCANNING,
	BT_DEV_EXPLICIT_SCAN,

#if defined(CONFIG_BLUETOOTH_BREDR)
	BT_DEV_ISCAN,
	BT_DEV_PSCAN,
#endif /* CONFIG_BLUETOOTH_BREDR */
};

struct nble_log_s {
	uint32_t param0;
	uint32_t param1;
	uint32_t param2;
};

void nble_log(const struct nble_log_s *par, char *data, uint8_t len);

struct nble_common_rsp {
	int status;
	void *user_data;
};

struct bt_local_addr {
	bt_addr_le_t id_addr;
#if defined(CONFIG_BLUETOOTH_PRIVACY)
	bt_addr_le_t rpa;
#endif
};

void on_nble_common_rsp(const struct nble_common_rsp *rsp);

void nble_panic_req(void *user_data);

struct nble_version {
	uint8_t version;
	uint8_t major;
	uint8_t minor;
	uint8_t patch;
	char version_string[20];
	uint8_t build_hash[4];
	uint8_t hash[4];
};

typedef void (*ble_get_version_cb_t)(const struct nble_version *ver);

void nble_get_version_req(ble_get_version_cb_t cb);

struct nble_get_version_rsp {
	ble_get_version_cb_t cb;
	struct nble_version ver;
};

void on_nble_get_version_rsp(const struct nble_get_version_rsp *rsp);

enum NBLE_GAP_SVC_ATTR_TYPE {
	/* Device Name, UUID 0x2a00 */
	NBLE_GAP_SVC_ATTR_NAME = 0,
	/* Appearance, UUID 0x2a01 */
	NBLE_GAP_SVC_ATTR_APPEARANCE,
	/* Peripheral Preferred Connection Parameters (PPCP), UUID 0x2a04 */
	NBLE_GAP_SVC_ATTR_PPCP = 4,
	/* Central Address Resolution (CAR), UUID 0x2aa6, BT 4.2 */
	NBLE_GAP_SVC_ATTR_CAR = 0xa6,
};

struct nble_gap_device_name {
	/* Security mode for writing device name, see BLE_GAP_SEC_MODES */
	uint8_t sec_mode;
	/* 0: no authorization, 1: authorization required */
	uint8_t authorization;
	/* Device name length (0-248) */
	uint8_t len;
	uint8_t name_array[20];
};

struct nble_conn_param {
	/* minimal connection interval: range 0x0006 to 0x0c80 (unit 1.25ms) */
	uint16_t interval_min;
	/* maximum connection interval: range 0x0006 to 0x0c80 must be bigger then min! */
	uint16_t interval_max;
	/* maximum connection slave latency: 0x0000 to 0x01f3 */
	uint16_t slave_latency;
	/* link supervision timeout: 0x000a to 0x0c80 (unit 10ms) */
	uint16_t link_sup_to;
};

struct nble_gap_service_req {
	/* GAP Characteristics attribute type see NBLE_GAP_SVC_ATTR_TYPE */
	uint16_t attr_type;
	union {
		struct nble_gap_device_name name;
		/* Appearance UUID */
		uint16_t appearance;
		/* Preferred Peripheral Connection Parameters */
		struct nble_conn_param ppcp;
		/* Central Address Resolution support 0: no, 1: yes */
		uint8_t car;
	};
};

void nble_gap_service_req(const struct nble_gap_service_req *req);

struct nble_dbg_req {
	uint32_t u0;
	uint32_t u1;
	void *user_data;
};

void nble_dbg_req(const struct nble_dbg_req *req);

struct nble_dbg_rsp {
	int status;
	uint32_t u0;
	uint32_t u1;
	void *user_data;
};

void on_nble_dbg_rsp(const struct nble_dbg_rsp *rsp);

typedef void (*nble_set_bda_cb_t)(int status, void *user_data, const bt_addr_le_t *bda);

struct nble_set_bda_req {
	nble_set_bda_cb_t cb;
	void *user_data;
	bt_addr_le_t bda;
};

void nble_set_bda_req(const struct nble_set_bda_req *req);

struct nble_set_bda_rsp {
	nble_set_bda_cb_t cb;
	void *user_data;
	int status;
	bt_addr_le_t bda;
};

void on_nble_set_bda_rsp(const struct nble_set_bda_rsp *rsp);

typedef void (*nble_get_bda_cb_t)(const bt_addr_le_t *bda, void *user_data);

struct nble_get_bda_req {
	nble_get_bda_cb_t cb;
	void *user_data;
};

void nble_get_bda_req(const struct nble_get_bda_req *req);

struct nble_get_bda_rsp {
	nble_get_bda_cb_t cb;
	void *user_data;
	bt_addr_le_t bda;
};

void on_nble_get_bda_rsp(const struct nble_get_bda_rsp *rsp);

struct nble_eir_data {
	uint8_t len;
	uint8_t data[31];
};

struct nble_gap_set_adv_data_req {
	/* Advertisement data, maybe 0 (length) */
	struct nble_eir_data ad;
	/* Scan response data, maybe 0 (length) */
	struct nble_eir_data sd;
};

void nble_gap_set_adv_data_req(struct nble_gap_set_adv_data_req *req);

struct nble_gap_set_adv_params_req {
	uint16_t timeout;
	/* min interval 0xffff: use default 0x0800 */
	uint16_t interval_min;
	/* max interval 0xffff: use default 0x0800 */
	uint16_t interval_max;
	/* advertisement types see GAP_ADV_TYPES */
	uint8_t type;
	/* filter policy to apply with white list */
	uint8_t filter_policy;
	/* bd address of peer device in case of directed advertisement */
	bt_addr_le_t peer_bda;
};

void nble_gap_set_adv_params_req(struct nble_gap_set_adv_params_req *req);

void nble_gap_start_adv_req(void);

void on_nble_gap_start_adv_rsp(const struct nble_common_rsp *rsp);

void nble_gap_stop_adv_req(void *user_data);

struct nble_gap_conn_update_req {
	uint16_t conn_handle;
	struct nble_conn_param params;
};

void nble_gap_conn_update_req(const struct nble_gap_conn_update_req *req);

void on_nble_gap_conn_update_rsp(const struct nble_common_rsp *rsp);

struct nble_gap_disconnect_req {
	uint16_t conn_handle;
	uint8_t reason;
};

void nble_gap_disconnect_req(const struct nble_gap_disconnect_req *req);

struct nble_sm_config_rsp {
	void *user_data;
	int status;
	bool sm_bond_dev_avail;
};

void on_nble_sm_config_rsp(struct nble_sm_config_rsp *rsp);

struct nble_sm_pairing_param {
	/* authentication level see BLE_GAP_SM_OPTIONS */
	uint8_t auth;
	uint8_t io_capabilities;
	uint8_t max_key_size;
	uint8_t min_key_size;
	uint8_t oob_flag;
};

struct nble_sm_security_req {
	struct bt_conn *conn;
	uint16_t conn_handle;
	/* Local authentication/bonding parameters */
	struct nble_sm_pairing_param params;
};

void nble_sm_security_req(const struct nble_sm_security_req *req);

enum NBLE_SM_PASSKEY_TYPE {
	NBLE_SM_PK_PASSKEY,
	NBLE_SM_PK_OOB,
};

struct nble_sm_passkey {
	uint8_t type; /* see NBLE_SM_PASSKEY_TYPE */
	union {
		uint32_t passkey;
		uint8_t oob[16];
		uint8_t reason;
	};
};

struct nble_sm_passkey_reply_req {
	struct bt_conn *conn;
	uint16_t conn_handle;
	struct nble_sm_passkey params;
};

void nble_sm_passkey_reply_req(const struct nble_sm_passkey_reply_req *req);

struct nble_sm_clear_bonds_req {
	bt_addr_le_t addr;
};

void nble_sm_clear_bonds_req(const struct nble_sm_clear_bonds_req *req);

struct nble_sm_common_rsp {
	int status;
	struct bt_conn *conn;
};

void on_nble_sm_common_rsp(const struct nble_sm_common_rsp *rsp);

struct nble_sm_pairing_response_req {
	struct bt_conn *conn;
	uint16_t conn_handle;
	struct nble_sm_pairing_param params;
};

void nble_sm_pairing_response_req(const struct nble_sm_pairing_response_req *req);

struct nble_sm_error_req {
	struct bt_conn *conn;
	uint16_t conn_handle;
	uint8_t reason;
};

void nble_sm_error_req(const struct nble_sm_error_req *req);

struct nble_gap_set_rssi_report_req {
	uint16_t conn_handle;
	/* RSSI operation see NBLE_GAP_RSSI_OPS */
	uint8_t op;
	/* Channel for RSSI enabling */
	uint8_t channel;
	/* minimum RSSI dBm change to report a new RSSI value */
	uint8_t delta_dBm;
	/* number of delta_dBm changes before sending a new RSSI report */
	uint8_t min_count;
};

void nble_gap_set_rssi_report_req(const struct nble_gap_set_rssi_report_req *req,
				  void *user_data);

void on_nble_gap_set_rssi_report_rsp(const struct nble_common_rsp *rsp);

struct nble_scan_param {
	uint16_t interval;
	uint16_t window;
	/* Unused for the connection request */
	uint8_t scan_type;
	/* Unused for the connection request */
	uint8_t use_whitelist;
};

struct nble_gap_start_scan_req {
	struct nble_scan_param scan_params;
};

void nble_gap_start_scan_req(const struct nble_gap_start_scan_req *req);

void nble_gap_stop_scan_req(void);

void on_nble_gap_scan_start_stop_rsp(const struct nble_common_rsp *rsp);

struct nble_gap_connect_req {
	bt_addr_le_t bda;
	struct nble_conn_param conn_params;
	struct nble_scan_param scan_params;
};

void nble_gap_connect_req(const struct nble_gap_connect_req *req,
			  void *user_data);

void on_nble_gap_connect_rsp(const struct nble_common_rsp *rsp);

void nble_gap_cancel_connect_req(void *user_data);

void on_nble_gap_cancel_connect_rsp(const struct nble_common_rsp *rsp);

/* Temporary patch: RSSI processing for UAS */
struct nble_uas_rssi_calibrate_req {
	float distance;
};
void nble_uas_rssi_calibrate_req(const struct nble_uas_rssi_calibrate_req *req);

/* Temporary patch: RSSI processing for UAS */
struct nble_uas_bucket_change {
	uint8_t distance;
};

void on_nble_uas_bucket_change(const struct nble_uas_bucket_change *par);

struct nble_gap_set_tx_power_req {
	int8_t tx_power;
};

void nble_gap_set_tx_power_req(const struct nble_gap_set_tx_power_req *req);

void on_nble_gap_set_tx_power_rsp(const struct nble_common_rsp *rsp);

struct nble_conn_values {
	/* Connection interval (unit 1.25 ms) */
	uint16_t interval;
	/* Connection latency (unit interval) */
	uint16_t latency;
	/* Connection supervision timeout (unit 10ms)*/
	uint16_t supervision_to;
};

struct nble_gap_connect_evt {
	uint16_t conn_handle;
	struct nble_conn_values conn_values;
	/* 0 if connected as master, otherwise as slave */
	uint8_t role_slave;
	/* Address of peer device */
	bt_addr_le_t peer_bda;
};

void on_nble_gap_connect_evt(const struct nble_gap_connect_evt *evt);

struct nble_gap_disconnect_evt {
	uint16_t conn_handle;
	uint8_t hci_reason;
};

void on_nble_gap_disconnect_evt(const struct nble_gap_disconnect_evt *evt);

struct nble_gap_conn_update_evt {
	uint16_t conn_handle;
	struct nble_conn_values conn_values;
};

void on_nble_gap_conn_update_evt(const struct nble_gap_conn_update_evt *evt);

struct nble_gap_adv_report_evt {
	bt_addr_le_t addr;
	int8_t rssi;
	uint8_t adv_type;
};

void on_nble_gap_adv_report_evt(const struct nble_gap_adv_report_evt *evt,
				const uint8_t *data, uint8_t len);

struct nble_gap_dir_adv_timeout_evt {
	uint16_t conn_handle;
	uint16_t error;
};

void on_nble_gap_dir_adv_timeout_evt(const struct nble_gap_dir_adv_timeout_evt *evt);

#define BLE_GAP_RSSI_EVT_SIZE 32

struct nble_gap_rssi_evt {
	uint16_t conn_handle;
	int8_t rssi_data[BLE_GAP_RSSI_EVT_SIZE];
};

void on_nble_gap_rssi_evt(const struct nble_gap_rssi_evt *evt);

struct nble_sm_passkey_req_evt {
	uint16_t conn_handle;
	uint8_t key_type;
};

void on_nble_sm_passkey_req_evt(const struct nble_sm_passkey_req_evt *evt);

struct nble_sm_passkey_disp_evt {
	uint16_t conn_handle;
	uint32_t passkey;
	uint8_t passkey_confirm;
};

void on_nble_sm_passkey_disp_evt(const struct nble_sm_passkey_disp_evt *evt);

enum NBLE_SM_STATUS_EVT {
	NBLE_SM_STATUS_START_PAIRING,
	NBLE_SM_STATUS_BONDING_COMPLETE,
	NBLE_SM_STATUS_LINK_ENCRYPTED,
	NBLE_SM_STATUS_LINK_SECURITY_CHANGE,
};

struct nble_link_sec {
	bt_security_t sec_level;
	uint8_t enc_size;
};

struct nble_sm_status_evt {
	uint16_t conn_handle;
	uint8_t evt_type; /* see NBLE_SM_STATUS_EVT */
	int status;
	union {
		struct nble_link_sec enc_link_sec;
		bt_addr_le_t addr;
	};
};

void on_nble_sm_status_evt(const struct nble_sm_status_evt *evt);

struct nble_sec_param {
	uint8_t auth;
	uint8_t io_capabilities;
	uint8_t min_key_size;
	uint8_t max_key_size;
};

struct nble_sm_pairing_request_evt {
	uint16_t conn_handle;
	struct nble_sec_param sec_param;
};

void on_nble_sm_pairing_request_evt(const struct nble_sm_pairing_request_evt *evt);

struct nble_sm_security_request_evt {
	uint16_t conn_handle;
	struct nble_sec_param sec_param;
};

void on_nble_sm_security_request_evt(const struct nble_sm_security_request_evt *evt);

struct nble_sm_bond_info;
typedef void (*ble_bond_info_cb_t)(const struct nble_sm_bond_info *info,
				   const bt_addr_le_t *addr, uint16_t len,
				   void *user_data);

struct nble_sm_bond_info_req {
	ble_bond_info_cb_t cb;
	void *user_data;
	bool include_bonded_addrs;
};

void nble_sm_bond_info_req(const struct nble_sm_bond_info_req *req);

struct nble_sm_bond_info {
	int err;
	uint8_t addr_count;
	uint8_t irk_count;
};

struct nble_sm_bond_info_rsp {
	ble_bond_info_cb_t cb;
	void *user_data;
	struct nble_sm_bond_info info;
};

void on_nble_sm_bond_info_rsp(const struct nble_sm_bond_info_rsp *rsp,
			      const bt_addr_le_t *peer_addr, uint16_t len);

struct nble_uart_test_req {
	/* Test type */
	uint16_t test_type;
	/* Number of loops */
	uint16_t nb_loops;
	/* The maximum delay */
	uint16_t max_delay;
	/* The maximum length */
	uint16_t max_len;
};

void nble_uart_test_req(const struct nble_uart_test_req *req,
		const uint8_t *data, uint8_t len);

struct nble_uart_test_evt {
	/* Number of loops executed */
	uint16_t nb_loops;
};

void on_nble_uart_test_evt(const struct nble_uart_test_evt *evt,
		const uint8_t *data, uint8_t len);

struct nble_gap_rpa_update_evt {
	bt_addr_le_t addr;
};

void on_nble_gap_rpa_update_evt(const struct nble_gap_rpa_update_evt *evt);

/*
 * The following functions are NOT RPC functions
 */

void ble_gap_get_version(ble_get_version_cb_t func);

void ble_gap_get_bda_info(struct bt_local_addr *addr);

enum NBLE_GAP_RSSI_OPS {
	NBLE_GAP_RSSI_DISABLE_REPORT = 0,
	NBLE_GAP_RSSI_ENABLE_REPORT
};

typedef void (*rssi_report_t)(const int8_t *rssi_data);

typedef void (*rssi_report_resp_t)(int status);

struct ble_rssi_report_params {
	/* RSSI operation see NBLE_GAP_RSSI_OPS */
	uint8_t op;
	/* Channel for RSSI enabling */
	uint8_t channel;
	/* minimum RSSI dBm change to report a new RSSI value */
	uint8_t delta_dBm;
	/* number of delta_dBm changes before sending a new RSSI report */
	uint8_t min_count;
};

void ble_gap_set_rssi_report(struct ble_rssi_report_params *par,
			     struct bt_conn *conn,
			     rssi_report_resp_t resp_cb, rssi_report_t evt_cb);

/* Hook function to release to BT controller reset */
void nble_curie_unreset_hook(void);

/* Hook function to display log from the BT controller */
void nble_curie_log_hook(char *fmt, ...);

/* Function to set the RPA address */
int le_set_rpa(void);

int bt_le_scan_update(bool fast_scan);

int set_advertise_enable(void);
#endif /* GAP_INTERNAL_H_ */
