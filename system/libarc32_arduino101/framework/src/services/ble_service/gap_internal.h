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

enum NBLE_GAP_SM_PASSKEY_TYPE {
	NBLE_GAP_SM_REJECT = 0,
	NBLE_GAP_SM_PK_PASSKEY,
	NBLE_GAP_SM_PK_OOB,
};

enum NBLE_GAP_SM_EVT {
	NBLE_GAP_SM_EVT_START_PAIRING,
	NBLE_GAP_SM_EVT_BONDING_COMPLETE,
	NBLE_GAP_SM_EVT_LINK_ENCRYPTED,
	NBLE_GAP_SM_EVT_LINK_SECURITY_CHANGE,
};

enum NBLE_GAP_RSSI_OPS {
	NBLE_GAP_RSSI_DISABLE_REPORT = 0,
	NBLE_GAP_RSSI_ENABLE_REPORT
};

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


struct nble_response {
	int status;
	void *user_data;
};

struct nble_gap_device_name {
	/* Security mode for writing device name, @ref BLE_GAP_SEC_MODES */
	uint8_t sec_mode;
	/* 0: no authorization, 1: authorization required */
	uint8_t authorization;
	/* Device name length (0-248) */
	uint8_t len;
	uint8_t name_array[20];
};

struct nble_gap_connection_values {
	/* Connection interval (unit 1.25 ms) */
	uint16_t interval;
	/* Connection latency (unit interval) */
	uint16_t latency;
	/* Connection supervision timeout (unit 10ms)*/
	uint16_t supervision_to;
};


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

struct nble_gap_connection_params {
	/* minimal connection interval: range 0x0006 to 0x0c80 (unit 1.25ms) */
	uint16_t interval_min;
	/* maximum connection interval: range 0x0006 to 0x0c80 must be bigger then min! */
	uint16_t interval_max;
	/* maximum connection slave latency: 0x0000 to 0x01f3 */
	uint16_t slave_latency;
	/* link supervision timeout: 0x000a to 0x0c80 (unit 10ms) */
	uint16_t link_sup_to;
};

struct nble_gap_scan_parameters {
	/* If 1, perform active scanning (scan requests). */
	uint8_t     active;
	/* If 1, ignore unknown devices (non whitelisted). */
	uint8_t     selective;
	/* Scan interval between 0x0004 and 0x4000 in 0.625ms units (2.5ms to 10.24s). */
	uint16_t    interval;
	/* Scan window between 0x0004 and 0x4000 in 0.625ms units (2.5ms to 10.24s). */
	uint16_t    window;
	/* Scan timeout between 0x0001 and 0xFFFF in seconds, 0x0000 disables timeout. */
	uint16_t    timeout;
};

struct nble_gap_service_write_params {
	/* GAP Characteristics attribute type  @ref BLE_GAP_SVC_ATTR_TYPE */
	uint16_t attr_type;
	union {
		struct nble_gap_device_name name;
		/* Appearance UUID */
		uint16_t appearance;
		/* Preferred Peripheral Connection Parameters */
		struct nble_gap_connection_params conn_params;
		/* Central Address Resolution support 0: no, 1: yes */
		uint8_t car;
	};
};

struct nble_service_read_bda_response {
	int status;
	/* If @ref status ok */
	bt_addr_le_t bd;
	void *user_data;
};

struct nble_service_write_response {
	int status;
	/* GAP Characteristics attribute type  @ref BLE_GAP_SVC_ATTR_TYPE */
	uint16_t attr_type;
	void *user_data;
};

struct nble_gap_service_read_params {
	/* Type of GAP data characteristic to read @ref BLE_GAP_SVC_ATTR_TYPE */
	uint16_t attr_type;
};

struct nble_debug_params {
	uint32_t u0;
	uint32_t u1;
};

struct nble_debug_resp {
	int status;
	uint32_t u0;
	uint32_t u1;
	void *user_data;
};

typedef void (*nble_set_bda_cb_t)(int status, void *user_data);

struct nble_set_bda_params {
	bt_addr_le_t bda;
	nble_set_bda_cb_t cb;
	void *user_data;
};

struct nble_set_bda_rsp {
	nble_set_bda_cb_t cb;
	void *user_data;
	int status;
};

struct bt_eir_data {
	uint8_t len;
	uint8_t data[31];
};

struct nble_gap_adv_params {
	uint16_t timeout;
	/* min interval 0xffff: use default 0x0800 */
	uint16_t interval_min;
	/* max interval 0xffff: use default 0x0800 */
	uint16_t interval_max;
	/* advertisement types @ref GAP_ADV_TYPES */
	uint8_t type;
	/* filter policy to apply with white list */
	uint8_t filter_policy;
	/* bd address of peer device in case of directed advertisement */
	bt_addr_le_t peer_bda;
};

struct nble_gap_ad_data_params {
	/* Advertisement data, maybe 0 (length) */
	struct bt_eir_data ad;
	/* Scan response data, maybe 0 (length) */
	struct bt_eir_data sd;
};

struct nble_log_s {
	uint8_t param0;
	uint8_t param1;
	uint8_t param2;
	uint8_t param3;
};

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

void nble_log(const struct nble_log_s *param, char *buf, uint8_t buflen);

void on_nble_up(void);

void nble_gap_service_write_req(const struct nble_gap_service_write_params *params);

void on_nble_gap_read_bda_rsp(const struct nble_service_read_bda_response *params);

void nble_gap_dbg_req(const struct nble_debug_params *params, void *user_data);

void on_nble_gap_dbg_rsp(const struct nble_debug_resp *params);

void on_nble_set_bda_rsp(const struct nble_set_bda_rsp *params);

void nble_set_bda_req(const struct nble_set_bda_params *params);

void nble_gap_set_adv_data_req(struct nble_gap_ad_data_params *ad_data_params);

void nble_gap_set_adv_params_req(struct nble_gap_adv_params *adv_params);

void nble_gap_start_adv_req(void);

void on_nble_gap_start_advertise_rsp(const struct nble_response *params);

void nble_gap_stop_adv_req(void *user_data);

void nble_gap_read_bda_req(void *priv);

struct nble_gap_irk_info {
	/* Identity Resolving Key (IRK) */
	uint8_t irk[16];
};

void on_nble_common_rsp(const struct nble_response *params);

struct nble_gap_connect_update_params {
	uint16_t conn_handle;
	struct nble_gap_connection_params params;
};

void nble_gap_conn_update_req(const struct nble_gap_connect_update_params *params);

void on_nble_gap_conn_update_rsp(const struct nble_response *params);

struct nble_gap_connect_req_params {
	bt_addr_le_t bda;
	struct nble_gap_connection_params conn_params;
	struct nble_gap_scan_parameters scan_params;
};

struct nble_gap_disconnect_req_params {
	uint16_t conn_handle;
	uint8_t reason;
};

void nble_gap_disconnect_req(const struct nble_gap_disconnect_req_params *params);

struct nble_gap_sm_config_params {
	/* Security options (@ref BLE_GAP_SM_OPTIONS) */
	uint8_t options;
	/* I/O Capabilities to allow passkey exchange (@ref BLE_GAP_IO_CAPABILITIES) */
	uint8_t io_caps;
	/* Maximum encryption key size (7-16) */
	uint8_t key_size;
	uint8_t oob_present;
};

void nble_gap_sm_config_req(const struct nble_gap_sm_config_params *params);

struct nble_gap_sm_config_rsp {
	void *user_data;
	int status;
	bool sm_bond_dev_avail;
};

void on_nble_gap_sm_config_rsp(struct nble_gap_sm_config_rsp *params);


struct nble_gap_sm_pairing_params {
	/* authentication level see @ref BLE_GAP_SM_OPTIONS */
	uint8_t auth_level;
};

struct nble_gap_sm_security_params {
	struct bt_conn *conn;
	uint16_t conn_handle;
	/* Local authentication/bonding parameters */
	struct nble_gap_sm_pairing_params params;
};

void nble_gap_sm_security_req(const struct nble_gap_sm_security_params *
			    params);

struct nble_gap_sm_passkey {
	uint8_t type;
	union {
		uint32_t passkey;
		uint8_t oob[16];
		uint8_t reason;
	};
};

struct nble_gap_sm_key_reply_req_params {
	struct bt_conn *conn;
	uint16_t conn_handle;
	struct nble_gap_sm_passkey params;
};

void nble_gap_sm_passkey_reply_req(const struct nble_gap_sm_key_reply_req_params
				   *params);

struct nble_gap_sm_clear_bond_req_params {
	bt_addr_le_t addr;
};

void nble_gap_sm_clear_bonds_req(const struct nble_gap_sm_clear_bond_req_params
				 *params);

struct nble_gap_sm_response {
	int status;
	struct bt_conn *conn;
};

void on_nble_gap_sm_common_rsp(const struct nble_gap_sm_response *rsp);

/**
 * Callback for rssi event.
 */
typedef void (*rssi_report_t)(const int8_t *rssi_data);

/**
 * Callback for rssi report response.
 */
typedef void (*rssi_report_resp_t)(int status);

struct nble_rssi_report_params {
	uint16_t conn_handle;
	/* RSSI operation @ref NBLE_GAP_RSSI_OPS */
	uint8_t op;
	/* Channel for RSSI enabling */
	uint8_t channel;
	/* minimum RSSI dBm change to report a new RSSI value */
	uint8_t delta_dBm;
	/* number of delta_dBm changes before sending a new RSSI report */
	uint8_t min_count;
};

void ble_gap_set_rssi_report(struct nble_rssi_report_params *params,
			      struct bt_conn *conn,
			      rssi_report_resp_t resp_cb, rssi_report_t evt_cb);

void nble_gap_set_rssi_report_req(const struct nble_rssi_report_params *params,
			    void *user_data);

void on_nble_gap_set_rssi_report_rsp(const struct nble_response *params);

struct nble_gap_scan_params {
	uint16_t interval;
	uint16_t window;
	uint8_t scan_type;
	uint8_t use_whitelist;
};

void nble_gap_start_scan_req(const struct nble_gap_scan_params *params);

void nble_gap_stop_scan_req(void);

void on_nble_gap_scan_start_stop_rsp(const struct nble_response *rsp);

void nble_gap_connect_req(const struct nble_gap_connect_req_params *params,
		void *user_data);

void on_nble_gap_connect_rsp(const struct nble_response *params);

void nble_gap_cancel_connect_req(void *priv);

void on_nble_gap_cancel_connect_rsp(const struct nble_response *params);

enum BLE_GAP_SET_OPTIONS {
	BLE_GAP_SET_CH_MAP = 0,
};

struct nble_gap_channel_map {
	/* connection on which to change channel map */
	uint16_t conn_handle;
	/* 37 bits are used of the 40 bits (LSB) */
	uint8_t map[5];
};


struct nble_gap_set_option_params {
	/* Option to set @ref BLE_GAP_SET_OPTIONS */
	uint8_t op;
	union {
		struct nble_gap_channel_map ch_map;
	};
};

/*
 *  Generic request op codes.
 * This allows to access some non connection related commands like DTM.
 */
enum BLE_GAP_GEN_OPS {
	/* Not used now. */
	DUMMY_VALUE = 0,
};

struct nble_gap_gen_cmd_params {
	/* @ref BLE_GAP_GEN_OPS */
	uint8_t op_code;
};

/* Temporary patch: RSSI processing for UAS */
struct nble_uas_rssi_calibrate {
	float distance;
};
void nble_uas_rssi_calibrate_req(const struct nble_uas_rssi_calibrate *p_struct);

/* Temporary patch: RSSI processing for UAS */
struct nble_uas_bucket_change {
	uint8_t distance;
};
void on_nble_uas_bucket_change(const struct nble_uas_bucket_change *p_params);

struct nble_version {
	uint8_t version;
	uint8_t major;
	uint8_t minor;
	uint8_t patch;
	char version_string[20];
	uint8_t hash[4];
};

typedef void (*ble_get_version_cb_t)(const struct nble_version *ver);

struct nble_gap_get_version_param {
	ble_get_version_cb_t cb;
};

struct nble_version_response {
	struct nble_gap_get_version_param params;
	struct nble_version ver;
};

void nble_get_version_req(const struct nble_gap_get_version_param *params);

void on_nble_get_version_rsp(const struct nble_version_response *params);

void nble_gap_dtm_init_req(void *user_data);

void on_nble_gap_dtm_init_rsp(void *user_data);

struct nble_gap_tx_power_params {
	int8_t tx_power;
};

void nble_gap_tx_power_req(const struct nble_gap_tx_power_params *params);

void on_nble_gap_tx_power_rsp(const struct nble_response *params);

struct nble_gap_connect_evt {
	uint16_t conn_handle;
	struct nble_gap_connection_values conn_values;
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
	struct nble_gap_connection_values conn_values;
};

void on_nble_gap_conn_update_evt(const struct nble_gap_conn_update_evt *evt);

struct nble_gap_adv_report_evt {
	bt_addr_le_t addr;
	int8_t rssi;
	uint8_t adv_type;
};

void on_nble_gap_adv_report_evt(const struct nble_gap_adv_report_evt *evt,
				const uint8_t *buf, uint8_t len);

struct nble_gap_dir_adv_timeout_evt {
	uint16_t conn_handle;
	uint16_t error;
};

void on_nble_gap_dir_adv_timeout_evt(const struct nble_gap_dir_adv_timeout_evt *p_evt);

#define BLE_GAP_RSSI_EVT_SIZE	32
struct nble_gap_rssi_evt {
	uint16_t conn_handle;
	int8_t rssi_data[BLE_GAP_RSSI_EVT_SIZE];
};

void on_nble_gap_rssi_evt(const struct nble_gap_rssi_evt *evt);

struct nble_gap_timout_evt {
	uint16_t conn_handle;
	/* reason for timeout @ref BLE_SVC_GAP_TIMEOUT_REASON */
	int reason;
};

struct nble_gap_sm_passkey_req_evt {
	uint16_t conn_handle;
	uint8_t key_type;
};

void on_nble_gap_sm_passkey_req_evt(const struct nble_gap_sm_passkey_req_evt *evt);

struct nble_gap_sm_passkey_disp_evt {
	uint16_t conn_handle;
	uint32_t passkey;
};

void on_nble_gap_sm_passkey_display_evt(const struct nble_gap_sm_passkey_disp_evt *evt);

struct nble_link_sec {
	bt_security_t sec_level;
	uint8_t enc_size;
};

struct nble_gap_sm_status_evt {
	uint16_t conn_handle;
	uint8_t evt_type;
	int status;
	struct nble_link_sec enc_link_sec;
};

void on_nble_gap_sm_status_evt(const struct nble_gap_sm_status_evt *evt);

void on_nble_set_bda_rsp(const struct nble_set_bda_rsp *params);

enum BLE_INFO_REQ_TYPES {
	BLE_INFO_BONDING = 1, /* Get bonding database related information */
	BLE_INFO_LAST /* Keep last */
};

struct ble_gap_bonded_dev_info
{
	uint8_t       addr_count;      /* Count of le_addr in array. */
	uint8_t       irk_count;       /* IRK count */
	bt_addr_le_t  le_addr[];       /* Bonded device address */
};

struct ble_get_info_rsp {
	int status; 		/* Response status */
	uint8_t info_type; 	/* Requested information type */
	struct ble_gap_bonded_dev_info info_params;
};

struct nble_gap_sm_bond_info;
typedef void (*ble_bond_info_cb_t)(const struct nble_gap_sm_bond_info *info,
			      const bt_addr_le_t *addr, uint16_t len,
			      void *user_data);

struct nble_gap_sm_bond_info_param {
	ble_bond_info_cb_t cb;
	void *user_data;
	bool include_bonded_addrs;
};

void nble_gap_sm_bond_info_req(const struct nble_gap_sm_bond_info_param *params);

struct nble_gap_sm_bond_info {
	int err;
	uint8_t addr_count;
	uint8_t irk_count;
};

struct nble_gap_sm_bond_info_rsp {
	ble_bond_info_cb_t cb;
	void *user_data;
	struct nble_gap_sm_bond_info info;
};

void on_nble_gap_sm_bond_info_rsp(const struct nble_gap_sm_bond_info_rsp *rsp,
		const bt_addr_le_t *peer_addr, uint16_t len);

void ble_gap_get_bonding_info(ble_bond_info_cb_t func, void *user_data,
			      bool include_bonded_addrs);

void ble_gap_get_version(ble_get_version_cb_t func);

#endif /* GAP_INTERNAL_H_ */
