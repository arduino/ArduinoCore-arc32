/*
 * Copyright (c) 2015, Intel Corporation. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef __BLE_SERVICE_GAP_H__
#define __BLE_SERVICE_GAP_H__

#include "cfw/cfw.h"
#include "cfw/cfw_client.h"
#include "infra/version.h"
#include "ble_service_msg.h"
#include "ble_service.h"

/**
 * @defgroup ble_core_service BLE Core Service GAP/GATT APIs
 *
 * BLE Core service GAP/GATT APIs used by BLE service.
 *
 * @ingroup ble_service
 * @{
 */

/** Macro to convert milliseconds to a specific unit */
#define MSEC_TO_0_625_MS_UNITS(TIME) (((TIME) * 1000) / 625)
#define MSEC_TO_1_25_MS_UNITS(TIME) (((TIME) * 1000) / 1250)
#define MSEC_TO_10_MS_UNITS(TIME) ((TIME) / 10)

/**
 * BLE GAP Status return codes.
 */
enum BLE_SVC_GAP_STATUS_CODES {
	BLE_SVC_GAP_STATUS_SUCCESS = BLE_STATUS_SUCCESS, /**< GAP success */
	BLE_SVC_GAP_STATUS_ERROR = BLE_STATUS_GATT_BASE, /**< Generic GAP error */
	BLE_SVC_GAP_STATUS_INVALID_UUID_LIST, /**< Invalid UUID list provided (e.g. advertisement) */
	/* TODO: add more status codes */
};

/**
 * BLE GAP addr types.
 *
 * BLE GAP supported address types
 */
enum BLE_ADDR_TYPES {
	BLE_ADDR_PUBLIC = 0, /**< BD address assigned by IEEE */
	BLE_ADDR_PRIVATE_RANDOM_STATIC, /**< Random address */
	BLE_ADDR_RRIVATE_RANDOM_PRIVATE_RESOLVABLE, /**< Resolvable Private Random address */
	BLE_ADDR_PRIVATE_RANDOM_PRIVATE_NONRESOLVABLE /**< Non-resolvable Private Random address */
};

/**
 * BT/BLE address Length.
 */
#define BLE_ADDR_LEN 6

#define BLE_GAP_SEC_RAND_LEN    8	/**< Random Security number length (64 bits) */
#define BLE_GAP_SEC_MAX_KEY_LEN 16	/**< Maximum security key len (LTK, CSRK) */

#define BLE_SVC_GAP_HANDLE_INVALID 0xffff /**< Invalid GAP connection handle */

/**
 * Device GAP name characteristic write permission.
 *
 * If the characteristic shall be writable, use a combination of the values
 * defined in @ref BLE_GAP_SEC_MODES
 */
#define BLE_DEVICE_NAME_WRITE_PERM GAP_SEC_NO_PERMISSION

typedef struct {
	uint8_t type;		/**< BLE Address type @ref BLE_ADDR_TYPES */
	uint8_t addr[BLE_ADDR_LEN];
				/**< BD address, little endian format */
} ble_addr_t;

/**
 * GAP device roles.
 */
enum BLE_ROLES {
	BLE_ROLE_INVALID = 0,
	BLE_ROLE_PERIPHERAL = 0x01,
	BLE_ROLE_CENTRAL = 0x02
};

typedef uint8_t ble_role_t;

/**
 * BLE core (GAP, GATT) Message IDs used for request, response, events and indications.
 */
enum BLE_GAP_MSG_ID {
	MSG_ID_BLE_GAP_WR_CONF_REQ = MSG_ID_BLE_GAP_BASE,
	MSG_ID_BLE_GAP_RD_BDA_REQ,
	MSG_ID_BLE_GAP_WR_ADV_DATA_REQ,
	MSG_ID_BLE_GAP_WR_WHITE_LIST_REQ,
	MSG_ID_BLE_GAP_CLR_WHITE_LIST_REQ,
	MSG_ID_BLE_GAP_ENABLE_ADV_REQ,
	MSG_ID_BLE_GAP_DISABLE_ADV_REQ,
	MSG_ID_BLE_GAP_CONN_UPDATE_REQ,
	MSG_ID_BLE_GAP_DISCONNECT_REQ,
	MSG_ID_BLE_GAP_SERVICE_WRITE_REQ,
	MSG_ID_BLE_GAP_SERVICE_READ_REQ,
	MSG_ID_BLE_GAP_SM_CONFIG_REQ,
	MSG_ID_BLE_GAP_SM_PAIRING_REQ,
	MSG_ID_BLE_GAP_SM_PASSKEY_REQ,
	MSG_ID_BLE_GAP_SET_RSSI_REPORT_REQ,
	MSG_ID_BLE_GAP_SCAN_START_REQ,
	MSG_ID_BLE_GAP_SCAN_STOP_REQ,
	MSG_ID_BLE_GAP_CONNECT_REQ,
	MSG_ID_BLE_GAP_CONNECT_CANCEL_REQ,
	MSG_ID_BLE_GAP_SET_OPTIONS_REQ,
	MSG_ID_BLE_GAP_GENERIC_CMD_REQ,
	MSG_ID_BLE_GAP_GET_VERSION_REQ,
	MSG_ID_BLE_GAP_DTM_INIT_REQ,
	MSG_ID_BLE_CTRL_LOG_REQ,
	MSG_ID_BLE_GAP_REQ_LAST,

	/** BLE GAP Response Messages IDs. */
	MSG_ID_BLE_GAP_WR_CONF_RSP = MSG_ID_BLE_GAP_RSP,
						    /**< Write controller config: own Bluetooth Device Address, tx power */
	MSG_ID_BLE_GAP_RD_BDA_RSP,		    /**< Read own Bluetooth Device Address */
	MSG_ID_BLE_GAP_WR_ADV_DATA_RSP,		    /**< Write Advertising Data and Scan response data */
	MSG_ID_BLE_GAP_WR_WHITE_LIST_RSP,	    /**< Write white list to controller */
	MSG_ID_BLE_GAP_CLR_WHITE_LIST_RSP,	    /**< Clear current white list */
	MSG_ID_BLE_GAP_ENABLE_ADV_RSP,		    /**< Enable Advertising */
	MSG_ID_BLE_GAP_DISABLE_ADV_RSP,		    /**< Disable Advertising */
	MSG_ID_BLE_GAP_CONN_UPDATE_RSP,		    /**< Update Connection */
	MSG_ID_BLE_GAP_DISCONNECT_RSP,		    /**< Disconnect */
	MSG_ID_BLE_GAP_SERVICE_WRITE_RSP,	    /**< Write GAP Service specific like device name, appearance and PPCPparameters */
	MSG_ID_BLE_GAP_SERVICE_READ_RSP,	    /**< Read GAP Service specific like device name, appearance and PPCPparameters */
	MSG_ID_BLE_GAP_SM_CONFIG_RSP, /**< Response to @ref ble_gap_sm_config */
	MSG_ID_BLE_GAP_SM_PAIRING_RSP, /**< Response to @ref ble_gap_sm_pairing_req */
	MSG_ID_BLE_GAP_SM_PASSKEY_RSP, /**< Response to @ref ble_gap_sm_passkey_reply */
	MSG_ID_BLE_GAP_SET_RSSI_REPORT_RSP,	    /**< Enable/Disable reporting of changes in RSSI */
	MSG_ID_BLE_GAP_SCAN_START_RSP,		    /**< Start Scanning */
	MSG_ID_BLE_GAP_SCAN_STOP_RSP,		    /**< Stop Scanning */
	MSG_ID_BLE_GAP_CONNECT_RSP,		    /**< Start Connection procedure */
	MSG_ID_BLE_GAP_CONNECT_CANCEL_RSP,	    /**< Cancel ongoing connection procedure */
	MSG_ID_BLE_GAP_SET_OPTIONS_RSP,		    /**< Set gap options (e.g. co-ex, master/central role) */
	MSG_ID_BLE_GAP_GENERIC_CMD_RSP,		    /**< Generic non connection related requests */
	MSG_ID_BLE_GAP_GET_VERSION_RSP,
	MSG_ID_BLE_GAP_DTM_INIT_RSP,
	MSG_ID_BLE_CTRL_LOG_RSP, /**< BLE controller logging message */
	MSG_ID_BLE_GAP_RSP_LAST,

	/** GAP related events. */
	MSG_ID_BLE_GAP_CONNECT_EVT = MSG_ID_BLE_GAP_EVT, /**< Connection established */
	MSG_ID_BLE_GAP_DISCONNECT_EVT,		    /**< Disconnect from peer */
	MSG_ID_BLE_GAP_CONN_UPDATE_EVT,	    /**< Connection Parameters update event (in central, they have been updated, in peripheral, also includes the status of the request) */
	MSG_ID_BLE_GAP_SM_PAIRING_STATUS_EVT, /**< Pairing request status event */
	MSG_ID_BLE_GAP_SM_PASSKEY_REQ_EVT, /**< Pairing passkey request (6 digits or 16 byte OOB data) */
	MSG_ID_BLE_GAP_TO_EVT,			    /**< GAP Timeout event */
	MSG_ID_BLE_GAP_ADV_DATA_EVT,		    /**< Advertising raw data event (central role) */
	MSG_ID_BLE_GAP_RSSI_EVT,		    /**< Signal strength change event */
	MSG_ID_BLE_GAP_GENERIC_CMD_EVT,		    /**< Generic command request event */
	MSG_ID_BLE_CTRL_LOG_EVT,		    /**< BLE Controller Logging Events */
	MSG_ID_BLE_GAP_EVT_LAST,
};

/**
 * Generic BLE Status Response.
 * Short status response for commands not returning any additional data
 */
struct ble_rsp {
	struct cfw_message header; /**< Component framework message header (@ref cfw), MUST be first element of structure */
	ble_status_t status; /**< Response status @ref BLE_STATUS */
};

/**
 * Connection requested parameters.
 */
struct ble_gap_connection_params {
	uint16_t interval_min;	    /**< minimal connection interval: range 0x0006 to 0x0c80 (unit 1.25ms) */
	uint16_t interval_max;	    /**< maximum connection interval: range 0x0006 to 0x0c80 must be bigger then min! */
	uint16_t slave_latency;	    /**< maximum connection slave latency: 0x0000 to 0x01f3 */
	uint16_t link_sup_to;	    /**< link supervision timeout: 0x000a to 0x0c80 (unit 10ms) */
};

/**
 * Connection values.
 */
struct ble_gap_connection_values {
	uint16_t interval; /**< Connection interval (unit 1.25 ms) */
	uint16_t latency; /**< Connection latency (unit interval) */
	uint16_t supervision_to; /**< Connection supervision timeout (unit 10ms)*/
};

/**
 * Initial GAP configuration
 */
struct ble_wr_config {
	ble_addr_t *p_bda;
	uint8_t *p_name; /**< GAP Device name, NULL terminated! */
	uint16_t appearance; /**< see BLE spec */
	int8_t tx_power;
	struct ble_gap_connection_params peripheral_conn_params; /**< Peripheral preferred */
	struct ble_gap_connection_params central_conn_params; /**< Central supported range */
};

/** Read BD address response. */
typedef struct {
	struct cfw_message header; /**< Component framework message header (@ref cfw), MUST be first element of structure */
	ble_status_t status; /**< Response status @ref BLE_STATUS */
	ble_addr_t bd; /**< if status ok @ref ble_addr_t */
} ble_bda_rd_rsp_t;

struct ble_gap_adv_rsp_data {
	uint8_t *p_data;		    /**< max 31 bytes! */
	uint8_t len;
};

/**
 * Advertising types, see BT spec vol 6, Part B, chapter 2.3.
 */
enum GAP_ADV_TYPES {
	ADV_IND = 0x00,		    /**< Connectable undirected advertising */
	ADV_DIRECT_IND = 0x01,	    /**< Connectable high duty cycle advertising */
	ADV_NONCONN_IND = 0x02,	    /**< Non connectable undirected advertising */
	ADV_SCAN_IND = 0x06,	    /**< Scannable undirected advertising */
	ADV_SCAN_RSP = 0x81,	    /**< Scan response, only a return value in @ref ble_gap_adv_data_evt_t */
	ADV_RESERVED		/* keep last */
};

typedef struct {
	uint8_t irk[BLE_GAP_SEC_MAX_KEY_LEN];
					    /**< Identity Resolving Key (IRK) */
} ble_gap_irk_info_t;

struct ble_gap_whitelist_info {
	ble_addr_t **pp_bd;		/**< list of bd addresses */
	ble_gap_irk_info_t **pp_key;	/**< list of irk keys (for address resolution offload) */
	uint8_t bd_count;		/**< number of bd addresses */
	uint8_t key_count;		/**< number of keys */
};

/**
 * Advertisement options.
 */
enum BLE_GAP_ADV_OPTIONS {
	BLE_GAP_OPT_ADV_DEFAULT = 0,	    /**< no specific option */
	BLE_GAP_OPT_ADV_WHITE_LISTED = 0x02 /**< use white list and only report whitelisted devices */
};

/**
 * Advertisement parameters.
 */
typedef struct {
	uint16_t timeout;
	uint16_t interval_min;	    /**< min interval 0xffff: use default 0x0800 */
	uint16_t interval_max;	    /**< max interval 0xffff: use default 0x0800 */
	uint8_t type;		    /**< advertisement types @ref GAP_ADV_TYPES */
	uint8_t filter_policy;	    /**< filter policy to apply with white list */
	ble_addr_t *p_peer_bda;	    /**< bd address of peer device in case of directed advertisement */
	uint8_t options;	    /**< options see @ref BLE_GAP_ADV_OPTIONS (to be ORed) */
} ble_gap_adv_param_t;

/**
 * Generic BLE Status. Response
 * Short status response for commands not returning any additional data
 */
typedef struct {
	struct cfw_message header; /**< Component framework message header (@ref cfw), MUST be first element of structure */
	ble_status_t status; /**< Response status @ref BLE_STATUS */
	uint32_t wl_handle; /**< reference handle. to be used for clearing it later */
} ble_gap_wr_white_list_rsp_t;


/**
 * Appearance read response message.
 */
typedef struct {
	struct cfw_message header; /**< Component framework message header (@ref cfw), MUST be first element of structure */
	ble_status_t status; /**< Response status @ref BLE_STATUS */
	uint16_t uuid; /**< value of GAP appearance characteristic */
} ble_rd_appearance_rsp_t;

/**
 * LE security modes.
 *
 * see BT spec  PART C, 10.2
 *
 * - Security mode 1
 *   - Level 1: No security at all (service may use data signing)
 *   - Level 2: Unauthenticated (no MITM protection pairing with encryption
 *   - Level 3: Authenticated (MITM protection) pairing with encryption
 *   - Level 4: Authenticated (MITM protection) LE Secure Connection wi
 *
 * - Security mode 2 (data signing)
 *   - Level 1: Unauthenticated pairing with data signing
 *   - Level 2: Authenticated (MITM protection) with data signing
 */
enum BLE_GAP_SEC_MODES {
	GAP_SEC_NO_PERMISSION = 0, /**< No access permitted. */
	GAP_SEC_LEVEL_1,
	GAP_SEC_LEVEL_2,
	GAP_SEC_LEVEL_3,
	GAP_SEC_LEVEL_4,
	GAP_SEC_MODE_1 = 0x10,
	GAP_SEC_MODE_2 = 0x20	/**< only used for data signing, level 1 or 2 */
};

struct ble_gap_svc_local_name {
	uint8_t sec_mode;	/**< security mode for writing device name, @ref BLE_GAP_SEC_MODES */
	uint8_t authorization;	/**< 0: no authorization, 1: authorization required */
	uint8_t len;		/**< device name length (0-248) */
	const uint8_t *p_name;	/**< name to to write */
};

enum BLE_GAP_SVC_ATTR_TYPE {
	GAP_SVC_ATTR_NAME = 0,		    /**< Device Name, UUID 0x2a00 */
	GAP_SVC_ATTR_APPEARANCE,	    /**< Appearance, UUID 0x2a01 */
	GAP_SVC_ATTR_PPCP = 4,		    /**< Peripheral Preferred Connection Parameters (PPCP), UUID 0x2a04 */
	GAP_SVC_ATTR_CAR = 0xa6,	    /**< Central Address Resolution (CAR), UUID 0x2aa6, BT 4.2 */
};

struct ble_gap_service_write_params {
	uint16_t attr_type;		      /**< GAP Characteristics attribute type  @ref BLE_GAP_SVC_ATTR_TYPE */
	union {
		struct ble_gap_svc_local_name name;
		uint16_t appearance;	      /**< Appearance UUID */
		struct ble_gap_connection_params conn_params;
						 /**< Preferred Peripheral Connection Parameters */
		uint8_t car;		      /**< Central Address Resolution support 0: no, 1: yes */
	};
};

struct ble_gap_service_read_rsp {
	struct cfw_message header; /**< Component framework message header (@ref cfw), MUST be first element of structure */
	ble_status_t status; /**< status of read operation @ref BLE_STATUS, in case failure union shall be empty */
	uint16_t attr_type; /**< type of attribute returned (valid even in failure case! */
	union {
		struct ble_gap_svc_local_name name;
		uint16_t appearance; /**< Appearance UUID */
		struct ble_gap_connection_params conn_params; /**< Preferred Peripheral Connection Parameters */
		uint8_t car; /** Central Address Resolution support 0: no, 1: yes */
	};
};

/**
 * GAP security manager options for bonding/authentication procedures, see Vol 3: Part H, 3.5.
 */
enum BLE_GAP_SM_OPTIONS {
	BLE_GAP_BONDING = 0x01,		/**< SMP supports bonding */
	BLE_GAP_MITM = 0x04,		/**< SMP requires Man In The Middle protection */
	BLE_GAP_OOB = 0x08		/**< SMP supports Out Of Band data */
};

/**
 * IO capabilities, see Vol 3: PART H, 3.5.
 */
enum BLE_GAP_IO_CAPABILITIES {
	BLE_GAP_IO_DISPLAY_ONLY = 0,
	BLE_GAP_IO_DISPLAY_YESNO = 1,
	BLE_GAP_IO_KEYBOARD_ONLY = 2,
	BLE_GAP_IO_NO_INPUT_NO_OUTPUT = 3,
	BLE_GAP_IO_KEYBOARD_DISPLAY = 4
};

/**
 * Security manager configuration parameters.
 *
 * options and io_caps will define there will be a passkey request or not.
 * It is assumed that io_caps and options are compatible.
 */
struct ble_gap_sm_config_params {
	uint8_t options; /**< Security options (@ref BLE_GAP_SM_OPTIONS) */
	uint8_t io_caps; /**< I/O Capabilities to allow passkey exchange (@ref BLE_GAP_IO_CAPABILITIES) */
	uint8_t key_size; /**< Maximum encryption key size (7-16) */
};

/**
 * Security manager pairing parameters.
 */
struct ble_gap_sm_pairing_params {
	uint8_t auth_level; /**< authentication level see @ref BLE_GAP_SM_OPTIONS */
};

/**
 * Security manager passkey type.
 */
enum BLE_GAP_SM_PASSKEY_TYPE {
	BLE_GAP_SM_PASSKEY = 0, /**< Security data is a passkey. */
	BLE_GAP_SM_OBB, /**< Security data is 16 bytes of OOB data */
};
/**
 * Security reply to incoming security request.
 */
struct ble_gap_sm_passkey {
	uint8_t type; /**< Security data type in this reply @ref BLE_GAP_SM_PASSKEY_TYPE */
	union {
		uint8_t passkey[6]; /**< 6 digits (string) */
		uint8_t oob[16]; /**< 16 bytes of OOB security data */
	};
};

/**
 * RSSI operation definition.
 */
enum BLE_GAP_RSSI_OPS {
	BLE_GAP_RSSI_DISABLE_REPORT = 0,
	BLE_GAP_RSSI_ENABLE_REPORT
};

enum BLE_GAP_SCAN_OPTIONS {
	BLE_GAP_SCAN_DEFAULT = 0,	    /**< no specific option */
	BLE_GAP_SCAN_ACTIVE = 0x01,	    /**< do an active scan (request scan response */
	BLE_GAP_SCAN_WHITE_LISTED = 0x02    /**< Use white list and only report whitelisted devices */
};

enum BLE_GAP_SET_OPTIONS {
	BLE_GAP_SET_CH_MAP = 0,		    /**< Set channel map */
};

typedef struct {
	uint16_t conn_handle;		    /**< connection on which to change channel map */
	uint8_t map[5];			    /**< 37 bits are used of the 40 bits (LSB) */
} ble_gap_channel_map_t;

/**
 * GAP option data structure.
 */
typedef union {
	ble_gap_channel_map_t ch_map;	    /**< BLE channel map to set see BT spec */
} ble_gap_option_t;

/**
 * Scan parameters.
 *
 * @note Check BT core spec for high low duty cycle interval & window size!
 */
typedef struct {
	uint16_t timeout;		/**< scan timeout in s, 0 never */
	uint16_t interval;		/**< interval: 0x4 - 0x4000. (unit: 0.625ms), use default: 0xffff (0x0010) */
	uint16_t window;		/**< Window: 0x4 - 0x4000. (unit: 0.625ms), use default 0xffff (= 0x0010) */
	uint8_t options;		/**< scan options, ORed options from @ref BLE_GAP_SCAN_OPTIONS */
} ble_gap_scan_param_t;

/**
 * Connect event @ref MSG_ID_BLE_GAP_CONNECT_EVT.
 */
struct ble_gap_connect_evt {
	struct ble_gap_connection_values conn_values; /**< Connection values */
	uint8_t role; /**< role in this connection @ref */
	ble_addr_t peer_bda; /**< address of peer device */
};

/**
 * Disconnect event @ref MSG_ID_BLE_GAP_DISCONNECT_EVT.
 */
struct ble_gap_disconnected_evt {
	uint8_t hci_reason; /**< HCI disconnect reason */
};

/**
 * Updated connection event @ref MSG_ID_BLE_GAP_CONN_UPDATE_EVT.
 */
struct ble_gap_conn_update_evt {
	struct ble_gap_connection_values conn_values;
};

/**
 * Security manager pairing status event @ref MSG_ID_BLE_GAP_SM_PAIRING_STATUS_EVT.
 */
struct ble_gap_sm_pairing_status_evt {
	uint16_t conn_handle;
	uint16_t status;
};

/**
 * Security manager passkey request event @ref MSG_ID_BLE_GAP_SM_PASSKEY_REQ_EVT.
 */
struct ble_gap_sm_passkey_req_evt {
	uint8_t dummy;
};

/**
 * GAP/SMP security result status code.
 * see Vol 3: Part H, chapter 3.5.5.
 */
enum BLE_GAP_SEC_RESULT_STATUS {
	BLE_GAP_SEC_STATUS_SUCCESS = 0,
				    /**< bonding/pairing completed successfully */
	BLE_GAP_SEC_STATUS_PASSKEY_ENTRY_FAILED,/**< passkey entry failed */
	BLE_GAP_SEC_STATUS_OOB_NOT_AVAILABLE,	/**< Out of Band data is not available */
	BLE_GAP_SEC_STATUS_AUTH_REQUIREMENTS,	/**< Authentication requirements not met due to IO cap */
	BLE_GAP_SEC_STATUS_CONFIRM_VALUE,	/**< Confirm value does not match calculated value */
	BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPPORTED,
						/**< Pairing not supported by the device  */
	BLE_GAP_SEC_STATUS_ENC_KEY_SIZE,	/**< Encryption key size insufficient */
	BLE_GAP_SEC_STATUS_SMP_CMD_UNSUPPORTED,	/**< Unsupported SMP command on this device */
	BLE_GAP_SEC_STATUS_UNSPECIFIED,		/**< Failure due to unspecified reason */
	BLE_GAP_SEC_STATUS_REPEATED_ATTEMPTS,	/**< Pairing/authent disallowed due to too little time elapsed since last attempt */
	BLE_GAP_SEC_STATUS_INVALID_PARAMS,	/**< Invalid parameters due to length or parameters */
	/* 4.2 spec only ? */
	BLE_GAP_SEC_STATUS_DHKEY_CHECK_FAILED,	/**< Remote device indicates that DHKey does not match local calculated key */
	BLE_GAP_SEC_STATUS_NUMERIC_COMP_FAILED,	/**< values in numeric key comparison protocol do not match */
	BLE_GAP_SEC_STATUS_BREDR_PAIRING_INPROGRESS,/**< Failure due to BR/EDR pairing request */
	BLE_GAP_SEC_STATUS_CROSS_TSPRT_KEY_GEN_DIS,
						/**< BR/EDR link key generation can not be use for LE keys handling */
};

enum BLE_SVC_GAP_TIMEOUT_REASON {
	BLE_SVC_GAP_TO_ADV, /**< Advertisement Stopped. */
	BLE_SVC_GAP_TO_SEC_REQ, /**< Security Request took too long. */
	BLE_SVC_GAP_TO_SCAN, /**< Scanning stopped. */
	BLE_SVC_GAP_TO_CONN, /**< Connection Link timeout. */
};

/**
 * GAP timeout event (e.g. protocol error) MSG_ID_BLE_GAP_TO_EVT.
 */
struct ble_gap_timout_evt {
	int reason;	    /**< reason for timeout @ref BLE_SVC_GAP_TIMEOUT_REASON */
};

/**
 * Advertisement data structure (central role) @ref MSG_ID_BLE_GAP_ADV_DATA_EVT.
 */
struct ble_gap_adv_data_evt {
	ble_addr_t remote_bda;	    /**< address of remote device */
	int8_t rssi;		    /**< signal strength compared to 0 dBm */
	uint8_t type;		    /**< type of advertisement data or scan response @ref GAP_ADV_TYPES */
	uint8_t len;		    /**< length of advertisement data or scap response data */
	uint8_t data[];		    /**< Advertisement or scan response data */
};

/**
 * Connection Parameter update request event @ref MSG_ID_BLE_GAP_CONN_PARAM_UPDATE_REQ_EVT.
 *
 * @note reply with @ref ble_gap_conn_update_params
 */
struct ble_gap_conn_param_update_req_evt {
	struct ble_gap_connection_params param;
};

/**
 * RSSI signal strength event @ref MSG_ID_BLE_GAP_RSSI_EVT.
 */
struct ble_gap_rssi_evt {
	int8_t rssi_lvl;	/**< RSSI level (compared to 0 dBm) */
};

/**
 * RSSI report parameters @ref MSG_ID_BLE_GAP_SET_RSSI_REPORT_REQ.
 */
struct rssi_report_params {
	uint16_t conn_hdl;	/**< Connection handle */
	uint8_t op;	/**< RSSI operation @ref BLE_GAP_RSSI_OPS */
	uint8_t delta_dBm;	/**< minimum RSSI dBm change to report a new RSSI value */
	uint8_t min_count;	/**< number of delta_dBm changes before sending a new RSSI report */
};

/** Test Mode opcodes. */
enum TEST_OPCODE {
	BLE_TEST_INIT_DTM = 0x01,	/**< Put BLE controller in HCI UART DTM test mode */
	BLE_TEST_START_DTM_RX = 0x1d,	/**< LE receiver test HCI opcode */
	BLE_TEST_START_DTM_TX = 0x1e,	/**< LE transmitter test HCI opcode */
	BLE_TEST_END_DTM = 0x1f,	/**< End LE DTM TEST */
	/* vendor specific commands start at 0x80 */
	BLE_TEST_SET_TXPOWER = 0x80,	/**< Set Tx power. To be called before start of tx test */
	BLE_TEST_START_TX_CARRIER,	/**< Start Tx Carrier Test */
};

/**
 * Rx direct test mode data structure.
 */
struct ble_dtm_rx_test {
	uint8_t freq;		    /**< rf channel 0x00 - 0x27, resulting F = 2402 MHz + [freq * 2 MHz] */
};

/**
 * Tx direct test mode data structure
 */
struct ble_dtm_tx_test {
	uint8_t freq;		    /**< rf channel 0x00 - 0x27 where resulting F = 2402 + [freq * 2 MHz] */
	uint8_t len;		    /**< length of test data payload for each packet */
	uint8_t pattern;	    /**< packet payload pattern type, 0x00 - 0x02 mandatory */
};

/**
 * Tx power settings data structure.
 */
struct ble_set_txpower {
	int8_t dbm;		    /**< Tx power level to set (e.g. -30: -30 dBm). Depends on BLE Controller */
};

/**
 * RX test result data.
 */
struct ble_dtm_test_result {
	uint16_t mode;
	uint16_t nb;
};

/**
 * Direct Test mode command params
 */
struct ble_test_cmd {
	uint8_t mode;		/**< test mode to execute @ref TEST_OPCODE */
	union {
		struct ble_dtm_rx_test rx;
		struct ble_dtm_tx_test tx;
		struct ble_set_txpower tx_pwr;	/**< Tx power to use for Tx tests. */
	};
};

/**
 * BLE GAP event structure.
 */
struct  ble_gap_event {
	struct cfw_message header; /**< Component framework message header (@ref cfw), MUST be first element of structure */
	uint16_t conn_handle; /**< connection handle */
	union {
		struct ble_gap_connect_evt connected;       /**< connected event parameters */
		struct ble_gap_disconnected_evt disconnected;    /**< disconnected reason */
		struct ble_gap_conn_update_evt conn_updated;     /**< connection updated */
		struct ble_gap_sm_pairing_status_evt sm_pairing_status; /**< Security Manager pairing status */
		struct ble_gap_sm_passkey_req_evt sm_passkey_req;  /**< Security Manager passkey request */
							    /**< connection related security update */
		struct ble_gap_timout_evt timeout;		    /**< gap timeout occurred */
		struct ble_gap_adv_data_evt adv_data;	    /**< advertisement data */
		struct ble_gap_conn_param_update_req_evt conn_param_req;
							    /**< update request from remote for connection parameters */
		struct ble_gap_rssi_evt rssi;		    /**< new rssi level if rssi reporting is enabled */
	};
};

/** Generic request op codes.
 * This allows to access some non connection related commands like DTM.
 */
enum BLE_GAP_GEN_OPS {
	DUMMY_VALUE = 0,		    /**< Not used now. */
};

/**
 * Generic command parameters.
 *
 * @note Independent of connection!
 */
struct ble_gap_gen_cmd_params {
	uint8_t op_code; /**< @ref BLE_GAP_GEN_OPS */
};

struct ble_version_msg {
	struct cfw_message header; /**< Component framework message header (@ref cfw), MUST be first element of structure */
	ble_status_t status;
	struct version_header version; /**< Nordic version header */
};

struct ble_dtm_init_msg {
	struct cfw_message header; /**< Component framework message header (@ref cfw), MUST be first element of structure */
	ble_status_t status;
};

struct ble_dtm_result_msg {
	struct cfw_message header; /**< Component framework message header (@ref cfw), MUST be first element of structure */
	ble_status_t status;
	struct ble_dtm_test_result result; /**< Result data of DTM RX test */
};

/**
 * Generic request message response or event.
 */
struct ble_generic_msg {
	struct cfw_message header; /**< Component framework message header (@ref cfw), MUST be first element of structure */
	ble_status_t status;
	uint8_t op_code; /**< Opcode to which this message is applicable @ref BLE_GAP_GEN_OPS */
};

/**
 * Set Enable configuration parameters (BD address, etc).
 *
 * This shall put the controller stack into a usable (enabled) state.
 * Hence this should be called first!
 *
 * @param p_svc_handle service handle
 * @param p_config     BLE write configuration
 * @param p_priv       pointer to private data
 *
 * @return @ref OS_ERR_TYPE
 * @return MSG_ID_BLE_GAP_WR_CONF_RSP @ref ble_rsp, TODO: return maybe more info?
 */
int ble_gap_set_enable_config(svc_client_handle_t * p_svc_handle,
			 const struct ble_wr_config * p_config, void *p_priv);

/**
 * Read BD address from Controller.
 *
 *
 * @param p_svc_handle service handle
 * @param p_priv       pointer to private data
 *
 * @return @ref OS_ERR_TYPE
 * @return MSG: @ref MSG_ID_BLE_GAP_RD_BDA_RSP @ref ble_bda_rd_rsp_t
 */
int ble_gap_read_bda(svc_client_handle_t * p_svc_handle, void *p_priv);

/**
 * Write Advertisement data to BLE controller.
 *
 * Store advertisement data in BLE controller. It needs to be done BEFORE starting advertisement
 *
 * @param p_svc_handle service handle
 * @param p_adv_data   adv data to store in BLE controller
 * @param p_scan_data  scan response data to store in controller, can be NULL
 * @param p_priv       pointer to private data
 *
 * @return @ref OS_ERR_TYPE,
 * @return MSG: MSG_ID_BLE_GAP_WR_ADV_DATA_RSP @ref ble_rsp
 */
int ble_gap_wr_adv_data(svc_client_handle_t * p_svc_handle,
			  const struct ble_gap_adv_rsp_data * p_adv_data,
			  const struct ble_gap_adv_rsp_data * p_scan_data,
			  void *p_priv);

/**
 * Write white list to the BLE controller.
 *
 * Store white in BLE controller. It needs to be done BEFORE starting advertisement or
 * start scanning
 *
 * @param p_svc_handle service handle
 * @param p_white_list white list to store in the controller
 * @param p_priv       pointer to private data
 *
 * @return @ref OS_ERR_TYPE,
 * @return MSG: MSG_ID_BLE_GAP_WR_WHITE_LIST @ref ble_gap_wr_white_list_rsp_t
 */
int ble_gap_wr_white_list(svc_client_handle_t * p_svc_handle,
			  const struct ble_gap_whitelist_info * p_white_list,
			  void *p_priv);

/**
 * Clear previously stored white list.
 *
 * @param p_svc_handle service handle
 * @param wl_handle handle to the white list previously stored
 * @param p_priv       pointer to private data
 *
 * @return @ref OS_ERR_TYPE
 * @return MSG: MSG_ID_BLE_GAP_CLR_WHITE_LIST @ref ble_rsp
 */
int ble_gap_clr_white_list(svc_client_handle_t * p_svc_handle,
			   uint32_t wl_handle, void *p_priv);

/**
 * Start advertising.
 *
 * @param p_svc_handle service handle
 * @param p_adv_param advertisement
 * @param p_priv       pointer to private data
 *
 * @return @ref OS_ERR_TYPE
 * @return MSG: MSG_ID_BLE_GAP_ENABLE_ADV @ref ble_rsp
 */
int ble_gap_start_advertise(svc_client_handle_t * p_svc_handle,
			    const ble_gap_adv_param_t * p_adv_param,
			    void *p_priv);

/**
 * Stop advertising.
 *
 * @param p_svc_handle service handle
 * @param p_priv       pointer to private data
 *
 * @return @ref OS_ERR_TYPE
 * @return MSG: MSG_ID_BLE_GAP_DISABLE_ADV @ref ble_rsp
 */
int ble_gap_stop_advertise(svc_client_handle_t * p_svc_handle, void *p_priv);

/**
 * Update connection.
 *
 * This function's behavior depends on the role of the connection:
 * - in peripheral mode, it sends an L2CAP signaling connection parameter
 *  update request based the values in @ref p_conn_param
 *  and the action can be taken by the central at link layer
 *  - in central mode, it will send a link layer command to change the
 *  connection values based on the values in @ref p_conn_param where the
 *  connection interval is interval_min
 *
 * When the connection is updated, the event @ref MSG_ID_BLE_GAP_CONN_UPDATE_EVT will
 * be received.
 *
 * @param conn_handle Connection handle
 * @param p_conn_param Connection parameters
 * @param p_priv pointer to private data
 *
 * @return @ref OS_ERR_TYPE
 * @return MSG: @ref MSG_ID_BLE_GAP_CONN_UPDATE_RSP @ref ble_rsp
 */
int ble_gap_conn_update(svc_client_handle_t * p_svc_handle,
			uint16_t conn_handle,
			const struct ble_gap_connection_params * p_conn_param,
			void *p_priv);

/**
 * Disconnect connection (peripheral or central role).
 *
 * @param p_svc_handle service handle
 * @param conn_hhdl    connection to terminate
 * @param reason       HCI reason for connection termination, most often 0x16 (connection terminated by local host)
 * @param p_priv       pointer to private data
 *
 * @return @ref OS_ERR_TYPE
 * @return MSG: MSG_ID_BLE_GAP_DISCONNECT @ref ble_rsp, MSG_ID_BLE_GAP_DISCONNECT_EVT @ref ble_gap_disconnected_evt_t
 */
int ble_gap_disconnect(svc_client_handle_t * p_svc_handle,
		       uint16_t conn_hhdl, uint8_t reason,
		       void *p_priv);
/**
 * Write GAP Service Attribute Characteristics.
 *
 * @param p_svc_handle service handle
 * @param p_params data of characteristic to write
 * @param p_priv       pointer to private data
 *
 * @return @ref OS_ERR_TYPE
 * @return MSG: MSG_ID_BLE_GAP_SERVICE_WRITE_RSP @ref ble_rsp
 */
int ble_gap_service_write(svc_client_handle_t * p_svc_handle,
			  const struct ble_gap_service_write_params * p_params,
			  void *p_priv);

/**
 * Read GAP Service Characteristics.
 *
 * @param p_svc_handle service handle
 * @param type type of GAP service data characteristic to read @ref BLE_GAP_SVC_ATTR_TYPE
 * @param p_priv       pointer to private data
 *
 * @return @ref OS_ERR_TYPE,
 * @return MSG: MSG_ID_BLE_GAP_SERVICE_READ_RSP @ref ble_gap_service_read_rsp
 */
int ble_gap_service_read(svc_client_handle_t * p_svc_handle,
			 uint16_t type, void * p_priv);

/**
 * Function for configuring the security manager.
 *
 * @param h Service client
 * @param p_params local authentication/bonding parameters
 * @param p_priv       pointer to private data
 *
 * @return @ref OS_ERR_TYPE
 *
 * @note Upon completion of the procedure, the client will receive
 * a message @ref MSG_ID_BLE_GAP_SM_CONFIG_RSP
 */
int ble_gap_sm_config(const svc_client_handle_t * h,
		    const struct ble_gap_sm_config_params * p_params,
		    void *p_priv);

/**
 * Initiate the bonding procedure (central).
 *
 * @param h            Service client
 * @param conn_handle  connection on which bonding procedure is executed
 * @param p_params     local authentication/bonding parameters
 * @param p_priv       pointer to private data
 *
 * @return @ref OS_ERR_TYPE
 *
 * @note Upon completion of the procedure, the client receives
 * @ref MSG_ID_BLE_GAP_SM_PAIRING_RSP
 */
int ble_gap_sm_pairing_req(const svc_client_handle_t * h,
		uint16_t conn_handle,
		const struct ble_gap_sm_pairing_params * p_params,
		void *p_priv);

/**
 * Reply to an incoming passkey request event (@ref MSG_ID_BLE_GAP_SM_PASSKEY_REQ_EVT).
 *
 * @param p_svc_handle service handle
 * @param conn_handle  connection on which bonding is going on
 * @param p_params     bonding security reply
 * @param p_priv       pointer to private data
 *
 * @return @ref OS_ERR_TYPE
 *
 * @note Upon completion of the procedure, the client receives
 * @ref MSG_ID_BLE_GAP_SM_PASSKEY_RSP
 */
int ble_gap_sm_passkey_reply(svc_client_handle_t * p_svc_handle,
			     uint16_t conn_handle,
			     const struct ble_gap_sm_passkey * p_params,
			     void *p_priv);

/**
 * Enable disable the reporting of the RSSI value.
 *
 * @param p_svc_handle service handle
 * @param conf RSSI report parameters @ref MSG_ID_BLE_GAP_SET_RSSI_REPORT_REQ
 * @param p_priv  pointer to private data
 *
 * @return @ref OS_ERR_TYPE,
 * @return MSG: MSG_ID_BLE_GAP_SET_RSSI_REPORT_RSP @ref ble_rsp
 */
int ble_gap_set_rssi_report(svc_client_handle_t * p_svc_handle,
			    const struct rssi_report_params *params,
			    void *p_priv);

/**
 * Start scanning for BLE devices doing advertisement.
 *
 * @param p_svc_handle service handle
 * @param p_scan_params scan parameters to use @ref ble_gap_scan_param_t
 * @param p_priv       pointer to private data
 *
 * @return @ref OS_ERR_TYPE,
 * @return MSG: MSG_ID_BLE_GAP_SCAN_START_RSP @ref ble_rsp
 */
int ble_gap_start_scan(svc_client_handle_t * p_svc_handle,
		       const ble_gap_scan_param_t * p_scan_params,
		       void *p_priv);

/**
 * Stop scanning.
 *
 * @param p_svc_handle service handle
 * @param p_priv       pointer to private data
 *
 * @return @ref OS_ERR_TYPE,
 * @return MSG: MSG_ID_BLE_GAP_STOP_START_RSP @ref ble_rsp
 */
int ble_gap_stop_scan(svc_client_handle_t * p_svc_handle, void *p_priv);

/**
 * Connect to a Remote Device.
 *
 * @param p_svc_handle service handle
 * @param p_bd bd to connect to. shall be null if BLE_GAP_SCAN_WHITE_LISTED option is set in @ref ble_gap_scan_param_t
 * @param p_scan_params scan parameters
 * @param p_conn_params connection parameters
 * @param p_priv        pointer to private data
 *
 * @return @ref OS_ERR_TYPE,
 * @return MSG: MSG_ID_BLE_GAP_CONNECT_RSP @ref ble_rsp,
 * @return MSG: MSG_ID_BLE_GAP_CONNECT_EVT @ref ble_gap_connect_evt_t
 */
int ble_gap_connect(svc_client_handle_t * p_svc_handle, const ble_addr_t * p_bd,
		    const ble_gap_scan_param_t * p_scan_params,
		    const struct ble_gap_connection_params * p_conn_params,
		    void *p_priv);

/**
 * Cancel an ongoing connection attempt.
 *
 * @param p_svc_handle service handle
 * @param p_bd bd      address of device for which the connection shall be canceled
 * @param p_priv       pointer to private data
 *
 * @return @ref OS_ERR_TYPE,
 * @return MSG: MSG_ID_BLE_GAP_CONNECT @ref ble_rsp
 */
int ble_gap_cancel_connect(svc_client_handle_t * p_svc_handle,
			   const ble_addr_t * p_bd, void *p_priv);

/**
 * Set a gap option (channel map etc) on a connection.
 *
 * @param p_svc_handle service handle
 * @param op option to set @ref BLE_GAP_SET_OPTIONS
 * @param p_opt bd address of device for which the connection shall be canceled ble_gap_option_t
 * @param p_priv       pointer to private data
 *
 * @return @ref OS_ERR_TYPE,
 * @return MSG: MSG_ID_BLE_GAP_SET_OPTIONS @ref ble_rsp
 */
int ble_gap_set_option(svc_client_handle_t * p_svc_handle, uint8_t op,
		       const ble_gap_option_t * p_opt, void *p_priv);

/**
 * Set a gap option (channel map etc) on a connection.
 *
 * @param p_svc_handle service handle
 * @param p_params bd address of device for which the connection shall be canceled ble_gap_option_t
 * @param p_priv       pointer to private data
 *
 * @return @ref OS_ERR_TYPE,
 * @return MSG: MSG_ID_BLE_GAP_GENERIC_CMD_RSP @ref ble_rsp or @ref ble_generic_msg
 */
int ble_gap_generic_cmd_req(svc_client_handle_t * p_svc_handle,
			    const struct ble_gap_gen_cmd_params *p_params,
			    void *p_priv);

/**
 * Get nordic version.
 *
 * @param p_svc_handle service handle
 * @param p_priv       pointer to private data
 *
 * @return @ref OS_ERR_TYPE,
 * @return MSG: MSG_ID_BLE_GAP_GET_VERSION_RSP @ref ble_rsp or @ref ble_generic_msg
 */
int ble_gap_get_version_req(svc_client_handle_t * p_svc_handle,
			    void *p_priv);

/**
 * Init dtm mode.
 *
 * @param p_svc_handle service handle
 * @param p_params bd address of device for which the connection shall be canceled ble_gap_option_t
 *
 * @return @ref OS_ERR_TYPE,
 * @return MSG: MSG_ID_BLE_GAP_DTM_INIT_RSP @ref ble_rsp or @ref ble_generic_msg
 */
int ble_gap_dtm_init_req(svc_client_handle_t * p_svc_handle,
			    void *p_priv);
/** @} */

#endif /* __BLE_SVC_API_H__ */
