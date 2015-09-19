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

#ifndef __BLE_SERVICE_CORE_INT_H__
#define __BLE_SERVICE_CORE_INT_H__

#include <stdint.h>
#include "services/ble/ble_service_gap_api.h"
#include "services/ble/ble_service_gattc_api.h"

/* Forward declarations */
struct _ble_service_cb;
struct ble_enable_req_msg;

/**
 * BLE common structures.
 */

struct ble_svc_string {
	uint8_t *p_string;		/**< String utf8 encoded */
	uint8_t len;			/**< length of string */
};

struct ble_svc_sec_mode {
	uint8_t rd_perm;
	uint8_t wr_perm;
};

struct ble_svc_cccd_sec_mode {
	uint8_t cccd_wr_perm;
	uint8_t rd_perm;		/**< Read permissions. */
	uint8_t wr_perm;		/**< Write permissions. */
};

struct ble_svc_report_reference {
	uint8_t report_id;	/**< Non-zero value if these is more than one instance of the same Report Type */
	uint8_t report_type;	/**< Type of Report characteristic */
};

struct ble_gap_write_config_req_msg {
	struct cfw_message header; /**< Component framework message header (@ref cfw), MUST be first element of structure */
	uint16_t appearance;
	uint8_t bda_len;
	uint8_t name_len;
	int8_t tx_power;
	struct ble_gap_connection_params peripheral_conn_params;
	struct ble_gap_connection_params central_conn_params;
	uint8_t data[];
};

struct ble_gap_wr_adv_data_req_msg {
	struct cfw_message header; /**< Component framework message header (@ref cfw), MUST be first element of structure */
	uint8_t adv_len;
	uint8_t scan_rsp_len;
	uint8_t data[];
	/* adv_data,
	 * scan_rsp_dat */
};

struct ble_gap_start_advertise_req_msg {
	struct cfw_message header; /**< Component framework message header (@ref cfw), MUST be first element of structure */
	uint16_t timeout;
	uint16_t interval_min;	    /**< min interval 0xffff: use default 0x0800 */
	uint16_t interval_max;	    /**< max interval 0xffff: use default 0x0800 */
	uint8_t type;		    /**< advertisement types @ref GAP_ADV_TYPES */
	uint8_t filter_policy;	    /**< filter policy to apply with white list */
	uint8_t options;	    /**< options see @ref BLE_GAP_ADV_OPTIONS (to be ORed) */
	uint8_t bda_len;	/**< non 0 if peer_bda is present */
	uint8_t peer_bda[];	/**< format ble_addr_t */
};

struct ble_gap_conn_update_req_msg {
	struct cfw_message header; /**< Component framework message header (@ref cfw), MUST be first element of structure */
	uint16_t conn_handle;
	struct ble_gap_connection_params conn_params;
};

struct ble_gap_svc_local_name_req {
	uint8_t sec_mode;	    /**< security mode for writing device name, @ref BLE_GAP_SEC_MODES (GAP_SEC_NO_PERMISSION: write forbidden) */
	uint8_t authorization;		    /**< 0: no authorization, 1: authorization required */
	uint8_t len;			    /**< device name length (0-248) */
	uint8_t name_array[];		    /**< name to to write */
};

struct ble_gap_service_write_req_msg {
	struct cfw_message header; /**< Component framework message header (@ref cfw), MUST be first element of structure */
	uint16_t attr_type;		      /**< GAP Characteristics attribute type  @ref BLE_GAP_SVC_ATTR_TYPE */
	union {
		struct ble_gap_svc_local_name_req name;
		uint16_t appearance;	      /**< Appearance UUID */
		struct ble_gap_connection_params conn_params;
						 /**< Preferred Peripheral Connection Parameters */
		uint8_t car;		      /**< Central Address Resolution support 0: no, 1: yes */
	};
};

struct ble_gap_disconnect_req_msg {
	struct cfw_message header; /**< Component framework message header (@ref cfw), MUST be first element of structure */
	uint16_t conn_handle; /**< Connection handle*/
	uint8_t reason; /**< Reason of the disconnect*/
};

struct ble_gap_sm_config_req_msg {
	struct cfw_message header; /**< Component framework message header (@ref cfw), MUST be first element of structure */
	struct ble_gap_sm_config_params params;
};

struct ble_gap_sm_pairing_req_msg {
	struct cfw_message header; /**< Component framework message header (@ref cfw), MUST be first element of structure */
	struct ble_gap_sm_pairing_params params;
	uint16_t conn_handle;
};

struct ble_dtm_test_req_msg {
	struct cfw_message header; /**< Component framework message header (@ref cfw), MUST be first element of structure */
	struct ble_test_cmd params;
};

struct ble_gap_set_rssi_report_req_msg {
	struct cfw_message header; /**< Component framework message header (@ref cfw), MUST be first element of structure */
	struct rssi_report_params params; /**< RSSI report config @ref rssi_report_cfg */
};

struct ble_gattc_discover_primary_service_req_msg {
	struct cfw_message header;
	uint16_t conn_handle; /**< Connection handle */
	uint8_t data[]; /**< Variable length data of the request (UUID) */
};

struct ble_gattc_discover_included_service_req_msg {
	struct cfw_message header;
	uint16_t conn_handle;
	struct ble_gattc_handle_range handle_range;
};

struct ble_gattc_discover_characteristic_req_msg {
	struct cfw_message header;
	uint16_t conn_handle;
	struct ble_gattc_handle_range handle_range;
};

struct ble_gattc_discover_descriptor_req_msg {
	struct cfw_message header;
	uint16_t conn_handle;
	struct ble_gattc_handle_range handle_range;
};

struct ble_gattc_read_characteristic_req_msg {
	struct cfw_message header; /**< Component framework message header (@ref cfw), MUST be first element of structure */
	uint16_t conn_handle; /**< Connection handle*/
	uint16_t char_handle; /**< handle of the attribute to be read */
	uint16_t offset; /**< offset into the attribute value to be read */
};

struct _ble_gattc_wr_characteristic {
	uint16_t char_handle; /**< handle of characteristic */
	uint16_t len; /**< if len is bigger then ATT MTU size, the controller fragment buffer itself */
	uint8_t wr_type; /**< type of write operation @ref BLE_GATT_WR_OP_TYPES */
	uint8_t value[]; /**< characteristic value to write */
};

struct ble_gattc_write_char_op_req_msg {
	struct cfw_message header;
	uint16_t conn_handle;
	struct _ble_gattc_wr_characteristic wr_char_param;
};

/**
 * Generic BLE controller commands.
 */
struct ble_generic_cmd_req_msg {
	struct cfw_message header; /**< Component framework message header (@ref cfw), MUST be first element of structure */
};


void ble_core_resume_enable(struct ble_enable_req_msg *req, struct _ble_service_cb * p_cb, uint8_t * p_name);

void ble_core_delete_conn_params_timer(void);

#endif
