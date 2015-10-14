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

#include <string.h>
#include "services/ble/ble_service_gap_api.h"
#include "ble_protocol.h"
#include "ble_service_core_int.h"
#include "infra/log.h"

int ble_gap_set_enable_config(svc_client_handle_t * p_svc_handle,
			      const struct ble_wr_config * p_config,
			      void * p_priv)
{
	struct ble_gap_write_config_req_msg *msg;
	int total_len = sizeof(*msg);
	int str_len = 0;

	if (p_config->p_bda)
		total_len += sizeof(ble_addr_t);
	if (p_config->p_name) {
		str_len = strlen((char *)p_config->p_name);
		if (str_len > BLE_MAX_DEVICE_NAME)
			return -2;
		total_len += str_len;
	}
	msg = (struct ble_gap_write_config_req_msg *)
			cfw_alloc_message_for_service(p_svc_handle,
					  MSG_ID_BLE_GAP_WR_CONF_REQ,
					  total_len,
					  p_priv);
	msg->name_len = str_len;
	msg->appearance = p_config->appearance;
	msg->tx_power = p_config->tx_power;
	msg->central_conn_params = p_config->central_conn_params;
	msg->peripheral_conn_params = p_config->peripheral_conn_params;
	if (p_config->p_bda) {
		msg->bda_len = sizeof(ble_addr_t);
		memcpy(msg->data, p_config->p_bda, msg->bda_len);
	} else
		msg->bda_len = 0;
	if (p_config->p_name)
		strcpy((char *)&msg->data[msg->bda_len], (char *)p_config->p_name);

	return cfw_send_message(msg);
}

int ble_gap_read_bda(svc_client_handle_t * p_svc_handle, void * p_priv)
{
	struct cfw_message *msg = cfw_alloc_message_for_service(p_svc_handle,
								MSG_ID_BLE_GAP_RD_BDA_REQ,
								sizeof(*msg),
								p_priv);
	return cfw_send_message(msg);
}

int ble_gap_wr_adv_data(svc_client_handle_t * p_svc_handle,
			  const struct ble_gap_adv_rsp_data * p_adv_data,
			  const struct ble_gap_adv_rsp_data * p_scan_data,
			  void * p_priv)
{
	int data_len = sizeof(struct ble_gap_wr_adv_data_req_msg);
	struct ble_gap_wr_adv_data_req_msg *msg;

	if (NULL != p_adv_data)
		data_len += p_adv_data->len;
	if (NULL != p_scan_data)
		data_len += p_scan_data->len;

	msg = (struct ble_gap_wr_adv_data_req_msg *)cfw_alloc_message_for_service(
			p_svc_handle, MSG_ID_BLE_GAP_WR_ADV_DATA_REQ,
				data_len, p_priv);

	if (NULL != p_adv_data) {
		int i;
		msg->adv_len = p_adv_data->len;
		for (i = 0; i < msg->adv_len; i++)
			msg->data[i] = p_adv_data->p_data[i];
	} else
		msg->adv_len = 0;

	if (NULL != p_scan_data) {
		int i;
		msg->scan_rsp_len = p_scan_data->len;
		for (i = 0; i < msg->adv_len; i++)
			msg->data[msg->adv_len + i] = p_scan_data->p_data[i];
	} else
		msg->scan_rsp_len = 0;

	return cfw_send_message(msg);
}


int ble_gap_wr_white_list(svc_client_handle_t * p_svc_handle,
			  const struct ble_gap_whitelist_info * p_white_list,
			  void * p_priv)
{
	struct cfw_message *msg = cfw_alloc_message_for_service(p_svc_handle,
								MSG_ID_BLE_GAP_WR_WHITE_LIST_REQ,
								sizeof(*msg),
								p_priv);
	return cfw_send_message(msg);
}

int ble_gap_clr_white_list(svc_client_handle_t * p_svc_handle,
			   uint32_t wl_hdl, void *p_priv)
{
	struct cfw_message *msg = cfw_alloc_message_for_service(p_svc_handle,
								MSG_ID_BLE_GAP_CLR_WHITE_LIST_REQ,
								sizeof(*msg),
								p_priv);
	return cfw_send_message(msg);
}

int ble_gap_start_advertise(svc_client_handle_t * p_svc_handle,
			    const ble_gap_adv_param_t * p_adv_param,
			    void *p_priv)
{
	struct ble_gap_start_advertise_req_msg *msg;
	int data_len = sizeof(struct ble_gap_start_advertise_req_msg);

	if ((NULL != p_adv_param) && (NULL != p_adv_param->p_peer_bda)) {
		data_len += sizeof(ble_addr_t);
	}
	msg = (struct ble_gap_start_advertise_req_msg *)
	    cfw_alloc_message_for_service(p_svc_handle,
					  MSG_ID_BLE_GAP_ENABLE_ADV_REQ,
					  data_len,
					  p_priv);
	if (NULL != p_adv_param) {
		uint8_t i;
		msg->filter_policy = p_adv_param->filter_policy;
		msg->interval_max = p_adv_param->interval_max;
		msg->interval_min = p_adv_param->interval_min;
		msg->options = p_adv_param->options;
		msg->timeout = p_adv_param->timeout;
		msg->type = p_adv_param->type;
		if (NULL != (p_adv_param->p_peer_bda))
		{
			msg->peer_bda[0] = p_adv_param->p_peer_bda->type;
			for (i = 1; i <= BLE_ADDR_LEN; i++)
			{
				msg->peer_bda[i] = p_adv_param->p_peer_bda->addr[i-1];
			}
			msg->bda_len = i;
		} else
			msg->bda_len = 0;
	}
	return cfw_send_message(msg);
}

int ble_gap_stop_advertise(svc_client_handle_t * p_svc_handle, void *p_priv)
{
	struct cfw_message *msg = cfw_alloc_message_for_service(p_svc_handle,
								MSG_ID_BLE_GAP_DISABLE_ADV_REQ,
								sizeof(*msg),
								p_priv);
	return cfw_send_message(msg);
}

int ble_gap_conn_update(svc_client_handle_t * p_svc_handle,
			uint16_t conn_handle,
			const struct ble_gap_connection_params * p_conn_param,
			void *p_priv)
{
	CFW_ALLOC_FOR_SVC(struct ble_gap_conn_update_req_msg, msg, p_svc_handle,
			MSG_ID_BLE_GAP_CONN_UPDATE_REQ, 0, p_priv);

	msg->conn_handle = conn_handle;
	msg->conn_params = *p_conn_param;

	return cfw_send_message(msg);
}

int ble_gap_disconnect(svc_client_handle_t * p_svc_handle,
		       uint16_t conn_hdl, uint8_t reason,
		       void *p_priv)
{
	struct ble_gap_disconnect_req_msg *msg;

	msg = (struct ble_gap_disconnect_req_msg*)cfw_alloc_message_for_service(p_svc_handle,
								MSG_ID_BLE_GAP_DISCONNECT_REQ,
								sizeof(*msg),
								p_priv);

	msg->reason = reason;
	msg->conn_handle = conn_hdl;

	return cfw_send_message(msg);
}

int ble_gap_service_write(svc_client_handle_t * p_svc_handle,
			  const struct ble_gap_service_write_params * p_params,
			  void *p_priv)
{
	struct ble_gap_service_write_req_msg *msg;
	int total_len = sizeof(*msg);
	if ((p_params->attr_type == GAP_SVC_ATTR_NAME) && (p_params->name.len)) {
		total_len += p_params->name.len;
	}
	msg = (struct ble_gap_service_write_req_msg *)
			cfw_alloc_message_for_service(p_svc_handle,
					  MSG_ID_BLE_GAP_SERVICE_WRITE_REQ,
					  total_len,
					  p_priv);
	msg->attr_type = p_params->attr_type;

	switch (p_params->attr_type) {
	case GAP_SVC_ATTR_NAME:
		msg->name.authorization = p_params->name.authorization;
		msg->name.len = p_params->name.len;
		msg->name.sec_mode = p_params->name.sec_mode;
		if (msg->name.len)
			strcpy((char *)&msg->name.name_array[0], (char *)p_params->name.p_name);
		break;
	case GAP_SVC_ATTR_APPEARANCE:
		msg->appearance = p_params->appearance;
		break;
	case GAP_SVC_ATTR_PPCP:
		msg->conn_params = p_params->conn_params;
		break;
	case GAP_SVC_ATTR_CAR:
		msg->car = p_params->car;
		break;
	default:
		pr_warning(LOG_MODULE_BLE, "ble_gap_srv_wr: Attr "
				"not supported : 0x%x", p_params->attr_type);
	}

	return cfw_send_message(msg);
}

int ble_gap_service_read(svc_client_handle_t * p_svc_handle,
			 uint16_t type, void *p_priv)
{
	struct cfw_message *msg = cfw_alloc_message_for_service(p_svc_handle,
								MSG_ID_BLE_GAP_SERVICE_READ_REQ,
								sizeof(*msg),
								p_priv);
	return cfw_send_message(msg);
}

int ble_gap_sm_config(const svc_client_handle_t * h,
		      const struct ble_gap_sm_config_params * p_params,
		      void *p_priv)
{
	CFW_ALLOC_FOR_SVC(struct ble_gap_sm_config_req_msg, msg, h, MSG_ID_BLE_GAP_SM_CONFIG_REQ, 0, p_priv);

	msg->params = *p_params;

	return cfw_send_message(msg);
}

int ble_gap_sm_pairing_req(const svc_client_handle_t * h,
		uint16_t conn_handle,
		const struct ble_gap_sm_pairing_params * p_params,
		void *p_priv)
{
	CFW_ALLOC_FOR_SVC(struct ble_gap_sm_pairing_req_msg, msg, h, MSG_ID_BLE_GAP_SM_PAIRING_REQ, 0, p_priv);

	msg->params = *p_params;
	msg->conn_handle = conn_handle;

	return cfw_send_message(msg);
}

int ble_gap_set_rssi_report(svc_client_handle_t * p_svc_handle,
		const struct rssi_report_params *params,
		void *p_priv)
{
	CFW_ALLOC_FOR_SVC(struct ble_gap_set_rssi_report_req_msg, msg, p_svc_handle, MSG_ID_BLE_GAP_SET_RSSI_REPORT_REQ, 0, p_priv);
	msg->params = *params;
	return cfw_send_message(msg);
}

int ble_gap_start_scan(svc_client_handle_t * p_svc_handle,
		       const ble_gap_scan_param_t * p_scan_params, void * p_priv)
{
	struct cfw_message *msg = cfw_alloc_message_for_service(p_svc_handle,
								MSG_ID_BLE_GAP_SCAN_START_REQ,
								sizeof(*msg),
								p_priv);
	return cfw_send_message(msg);
}

int ble_gap_stop_scan(svc_client_handle_t * p_svc_handle, void *p_priv)
{
	struct cfw_message *msg = cfw_alloc_message_for_service(p_svc_handle,
								MSG_ID_BLE_GAP_SCAN_STOP_REQ,
								sizeof(*msg),
								p_priv);
	return cfw_send_message(msg);
}

int ble_gap_connect(svc_client_handle_t * p_svc_handle, const ble_addr_t * p_bd,
		    const ble_gap_scan_param_t * p_scan_params,
		    const struct ble_gap_connection_params * p_conn_params,
		    void *p_priv)
{
	struct cfw_message *msg = cfw_alloc_message_for_service(p_svc_handle,
								MSG_ID_BLE_GAP_CONNECT_REQ,
								sizeof(*msg),
								p_priv);
	return cfw_send_message(msg);
}

int ble_gap_cancel_connect(svc_client_handle_t * p_svc_handle,
			   const ble_addr_t * p_bd, void *p_priv)
{
	struct cfw_message *msg = cfw_alloc_message_for_service(p_svc_handle,
								MSG_ID_BLE_GAP_CONNECT_CANCEL_REQ,
								sizeof(*msg),
								p_priv);
	return cfw_send_message(msg);
}

int ble_gap_set_option(svc_client_handle_t * p_svc_handle, uint8_t op,
		       const ble_gap_option_t * p_opt, void *p_priv)
{
	struct cfw_message *msg = cfw_alloc_message_for_service(p_svc_handle,
								MSG_ID_BLE_GAP_SET_OPTIONS_REQ,
								sizeof(*msg),
								p_priv);
	return cfw_send_message(msg);
}

int ble_gap_generic_cmd_req(svc_client_handle_t * p_svc_handle,
			    const struct ble_gap_gen_cmd_params *p_params,
			    void *p_priv)
{
	struct ble_generic_cmd_req_msg *msg = (struct ble_generic_cmd_req_msg *)
	    cfw_alloc_message_for_service(p_svc_handle,
					  MSG_ID_BLE_GAP_GENERIC_CMD_REQ,
					  sizeof(*msg),
					  p_priv);

	return cfw_send_message(msg);
}

int ble_gap_get_version_req(svc_client_handle_t * p_svc_handle, void * p_priv)
{
	struct cfw_message *msg = (struct cfw_message *)
	    cfw_alloc_message_for_service(p_svc_handle,
					  MSG_ID_BLE_GAP_GET_VERSION_REQ,
					  sizeof(*msg),
					  p_priv);
	return cfw_send_message(msg);
}

int ble_gap_dtm_init_req(svc_client_handle_t * p_svc_handle, void * p_priv)
{
	struct cfw_message *msg = (struct cfw_message *)
	    cfw_alloc_message_for_service(p_svc_handle,
					  MSG_ID_BLE_GAP_DTM_INIT_REQ,
					  sizeof(*msg),
					  p_priv);
	return cfw_send_message(msg);
}
