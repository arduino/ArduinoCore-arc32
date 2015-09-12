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

#include "services/ble/ble_service_gatt.h"
#include "services/ble/ble_service_gatts_api.h"
#include "ble_service_core_int.h"
#include "ble_service_gatt_int.h"
#include "ble_service_utils.h"
#include <string.h>

int ble_gatts_add_service(svc_client_handle_t * p_svc_handle,
			  const struct bt_uuid * p_uuid,
			  uint8_t type,
			  ble_gatts_add_svc_cback_t cback,
			  void *p_priv)
{
	struct ble_gatts_add_service_req_msg *msg =
	    (struct ble_gatts_add_service_req_msg *)
	    cfw_alloc_message_for_service(p_svc_handle,
					  MSG_ID_BLE_GATTS_ADD_SERVICE_REQ,
					  sizeof(*msg) + ble_sizeof_bt_uuid(p_uuid),
					  p_priv);
	msg->cback = cback;
	msg->type = type;
	uint8_t *p = (uint8_t *)&msg->data;
	ble_encode_bt_uuid(p_uuid, p);
	return cfw_send_message(msg);
}

int ble_gatts_add_included_svc(svc_client_handle_t * p_svc_handle,
			       uint16_t svc_handle,
			       uint16_t svc_handle_to_include,
			       void *p_priv)
{
	struct cfw_message *msg = cfw_alloc_message_for_service(p_svc_handle,
								MSG_ID_BLE_GATTS_ADD_INCL_SVC_REQ,
								sizeof(*msg),
								p_priv);
	return cfw_send_message(msg);
}

int ble_gatts_add_characteristic(svc_client_handle_t * p_svc_handle,
				 uint16_t svc_handle,
				 const struct ble_gatts_characteristic * p_char,
				 ble_gatts_add_char_cback_t cback,
				 void *p_priv)
{
	struct ble_gatts_add_char_req_msg *msg;
	int data_len = sizeof(struct ble_gatts_add_char_req_msg);
	uint8_t * p;

	// Sanity check
	if (!p_char || !p_char->p_uuid)
		return E_OS_ERR;

	// compute the variable length part of the message
	data_len += ble_sizeof_bt_uuid(p_char->p_uuid);
	data_len += p_char->init_len;
	if (p_char->p_user_desc)
		data_len += p_char->p_user_desc->len;
	if (p_char->p_char_pf_desc)
		data_len += sizeof(struct ble_gatt_pf_desc);

	msg = (struct ble_gatts_add_char_req_msg *)
	    cfw_alloc_message_for_service(p_svc_handle,
					  MSG_ID_BLE_GATTS_ADD_CHARACTERISTIC_REQ,
					  data_len, p_priv);
	msg->cback = cback;
	p = msg->data;
	p = ble_encode_bt_uuid(p_char->p_uuid, p);
	msg->svc_handle = svc_handle;
	msg->perms = p_char->perms;
	msg->props = p_char->props;
	msg->max_len = p_char->max_len;
	msg->init_len = p_char->init_len;
	memcpy(p, p_char->p_value, p_char->init_len);
	p += p_char->init_len;
	if (p_char->p_user_desc) {
		msg->ud_len = p_char->p_user_desc->len;
		memcpy(p, p_char->p_user_desc->buffer, p_char->p_user_desc->len);
		p += p_char->p_user_desc->len;
	}
	if (p_char->p_char_pf_desc) {
		msg->pf_len = sizeof(struct ble_gatt_pf_desc);
		memcpy(p, p_char->p_char_pf_desc, msg->pf_len);
		p += msg->pf_len;
	}
	return cfw_send_message(msg);
}

int ble_gatts_add_descriptor(svc_client_handle_t * p_svc_handle,
			     const struct ble_gatts_descriptor * p_desc,
			     ble_gatts_add_desc_cback_t cback,
			     void * p_priv)
{
	struct ble_gatts_add_desc_req_msg *req;
	uint8_t * p;
	int data_len = sizeof(struct ble_gatts_add_desc_req_msg);

	/* Sanity check */
	if (!p_desc || !p_desc->length || !p_desc->p_value || !p_desc->p_uuid)
		return E_OS_ERR;

	// compute the variable length part of the message
	data_len += ble_sizeof_bt_uuid(p_desc->p_uuid);
	data_len += p_desc->length;

	req = (struct ble_gatts_add_desc_req_msg *)
	    cfw_alloc_message_for_service(p_svc_handle,
			    MSG_ID_BLE_GATTS_ADD_DESCRIPTOR_REQ,
			    data_len, p_priv);
	req->cback = cback;
	p = req->data;
	p = ble_encode_bt_uuid(p_desc->p_uuid, p);
	req->length = p_desc->length;
	req->perms = p_desc->perms;
	memcpy(p, p_desc->p_value, p_desc->length);

	return cfw_send_message(req);
}


int ble_gatts_start_service(svc_client_handle_t * p_svc_handle,
			    uint16_t svc_handle,
			    void *p_priv)
{
	struct cfw_message *msg = cfw_alloc_message_for_service(p_svc_handle,
								MSG_ID_BLE_GATTS_START_SERVICE_REQ,
								sizeof(*msg),
								p_priv);
	return cfw_send_message(msg);
}

int ble_gatts_remove_service(svc_client_handle_t * p_svc_handle,
			     uint16_t svc_handle,
			     void *p_priv)
{
	struct cfw_message *msg = cfw_alloc_message_for_service(p_svc_handle,
								MSG_ID_BLE_GATTS_REMOVE_SERVICE_REQ,
								sizeof(*msg),
								p_priv);
	return cfw_send_message(msg);
}

int ble_gatts_send_svc_changed(svc_client_handle_t * p_svc_handle,
			       uint16_t conn_handle,
			       uint16_t start_handle,
			       uint16_t end_handle,
			       void *p_priv)
{
	struct cfw_message *msg = cfw_alloc_message_for_service(p_svc_handle,
								MSG_ID_BLE_GATTS_INDICATE_SERVICE_CHANGE_REQ,
								sizeof(*msg),
								p_priv);
	return cfw_send_message(msg);
}

int ble_gatts_set_attribute_value(svc_client_handle_t * p_svc_handle,
				 uint16_t value_handle,
				 uint16_t len,
				 const uint8_t * p_value,
				 uint16_t offset,
				 void *p_priv)
{
	int i;
	struct ble_gatts_set_attribute_value_msg *msg =
	    (struct ble_gatts_set_attribute_value_msg *)
	    cfw_alloc_message_for_service(p_svc_handle,
					  MSG_ID_BLE_GATTS_SET_ATTRIBUTE_VALUE_REQ,
					  sizeof(*msg)+len,
					  p_priv);
	msg->value_handle = value_handle;
	msg->len = len;
	msg->offset = offset;

	for (i = 0; i < len; i++)
		msg->data[i] = p_value[i];
	return cfw_send_message(msg);
}

int ble_gatts_get_attribute_value(svc_client_handle_t * p_svc_handle,
				 uint16_t value_handle,
				 uint16_t len,
				 uint16_t offset,
				 void *p_priv)
{
	struct cfw_message *msg =  cfw_alloc_message_for_service(p_svc_handle,
			MSG_ID_BLE_GATTS_GET_ATTRIBUTE_VALUE_REQ, sizeof(*msg),
			p_priv);

	return cfw_send_message(msg);
}

int ble_gatts_send_notif(svc_client_handle_t * p_svc_handle,
			     uint16_t conn_handle,
			     const ble_gatts_ind_params_t * p_params,
			     ble_gatts_notif_ind_cback_t cback,
			     void *p_priv)
{
	struct ble_gatts_send_notif_ind_msg *msg =
			(struct ble_gatts_send_notif_ind_msg *)
			cfw_alloc_message_for_service(p_svc_handle,
					MSG_ID_BLE_GATTS_SEND_NOTIF_REQ,
					sizeof(*msg) + p_params->len,
					p_priv);

	msg->cback = cback;
	msg->params.conn_handle = conn_handle;
	msg->params.val_handle = p_params->val_handle;
	msg->params.len = p_params->len;
	msg->params.offset = p_params->offset;
	memcpy(msg->params.data, p_params->p_data, msg->params.len);
	return cfw_send_message(msg);
}

int ble_gatts_send_ind(svc_client_handle_t * p_svc_handle,
			     uint16_t conn_handle,
			     const ble_gatts_ind_params_t * p_params,
			     ble_gatts_notif_ind_cback_t cback,
			     void *p_priv)
{
	struct ble_gatts_send_notif_ind_msg *msg =
			(struct ble_gatts_send_notif_ind_msg *)
			cfw_alloc_message_for_service(p_svc_handle,
					MSG_ID_BLE_GATTS_SEND_IND_REQ,
					sizeof(*msg) + p_params->len,
					p_priv);

	msg->cback = cback;
	msg->params.conn_handle = conn_handle;
	msg->params.val_handle = p_params->val_handle;
	msg->params.len = p_params->len;
	msg->params.offset = p_params->offset;
	memcpy(msg->params.data, p_params->p_data, msg->params.len);
	return cfw_send_message(msg);
}

int ble_gatts_write_conn_attributes(svc_client_handle_t * p_svc_handle,
				    uint16_t conn_handle,
				    const uint8_t * p_data,
				    uint16_t len,
				    void *p_priv)
{
	struct cfw_message *msg = cfw_alloc_message_for_service(p_svc_handle,
								MSG_ID_BLE_GATTS_WR_CONN_ATTRIBUTES_REQ,
								sizeof(*msg),
								p_priv);
	return cfw_send_message(msg);
}

int ble_gatts_read_conn_attributes(svc_client_handle_t * p_svc_handle,
				   uint16_t conn_handle,
				   void *p_priv)
{
	struct cfw_message *msg = cfw_alloc_message_for_service(p_svc_handle,
								MSG_ID_BLE_GATTS_RD_CONN_ATTRIBUTES_REQ,
								sizeof(*msg),
								p_priv);
	return cfw_send_message(msg);
}
