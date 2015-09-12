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

#ifndef __BLE_SERVICE_GATT_INT_H__
#define __BLE_SERVICE_GATT_INT_H__

#include "services/ble/ble_service_gatt.h"
#include "services/ble/ble_service_gatts_api.h"

/**
 * BLE GATT related shared internal structures between master and BLE controller.
 */
struct ble_gatts_add_service_req_msg {
	struct cfw_message header; /**< Component framework message header (@ref cfw), MUST be first element of structure */
	ble_gatts_add_svc_cback_t cback; /**< Callback function to be returnedn in response message */
	uint8_t type;	/**< Primary or secondary \ref BLE_GATT_SVC_TYPES. */
	uint8_t data[];	/**< Variable length data of the request (UUID) */
};

/**
 * Component framework message to add a characteristic to a service
 */
struct ble_gatts_add_char_req_msg {
	struct cfw_message header; /**< Component framework message header (@ref cfw), MUST be first element of structure */
	ble_gatts_add_char_cback_t cback; /**< Callback function, to be returned to sender! */
	uint16_t svc_handle; /**< Handle of the service to which the characteristic should be added */
	struct ble_gatts_permissions perms;
	struct ble_gatt_char_properties props;
	uint16_t max_len; /**< Maximum length of the characteristic value */
	uint16_t init_len; /**< Length of the initialization value */
	uint8_t ud_len; /**< Length of the characteristic User Description */
	uint8_t pf_len;

	uint8_t data[]; /**< Variable length data of the request (UUID, init value, user descriptor) */
};

/**
 * Component framework message to add a descriptor to a characteristic
 */
struct ble_gatts_add_desc_req_msg {
	struct cfw_message header; /**< Component framework message header (@ref cfw), MUST be first element of structure */
	ble_gatts_add_desc_cback_t cback; /**< Callback function, to be returned to sender! */
	uint16_t length;
	struct ble_gatts_permissions perms; /**< Read/Write permissions for descriptor */
	uint8_t data[]; /**< Variable length data of the request (descriptor value) */
};

struct ble_gatts_start_service_msg {
	struct cfw_message header; /**< Component framework message header (@ref cfw), MUST be first element of structure */
	uint16_t svc_handle;
};

struct ble_gatts_set_attribute_value_msg {
	struct cfw_message header; /**< Component framework message header (@ref cfw), MUST be first element of structure */
	uint16_t value_handle;	/* mandatory */
	uint16_t len;
	uint16_t offset;	/* by default 0 */
	uint8_t data[];		/* value to update */
};

struct ble_gatts_notif_ind_params {
	uint16_t conn_handle;
	uint16_t val_handle;
	uint16_t len;		/* may be 0, in this case already stored data is sent */
	uint16_t offset;
	uint8_t data[];
};

struct ble_gatts_send_notif_ind_msg {
	struct cfw_message header; /**< Component framework message header (@ref cfw), MUST be first element of structure */
	ble_gatts_notif_ind_cback_t cback; /**< Callback function to be returned in response message */
	struct ble_gatts_notif_ind_params params; /* parameters for notification/indication */
};

#endif
