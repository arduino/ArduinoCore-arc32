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

#ifndef __BLE_SERVICE_GATTC_H__
#define __BLE_SERVICE_GATTC_H__

#include "ble_service.h"
#include "ble_service_gap_api.h"
#include "ble_service_gatt.h"

/** 
 * @defgroup ble_core_service_gattc BLE Core Service GATTC
 * @ingroup ble_core_service
 *
 * BLE Core Service GATTC APIs used to implement GATT Clients.
 *
 * This is typically only used to add new client services to BLE service.
 *
 * It provides the following services:
 * - Discover remote \b primary services or a specific service
 * - Discover remote characteristics
 * - Discover remote descriptors
 * - read/write remote characteristics
 * - Getting notified on characteristic changes
 *
 * @{
 */

/**
 * Generic GATTC response message.
 */
struct ble_gattc_rsp {
	struct cfw_message header; /**< Component framework message header (@ref cfw), MUST be first element of structure */
	ble_status_t status;
	uint16_t conn_handle; /**< GAP connection handle */
};

/**
 * Generic GATTC error event.
 */
struct ble_gattc_err_rsp_evt {
	uint16_t err_handle;    /**< handle of char attribute causing the failure */
};

/**
 * Handle range for a service operation.
 */
struct ble_gattc_handle_range {
	uint16_t start_handle;
	uint16_t end_handle;
};

typedef struct {
	struct ble_gattc_handle_range handle_range; /**< range of characteristic handles within a service */
	struct bt_uuid uuid; /**< service uuid */
} ble_gattc_svc_t;

/**
 * Primary Service discovery Indication message @ref MSG_ID_BLE_GATTC_DISC_PRIM_SVC_EVT.
 */
typedef struct {
	uint16_t svc_count;		/**< number of service included into this indication */
	ble_gattc_svc_t service_found[];/**< array on fouTnd services */
} ble_gattc_primary_svc_disc_evt_t;

/**
 * Included service.
 */
typedef struct {
	uint16_t incl_handle;		/**< handle of included service */
	ble_gattc_svc_t svc;		/**< included service */
} ble_gattc_incl_svc_t;

/**
 * Discovered included services @ref MSG_ID_BLE_GATTC_DISC_INCL_SVC_EVT.
 */
typedef struct {
	uint16_t incl_count; /**< Number of included services */
	ble_gattc_incl_svc_t included[]; /**< Array on found services */
} ble_gattc_incl_svc_disc_evt_t;

typedef struct {
	struct ble_gatt_char_properties char_properties;	/**< characteristic properties */
	uint16_t decl_handle;			/**< Characteristic declaration handle */
	uint16_t value_handle;			/**< Char's value handle */
	struct bt_uuid uuid;			/**< Characteristic's UUID */
} ble_gattc_characteristic_t;

/**
 * Discovered characteristics indication @ref MSG_ID_BLE_GATTC_DISC_CHAR_EVT.
 */
typedef struct {
	uint16_t char_count;			/**< number of characteristics in this message */
	ble_gattc_characteristic_t chars[];	/**< characteristics data as per char_count */
} ble_gattc_char_disc_evt_t;

/**
 * GATTC descriptor.
 */
typedef struct {
	uint16_t handle;	/**< descriptor handle */
	struct bt_uuid uuid;	/**< uuid of the descriptor */
} ble_gattc_descriptor_t;

/**
 * Descriptor discover indication.
 */
typedef struct {
	uint16_t desc_count;			/**< number of descriptors in this message */
	ble_gattc_descriptor_t descs[];		/**< found descriptors */
} ble_gattc_desc_disc_evt_t;

enum BLE_GATTC_RD_CHAR_TYPES {
	BLE_GATTC_RD_CHAR_BY_UUID = 0,		/**< Read characteristic by UUID */
	BLE_GATTC_RD_CHAR,			/**< Read (Long) characteristic or (Long) descriptor. Maybe called multiple times in case of long */
	BLE_GATTC_RD_CHAR_MULTIPLE		/**< Read multiple characteristic attributes */
};

/**
 * Characteristic read by using UUID.
 */
typedef struct {
	struct ble_gattc_handle_range handle_range;	/**< characteristic or descriptor handle range */
	struct bt_uuid *p_uuid; /**< uuid of characteristic to read */
} ble_gattc_rd_char_by_uuid_t;

/**
 * Characteristic or descriptor read.
 *
 * Maybe used for long too.
 */
typedef struct {
	uint16_t handle;			/**< attribute handle for reading */
	uint16_t offset;			/**< offset into attribute data to read */
} ble_gattc_rd_char_t;

/**
 * Read multiple characteristics values.
 */
typedef struct {
	uint16_t handle_count;			/**< number of handles in this structure */
	uint16_t handle[];			/**< handles of attributes to read from */
} ble_gattc_rd_multi_char_t;

typedef struct {
	union {
		ble_gattc_rd_char_by_uuid_t char_by_uuid;
		ble_gattc_rd_char_t char_desc;	    /**< (Long) characteristic or descriptor to read */
		ble_gattc_rd_multi_char_t multi_char;
						    /**< read multiple characteristics */
	};
} ble_gattc_rd_characteristic_t;

typedef struct {
	uint16_t char_handle;		/**< handle of characteristic */
	uint16_t len;			/**< if len is bigger then ATT MTU size, the controller fragment buffer itself */
	uint8_t *p_value;		/**< characteristic value to write */
	uint8_t wr_type;		/**< type of write operation @ref BLE_GATT_WR_OP_TYPES */
} ble_gattc_wr_characteristic_t;

/**
 * Read characteristic response indication (@ref MSG_ID_BLE_GATTC_RD_EVT).
 */
typedef struct {
	uint16_t handle;		/**< handle of characteristic attribute read */
	uint16_t offset;		/**< offset of data returned */
	uint16_t len;			/**< length of data returned */
	uint8_t data[];			/**< characteristic attribute data */
} ble_gattc_rd_char_evt_t;

/**
 * Characteristic write response indication @ref MSG_ID_BLE_GATTC_WR_EVT.
 */
typedef struct {
	uint16_t char_handle;
	uint16_t len;
} ble_gattc_wr_char_evt_t;

/**
 * Handle value indication or notification indication/event (@ref MSG_ID_BLE_GATTC_HDL_NOTIF_EVT).
 */
typedef struct {
	uint16_t handle;	/**< handle of characteristic being notified/indicated */
	uint16_t len;		/**< length of value included into this indication */
	uint8_t type;		/**< notification versus indication, @ref BLE_GATT_IND_TYPES */
	uint8_t data[];         /**< value data received */
} ble_gattc_value_evt_t;

/**
 * GATT timeout reason.
 */
typedef struct {
	uint16_t reason;	/**< GATT timeout reason */
} ble_gattc_to_evt_t;

/**
 * GATTC indication or response message structure applicable to most indications/events/responses.
 */
struct ble_gattc_evt_msg {
	struct cfw_message header; /**< Component framework message header (@ref cfw), MUST be first element of structure */
	ble_status_t status;
	uint16_t conn_handle;
	union {
		struct ble_gattc_err_rsp_evt err_rsp; /**< returned only if status != BLE_GATT_STATUS_SUCCESS */
		ble_gattc_primary_svc_disc_evt_t prim_svc_disc;
							    /**< primary service discovery indication event */
		ble_gattc_incl_svc_disc_evt_t incl_svc_disc; /**< included services discovered */
		ble_gattc_char_disc_evt_t char_disc;	    /**< characteristic discover event/indication */
		ble_gattc_desc_disc_evt_t desc_disc;	    /**< discovered descriptors indication/event */
		ble_gattc_rd_char_evt_t char_rd;	    /**< read characteristic indication/event */
		ble_gattc_wr_char_evt_t char_wr;	    /**< characteristic write indication event */
		ble_gattc_value_evt_t val_ind;		    /**< value indication or notification */
		ble_gattc_to_evt_t timeout_ind;		    /**< gattc timeout protocol error */
	};						    /**< in case for responses, union is not used! */
};

/**
 * Discover primary service.
 *
 * @param p_svc_handle service handle
 * @param conn_handle  connection to use
 * @param p_svc_uuid   points to service UUID. if NULL, all services found are returned
 * @param p_priv       pointer to private data
 *
 * @return @ref OS_ERR_TYPE
 * @return MSG: MSG_ID_BLE_GATTC_DISCOVER_PRIMARY_SERVICE_RSP @ref ble_gattc_rsp
 * @return EVT: MSG_ID_BLE_GATTC_DISC_PRIM_SVC_EVT @ref ble_gattc_primary_svc_disc_evt_t
 */
int ble_gattc_discover_primary_service(svc_client_handle_t * p_svc_handle,
				       uint16_t conn_handle,
				       const struct bt_uuid * p_svc_uuid,
				       void *p_priv);

/**
 * Discover included services on a previously discovered primary service.
 *
 * @param p_svc_handle   service handle
 * @param conn_handle    connection to use
 * @param p_handle_range handle range previously returned by @ref ble_gattc_primary_svc_disc_evt_t
 * @param p_priv         pointer to private data
 *
 * @return @ref OS_ERR_TYPE
 * @return MSG: MSG_ID_BLE_GATTC_DISCOVER_INCLUDED_SERVICES_RSP @ref ble_gattc_rsp
 */
int ble_gattc_discover_included_service(svc_client_handle_t * p_svc_handle,
					uint16_t conn_handle,
					const struct ble_gattc_handle_range *
					p_handle_range, void *p_priv);

/**
 * Discover characteristics on a service.
 *
 * May be called several times if not all characteristics have been discovered.
 * In this case a new handle range needs to be provided.
 *
 * @param p_svc_handle   service handle
 * @param conn_handle    connection to use
 * @param p_handle_range handle range
 * @param p_priv         pointer to private data
 *
 * @return @ref OS_ERR_TYPE
 * @return MSG: MSG_ID_BLE_GATTC_DISCOVER_CHAR_RSP @ref ble_gattc_rsp
 *          @ref MSG_ID_BLE_GATTC_DISC_CHAR_EVT @ref ble_gattc_char_disc_evt_t
 */
int ble_gattc_discover_characteristic(svc_client_handle_t * p_svc_handle,
				      uint16_t conn_handle,
				      const struct ble_gattc_handle_range * p_handle_range,
					  void *p_priv);

/**
 * Discover characteristics on a service.
 *
 * May be called several times if not all characteristics have been discovered.
 * In this case a new handle range needs to be provided.
 *
 * @param p_svc_handle   service handle
 * @param conn_handle    connection to use
 * @param p_handle_range handle range
 * @param p_priv         pointer to private data
 *
 * @return @ref OS_ERR_TYPE
 * @return MSG: MSG_ID_BLE_GATTC_DISCOVER_DESCRIPTOR_RSP @ref ble_gattc_rsp
 *          @ref MSG_ID_BLE_GATTC_DISC_DESCR_EVT @ref ble_gattc_desc_disc_evt_t
 */
int ble_gattc_discover_descriptor(svc_client_handle_t * p_svc_handle,
				  uint16_t conn_handle,
				  const struct ble_gattc_handle_range *
				  p_handle_range, void *p_priv);

/**
 * Read characteristic on remote server.
 *
 * @param p_svc_handle    service handle
 * @param conn_handle     connection to use
 * @param type            type of read to execute @ref BLE_GATTC_RD_CHAR_TYPES
 * @param p_rd_char_param read type specific characteristic read parameter
 * @param p_priv          pointer to private data
 *
 * @return @ref OS_ERR_TYPE
 * @return MSG: MSG_ID_BLE_GATTC_RD_CHARS_RSP @ref ble_gattc_rsp
 * @return EVT: MSG_ID_BLE_GATTC_RD_EVT @ref ble_gattc_rd_char_evt_t
 */
int ble_gattc_read_characteristic(svc_client_handle_t * p_svc_handle,
				  uint16_t conn_handle,
				  uint8_t type,
				  const ble_gattc_rd_characteristic_t * p_rd_char_param,
				  void *p_priv);

/**
 * Write characteristic on server.
 *
 * @param p_svc_handle service handle
 * @param conn_handle connection to use
 * @param p_wr_char_param write characteristic on remote service
 * @param p_priv       pointer to private data
 *
 * @return @ref OS_ERR_TYPE,
 * @return MSG: MSG_ID_BLE_GATTC_WR_OP_RSP @ref ble_gattc_rsp
 */
int ble_gattc_write_char_op(svc_client_handle_t * p_svc_handle,
			    uint16_t conn_handle,
			    const ble_gattc_wr_characteristic_t *
			    p_wr_char_param,
			    void *p_priv);


/**
 * Write characteristic on server.
 *
 * @param p_svc_handle service handle
 * @param conn_handle  connection to use
 * @param val_handle   handle to confirm and received by Handle Value Indication (@ref MSG_ID_BLE_GATTC_HDL_NOTIF_EVT)
 * @param p_priv       pointer to private data
 *
 * @return @ref OS_ERR_TYPE,
 * @return MSG: MSG_ID_BLE_GATTC_SEND_HANDLE_VALUE_RSP @ref ble_gattc_rsp
 */
int ble_gattc_send_confirm_handle_value(svc_client_handle_t * p_svc_handle,
					uint16_t conn_handle,
					uint16_t val_handle,
					void *p_priv);

/** @} */

#endif
