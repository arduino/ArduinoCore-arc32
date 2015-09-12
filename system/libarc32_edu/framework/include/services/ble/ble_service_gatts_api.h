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

#ifndef __BLE_SERVICE_GATTS_H__
#define __BLE_SERVICE_GATTS_H__

#include "ble_service.h"
#include "ble_service_gap_api.h"
#include "ble_service_gatt.h"

/**  @defgroup ble_core_service_gatts BLE Core Service GATTS
 * @ingroup ble_core_service
 *
 * BLE Core GATTS Service APIs to implement GATT Servers.
 *
 * This API should only be used by BLE service to implement additional BLE profiles/services.
 *
 * Those the GATT server APIs provide the following services:
 * - Create an new (server) BLE service
 * - Add characteristics to the service
 * - Write local server characteristics
 * - Receive data when updated by client
 *
 * @note If a service is based on a 128 bit UUID (vendor service), all the characteristic
 * need to use the same 128 bit UUID base and only vary octets 12-13 of base UUID.
 *
 * @{
 */

/**
 * BLE GATTS max attribute length.
 * @note BLE controller dependent
 */
#define BLE_SVC_GATTS_FIX_ATTR_LEN_MAX 510 /**< Maximum length for fixed length Attribute Values. */
#define BLE_SVC_GATTS_VAR_ATTR_LEN_MAX 512 /**< Maximum length for variable length Attribute Values. */

/* forward declaration for callback handlers */
struct _ble_service_cb;
struct ble_gatts_add_svc_rsp;
struct ble_gatts_add_char_rsp;
struct ble_gatts_add_desc_rsp;
struct ble_gatts_notif_ind_rsp_msg;

/**
 * Generic GATTS response message.
 */
typedef struct {
	struct cfw_message header; /**< Component framework message header (@ref cfw), MUST be first element of structure */
	ble_status_t status;
	uint16_t conn_handle; /**< GAP connection handle */
} ble_gatts_rsp_t;

/**
 * Add Service callback handler.
 */
typedef int (* ble_gatts_add_svc_cback_t)(struct ble_gatts_add_svc_rsp * rsp,
		struct _ble_service_cb * p_cb);

/**
 * Add service response message.
 */
struct ble_gatts_add_svc_rsp {
	struct cfw_message header; /**< Component framework message header (@ref cfw), MUST be first element of structure */
	ble_status_t status; /**< status of service creation */
	ble_gatts_add_svc_cback_t cback; /**< Callback function to execute on reception of this message */
	uint16_t svc_handle; /**< Handle of newly allocated service (only valid in case of success. */
};

/**
 * Include service response.
 */
typedef struct {
	struct cfw_message header; /**< Component framework message header (@ref cfw), MUST be first element of structure */
	ble_status_t status; /**< status of service creation */
	uint16_t svc_handle; /**< updated handle of included service (only valid in case of success */
} ble_gatts_incl_svc_rsp_t;

/**
 * ATT attribute permission
 */
struct ble_gatts_permissions {
	uint8_t rd;    /**< Read permissions, @ref BLE_GAP_SEC_MODES */
	uint8_t wr;    /**< Write permissions @ref BLE_GAP_SEC_MODES */
};

/**
 * GATT characteristic.
 */
struct ble_gatts_characteristic {
	struct bt_uuid * p_uuid;                        /**< Pointer to the characteristic UUID. */
	struct ble_gatts_permissions perms;             /**< Characteristic value attribute permissions */
	struct ble_gatt_char_properties props;          /**< Characteristic Properties. @ref ble_gatt_char_properties */
	uint16_t max_len;                               /**< Maximum characteristic value length in bytes, see @ref BLE_SVC_GATTS_FIX_ATTR_LEN_MAX or @ref BLE_SVC_GATTS_VAR_ATTR_LEN_MAX. */
	uint16_t init_len;                              /**< Initial characteristic value length in bytes. */
	uint8_t * p_value;                              /**< Pointer to the characteristic initialization value */
	// optional descriptors
	struct ble_gatt_char_user_desc * p_user_desc;   /**< Optional user description of the characteristic, NULL if not required */
	struct ble_gatt_pf_desc *p_char_pf_desc;        /**< Pointer to a presentation format structure or NULL if the descriptor is not required. */
};

/**
 * GATT generic descriptor.
 */
struct ble_gatts_descriptor {
	struct bt_uuid * p_uuid; /**< Pointer to the descriptor UUID. */
	uint8_t * p_value; /**< Value of the descriptor */
	uint16_t length; /**< Length of the descriptor value */
	struct ble_gatts_permissions perms; /**< Descriptor attribute permissions */
};

struct ble_gatts_char_handles {
	uint16_t value_handle;		    /**< Handle to the characteristic value. */
	uint16_t cccd_handle;		    /**< Handle to the Client Characteristic Configuration Descriptor, or BLE_GATT_HANDLE_INVALID if not present. */
	uint16_t sccd_handle;		    /**< Handle to the Server Characteristic Configuration Descriptor, or BLE_GATT_HANDLE_INVALID if not present. */
};

/**
 * Add Service callback handler.
 */
typedef int (* ble_gatts_add_char_cback_t)(struct ble_gatts_add_char_rsp * rsp,
		struct _ble_service_cb * p_cb);

/**
 * Add characteristic response message.
 */
struct ble_gatts_add_char_rsp {
	struct cfw_message header; /**< Component framework message header (@ref cfw), MUST be first element of structure */
	ble_status_t status; /**< Status of the operation. */
	ble_gatts_add_char_cback_t cback; /**< Callback function to call on reception of this message */
	struct ble_gatts_char_handles char_h; /**< Handles of the created characteristic */
};

/**
 * Add Service callback handler.
 */
typedef int (* ble_gatts_add_desc_cback_t)(struct ble_gatts_add_desc_rsp * rsp,
		struct _ble_service_cb * p_cb);

/**
 * Add descriptor response message.
 */
struct ble_gatts_add_desc_rsp {
	struct cfw_message header; /**< Component framework message header (@ref cfw), MUST be first element of structure */
	ble_status_t status; /**< Status of the operation. */
	ble_gatts_add_desc_cback_t cback; /**< Callback function to call on reception of this message */
	uint16_t handle; /**< Handle of the created descriptor */
};

/**
 * Set attribute response message.
 */
struct  ble_gatts_set_attr_rsp_msg {
	struct cfw_message header; /**< Component framework message header (@ref cfw), MUST be first element of structure */
	ble_status_t status;
	uint16_t value_handle;
};

/**
 * Notification/Indication callback.
 */
typedef int (* ble_gatts_notif_ind_cback_t)(struct ble_gatts_notif_ind_rsp_msg * rsp,
		struct _ble_service_cb * p_cb);

/**
 * Notification/Indication response message.
 */
struct ble_gatts_notif_ind_rsp_msg {
	struct cfw_message header; /**< Component framework message header (@ref cfw), MUST be first element of structure */
	ble_status_t status;
	uint16_t conn_handle;
	ble_gatts_notif_ind_cback_t cback; /**< Callback function to call on reception of this message */
	uint16_t handle;                /**< Characteristic value handle */
};

/**
 * Shortened attribute type definitions.
 * See BT Spec r Vol 3, PART G, chapter 3
 */
enum BLE_SVC_GATTS_ATTR_TYPES {
	BLE_SVC_GATTS_ATTR_TYPE_NONE = 0,
	BLE_SVC_GATTS_ATTR_TYPE_PRIMARY_SVC_DECL,
					    /**< primary service attribute declaration (chpt 3.1) */
	BLE_SVC_GATTS_ATTR_TYPE_SECONDARY_SVC_DECL,
					    /**< secondary service attribute declaration (chpt 3.1) */
	BLE_SVC_GATTS_ATTR_TYPE_INCLUDE_DECL,   /**< include attribute declaration (3.2) */
	BLE_SVC_GATTS_ATTR_TYPE_CHAR_DECL,	    /**< characteristic declaration (3.3.1) */
	BLE_SVC_GATTS_ATTR_TYPE_CHAR_VALUE_DECL,/**< Characteristic value declaration */
	BLE_SVC_GATTS_ATTR_TYPE_DESC_DECL,	    /**< descriptor declaration */
};

/**
 * GATT server write ops.
 */
enum BLE_GATTS_WR_OPS {
	BLE_GATTS_OP_NONE = 0,
	BLE_GATTS_OP_WR,		    /**< 3.4.5.1 Write Request (Attribute), expects write response */
	BLE_GATTS_OP_WR_CMD,		    /**< 3.4.5.3 Write Command (Attribute) NO response sent */
	BLE_GATTS_OP_WR_CMD_SIGNED,	    /**< 3.4.5.4 Write Command Signed (Attribute), NO response sent */
	BLE_GATTS_OP_WR_PREP_REQ,	    /**< 3.4.6.1 Write Prepare Request, expects a prepare write request response */
	BLE_GATTS_OP_WR_EXE_REQ_CANCEL,	    /**< 3.4.6.3 Cancel Executed Write Request, cancel and clear queue (flags = 0) */
	BLE_GATTS_OP_WR_EXE_REQ_IMM	    /**< 3.4.6.3 Immediately Execute Write Request */
};

/**
 * Write authorization context data structure.
 */
typedef struct {
	uint16_t attr_handle;			    /**< handle of attribute to write */
	uint16_t offset;
	uint16_t len;
	uint8_t op;				    /**< @ref BLE_GATTS_WR_OPS */
	uint8_t data[];
} ble_gatt_wr_evt_t;

/**
 * Read authorization context data structure.
 */
typedef struct {
	uint16_t attr_handle;			    /**< handle of attribute to been read */
	uint16_t offset;
} ble_gatt_rd_evt_t;

/**
 * Connection attribute data is missing @ref MSG_ID_BLE_GATTS_CONN_ATTRIB_MISSING_EVT.
 */
typedef struct {
	uint16_t miss_type;		/**< missing connection attribute type */
} ble_gatts_conn_attrib_missing_evt_t;

/**
 * Handle Value Confirmation, @ref MSG_ID_BLE_GATTS_INDICATION_CONF_EVT in response an handle value indication.
 */
typedef struct {
	uint16_t handle;		    /**< attribute handle of indication value sent */
} ble_gatts_handle_value_conf_evt_t;

/**
 * Read attribute value rsp message.
 */
typedef struct {
	struct cfw_message header; /**< Component framework message header (@ref cfw), MUST be first element of structure */
	ble_status_t status;
	uint16_t conn_handle;		    /**< GAP connection handle */
	uint16_t val_handle;		    /**< handle of attribute value */
	uint16_t len;			    /**< length of value returned */
	uint16_t offset;		    /**< offset in the value. the same as in the rd request! */
	uint8_t value[];		    /**< value data of length \ref len */
} ble_gatts_rd_attrib_val_rsp_t;

/**
 * Indication or notification.
 */
typedef struct {
	uint16_t val_handle;		    /**< handle of attribute value */
	uint16_t len;			    /**< len of attribute data value to indicate */
	uint8_t *p_data;		    /**< data to indicate (maybe NULL if currently stored shall be used) */
	uint16_t offset;		    /**< optional offset into attribute value data */
} ble_gatts_ind_params_t;

/**
 * Connection related attribute data parameters (stack specific).
 */
typedef struct {
	struct cfw_message header; /**< Component framework message header (@ref cfw), MUST be first element of structure */
	ble_status_t status;		/**< result for request, if not success, the data afterwards may not be valid */
	uint16_t conn_handle;		/**< connection handle of the retrieved connection attribute data */
	uint16_t len;			/**< length of the following connection attribute data */
	uint8_t conn_attr_data[];
} ble_gatts_rd_conn_attr_rsp_t;

/**
 * GATTS timeout @ref MSG_ID_BLE_GATTS_TO_EVT.
 */
typedef struct {
	uint16_t reason;	/**< reason for timeout */
} ble_gatts_timeout_evt_t;

/**
 * BLE GATTS Indication Data structure.
 */
struct ble_gatts_evt_msg {
	struct cfw_message header; /**< Component framework message header (@ref cfw), MUST be first element of structure */
	ble_status_t status; /**< result for request, if not success, the data afterwards may not be valid */
	uint16_t conn_handle;
	union {
		ble_gatt_wr_evt_t wr;			    /**< write indication */
		ble_gatts_conn_attrib_missing_evt_t conn_attr;
							    /**< connection related attribute missing */
		ble_gatts_handle_value_conf_evt_t handle_val_conf;
							    /**< value confirmation (confirmation of an indication) */
		ble_gatts_timeout_evt_t timeout;	    /**< GATTS timeout occurred */
	};
};

/**
 * Create an new service, primary or include.
 *
 * @param p_svc_handle service handle
 * @param p_uuid       UUID of new service
 * @param type         primary versus included @ref BLE_GATT_SVC_TYPES
 * @param cback        Callback function to be called on reception of add service response
 * @param p_priv       pointer to private data
 *
 * @return @ref OS_ERR_TYPE
 * @return MSG: MSG_ID_BLE_GATTS_ADD_SERVICE_RSP @ref ble_gatts_add_svc_rsp
 */
int ble_gatts_add_service(svc_client_handle_t * p_svc_handle,
			  const struct bt_uuid * p_uuid,
			  uint8_t type,
			  ble_gatts_add_svc_cback_t cback,
			  void *p_priv);

/**
 * Include a (secondary) service into a primary service.
 *
 * @param p_svc_handle          service handle
 * @param svc_handle            service to which to add the included service
 * @param svc_handle_to_include the previously created includable service
 * @param p_priv                pointer to private data
 *
 * @return @ref OS_ERR_TYPE
 * @return MSG: MSG_ID_BLE_GATTS_ADD_INCL_SVC @ref ble_gatts_incl_svc_rsp_t
 */
int ble_gatts_add_included_svc(svc_client_handle_t * p_svc_handle,
			       uint16_t svc_handle,
			       uint16_t svc_handle_to_include,
			       void *p_priv);

/**
 * Add a characteristic to service.
 *
 * @note this may called with the same UUID. the returned handle will be different in this case to
 *       distinguish multiple instances of the same char
 *
 * @param p_svc_handle service handle
 * @param svc_handle   service to which to add the characteristic
 * @param p_char       meta data for characteristic
 * @param cback        Callback function called on reception of response message
 * @param p_priv       pointer to private data
 *
 * @return @ref OS_ERR_TYPE
 * @return MSG: MSG_ID_BLE_GATTS_ADD_CHARACTERISTIC @ref ble_gatts_add_char_rsp
 */
int ble_gatts_add_characteristic(svc_client_handle_t * p_svc_handle,
				 uint16_t svc_handle,
				 const struct ble_gatts_characteristic * p_char,
				 ble_gatts_add_char_cback_t cback,
				 void *p_priv);

/**
 * Add a descriptor to the last added characteristic.
 *
 * @note The descriptor is automatically added to the latest
 *       added characteristic
 *
 * @param p_svc_handle service handle
 * @param p_desc       description of the descriptor
 * @param cback        Callback function called on reception of response message
 * @param p_priv       pointer to private data
 *
 * @return @ref OS_ERR_TYPE
 * @return MSG: MSG_ID_BLE_GATTS_ADD_DESCRIPTOR @ref ble_gatts_add_desc_rsp
 */
int ble_gatts_add_descriptor(svc_client_handle_t * p_svc_handle,
			     const struct ble_gatts_descriptor * p_desc,
			     ble_gatts_add_desc_cback_t cback,
			     void * p_priv);

/**
 * Start BLE Service setup before.
 *
 * @param p_svc_handle service handle
 * @param svc_handle service to start
 * @param p_priv       pointer to private data
 *
 * @return @ref OS_ERR_TYPE
 * @return MSG: MSG_ID_BLE_GATTS_START_SERVICE @ref ble_rsp
 */
int ble_gatts_start_service(svc_client_handle_t * p_svc_handle,
			    uint16_t svc_handle,
			    void *p_priv);

/**
 * Stop and remove service.
 *
 * @note Not yet supported
 *
 * @param p_svc_handle service handle
 * @param svc_handle   handle of characteristic to which to add the descriptor
 * @param p_priv       pointer to private data
 *
 * @return @ref OS_ERR_TYPE
 * @return MSG: MSG_ID_BLE_GATTS_REMOVE_SERVICE_RSP @ref ble_rsp
 */
int ble_gatts_remove_service(svc_client_handle_t * p_svc_handle,
			     uint16_t svc_handle,
			     void *p_priv);

/**
 * Send a service change indication.
 *
 * @note Not yet supported
 *
 * @param p_svc_handle service handle
 * @param conn_handle  handle of the connection affected by the service layout change
 * @param start_handle start handle of changed attribute handle range
 * @param end_handle   end handle of changed attribute handle range
 * @param p_priv       pointer to private data
 *
 * @return @ref OS_ERR_TYPE
 * @return MSG: MSG_ID_BLE_GATTS_INDICATE_SERVICE_CHANGE @ref ble_gatts_rsp_t
 */
int ble_gatts_send_svc_changed(svc_client_handle_t * p_svc_handle,
			       uint16_t conn_handle,
			       uint16_t start_handle,
			       uint16_t end_handle,
			       void *p_priv);

/**
 * Set an attribute value.
 *
 * @param p_svc_handle service handle
 * @param value_handle handle of value to change
 * @param len          length of attribute value to write
 * @param p_value      attribute value data to write
 * @param offset       optional offset from which on to write the attribute value data
 * @param p_priv       pointer to private data
 *
 * @return @ref OS_ERR_TYPE
 * @return MSG: MSG_ID_BLE_GATTS_WR_ATTRIBUTE_VALUE ble_gatts_wr_attr_rsp_msg
 */
int ble_gatts_set_attribute_value(svc_client_handle_t * p_svc_handle,
				 uint16_t value_handle,
				 uint16_t len,
				 const uint8_t * p_value,
				 uint16_t offset,
				 void *p_priv);

/**
 * Get an attribute value.
 *
 * @param p_svc_handle service handle
 * @param value_handle handle of the attribute value to retrieve
 * @param len          length of the attribute value to get
 * @param offset       optional offset from which on to get the attribute value
 * @param p_priv       pointer to private data
 *
 * @return @ref OS_ERR_TYPE
 * @return MSG: MSG_ID_BLE_GATTS_RD_ATTRIBUTE_VALUE @ref ble_gatts_rd_attrib_val_rsp_t
 */
int ble_gatts_get_attribute_value(svc_client_handle_t * p_svc_handle,
				 uint16_t value_handle,
				 uint16_t len,
				 uint16_t offset,
				 void *p_priv);

/**
 * Send notification.
 *
 * @param p_svc_handle service handle
 * @param conn_handle  handle of the connection affected by the service layout change
 * @param p_ind_params length of attribute value to write
 * @param cback        callback function to be called on reception of reception of notif response
 * @param p_priv       pointer to private data
 *
 * @return @ref OS_ERR_TYPE
 * @return MSG: MSG_ID_BLE_GATTS_SEND_NOTIF_RSP @ref ble_gatts_notif_ind_rsp_msg
 */
int ble_gatts_send_notif(svc_client_handle_t * p_svc_handle,
			     uint16_t conn_handle,
			     const ble_gatts_ind_params_t * p_params,
			     ble_gatts_notif_ind_cback_t cback,
			     void *p_priv);

/**
 * Send indication.
 *
 * @param p_svc_handle service handle
 * @param conn_handle  handle of the connection affected by the service layout change
 * @param p_ind_params length of attribute value to write
 * @param cback        callback function to be called on reception of reception of ind response
 * @param p_priv       pointer to private data
 *
 * @return @ref OS_ERR_TYPE
 * @return MSG: MSG_ID_BLE_GATTS_SEND_NOTIF_RSP @ref ble_gatts_notif_ind_rsp_msg
 */
int ble_gatts_send_ind(svc_client_handle_t * p_svc_handle,
			     uint16_t conn_handle,
			     const ble_gatts_ind_params_t * p_params,
			     ble_gatts_notif_ind_cback_t cback,
			     void *p_priv);

/**
 * Write stack specific data of a previously bonded connection.
 *
 * @note this data is typically stored in NVRAM in relation ship to the bonded device!
 *       (e.g. CCD)
 *
 * @param p_svc_handle service handle
 * @param conn_handle  handle of the connection
 * @param p_data       data blob specific to stack to write
 * @param len          length of above byte stream (little endian)
 * @param p_priv       pointer to private data
 *
 * @return @ref OS_ERR_TYPE
 * @return MSG: MSG_ID_BLE_GATTS_WR_CONN_ATTRIBUTES @ref ble_gatts_rsp_t
 */
int ble_gatts_write_conn_attributes(svc_client_handle_t * p_svc_handle,
				    uint16_t conn_handle,
				    const uint8_t * p_data,
				    uint16_t len,
				    void *p_priv);

/**
 * Read stack specific data of the bonded connection.
 *
 * @note this data is typically stored in NVRAM in relation ship to the bonded device!
 *
 * @param p_svc_handle service handle
 * @param conn_handle  handle of the connection
 * @param p_priv       pointer to private data
 *
 * @return @ref OS_ERR_TYPE
 * @return MSG: MSG_ID_BLE_GATTS_RD_CONN_ATTRIBUTES @ref ble_gatts_rd_conn_attr_rsp_t
 */
int ble_gatts_read_conn_attributes(svc_client_handle_t * p_svc_handle,
				   uint16_t conn_handle,
				   void *p_priv);

/** @} */

#endif
