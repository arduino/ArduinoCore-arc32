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

#ifndef __BLE_SERVICE_GATT_H__
#define __BLE_SERVICE_GATT_H__

#include "ble_service.h"
#include "ble_service_gap_api.h"

/** GATT common definitions.
 *
 * @ingroup ble_core_service
 *
 * @addtogroup ble_core_service_gatt BLE core service common GATT definitions
 * @{
 */

/**
 * GATT Success code and error codes.
 */
enum BLE_SVC_GATT_STATUS_CODES {
	BLE_SVC_GATT_STATUS_SUCCESS = BLE_STATUS_SUCCESS, /**< GATT success @ref BLE_STATUS_SUCCESS */
	BLE_SVC_GATT_STATUS_ENCRYPTED_MITM = BLE_SVC_GATT_STATUS_SUCCESS,
	BLE_SVC_GATT_STATUS_INVALID_HANDLE = BLE_STATUS_GATT_BASE + 0x01,/**< 0x01 see BT Spec Vol 3: Part F (ATT), chapter 3.4.1.1 */
	BLE_SVC_GATT_STATUS_READ_NOT_PERMIT,
	BLE_SVC_GATT_STATUS_WRITE_NOT_PERMIT,
	BLE_SVC_GATT_STATUS_INVALID_PDU,
	BLE_SVC_GATT_STATUS_INSUF_AUTHENTICATION,
	BLE_SVC_GATT_STATUS_REQ_NOT_SUPPORTED,
	BLE_SVC_GATT_STATUS_INVALID_OFFSET,
	BLE_SVC_GATT_STATUS_INSUF_AUTHORIZATION,
	BLE_SVC_GATT_STATUS_PREPARE_Q_FULL,
	BLE_SVC_GATT_STATUS_NOT_FOUND,
	BLE_SVC_GATT_STATUS_NOT_LONG,
	BLE_SVC_GATT_STATUS_INSUF_KEY_SIZE,
	BLE_SVC_GATT_STATUS_INVALID_ATTR_LEN,
	BLE_SVC_GATT_STATUS_ERR_UNLIKELY,
	BLE_SVC_GATT_STATUS_INSUF_ENCRYPTION,
	BLE_SVC_GATT_STATUS_UNSUPPORT_GRP_TYPE,
	BLE_SVC_GATT_STATUS_INSUF_RESOURCE,

	/**< TODO: maybe be not needed, to be covered by generic GAP status */
	BLE_SVC_GATT_STATUS_NO_RESOURCES = BLE_STATUS_GATT_BASE | 0x80,
	BLE_SVC_GATT_STATUS_INTERNAL_ERROR,
	BLE_SVC_GATT_STATUS_WRONG_STATE,
	BLE_SVC_GATT_STATUS_DB_FULL,
	BLE_SVC_GATT_STATUS_BUSY,
	BLE_SVC_GATT_STATUS_ERROR,
	BLE_SVC_GATT_STATUS_CMD_STARTED,
	BLE_SVC_GATT_STATUS_ILLEGAL_PARAMETER,
	BLE_SVC_GATT_STATUS_PENDING,
	BLE_SVC_GATT_STATUS_AUTH_FAIL,
	BLE_SVC_GATT_STATUS_MORE,
	BLE_SVC_GATT_STATUS_INVALID_CFG,
	BLE_SVC_GATT_STATUS_SERVICE_STARTED,
	BLE_SVC_GATT_STATUS_ENCRYPTED_NO_MITM,
	BLE_SVC_GATT_STATUS_NOT_ENCRYPTED,
	BLE_SVC_GATT_STATUS_CONGESTED,
};

/**
 * GATT Server Message ID definitions.
 */
enum BLE_GATTS_MSG_ID {
	/**< GATT Server Requests */
	MSG_ID_BLE_GATTS_ADD_SERVICE_REQ = MSG_ID_BLE_GAP_REQ_LAST,
	MSG_ID_BLE_GATTS_ADD_INCL_SVC_REQ,
	MSG_ID_BLE_GATTS_ADD_CHARACTERISTIC_REQ,
	MSG_ID_BLE_GATTS_ADD_DESCRIPTOR_REQ,
	MSG_ID_BLE_GATTS_START_SERVICE_REQ,
	MSG_ID_BLE_GATTS_REMOVE_SERVICE_REQ,
	MSG_ID_BLE_GATTS_INDICATE_SERVICE_CHANGE_REQ,
	MSG_ID_BLE_GATTS_SET_ATTRIBUTE_VALUE_REQ,
	MSG_ID_BLE_GATTS_GET_ATTRIBUTE_VALUE_REQ,
	MSG_ID_BLE_GATTS_SEND_NOTIF_REQ,
	MSG_ID_BLE_GATTS_SEND_IND_REQ,
	MSG_ID_BLE_GATTS_SEND_RW_AUTHORIZATION_REQ,
	MSG_ID_BLE_GATTS_WR_CONN_ATTRIBUTES_REQ,
	MSG_ID_BLE_GATTS_RD_CONN_ATTRIBUTES_REQ /* 37 */ ,
	MSG_ID_BLE_GATTS_REQ_LAST,

	/**< GATT Server Requests */
	MSG_ID_BLE_GATTS_ADD_SERVICE_RSP = MSG_ID_BLE_GAP_RSP_LAST,
							/**< create new service */
	MSG_ID_BLE_GATTS_ADD_INCL_SVC_RSP,
	MSG_ID_BLE_GATTS_ADD_CHARACTERISTIC_RSP,
	MSG_ID_BLE_GATTS_ADD_DESCRIPTOR_RSP,
	MSG_ID_BLE_GATTS_START_SERVICE_RSP,		    /**< enable created service */
	MSG_ID_BLE_GATTS_REMOVE_SERVICE_RSP,	    /**< stop and remove service */
	MSG_ID_BLE_GATTS_INDICATE_SERVICE_CHANGE_RSP,	/**< indicate a service change */
	MSG_ID_BLE_GATTS_SET_ATTRIBUTE_VALUE_RSP,
	MSG_ID_BLE_GATTS_GET_ATTRIBUTE_VALUE_RSP,
	MSG_ID_BLE_GATTS_SEND_NOTIF_RSP,	    /**< send notification */
	MSG_ID_BLE_GATTS_SEND_IND_RSP,	    /**< send indication */
	MSG_ID_BLE_GATTS_SEND_RW_AUTHORIZATION_RSP,	    /**< authorize a R/W request from remote */
	MSG_ID_BLE_GATTS_WR_CONN_ATTRIBUTES_RSP,	    /**< write connection related attributes (previously bonded!) */
	MSG_ID_BLE_GATTS_RD_CONN_ATTRIBUTES_RSP /* 37 */ ,	    /**< read connection related attributes (only for bonded connections!*/
	MSG_ID_BLE_GATTS_RSP_LAST,

	/**< GATT Server Events */
	MSG_ID_BLE_GATTS_WRITE_EVT = MSG_ID_BLE_GAP_EVT_LAST, /**< remote client write happened */
	MSG_ID_BLE_GATTS_RW_AUTHORIZATION_REQ_EVT,  /**< remote client R/W authorization request */
	MSG_ID_BLE_GATTS_CONN_ATTRIB_MISSING_EVT,   /**< connection related attributes have not been set, access pending */
	MSG_ID_BLE_GATTS_INDICATION_CONF_EVT,	    /**< indication confirmation event */
	MSG_ID_BLE_GATTS_SVC_CHG_CONF_EVT,	    /**< confirmation of service change indication (no params) */
	MSG_ID_BLE_GATTS_TO_EVT,		    /**< GATTS timeout indication */
	MSG_ID_BLE_GATTS_EVT_LAST
};

/**
 * GATT Client Message ID definitions.
 */
enum BLE_GATTC_MSG_ID {
	/**< GATT Client Requests, responses, events and indications */
	MSG_ID_BLE_GATTC_DISCOVER_PRIMARY_SERVICE_REQ =
	    MSG_ID_BLE_GATTS_REQ_LAST,
	MSG_ID_BLE_GATTC_DISCOVER_INCLUDED_SERVICES_REQ,
	MSG_ID_BLE_GATTC_DISCOVER_CHAR_REQ,
	MSG_ID_BLE_GATTC_DISCOVER_DESCRIPTOR_REQ,
	MSG_ID_BLE_GATTC_RD_CHARS_REQ,
	MSG_ID_BLE_GATTC_WR_OP_REQ,
	MSG_ID_BLE_GATTC_SEND_HANDLE_VALUE_REQ /* 44 */ ,

	/** GATT Client Requests, responses, events and indications */
	MSG_ID_BLE_GATTC_DISCOVER_PRIMARY_SERVICE_RSP = MSG_ID_BLE_GATTS_RSP_LAST,  /**< discover primary service */
	MSG_ID_BLE_GATTC_DISCOVER_INCLUDED_SERVICES_RSP,/**< find included service procedure */
	MSG_ID_BLE_GATTC_DISCOVER_CHAR_RSP,		    /**< discover characteristics of a service */
	MSG_ID_BLE_GATTC_DISCOVER_DESCRIPTOR_RSP,	    /**< discover descriptor of a characteristic */
	MSG_ID_BLE_GATTC_RD_CHARS_RSP,		    /**< read characteristic or long characteristics */
	MSG_ID_BLE_GATTC_WR_OP_RSP,			    /**< different types of write operations */
	MSG_ID_BLE_GATTC_SEND_HANDLE_VALUE_RSP /* 44 */ ,   /**< send attribute handle to server */

	/** GATT Client Events */
	MSG_ID_BLE_GATTC_DISC_PRIM_SVC_EVT = MSG_ID_BLE_GATTS_EVT_LAST,	    /**< primary service discovery response */
	MSG_ID_BLE_GATTC_DISC_INCL_SVC_EVT,	    /**< include service discovery response */
	MSG_ID_BLE_GATTC_DISC_CHAR_EVT,		    /**< characteristic discovery response */
	MSG_ID_BLE_GATTC_DISC_DESCR_EVT,	    /**< descriptor discovery response */
	MSG_ID_BLE_GATTC_RD_EVT,		    /**< data read response */
	MSG_ID_BLE_GATTC_WR_EVT,		    /**< data write response */
	MSG_ID_BLE_GATTC_HDL_NOTIF_EVT,		    /**< handle indication/notification event */
	MSG_ID_BLE_GATTC_TO_EVT,		    /**< GATT Client timeout event */
	MSG_ID_BLE_GATTC_LAST
};

/**
 * Maximum UUID size - 16 bytes, and structure to hold any type of UUID.
 */
#define MAX_UUID_SIZE              16

#define BLE_GATT_INVALID_HANDLE              0x0000  /**< reserved invalid attribute handle */
#define BLE_GATT_MAX_HANDLE                  0xffff  /**< maximum handle in a BLE server */
#define BLE_GATT_START_HANDLE_DISCOVER       0x0001  /**< Value of start handle during discovery. */

/** BT uuid types defined as length. */
enum BT_UUID_TYPES {
	BT_UUID16 = 2, /**< 16 bit UUID type */
	BT_UUID32 = 4, /**< 32 bit UUID type */
	BT_UUID128 = 16 /**< 128 bit UUID type */
};

/**
 * Generic uuid structure specific to BT/BLE.
 */
struct bt_uuid {
	uint8_t type; /**< UUID type (encoded as length of the union element) @ref BT_UUID_TYPES */
	union {
		uint16_t uuid16;
		uint32_t uuid32;
		uint8_t uuid128[MAX_UUID_SIZE];
	};
};

/**
 * UUID and Handle combination for services and characteristics
 *
 *  Make sure this is 32 bit aligned!
 */
struct bt_uuid_handle_tuple {
	void *p_priv;		/**< Service private reference handle. */
	uint16_t handle;	/** Service or characteristic handle. */
};

/**
 * GATT service types, primary versus secondary/included one.
 */
enum BLE_GATT_SVC_TYPES {
	BLE_GATT_SVC_PRIMARY = 0,	    /**< primary service */
	BLE_GATT_SVC_INCLUDED		    /**< include service (must be referenced by a primary) */
};

/**
 * Characteristic properties.
 */
enum BLE_GATT_CHAR_PROPS {
	BLE_GATT_CHAR_PROP_BIT_NONE = 0,
	BLE_GATT_CHAR_PROP_BIT_BROADCAST = 0x01,
	BLE_GATT_CHAR_PROP_BIT_READ = 0x02,
	BLE_GATT_CHAR_PROP_BIT_WRITE_NR = 0x04,
	BLE_GATT_CHAR_PROP_BIT_WRITE = 0x08,
	BLE_GATT_CHAR_PROP_BIT_NOTIFY = 0x10,
	BLE_GATT_CHAR_PROP_BIT_INDICATE = 0x20,
	BLE_GATT_CHAR_PROP_BIT_AUTH = 0x40,
	BLE_GATT_CHAR_PROP_BIT_EXTEND = 0x80/**< if set the extend property @ref BLE_GATT_CHAR_EXT_PROPS is present! */
};

/**
 * Extended characteristic properties.
 */
enum BLE_GATT_CHAR_EXT_PROPS {
	BLE_GATT_CHAR_EXT_PROP_BIT_NONE = 0,
	BLE_GATT_CHAR_EXT_PROP_RELIABLE_WR = 0x0001,    /**< Reliable write procedure is supported */
	BLE_GATT_CHAR_EXT_PROP_WR_AUX = 0x0002,         /**< User Descriptor Writes are permitted */
};

struct ble_gatt_char_properties {
	uint8_t props;          /**< properties, @ref BLE_GATT_CHAR_PROPS */
	uint16_t ext_props;     /**< extended properties, @ref BLE_GATT_CHAR_EXT_PROPS, valid if BLE_GATT_CHAR_PROP_BIT_EXTEND set */
};

/**
 * Format of the value of a characteristic, enumeration type.
 */
enum BLE_GATT_FORMATS {
	BLE_GATT_FORMAT_RES,	/* rfu */
	BLE_GATT_FORMAT_BOOL,	/* 0x01 boolean */
	BLE_GATT_FORMAT_2BITS,	/* 0x02 2 bit */
	BLE_GATT_FORMAT_NIBBLE,	/* 0x03 nibble */
	BLE_GATT_FORMAT_UINT8,	/* 0x04 uint8 */
	BLE_GATT_FORMAT_UINT12,	/* 0x05 uint12 */
	BLE_GATT_FORMAT_UINT16,	/* 0x06 uint16 */
	BLE_GATT_FORMAT_UINT24,	/* 0x07 uint24 */
	BLE_GATT_FORMAT_UINT32,	/* 0x08 uint32 */
	BLE_GATT_FORMAT_UINT48,	/* 0x09 uint48 */
	BLE_GATT_FORMAT_UINT64,	/* 0x0a uint64 */
	BLE_GATT_FORMAT_UINT128,/* 0x0B uint128 */
	BLE_GATT_FORMAT_SINT8,	/* 0x0C signed 8 bit integer */
	BLE_GATT_FORMAT_SINT12,	/* 0x0D signed 12 bit integer */
	BLE_GATT_FORMAT_SINT16,	/* 0x0E signed 16 bit integer */
	BLE_GATT_FORMAT_SINT24,	/* 0x0F signed 24 bit integer */
	BLE_GATT_FORMAT_SINT32,	/* 0x10 signed 32 bit integer */
	BLE_GATT_FORMAT_SINT48,	/* 0x11 signed 48 bit integer */
	BLE_GATT_FORMAT_SINT64,	/* 0x12 signed 64 bit integer */
	BLE_GATT_FORMAT_SINT128,/* 0x13 signed 128 bit integer */
	BLE_GATT_FORMAT_FLOAT32,/* 0x14 float 32 */
	BLE_GATT_FORMAT_FLOAT64,/* 0x15 float 64 */
	BLE_GATT_FORMAT_SFLOAT,	/* 0x16 IEEE-11073 16 bit SFLOAT */
	BLE_GATT_FORMAT_FLOAT,	/* 0x17 IEEE-11073 32 bit SFLOAT */
	BLE_GATT_FORMAT_DUINT16,/* 0x18 IEEE-20601 format */
	BLE_GATT_FORMAT_UTF8S,	/* 0x19 UTF-8 string */
	BLE_GATT_FORMAT_UTF16S,	/* 0x1a UTF-16 string */
	BLE_GATT_FORMAT_STRUCT,	/* 0x1b Opaque structure */
	BLE_GATT_FORMAT_MAX	/* 0x1c or above reserved */
};

/**
 * GATT characteristic user description.
 */
struct ble_gatt_char_user_desc {
	uint8_t *buffer;        /**< Pointer to a UTF-8 string. */
	uint8_t len;            /**< The size in bytes of the user description. */
};

/**
 * GATT characteristic presentation format description.
 */
struct ble_gatt_pf_desc {
	uint16_t unit;      /**< as UUIUD defined by SIG */
	uint16_t descr;     /**< as UUID as defined by SIG */
	uint8_t format;     /**< @ref BLE_GATT_FORMATS */
	int8_t exp;         /**< see Unit from Bluetooth Assigned Numbers, https://developer.bluetooth.org/gatt/units/Pages/default.aspx */
	uint8_t name_spc;   /**< name space of the description */
} ;

/**
 * GATT indication types.
 */
enum BLE_GATT_IND_TYPES {
	BLE_GATT_IND_TYPE_NONE = 0,
	BLE_GATT_IND_TYPE_NOTIFICATION,
	BLE_GATT_IND_TYPES_INDICATION,
};

/**
 * GATT Write operation types
 *
 * (BT spec Vol 3, Part G, chapter. 4.9)
 * @note long char write, Prepare & Exe request are handled internally to the controller stack
 */
enum BLE_GATT_WR_OP_TYPES {
	BLE_GATT_WR_OP_NOP = 0,	    /**< normally not used except to cancel BLE_GATT_WR_OP_REQ long char write procedure */
	BLE_GATT_WR_OP_CMD,	    /**< Write Command, (no response) */
	BLE_GATT_WR_OP_REQ,	    /**< Write Request, Write response is received , if length is longer then ATT MTU, Prepare write procedure */
	BLE_GATT_WR_OP_SIGNED_CMD,  /**< Signed Write Command */
};

/** @} */

#endif
