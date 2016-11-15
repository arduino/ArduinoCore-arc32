/*
 * Copyright (c) 2015 Intel Corporation.  All rights reserved.
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.

 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.

 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
 *
 */

#ifndef _BLE_CHARACTERISTIC_H_INCLUDED
#define _BLE_CHARACTERISTIC_H_INCLUDED

#include "BLECommon.h"

#include "BLEAttribute.h"
#include "BLECentralHelper.h"
#include "BLEDescriptor.h"

/**
 * BLE Characteristic Events
 */
enum BLECharacteristicEvent {
    BLEWritten = 0,
    BLESubscribed = 1,
    BLEUnsubscribed = 2,

    BLECharacteristicEventLast = 3
};

/* Forward declaration needed for callback function prototype below */
class BLECharacteristic;
class BLEPeripheral;
class BLEHelper;

/** Function prototype for BLE Characteristic event callback */
typedef void (*BLECharacteristicEventHandler)(BLEHelper &bleHelper, BLECharacteristic &characteristic);

/**
 * BLE Characteristic Property types
 */
enum BLEProperty {
    // broadcast (0x01) not supported
    BLERead                 = 0x02,
    BLEWriteWithoutResponse = 0x04,
    BLEWrite                = 0x08,
    BLENotify               = 0x10,
    BLEIndicate             = 0x20
};

/**
 * BLE GATT Characteristic
 */
class BLECharacteristic : public BLEAttribute {
public:
    /**
     * Constructor for BLE Characteristic
     *
     * @param[in] uuid         16-bit or 128-bit UUID (in string form) defined by BLE standard
     * @param[in] properties   Characteristic property mask
     * @param[in] maxLength    Maximum data length required for characteristic value (<= BLE_MAX_ATTR_DATA_LEN)
     */
    BLECharacteristic(const char* uuid,
                      const unsigned char properties,
                      const unsigned short maxLength);

    /**
     * Constructor for BLE Characteristic
     *
     * @param[in] uuid         16-bit or 128-bit UUID (in string form) defined by BLE standard
     * @param[in] properties   Characteristic property mask
     * @param[in] value        String value for characteristic (string length (<= BLE_MAX_ATTR_DATA_LEN))
     */
    BLECharacteristic(const char* uuid,
                      const unsigned char properties,
                      const char* value);

    virtual ~BLECharacteristic();

    /**
     * Set the current value of the Characteristic
     *
     * @param[in] value  New value to set, as a byte array.  Data is stored in internal copy.
     * @param[in] length Length, in bytes, of valid data in the array to write.
     *               Must not exceed maxLength set for this characteristic.
     *
     * @return bool true set value success, false on error
     */
    bool setValue(const unsigned char value[], unsigned short length);

    /**
     * Set the current value of the Characteristic
     *
     * @param[in] central The central device that update the value.
     * @param[in] value  New value to set, as a byte array.  Data is stored in internal copy.
     * @param[in] length Length, in bytes, of valid data in the array to write.
     *               Must not exceed maxLength set for this characteristic.
     *
     * @return bool true set value success, false on error
     */
    void setValue(BLEHelper& blehelper, const uint8_t value[], uint16_t length);

    /**
     * Get the property mask of the Characteristic
     *
     * @return unsigned char property mask of the Characteristic
     */
    unsigned char properties(void) const;

    /**
     * Get the (maximum) size of the Characteristic
     *
     * @return unsigned size of characateristic in bytes
     */
    unsigned short valueSize(void) const;

    /**
     * Get data pointer to the value of the Characteristic
     *
     * @return const unsigned char* pointer to the value of the Characteristic
     */
    const unsigned char* value(void) const;

    /**
     * Get the current length of the value of the Characteristic
     *
     * @return unsigned short size of characateristic value in bytes
     */
    unsigned short valueLength() const;

    unsigned char operator[] (int offset) const;

    /**
     * Has the value of the Characteristic been written by a central
     *
     * @return bool true is central has updated characteristic value, otherwise false
     */
    bool written(void);

    /**
     * Is a central listening for notifications or indications of the Characteristic
     *
     * @return bool true is central is subscribed, otherwise false
     */
    bool subscribed(void);

    /**
     * Provide a function to be called when events related to this Characteristic are raised
     *
     * @param[in] event Event type to set event handler for
     * @param[in] callback  Pointer to callback function to invoke when the event occurs.
     */
    void setEventHandler(BLECharacteristicEvent event, BLECharacteristicEventHandler callback);

    /**
     * @brief   Get Notify Attribute counter that created
     *
     * @param   none
     *
     * @return  unsigned char   The totla number of the notify attributes
     *
     * @note  none
     */
    static unsigned char numNotifyAttributes(void);
    
    /**
     * @brief   Schedule the read request to read the characteristic in peripheral
     *
     * @param[in]   peripheral  The peripheral device that want to read.
     *
     * @return  bool    Indicate the success or error
     *
     * @note  Only for central device
     */
    bool read(BLEPeripheralHelper &peripheral);
    
    /**
     * @brief   Schedule the write request to update the characteristic in peripheral
     *
     * @param[in]   peripheral   The peripheral device that want to be updated
     * @param[in]   value       New value to set, as a byte array.  Data is stored in internal copy.
     * @param[in]   length      Length, in bytes, of valid data in the array to write.
     *                      Must not exceed maxLength set for this characteristic.
     *
     * @return  bool true set value success, false on error
     *
     * @note  none
     */
    bool write(BLEPeripheralHelper &peripheral, 
               const unsigned char value[], 
               uint16_t length);

protected:
    friend class BLEProfile;
    friend int profile_longflush_process(struct bt_conn *conn,
                                         const struct bt_gatt_attr *attr, 
                                         uint8_t flags);
    friend ssize_t profile_longwrite_process(struct bt_conn *conn,
                                     const struct bt_gatt_attr *attr,
                                     const void *buf, uint16_t len,
                                     uint16_t offset);
    
    void addCharacteristicDeclaration(bt_gatt_attr_t *gatt_attr);
    void addCharacteristicValue(bt_gatt_attr_t *gatt_attr);
    void addCharacteristicConfigDescriptor(bt_gatt_attr_t *gatt_attr);
    
    bool longCharacteristic();
    
    void setBuffer(BLEHelper& blehelper, 
                   const uint8_t value[], 
                   uint16_t length, 
                   uint16_t offset);
    void discardBuffer();
    void syncupBuffer2Value(BLEHelper& blehelper);
    
    /**
     * @brief   Get the characteristic value handle
     *
     * @param   none
     *
     * @return  none
     *
     * @note  Only for peripheral
     */
    uint16_t valueHandle(void);
    
    /**
     * @brief   Get characteristic configuration descriptor value handle
     *
     * @param   none
     *
     * @return  uint16_t        The value handle
     *                           0 is invalid handle
     *
     * @note  Only for peripheral
     */
    uint16_t cccdHandle(void);

    
    void setUserDescription(BLEDescriptor *descriptor);
    void setPresentationFormat(BLEDescriptor *descriptor);
    
    _bt_gatt_ccc_t* getCccCfg(void);
    bt_gatt_chrc_t* getCharacteristicAttValue(void);
    static bt_uuid_t* getCharacteristicAttributeUuid(void);
    static bt_uuid_t* getClientCharacteristicConfigUuid(void);
    
    /**
     * @brief   Get the characteristic permission
     *
     * @param   none
     *
     * @return  uint8_t The characteristic permission
     *
     * @note  none
     */
    uint8_t getPermission(void);
    
    /**
     * @brief   For central to discover the peripherial profile
     *
     * @param[in]   attr    The discover response
     *
     * @param[in]   params  The discover parameter that need to fill
     *
     * @return  none
     *
     * @note  Only for central
     */
    void discover(const bt_gatt_attr_t *attr,
			      bt_gatt_discover_params_t *params);
    
    /**
     * @brief   For central to discover the peripherial profile
     *
     * @param[in]   params  The discover parameter that need to fill
     *
     * @return  none
     *
     * @note  Only for central
     */
    void discover(bt_gatt_discover_params_t *params);
    
    /**
     * @brief   Get the subscribe parameter
     *
     * @param   none
     *
     * @return bt_gatt_subscribe_params_t * the subscribe parameter
     *
     * @note  Only for central
     */
   bt_gatt_subscribe_params_t* getSubscribeParams();

private:
    void _setValue(const uint8_t value[], uint16_t length);

private:
    
    static unsigned char _numNotifyAttributes;
    static bt_uuid_16_t _gatt_chrc_uuid;
    static bt_uuid_16_t _gatt_ccc_uuid;
    
    unsigned short _value_size;
    unsigned short _value_length;
    unsigned char* _value;
    unsigned char* _value_buffer;
    bool _written;

    uint16_t _value_handle;
    bt_gatt_ccc_cfg_t	_ccc_cfg;
    _bt_gatt_ccc_t _ccc_value;
    bt_gatt_chrc_t _gatt_chrc;

    BLEDescriptor* _user_description;
    BLEDescriptor* _presentation_format;
    
    bt_gatt_attr_t *_attr_chrc_declaration;
    bt_gatt_attr_t *_attr_chrc_value;
    bt_gatt_attr_t *_attr_cccd;
    
    // For central device to subscribe the Notification/Indication
    bt_gatt_subscribe_params_t _sub_params;
    
    bool _reading;
    bt_gatt_read_params_t _read_params;
    BLECharacteristicEventHandler _event_handlers[BLECharacteristicEventLast];
};

#endif // _BLE_CHARACTERISTIC_H_INCLUDED
