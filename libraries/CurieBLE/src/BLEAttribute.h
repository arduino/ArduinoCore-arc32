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

#ifndef _BLE_ATTRIBUTE_H_INCLUDED
#define _BLE_ATTRIBUTE_H_INCLUDED

#include "BLECommon.h"

/// BLE attribute tyep enum
typedef enum  {
    BLETypeService        = 0x2800, ///< the service type
    BLETypeCharacteristic = 0x2803, ///< the characteristic type
    BLETypeDescriptor     = 0x2900  ///< the descriptor type
}BLEAttributeType;

// Class declare
class BLEProfile;
class BLEPeripheral;
class BLEPeripheralHelper;

class BLEAttribute {
public:
    
    /**
     * Get the string representation of the Attribute
     *
     * @return const char* string representation of the Attribute
     */
    const char* uuid(void) const;
    
    /**
     * Get the string representation of the Attribute
     *
     * @return const char* string representation of the Attribute
     */
    const char* uuid_cstr(void) const;
    
    /**
     * @brief   Get the UUID raw data
     *
     * @param   none
     *
     * @return  bt_uuid_t*     The pointer of UUID
     *
     * @note  none
     */
    bt_uuid_t *uuid(void);
    
    /**
     * @brief   Compare the UUID with the paramater data
     *
     * @param[in]   data 		The pointer of data
     *
     * @param[in]  uuidsize     The max size of UUID
     *
     * @return  bool    true - UUID is the same with data
     *                  false- UUID is not the same with data
     *
     * @note  none
     */
    bool uuidCompare(const uint8_t *data, uint8_t uuidsize);

protected:
    //friend BLEPeripheral;
    friend BLEProfile;

    friend ssize_t profile_write_process(bt_conn_t *conn,
                                     const bt_gatt_attr_t *attr,
                                     const void *buf, uint16_t len,
                                     uint16_t offset);
    friend ssize_t profile_read_process(bt_conn_t *conn,
                                         const bt_gatt_attr_t *attr,
                                         void *buf, uint16_t len,
                                         uint16_t offset);

    friend ssize_t profile_longwrite_process(struct bt_conn *conn,
                                     const struct bt_gatt_attr *attr,
                                     const void *buf, uint16_t len,
                                     uint16_t offset);
    friend int profile_longflush_process(struct bt_conn *conn,
                                         const struct bt_gatt_attr *attr, 
                                         uint8_t flags);

    BLEAttribute(const char* uuid, BLEAttributeType type);

    BLEAttributeType type(void) const;
    uint16_t handle(void);
    void setHandle(uint16_t handle);

    static unsigned char numAttributes(void);
    // The below APIs are for central device to discover peripheral devices
    virtual void discover(bt_gatt_discover_params_t *params) = 0;
    virtual void discover(const bt_gatt_attr_t *attr,
                          bt_gatt_discover_params_t *params) = 0;
    
    /**
     * @brief   Get attribute's discover state
     *
     * @param   none
     *
     * @return  bool    true - In discovering state
     *                  false- Not discovering
     *
     * @note  none
     */
    bool discovering();
    
    bool _discoverying;
private:
    static unsigned char _numAttributes;

    const char* _uuid_cstr;
    bt_uuid_128_t _uuid;
    
    BLEAttributeType _type;
    uint16_t _handle;
    
};

#endif // _BLE_ATTRIBUTE_H_INCLUDED
