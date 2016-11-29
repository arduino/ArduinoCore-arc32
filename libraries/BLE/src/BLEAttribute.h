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


class BLEAttribute {
public:
    /**
     * @brief   Get the UUID raw data
     *
     * @param   none
     *
     * @return  bt_uuid_t*     The pointer of UUID
     *
     * @note  none
     */
    const bt_uuid_t *bt_uuid(void);
    
    /**
     * @brief   Compare the UUID with the paramater data
     *
     * @param[in]   data 		The pointer of data
     *
     * @param[in]  uuidsize     The max size of UUID
     *
     * @return  bool    true - UUID is the same with data
     *                  		false- UUID is not the same with data
     *
     * @note  none
     */
    bool compareUuid(const char* uuid);
    bool compareUuid(const bt_uuid_t* uuid);
    
    BLEAttributeType type(void) const;

protected:
    BLEAttribute(const char* uuid, BLEAttributeType type);
    BLEAttribute(const bt_uuid_t* uuid, BLEAttributeType type);
private:
    bt_uuid_128_t _uuid;
    
    BLEAttributeType _type;
    
};

#endif // _BLE_ATTRIBUTE_H_INCLUDED
