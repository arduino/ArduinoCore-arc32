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

enum BLEAttributeType {
    BLETypeService        = 0x2800,
    BLETypeCharacteristic = 0x2803,
    BLETypeDescriptor     = 0x2900
};

class BLEPeripheral;

class BLEAttribute {
public:
    
    /**
     * Get the string representation of the Attribute
     *
     * @return const char* string representation of the Attribute
     */
    const char* uuid(void) const;

protected:
    friend BLEPeripheral;

    BLEAttribute(const char* uuid, enum BLEAttributeType type);

    BLEAttributeType type(void) const;
    bt_uuid btUuid(void) const;
    uint16_t handle(void);
    void setHandle(uint16_t handle);

    static unsigned char numAttributes(void);

private:
    static unsigned char _numAttributes;

    const char* _uuid;
    enum BLEAttributeType _type;
    uint16_t _handle;
};

#endif // _BLE_ATTRIBUTE_H_INCLUDED
