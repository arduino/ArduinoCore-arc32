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

#include "BleCommon.h"

enum BleAttributeType {
    BleTypeService        = 0x2800,
    BleTypeCharacteristic = 0x2803,
    BleTypeDescriptor     = 0x2900
};

class BleAttribute {
public:
    BleAttribute(const char* uuid, enum BleAttributeType type);

    const char* uuid() const;
    enum BleAttributeType type() const;

protected:
    static unsigned char numAttributes();

    struct bt_uuid _uuid;

private:
    static unsigned char _numAttributes;

    const char* _uuidStr;
    enum BleAttributeType _type;
};

#endif // _BLE_ATTRIBUTE_H_INCLUDED
