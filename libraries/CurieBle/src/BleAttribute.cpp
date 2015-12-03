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

#include "BleAttribute.h"

#include "BleUuid.h"

unsigned char BleAttribute::_numAttributes = 0;

BleAttribute::BleAttribute(const char* uuid, enum BleAttributeType type) :
    _uuid(uuid),
    _type(type),
    _handle(0)
{
    _numAttributes++;
}

const char*
BleAttribute::uuid() const {
    return _uuid;
}

enum BleAttributeType
BleAttribute::type() const {
    return this->_type;
}

uint16_t
BleAttribute::handle() {
    return _handle;
}

void
BleAttribute::setHandle(uint16_t handle) {
    _handle = handle;
}


bt_uuid
BleAttribute::btUuid() const {
    BleUuid bleUuid = BleUuid(uuid());
    
    return bleUuid.uuid();
}

unsigned char
BleAttribute::numAttributes() {
    return _numAttributes;
}
