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

#include "BLEAttribute.h"

#include "BLEUuid.h"

unsigned char BLEAttribute::_numAttributes = 0;

BLEAttribute::BLEAttribute(const char* uuid, enum BLEAttributeType type) :
    _uuid(uuid),
    _type(type),
    _handle(0)
{
    _numAttributes++;
}

const char*
BLEAttribute::uuid() const {
    return _uuid;
}

enum BLEAttributeType
BLEAttribute::type() const {
    return this->_type;
}

uint16_t
BLEAttribute::handle() {
    return _handle;
}

void
BLEAttribute::setHandle(uint16_t handle) {
    _handle = handle;
}


bt_uuid
BLEAttribute::btUuid() const {
    BLEUuid bleUuid = BLEUuid(uuid());
    
    return bleUuid.uuid();
}

unsigned char
BLEAttribute::numAttributes() {
    return _numAttributes;
}
