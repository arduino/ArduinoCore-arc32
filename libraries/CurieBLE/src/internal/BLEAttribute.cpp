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

#include "CurieBLE.h"
#include "BLEAttribute.h"

#include "./internal/BLEUtils.h"


BLEAttribute::BLEAttribute(const char* uuid, BLEAttributeType type) :
    _type(type)
{
    memset(&_uuid, 0, sizeof (_uuid));
    BLEUtils::uuidString2BT(uuid, (bt_uuid_t*)&_uuid);
}

BLEAttribute::BLEAttribute(const bt_uuid_t* uuid, BLEAttributeType type) :
    _type(type)
{
    memcpy(&_uuid, uuid, sizeof (_uuid));
}

const bt_uuid_t *BLEAttribute::bt_uuid(void)
{
    return (bt_uuid_t *)&_uuid;
}


BLEAttributeType
BLEAttribute::type() const {
    return this->_type;
}

bool BLEAttribute::compareUuid(const bt_uuid_t* uuid)
{
    int cmpresult = 0;
    cmpresult = bt_uuid_cmp(uuid, (const bt_uuid_t*)&_uuid);
    return (cmpresult == 0);
    
}

bool BLEAttribute::compareUuid(const char* uuid)
{
    bt_uuid_128_t temp;
    BLEUtils::uuidString2BT(uuid,(bt_uuid_t *)&temp);
    return compareUuid((bt_uuid_t *)&temp);
}

