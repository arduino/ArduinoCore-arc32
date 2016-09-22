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

unsigned char BLEAttribute::_numAttributes = 0;

BLEAttribute::BLEAttribute(const char* uuid, BLEAttributeType type) :
    _uuid_cstr(uuid),
    _type(type),
    _handle(0)
{
    char temp[] = {0, 0, 0};
    int strLength = strlen(uuid);
    int length = 0;
    
    _numAttributes++;
    
    memset (&_uuid, 0x00, sizeof(_uuid));

    for (int i = strLength - 1; i >= 0 && length < MAX_UUID_SIZE; i -= 2)
    {
        if (uuid[i] == '-') 
        {
            i++;
            continue;
        }

        temp[0] = uuid[i - 1];
        temp[1] = uuid[i];

        _uuid.val[length] = strtoul(temp, NULL, 16);

        length++;
    }

    if (length == 2)
    {
        uint16_t temp = (_uuid.val[1] << 8)| _uuid.val[0];
        _uuid.uuid.type = BT_UUID_TYPE_16;
        ((bt_uuid_16_t*)(&_uuid.uuid))->val = temp;
    }
    else
    {
        _uuid.uuid.type = BT_UUID_TYPE_128;
    }
}

const char*
BLEAttribute::uuid() const {
    return _uuid_cstr;
}

const char*
BLEAttribute::uuid_cstr() const {
    return _uuid_cstr;
}

bt_uuid_t *BLEAttribute::uuid(void)
{
    return (bt_uuid_t *)&_uuid;
}


BLEAttributeType
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


unsigned char
BLEAttribute::numAttributes() {
    return _numAttributes;
}

bool BLEAttribute::discovering()
{
    return _discoverying;
}

bool BLEAttribute::uuidCompare(const uint8_t *data, uint8_t uuidsize)
{
	bt_uuid_t * serviceuuid = this->uuid();
    
 	bool status = true;  
	if(serviceuuid->type == BT_UUID_TYPE_16 && uuidsize == UUID_SIZE_16)
	{
		status = memcmp (&((bt_uuid_16_t*)serviceuuid)->val, data, UUID_SIZE_16);
	}
	else if(serviceuuid->type == BT_UUID_TYPE_128 && uuidsize == UUID_SIZE_128)
	{
		status = memcmp (((bt_uuid_128_t*)serviceuuid)->val, data, UUID_SIZE_128);
	}

	return !status;
}
