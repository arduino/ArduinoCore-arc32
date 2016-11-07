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
#include "BLEDescriptorImp.h"

#include "internal/ble_client.h"

#include "BLECallbacks.h"

BLEDescriptorImp::BLEDescriptorImp(BLEDevice& bledevice,
                                   BLEDescriptor &descriptor):
     BLEAttribute(descriptor.uuid(), BLETypeDescriptor),
    _value_handle(0)
{
    
    _properties = descriptor.properties();
    _value_length = descriptor.valueLength();
    _value = (unsigned char*)malloc(_value_length);

    memcpy(_value, descriptor.value(), _value_length);
}

BLEDescriptorImp::BLEDescriptorImp(const bt_uuid_t* uuid, 
                                   unsigned char properties, 
                                   uint16_t handle,
                                   BLEDevice& bledevice):
    BLEAttribute(uuid, BLETypeDescriptor),
    _value_handle(handle),
    _properties(properties)
{
    _value_length = BLE_MAX_ATTR_DATA_LEN;
    _value = (unsigned char*)malloc(_value_length);

    memset(_value, 0, _value_length);
}

BLEDescriptorImp::~BLEDescriptorImp() {
    if (_value) {
        free(_value);
        _value = NULL;
    }
}

const unsigned char*
BLEDescriptorImp::value() const
{
    return _value;
}

unsigned short
BLEDescriptorImp::valueLength() const
{
    return _value_length;
}

unsigned char
BLEDescriptorImp::operator[] (int offset) const
{
    return _value[offset];
}

int BLEDescriptorImp::updateProfile(bt_gatt_attr_t *attr_start, int& index)
{
    bt_gatt_attr_t *start = attr_start;
    start->uuid = (struct bt_uuid *)bt_uuid();
    start->perm = BT_GATT_PERM_READ;
    start->read = profile_read_process;
    start->user_data = (void*)((BLEAttribute*)this);
    
    pr_debug(LOG_MODULE_BLE, "Descriptor-%p", start);
    index++;
    return 1;
}

unsigned char BLEDescriptorImp::properties() const
{
    return _properties;
}

int BLEDescriptorImp::valueSize() const
{
    return _value_length;
}


