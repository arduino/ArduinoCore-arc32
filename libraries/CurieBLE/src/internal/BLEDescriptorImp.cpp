/*
 * Copyright (c) 2016 Intel Corporation.  All rights reserved.
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
#include "BLEUtils.h"

BLEDescriptorImp::BLEDescriptorImp(BLEDevice& bledevice,
                                   BLEDescriptor &descriptor):
    BLEAttribute(descriptor.uuid(), BLETypeDescriptor),
    _value_handle(0),
    _bledev(bledevice),
    _reading(false),
    _attr_desc_value(NULL)
{
    
    _properties = descriptor.properties();
    _value_length = descriptor.valueLength();
    _value = (unsigned char*)malloc(_value_length);

    if (_value)
      memcpy(_value, descriptor.value(), _value_length);
    else
      _value_length = 0;
}

BLEDescriptorImp::BLEDescriptorImp(const bt_uuid_t* uuid, 
                                   unsigned char properties, 
                                   uint16_t handle,
                                   BLEDevice& bledevice):
    BLEAttribute(uuid, BLETypeDescriptor),
    _value_length(0),
    _value_handle(handle),
    _value(NULL),
    _properties(properties),
    _bledev(bledevice),
    _reading(false),
    _attr_desc_value(NULL)
{
    memset(&_read_params, 0, sizeof(_read_params));
}


BLEDescriptorImp::BLEDescriptorImp(const BLEDescriptorImp& rhs) :
    BLEAttribute(rhs),
    _reading(false),
    _attr_desc_value(rhs._attr_desc_value)
{
    _value_length = rhs._value_length;
    _value = (unsigned char *)malloc(_value_length);
    if (_value)
        memcpy(_value, rhs._value, sizeof(_value_length));
    else
        _value_length = 0;

    _value_handle = rhs._value_handle;
    _properties = rhs._properties;
    _bledev = BLEDevice(&rhs._bledev);
}


BLEDescriptorImp& BLEDescriptorImp::operator=(const BLEDescriptorImp& that)
{
    if (this != &that)
    {
        BLEAttribute::operator=(that);
        if (_value)
            free(_value);

        _value_length = that._value_length;
        _value = (unsigned char *)malloc(_value_length);
        if (_value)
            memcpy(_value, that._value, sizeof(_value_length));
        else
            _value_length = 0;

        _value_handle = that._value_handle;
        _properties = that._properties;
        _bledev = BLEDevice(&that._bledev);
        _attr_desc_value = that._attr_desc_value;
    }
    return *this;
}

BLEDescriptorImp::~BLEDescriptorImp() {
    if (_value != (unsigned char *)NULL) {
        free(_value);
        _value = (unsigned char *)NULL;
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

uint16_t
BLEDescriptorImp::valueHandle() const
{
    uint16_t handle = 0;
    if (NULL != _attr_desc_value)
    {
        //GATT server
        handle = _attr_desc_value->handle;
    }
    else
    {
        // GATT client
        handle = _value_handle;
    }
    
    return handle;
}

unsigned char BLEDescriptorImp::properties() const
{
    return _properties;
}

int BLEDescriptorImp::valueSize() const
{
    return _value_length;
}

bool BLEDescriptorImp::read()
{
    int retval = 0;
    bt_conn_t* conn = NULL;
    
    if (true == BLEUtils::isLocalBLE(_bledev))
    {
        // GATT server can't read
        return false;
    }
    
    if (_reading)
    {
        // Already in reading state
        return false;
    }
    
    _read_params.func = profile_descriptor_read_rsp_process;
    _read_params.handle_count = 1;
    _read_params.single.handle = _value_handle;
    _read_params.single.offset = 0;
    
    if (0 == _read_params.single.handle)
    {
        // Discover not complete
        return false;
    }
    
    conn = bt_conn_lookup_addr_le(_bledev.bt_le_address());
    if (NULL == conn)
    {
        return false;
    }
    
    // Send read request
    retval = bt_gatt_read(conn, &_read_params);
    bt_conn_unref(conn);
    if (0 == retval)
    {
        _reading = true;
    }
    return _reading;
}

bool BLEDescriptorImp::writeValue(const byte value[], 
                                  int length, 
                                  int offset)
{
    bool ret = true;
    int total_length = length + offset;
    int write_len = length;
    if (total_length > BLE_MAX_ATTR_DATA_LEN)
    {
        return false;
    }
    
    if (NULL == _value)
    {
        _value_length = length + offset;
        _value = (unsigned char*)malloc(_value_length);

        if (NULL != _value)
        {
            memset(_value, 0, _value_length);
        }
        else
        {
            _value_length = 0;
            ret = false;
        }
    }
    
    if (_value_length < total_length)
    {
        write_len = _value_length - offset;
    }
    
    if (NULL != _value)
    {
        memcpy(_value + offset, value, write_len);
    }
    return ret;
}


