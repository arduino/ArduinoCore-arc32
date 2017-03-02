/*
  BLE Descriptor API
  Copyright (c) 2016 Arduino LLC. All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301 USA
*/
#include <errno.h>

#include "./internal/BLEAttribute.h"
#include "BLEDescriptor.h"
#include "./internal/BLEUtils.h"
#include "./internal/BLEDescriptorImp.h"

BLEDescriptor::BLEDescriptor():
    _properties(0),
    _value_size(0),
    _value(NULL)
{
    memset(_uuid_cstr, 0, sizeof (_uuid_cstr));
}
    
BLEDescriptor::BLEDescriptor(BLEDescriptorImp* descriptorImp, 
                             const BLEDevice *bleDev):
    _bledev(bleDev),
    _value_size(0),
    _value(NULL),
    _internal(descriptorImp)
{
    _properties = descriptorImp->properties();
    memset(_uuid_cstr, 0, sizeof (_uuid_cstr));
    BLEUtils::uuidBT2String(descriptorImp->bt_uuid(), _uuid_cstr);
}

BLEDescriptor::BLEDescriptor(const char* uuid, 
                             const unsigned char value[], 
                             unsigned short valueLength):
    _bledev(),
    _internal(NULL)
{
    bt_uuid_128_t uuid_tmp;
    memset(_uuid_cstr, 0, sizeof (_uuid_cstr));
    BLEUtils::uuidString2BT(uuid, (bt_uuid_t *)&uuid_tmp);
    BLEUtils::uuidBT2String((const bt_uuid_t *)&uuid_tmp, _uuid_cstr);
    
    _bledev.setAddress(*BLEUtils::bleGetLoalAddress());
    
    _value_size = (valueLength > BLE_MAX_ATTR_LONGDATA_LEN) ? BLE_MAX_ATTR_LONGDATA_LEN : valueLength;
    _value = (unsigned char*)malloc(_value_size);
    if (NULL != _value)
    {
        memcpy(_value, value, _value_size);
    }
    else
    {
        errno = ENOMEM;
        _value_size = 0;
    }
}

BLEDescriptor::BLEDescriptor(const char* uuid, 
                             const char* value):
     BLEDescriptor(uuid, (const unsigned char*)value, strlen(value))
{}

BLEDescriptor::BLEDescriptor(const BLEDescriptor& rhs):
    _bledev(&rhs._bledev),
    _properties(rhs._properties),
    _value_size(0),
    _value(NULL),
    _internal(rhs._internal)
{
    memcpy(_uuid_cstr, rhs._uuid_cstr, sizeof(_uuid_cstr));
    if (NULL == _internal && rhs._value_size > 0)
    {
        _value = (unsigned char*)malloc(rhs._value_size);  // Sid. KW: allocate memory for _value, not local
        if (_value) 
        {
            memcpy(_value, rhs._value, rhs._value_size);
            _value_size = rhs._value_size;
        }
        else
        {
            errno = ENOMEM;
        }
    }
}

BLEDescriptor& BLEDescriptor::operator= (const BLEDescriptor& rhs)
{
    if (this != &rhs)
    {
        memcpy(_uuid_cstr, rhs._uuid_cstr, sizeof(_uuid_cstr));
        _properties = rhs._properties;
        _bledev = BLEDevice(&rhs._bledev);
        _internal = rhs._internal;
        if (NULL == _internal && rhs._value_size > 0)
        {
            if (_value_size < rhs._value_size)
            {
                _value_size = rhs._value_size;
                
                if (NULL != _value)
                    free(_value);
                _value = (unsigned char*)malloc(_value_size);
            }
            
            if (NULL != _value)
            {
                memcpy(_value, rhs._value, rhs._value_size);
            }
            else
            {
                _value_size = 0;
                errno = ENOMEM;
            }
        }
    }
    return *this;
}

BLEDescriptor::~BLEDescriptor()
{
    if (_value)
    {
        free(_value);
        _value = NULL;
    }
}

const char* BLEDescriptor::uuid() const
{
    return _uuid_cstr;
}

const byte* BLEDescriptor::value() const
{
    const byte* ret = _value;
    if (NULL != _internal)
    {
        ret = _internal->value();
    }
    return ret;
}

int BLEDescriptor::valueLength() const
{
    int ret = _value_size;
    if (NULL != _internal)
    {
        ret = _internal->valueLength();
    }
    return ret;
}

BLEDescriptor::operator bool() const
{
    return (strlen(_uuid_cstr) > 3);
}

unsigned char BLEDescriptor::properties() const
{
    return _properties;
}


int BLEDescriptor::valueSize() const
{
    int ret = _value_size;
    if (NULL != _internal)
    {
        ret = _internal->valueSize();
    }
    return ret;
}

bool BLEDescriptor::read()
{
    bool retVar = false;
    
    if (NULL != _internal)
    {
        retVar = _internal->read();
    }
    return retVar;
}

