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
#include "BLEAttribute.h"
#include "BLEDescriptor.h"
#include "BLEUtils.h"
#include "BLEDescriptorImp.h"

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
    _value(NULL)
{
    _properties = descriptorImp->properties();
    memset(_uuid_cstr, 0, sizeof (_uuid_cstr));
    BLEUtils::uuidBT2String(descriptorImp->bt_uuid(), _uuid_cstr);
    
    _value_size = descriptorImp->valueSize();
    _value = (unsigned char*)balloc(_value_size, NULL);
    memcpy(_value, descriptorImp->value(), _value_size);
}

BLEDescriptor::BLEDescriptor(const char* uuid, 
                             const unsigned char value[], 
                             unsigned short valueLength):
    _bledev()
{
    bt_uuid_128_t uuid_tmp;
    memset(_uuid_cstr, 0, sizeof (_uuid_cstr));
    BLEUtils::uuidString2BT(uuid, (bt_uuid_t *)&uuid_tmp);
    BLEUtils::uuidBT2String((const bt_uuid_t *)&uuid_tmp, _uuid_cstr);
    
    _bledev.setAddress(*BLEUtils::bleGetLoalAddress());
    
    _value_size = valueLength > BLE_MAX_ATTR_LONGDATA_LEN ? BLE_MAX_ATTR_LONGDATA_LEN : valueLength;
    _value = (unsigned char*)balloc(_value_size, NULL);
    memcpy(_value, value, _value_size);
}

BLEDescriptor::BLEDescriptor(const char* uuid, 
                             const char* value):
     BLEDescriptor(uuid, (const unsigned char*)value, strlen(value))
{}

BLEDescriptor::~BLEDescriptor()
{
    if (_value)
    {
        bfree(_value);
        _value = NULL;
    }
}

const char* BLEDescriptor::uuid() const
{
    return _uuid_cstr;
}

const byte* BLEDescriptor::value() const
{
    // TODO: Not support now
    return _value;
}

int BLEDescriptor::valueLength() const
{
    // TODO: Not support now
    return _value_size;
}

byte BLEDescriptor::operator[] (int offset) const
{
    // TODO: Not support now
    return 0;
}

BLEDescriptor::operator bool() const
{
    // TODO: Not support now
    return false;
}

bool BLEDescriptor::writeValue(const byte value[], int length)
{
    // TODO: Not support now
    return false;
}

bool BLEDescriptor::writeValue(const byte value[], int length, int offset)
{
    // TODO: Not support now
    return false;
}

bool BLEDescriptor::writeValue(const char* value)
{
    // TODO: Not support now
    return false;
}

// GATT client Write the value of the descriptor
bool BLEDescriptor::write(const byte value[], int length)
{
    // TODO: Not support now
    return false;
}

bool BLEDescriptor::write(const byte value[], int length, int offset)
{
    // TODO: Not support now
    return false;
}

bool BLEDescriptor::write(const char* value)
{
    // TODO: Not support now
    return false;
}

bool BLEDescriptor::read()
{
    // TODO: Not support now
    return false;
}

unsigned char BLEDescriptor::properties() const
{
    return _properties;
}


int BLEDescriptor::valueSize() const
{
    return _value_size;
}

