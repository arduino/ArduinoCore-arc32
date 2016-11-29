/*
  BLE Attribute with value API
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
#include "Arduino.h"
#include "BLEAttributeWithValue.h"

BLEAttributeWithValue::BLEAttributeWithValue()
{

}

    // intepret the value of the attribute with the specified type
String BLEAttributeWithValue::stringValue() const
{
    const char *retTemp = (const char *)this->value();
    return retTemp;
}

char BLEAttributeWithValue::charValue() const
{
    char ret = this->operator[](0);
    return ret;
}

unsigned char BLEAttributeWithValue::unsignedCharValue() const
{
    unsigned char ret = this->operator[](0);
    return ret;
}

byte BLEAttributeWithValue::byteValue() const
{
    return this->operator[](0);
}

short BLEAttributeWithValue::shortValue() const
{
    short retTmp = 0;
    memcpy(&retTmp, this->value(), sizeof(retTmp));
    return retTmp;
}

unsigned short BLEAttributeWithValue::unsignedShortValue() const
{
    unsigned short retTmp = 0;
    memcpy(&retTmp, this->value(), sizeof(retTmp));
    return retTmp;
}

int BLEAttributeWithValue::intValue() const
{
    int retTmp = 0;
    memcpy(&retTmp, this->value(), sizeof(retTmp));
    return retTmp;
}

unsigned int BLEAttributeWithValue::unsignedIntValue() const
{
    unsigned int retTmp = 0;
    memcpy(&retTmp, this->value(), sizeof(retTmp));
    return retTmp;
}

long BLEAttributeWithValue::longValue() const
{
    long retTmp = 0;
    memcpy(&retTmp, this->value(), sizeof(retTmp));
    return retTmp;
}

unsigned long BLEAttributeWithValue::unsignedLongValue() const
{
    unsigned long retTmp = 0;
    memcpy(&retTmp, this->value(), sizeof(retTmp));
    return retTmp;
}

float BLEAttributeWithValue::floatValue() const
{
    float retTmp = 0;
    memcpy(&retTmp, this->value(), sizeof(retTmp));
    return retTmp;
}

double BLEAttributeWithValue::doubleValue() const
{
    double retTmp = 0;
    memcpy(&retTmp, this->value(), sizeof(retTmp));
    return retTmp;
}

// write the value of the attribute with the specified type
bool BLEAttributeWithValue::writeString(const String& s)
{
    if (s.length() > (unsigned int)this->valueSize())
    {
        return false;
    }
    return this->writeValue((const byte*)s.c_str(), s.length());
}

bool BLEAttributeWithValue::writeString(const char* s)
{
    if (strlen(s) > (unsigned int)this->valueSize())
    {
        return false;
    }
    return this->writeValue((const byte*)s, strlen(s));
}

bool BLEAttributeWithValue::writeChar(char c)
{
    return this->writeValue((const byte*)&c, sizeof(c));
}

bool BLEAttributeWithValue::writeUnsignedChar(unsigned char c)
{
    return this->writeValue((const byte*)&c, sizeof(c));
}

bool BLEAttributeWithValue::writeByte(byte b)
{
    return this->writeValue((const byte*)&b, sizeof(b));
}

bool BLEAttributeWithValue::writeShort(short s)
{
    return this->writeValue((const byte*)&s, sizeof(s));
}

bool BLEAttributeWithValue::writeUnsignedShort(unsigned short s)
{
    return this->writeValue((const byte*)&s, sizeof(s));
}

bool BLEAttributeWithValue::writeInt(int i)
{
    return this->writeValue((const byte*)&i, sizeof(i));
}

bool BLEAttributeWithValue::writeUnsignedInt(unsigned int i)
{
    return this->writeValue((const byte*)&i, sizeof(i));
}

bool BLEAttributeWithValue::writeLong(long l)
{
    return this->writeValue((const byte*)&l, sizeof(l));
}

bool BLEAttributeWithValue::writeUnsignedLong(unsigned int l)
{
    return this->writeValue((const byte*)&l, sizeof(l));
}

bool BLEAttributeWithValue::writeFloat(float f)
{
    return this->writeValue((const byte*)&f, sizeof(f));
}

bool BLEAttributeWithValue::writeDouble(double d)
{
    return this->writeValue((const byte*)&d, sizeof(d));
}


