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

#ifndef ARDUINO_BLE_ATTRIBUTE_WITH_VALUE__H
#define ARDUINO_BLE_ATTRIBUTE_WITH_VALUE__H

class BLEAttributeWithValue
{
  public:
    BLEAttributeWithValue();

    virtual int valueSize() const = 0; // returns the length of the attribute value
    virtual const byte* value() const = 0; // returns the value of the attribute array
    virtual byte operator[] (int offset) const = 0; // access an attribute value at the specified offset
    virtual bool writeValue(const byte value[], int length) = 0;

    // intepret the value of the attribute with the specified type
    String stringValue() const;
    char charValue() const;
    unsigned char unsignedCharValue() const;
    byte byteValue() const;
    short shortValue() const;
    unsigned short unsignedShortValue() const;
    int intValue() const;
    unsigned int unsignedIntValue() const;
    long longValue() const;
    unsigned long unsignedLongValue() const;
    float floatValue() const;
    double doubleValue() const;

    // write the value of the attribute with the specified type
    bool writeString(const String& s);
    bool writeString(const char* s);
    bool writeChar(char c);
    bool writeUnsignedChar(unsigned char c);
    bool writeByte(byte b);
    bool writeShort(short s);
    bool writeUnsignedShort(unsigned short s);
    bool writeInt(int i);
    bool writeUnsignedInt(unsigned int i);
    bool writeLong(long l);
    bool writeUnsignedLong(unsigned int l);
    bool writeFloat(float f);
    bool writeDouble(double d);
};

#endif
