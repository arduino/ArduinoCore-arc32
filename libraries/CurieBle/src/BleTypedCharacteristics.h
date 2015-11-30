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

#ifndef _BLE_TYPED_CHARACTERISTICS_H_INCLUDED
#define _BLE_TYPED_CHARACTERISTICS_H_INCLUDED

#include "BleTypedCharacteristic.h"

class BleBoolCharacteristic : public BleTypedCharacteristic<bool> {
public:
    BleBoolCharacteristic(const char* uuid, unsigned char properties);
};

class BleCharCharacteristic : public BleTypedCharacteristic<char> {
public:
    BleCharCharacteristic(const char* uuid, unsigned char properties);
};

class BleUnsignedCharCharacteristic : public BleTypedCharacteristic<unsigned char> {
public:
    BleUnsignedCharCharacteristic(const char* uuid, unsigned char properties);
};

class BleShortCharacteristic : public BleTypedCharacteristic<short> {
public:
    BleShortCharacteristic(const char* uuid, unsigned char properties);
};

class BleUnsignedShortCharacteristic : public BleTypedCharacteristic<unsigned short> {
public:
    BleUnsignedShortCharacteristic(const char* uuid, unsigned char properties);
};

class BleIntCharacteristic : public BleTypedCharacteristic<int> {
public:
    BleIntCharacteristic(const char* uuid, unsigned char properties);
};

class BleUnsignedIntCharacteristic : public BleTypedCharacteristic<unsigned int> {
public:
    BleUnsignedIntCharacteristic(const char* uuid, unsigned char properties);
};

class BleLongCharacteristic : public BleTypedCharacteristic<long> {
public:
    BleLongCharacteristic(const char* uuid, unsigned char properties);
};

class BleUnsignedLongCharacteristic : public BleTypedCharacteristic<unsigned long> {
public:
    BleUnsignedLongCharacteristic(const char* uuid, unsigned char properties);
};

class BleFloatCharacteristic : public BleTypedCharacteristic<float> {
public:
    BleFloatCharacteristic(const char* uuid, unsigned char properties);
};

class BleDoubleCharacteristic : public BleTypedCharacteristic<double> {
public:
    BleDoubleCharacteristic(const char* uuid, unsigned char properties);
};

#endif // _BLE_TYPED_CHARACTERISTICS_H_INCLUDED
