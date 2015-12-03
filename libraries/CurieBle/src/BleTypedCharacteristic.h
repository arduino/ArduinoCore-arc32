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

#ifndef _BLE_TYPED_CHARACTERISTIC_H_INCLUDED
#define _BLE_TYPED_CHARACTERISTIC_H_INCLUDED

#include "Arduino.h"

#include "BleCharacteristic.h"

template<typename T> class BleTypedCharacteristic : public BleCharacteristic
{
public:
    BleTypedCharacteristic(const char* uuid, unsigned char properties);

    BleStatus setValue(T value);
    T value();

    BleStatus setValueLE(T value);
    T valueLE();

    BleStatus setValueBE(T value);
    T valueBE();

private:
    T byteSwap(T value);
};

template<typename T> BleTypedCharacteristic<T>::BleTypedCharacteristic(const char* uuid, unsigned char properties) :
  BleCharacteristic(uuid, properties, sizeof(T))
{
    T value;
    memset(&value, 0x00, sizeof(value));

    setValue(value);
}

template<typename T> BleStatus BleTypedCharacteristic<T>::setValue(T value) {
    return BleCharacteristic::setValue((unsigned char*)&value, sizeof(T));
}

template<typename T> T BleTypedCharacteristic<T>::value() {
    T value;

    memcpy(&value, (unsigned char*)BleCharacteristic::value(), BleCharacteristic::valueSize());

    return value;
}

template<typename T> BleStatus BleTypedCharacteristic<T>::setValueLE(T value) {
    return setValue(value);
}

template<typename T> T BleTypedCharacteristic<T>::valueLE() {
    return value();
}

template<typename T> BleStatus BleTypedCharacteristic<T>::setValueBE(T value) {
    return setValue(byteSwap(value));
}

template<typename T> T BleTypedCharacteristic<T>::valueBE() {
    return byteSwap(value());
}

template<typename T> T BleTypedCharacteristic<T>::byteSwap(T value) {
    T result;
    unsigned char* src = (unsigned char*)&value;
    unsigned char* dst = (unsigned char*)&result;

    for (int i = 0; i < sizeof(T); i++) {
        dst[i] = src[sizeof(T) - i - 1];
    }

    return result;
}

#endif // _BLE_TYPED_CHARACTERISTIC_H_INCLUDED
