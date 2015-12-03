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

#include "BLECharacteristic.h"

template<typename T> class BLETypedCharacteristic : public BLECharacteristic
{
public:
    BLETypedCharacteristic(const char* uuid, unsigned char properties);

    bool setValue(T value);
    T value(void);

    bool setValueLE(T value);
    T valueLE(void);

    bool setValueBE(T value);
    T valueBE(void);

private:
    T byteSwap(T value);
};

template<typename T> BLETypedCharacteristic<T>::BLETypedCharacteristic(const char* uuid, unsigned char properties) :
  BLECharacteristic(uuid, properties, sizeof(T))
{
    T value;
    memset(&value, 0x00, sizeof(value));

    setValue(value);
}

template<typename T> bool BLETypedCharacteristic<T>::setValue(T value) {
    return BLECharacteristic::setValue((unsigned char*)&value, sizeof(T));
}

template<typename T> T BLETypedCharacteristic<T>::value() {
    T value;

    memcpy(&value, (unsigned char*)BLECharacteristic::value(), BLECharacteristic::valueSize());

    return value;
}

template<typename T> bool BLETypedCharacteristic<T>::setValueLE(T value) {
    return setValue(value);
}

template<typename T> T BLETypedCharacteristic<T>::valueLE() {
    return value();
}

template<typename T> bool BLETypedCharacteristic<T>::setValueBE(T value) {
    return setValue(byteSwap(value));
}

template<typename T> T BLETypedCharacteristic<T>::valueBE() {
    return byteSwap(value());
}

template<typename T> T BLETypedCharacteristic<T>::byteSwap(T value) {
    T result;
    unsigned char* src = (unsigned char*)&value;
    unsigned char* dst = (unsigned char*)&result;

    for (int i = 0; i < sizeof(T); i++) {
        dst[i] = src[sizeof(T) - i - 1];
    }

    return result;
}

#endif // _BLE_TYPED_CHARACTERISTIC_H_INCLUDED
