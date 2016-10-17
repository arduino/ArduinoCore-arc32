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

#include "CurieBLE.h"

#include "BLECharacteristic.h"

template<typename T> class BLETypedCharacteristic : public BLECharacteristic
{
public:
    /**
     * @brief   The constructor of the template BLE Characteristic
     *
     * @param[in]   uuid        The characteristic UUID 16/128 bits
     *
     * @param[in]   properties  The property of the characteristic (BLERead, 
     *                           BLEWrite or BLE Notify. Combine with | )
     *
     * @return  none
     *
     * @note  none
     */
    BLETypedCharacteristic(const char* uuid, unsigned char properties);

    /**
     * @brief   Set the characteristic value
     *
     * @param[in]   value   New value to set
     *
     * @return      bool    true - set value success, 
     *                      false - on error
     *
     * @note  none
     */
    bool setValue(T value);
    
    /**
     * @brief   Update the characteristic value
     *
     * @param[in]   value   New value to set
     *
     * @return      bool    true - set value success, 
     *                      false - on error
     *
     * @note  none
     */
    bool writeValue(T value);
    
    /**
     * @brief   Get the value of the Characteristic
     *
     * @param   none
     *
     * @return  T       The value of characteristic
     *
     * @note  none
     */
    T value(void);

    /**
     * @brief   Set the characteristic value in Little Endian
     *
     * @param[in]   value   New value to set
     *
     * @return      bool    true - set value success, 
     *                      false - on error
     *
     * @note  none
     */
    bool setValueLE(T value);
    /**
     * @brief   Get the value of the Characteristic in Little Endian
     *
     * @param   none
     *
     * @return  T       The value of characteristic
     *
     * @note  none
     */
    T valueLE(void);

    /**
     * @brief   Set the characteristic value in Big Endian
     *
     * @param[in]   value   New value to set
     *
     * @return      bool    true - set value success, 
     *                      false - on error
     *
     * @note  none
     */
    bool setValueBE(T value);
    /**
     * @brief   Get the value of the Characteristic in Big Endian
     *
     * @param   none
     *
     * @return  T       The value of characteristic
     *
     * @note  none
     */
    T valueBE(void);

private:
    /**
     * @brief   Swap the bytes
     *
     * @param   value   The typed value
     *
     * @return  T       The swapped value
     *
     * @note  none
     */
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

template<typename T> bool BLETypedCharacteristic<T>::writeValue(T value) {
    return BLECharacteristic::writeValue((unsigned char*)&value, sizeof(T));
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
