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

#include "BLETypedCharacteristic.h"

class BLEBoolCharacteristic : public BLETypedCharacteristic<bool> {
public:
    /**
     * @brief   Instantiate a bool Typed Characteristic. 
     *           Default constructor for BLE bool Characteristic
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
    BLEBoolCharacteristic(const char* uuid, unsigned char properties);
};

class BLEByteCharacteristic : public BLETypedCharacteristic<byte> {
public:
    /**
     * @brief   Instantiate a Byte Typed Characteristic. 
     *           Default constructor for BLE Byte Characteristic
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
    BLEByteCharacteristic(const char* uuid, unsigned char properties);
};

class BLECharCharacteristic : public BLETypedCharacteristic<char> {
public:
    /**
     * @brief   Instantiate a Char Typed Characteristic. 
     *           Default constructor for BLE Char Characteristic
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
    BLECharCharacteristic(const char* uuid, unsigned char properties);
};

class BLEUnsignedCharCharacteristic : public BLETypedCharacteristic<unsigned char> {
public:
    /**
     * @brief   Instantiate a Unsigned Char Typed Characteristic. 
     *           Default constructor for BLE Unsigned Char Characteristic
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
    BLEUnsignedCharCharacteristic(const char* uuid, unsigned char properties);
};

class BLEShortCharacteristic : public BLETypedCharacteristic<short> {
public:
    /**
     * @brief   Instantiate a Short Typed Characteristic. 
     *           Default constructor for BLE short Characteristic
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
    BLEShortCharacteristic(const char* uuid, unsigned char properties);
};

class BLEUnsignedShortCharacteristic : public BLETypedCharacteristic<unsigned short> {
public:
    /**
     * @brief   Instantiate a Unsigned Short Typed Characteristic. 
     *           Default constructor for BLE Unsigned Short Characteristic
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
    BLEUnsignedShortCharacteristic(const char* uuid, unsigned char properties);
};

class BLEIntCharacteristic : public BLETypedCharacteristic<int> {
public:
    /**
     * @brief   Instantiate a Int Typed Characteristic. 
     *           Default constructor for BLE Int Characteristic
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
    BLEIntCharacteristic(const char* uuid, unsigned char properties);
};

class BLEUnsignedIntCharacteristic : public BLETypedCharacteristic<unsigned int> {
public:
    /**
     * @brief   Instantiate a Unsigned Int Typed Characteristic. 
     *           Default constructor for BLE Unsigned Int Characteristic
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
    BLEUnsignedIntCharacteristic(const char* uuid, unsigned char properties);
};

class BLELongCharacteristic : public BLETypedCharacteristic<long> {
public:
    /**
     * @brief   Instantiate a Long Typed Characteristic. 
     *           Default constructor for BLE Long Characteristic
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
    BLELongCharacteristic(const char* uuid, unsigned char properties);
};

class BLEUnsignedLongCharacteristic : public BLETypedCharacteristic<unsigned long> {
public:
    /**
     * @brief   Instantiate a Unsigned Long Typed Characteristic. 
     *           Default constructor for BLE Unsigned Long Characteristic
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
    BLEUnsignedLongCharacteristic(const char* uuid, unsigned char properties);
};

class BLEFloatCharacteristic : public BLETypedCharacteristic<float> {
public:
    /**
     * @brief   Instantiate a Float Typed Characteristic. 
     *           Default constructor for BLE Float Characteristic
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
    BLEFloatCharacteristic(const char* uuid, unsigned char properties);
};

class BLEDoubleCharacteristic : public BLETypedCharacteristic<double> {
public:
    /**
     * @brief   Instantiate a Double Typed Characteristic. 
     *           Default constructor for BLE Double Characteristic
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
    BLEDoubleCharacteristic(const char* uuid, unsigned char properties);
};

#endif // _BLE_TYPED_CHARACTERISTICS_H_INCLUDED
