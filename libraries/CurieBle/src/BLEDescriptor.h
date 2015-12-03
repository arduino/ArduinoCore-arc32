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

#ifndef _BLE_DESCRIPTOR_H_INCLUDED
#define _BLE_DESCRIPTOR_H_INCLUDED

#include "BLEAttribute.h"

/**
 * BLE GATT Descriptor class
 */
class BLEDescriptor : public BLEAttribute {
public:
    /**
     * Constructor for BLE Descriptor
     *
     * @param uuid        16-bit UUID (in string form) defined by BLE standard
     * @param value       Value of descriptor, as a byte array.  Data is stored in internal copy.
     * @param valueLength Data length required for descriptor value (<= BLE_MAX_ATTR_DATA_LEN)
     */
    BLEDescriptor(const char* uuid, const unsigned char value[], unsigned short valueLength);

    virtual ~BLEDescriptor();

    /**
     * Constructor for BLE Descriptor
     *
     * @param uuid        16-bit UUID (in string form) defined by BLE standard
     * @param value       String value of descriptor.  Data is stored in internal copy. 
     *                    (String length <= BLE_MAX_ATTR_DATA_LEN)
     */
    BLEDescriptor(const char* uuid, const char* value);

    /**
     * Get data pointer to the value of the Descriptor
     *
     * @return const unsigned char* pointer to the value of the Descriptor
     */
    const unsigned char* value(void) const;

    /**
     * Get the length of the value of the Descriptor
     *
     * @return unsigned short size of Descriptor value in bytes
     */
    unsigned short valueLength(void) const;


    unsigned char operator[] (int offset) const;

protected:
    bool add(uint16_t serviceHandle);

    friend BLEPeripheral;

private:
    unsigned short _value_length;
    unsigned char* _value;
};

#endif // _BLE_DESCRIPTOR_H_INCLUDED
