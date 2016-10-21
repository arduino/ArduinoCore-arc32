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

#ifndef _BLE_DESCRIPTORIMP_H_INCLUDED
#define _BLE_DESCRIPTORIMP_H_INCLUDED

#include "ArduinoBLE.h"

/**
 * BLE GATT Descriptor class
 */
class BLEDescriptorImp: public BLEAttribute{
public:
    /**
     * Constructor for BLE Descriptor
     *
     * @param[in] uuid        16-bit UUID (in string form) defined by BLE standard
     * @param[in] value       Value of descriptor, as a byte array.  Data is stored in internal copy.
     * @param[in] valueLength Data length required for descriptor value (<= BLE_MAX_ATTR_DATA_LEN)
     */
    BLEDescriptorImp(BLEDevice& bledevice, BLEDescriptor &descriptor);
    BLEDescriptorImp(const bt_uuid_t* uuid, 
                     uint16_t handle,
                     BLEDevice& bledevice);

    virtual ~BLEDescriptorImp();

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
    
    int updateProfile(bt_gatt_attr_t *attr_start, int& index);

    unsigned char operator[] (int offset) const;

protected:


private:
    unsigned short _value_length;
    unsigned short _value_handle;
    unsigned char* _value;
    
    bt_uuid_128 _descriptor_uuid;
};

#endif // _BLE_DESCRIPTOR_H_INCLUDED
