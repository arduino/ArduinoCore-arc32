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

#include "BleDescriptor.h"

#include "internal/ble_client.h"

BleDescriptor::BleDescriptor(const char* uuid, const uint8_t value[], uint16_t valueLength) :
    BleAttribute(uuid, BleTypeDescriptor)
{
    if (valueLength > BLE_MAX_ATTR_DATA_LEN) {
        valueLength = BLE_MAX_ATTR_DATA_LEN;
    }
    _data_len = valueLength;

    memcpy(_data, value, _data_len);
}

BleDescriptor::BleDescriptor(const char* uuid, const char* value) :
    BleDescriptor(uuid, (const uint8_t*)value, strlen(value))
{
}

const uint8_t*
BleDescriptor::BleDescriptor::value() const
{
    return _data;
}

uint16_t
BleDescriptor::valueLength() const
{
    return _data_len;
}

uint8_t
BleDescriptor::operator[] (int offset) const
{
    return _data[offset];
}

BleStatus
BleDescriptor::add(uint16_t serviceHandle)
{
    bt_uuid uuid = btUuid();
    struct ble_gatts_descriptor desc;
    uint16_t handle = 0;

    memset(&desc, 0, sizeof(desc));

    desc.p_uuid = &uuid;

    // this class only supports read-only descriptors
    desc.perms.rd = GAP_SEC_MODE_1 | GAP_SEC_LEVEL_1;
    desc.perms.wr = GAP_SEC_NO_PERMISSION;

    BleStatus status = ble_client_gatts_add_descriptor(serviceHandle, &desc, &handle);

    if (BLE_STATUS_SUCCESS == status) {
        setHandle(handle);
        status = ble_client_gatts_set_attribute_value(handle, _data_len, _data, 0);
    }

    return status; 
}
