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

BleDescriptor::BleDescriptor(const char* uuid, 
                             const uint16_t maxLength,
                             const BleClientAccessMode clientAccess)
    : BleAttribute(uuid, BleTypeDescriptor)
{
    _initialised = false;

    _desc.p_uuid = &_uuid;

    memset(_data, 0, maxLength);
    _desc.length = maxLength;
    _desc.p_value = _data;

    if ((clientAccess == BLE_CLIENT_ACCESS_READ_ONLY) ||
        (clientAccess == BLE_CLIENT_ACCESS_READ_WRITE))
        _desc.perms.rd = GAP_SEC_MODE_1 | GAP_SEC_LEVEL_1;
    else
        _desc.perms.rd = GAP_SEC_NO_PERMISSION;

    if ((clientAccess == BLE_CLIENT_ACCESS_WRITE_ONLY) ||
        (clientAccess == BLE_CLIENT_ACCESS_READ_WRITE))
        _desc.perms.wr = GAP_SEC_MODE_1 | GAP_SEC_LEVEL_1;
    else
        _desc.perms.wr = GAP_SEC_NO_PERMISSION;
}

BleDescriptor::BleDescriptor(const char* uuid, const uint8_t value[], uint16_t valueLength) :
    BleDescriptor(uuid, valueLength, BLE_CLIENT_ACCESS_READ_ONLY)
{
    setValue(value, valueLength);
}

BleDescriptor::BleDescriptor(const char* uuid, const char* value) :
    BleDescriptor(uuid, (const uint8_t*)value, strlen(value))
{
}

uint16_t
BleDescriptor::valueSize() const
{
    return _desc.length;
}

const uint8_t*
BleDescriptor::BleDescriptor::value() const
{
    return _desc.p_value;
}

uint16_t
BleDescriptor::valueLength() const
{
    return _desc.length;
}

uint8_t
BleDescriptor::operator[] (int offset) const
{
    return _desc.p_value[offset];
}

BleStatus
BleDescriptor::_setValue(void)
{
    if (!_initialised)
        return BLE_STATUS_WRONG_STATE;

    if (_desc.length > BLE_MAX_ATTR_DATA_LEN)
        return BLE_STATUS_NOT_ALLOWED;

    return ble_client_gatts_set_attribute_value(_handle, _desc.length, _data, 0);
}

BleStatus
BleDescriptor::setValue(const uint8_t value[],
                        const uint16_t length)
{
    if (!value)
        return BLE_STATUS_NOT_ALLOWED;
    if (length > BLE_MAX_ATTR_DATA_LEN)
        return BLE_STATUS_NOT_ALLOWED;

    /* Cache the value locally */
    memcpy(_data, value, length);
    _desc.length = length;

    return _setValue();
}

void
BleDescriptor::setEventCallback(BleDescriptorEventCb callback,
                                void *arg)
{
    noInterrupts();
    _event_cb = callback; /* callback disabled if NULL */
    _event_cb_arg = arg;
    interrupts();
}
