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

void
BleDescriptor::_init(const uint16_t maxLength,
                     const BleClientAccessMode clientAccess)
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

BleDescriptor::BleDescriptor(const uint16_t uuid16,
                             const uint16_t maxLength,
                             const BleClientAccessMode clientAccess)
{
    _uuid.type = BT_UUID16;
    _uuid.uuid16 = uuid16;

    _init(maxLength, clientAccess);
}

BleDescriptor::BleDescriptor(const uint8_t uuid128[],
                             const uint16_t maxLength,
                             const BleClientAccessMode clientAccess)
{
    _uuid.type = BT_UUID128;
    memcpy(&_uuid.uuid128, uuid128, MAX_UUID_SIZE);

    _init(maxLength, clientAccess);
}

BleStatus
BleDescriptor::_setValue(void)
{
    if (!_initialised)
        return BLE_STATUS_WRONG_STATE;

    BleStatus status = ble_client_gatts_set_attribute_value(_handle, _desc.length, _data, 0);
    if (BLE_STATUS_SUCCESS != status)
        return status;

    return BLE_STATUS_SUCCESS;
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

BleStatus
BleDescriptor::setValue(const String &str)
{
    str.getBytes((unsigned char *)&_data, (unsigned int)BLE_MAX_ATTR_DATA_LEN, 0U);
    _desc.length = str.len + 1;
    return _setValue();
}

BleStatus
BleDescriptor::setValue(const char *cstr)
{
    return setValue((uint8_t *)cstr, (uint16_t) (strlen(cstr) + 1));
}

BleStatus
BleDescriptor::setValue(const char &value)
{
    uint8_t *p = _data;
    INT8_TO_LESTREAM(p, value);
    _desc.length = sizeof(value);
    return _setValue();
}

BleStatus
BleDescriptor::setValue(const unsigned char &value)
{
    uint8_t *p = _data;
    UINT8_TO_LESTREAM(p, value);
    _desc.length = sizeof(value);
    return _setValue();
}

BleStatus
BleDescriptor::setValue(const short &value)
{
    uint8_t *p = _data;
    INT16_TO_LESTREAM(p, value);
    _desc.length = sizeof(value);
    return _setValue();
}

BleStatus
BleDescriptor::setValue(const unsigned short &value)
{
    uint8_t *p = _data;
    UINT16_TO_LESTREAM(p, value);
    _desc.length = sizeof(value);
    return _setValue();
}

BleStatus
BleDescriptor::setValue(const int &value)
{
    uint8_t *p = _data;
    INT32_TO_LESTREAM(p, value);
    _desc.length = sizeof(value);
    return _setValue();
}

BleStatus
BleDescriptor::setValue(const unsigned int &value)
{
    uint8_t *p = _data;
    UINT32_TO_LESTREAM(p, value);
    _desc.length = sizeof(value);
    return _setValue();
}

BleStatus
BleDescriptor::setValue(const long &value)
{
    uint8_t *p = _data;
    INT32_TO_LESTREAM(p, value);
    _desc.length = sizeof(value);
    return _setValue();
}

BleStatus
BleDescriptor::setValue(const unsigned long &value)
{
    uint8_t *p = _data;
    UINT32_TO_LESTREAM(p, value);
    _desc.length = sizeof(value);
    return _setValue();
}

BleStatus
BleDescriptor::getValue(uint8_t value[], uint16_t &length)
{
    if (!_initialised)
        return BLE_STATUS_WRONG_STATE;

    memcpy(value, _data, _desc.length);
    length = _desc.length;

    return BLE_STATUS_SUCCESS;
}

BleStatus
BleDescriptor::getValue(String &str)
{
    str = (char *)_data;
    return BLE_STATUS_SUCCESS;
}

BleStatus
BleDescriptor::getValue(char *cstr)
{
    memcpy(cstr, _data, _desc.length);
    return BLE_STATUS_SUCCESS;
}

BleStatus
BleDescriptor::getValue(char &value)
{
    uint8_t *p = _data;
    LESTREAM_TO_INT8(p, value);
    return BLE_STATUS_SUCCESS;
}

BleStatus
BleDescriptor::getValue(unsigned char &value)
{
    uint8_t *p = _data;
    LESTREAM_TO_UINT8(p, value);
    return BLE_STATUS_SUCCESS;
}

BleStatus
BleDescriptor::getValue(short &value)
{
    uint8_t *p = _data;
    LESTREAM_TO_INT16(p, value);
    return BLE_STATUS_SUCCESS;
}

BleStatus
BleDescriptor::getValue(unsigned short &value)
{
    uint8_t *p = _data;
    LESTREAM_TO_UINT16(p, value);
    return BLE_STATUS_SUCCESS;
}

BleStatus
BleDescriptor::getValue(int &value)
{
    uint8_t *p = _data;
    LESTREAM_TO_INT32(p, value);
    return BLE_STATUS_SUCCESS;
}

BleStatus
BleDescriptor::getValue(unsigned int &value)
{
    uint8_t *p = _data;
    LESTREAM_TO_UINT32(p, value);
    return BLE_STATUS_SUCCESS;
}

BleStatus
BleDescriptor::getValue(long &value)
{
    uint8_t *p = _data;
    LESTREAM_TO_INT32(p, value);
    return BLE_STATUS_SUCCESS;
}

BleStatus
BleDescriptor::getValue(unsigned long &value)
{
    uint8_t *p = _data;
    LESTREAM_TO_UINT32(p, value);
    return BLE_STATUS_SUCCESS;
}

BleStatus
BleDescriptor::setEventCallback(BleDescriptorEventCb callback,
                                void *arg)
{
    _event_cb = callback;
    _event_cb_arg = arg;
    return BLE_STATUS_SUCCESS;
}
