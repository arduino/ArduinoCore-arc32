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

#include "BleCharacteristic.h"
#include "internal/ble_client.h"

#define BLE_CCCD_NOTIFY_EN_MASK   0x1
#define BLE_CCCD_INDICATE_EN_MASK 0x2

BleCharacteristic::BleCharacteristic(const char* uuid,
                      const uint8_t properties,
                      const uint16_t maxLength) :
    BleAttribute(uuid, BleTypeCharacteristic),
    _properties(properties),
    _data_len(0),
    _written(false),
    _cccd_value(0),
    _value_handle(0),
    _cccd_handle(0),
    _user_description(NULL),
    _presentation_format(NULL)
{
    _max_len = maxLength > BLE_MAX_ATTR_DATA_LEN ? BLE_MAX_ATTR_DATA_LEN : maxLength;

    memset(_event_handlers, 0, sizeof(_event_handlers));
}

BleCharacteristic::BleCharacteristic(const char* uuid,
                      const uint8_t properties,
                      const char* value) :
    BleCharacteristic(uuid, properties, strlen(value))
{
    setValue((const uint8_t*)value, strlen(value));
}

uint8_t
BleCharacteristic::properties() const
{
    return _properties;
}

BleStatus
BleCharacteristic::setValue(const uint8_t value[], uint16_t length)
{
    BleStatus status;

     _setValue(value, length);

    if (_value_handle) {
        status = ble_client_gatts_set_attribute_value(_value_handle, _data_len, _data, 0);
        if (BLE_STATUS_SUCCESS != status) {
            return status;
        }

        if (subscribed()) {
            boolean_t indication = (_cccd_value & BLE_CCCD_INDICATE_EN_MASK);

            status = ble_client_gatts_send_notif_ind(_value_handle, _data_len, _data, 0, indication);
            if (BLE_STATUS_SUCCESS != status) {
                return status;
            }
        }
    }

    return BLE_STATUS_SUCCESS;
}

void
BleCharacteristic::setValue(BleCentral& central, const uint8_t* value, uint16_t length)
{
    _setValue(value, length);

    _written = true;

    if (_event_handlers[BleWritten]) {
        _event_handlers[BleWritten](central, *this);
    }
}

uint16_t
BleCharacteristic::valueSize() const
{
    return _max_len;
}

const uint8_t*
BleCharacteristic::value() const
{
    return _data;
}

uint16_t
BleCharacteristic::valueLength() const
{
    return _data_len;
}

uint8_t
BleCharacteristic::operator[] (int offset) const
{
    return _data[offset];
}

boolean_t
BleCharacteristic::written()
{
    boolean_t written = _written;

    _written = false;

    return written;
}

boolean_t
BleCharacteristic::subscribed()
{
    return (_cccd_value & (BLE_CCCD_NOTIFY_EN_MASK | BLE_CCCD_INDICATE_EN_MASK));
}

void
BleCharacteristic::setEventHandler(BleCharacteristicEvent event, BleCharacteristicEventHandler callback)
{
    noInterrupts();
    if (event < sizeof(_event_handlers)) {
        _event_handlers[event] = callback;
    }
    interrupts();
}

BleStatus
BleCharacteristic::add(uint16_t serviceHandle)
{
    bt_uuid uuid = btUuid();

    struct ble_gatts_characteristic char_data;
    struct ble_gatts_char_handles handles;
    struct ble_gatt_char_user_desc user_desc;
    struct ble_gatt_pf_desc pf_desc;

    memset(&char_data, 0, sizeof(char_data));

    char_data.p_uuid = &uuid;
    char_data.props.props = _properties;

    if (_properties & (BleRead | BleNotify | BleIndicate)) {
        char_data.perms.rd = GAP_SEC_MODE_1 | GAP_SEC_LEVEL_1;
    } else {
        char_data.perms.rd = GAP_SEC_NO_PERMISSION;
    }

    if (_properties & (BleWriteWithoutResponse | BleWrite)) {
        char_data.perms.wr = GAP_SEC_MODE_1 | GAP_SEC_LEVEL_1;
    } else {
        char_data.perms.rd = GAP_SEC_NO_PERMISSION;
    }

    char_data.init_len = _data_len;
    char_data.max_len = _max_len;
    char_data.p_value = _data;

    if (_user_description) {
        user_desc.buffer = (uint8_t*)_user_description->value();
        user_desc.len = _user_description->valueLength();

        char_data.p_user_desc = &user_desc;
    }

    if (_presentation_format) {
        memcpy(&pf_desc, _presentation_format->value(), sizeof(pf_desc));

        char_data.p_char_pf_desc = &pf_desc;
    }

    BleStatus status = ble_client_gatts_add_characteristic(serviceHandle, &char_data, &handles);
    if (BLE_STATUS_SUCCESS == status) {
        _value_handle = handles.value_handle;
        _cccd_handle = handles.cccd_handle;
    }

    return status;
}

uint16_t
BleCharacteristic::valueHandle()
{
    return _value_handle;
}

uint16_t
BleCharacteristic::cccdHandle()
{
    return _cccd_handle;
}

void
BleCharacteristic::setCccdValue(BleCentral& central, uint16_t value)
{
    if (_cccd_value != value) {
        _cccd_value = value;

        if (subscribed()) {
            if (_event_handlers[BleSubscribed]) {
                _event_handlers[BleSubscribed](central, *this);
            }
        } else {
            if (_event_handlers[BleUnsubscribed]) {
                _event_handlers[BleUnsubscribed](central, *this);
            }
        }
    }
}

void
BleCharacteristic::setUserDescription(BleDescriptor *descriptor)
{
    _user_description = descriptor;
}

void BleCharacteristic::setPresentationFormat(BleDescriptor *descriptor)
{
    _presentation_format = descriptor;
}

void
BleCharacteristic::_setValue(const uint8_t value[], uint16_t length)
{
    if (length > _max_len) {
        length = _max_len;
    }

    memcpy(_data, value, length);
    _data_len = length;
}
