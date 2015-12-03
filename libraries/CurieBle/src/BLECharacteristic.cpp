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

#include "BLECharacteristic.h"
#include "internal/ble_client.h"

#define BLE_CCCD_NOTIFY_EN_MASK   0x1
#define BLE_CCCD_INDICATE_EN_MASK 0x2

BLECharacteristic::BLECharacteristic(const char* uuid,
                      const unsigned char properties,
                      const unsigned short maxLength) :
    BLEAttribute(uuid, BLETypeCharacteristic),
    _properties(properties),
    _value_length(0),
    _written(false),
    _cccd_value(0),
    _value_handle(0),
    _cccd_handle(0),
    _user_description(NULL),
    _presentation_format(NULL)
{
    _value_size = maxLength > BLE_MAX_ATTR_DATA_LEN ? BLE_MAX_ATTR_DATA_LEN : maxLength;
    _value = (unsigned char*)malloc(_value_length);

    memset(_event_handlers, 0, sizeof(_event_handlers));
}

BLECharacteristic::BLECharacteristic(const char* uuid,
                      const unsigned char properties,
                      const char* value) :
    BLECharacteristic(uuid, properties, strlen(value))
{
    setValue((const uint8_t*)value, strlen(value));
}

BLECharacteristic::~BLECharacteristic()
{
    if (_value) {
        free(_value);
        _value = NULL;
    }
}

unsigned char
BLECharacteristic::properties() const
{
    return _properties;
}

bool
BLECharacteristic::setValue(const unsigned char value[], uint16_t length)
{
    BleStatus status;

     _setValue(value, length);

    if (_value_handle) {
        status = ble_client_gatts_set_attribute_value(_value_handle, _value_length, _value, 0);
        if (BLE_STATUS_SUCCESS != status) {
            return false;
        }

        if (subscribed()) {
            boolean_t indication = (_cccd_value & BLE_CCCD_INDICATE_EN_MASK);

            status = ble_client_gatts_send_notif_ind(_value_handle, _value_length, _value, 0, indication);
            if (BLE_STATUS_SUCCESS != status) {
                return false;
            }
        }
    }

    return true;
}

void
BLECharacteristic::setValue(BLECentral& central, const unsigned char* value, unsigned short length)
{
    _setValue(value, length);

    _written = true;

    if (_event_handlers[BLEWritten]) {
        _event_handlers[BLEWritten](central, *this);
    }
}

unsigned short
BLECharacteristic::valueSize() const
{
    return _value_size;
}

const unsigned char*
BLECharacteristic::value() const
{
    return _value;
}

unsigned short
BLECharacteristic::valueLength() const
{
    return _value_length;
}

unsigned char
BLECharacteristic::operator[] (int offset) const
{
    return _value[offset];
}

bool
BLECharacteristic::written()
{
    boolean_t written = _written;

    _written = false;

    return written;
}

bool
BLECharacteristic::subscribed()
{
    return (_cccd_value & (BLE_CCCD_NOTIFY_EN_MASK | BLE_CCCD_INDICATE_EN_MASK));
}

void
BLECharacteristic::setEventHandler(BLECharacteristicEvent event, BLECharacteristicEventHandler callback)
{
    noInterrupts();
    if (event < sizeof(_event_handlers)) {
        _event_handlers[event] = callback;
    }
    interrupts();
}

bool
BLECharacteristic::add(uint16_t serviceHandle)
{
    bt_uuid uuid = btUuid();

    struct ble_gatts_characteristic char_data;
    struct ble_gatts_char_handles handles;
    struct ble_gatt_char_user_desc user_desc;
    struct ble_gatt_pf_desc pf_desc;

    memset(&char_data, 0, sizeof(char_data));

    char_data.p_uuid = &uuid;
    char_data.props.props = _properties;

    if (_properties & (BLERead | BLENotify | BLEIndicate)) {
        char_data.perms.rd = GAP_SEC_MODE_1 | GAP_SEC_LEVEL_1;
    } else {
        char_data.perms.rd = GAP_SEC_NO_PERMISSION;
    }

    if (_properties & (BLEWriteWithoutResponse | BLEWrite)) {
        char_data.perms.wr = GAP_SEC_MODE_1 | GAP_SEC_LEVEL_1;
    } else {
        char_data.perms.wr = GAP_SEC_NO_PERMISSION;
    }

    char_data.init_len = _value_length;
    char_data.max_len = _value_size;
    char_data.p_value = _value;

    if (_user_description) {
        user_desc.buffer = (uint8_t*)_user_description->value();
        user_desc.len = _user_description->valueLength();

        char_data.p_user_desc = &user_desc;
    }

    if (_presentation_format) {
        const uint8_t* pfValue = _presentation_format->value();

        pf_desc.format = pfValue[0];
        pf_desc.exp = pfValue[1];
        pf_desc.unit = (pfValue[3] << 8) | pfValue[2];
        pf_desc.name_spc = pfValue[4];
        pf_desc.descr = (pfValue[6] << 8) | pfValue[5];

        char_data.p_char_pf_desc = &pf_desc;
    }

    BleStatus status = ble_client_gatts_add_characteristic(serviceHandle, &char_data, &handles);
    if (BLE_STATUS_SUCCESS == status) {
        _value_handle = handles.value_handle;
        _cccd_handle = handles.cccd_handle;
    }

    return (BLE_STATUS_SUCCESS == status);
}

uint16_t
BLECharacteristic::valueHandle()
{
    return _value_handle;
}

uint16_t
BLECharacteristic::cccdHandle()
{
    return _cccd_handle;
}

void
BLECharacteristic::setCccdValue(BLECentral& central, uint16_t value)
{
    if (_cccd_value != value) {
        _cccd_value = value;

        if (subscribed()) {
            if (_event_handlers[BLESubscribed]) {
                _event_handlers[BLESubscribed](central, *this);
            }
        } else {
            if (_event_handlers[BLEUnsubscribed]) {
                _event_handlers[BLEUnsubscribed](central, *this);
            }
        }
    }
}

void
BLECharacteristic::setUserDescription(BLEDescriptor *descriptor)
{
    _user_description = descriptor;
}

void
BLECharacteristic::setPresentationFormat(BLEDescriptor *descriptor)
{
    _presentation_format = descriptor;
}

void
BLECharacteristic::_setValue(const uint8_t value[], uint16_t length)
{
    if (length > _value_size) {
        length = _value_size;
    }

    memcpy(_value, value, length);
    _value_length = length;
}
