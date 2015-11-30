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

#define BLE_CCCD_DESCRIPTOR_UUID "2092"

#define BLE_CCCD_NOTIFY_EN_MASK   0x1
#define BLE_CCCD_INDICATE_EN_MASK 0x2

/* Each characteristic has a Client Characteristic Configuration Descriptor (CCCD)
 * This callback is invoked whenever the CCCD for a given characteristic is updated
 * by a remote client to enable/disable receipt of notifications/indications
 */
void
_cccdEventHandler(BleDescriptor &cccd, BleDescriptorEvent event, void *arg)
{
    if (BLE_DESC_EVENT_WRITE == event) {
        BleStatus status;
        uint16_t cccdVal;
        BleCharacteristic *ch = (BleCharacteristic *)arg;

        status = cccd.getValue(cccdVal);
        if (BLE_STATUS_SUCCESS != status)
            return;

        ch->_notifyEnabled = (cccdVal & BLE_CCCD_NOTIFY_EN_MASK) ? true : false;
        ch->_indicateEnabled = (cccdVal & BLE_CCCD_INDICATE_EN_MASK) ? true : false;

        if (ch->_notifyEnabled || ch->_indicateEnabled) {
            if (ch->_event_handlers[BleSubscribed]) {
                ch->_event_handlers[BleSubscribed](*ch);
            }
        } else {
            if (ch->_event_handlers[BleUnsubscribed]) {
                ch->_event_handlers[BleUnsubscribed](*ch);
            }
        }
    }
}

BleCharacteristic::BleCharacteristic(const char* uuid,
                      const uint8_t properties,
                      const uint16_t maxLength) :
    BleAttribute(uuid, BleTypeCharacteristic),
    _cccd(BLE_CCCD_DESCRIPTOR_UUID, sizeof(uint16_t), BLE_CLIENT_ACCESS_READ_WRITE)
{
    _initialised = false;
    _connected = false;
    _notifyEnabled = false;
    _indicateEnabled = false;

    _data_len = 0;
    _data[0] = 0;

    _properties = properties;

    memset(&_char_data, 0, sizeof(_char_data));

    _char_data.p_uuid = &_uuid;
    _char_data.props.props = properties;

    if (properties & (BleRead | BleNotify | BleIndicate)) {
        _char_data.perms.rd = GAP_SEC_MODE_1 | GAP_SEC_LEVEL_1;
    } else {
        _char_data.perms.rd = GAP_SEC_NO_PERMISSION;
    }

    if (properties & (BleWriteWithoutResponse | BleWrite)) {
        _char_data.perms.wr = GAP_SEC_MODE_1 | GAP_SEC_LEVEL_1;
    } else {
        _char_data.perms.rd = GAP_SEC_NO_PERMISSION;
    }

    _char_data.init_len = _data_len;
    _char_data.max_len = maxLength > BLE_MAX_ATTR_DATA_LEN ? BLE_MAX_ATTR_DATA_LEN : maxLength;
    _char_data.p_value = _data;

    memset(_event_handlers, 0, sizeof(_event_handlers));
}

BleCharacteristic::BleCharacteristic(const char* uuid,
                      const uint8_t properties,
                      const char* value) :
    BleCharacteristic(uuid, properties, strlen(value))
{
    setValue(value);
}

BleStatus
BleCharacteristic::addUserDescription(const char *description)
{
    if (_initialised)
        return BLE_STATUS_WRONG_STATE;

    if (description && description[0]) {
        _user_desc.len = strlen(description);
        if (_user_desc.len > sizeof(_user_desc_data))
            _user_desc.len = sizeof(_user_desc_data);
        memset(_user_desc_data, 0, sizeof(_user_desc_data));
        memcpy(_user_desc_data, description, _user_desc.len);
        _user_desc.buffer = _user_desc_data;

        _char_data.p_user_desc = &_user_desc;
    }

    return BLE_STATUS_SUCCESS;
}

BleStatus
BleCharacteristic::addPresentationFormat(const uint8_t  format,
                                         const int8_t   exponent,
                                         const uint16_t unit,
                                         const uint8_t  nameSpace,
                                         const uint16_t description)
{
    if (_initialised)
        return BLE_STATUS_WRONG_STATE;

    _pf_desc.format = format;
    _pf_desc.exp = exponent;
    _pf_desc.unit = unit;
    _pf_desc.name_spc = nameSpace;
    _pf_desc.descr = description;

    _char_data.p_char_pf_desc = &_pf_desc;

    return BLE_STATUS_SUCCESS;
}

BleStatus
BleCharacteristic::_setValue(void)
{
    BleStatus status;

    if ((_data_len > BLE_MAX_ATTR_DATA_LEN) || (_data_len > _char_data.max_len))
        return BLE_STATUS_NOT_ALLOWED;

    if (_initialised) {
        status = ble_client_gatts_set_attribute_value(_handles.value_handle,
                                                      _data_len, _data, 0);
        if (BLE_STATUS_SUCCESS != status)
            return status;

        if (_notifyEnabled || _indicateEnabled) {
            status = ble_client_gatts_send_notif_ind(_handles.value_handle, _data_len, _data, 0, _indicateEnabled);
            if (BLE_STATUS_SUCCESS != status)
                return status;

            if (_indicateEnabled && _event_handlers[BleAcked])
                _event_handlers[BleAcked](*this);
        }
    }

    return BLE_STATUS_SUCCESS;
}

BleStatus
BleCharacteristic::setValue(const uint8_t value[], const uint16_t length)
{
    if (!value)
        return BLE_STATUS_NOT_ALLOWED;
    if (length > _char_data.max_len)
        return BLE_STATUS_NOT_ALLOWED;

    /* Cache the value locally */
    memcpy(_data, value, length);
    _data_len = length;

    return _setValue();
}

BleStatus
BleCharacteristic::setValue(const String &str)
{
    str.getBytes((unsigned char *)&_data, (unsigned int)_char_data.max_len, 0U);
    _data_len = str.length() + 1;
    return _setValue();
}

BleStatus
BleCharacteristic::setValue(const char *cstr)
{
    return setValue((uint8_t *)cstr, (uint16_t) (strlen(cstr)));
}

BleStatus
BleCharacteristic::setValue(const char &value)
{
    uint8_t *p = _data;
    INT8_TO_LESTREAM(p, value);
    _data_len = sizeof(value);
    return _setValue();
}

BleStatus
BleCharacteristic::setValue(const unsigned char &value)
{
    uint8_t *p = _data;
    UINT8_TO_LESTREAM(p, value);
    _data_len = sizeof(value);
    return _setValue();
}

BleStatus
BleCharacteristic::setValue(const short &value)
{
    uint8_t *p = _data;
    INT16_TO_LESTREAM(p, value);
    _data_len = sizeof(value);
    return _setValue();
}

BleStatus
BleCharacteristic::setValue(const unsigned short &value)
{
    uint8_t *p = _data;
    UINT16_TO_LESTREAM(p, value);
    _data_len = sizeof(value);
    return _setValue();
}

BleStatus
BleCharacteristic::setValue(const int &value)
{
    uint8_t *p = _data;
    INT32_TO_LESTREAM(p, value);
    _data_len = sizeof(value);
    return _setValue();
}

BleStatus
BleCharacteristic::setValue(const unsigned int &value)
{
    uint8_t *p = _data;
    UINT32_TO_LESTREAM(p, value);
    _data_len = sizeof(value);
    return _setValue();
}

BleStatus
BleCharacteristic::setValue(const long &value)
{
    uint8_t *p = _data;
    INT32_TO_LESTREAM(p, value);
    _data_len = sizeof(value);
    return _setValue();
}

BleStatus
BleCharacteristic::setValue(const unsigned long &value)
{
    uint8_t *p = _data;
    UINT32_TO_LESTREAM(p, value);
    _data_len = sizeof(value);
    return _setValue();
}

BleStatus
BleCharacteristic::getValue(uint8_t value[], uint16_t &length) const
{
    if (!_initialised)
        return BLE_STATUS_WRONG_STATE;

    memcpy(value, _data, _data_len);
    length = _data_len;

    return BLE_STATUS_SUCCESS;
}

BleStatus
BleCharacteristic::getValue(String &str) const
{
    str = (char *)_data;
    return BLE_STATUS_SUCCESS;
}

BleStatus
BleCharacteristic::getValue(char *cstr) const
{
    memcpy(cstr, _data, _data_len);
    return BLE_STATUS_SUCCESS;
}

BleStatus
BleCharacteristic::getValue(char &value) const
{
    const uint8_t *p = _data;
    LESTREAM_TO_INT8(p, value);
    return BLE_STATUS_SUCCESS;
}

BleStatus
BleCharacteristic::getValue(unsigned char &value) const
{
    const uint8_t *p = _data;
    LESTREAM_TO_UINT8(p, value);
    return BLE_STATUS_SUCCESS;
}

BleStatus
BleCharacteristic::getValue(short &value) const
{
    const uint8_t *p = _data;
    LESTREAM_TO_INT16(p, value);
    return BLE_STATUS_SUCCESS;
}

BleStatus
BleCharacteristic::getValue(unsigned short &value) const
{
    const uint8_t *p = _data;
    LESTREAM_TO_UINT16(p, value);
    return BLE_STATUS_SUCCESS;
}

BleStatus
BleCharacteristic::getValue(int &value) const
{
    const uint8_t *p = _data;
    LESTREAM_TO_INT32(p, value);
    return BLE_STATUS_SUCCESS;
}

BleStatus
BleCharacteristic::getValue(unsigned int &value) const
{
    const uint8_t *p = _data;
    LESTREAM_TO_UINT32(p, value);
    return BLE_STATUS_SUCCESS;
}

BleStatus
BleCharacteristic::getValue(long &value) const
{
    const uint8_t *p = _data;
    LESTREAM_TO_INT32(p, value);
    return BLE_STATUS_SUCCESS;
}

BleStatus
BleCharacteristic::getValue(unsigned long &value) const
{
    const uint8_t *p = _data;
    LESTREAM_TO_UINT32(p, value);
    return BLE_STATUS_SUCCESS;
}

uint16_t
BleCharacteristic::valueSize() const
{
    return _char_data.max_len;
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
BleCharacteristic::addDescriptor(BleDescriptor &descriptor)
{
    BleStatus status;

    if (!_initialised)
        return BLE_STATUS_WRONG_STATE;
    if (_num_descriptors >= BLE_MAX_DESCRIPTORS)
        return BLE_STATUS_ERROR;

    /* If this service has a 128-bit UUID, it shall be inherited
     * by included services, characteristics, and descriptors
     */
    if ((BT_UUID128 == _uuid.type) && (BT_UUID16 == descriptor._uuid.type))
        BLE_UUID16_TO_UUID128(descriptor._uuid, _uuid);

    status = ble_client_gatts_add_descriptor(_svc_handle,
                                             &descriptor._desc,
                                             &descriptor._handle);
    if (BLE_STATUS_SUCCESS == status) {
        descriptor._initialised = true;
        _descriptors[_num_descriptors++] = &descriptor;
    }

    return status;
}


BleDescriptor *
BleCharacteristic::_matchDescriptor(uint16_t handle) const
{
    for (unsigned i = 0; i < _num_descriptors; i++) {
        BleDescriptor *desc = _descriptors[i];
        if (handle == desc->_handle)
            return desc;
    }

    return NULL;
}

void
BleCharacteristic::_addCCCDescriptor(void)
{
    /* Activate our CCCD descriptor object */
    _cccd._handle = _handles.cccd_handle;
    _cccd._initialised = true;
    _cccd.setEventCallback(_cccdEventHandler, (void *)this);
    _descriptors[_num_descriptors++] = &_cccd;
}

void
BleCharacteristic::_setConnectedState(boolean_t connected)
{
    _connected = connected;

    /* Reset the state of these internal variables when connection is dropped */
    if (!connected) {
        _notifyEnabled = false;
        _indicateEnabled = false;
    }

    /* Cascade the connected-state update to descriptors */
    for (unsigned i = 0; i < _num_descriptors; i++)
        _descriptors[i]->_setConnectedState(connected);
}
