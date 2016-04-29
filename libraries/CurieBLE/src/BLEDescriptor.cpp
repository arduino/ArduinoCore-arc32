/*
 * Copyright (c) 2015 Intel Corporation.  All rights reserved.
 */
#include "BLEDescriptor.h"
#include "internal/ble_client.h"

BLEDescriptor::BLEDescriptor(const char* uuid, const unsigned char value[], unsigned short valueLength) :
    BLEAttribute(uuid, BLETypeDescriptor)
{
    if (valueLength > BLE_MAX_ATTR_DATA_LEN) {
        valueLength = BLE_MAX_ATTR_DATA_LEN;
    }
    _value_length = valueLength;
    _value = (unsigned char*)malloc(_value_length);

    memcpy(_value, value, _value_length);
}

BLEDescriptor::~BLEDescriptor() {
    if (_value) {
        free(_value);
        _value = NULL;
    }
}

BLEDescriptor::BLEDescriptor(const char* uuid, const char* value) :
    BLEDescriptor(uuid, (const uint8_t*)value, strlen(value))
{
}

const unsigned char*
BLEDescriptor::BLEDescriptor::value() const
{
    return _value;
}

unsigned short
BLEDescriptor::valueLength() const
{
    return _value_length;
}

unsigned char
BLEDescriptor::operator[] (int offset) const
{
    return _value[offset];
}

bool
BLEDescriptor::add(uint16_t serviceHandle)
{
    bt_uuid uuid = btUuid();
    struct ble_gatts_descriptor desc;
    uint16_t handle = 0;

    memset(&desc, 0, sizeof(desc));

    desc.p_uuid = &uuid;

    desc.p_value = _value;
    desc.length = _value_length;

    // this class only supports read-only descriptors
    desc.perms.rd = GAP_SEC_MODE_1 | GAP_SEC_LEVEL_1;
    desc.perms.wr = GAP_SEC_NO_PERMISSION;

    return (ble_client_gatts_add_descriptor(serviceHandle, &desc, &handle) == BLE_STATUS_SUCCESS); 
}
