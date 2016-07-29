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
#include "BLEPeripheralHelper.h"
#include "internal/ble_client.h"

uint8_t profile_notify_process (struct bt_conn *conn,
                             struct bt_gatt_subscribe_params *params,
                             const void *data, uint16_t length);
uint8_t profile_read_rsp_process(struct bt_conn *conn, int err,
                                 struct bt_gatt_read_params *params,
                                 const void *data, 
                                 uint16_t length);

unsigned char BLECharacteristic::_numNotifyAttributes = 0;

struct bt_uuid_16 BLECharacteristic::_gatt_chrc_uuid = {BT_UUID_TYPE_16, BT_UUID_GATT_CHRC_VAL};
struct bt_uuid_16 BLECharacteristic::_gatt_ccc_uuid = {BT_UUID_TYPE_16, BT_UUID_GATT_CCC_VAL};

BLECharacteristic::BLECharacteristic(const char* uuid,
                      const unsigned char properties,
                      const unsigned short maxLength) :
    BLEAttribute(uuid, BLETypeCharacteristic),
    _value_length(0),
    _written(false),
    _user_description(NULL),
    _presentation_format(NULL),
    _attr_chrc_declaration(NULL),
    _attr_chrc_value(NULL),
    _attr_cccd(NULL)
{
    _value_size = maxLength > BLE_MAX_ATTR_DATA_LEN ? BLE_MAX_ATTR_DATA_LEN : maxLength;
    _value = (unsigned char*)malloc(_value_size);
    
    memset(&_ccc_cfg, 0, sizeof(_ccc_cfg));
    memset(&_ccc_value, 0, sizeof(_ccc_value));
    memset(&_gatt_chrc, 0, sizeof(_gatt_chrc));
    memset(&_sub_params, 0, sizeof(_sub_params));
    
    _ccc_value.cfg = &_ccc_cfg;
    _ccc_value.cfg_len = 1;
    if (BLERead & properties)
    {
        _gatt_chrc.properties |= BT_GATT_CHRC_READ;
    }
    if (BLEWrite & properties)
    {
        _gatt_chrc.properties |= BT_GATT_CHRC_WRITE;
    }
    if (BLEWriteWithoutResponse & properties)
    {
        _gatt_chrc.properties |= BT_GATT_CHRC_WRITE_WITHOUT_RESP;
    }
    if (BLENotify & properties)
    {
        _gatt_chrc.properties |= BT_GATT_CHRC_NOTIFY;
        _sub_params.value |= BT_GATT_CCC_NOTIFY;
    }
    if (BLEIndicate & properties)
    {
        _gatt_chrc.properties |= BT_GATT_CHRC_INDICATE;
        _sub_params.value |= BT_GATT_CCC_INDICATE;
    }
    _gatt_chrc.uuid = this->uuid();
    memset(_event_handlers, 0, sizeof(_event_handlers));
    
    _numNotifyAttributes++;
    if (properties & (BT_GATT_CHRC_NOTIFY | BT_GATT_CHRC_INDICATE))
    {
        _numNotifyAttributes++;
    }
    _sub_params.notify = profile_notify_process;
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
    return _gatt_chrc.properties;
}

bool
BLECharacteristic::setValue(const unsigned char value[], uint16_t length)
{
    int status;

     _setValue(value, length);

    if (_attr_chrc_value)
    {
        // TODO: Notify for peripheral.
        //        Write request for central.
        status = bt_gatt_notify(NULL, _attr_chrc_value, value, length, NULL);
        if (0 != status)
        {
            return false;
        }
    }
    return true;
}

void
BLECharacteristic::setValue(BLEHelper& blehelper, const unsigned char* value, unsigned short length)
{
    //BLEHelper *bledevice = &central;
    _setValue(value, length);

    _written = true;
    _reading = false;

    if (_event_handlers[BLEWritten]) {
        _event_handlers[BLEWritten](blehelper, *this);
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
    return (_gatt_chrc.properties & (BT_GATT_CHRC_NOTIFY | BT_GATT_CHRC_INDICATE));
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

uint16_t
BLECharacteristic::valueHandle()
{
    uint16_t handle = 0;
    if (NULL != _attr_chrc_value)
    {
        handle = _attr_chrc_value->handle;
    }
    
    return handle;
}

uint16_t
BLECharacteristic::cccdHandle()
{
    uint16_t handle = 0;
    if (NULL != _attr_cccd)
    {
        handle = _attr_cccd->handle;
    }
    return handle;
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

unsigned char
BLECharacteristic::numNotifyAttributes(void) {
    return _numNotifyAttributes;
}

struct _bt_gatt_ccc* BLECharacteristic::getCccCfg(void)
{
    return &_ccc_value;
}

struct bt_gatt_chrc* BLECharacteristic::getCharacteristicAttValue(void)
{
    return &_gatt_chrc;
}

uint8_t BLECharacteristic::getPermission(void)
{
    uint8_t perm = 0;
    if (_gatt_chrc.properties & BT_GATT_CHRC_READ)
    {
        perm |= BT_GATT_PERM_READ;
    }
    if (_gatt_chrc.properties & (BT_GATT_CHRC_WRITE | BT_GATT_CHRC_WRITE_WITHOUT_RESP))
    {
        perm |= BT_GATT_PERM_WRITE;
    }
    return perm;
}

struct bt_uuid* BLECharacteristic::getCharacteristicAttributeUuid(void)
{
    return (struct bt_uuid*) &_gatt_chrc_uuid;
}
struct bt_uuid* BLECharacteristic::getClientCharacteristicConfigUuid(void)
{
	return (struct bt_uuid*) &_gatt_ccc_uuid;
}


void BLECharacteristic::addCharacteristicDeclaration(struct bt_gatt_attr *gatt_attr)
{
    _attr_chrc_declaration = gatt_attr;
}

void BLECharacteristic::addCharacteristicValue(struct bt_gatt_attr *gatt_attr)
{
    _attr_chrc_value = gatt_attr;
}

void BLECharacteristic::addCharacteristicConfigDescriptor(struct bt_gatt_attr *gatt_attr)
{
    _attr_cccd = gatt_attr;
}

void BLECharacteristic::discover(struct bt_gatt_discover_params *params)
{
    params->type = BT_GATT_DISCOVER_CHARACTERISTIC;
    params->uuid = this->uuid();
    // Start discovering
    _discoverying = true;
    // Re-Init the read/write parameter
    _reading = false;
}


void BLECharacteristic::discover(const struct bt_gatt_attr *attr,
			                     struct bt_gatt_discover_params *params)
{
    if (!attr)
    {
        // Discovery complete
        _discoverying = false;
        return;
    }
    
    // Chracteristic Char
    if (params->uuid == this->uuid())
    {
        // Set Discover CCCD parameter
        params->start_handle = attr->handle + 2;
        if (subscribed())
        {
            // Include CCCD
            params->type = BT_GATT_DISCOVER_DESCRIPTOR;
            params->uuid = this->getClientCharacteristicConfigUuid();
        }
        else
        {
            // Complete the discover
            _discoverying = false;
        }
    }
    else if (params->uuid == this->getClientCharacteristicConfigUuid())
    {
        params->start_handle = attr->handle + 1;
        _discoverying = false;
    }
}

struct bt_gatt_subscribe_params *BLECharacteristic::getSubscribeParams()
{
    return &_sub_params;
}

bool BLECharacteristic::read(BLEPeripheralHelper &peripheral)
{
    int retval = 0;
    struct bt_conn* conn = NULL;
    if (_reading)
    {
        // Already in reading state
        return false;
    }
    
    _read_params.func = profile_read_rsp_process;
    _read_params.handle_count = 1;
    _read_params.single.handle = peripheral.valueHandle(this);
    _read_params.single.offset = 0;
    
    if (0 == _read_params.single.handle)
    {
        // Discover not complete
        return false;
    }
    
    conn = bt_conn_lookup_addr_le(peripheral.bt_le_address());
    if (NULL == conn)
    {
        return false;
    }
    
    // Send read request
    retval = bt_gatt_read(conn, &_read_params);
    bt_conn_unref(conn);
    if (0 == retval)
    {
        _reading = true;
    }
    return _reading;
}

bool BLECharacteristic::write(BLEPeripheralHelper &peripheral, 
                              const unsigned char value[], 
                              uint16_t length)
{
    int retval = 0;
    struct bt_conn* conn = NULL;
    
    conn = bt_conn_lookup_addr_le(peripheral.bt_le_address());
    if (NULL == conn)
    {
        return false;
    }
    
    // Send read request
    retval = bt_gatt_write_without_response(conn, 
                                            peripheral.valueHandle(this),
                                            value, length, false);
    bt_conn_unref(conn);
    return (0 == retval);
}


