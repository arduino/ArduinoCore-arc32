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

#include "BLEAttribute.h"
#include "BLECharacteristicImp.h"

#include "BLECallbacks.h"
#include "BLEUtils.h"

bt_uuid_16_t BLECharacteristicImp::_gatt_chrc_uuid = {BT_UUID_TYPE_16, BT_UUID_GATT_CHRC_VAL};
bt_uuid_16_t BLECharacteristicImp::_gatt_ccc_uuid = {BT_UUID_TYPE_16, BT_UUID_GATT_CCC_VAL};

BLECharacteristicImp::BLECharacteristicImp(BLECharacteristic& characteristic, 
                                           const BLEDevice& bledevice):
    BLEAttribute(characteristic.uuid(), BLETypeCharacteristic),
    _value_length(0),
    _value_buffer(NULL),
    _value_updated(false),
    _attr_chrc_declaration(NULL),
    _attr_chrc_value(NULL),
    _attr_cccd(NULL),
    _ble_device()
{
    unsigned char properties = characteristic._properties;
    _value_size = characteristic._value_size;
    _value = (unsigned char*)balloc(_value_size, NULL);
    if (_value_size > BLE_MAX_ATTR_DATA_LEN)
    {
        _value_buffer = (unsigned char*)balloc(_value_size, NULL);
    }
    
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
    _gatt_chrc.uuid = (bt_uuid_t*)this->bt_uuid();//&_characteristic_uuid;//this->uuid();
    memset(_event_handlers, 0, sizeof(_event_handlers));
    
    _sub_params.notify = profile_notify_process;
    
    if (NULL != characteristic._value)
    {
        memcpy(_value, characteristic._value, _value_size);
    }
        
    // Update BLE device object
    _ble_device.setAddress(*bledevice.bt_le_address());
    
    characteristic.setBLECharacteristicImp(this);
    memset(&_descriptors_header, 0, sizeof(_descriptors_header));
}

BLECharacteristicImp::~BLECharacteristicImp()
{
    releaseDescriptors();
    if (_value) {
        bfree(_value);
        _value = NULL;
    }
    if (_value_buffer)
    {
        bfree(_value_buffer);
        _value_buffer = NULL;
    }
}

unsigned char
BLECharacteristicImp::properties() const
{
    return _gatt_chrc.properties;
}

bool BLECharacteristicImp::writeValue(const byte value[], int length)
{
    int status;
    
    _setValue(value, length);
    
    // Address same is GATT server. Send notification if CCCD enabled
    // Different is GATT client. Send write request
    if (true == BLEUtils::isLocalBLE(_ble_device) &&
        NULL != _attr_chrc_value)
    {
        // Notify for peripheral.
        status = bt_gatt_notify(NULL, _attr_chrc_value, value, length, NULL);
        if (0 != status)
        {
            return false;
        }
    }
    
    //Not schedule write request for central
    // The write request may failed. 
    // If user want to get latest set value. Call read and get the real value
    return true;
}

bool
BLECharacteristicImp::setValue(const unsigned char value[], uint16_t length)
{
    _setValue(value, length);
    // Read response/Notification/Indication for GATT client
    // Write request for GATT server
    if (_event_handlers[BLEWritten]) 
    {
        BLECharacteristic chrcTmp(this, &_ble_device);
        _event_handlers[BLEWritten](_ble_device, chrcTmp);
    }
    
    return true;
}

unsigned short
BLECharacteristicImp::valueSize() const
{
    return _value_size;
}

const unsigned char*
BLECharacteristicImp::value() const
{
    return _value;
}

unsigned short
BLECharacteristicImp::valueLength() const
{
    return _value_length;
}

unsigned char
BLECharacteristicImp::operator[] (int offset) const
{
    return _value[offset];
}

bool
BLECharacteristicImp::written()
{
    bool written = false;
    if (true == BLEUtils::isLocalBLE(_ble_device))
    {
        // GATT server. The characteristic on local device
        written = _value_updated;
        _value_updated = false;
    }

    return written;
}

bool BLECharacteristicImp::valueUpdated()
{
    bool updated = false;
    if (false == BLEUtils::isLocalBLE(_ble_device))
    {
        // GATT client. The characteristic on remote device.
        updated = _value_updated;
        _value_updated = false;
    }
    return updated;
}

bool
BLECharacteristicImp::subscribed()
{
    return (_gatt_chrc.properties & (BT_GATT_CHRC_NOTIFY | BT_GATT_CHRC_INDICATE));
}

void
BLECharacteristicImp::setEventHandler(BLECharacteristicEvent event, BLECharacteristicEventHandler callback)
{
    noInterrupts();
    if (event < sizeof(_event_handlers)) {
        _event_handlers[event] = callback;
    }
    interrupts();
}

uint16_t
BLECharacteristicImp::valueHandle()
{
    uint16_t handle = 0;
    if (NULL != _attr_chrc_value)
    {
        handle = _attr_chrc_value->handle;
    }
    
    return handle;
}

uint16_t
BLECharacteristicImp::cccdHandle()
{
    uint16_t handle = 0;
    if (NULL != _attr_cccd)
    {
        handle = _attr_cccd->handle;
    }
    return handle;
}

void
BLECharacteristicImp::_setValue(const uint8_t value[], uint16_t length)
{
    if (length > _value_size) {
        length = _value_size;
    }
    
    _value_updated = true;
    memcpy(_value, value, length);
    _value_length = length;
}

_bt_gatt_ccc_t* BLECharacteristicImp::getCccCfg(void)
{
    return &_ccc_value;
}

bt_gatt_chrc_t* BLECharacteristicImp::getCharacteristicAttValue(void)
{
    return &_gatt_chrc;
}

uint8_t BLECharacteristicImp::getPermission(void)
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

bt_uuid_t* BLECharacteristicImp::getCharacteristicAttributeUuid(void)
{
    return (bt_uuid_t*) &_gatt_chrc_uuid;
}

bt_uuid_t* BLECharacteristicImp::getClientCharacteristicConfigUuid(void)
{
    return (bt_uuid_t*) &_gatt_ccc_uuid;
}

#if 0
void BLECharacteristicImp::discover(bt_gatt_discover_params_t *params)
{
    params->type = BT_GATT_DISCOVER_CHARACTERISTIC;
    params->uuid = this->uuid();
    // Start discovering
    _discoverying = true;
    // Re-Init the read/write parameter
    _reading = false;
}


void BLECharacteristicImp::discover(const bt_gatt_attr_t *attr,
			                     bt_gatt_discover_params_t *params)
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
#endif

bt_gatt_subscribe_params_t *BLECharacteristicImp::getSubscribeParams()
{
    return &_sub_params;
}

bool BLECharacteristicImp::read()
{
    int retval = 0;
    bt_conn_t* conn = NULL;
    
    if (true == BLEUtils::isLocalBLE(_ble_device))
    {
        // GATT server can't write
        return false;
    }
    
    if (_reading)
    {
        // Already in reading state
        return false;
    }
    
    _read_params.func = profile_read_rsp_process;
    _read_params.handle_count = 1;
    _read_params.single.handle = _value_handle;
    _read_params.single.offset = 0;
    
    if (0 == _read_params.single.handle)
    {
        // Discover not complete
        return false;
    }
    
    conn = bt_conn_lookup_addr_le(_ble_device.bt_le_address());
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

bool BLECharacteristicImp::write(const unsigned char value[], 
                                 uint16_t length)
{
    int retval = 0;
    bt_conn_t* conn = NULL;
    
    if (true == BLEUtils::isLocalBLE(_ble_device))
    {
        // GATT server can't write
        return false;
    }
    
    conn = bt_conn_lookup_addr_le(_ble_device.bt_le_address());
    if (NULL == conn)
    {
        return false;
    }
    
    // Send read request
    retval = bt_gatt_write_without_response(conn, 
                                            _value_handle,
                                            value, 
                                            length, 
                                            false);
    bt_conn_unref(conn);
    return (0 == retval);
}

void BLECharacteristicImp::setBuffer(const uint8_t value[], 
                                      uint16_t length, 
                                      uint16_t offset)
{
    if (length + offset > _value_size) {
        // Ignore the data
        return;
    }

    memcpy(_value_buffer + offset, value, length);
}

void BLECharacteristicImp::syncupBuffer2Value()
{
    setValue(_value_buffer, _value_size);
}

void BLECharacteristicImp::discardBuffer()
{
    memcpy(_value_buffer, _value, _value_size);
}

bool BLECharacteristicImp::longCharacteristic()
{
    return (_value_size > BLE_MAX_ATTR_DATA_LEN);
}

int BLECharacteristicImp::updateProfile(bt_gatt_attr_t *attr_start, int& index)
{
    bt_gatt_attr_t *start = attr_start;
    int base_index = index;
    int offset = 0;
    int counter = 0;
    
    // Characteristic declare
    memset(start, 0, sizeof(bt_gatt_attr_t));
    start->uuid = getCharacteristicAttributeUuid();
    start->perm = BT_GATT_PERM_READ;
    start->read = bt_gatt_attr_read_chrc;
    start->user_data = this->getCharacteristicAttValue();
    pr_info(LOG_MODULE_BLE, "chrc-%p, uuid type-%d", start, start->uuid->type);
    
    start++;
    index++;
    counter++;
    
    // Descriptor
    memset(start, 0, sizeof(bt_gatt_attr_t));
    start->uuid = (bt_uuid_t *)bt_uuid();
    start->perm = this->getPermission();
    start->user_data = (void*)((BLEAttribute*)this);
    start->read = profile_read_process;

    if (this->longCharacteristic() == false)
    {
        // Normal characteristic MAX. 20
        start->write = profile_write_process;
    }
    else
    {
        // Long characteristic. MAX. 512
        start->write = profile_longwrite_process;
        start->flush = profile_longflush_process;
    }
    _attr_chrc_value = start;
    pr_debug(LOG_MODULE_BLE, "chrcdescripor-%p, chimp-%p type-%d", start, this, this->type());
    
    start++;
    index++;
    counter++;
    
    if (this->subscribed())
    {
        // Descriptor
        memset(start, 0, sizeof(bt_gatt_attr_t));
        start->uuid = this->getClientCharacteristicConfigUuid();
        start->perm = BT_GATT_PERM_READ | BT_GATT_PERM_WRITE;
        start->read = bt_gatt_attr_read_ccc;
        start->write = bt_gatt_attr_write_ccc;
        start->user_data = this->getCccCfg();
        
        pr_info(LOG_MODULE_BLE, "cccd-%p", start);
        
        start++;
        index++;
        counter++;
    }
    
    BLEDescriptorNodePtr node = _descriptors_header.next;
    while (NULL != node)
    {
        BLEDescriptorImp *descriptorImp = node->value;
        start = attr_start + index - base_index;
        offset = descriptorImp->updateProfile(start, index);
        counter += offset;
        node = node->next;
    }
    pr_debug(LOG_MODULE_BLE, "%s:type-%d", __FUNCTION__, this->type());
    return counter;
}

int BLECharacteristicImp::addDescriptor(BLEDescriptor& descriptor)
{
    BLEDescriptorImp* descriptorImp = new BLEDescriptorImp(_ble_device, descriptor);
    pr_debug(LOG_MODULE_BLE, "%s-%d",__FUNCTION__, __LINE__);
    if (NULL == descriptorImp)
    {
        return BLE_STATUS_NO_MEMORY;
    }
    
    pr_debug(LOG_MODULE_BLE, "%s-%d",__FUNCTION__, __LINE__);
    BLEDescriptorNodePtr node = link_node_create(descriptorImp);
    if (NULL == node)
    {
        delete[] descriptorImp;
        return BLE_STATUS_NO_MEMORY;
    }
    link_node_insert_last(&_descriptors_header, node);
    pr_debug(LOG_MODULE_BLE, "%s-%d",__FUNCTION__, __LINE__);
    return BLE_STATUS_SUCCESS;
}

void BLECharacteristicImp::releaseDescriptors()
{
    BLEDescriptorNodePtr node = link_node_get_first(&_descriptors_header);
    
    while (NULL != node)
    {
        BLEDescriptorImp* descriptorImp = node->value;
        delete[] descriptorImp;
        link_node_remove_first(&_descriptors_header);
        node = link_node_get_first(&_descriptors_header);
    }
}

int BLECharacteristicImp::getAttributeCount()
{
    int counter = link_list_size(&_descriptors_header) + 2; // Declaration and descriptor
    return counter;
}

int BLECharacteristicImp::descriptorCount() const
{
    int counter = link_list_size(&_descriptors_header);
    return counter;
}


