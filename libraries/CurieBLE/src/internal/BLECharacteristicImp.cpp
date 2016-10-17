/*
 * Copyright (c) 2016 Intel Corporation.  All rights reserved.
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
#include <errno.h>

#include "BLEAttribute.h"
#include "BLEServiceImp.h"
#include "BLECharacteristicImp.h"

#include "BLECallbacks.h"
#include "BLEUtils.h"

bt_uuid_16_t BLECharacteristicImp::_gatt_chrc_uuid = {BT_UUID_TYPE_16, BT_UUID_GATT_CHRC_VAL};
bt_uuid_16_t BLECharacteristicImp::_gatt_ccc_uuid = {BT_UUID_TYPE_16, BT_UUID_GATT_CCC_VAL};

BLECharacteristicImp::BLECharacteristicImp(const bt_uuid_t* uuid, 
                                           unsigned char properties,
                                           uint16_t handle,
                                           const BLEDevice& bledevice):
    BLEAttribute(uuid, BLETypeCharacteristic),
    _value_length(0),
    _value_buffer(NULL),
    _value_updated(false),
    _value_handle(handle),
    _cccd_handle(0),
    _attr_chrc_value(NULL),
    _attr_cccd(NULL),
    _subscribed(false),
    _reading(false),
    _ble_device()
{
    _value_size = BLE_MAX_ATTR_DATA_LEN;// Set as MAX value. TODO: long read/write need to twist
    _value = (unsigned char*)malloc(_value_size);

    // TODO: Enable when max value is not set.
    //    if (_value_size > BLE_MAX_ATTR_DATA_LEN)
    //    {
    //        _value_buffer = (unsigned char*)malloc(_value_size);
    //    }

    if (_value)
    {
      memset(_value, 0, _value_size);
    }
    else
    {
        errno = ENOMEM;
    }
    
    memset(&_ccc_cfg, 0, sizeof(_ccc_cfg));
    memset(&_ccc_value, 0, sizeof(_ccc_value));
    memset(&_gatt_chrc, 0, sizeof(_gatt_chrc));
    memset(&_sub_params, 0, sizeof(_sub_params));
    memset(&_discover_params, 0, sizeof(_discover_params));
    
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
    memset(_oldevent_handlers, 0, sizeof(_oldevent_handlers));
    
    _sub_params.notify = profile_notify_process;
        
    // Update BLE device object
    _ble_device.setAddress(*bledevice.bt_le_address());
    
    memset(&_descriptors_header, 0, sizeof(_descriptors_header));
}

BLECharacteristicImp::BLECharacteristicImp(BLECharacteristic& characteristic, 
                                           const BLEDevice& bledevice):
    BLEAttribute(characteristic.uuid(), BLETypeCharacteristic),
    _value_length(0),
    _value_buffer(NULL),
    _value_updated(false),
    _value_handle(0),
    _cccd_handle(0),
    _attr_chrc_value(NULL),
    _attr_cccd(NULL),
    _subscribed(false),
    _reading(false),
    _ble_device()
{
    unsigned char properties = characteristic._properties;
    _value_size = characteristic._value_size;
    _value = (unsigned char*)malloc(_value_size);
    if (_value == NULL)
    {
        errno = ENOMEM;
    }
    if (_value_size > BLE_MAX_ATTR_DATA_LEN)
    {
        _value_buffer = (unsigned char*)malloc(_value_size);
    }
    
    memset(&_ccc_cfg, 0, sizeof(_ccc_cfg));
    memset(&_ccc_value, 0, sizeof(_ccc_value));
    memset(&_gatt_chrc, 0, sizeof(_gatt_chrc));
    memset(&_sub_params, 0, sizeof(_sub_params));
    memset(&_discover_params, 0, sizeof(_discover_params));
    
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

    memcpy(_event_handlers, characteristic._event_handlers, sizeof(_event_handlers));
    memcpy(_oldevent_handlers, characteristic._oldevent_handlers, sizeof(_oldevent_handlers));
    
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
    if (_value)
    {
        free(_value);
        _value = (unsigned char *)NULL;
    }
    
    if (_value_buffer)
    {
        free(_value_buffer);
        _value_buffer = (unsigned char *)NULL;
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
    bool retVal = false;
    
    _setValue(value, length, 0);
    
    // Address same is GATT server. Send notification if CCCD enabled
    // Different is GATT client. Send write request
    if (true == BLEUtils::isLocalBLE(_ble_device) &&
        NULL != _attr_chrc_value)
    {
        // Notify for peripheral.
        status = bt_gatt_notify(NULL, _attr_chrc_value, value, length, NULL);
    // Sid.  KW found status is always 0
    //        if (!status)
    //        {
            retVal = true;
    //        }
    }
    
    //Not schedule write request for central
    // The write request may failed. 
    // If user want to get latest set value. Call read and get the real value
    return retVal;
}

bool BLECharacteristicImp::writeValue(const byte value[], int length, int offset)
{
    int status;
    bool retVal = false;
    
    _setValue(value, length, offset);
    
    // Address same is GATT server. Send notification if CCCD enabled
    // Different is GATT client. Send write request
    if (true == BLEUtils::isLocalBLE(_ble_device) &&
        NULL != _attr_chrc_value)
    {
        // Notify for peripheral.
        status = bt_gatt_notify(NULL, _attr_chrc_value, value, length, NULL);
	// Sid.  KW found status is always 0.
	//        if (!status)
	//        {
            retVal = true;
	//        }
    }
    
    //Not schedule write request for central
    // The write request may failed. 
    // If user want to get latest set value. Call read and get the real value
    return retVal;
}

bool
BLECharacteristicImp::setValue(const unsigned char value[], uint16_t length)
{
    _setValue(value, length, 0);
    if (BLEUtils::isLocalBLE(_ble_device) == true)
    {
        // GATT server
        // Write request for GATT server
        if (_event_handlers[BLEWritten]) 
        {
            BLECharacteristic chrcTmp(this, &_ble_device);
            _event_handlers[BLEWritten](_ble_device, chrcTmp);
        }
        
        if (_oldevent_handlers[BLEWritten]) 
        {
            BLECharacteristic chrcTmp(this, &_ble_device);
            BLECentral central(_ble_device);
            _oldevent_handlers[BLEWritten](central, chrcTmp);
        }
    }
    else
    {
        // GATT client
        // Discovered attribute
        // Read response/Notification/Indication for GATT client
        if (_reading)
        {
            // Read response received. Not block the other reading.
            _reading = false;
        }
        
        if (_event_handlers[BLEValueUpdated]) 
        {
            BLECharacteristic chrcTmp(this, &_ble_device);
            _event_handlers[BLEValueUpdated](_ble_device, chrcTmp);
        }
        
        if (_oldevent_handlers[BLEValueUpdated]) 
        {
            BLECharacteristic chrcTmp(this, &_ble_device);
            BLECentral central(_ble_device);
            _oldevent_handlers[BLEValueUpdated](central, chrcTmp);
        }
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
    if (false == BLEUtils::isLocalBLE(_ble_device))
    {
        // GATT client
        return _subscribed;
    }
    else
    {
        // GATT server
        return (_ccc_value.value & (BT_GATT_CCC_NOTIFY | BT_GATT_CCC_INDICATE));
    }
}

bool BLECharacteristicImp::canNotify()
{
    if (false == BLEUtils::isLocalBLE(_ble_device))
    {
        // GATT client can't subscribe
        return false;
    }
    
    // GATT server
    return (_ccc_value.value & BT_GATT_CCC_NOTIFY);
}

bool BLECharacteristicImp::canIndicate()
{
    if (false == BLEUtils::isLocalBLE(_ble_device))
    {
        // GATT client can't subscribe
        return false;
    }
    
    // GATT server
    return (_ccc_value.value & BT_GATT_CCC_INDICATE);
}

bool BLECharacteristicImp::unsubscribe(void)
{
    int retval = 0;
    bt_conn_t* conn = NULL;
    
    if (true == BLEUtils::isLocalBLE(_ble_device))
    {
        // GATT server can't subscribe
        return false;
    }
    
    if (false == _subscribed)
    {
        return true;
    }
    
    _sub_params.value = 0;
    
    if (0 == (_gatt_chrc.properties & (BT_GATT_CHRC_NOTIFY | BT_GATT_CHRC_INDICATE)))
    {
        // The characteristic not support the Notify and Indicate
        return false;
    }
    
    conn = bt_conn_lookup_addr_le(_ble_device.bt_le_address());
    if (NULL == conn)
    {
        return false;
    }
    
    bt_addr_le_copy(&_sub_params._peer, bt_conn_get_dst(conn));
    _sub_params.ccc_handle = _cccd_handle;
    _sub_params.value_handle = _value_handle;
    
    // Enable CCCD to allow peripheral send Notification/Indication
    retval = bt_gatt_unsubscribe(conn, &_sub_params);
    bt_conn_unref(conn);
    if (0 == retval)
    {
        _subscribed = false;
    }
    return _subscribed;
}

bool BLECharacteristicImp::subscribe(void)
{
    int retval = 0;
    bt_conn_t* conn = NULL;
    
    if (true == BLEUtils::isLocalBLE(_ble_device))
    {
        // GATT server can't subscribe
        return false;
    }
    
    if (_gatt_chrc.properties & BT_GATT_CHRC_NOTIFY)
    {
        _sub_params.value |= BT_GATT_CCC_NOTIFY;
    }
    
    if (_gatt_chrc.properties & BT_GATT_CHRC_INDICATE)
    {
        _sub_params.value |= BT_GATT_CCC_INDICATE;
    }
    
    if (_sub_params.value == 0)
    {
        return false;
    }
    
    conn = bt_conn_lookup_addr_le(_ble_device.bt_le_address());
    if (NULL == conn)
    {
        return false;
    }
    
    bt_addr_le_copy(&_sub_params._peer, bt_conn_get_dst(conn));
    _sub_params.ccc_handle = _cccd_handle;
    _sub_params.value_handle = _value_handle;
    
    // Enable CCCD to allow peripheral send Notification/Indication
    retval = bt_gatt_subscribe(conn, &_sub_params);
    bt_conn_unref(conn);
    if (0 == retval)
    {
        _subscribed = true;
    }
    return _subscribed;
}

void
BLECharacteristicImp::setEventHandler(BLECharacteristicEvent event, BLECharacteristicEventHandler callback)
{
    noInterrupts();
    if (event < BLECharacteristicEventLast) {
        _event_handlers[event] = callback;
    }
    interrupts();
}

void
BLECharacteristicImp::setEventHandler(BLECharacteristicEvent event, BLECharacteristicEventHandlerOld callback)
{
    noInterrupts();
    if (event < BLECharacteristicEventLast) {
        _oldevent_handlers[event] = callback;
    }
    interrupts();
}

void
BLECharacteristicImp::setHandle(uint16_t handle)
{
    // GATT client
    _value_handle = handle;
}

void
BLECharacteristicImp::setCCCDHandle(uint16_t handle)
{
    // GATT client
    _cccd_handle = handle;
}

uint16_t
BLECharacteristicImp::valueHandle()
{
    uint16_t handle = 0;
    if (NULL != _attr_chrc_value)
    {
        //GATT server
        handle = _attr_chrc_value->handle;
    }
    else
    {
        // GATT client
        handle = _value_handle;
    }
    
    return handle;
}

void
BLECharacteristicImp::_setValue(const uint8_t value[], uint16_t length, uint16_t offset)
{
    if (length + offset > _value_size)
    {
        if (_value_size > offset)
        {
            uint16_t temp_len = _value_size - offset;
            if (length > temp_len)
            {
                length = temp_len;
            }
        }
        else
        {
            return;
        }
    }
    
    _value_updated = true;
    memcpy(_value + offset, value, length);
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
  if ((length + offset > _value_size) ||
      ((unsigned char *)NULL == _value_buffer)) {
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
  if(_value_buffer)
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
    
    if (0 != (_gatt_chrc.properties & (BT_GATT_CHRC_NOTIFY | BT_GATT_CHRC_INDICATE)))
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
    BLEDescriptorImp* descriptorImp = descrptor(descriptor.uuid());
    if (NULL != descriptorImp)
    {
        return BLE_STATUS_SUCCESS;
    }
    
    descriptorImp = new BLEDescriptorImp(_ble_device, descriptor);
    pr_debug(LOG_MODULE_BLE, "%s-%d",__FUNCTION__, __LINE__);
    if (NULL == descriptorImp)
    {
        return BLE_STATUS_NO_MEMORY;
    }
    
    pr_debug(LOG_MODULE_BLE, "%s-%d",__FUNCTION__, __LINE__);
    BLEDescriptorNodePtr node = link_node_create(descriptorImp);
    if (NULL == node)
    {
        delete descriptorImp;
        return BLE_STATUS_NO_MEMORY;
    }
    link_node_insert_last(&_descriptors_header, node);
    pr_debug(LOG_MODULE_BLE, "%s-%d",__FUNCTION__, __LINE__);
    return BLE_STATUS_SUCCESS;
}

int BLECharacteristicImp::addDescriptor(const bt_uuid_t* uuid, 
                                        unsigned char property, 
                                        uint16_t handle)
{
    BLEDescriptorImp* descriptorImp = descrptor(uuid);
    if (NULL != descriptorImp)
    {
        return BLE_STATUS_SUCCESS;
    }
    
    descriptorImp = new BLEDescriptorImp(uuid, property, handle, _ble_device);
    pr_debug(LOG_MODULE_BLE, "%s-%d",__FUNCTION__, __LINE__);
    if (NULL == descriptorImp)
    {
        return BLE_STATUS_NO_MEMORY;
    }
    
    BLEDescriptorNodePtr node = link_node_create(descriptorImp);
    if (NULL == node)
    {
        delete descriptorImp;
        return BLE_STATUS_NO_MEMORY;
    }
    link_node_insert_last(&_descriptors_header, node);
    pr_debug(LOG_MODULE_BLE, "%s-%d",__FUNCTION__, __LINE__);
    return BLE_STATUS_SUCCESS;
}

BLEDescriptorImp* BLECharacteristicImp::descrptor(const bt_uuid_t* uuid)
{
    BLEDescriptorImp* descriptorImp = NULL;
    BLEDescriptorNodePtr node = link_node_get_first(&_descriptors_header);
    
    while (NULL != node)
    {
        descriptorImp = node->value;
        if (true == descriptorImp->compareUuid(uuid))
        {
            break;
        }
        node = node->next;
    }
    
    if (NULL == node)
    {
        descriptorImp = NULL;
    }
    return descriptorImp;
}

BLEDescriptorImp* BLECharacteristicImp::descrptor(const char* uuid)
{
    bt_uuid_128_t uuid_tmp;
    BLEUtils::uuidString2BT(uuid, (bt_uuid_t *)&uuid_tmp);
    return descrptor((const bt_uuid_t *)&uuid_tmp);
}


BLEDescriptorImp* BLECharacteristicImp::descrptor(int index)
{
    BLEDescriptorImp* descriptorImp = NULL;
    BLEDescriptorNodePtr node = link_node_get_first(&_descriptors_header);
    while (NULL != node)
    {
        if (0 >= index)
        {
            descriptorImp = node->value;
            break;
        }
        index--;
        node = node->next;
    }
    return descriptorImp;
}

void BLECharacteristicImp::releaseDescriptors()
{
    BLEDescriptorNodePtr node = link_node_get_first(&_descriptors_header);
    
    while (NULL != node)
    {
        BLEDescriptorImp* descriptorImp = node->value;
        delete descriptorImp;
        link_node_remove_first(&_descriptors_header);
        node = link_node_get_first(&_descriptors_header);
    }
}

int BLECharacteristicImp::getAttributeCount()
{
    int counter = link_list_size(&_descriptors_header) + 2; // Declaration and descriptor
    // Notification/Indecation
    if (_gatt_chrc.properties & (BT_GATT_CHRC_NOTIFY | BT_GATT_CHRC_INDICATE))
    {
        counter++;
    }
    return counter;
}

int BLECharacteristicImp::descriptorCount() const
{
    int counter = link_list_size(&_descriptors_header);
    return counter;
}

bool BLECharacteristicImp::discoverAttributes(BLEDevice* device)
{
    
    int err;
    bt_conn_t* conn;
    bt_gatt_discover_params_t* temp = NULL;
    const bt_uuid_t* service_uuid = bt_uuid();
    
    if (service_uuid->type == BT_UUID_TYPE_16)
    {
        uint16_t uuid_tmp ;//= ((bt_uuid_16_t*)service_uuid)->val;
        memcpy(&uuid_tmp, &((bt_uuid_16_t*)service_uuid)->val, sizeof(uuid_tmp));
        if (BT_UUID_GAP_VAL == uuid_tmp ||
            BT_UUID_GATT_VAL == uuid_tmp)
        {
            return false;
        }
    }

    conn = bt_conn_lookup_addr_le(device->bt_le_address());
    if (NULL == conn)
    {
        // Link lost
        pr_debug(LOG_MODULE_BLE, "Can't find connection\n");
        return false;
    }
    temp = &_discover_params;
    temp->start_handle = _value_handle + 1;
    temp->end_handle = _value_handle + 20; // TODO: the max descriptor is not more than 20
    temp->uuid = NULL;
    temp->type = BT_GATT_DISCOVER_DESCRIPTOR;
    temp->func = profile_discover_process;
    pr_debug(LOG_MODULE_BLE, "%s-%d-charc",__FUNCTION__, __LINE__);
    err = bt_gatt_discover(conn, temp);
    bt_conn_unref(conn);
    if (err)
    {
        pr_debug(LOG_MODULE_BLE, "Discover failed(err %d)\n", err);
        return false;
    }
    return true;
}

bool BLECharacteristicImp::isClientCharacteristicConfigurationDescriptor(const bt_uuid_t* uuid)
{
    bool ret = false;
    uint16_t cccd_uuid = BT_UUID_GATT_CCC_VAL;
    if (uuid->type == BT_UUID_TYPE_16)
    {
        if (0 == memcmp(&BT_UUID_16(uuid)->val, &cccd_uuid, sizeof(uint16_t)))
        {
            ret = true;
        }
    }
    return ret;
}

uint8_t BLECharacteristicImp::discoverResponseProc(bt_conn_t *conn,
                                                   const bt_gatt_attr_t *attr,
                                                   bt_gatt_discover_params_t *params)
{
    const bt_addr_le_t* dst_addr = bt_conn_get_dst(conn);
    BLEDevice device(dst_addr);
    uint8_t retVal = BT_GATT_ITER_STOP;
    
    pr_debug(LOG_MODULE_BLE, "%s-%d: type-%d", __FUNCTION__, __LINE__, params->type);

    // Process the service
    switch (params->type)
    {
        case BT_GATT_DISCOVER_DESCRIPTOR:
        {
            if (NULL != attr)
            {
                retVal = BT_GATT_ITER_CONTINUE;
                const bt_uuid_t* desc_uuid = attr->uuid;
                uint16_t desc_handle = attr->handle;
    pr_debug(LOG_MODULE_BLE, "%s-%d:handle-%d:%d", __FUNCTION__, __LINE__,attr->handle, desc_handle);
                if (isClientCharacteristicConfigurationDescriptor(desc_uuid))
                {
                    setCCCDHandle(desc_handle);
                }
                else if (bt_uuid_cmp(BLEServiceImp::getPrimayUuid(), desc_uuid) == 0 ||
                         bt_uuid_cmp(getCharacteristicAttributeUuid(), desc_uuid) == 0 )
                {
                    retVal = BT_GATT_ITER_STOP;
                }
                else
                {
                    int retval = (int)addDescriptor(desc_uuid,
                                                    attr->perm,
                                                    desc_handle);
                    
                    if (BLE_STATUS_SUCCESS != retval)
                    {
                        pr_error(LOG_MODULE_BLE, "%s-%d: Error-%d", 
                                 __FUNCTION__, __LINE__, retval);
                        errno = ENOMEM;
                        retVal = BT_GATT_ITER_STOP;
                    }
                    
                }
            }
            break;
        }
        default:
        {
            break;
        }
    }
    return retVal;
}


