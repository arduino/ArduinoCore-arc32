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

#include "internal/ble_client.h"

#include "BLEServiceImp.h"
#include "BLECallbacks.h"
#include "BLEUtils.h"
#include "BLECharacteristicImp.h"

bt_uuid_16_t BLEServiceImp::_gatt_primary_uuid = {BT_UUID_TYPE_16, BT_UUID_GATT_PRIMARY_VAL};
bt_gatt_read_params_t BLEServiceImp::_read_params;

bt_uuid_t *BLEServiceImp::getPrimayUuid(void)
{
    return (bt_uuid_t *)&_gatt_primary_uuid;
}

BLEServiceImp::BLEServiceImp(BLEService& service): 
    BLEAttribute(service.uuid(), BLETypeService), 
    _start_handle(0),
    _end_handle(0xFFFF),
    _reading(false),
    _cur_discover_chrc(NULL),
    _discovering(false)
{
    memset(&_characteristics_header, 0, sizeof(_characteristics_header));
    service.setServiceImp(this);
}

BLEServiceImp::BLEServiceImp(const bt_uuid_t* uuid): 
    BLEAttribute(uuid, BLETypeService),
    _start_handle(0),
    _end_handle(0xFFFF),
    _reading(false),
    _cur_discover_chrc(NULL),
    _discovering(false)
{
    memset(&_characteristics_header, 0, sizeof(_characteristics_header));
}

BLEServiceImp::~BLEServiceImp()
{
    releaseCharacteristic();
}


int BLEServiceImp::addCharacteristic(BLEDevice& bledevice, BLECharacteristic& characteristic)
{
    BLECharacteristicImp* characteristicImp = NULL;
    
    characteristicImp = characteristic.fetchCharacteristicImp();
    if (NULL == characteristicImp)
    {
        characteristicImp = new BLECharacteristicImp(characteristic, bledevice);
        pr_debug(LOG_MODULE_BLE, "%s-%d",__FUNCTION__, __LINE__);
        if (NULL == characteristicImp)
        {
            return BLE_STATUS_NO_MEMORY;
        }
    }
    
    BLECharacteristicNodePtr node = link_node_create(characteristicImp);
    if (NULL == node)
    {
        delete characteristicImp;
        return BLE_STATUS_NO_MEMORY;
    }
    link_node_insert_last(&_characteristics_header, node);
    pr_debug(LOG_MODULE_BLE, "%s-%d",__FUNCTION__, __LINE__);
    return BLE_STATUS_SUCCESS;
}

int BLEServiceImp::addCharacteristic(BLEDevice& bledevice, 
                                     const bt_uuid_t* uuid, 
                                     uint16_t handle, 
                                     unsigned char properties)
{
    BLECharacteristicImp* characteristicImp = NULL;
    
    pr_debug(LOG_MODULE_BLE, "%s-%d:handle-%d",__FUNCTION__, __LINE__,handle);
    characteristicImp = new BLECharacteristicImp(uuid, 
                                                 properties, 
                                                 handle, 
                                                 bledevice);
    if (NULL == characteristicImp)
    {
        return BLE_STATUS_NO_MEMORY;
    }
    
    BLECharacteristicNodePtr node = link_node_create(characteristicImp);
    if (NULL == node)
    {
        delete characteristicImp;
        return BLE_STATUS_NO_MEMORY;
    }
    link_node_insert_last(&_characteristics_header, node);
    pr_debug(LOG_MODULE_BLE, "%s-%d",__FUNCTION__, __LINE__);
    return BLE_STATUS_SUCCESS;
}

int BLEServiceImp::updateProfile(bt_gatt_attr_t *attr_start, int& index)
{
    bt_gatt_attr_t *start = attr_start;
    int base_index = index;
    int offset = 0;
    int counter = 0;
    int ret = 0;
    start->uuid = BLEServiceImp::getPrimayUuid();
    start->perm = BT_GATT_PERM_READ;
    start->read = bt_gatt_attr_read_service;
    start->user_data = (void *)bt_uuid();

    pr_debug(LOG_MODULE_BLE, "service-%p", start);
    start++;
    index++;
    counter++;
    
    BLECharacteristicNodePtr node = _characteristics_header.next;
    while (NULL != node)
    {
        BLECharacteristicImp *characteristicImp = node->value;
        start = attr_start + index - base_index;
        offset = characteristicImp->updateProfile(start, index);
        counter += offset;
        node = node->next;
    }
    ret = bt_gatt_register(attr_start,
                           counter);
    
    return errorno_to_ble_status(ret);
}

int BLEServiceImp::getAttributeCount()
{
    int counter = 1;  // Service itself
    
    BLECharacteristicNodePtr node = _characteristics_header.next;
    while (NULL != node)
    {
        BLECharacteristicImp *characteristicImp = node->value;
        
        counter += characteristicImp->getAttributeCount();
        node = node->next;
    }
    return counter;
}

int BLEServiceImp::getCharacteristicCount()
{
    return link_list_size(&_characteristics_header);
}

void BLEServiceImp::releaseCharacteristic()
{
    BLECharacteristicNodePtr node = link_node_get_first(&_characteristics_header);
    pr_debug(LOG_MODULE_BLE, "%s-%d", __FUNCTION__, __LINE__);
    while (NULL != node)
    {
        BLECharacteristicImp* characteristicImp = node->value;
        delete characteristicImp;
        link_node_remove_first(&_characteristics_header);
        node = link_node_get_first(&_characteristics_header);
    }
    pr_debug(LOG_MODULE_BLE, "%s-%d", __FUNCTION__, __LINE__);
}


BLEDescriptorImp* BLEServiceImp::descriptor(uint16_t handle)
{
    BLEDescriptorImp* descriptorImp = NULL;
    
    BLECharacteristicImp* characteristicImp = NULL;
    BLECharacteristicNodePtr node = link_node_get_first(&_characteristics_header);
    while (NULL != node)
    {
        characteristicImp = node->value;
        descriptorImp = characteristicImp->descriptor(handle);
        if (descriptorImp != NULL)
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

BLECharacteristicImp* BLEServiceImp::characteristic(int index)
{
    BLECharacteristicImp* characteristicImp = NULL;
    BLECharacteristicNodePtr node = link_node_get_first(&_characteristics_header);
    while (NULL != node)
    {
        if (0 >= index)
        {
            characteristicImp = node->value;
            break;
        }
        index--;
        node = node->next;
    }
    return characteristicImp;
}

BLECharacteristicImp* BLEServiceImp::characteristic(uint16_t handle)
{
    BLECharacteristicImp* characteristicImp = NULL;
    BLECharacteristicNodePtr node = link_node_get_first(&_characteristics_header);
    while (NULL != node)
    {
        characteristicImp = node->value;
        if (handle == characteristicImp->valueHandle())
        {
            break;
        }
        node = node->next;
    }
    if (NULL == node)
    {
        characteristicImp = NULL;
    }
    return characteristicImp;
}

BLECharacteristicImp* BLEServiceImp::characteristic(const bt_uuid_t* uuid)
{
    BLECharacteristicImp* characteristicImp = NULL;
    BLECharacteristicNodePtr node = link_node_get_first(&_characteristics_header);
    
    while (NULL != node)
    {
        characteristicImp = node->value;
        if (true == characteristicImp->compareUuid(uuid))
        {
            break;
        }
        node = node->next;
    }
    
    if (NULL == node)
    {
        characteristicImp = NULL;
    }
    return characteristicImp;
}

BLECharacteristicImp* BLEServiceImp::characteristic(const char* uuid)
{
    bt_uuid_128_t uuid_tmp;
    BLEUtils::uuidString2BT(uuid, (bt_uuid_t *)&uuid_tmp);
    return characteristic((const bt_uuid_t *)&uuid_tmp);
}

bool BLEServiceImp::discovering()
{
    return (_cur_discover_chrc != NULL || 
            _reading || 
            _discovering);
}

void BLEServiceImp::setDiscovering(bool enable)
{
    _discovering = enable;
}

bool BLEServiceImp::discoverAttributes(BLEDevice* device)
{
    return discoverAttributes(device, _start_handle, _end_handle);
}

bool BLEServiceImp::discoverAttributes(BLEDevice* device, 
                                       uint16_t start_handle, 
                                       uint16_t end_handle)
{
    pr_debug(LOG_MODULE_BLE, "%s-%d", __FUNCTION__, __LINE__);
    int err;
    bt_conn_t* conn;
    bt_gatt_discover_params_t* temp = NULL;
    const bt_uuid_t* service_uuid = bt_uuid();
    
    // Filt out GAP/GATT service
    if (service_uuid->type == BT_UUID_TYPE_16)
    {
        uint16_t uuid_tmp;// = ((bt_uuid_16_t*)service_uuid)->val;
        
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
    temp->start_handle = start_handle;
    temp->end_handle = end_handle;
    temp->uuid = NULL;
    temp->type = BT_GATT_DISCOVER_CHARACTERISTIC;
    temp->func = profile_discover_process;
    
    err = bt_gatt_discover(conn, temp);
    bt_conn_unref(conn);
    if (err)
    {
        pr_debug(LOG_MODULE_BLE, "Discover failed(err %d)\n", err);
        return false;
    }
    setDiscovering(true);
    return true;
}

uint8_t BLEServiceImp::discoverResponseProc(bt_conn_t *conn,
                                            const bt_gatt_attr_t *attr,
                                            bt_gatt_discover_params_t *params)
{
    const bt_addr_le_t* dst_addr = bt_conn_get_dst(conn);
    BLEDevice device(dst_addr);
    uint8_t retVal = BT_GATT_ITER_STOP;
    
    //pr_debug(LOG_MODULE_BLE, "%s-%d: type-%d", __FUNCTION__, __LINE__, params->type);

    // Process the service
    switch (params->type)
    {
        case BT_GATT_DISCOVER_CHARACTERISTIC:
        {
            setDiscovering(false);
            if (NULL != attr)
            {
                //const bt_uuid_t* chrc_uuid = attr->uuid;
                uint16_t chrc_handle = attr->handle + 1;
                struct bt_gatt_chrc* psttemp = (struct bt_gatt_chrc*)attr->user_data;
                const bt_uuid_t* chrc_uuid = psttemp->uuid;
                
                uint16_t le16;
                memcpy(&le16, &BT_UUID_16(chrc_uuid)->val, sizeof(le16));
                if (chrc_uuid->type == BT_UUID_TYPE_16 && 
                    le16 == 0)
                {
                    // Read the UUID
                    readCharacteristic(device, chrc_handle);
                    //retVal = BT_GATT_ITER_CONTINUE;
                }
                else
                {
                    int retval = (int)addCharacteristic(device, 
                                                        psttemp->uuid,
                                                        chrc_handle,
                                                        psttemp->properties);
                    
                    if (BLE_STATUS_SUCCESS != retval)
                    {
                        pr_error(LOG_MODULE_BLE, "%s-%d: Error-%d", 
                                 __FUNCTION__, __LINE__, retval);
                        errno = ENOMEM;
                    }
                    else
                    {
                        retVal = BT_GATT_ITER_CONTINUE;
                    }
                }
            }
            break;
        }
        case BT_GATT_DISCOVER_DESCRIPTOR:
        {   
            if (NULL != _cur_discover_chrc)
            {
                retVal = _cur_discover_chrc->discoverResponseProc(conn, 
                                                                  attr, 
                                                                  params);
            }
            break;
        }
        default:
        {
            //attribute_tmp->discover(attr, &_discover_params);
            break;
        }
    }
    
    //pr_debug(LOG_MODULE_BLE, "%s-%d:ret-%d",__FUNCTION__, __LINE__, retVal);
    if (retVal == BT_GATT_ITER_STOP)
    {
        if (errno == ENOMEM)
        {
            _cur_discover_chrc = NULL;
            return retVal;
        }
        
        if (false == _reading)
        {
            discoverNextCharacteristic(device);
        }
    }
    return retVal;
}

void BLEServiceImp::discoverNextCharacteristic(BLEDevice &bledevice)
{
    const BLECharacteristicLinkNodeHeader* chrcHeader = &_characteristics_header;
    BLECharacteristicImp* chrcCurImp = NULL;
    BLECharacteristicNodePtr node = chrcHeader->next;
    
    // Discover next service
    while (node != NULL)
    {
        chrcCurImp = node->value;
        
        if (NULL == _cur_discover_chrc)
        {
            bool result = chrcCurImp->discoverAttributes(&bledevice);
            if (result == true)
            {
                // Record the current discovering service
                _cur_discover_chrc = chrcCurImp;
                break;
            }
        }
        else if (_cur_discover_chrc == chrcCurImp)
        {
            // Find next discoverable service
            _cur_discover_chrc = NULL;
        }
        node = node->next;
    }
}

bool BLEServiceImp::readCharacteristic(const BLEDevice &bledevice, uint16_t handle)
{
    int retval = 0;
    bt_conn_t* conn = NULL;
    
    if (true == BLEUtils::isLocalBLE(bledevice))
    {
        // GATT server can't write
        return false;
    }
    
    if (_reading)
    {
        return false;
    }
    
    _read_params.func = profile_characteristic_read_rsp_process;
    _read_params.handle_count = 1;
    _read_params.single.handle = handle - 1;
    _read_params.single.offset = 0;
    
    if (0 == _read_params.single.handle)
    {
        // Discover not complete
        return false;
    }
    
    conn = bt_conn_lookup_addr_le(bledevice.bt_le_address());
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
    pr_debug(LOG_MODULE_BLE, "%s-%d", __FUNCTION__, __LINE__);
    return _reading;
}

uint8_t BLEServiceImp::characteristicReadRspProc(bt_conn_t *conn, 
                                           int err,
                                           bt_gatt_read_params_t *params,
                                           const void *data, 
                                           uint16_t length)
{
    _reading = false;
    if (NULL == data)
    {
        return BT_GATT_ITER_STOP;
    }
    BLEDevice bleDevice(bt_conn_get_dst(conn));
    
    pr_debug(LOG_MODULE_BLE, "%s-%d:length-%d", __FUNCTION__, __LINE__, length);
    if (length == UUID_SIZE_128 + 3)
    {
        const uint8_t* rspdata = (const uint8_t*) data;
        bt_uuid_128_t uuid_tmp;
        uint16_t chrc_handle = rspdata[1] | (rspdata[2] << 8);
        uuid_tmp.uuid.type = BT_UUID_TYPE_128;
        memcpy(uuid_tmp.val, &rspdata[3], UUID_SIZE_128);
        int retval = (int)addCharacteristic(bleDevice, 
                                            (const bt_uuid_t*)&uuid_tmp,
                                            chrc_handle,
                                            rspdata[0]);
        
        if (BLE_STATUS_SUCCESS != retval)
        {
            pr_error(LOG_MODULE_BLE, "%s-%d: Error-%d", 
                     __FUNCTION__, __LINE__, retval);
            errno = ENOMEM;
        }
        else
        {
            if (false == discovering())
            {
                if (false == discoverAttributes(&bleDevice, chrc_handle + 1, _end_handle))
                {
                    discoverNextCharacteristic(bleDevice);
                }
            }
        }
    }
    pr_debug(LOG_MODULE_BLE, "%s-%d", __FUNCTION__, __LINE__);
    
    return BT_GATT_ITER_STOP;
}

