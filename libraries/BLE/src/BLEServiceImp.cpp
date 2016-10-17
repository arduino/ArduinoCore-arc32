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

#include "internal/ble_client.h"

#include "BLEServiceImp.h"
#include "BLECallbacks.h"
#include "BLEUtils.h"
#include "BLECharacteristicImp.h"

bt_uuid_16_t BLEServiceImp::_gatt_primary_uuid = {BT_UUID_TYPE_16, BT_UUID_GATT_PRIMARY_VAL};

bt_uuid_t *BLEServiceImp::getPrimayUuid(void)
{
    return (bt_uuid_t *)&_gatt_primary_uuid;
}

BLEServiceImp::BLEServiceImp(BLEService& service): 
    BLEAttribute(service.uuid(), BLETypeService), 
    _start_handle(0),
    _end_handle(0xFFFF)
{
    memset(&_characteristics_header, 0, sizeof(_characteristics_header));
    service.setServiceImp(this);
}

BLEServiceImp::BLEServiceImp(const bt_uuid_t* uuid): 
    BLEAttribute(uuid, BLETypeService),
    _start_handle(0),
    _end_handle(0xFFFF)
{
    memset(&_characteristics_header, 0, sizeof(_characteristics_header));
}

BLEServiceImp::~BLEServiceImp()
{
    releaseCharacteristic();
}


int BLEServiceImp::addCharacteristic(BLEDevice& bledevice, BLECharacteristic& characteristic)
{
    BLECharacteristicImp* characteristicImp = new BLECharacteristicImp(characteristic, bledevice);
    pr_debug(LOG_MODULE_BLE, "%s-%d",__FUNCTION__, __LINE__);
    if (NULL == characteristicImp)
    {
        return BLE_STATUS_NO_MEMORY;
    }
    
    BLECharacteristicNodePtr node = link_node_create(characteristicImp);
    if (NULL == node)
    {
        delete[] characteristicImp;
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
    return counter;
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
    
    while (NULL != node)
    {
        BLECharacteristicImp* characteristicImp = node->value;
        delete[] characteristicImp;
        link_node_remove_first(&_characteristics_header);
        node = link_node_get_first(&_characteristics_header);
    }
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

BLECharacteristicImp* BLEServiceImp::characteristic(const char* uuid)
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

bool BLEServiceImp::discoverAttributes(BLEDevice* device)
{
    
    int err;
    bt_conn_t* conn;
    bt_gatt_discover_params_t* temp = NULL;
    const bt_uuid_t* service_uuid = bt_uuid();
    
    if (service_uuid->type == BT_UUID_TYPE_16)
    {
        uint16_t uuid_tmp = ((bt_uuid_16_t*)service_uuid)->val;
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
    temp->start_handle = _start_handle;
    temp->end_handle = _end_handle;
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
    return true;
}

uint8_t BLEServiceImp::discoverResponseProc(bt_conn_t *conn,
                                            const bt_gatt_attr_t *attr,
                                            bt_gatt_discover_params_t *params)
{
    const bt_addr_le_t* dst_addr = bt_conn_get_dst(conn);
    BLEDevice device(dst_addr);
    uint8_t retVal = BT_GATT_ITER_STOP;
    
    pr_debug(LOG_MODULE_BLE, "%s-%d", __FUNCTION__, __LINE__);
    if (NULL == attr)
    {
        return BT_GATT_ITER_STOP;
    }
    const bt_uuid_t* chrc_uuid = attr->uuid;
    struct bt_gatt_chrc* psttemp = (struct bt_gatt_chrc*)attr->user_data;
    char uuidbuffer[37];
    BLEUtils::uuidBT2String(chrc_uuid, uuidbuffer);
    pr_debug(LOG_MODULE_BLE, "%s-%d: uuid type: %d\r\n%s", __FUNCTION__, __LINE__, chrc_uuid->type, uuidbuffer);
    
    BLEUtils::uuidBT2String(psttemp->uuid, uuidbuffer);
    pr_debug(LOG_MODULE_BLE, "%s-%d:%p uuid type: %d properties: %d\r\n%s", __FUNCTION__, __LINE__,psttemp, psttemp->uuid->type, psttemp->properties, uuidbuffer);
    // Process the service
    switch (params->type)
    {
        case BT_GATT_DISCOVER_CHARACTERISTIC:
        {
            //characteristicDiscoverRsp(attr, attribute_tmp);
            //send_discover = true;
            break;
        }
        default:
        {
            //attribute_tmp->discover(attr, &_discover_params);
            break;
        }
    }
    
    if (retVal == BT_GATT_ITER_STOP)
    {
        const BLECharacteristicLinkNodeHeader* serviceHeader = &_characteristics_header;
        BLECharacteristicImp* chrcCurImp = NULL;
        BLECharacteristicNodePtr node = serviceHeader->next;
        
        // Discover next service
        while (node != NULL)
        {
            chrcCurImp = node->value;
            
            if (NULL == _cur_discover_chrc)
            {
                bool result = true;//serviceCurImp->discoverAttributes(&device);
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
    return retVal;
}


