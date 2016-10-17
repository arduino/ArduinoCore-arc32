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
#include "BLECommon.h"
#include "BLEProfileManager.h"
#include "BLECharacteristicImp.h"

#include "BLECallbacks.h"
#include "BLEUtils.h"

BLEDevice BLE;

BLEProfileManager* BLEProfileManager::_instance = NULL;

BLEProfileManager* BLEProfileManager::instance()
{
    if (NULL == _instance)
    {
        _instance = new BLEProfileManager();
        BLE_LIB_ASSERT(_instance != NULL);
    }
    //pr_debug(LOG_MODULE_BLE, "%s-%d: %p", __FUNCTION__, __LINE__, _instance);
    return _instance;
}

BLEProfileManager::BLEProfileManager ():
    _start_discover(false),
    _discovering(false),
    _discover_rsp_timestamp(0),
    _cur_discover_service(NULL),
    _discover_one_service(false),
    _reading(false),
    _attr_base(NULL),
    _attr_index(0),
    _profile_registered(false),
    _disconnect_bitmap(0)
{
    //memset(_service_header_array, 0, sizeof(_service_header_array));
    memset(_discover_params, 0, sizeof(_discover_params));
    memset(_discover_uuid, 0, sizeof(_discover_uuid));
    
    memset(_addresses, 0, sizeof(_addresses));
    memset(&_discovering_ble_addresses, 0, sizeof(_discovering_ble_addresses));
    memset(&_read_params, 0, sizeof(_read_params));
    memset(&_read_service_header, 0, sizeof(_read_service_header));
    bt_addr_le_copy(&_addresses[BLE_MAX_CONN_CFG], BLEUtils::bleGetLoalAddress());
    for (int i = 0; i <= BLE_MAX_CONN_CFG; i++)
    {
        _service_header_array[i].next = NULL;
        _service_header_array[i].value = NULL;
    }
    
    pr_debug(LOG_MODULE_BLE, "%s-%d: Construct", __FUNCTION__, __LINE__);
}

BLEProfileManager::~BLEProfileManager (void)
{
    if (_attr_base)
    {
      free(_attr_base);
      _attr_base = (bt_gatt_attr_t *)NULL;
    }
    ServiceReadLinkNodePtr node = link_node_get_first(&_read_service_header);
    while (NULL != node)
    {
        link_node_remove_first(&_read_service_header);
        node = link_node_get_first(&_read_service_header);
    }
}

BLEServiceImp *
BLEProfileManager::addService (BLEDevice &bledevice, BLEService& service)
{
    
    BLEServiceLinkNodeHeader* serviceheader = getServiceHeader(bledevice);
    if (NULL == serviceheader)
    {
        int index = getUnusedIndex();
        if (index >= BLE_MAX_CONN_CFG)
        {
            return NULL;
        }
        serviceheader = &_service_header_array[index];
        bt_addr_le_copy(&_addresses[index], bledevice.bt_le_address());
    }
    BLEServiceImp *serviceImp = NULL;//this->service(bledevice, service.uuid());
    //if (NULL != serviceImp)
    //{
        // The service alreay exist
    //    return serviceImp;
    //}
    
    //if (NULL == serviceImp) // May trigger KW warning
    {
        serviceImp = service.fetchOutLocalServiceImp();
    }
    
    if (NULL == serviceImp)
    {
        serviceImp = new BLEServiceImp(service);
        if (NULL == serviceImp)
        {
            return serviceImp;
        }
    }
    
    BLEServiceNodePtr node = link_node_create(serviceImp);
    if (NULL == node)
    {
        delete serviceImp;
        return NULL;
    }
    link_node_insert_last(serviceheader, node);
    pr_debug(LOG_MODULE_BLE, "%s-%d", __FUNCTION__, __LINE__);
    return serviceImp;
}

BLEServiceImp *
BLEProfileManager::addService (BLEDevice &bledevice, const bt_uuid_t* uuid)
{
    BLEService svc_obj(uuid);
    return addService(bledevice, svc_obj);
}

BLEProfileManager::BLEServiceLinkNodeHeader* BLEProfileManager::getServiceHeader(const BLEDevice &bledevice)
{
    int i;
    for (i = 0; i <= BLE_MAX_CONN_CFG; i++)
    {
        if ((bt_addr_le_cmp(bledevice.bt_le_address(), &_addresses[i]) == 0))
        //if (true == BLEUtils::macAddressSame(*bledevice.bt_le_address(), _addresses[i]))
        {
            break;
        }
    }
    if (i > BLE_MAX_CONN_CFG)
    {
        return NULL;
    }
    return &_service_header_array[i];
}

const BLEProfileManager::BLEServiceLinkNodeHeader* BLEProfileManager::getServiceHeader(const BLEDevice &bledevice) const
{
    int i;
    for (i = 0; i <= BLE_MAX_CONN_CFG; i++)
    {
        if ((bt_addr_le_cmp(bledevice.bt_le_address(), &_addresses[i]) == 0))
        //if (true == BLEUtils::macAddressSame(*bledevice.bt_le_address(), _addresses[i]))
        {
            break;
        }
    }
    if (i > BLE_MAX_CONN_CFG)
    {
        return NULL;
    }
    return &_service_header_array[i];
}

int BLEProfileManager::getUnusedIndex()
{
    int i;
    for (i = 0; i < BLE_MAX_CONN_CFG; i++)
    {
        if (BLEUtils::macAddressValid(_addresses[i]) == false)
        {
            break;
        }
    }
    
    return i;
}

int BLEProfileManager::getAttributeCount(BLEDevice &bledevice)
{
    BLEServiceLinkNodeHeader* serviceHeader = getServiceHeader(bledevice);
    if (NULL == serviceHeader)
    {
        return 0;
    }
    
    int attrCounter = 0;
    
    BLEServiceNodePtr node = serviceHeader->next;
    while (node)
    {
        BLEServiceImp *service = node->value;
        attrCounter += service->getAttributeCount();
        node = node->next;
    }
    return attrCounter;
}

int BLEProfileManager::characteristicCount(const BLEDevice &bledevice) const
{
    const BLEServiceLinkNodeHeader* serviceHeader = getServiceHeader(bledevice);
    if (NULL == serviceHeader)
    {
        return 0;
    }
    
    int counter = 0;
    
    BLEServiceNodePtr node = serviceHeader->next;
    while (node)
    {
        BLEServiceImp *service = node->value;
        counter += service->getCharacteristicCount();
        node = node->next;
    }
    return counter;
}

int BLEProfileManager::serviceCount(const BLEDevice &bledevice) const
{
    const BLEServiceLinkNodeHeader* serviceHeader = getServiceHeader(bledevice);
    if (NULL == serviceHeader)
    {
        return 0;
    }
    return link_list_size(serviceHeader);
}

int BLEProfileManager::registerProfile(BLEDevice &bledevice)
{
    int ret = 0;
    
    bt_gatt_attr_t *start;
    BleStatus err_code = BLE_STATUS_SUCCESS;
    
    // The device is local BLE device. Register the service only allow local BLE device
    BLEServiceLinkNodeHeader* serviceHeader = &_service_header_array[BLE_MAX_CONN_CFG];
    if ((bt_addr_le_cmp(bledevice.bt_le_address(), &_addresses[BLE_MAX_CONN_CFG]) != 0))
    {
        return BLE_STATUS_FORBIDDEN;
    }
    
    int attr_counter = getAttributeCount(bledevice);
    if (0 == attr_counter)
    {
        return BLE_STATUS_NO_SERVICE;
    }
    
    if (NULL == _attr_base)
    {
        _attr_base = (bt_gatt_attr_t *)malloc(attr_counter * sizeof(bt_gatt_attr_t));
        if (NULL == _attr_base) {
            err_code = BLE_STATUS_NO_MEMORY;
        }
        else 
        {
            memset((void *)_attr_base, 0x00, (attr_counter * sizeof(bt_gatt_attr_t)));
            pr_info(LOG_MODULE_BLE, "_attr_base_-%p, size-%d, attr_counter-%d", _attr_base, sizeof(_attr_base), attr_counter);
        }
    }
    
    if (BLE_STATUS_SUCCESS != err_code)
    {
        if (NULL != _attr_base)
        {
            free(_attr_base);
        }
        return err_code;
    }
    
    pr_info(LOG_MODULE_BLE, "_attr_base_-%p", _attr_base);

    BLEServiceNodePtr node = serviceHeader->next;
    while (node)
    {
        BLEServiceImp *service = node->value;
        start = _attr_base + _attr_index;
        service->updateProfile(start, _attr_index);
        node = node->next;
    }
    
#if 0
    // Start debug
    int i;

    for (i = 0; i < _attr_index; i++) {
        {
            pr_info(LOG_MODULE_APP, "gatt-: i %d, type %d, u16 0x%x",
                   i, 
                   _attr_base[i].uuid->type,
                   BT_UUID_16(_attr_base[i].uuid)->val);
        }
    }
    
    delay(1000);
    // End for debug
#endif
    
    ret = bt_gatt_register(_attr_base,
                            _attr_index);
    pr_debug(LOG_MODULE_APP, "%s: ret, %d,_attr_index-%d", __FUNCTION__, ret, _attr_index);
    if (0 == ret)
    {
        _profile_registered = true;
    }
    return ret;
}

void BLEProfileManager::clearProfile(BLEServiceLinkNodeHeader* serviceHeader)
{
    if (NULL == serviceHeader)
    {
        return;
    }
    
    BLEServiceNodePtr node = link_node_get_first(serviceHeader);
    
    while (NULL != node)
    {
        BLEServiceImp *service = node->value;
        delete service;
        link_node_remove_first(serviceHeader);
        node = link_node_get_first(serviceHeader);
    }
}

BLECharacteristicImp* BLEProfileManager::characteristic(const BLEDevice &bledevice, int index)
{
    BLECharacteristicImp* characteristicImp = NULL;
    BLEServiceLinkNodeHeader* serviceHeader = getServiceHeader(bledevice);
    if (NULL == serviceHeader)
    {
        // Doesn't find the service
        return NULL;
    }
    int counter = 0;
    BLEServiceNodePtr node = serviceHeader->next;
    while (node != NULL)
    {
        BLEServiceImp *service = node->value;
        int counterTmp = service->getCharacteristicCount();
        if (counter + counterTmp > index)
        {
            break;
        }
        counter += counterTmp;
        node = node->next;
    }

    if (NULL != node)
    {
        BLEServiceImp *service = node->value;
        characteristicImp = service->characteristic(index - counter);
    }
    return characteristicImp;
}

BLECharacteristicImp* BLEProfileManager::characteristic(const BLEDevice &bledevice, uint16_t handle)
{
    BLECharacteristicImp* characteristicImp = NULL;
    BLEServiceLinkNodeHeader* serviceHeader = getServiceHeader(bledevice);
    if (NULL == serviceHeader)
    {
        // Doesn't find the service
        return NULL;
    }
    
    BLEServiceNodePtr node = serviceHeader->next;
    while (node != NULL)
    {
        BLEServiceImp *service = node->value;
        characteristicImp = service->characteristic(handle);
        if (NULL != characteristicImp)
        {
            break;
        }
        node = node->next;
    }
    return characteristicImp;
}

BLECharacteristicImp* BLEProfileManager::characteristic(const BLEDevice &bledevice, 
                                                        const char* uuid, 
                                                        int index)
{
    BLECharacteristicImp* characteristicImp = characteristic(bledevice, index);
    if (NULL != characteristicImp)
    {
        if (false == characteristicImp->compareUuid(uuid))
        {
            // UUID not align
            characteristicImp = NULL;
        }
    }
    return characteristicImp;
}

BLECharacteristicImp* BLEProfileManager::characteristic(const BLEDevice &bledevice, 
                                                        const char* uuid)
{
    BLECharacteristicImp* characteristicImp = NULL;
    BLEServiceLinkNodeHeader* serviceHeader = getServiceHeader(bledevice);
    if (NULL == serviceHeader)
    {
        // Doesn't find the service
        return NULL;
    }
    BLEServiceNodePtr node = serviceHeader->next;
    while (node != NULL)
    {
        BLEServiceImp *service = node->value;
        characteristicImp = service->characteristic(uuid);
        if (NULL != characteristicImp)
        {
            break;
        }
        node = node->next;
    }
    
    return characteristicImp;
}

BLEServiceImp* BLEProfileManager::service(const BLEDevice &bledevice, const char * uuid) const
{
    bt_uuid_128_t uuid_tmp;
    BLEUtils::uuidString2BT(uuid, (bt_uuid_t *)&uuid_tmp);
    //pr_debug(LOG_MODULE_BLE, "%s-%d", __FUNCTION__, __LINE__);
    return service(bledevice, (const bt_uuid_t *)&uuid_tmp);
}

BLEServiceImp* BLEProfileManager::service(const BLEDevice &bledevice, const bt_uuid_t* uuid) const
{
    BLEServiceImp* serviceImp = NULL;
    #if 1
    const BLEServiceLinkNodeHeader* serviceHeader = getServiceHeader(bledevice);
    if (NULL == serviceHeader)
    {
        // Doesn't find the service
        return NULL;
    }
    BLEServiceNodePtr node = serviceHeader->next;
    
    // Just for debug
    char uuid_tmp[37];
    BLEUtils::uuidBT2String(uuid, uuid_tmp);
    pr_debug(LOG_MODULE_BLE, "%s-%d: %s", __FUNCTION__, __LINE__, uuid_tmp);
    
    while (node != NULL)
    {
        serviceImp = node->value;
        if (true == serviceImp->compareUuid(uuid))
        {
            break;
        }
        node = node->next;
    }
    
    if (NULL == node)
    {
        serviceImp = NULL;
    }
    #endif
    return serviceImp;
}

BLEServiceImp* BLEProfileManager::service(const BLEDevice &bledevice, int index) const
{
    BLEServiceImp* serviceImp = NULL;
    const BLEServiceLinkNodeHeader* serviceHeader = getServiceHeader(bledevice);
    if (NULL == serviceHeader)
    {
        // Doesn't find the service
        return NULL;
    }
    BLEServiceNodePtr node = serviceHeader->next;
    
    while (node != NULL)
    {
        if (0 == index)
        {
            break;
        }
        index--;
        node = node->next;
    }
    if (NULL == node)
    {
        serviceImp = NULL;
    }
    else
    {
        serviceImp = node->value;
    }
    return serviceImp;
}

void BLEProfileManager::handleConnectedEvent(const bt_addr_le_t* deviceAddr)
{
    int index = getUnusedIndex();
    if (index >= BLE_MAX_CONN_CFG)
    {
        //BLE_STATUS_NO_MEMORY
        return;
    }
    bt_addr_le_copy(&_addresses[index], deviceAddr);
}

void BLEProfileManager::handleDisconnectedEvent(const bt_addr_le_t* deviceAddr)
{    
    int i;
    if ((bt_addr_le_cmp(deviceAddr, &_discovering_ble_addresses) == 0))
    {
        _start_discover = false;
        memset(&_discovering_ble_addresses, 0, sizeof(_discovering_ble_addresses));
    }
    for (i = 0; i < BLE_MAX_CONN_CFG; i++)
    {
        if ((bt_addr_le_cmp(deviceAddr, &_addresses[i]) == 0))
        {
            bitSet(_disconnect_bitmap, i);
            break;
        }
    }

}

void BLEProfileManager::handleDisconnectedPutOffEvent()
{    
    BLEServiceLinkNodeHeader* serviceheader = NULL;
    int i;
    if (_disconnect_bitmap == 0)
    {
        return;
    }
    
    for (i = 0; i < BLE_MAX_CONN_CFG; i++)
    {
        if (bitRead(_disconnect_bitmap, i) != 0)
        {
            serviceheader = &_service_header_array[i];
            clearProfile(serviceheader);
            memset(&_addresses[i], 0, sizeof(bt_addr_le_t));
            bitClear(_disconnect_bitmap, i);
        }
    }
}

bool BLEProfileManager::discoverAttributes(BLEDevice* device)
{
    int err;
    bt_conn_t* conn;
    int i = getDeviceIndex(device);
    bool ret = false;
    bt_gatt_discover_params_t* temp = NULL;
    
    errno = 0;
    pr_debug(LOG_MODULE_BLE, "%s-%d: index-%d,fun-%p", __FUNCTION__, __LINE__, i,profile_discover_process);

    if (_start_discover)
    {
        // Already in discover state
        return false;
    }
    

    if (i >= BLE_MAX_CONN_CFG)
    {
        // The device already in the buffer.
        //  This function only be called after connection established.
        return ret;
    }
    
    conn = bt_conn_lookup_addr_le(device->bt_le_address());
    if (NULL == conn)
    {
        // Link lost
        pr_debug(LOG_MODULE_BLE, "Can't find connection\n");
        return ret;
    }
    temp = &_discover_params[i];
    temp->start_handle = 1;
    temp->end_handle = 0xFFFF;
    temp->uuid = NULL;
    temp->type = BT_GATT_DISCOVER_PRIMARY;
    temp->func = profile_discover_process;
    
    err = bt_gatt_discover(conn, temp);
    bt_conn_unref(conn);
    if (err)
    {
        pr_debug(LOG_MODULE_BLE, "Discover failed(err %d)\n", err);
        return ret;
    }
    // Block it 
    memcpy(&_discovering_ble_addresses, device->bt_le_address(), sizeof(_discovering_ble_addresses));
    _discover_rsp_timestamp = millis();
    _start_discover = true;
    ret = true;
    while (_start_discover)  // Sid. KW warning acknowldged
    {
        delay(10);
        if ((millis() - _discover_rsp_timestamp) > 5000)
        {
            // Doesn't receive the Service read response
            _start_discover = false;
            _cur_discover_service = NULL;
            ret = false;
            _reading = false;
        }
        
        if (ENOMEM == errno)
        {
            pr_debug(LOG_MODULE_BLE, "%s-%d:Sys errno(err %d)\n", __FUNCTION__, __LINE__, errno);
            ret = false;
            break;
        }
    }
    return ret;
}

bool BLEProfileManager::discoverAttributesByService(BLEDevice* device, const bt_uuid_t* svc_uuid)
{
    errno = 0;
    if (_start_discover)
    {
        // Already in discover state
        return false;
    }
    
    bool ret = discoverService(device, svc_uuid);
    if (false == ret)
    {
        return false;
    }
    // Block it 
    memcpy(&_discovering_ble_addresses, device->bt_le_address(), sizeof(_discovering_ble_addresses));
    _discover_rsp_timestamp = millis();
    _start_discover = true;
    _discover_one_service = true;
    
    while (_start_discover)  // Sid. KW warning acknowldged
    {
        delay(10);
        if ((millis() - _discover_rsp_timestamp) > 5000)
        {
            // Doesn't receive the Service read response
            _start_discover = false;
            _cur_discover_service = NULL;
            ret = false;
            _reading = false;
        }
        
        if (ENOMEM == errno)
        {
            pr_debug(LOG_MODULE_BLE, "%s-%d:Sys errno(err %d)", __FUNCTION__, __LINE__, errno);
            ret = false;
            break;
        }
    }
    pr_debug(LOG_MODULE_BLE, "%s-%d:Discover Done", __FUNCTION__, __LINE__);
    _discover_one_service = false;
    
    return ret;
}


int BLEProfileManager::getDeviceIndex(const bt_addr_le_t* macAddr)
{
    int i;
    for (i = 0; i < BLE_MAX_CONN_CFG; i++)
    {
        if ((bt_addr_le_cmp(macAddr, &_addresses[i]) == 0))
        {
            break;
        }
    }
    return i;
}

int BLEProfileManager::getDeviceIndex(const BLEDevice* device)
{
    return getDeviceIndex(device->bt_le_address());
}

bool BLEProfileManager::discovering()
{
    bool ret = _discovering;
    if (_cur_discover_service != NULL)
    {
        ret = ret || _cur_discover_service->discovering();
    }
    return ret;
}

void BLEProfileManager::setDiscovering(bool discover)
{
    _discovering = discover;
}

uint8_t BLEProfileManager::discoverResponseProc(bt_conn_t *conn,
                                                const bt_gatt_attr_t *attr,
                                                bt_gatt_discover_params_t *params)
{
    const bt_addr_le_t* dst_addr = bt_conn_get_dst(conn);
    int i = getDeviceIndex(dst_addr);
    BLEDevice device(dst_addr);
    uint8_t retVal = BT_GATT_ITER_STOP;
    BLEServiceImp* service_tmp = NULL;
    _discover_rsp_timestamp = millis();
    //pr_debug(LOG_MODULE_BLE, "%s-%d: index-%d", __FUNCTION__, __LINE__, i);
    
    if (i >= BLE_MAX_CONN_CFG)
    {
        return BT_GATT_ITER_STOP;
    }

    // Process the service
    switch (params->type)
    {
        case BT_GATT_DISCOVER_CHARACTERISTIC:
        case BT_GATT_DISCOVER_DESCRIPTOR:
        {
            if (NULL != _cur_discover_service)
            {
                retVal = _cur_discover_service->discoverResponseProc(conn, 
                                                                     attr, 
                                                                     params);
            }
            break;
        }
        case BT_GATT_DISCOVER_PRIMARY:
        {
            if (NULL != attr)
            {
                struct bt_gatt_service *svc_value = (struct bt_gatt_service *)attr->user_data;
                const bt_uuid_t* svc_uuid = svc_value->uuid;
                uint16_t le16;
                memcpy(&le16, &BT_UUID_16(svc_uuid)->val, sizeof(le16));
                setDiscovering(false);
                
                if (svc_uuid->type == BT_UUID_TYPE_16 && 
                    le16 == 0)
                {
                    // Discover failed. The service may unknow type. 
                    //  Need read the value and discovery again.
                    readService(device, attr->handle);
                    retVal = BT_GATT_ITER_CONTINUE;
                }
                else
                {
                    service_tmp = addService(device, svc_value->uuid);
                    params->uuid = NULL;
                    
                    if (NULL != service_tmp)
                    {
                        service_tmp->setHandle(attr->handle);
                        service_tmp->setEndHandle(svc_value->end_handle);
                        if (_discover_one_service == false)
                            retVal = BT_GATT_ITER_CONTINUE;
                    }
                    else
                    {
                        retVal = BT_GATT_ITER_STOP;
                        errno = ENOMEM;
                        pr_debug(LOG_MODULE_BLE, "%s-%d: Add service failed", 
                                 __FUNCTION__, __LINE__);
                    }
                }
            }
            else
            {
                // Service discover complete
                retVal = BT_GATT_ITER_STOP;
            }
        }   
        default:
        {
            break;
        }
    }
    
    if (retVal == BT_GATT_ITER_STOP)
    {
        if (errno == ENOMEM)
        {
            // No memory. Stop discovery
            _cur_discover_service = NULL;
            _discover_one_service = false;
            return retVal;
        }
        
        pr_debug(LOG_MODULE_BLE, "%s-%d: Discover one service-%d", 
                             __FUNCTION__, __LINE__, _discover_one_service);
        if (true == _discover_one_service)
        {
            if (NULL != service_tmp)
            {
                pr_debug(LOG_MODULE_BLE, "%s-%d: Discover service", 
                                     __FUNCTION__, __LINE__);
                bool result = service_tmp->discoverAttributes(&device);
                if (result == true)
                {
                    // Record the current discovering service
                    _cur_discover_service = service_tmp;
                }
                else
                {
                    // Failed
                    _discover_one_service = false;
                }
            }
            else
            {
                if (discovering() == false)
                {
                    // Complete
                    _cur_discover_service = NULL;
                    _discover_one_service = false;
                }
            }
            
            if (_discover_one_service == false)
            {
                // Discover complete
                _start_discover = false;
                memset(&_discovering_ble_addresses, 0, sizeof(_discovering_ble_addresses));
            }
            return retVal;
        }
        
        checkReadService();
        if (discovering() == false)
        {
            const BLEServiceLinkNodeHeader* serviceHeader = getServiceHeader(device);
            BLEServiceImp* serviceCurImp = NULL;
            if (NULL == serviceHeader)
            {
                // Doesn't find the service
                return BT_GATT_ITER_STOP;
            }
            BLEServiceNodePtr node = serviceHeader->next;
            
            // Discover next service
            while (node != NULL)
            {
                serviceCurImp = node->value;
                
                if (NULL == _cur_discover_service)
                {
                    bool result = serviceCurImp->discoverAttributes(&device);
                    if (result == true)
                    {
                        // Record the current discovering service
                        _cur_discover_service = serviceCurImp;
                        break;
                    }
                }
                else if (_cur_discover_service == serviceCurImp)
                {
                    // Find next discoverable service
                    _cur_discover_service = NULL;
                }
                
                node = node->next;
            }
            if (NULL == node)
            {
                pr_debug(LOG_MODULE_BLE, "%s-%d: Discover completed", 
                                     __FUNCTION__, __LINE__);
                _start_discover = false;
                memset(&_discovering_ble_addresses, 0, sizeof(_discovering_ble_addresses));
            }
        }
    }
    return retVal;
}

void BLEProfileManager::serviceDiscoverComplete(const BLEDevice &bledevice)
{
    BLEServiceImp* serviceCurImp = NULL;
    BLEServiceImp* servicePrevImp = NULL;
    const BLEServiceLinkNodeHeader* serviceHeader = getServiceHeader(bledevice);
    if (NULL == serviceHeader)
    {
        // Doesn't find the service
        return ;
    }
    
    BLEServiceNodePtr node = serviceHeader->next;
    if (NULL != node)
    {
        servicePrevImp = node->value;
        node = node->next;  
    }
    
    // Update the service handles
    while (node != NULL)
    {
        serviceCurImp = node->value;
        if (NULL != serviceCurImp)
        {
            if (servicePrevImp)  // KW issue: Chk for NULL.
                servicePrevImp->setEndHandle(serviceCurImp->startHandle() - 1);
        }
        
        if (servicePrevImp)
        {
            pr_debug(LOG_MODULE_BLE, "Curr: start-%d, end-%d", servicePrevImp->startHandle(), servicePrevImp->endHandle());
        }
        servicePrevImp = serviceCurImp;
        if (servicePrevImp)  // KW issue: Chk for NULL.
            pr_debug(LOG_MODULE_BLE, "Curr: start-%d, end-%d", servicePrevImp->startHandle(), servicePrevImp->endHandle());
        node = node->next;
    }
    return;
}

bool BLEProfileManager::readService(const BLEDevice &bledevice, uint16_t handle)
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
        // Read response not back
        // Add to buffer
        ServiceRead_t temp;
        bt_addr_le_copy(&temp.address, bledevice.bt_le_address());
        temp.handle = handle;
        ServiceReadLinkNodePtr node = link_node_create(temp);
        link_node_insert_last(&_read_service_header, node);
        return true;
    }
    
    _read_params.func = profile_service_read_rsp_process;
    _read_params.handle_count = 1;
    _read_params.single.handle = handle;
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
        setDiscovering(true);
        _reading = true;
    }
    pr_debug(LOG_MODULE_BLE, "%s-%d", __FUNCTION__, __LINE__);
    return _reading;
}

void BLEProfileManager::checkReadService()
{
    ServiceReadLinkNodePtr node = link_node_get_first(&_read_service_header);
    while (NULL != node)
    {
        BLEDevice temp(&node->value.address);
        bool readResult = readService(temp, node->value.handle);
        link_node_remove_first(&_read_service_header);
        if (true == readResult)
        {
            break;
        }
        node = link_node_get_first(&_read_service_header);
    }
}

bool BLEProfileManager::discoverService(BLEDevice* device, const bt_uuid_t* svc_uuid)
{
    int err = 0;
    bt_conn_t* conn;
    int i = getDeviceIndex(device);
    bool ret = false;
    bt_gatt_discover_params_t* temp = NULL;
    
    pr_debug(LOG_MODULE_BLE, "%s-%d: index-%d,fun-%p", __FUNCTION__, __LINE__, i,profile_discover_process);

    if (i >= BLE_MAX_CONN_CFG)
    {
        // The device already in the buffer.
        //  This function only be called after connection established.
        return ret;
    }
    
    //BLEServiceImp* serviceImp = service(device, svc_uuid);
    //if (NULL == serviceImp)
    {
       //return ret;
    }
    
    conn = bt_conn_lookup_addr_le(device->bt_le_address());
    if (NULL == conn)
    {
        // Link lost
        pr_debug(LOG_MODULE_BLE, "Can't find connection\n");
        return ret;
    }
    
    memcpy(&_discover_uuid[i], svc_uuid, sizeof(bt_uuid_128_t));
    
    temp = &_discover_params[i];
    temp->start_handle = 1;
    temp->end_handle = 0xFFFF;
    temp->uuid = (bt_uuid_t*) &_discover_uuid[i];
    temp->type = BT_GATT_DISCOVER_PRIMARY;
    temp->func = profile_discover_process;
    
    err = bt_gatt_discover(conn, temp);
    bt_conn_unref(conn);
    if (err)
    {
        pr_debug(LOG_MODULE_BLE, "Discover failed(err %d)\n", err);
        return ret;
    }
    return true;
}

uint8_t BLEProfileManager::serviceReadRspProc(bt_conn_t *conn, 
                                           int err,
                                           bt_gatt_read_params_t *params,
                                           const void *data, 
                                           uint16_t length)
{
    _reading = false;
    _discover_rsp_timestamp = millis();
    if (NULL == data)
    {
        return BT_GATT_ITER_STOP;
    }
    BLEDevice bleDevice(bt_conn_get_dst(conn));
    
    pr_debug(LOG_MODULE_BLE, "%s-%d:length-%d", __FUNCTION__, __LINE__, length);
    if (length == UUID_SIZE_128)
    {
        bt_uuid_128_t uuid_tmp;
        uuid_tmp.uuid.type = BT_UUID_TYPE_128;
        memcpy(uuid_tmp.val, data, UUID_SIZE_128);
        BLEProfileManager::instance()->discoverService(&bleDevice, (const bt_uuid_t *)&uuid_tmp);
    }
    pr_debug(LOG_MODULE_BLE, "%s-%d", __FUNCTION__, __LINE__);
    
    return BT_GATT_ITER_STOP;
}



