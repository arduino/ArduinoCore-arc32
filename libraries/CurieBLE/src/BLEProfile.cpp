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
#include "BLEProfile.h"
#include "BLEPeripheral.h"
#include "BLECentralRole.h"
#include "BLEPeripheralRole.h"

// Only for peripheral
ssize_t profile_read_process(bt_conn_t *conn,
                             const bt_gatt_attr_t *attr,
                             void *buf, uint16_t len,
                             uint16_t offset)
{
    const unsigned char *pvalue;
    BLEAttribute *bleattr = (BLEAttribute *)attr->user_data;
    BLEAttributeType type = bleattr->type();
    if (BLETypeCharacteristic == type)
    {
        BLECharacteristic* blecharacteritic;
        blecharacteritic = (BLECharacteristic*)bleattr;
        pvalue = blecharacteritic->value();
        return bt_gatt_attr_read(conn, attr, buf, len, offset, pvalue,
                                 blecharacteritic->valueLength());
    }
    else if (BLETypeDescriptor == type)
    {
        BLEDescriptor *bledescriptor = (BLEDescriptor *)bleattr;
        pvalue = bledescriptor->value();
        return bt_gatt_attr_read(conn, attr, buf, len, offset, pvalue, bledescriptor->valueLength());
    }
    return 0;
}

// Only for peripheral
ssize_t profile_write_process(bt_conn_t *conn,
                                     const bt_gatt_attr_t *attr,
                                     const void *buf, uint16_t len,
                                     uint16_t offset)
{
    pr_info(LOG_MODULE_BLE, "%s1", __FUNCTION__);
    BLEAttribute *bleattr = (BLEAttribute *)attr->user_data;
    BLECharacteristic* blecharacteritic;
    BLEAttributeType type = bleattr->type();
    BLECentralHelper central = BLEPeripheralRole::instance()->central();
    if ((BLETypeCharacteristic != type) || 0 != offset)
    {
        return 0;
    }
    
    blecharacteritic = (BLECharacteristic*)bleattr;
    blecharacteritic->setValue(*((BLEHelper *)&central), (const uint8_t *) buf, len);
    
    return len;
}

ssize_t profile_longwrite_process(struct bt_conn *conn,
                                     const struct bt_gatt_attr *attr,
                                     const void *buf, uint16_t len,
                                     uint16_t offset)
{
    pr_info(LOG_MODULE_BLE, "%s1", __FUNCTION__);
    BLEAttribute *bleattr = (BLEAttribute *)attr->user_data;
    BLECharacteristic* blecharacteritic;
    BLEAttributeType type = bleattr->type();
    BLECentralHelper central = BLEPeripheralRole::instance()->central();
    if (BLETypeCharacteristic != type)
    {
        return 0;
    }
    
    blecharacteritic = (BLECharacteristic*)bleattr;
    blecharacteritic->setBuffer(*((BLEHelper *)&central), (const uint8_t *) buf, len, offset);
    
    return len;
}

int profile_longflush_process(struct bt_conn *conn,
                              const struct bt_gatt_attr *attr, 
                              uint8_t flags)
{
    BLEAttribute *bleattr = (BLEAttribute *)attr->user_data;
    BLECharacteristic* blecharacteritic;
    BLEAttributeType type = bleattr->type();
    BLECentralHelper central = BLEPeripheralRole::instance()->central();
    if (BLETypeCharacteristic != type)
    {
        return 0;
    }
    
    blecharacteritic = (BLECharacteristic*)bleattr;

    switch (flags) {
    case BT_GATT_FLUSH_DISCARD:
        /* Discard buffer reseting it back with data */
        blecharacteritic->discardBuffer();
        return 0;
    case BT_GATT_FLUSH_SYNC:
        /* Sync buffer to data */
        blecharacteritic->syncupBuffer2Value(*((BLEHelper *)&central));
        return 0;
    }

    return -EINVAL;
}


// Only for central
uint8_t profile_notify_process (bt_conn_t *conn,
                            bt_gatt_subscribe_params_t *params,
                             const void *data, uint16_t length)
{
    BLEPeripheralHelper* peripheral = BLECentralRole::instance()->peripheral(conn);// Find peripheral by bt_conn
    BLEAttribute* notifyatt = peripheral->attribute(params); // Find attribute by params
    BLECharacteristic *chrc = (BLECharacteristic *)notifyatt;
    
    //assert(notifyatt->type() == BLETypeCharacteristic);
    pr_debug(LOG_MODULE_APP, "%s1", __FUNCTION__);
    chrc->setValue(*((BLEHelper *)peripheral),(const unsigned char *)data, length);
    return BT_GATT_ITER_CONTINUE;
}

// Only for central
uint8_t profile_discover_process(bt_conn_t *conn,
                                 const bt_gatt_attr_t *attr,
                                 bt_gatt_discover_params_t *params)
{
    BLEPeripheralHelper* peripheral = BLECentralRole::instance()->peripheral(conn);// Find peripheral by bt_conn
    return peripheral->discover(attr);
}

// Only for GATT Client
uint8_t profile_read_rsp_process(bt_conn_t *conn, int err,
                                 bt_gatt_read_params_t *params,
                                 const void *data, 
                                 uint16_t length)
{
    if (NULL == data)
    {
        return BT_GATT_ITER_STOP;
    }
    BLEPeripheralHelper* peripheral = BLECentralRole::instance()->peripheral(conn);// Find peripheral by bt_conn
    BLEAttribute* readatt = peripheral->attribute(params->single.handle);
    BLECharacteristic *chrc = (BLECharacteristic *)readatt;
    
    //assert(readatt->type() == BLETypeCharacteristic);
    chrc->setValue(*((BLEHelper *)peripheral), (const unsigned char *)data, length);
    return BT_GATT_ITER_STOP;
}

BLEProfile::BLEProfile (BLEPeripheralHelper *peripheral):
    _attr_base(NULL),
    _attr_index(0),
    _attributes(NULL),
    _num_attributes(0),
    _sub_param(NULL),
    _sub_param_idx(0)
{
    _peripheral = peripheral;
    memset(&_discover_params, 0, sizeof(_discover_params));
    _discover_params.end_handle = 0xFFFF;
    _discover_params.start_handle = 0x0001;
    _discover_params.func = profile_discover_process;
}

BLEProfile::~BLEProfile (void)
{
    if (this->_attributes) {
        free(this->_attributes);
    }
    if (this->_attr_base)
    {
        free(this->_attr_base);
    }
    if (this->_sub_param)
    {
        free(this->_sub_param);
    }
}

BleStatus
BLEProfile::addAttribute (BLEAttribute& attribute)
{
    bt_gatt_attr_t *start;
    BleStatus err_code = BLE_STATUS_SUCCESS;
    
    if (NULL == _attributes) 
    {
        _attributes = (BLEAttribute**)malloc(BLEAttribute::numAttributes() * sizeof(BLEAttribute*));
        memset(_attributes, 0x00, BLEAttribute::numAttributes() * sizeof(BLEAttribute*));
        if (NULL == _attributes)
        {
            err_code = BLE_STATUS_NO_MEMORY;
        }
    }
    if (NULL == _attr_base)
    {
        _attr_base = (bt_gatt_attr_t *)malloc((BLEAttribute::numAttributes() + BLECharacteristic::numNotifyAttributes()) * sizeof(bt_gatt_attr_t));
        memset(_attr_base, 0x00, ((BLEAttribute::numAttributes() + BLECharacteristic::numNotifyAttributes()) * sizeof(bt_gatt_attr_t)));
        pr_info(LOG_MODULE_BLE, "_attr_base_-%p, size-%d", _attr_base, sizeof(_attr_base));
        if (NULL == _attr_base)
        {
            err_code = BLE_STATUS_NO_MEMORY;
        }
    }
    if (NULL == _sub_param)
    {
        _sub_param = (bt_gatt_subscribe_params_t *)malloc((BLECharacteristic::numNotifyAttributes()) * sizeof(bt_gatt_subscribe_params_t));
        memset(_sub_param, 0x00, ((BLECharacteristic::numNotifyAttributes()) * sizeof(bt_gatt_subscribe_params_t)));
        if (NULL == _sub_param)
        {
            err_code = BLE_STATUS_NO_MEMORY;
        }
    }
    
    if (BLE_STATUS_SUCCESS != err_code)
    {
        if (NULL != _attributes)
        {
            free(_attributes);
        }
        if (NULL != _attr_base)
        {
            free(_attr_base);
        }
        if (NULL != _sub_param)
        {
            free(_sub_param);
        }
        return err_code;
    }

    _attributes[_num_attributes] = &attribute;
    _num_attributes++;
    start = _attr_base + _attr_index;
    pr_info(LOG_MODULE_BLE, "_attr_base_-%p", _attr_base);

    BLEAttributeType type = attribute.type();
    pr_info(LOG_MODULE_BLE, "%s: idx-%d, %p, %d", __FUNCTION__,_num_attributes, &attribute ,attribute.uuid()->type);
        

    if (BLETypeCharacteristic == type)
    {
        BLECharacteristic* characteritic = (BLECharacteristic*) &attribute;
        
        // Characteristic
        memset(start, 0, sizeof(bt_gatt_attr_t));
        start->uuid = BLECharacteristic::getCharacteristicAttributeUuid();
        start->perm = BT_GATT_PERM_READ;
        start->read = bt_gatt_attr_read_chrc;
        start->user_data = characteritic->getCharacteristicAttValue();
        characteritic->addCharacteristicDeclaration(start);
        
        pr_info(LOG_MODULE_BLE, "chrc-%p, uuid type-%d", start, start->uuid->type);
        
        start++;
        _attr_index++;
        
        // Descriptor
        memset(start, 0, sizeof(bt_gatt_attr_t));
        start->uuid = characteritic->uuid();
        start->perm = characteritic->getPermission();
        start->user_data = (void*)&attribute;
        characteritic->addCharacteristicValue(start);
        start->read = profile_read_process;
        
        if (characteritic->longCharacteristic() == false)
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
        pr_info(LOG_MODULE_BLE, "desc-%p, uuid: 0x%x", start, ((bt_uuid_16_t*) start->uuid)->val);
        
        start++;
        _attr_index++;
        // CCCD
        if (characteritic->subscribed())
        {
            // Descriptor
            memset(start, 0, sizeof(bt_gatt_attr_t));
            start->uuid = characteritic->getClientCharacteristicConfigUuid();
            start->perm = BT_GATT_PERM_READ | BT_GATT_PERM_WRITE;
            start->read = bt_gatt_attr_read_ccc;
            start->write = bt_gatt_attr_write_ccc;
            start->user_data = characteritic->getCccCfg();
            characteritic->addCharacteristicConfigDescriptor(start);
            
            pr_info(LOG_MODULE_BLE, "cccd-%p", start);
            
            start++;
            _attr_index++;
        }
    } 
    else if (BLETypeService == type)
    {
        start->uuid = BLEService::getPrimayUuid();
        start->perm = BT_GATT_PERM_READ;
        start->read = bt_gatt_attr_read_service;
        start->user_data = attribute.uuid();
        
        pr_debug(LOG_MODULE_BLE, "service-%p", start);
        start++;
        _attr_index++;
    }
    else if (BLETypeDescriptor == type)
    {
        start->uuid = attribute.uuid();
        start->perm = BT_GATT_PERM_READ;
        start->read = profile_read_process;
        start->user_data = (void*)&attribute;
        
        pr_debug(LOG_MODULE_BLE, "Descriptor-%p", start);
        start++;
        _attr_index++;
    }
    return err_code;
}

int BLEProfile::registerProfile()
{
    int ret = 0;
    
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
    pr_debug(LOG_MODULE_APP, "%s: ret, %d", __FUNCTION__, ret);
    
    return ret;
}

void BLEProfile::characteristicDiscoverRsp(const bt_gatt_attr_t *attr, BLEAttribute* bleattr)
{
    bt_gatt_attr_t *attr_dec = declarationAttr(bleattr);
    if ((NULL != attr) && (NULL != attr_dec))
    {
        if (bt_uuid_cmp (attr_dec->uuid, attr->uuid) == 0)
        {
            attr_dec++;
            attr_dec->handle = attr->handle + 1;
        }
    }
    bleattr->discover(attr, &_discover_params);
}

void BLEProfile::descriptorDiscoverRsp(const bt_gatt_attr_t *attr, BLEAttribute* bleattr)
{
    int err;
    bt_gatt_attr_t *attr_dec = declarationAttr(bleattr);
    if (BLETypeCharacteristic == bleattr->type())
    {
        BLECharacteristic *chrc = (BLECharacteristic *)bleattr;
        if (bt_uuid_cmp (chrc->getClientCharacteristicConfigUuid(), attr->uuid) == 0)
        {
            //CCCD
            bt_gatt_attr_t *attr_chrc = attr_dec + 1;
            bt_gatt_attr_t *attr_cccd = attr_dec + 2;
            bt_gatt_subscribe_params_t *sub_param_tmp = chrc->getSubscribeParams();
            bt_gatt_subscribe_params_t *sub_param = _sub_param + _sub_param_idx;
            bt_conn_t *conn = bt_conn_lookup_addr_le(_peripheral->bt_le_address());
            if (NULL == conn)
            {
                // Link lost
                return;
            }
            
            _sub_param_idx++;
            attr_cccd->handle = attr->handle;
            memcpy(sub_param, sub_param_tmp, sizeof(bt_gatt_subscribe_params_t));
            sub_param->ccc_handle = attr_cccd->handle;
            sub_param->value_handle = attr_chrc->handle;
            
            // Enable CCCD to allow peripheral send Notification/Indication
            err = bt_gatt_subscribe(conn, sub_param);
            bt_conn_unref(conn);
            if (err && err != -EALREADY)
            {
                pr_debug(LOG_MODULE_APP, "Subscribe failed (err %d)\n", err);
            }
            bleattr->discover(attr, &_discover_params);
        }
        else
        {
            // Not CCCD
            //  If want to support more descriptor, 
            //   change the offset 3 as a loop to search the ATTR
            bt_gatt_attr_t *attr_descriptor = attr_dec + 3;
            if (attr_descriptor->uuid != NULL && 
                bt_uuid_cmp (attr_descriptor->uuid, attr->uuid) == 0)
            {
                attr_descriptor->handle = attr->handle;
            }
        }
    }
    else if (BLETypeDescriptor == bleattr->type())
    {
        bt_gatt_attr_t *attr_descriptor = attr_dec++; // The descriptor is separate
        if (bt_uuid_cmp (attr_dec->uuid, attr->uuid) == 0)
        {
            attr_descriptor->handle = attr->handle;
        }
        bleattr->discover(attr, &_discover_params);
    }
}

uint8_t BLEProfile::discover(const bt_gatt_attr_t *attr)
{
    BLEAttribute* attribute_tmp = NULL;
    int i;
    int err;
    uint8_t ret = BT_GATT_ITER_STOP;
    bool send_discover = false;
    
    for (i = 0; i < _num_attributes; i++)
    {
        // Find the discovering attribute
        attribute_tmp = _attributes[i];
        if (attribute_tmp->discovering())
        {
            if (NULL == attr)
            {
                attribute_tmp->discover(attr, &_discover_params);
                break;
            }
            // Discover success
            switch (_discover_params.type)
            {
                case BT_GATT_DISCOVER_CHARACTERISTIC:
                {
                    characteristicDiscoverRsp(attr, attribute_tmp);
                    send_discover = true;
                    break;
                }
                case BT_GATT_DISCOVER_DESCRIPTOR:
                {
                    descriptorDiscoverRsp(attr, attribute_tmp);
                    break;
                }
                case BT_GATT_DISCOVER_PRIMARY:
                    send_discover = true;
                default:
                {
                    attribute_tmp->discover(attr, &_discover_params);
                    break;
                }
            }
            break;
        }
    }
    
    //  Find next attribute to discover
    if (attribute_tmp->discovering() == false)
    {
        // Current attribute complete discovery
        i++;
        while (i < _num_attributes)
        {
            attribute_tmp = _attributes[i];
            if (attribute_tmp->type() == BLETypeDescriptor)
            {
                // The descriptor may have been discovered by previous descriptor
                bt_gatt_attr_t *attr_gatt = NULL;
                for (int j = 0; j < _attr_index; j++)
                {
                    attr_gatt = _attr_base + i;
                    if (attribute_tmp->uuid() == attr_gatt->uuid)
                    {
                        break;
                    }
                }
                
                if (attr_gatt->handle != 0)
                {
                    // Skip discovered descriptor
                    i++;
                    continue;
                }
            }
            
            attribute_tmp->discover(&_discover_params);
            ret = BT_GATT_ITER_CONTINUE;
            break;
        }
    }
    else
    {
        ret = BT_GATT_ITER_CONTINUE;
    }
    
    // Send the discover request if necessary
    if (send_discover && attribute_tmp->discovering())
    {
        bt_conn_t *conn = bt_conn_lookup_addr_le(_peripheral->bt_le_address());
        
        ret = BT_GATT_ITER_STOP;
        if (NULL == conn)
        {
            // Link lost
            pr_debug(LOG_MODULE_APP, "Can't find connection\n");
            return ret;
        }
        err = bt_gatt_discover(conn, &_discover_params);
        bt_conn_unref(conn);
        if (err)
        {
            pr_debug(LOG_MODULE_APP, "Discover failed(err %d)\n", err);
            return ret;
        }
    }
    return ret;
}


void BLEProfile::discover()
{
    int err;
    BLEService *serviceattr = (BLEService *)_attributes[0];
    bt_conn_t *conn = bt_conn_lookup_addr_le(_peripheral->bt_le_address());
    
    if (NULL == conn)
    {
        // Link lost
        pr_debug(LOG_MODULE_APP, "Can't find connection\n");
        return;
    }
    
    // Reset start handle
    _discover_params.start_handle = 0x0001;
    serviceattr->discover(&_discover_params);
    
    err = bt_gatt_discover(conn, &_discover_params);
    bt_conn_unref(conn);
    if (err)
    {
        pr_debug(LOG_MODULE_APP, "Discover failed(err %d)\n", err);
        return;
    }
}

BLEAttribute *BLEProfile::attribute(bt_gatt_subscribe_params_t *params)
{
    return attribute(params->value_handle);
}

BLEAttribute *BLEProfile::attribute(const bt_uuid_t* uuid)
{
    int i;
    BLEAttribute *attr_tmp = NULL;
    BLECharacteristic *chrc_tmp = NULL;
    bool att_found = false;
    
    for (i = 0; i < _num_attributes; i++)
    {
        attr_tmp = _attributes[i];
        if ((NULL == attr_tmp) || (attr_tmp->type() != BLETypeCharacteristic))
        {
            continue;
        }
        chrc_tmp = (BLECharacteristic *)attr_tmp;
        if (chrc_tmp->uuid() == uuid);
        {
            att_found = true;
            break;
        }
    }
    
    if (false == att_found)
    {
        pr_debug(LOG_MODULE_APP, "Attributes not found");
        // Didn't found the characteristic
        chrc_tmp = NULL;
    }
    return chrc_tmp;
}


BLEAttribute *BLEProfile::attribute(uint16_t handle)
{
    int i;
    bt_gatt_attr_t *attr_gatt = NULL;
    for (i = 0; i < _attr_index; i++)
    {
        attr_gatt = _attr_base + i;
        if (handle == attr_gatt->handle)
        {
            break;
        }
    }
    
    if (i < _attr_index && i > 1)
    {
        // Found the GATT ATTR
        //  Serach the attribute 
        //   Characteristic Declaration
        //   Characteristic Descriptor
        //   CCCD
        attr_gatt--;
        if (attr_gatt->uuid == BLECharacteristic::getCharacteristicAttributeUuid())
        {
            attr_gatt++;
        }
        else
        {
            attr_gatt--;
            if (attr_gatt->uuid == BLECharacteristic::getCharacteristicAttributeUuid())
            {
                attr_gatt++;
            }
            else
            {
                attr_gatt = NULL;
            }
        }
    }
    else
    {
        attr_gatt = NULL;
    }
    
    if (NULL != attr_gatt)
    {
        return attribute(attr_gatt->uuid);
    }
    return NULL;
}


void BLEProfile::clearHandles(void)
{
    int i;
    bt_gatt_attr_t *attr = NULL;
    // Didn't need to unsubscribe
    //  The stack will unsubscribe the notify when disconnected.
    //  The sub_param has some pointer. So can't call memset. Just reset the index.
    _sub_param_idx = 0;
    
    for (i = 0; i < _attr_index; i++)
    {
        // Clear the handle
        attr = _attr_base + i;
        attr->handle = 0;
    }
}

bt_gatt_attr_t* BLEProfile::declarationAttr(BLEAttribute *attr)
{
    int i;
    bt_gatt_attr_t *attr_gatt = NULL;
    
    for (i = 0; i < _attr_index; i++)
    {
        // Clear the handle
        attr_gatt = _attr_base + i;
        if (attr->uuid() == attr_gatt->uuid)
        {
            attr_gatt--;
            return attr_gatt;
        }
    }
    return NULL;
}

uint16_t BLEProfile::valueHandle(BLEAttribute *attr)
{
    uint16_t handle = 0;
    bt_gatt_attr_t *attr_gatt = declarationAttr(attr);
    attr_gatt++;
    if (attr_gatt->uuid == attr->uuid())
    {
        handle = attr_gatt->handle;
    }
    return handle;
}

uint16_t BLEProfile::cccdHandle(BLEAttribute *attr)
{
    uint16_t handle = 0;
    bt_gatt_attr_t *attr_gatt = declarationAttr(attr);
    attr_gatt+= 2;
    if (attr_gatt->uuid == BLECharacteristic::getClientCharacteristicConfigUuid())
    {
        handle = attr_gatt->handle;
    }
    return handle;
}



