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
ssize_t profile_read_process(struct bt_conn *conn,
                             const struct bt_gatt_attr *attr,
                             void *buf, uint16_t len,
                             uint16_t offset)
{
    const unsigned char *pvalue;
    BLEAttribute *bleattr = (BLEAttribute *)attr->user_data;
    BLECharacteristic* blecharacteritic;
    BLEAttributeType type = bleattr->type();
    if (BLETypeCharacteristic != type)
    {
        return 0;
    }
    blecharacteritic = (BLECharacteristic*)bleattr;
    pvalue = blecharacteritic->value();
    return bt_gatt_attr_read(conn, attr, buf, len, offset, pvalue,
                             blecharacteritic->valueLength());
}

// Only for peripheral
ssize_t profile_write_process(struct bt_conn *conn,
                                     const struct bt_gatt_attr *attr,
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


// Only for central
uint8_t profile_notify_process (struct bt_conn *conn,
                             struct bt_gatt_subscribe_params *params,
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
uint8_t profile_discover_process(struct bt_conn *conn,
                                 const struct bt_gatt_attr *attr,
                                 struct bt_gatt_discover_params *params)
{
    BLEPeripheralHelper* peripheral = BLECentralRole::instance()->peripheral(conn);// Find peripheral by bt_conn
    peripheral->discover(attr);
    return BT_GATT_ITER_STOP;
}

// Only for central
uint8_t profile_read_rsp_process(struct bt_conn *conn, int err,
                                 struct bt_gatt_read_params *params,
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

void BLEProfile::addAttribute (BLEAttribute& attribute)
{
    struct bt_gatt_attr *start;
    if (NULL == _attributes) 
    {
        _attributes = (BLEAttribute**)malloc(BLEAttribute::numAttributes() * sizeof(BLEAttribute*));
        memset(_attributes, 0x00, BLEAttribute::numAttributes() * sizeof(BLEAttribute*));
    }
    if (NULL == _attr_base)
    {
        _attr_base = (struct bt_gatt_attr *)malloc((BLEAttribute::numAttributes() + BLECharacteristic::numNotifyAttributes()) * sizeof(struct bt_gatt_attr));
        memset(_attr_base, 0x00, ((BLEAttribute::numAttributes() + BLECharacteristic::numNotifyAttributes()) * sizeof(struct bt_gatt_attr)));
        pr_info(LOG_MODULE_BLE, "_attr_base_-%p, size-%d", _attr_base, sizeof(_attr_base));
    }
    if (NULL == _sub_param)
    {
        _sub_param = (struct bt_gatt_subscribe_params *)malloc((BLECharacteristic::numNotifyAttributes()) * sizeof(struct bt_gatt_subscribe_params));
        memset(_sub_param, 0x00, ((BLECharacteristic::numNotifyAttributes()) * sizeof(struct bt_gatt_subscribe_params)));
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
        memset(start, 0, sizeof(struct bt_gatt_attr));
        start->uuid = BLECharacteristic::getCharacteristicAttributeUuid();
        start->perm = BT_GATT_PERM_READ;
        start->read = bt_gatt_attr_read_chrc;
        start->user_data = characteritic->getCharacteristicAttValue();
        characteritic->addCharacteristicDeclaration(start);
    pr_info(LOG_MODULE_BLE, "chrc-%p, uuid type-%d", start, start->uuid->type);
        start++;
        _attr_index++;
        
        // Descriptor
        memset(start, 0, sizeof(struct bt_gatt_attr));
        start->uuid = characteritic->uuid();
        start->perm = characteritic->getPermission();
        start->read = profile_read_process;
        start->write = profile_write_process;
        start->user_data = (void*)&attribute;
        characteritic->addCharacteristicValue(start);
    pr_info(LOG_MODULE_BLE, "desc-%p, uuid: 0x%x", start, ((struct bt_uuid_16*) start->uuid)->val);
        
        start++;
        _attr_index++;
        // CCCD
        if (characteritic->subscribed())
        {
    pr_info(LOG_MODULE_BLE, "cccd-%p", start);
            // Descriptor
            memset(start, 0, sizeof(struct bt_gatt_attr));
            start->uuid = characteritic->getClientCharacteristicConfigUuid();
            start->perm = BT_GATT_PERM_READ | BT_GATT_PERM_WRITE;
            start->read = bt_gatt_attr_read_ccc;
            start->write = bt_gatt_attr_write_ccc;
            start->user_data = characteritic->getCccCfg();
            characteritic->addCharacteristicConfigDescriptor(start);
            
            start++;
            _attr_index++;
        }
    } 
    else if (BLETypeService == type)
    {
    pr_info(LOG_MODULE_BLE, "service-%p", start);
        start->uuid = BLEService::getPrimayUuid();
        start->perm = BT_GATT_PERM_READ;
        start->read = bt_gatt_attr_read_service;
        start->user_data = attribute.uuid();
        start++;
        _attr_index++;
    }

}

int BLEProfile::registerProfile()
{
    int ret = 0;
    
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
    
    ret = bt_gatt_register(_attr_base,
                            _attr_index);
    pr_info(LOG_MODULE_APP, "%s: ret, %d", __FUNCTION__, ret);
    
    return ret;
}

void BLEProfile::discover(const struct bt_gatt_attr *attr)
{
    BLEAttribute* attribute = NULL;
    int err;
    int i;
    bool send_discover = false;
    
    for (i = 0; i < _num_attributes; i++)
    {
        attribute = _attributes[i];
        if (attribute->discovering())
        {
            if (NULL != attr)
            {
                // Discover success
                switch (_discover_params.type)
                {
                    case BT_GATT_DISCOVER_CHARACTERISTIC:
                    {
                        struct bt_gatt_attr *attr_dec = declarationAttr(attribute);
                        attr_dec++;
                        attr_dec->handle = attr->handle + 1;
                        break;
                    }
                    case BT_GATT_DISCOVER_DESCRIPTOR:
                    {
                        BLECharacteristic *chrc = (BLECharacteristic *)attribute;
                        struct bt_gatt_attr *attr_dec = declarationAttr(attribute);
                        struct bt_gatt_attr *attr_chrc = attr_dec + 1;
                        struct bt_gatt_attr *attr_cccd = attr_dec + 2;
                        struct bt_gatt_subscribe_params *sub_param_tmp = chrc->getSubscribeParams();
                        struct bt_gatt_subscribe_params *sub_param = _sub_param + _sub_param_idx;
                        struct bt_conn *conn = bt_conn_lookup_addr_le(_peripheral->bt_le_address());
                        if (NULL == conn)
                        {
                            // Link lost
                            return;
                        }
                        
                        _sub_param_idx++;
                        attr_cccd->handle = attr->handle;
                        memcpy(sub_param, sub_param_tmp, sizeof(struct bt_gatt_subscribe_params));
                        sub_param->ccc_handle = attr_cccd->handle;
                        sub_param->value_handle = attr_chrc->handle;
                        
                        // Enable CCCD to allow peripheral send Notification/Indication
                        err = bt_gatt_subscribe(conn, sub_param);
                        bt_conn_unref(conn);
                        if (err && err != -EALREADY)
                        {
                            pr_debug(LOG_MODULE_APP, "Subscribe failed (err %d)\n", err);
                        }
                        break;
                    }
                    case BT_GATT_DISCOVER_PRIMARY:
                    default:
                    {
                        // Do nothing
                        break;
                    }
                }
            }
            attribute->discover(attr, &_discover_params);
            break;
        }
    }
    
    // Send discover
    if (attribute->discovering())
    {
        send_discover = true;
    }
    else
    {
        // Current attribute complete discovery
        //  Find next attribute to discover
        i++;
        if (i < _num_attributes)
        {
            attribute = _attributes[i];
            attribute->discover(&_discover_params);
            send_discover = true;
        }
    }
    
    if (send_discover)
    {
        struct bt_conn *conn = bt_conn_lookup_addr_le(_peripheral->bt_le_address());
        
        if (NULL == conn)
        {
            // Link lost
            pr_debug(LOG_MODULE_APP, "Can't find connection\n");
            return;
        }
        err = bt_gatt_discover(conn, &_discover_params);
        bt_conn_unref(conn);
        if (err)
        {
            pr_debug(LOG_MODULE_APP, "Discover failed(err %d)\n", err);
            return;
        }
    }
}


void BLEProfile::discover()
{
    int err;
    BLEService *serviceattr = (BLEService *)_attributes[0];
    struct bt_conn *conn = bt_conn_lookup_addr_le(_peripheral->bt_le_address());
    
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

BLEAttribute *BLEProfile::attribute(struct bt_gatt_subscribe_params *params)
{
    return attribute(params->value_handle);
}

BLEAttribute *BLEProfile::attribute(const struct bt_uuid* uuid)
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
    struct bt_gatt_attr *attr_gatt = NULL;
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
    struct bt_gatt_attr *attr = NULL;
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

struct bt_gatt_attr* BLEProfile::declarationAttr(BLEAttribute *attr)
{
    int i;
    struct bt_gatt_attr *attr_gatt = NULL;
    
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
    struct bt_gatt_attr *attr_gatt = declarationAttr(attr);
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
    struct bt_gatt_attr *attr_gatt = declarationAttr(attr);
    attr_gatt+= 2;
    if (attr_gatt->uuid == BLECharacteristic::getClientCharacteristicConfigUuid())
    {
        handle = attr_gatt->handle;
    }
    return handle;
}



