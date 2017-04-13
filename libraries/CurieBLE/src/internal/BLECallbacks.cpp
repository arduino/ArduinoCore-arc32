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

#include "CurieBLE.h"

#include "BLEAttribute.h"
#include "BLECharacteristicImp.h"
#include "BLEDeviceManager.h"
#include "BLEProfileManager.h"

#include "BLECallbacks.h"

#include <atomic.h>
#include "../src/services/ble/conn_internal.h"

// GATT Server Only
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
        BLECharacteristicImp* blecharacteritic = (BLECharacteristicImp*)bleattr;
        pvalue = blecharacteritic->value();
        return bt_gatt_attr_read(conn, attr, buf, len, offset, pvalue,
                                 blecharacteritic->valueLength());
    }
    else if (BLETypeDescriptor == type)
    {
        BLEDescriptorImp* bledescriptor = (BLEDescriptorImp*)bleattr;
        pvalue = bledescriptor->value();
        return bt_gatt_attr_read(conn, attr, buf, len, offset, pvalue, bledescriptor->valueLength());
    }
    return 0;
}

// GATT server only
ssize_t profile_write_process(bt_conn_t *conn,
                              const bt_gatt_attr_t *attr,
                              const void *buf, uint16_t len,
                              uint16_t offset)
{
    pr_info(LOG_MODULE_BLE, "%s1", __FUNCTION__);
    BLEAttribute *bleattr = (BLEAttribute *)attr->user_data;
    BLECharacteristicImp* blecharacteritic;
    BLEAttributeType type = bleattr->type();
    if ((BLETypeCharacteristic != type) || 0 != offset)
    {
        return 0;
    }
    
    blecharacteritic = (BLECharacteristicImp*)bleattr;
    blecharacteritic->setValue((const uint8_t *) buf, len);
    return len;
}

ssize_t profile_longwrite_process(struct bt_conn *conn,
                                     const struct bt_gatt_attr *attr,
                                     const void *buf, uint16_t len,
                                     uint16_t offset)
{
    BLEAttribute *bleattr = (BLEAttribute *)attr->user_data;
    BLEAttributeType type = bleattr->type();
    if (BLETypeCharacteristic != type)
    {
        return 0;
    }
    BLECharacteristicImp *blecharacteritic = (BLECharacteristicImp*)bleattr;
    
    blecharacteritic->setBuffer((const uint8_t *) buf, len, offset);
    
    return len;
}

int profile_longflush_process(struct bt_conn *conn,
                              const struct bt_gatt_attr *attr, 
                              uint8_t flags)
{
    BLEAttribute *bleattr = (BLEAttribute *)attr->user_data;
    BLEAttributeType type = bleattr->type();
    if (BLETypeCharacteristic != type)
    {
        return 0;
    }
    BLECharacteristicImp *blecharacteritic = (BLECharacteristicImp*)bleattr;

    switch (flags)
    {
    case BT_GATT_FLUSH_DISCARD:
        /* Discard buffer reseting it back with data */
        blecharacteritic->discardBuffer();
        return 0;
    case BT_GATT_FLUSH_SYNC:
        /* Sync buffer to data */
        blecharacteritic->syncupBuffer2Value();
        return 0;
    }

    return -EINVAL;
}


// GATT client only
uint8_t profile_notify_process (bt_conn_t *conn,
                                bt_gatt_subscribe_params_t *params,
                                const void *data, uint16_t length)
{
    //BLEPeripheralHelper* peripheral = BLECentralRole::instance()->peripheral(conn);// Find peripheral by bt_conn
    //BLEAttribute* notifyatt = peripheral->attribute(params); // Find attribute by params
    BLECharacteristicImp* chrc = NULL;
    BLEDevice bleDevice(bt_conn_get_dst(conn));
    chrc = BLEProfileManager::instance()->characteristic(bleDevice, params->value_handle);
    
    //assert(notifyatt->type() == BLETypeCharacteristic);
    pr_debug(LOG_MODULE_APP, "%s1", __FUNCTION__);
    if (NULL != chrc)
    {
        chrc->setValue((const unsigned char *)data, length);
    }
    return BT_GATT_ITER_CONTINUE;
}

// GATT client only
uint8_t profile_discover_process(bt_conn_t *conn,
                                 const bt_gatt_attr_t *attr,
                                 bt_gatt_discover_params_t *params)
{
    uint8_t ret = BT_GATT_ITER_STOP;
    pr_debug(LOG_MODULE_BLE, "%s-%d", __FUNCTION__, __LINE__);
    ret = BLEProfileManager::instance()->discoverResponseProc(conn, attr, params);
    pr_debug(LOG_MODULE_BLE, "%s-%d", __FUNCTION__, __LINE__);
    return ret;
}

// GATT Client only
uint8_t profile_read_rsp_process(bt_conn_t *conn, 
                                 int err,
                                 bt_gatt_read_params_t *params,
                                 const void *data, 
                                 uint16_t length)
{
    BLECharacteristicImp *chrc = NULL;
    BLEDevice bleDevice(bt_conn_get_dst(conn));
    
    // Get characteristic by handle params->single.handle
    chrc = BLEProfileManager::instance()->characteristic(bleDevice, params->single.handle);
    
    if (chrc)  // KW issue: may be NULL and will be dereferenced
      chrc->setValue((const unsigned char *)data, length);
    pr_debug(LOG_MODULE_BLE, "%s-%d", __FUNCTION__, __LINE__);
    return BT_GATT_ITER_STOP;
}

uint8_t profile_descriptor_read_rsp_process(bt_conn_t *conn, 
                                            int err,
                                            bt_gatt_read_params_t *params,
                                            const void *data, 
                                            uint16_t length)
{
    if (NULL == data)
    {
        return BT_GATT_ITER_STOP;
    }
    BLEDescriptorImp *descriptor = NULL;
    BLEDevice bleDevice(bt_conn_get_dst(conn));
    
    // Get characteristic by handle params->single.handle
    descriptor = BLEProfileManager::instance()->descriptor(bleDevice, params->single.handle);
    
    //pr_debug(LOG_MODULE_BLE, "%s-%d", __FUNCTION__, __LINE__);
    if (descriptor)
    {
        descriptor->writeValue((const unsigned char *)data, length, params->single.offset);
    }
    //pr_debug(LOG_MODULE_BLE, "%s-%d: desc len-%d", __FUNCTION__, __LINE__, descriptor->valueLength());
    return BT_GATT_ITER_STOP;
}

uint8_t profile_service_read_rsp_process(bt_conn_t *conn, 
                                 int err,
                                 bt_gatt_read_params_t *params,
                                 const void *data, 
                                 uint16_t length)
{
    uint8_t ret = BLEProfileManager::instance()->serviceReadRspProc(conn, err, params, data, length);
    pr_debug(LOG_MODULE_BLE, "%s-%d:ret-%d", __FUNCTION__, __LINE__, ret);
    return ret;
}

uint8_t profile_characteristic_read_rsp_process(bt_conn_t *conn, 
                                                 int err,
                                                 bt_gatt_read_params_t *params,
                                                 const void *data, 
                                                 uint16_t length)
{
    BLEDevice bleDevice(bt_conn_get_dst(conn));
    BLEServiceImp* service_imp = BLEProfileManager::instance()->getServiceBySubHandle(bleDevice, params->single.handle);
    
    uint8_t ret = service_imp->characteristicReadRspProc(conn, 
                                                         err,
                                                         params,
                                                         data,
                                                         length);
    pr_debug(LOG_MODULE_BLE, "%s-%d:ret-%d", __FUNCTION__, __LINE__, ret);
    return ret;
}


void bleConnectEventHandler(bt_conn_t *conn, 
                            uint8_t err, 
                            void *param)
{
    BLEDeviceManager* p = (BLEDeviceManager*)param;

    p->handleConnectEvent(conn, err);
}

static uint8_t ble_gatt_disconnected_cb(const struct bt_gatt_attr *attr, void *user_data)
{
    struct bt_conn *conn = (struct bt_conn *)user_data;
    struct _bt_gatt_ccc *ccc;
    size_t i;
    
    /* Check attribute user_data must be of type struct _bt_gatt_ccc */
    if (attr->write != profile_gatt_attr_write_ccc) {
        return BT_GATT_ITER_CONTINUE;
    }
    
    ccc = (struct _bt_gatt_ccc *)attr->user_data;
    /* If already disabled skip */
    if (!ccc->value) {
        return BT_GATT_ITER_CONTINUE;
    }

    for (i = 0; i < ccc->cfg_len; i++)
    {
        /* Ignore configurations with disabled value */
        if (!ccc->cfg[i].value)
        {
            continue;
        }

        if (bt_addr_le_cmp(&conn->le.dst, &ccc->cfg[i].peer))
        {
            struct bt_conn *tmp;

            /* Skip if there is another peer connected */
            tmp = bt_conn_lookup_addr_le(&ccc->cfg[i].peer);
            if (tmp) {
                if (tmp->state == BT_CONN_CONNECTED) {
                    bt_conn_unref(tmp);
                    return BT_GATT_ITER_CONTINUE;
                }

                bt_conn_unref(tmp);
            }
        }
    }

    /* Reset value while disconnected */
    memset(&ccc->value, 0, sizeof(ccc->value));

    if (ccc->cfg_changed) {
        ccc->cfg_changed(ccc->value);
    }

    pr_debug(LOG_MODULE_BLE, "ccc %p reseted", ccc);

    return BT_GATT_ITER_CONTINUE;
}


void bleDisconnectEventHandler(bt_conn_t *conn, 
                                uint8_t reason, 
                                void *param)
{
    BLEDeviceManager* p = (BLEDeviceManager*)param;
    
    pr_info(LOG_MODULE_BLE, "Connect lost. Reason: %d", reason);
    bt_gatt_foreach_attr(0x0001, 0xffff, ble_gatt_disconnected_cb, conn);

    p->handleDisconnectEvent(conn, reason);
}

void bleParamUpdatedEventHandler(bt_conn_t *conn, 
                                 uint16_t interval,
                                 uint16_t latency, 
                                 uint16_t timeout, 
                                 void *param)
{
    BLEDeviceManager* p = (BLEDeviceManager*)param;

    p->handleParamUpdated(conn, interval, latency, timeout);
}


void ble_central_device_found(const bt_addr_le_t *addr, 
                              int8_t rssi, 
                              uint8_t type,
                              const uint8_t *ad, 
                              uint8_t len)
{
    //char dev[BT_ADDR_LE_STR_LEN];

    //bt_addr_le_to_str(addr, dev, sizeof(dev));
    //pr_debug(LOG_MODULE_BLE, "[DEVICE]: %s, AD evt type %u, AD data len %u, RSSI %i\n",
    //       dev, type, len, rssi);

    BLEDeviceManager::instance()->handleDeviceFound(addr, rssi, type,
                                                  ad, len);
}

void ble_on_write_no_rsp_complete(struct bt_conn *conn, uint8_t err,
                                         const void *data)
{
    BLECharacteristicImp::writeResponseReceived(conn, err, data);
}

ssize_t profile_gatt_attr_write_ccc(struct bt_conn *conn,
                                    const struct bt_gatt_attr *attr, 
                                    const void *buf,
                                    uint16_t len, 
                                    uint16_t offset)
{
    struct _bt_gatt_ccc *ccc = (struct _bt_gatt_ccc *)attr->user_data;
    const uint16_t *data = (const uint16_t *)buf;
    bool cccdChanged = (ccc->value != *data);
    ssize_t retValue = bt_gatt_attr_write_ccc(conn, attr, buf, len, offset);
    if (cccdChanged)
    {
        // Find characteristic and do notification
        const struct bt_gatt_attr *attrChrc = attr - 1;
        BLEAttribute *bleattr = (BLEAttribute *)attrChrc->user_data;
        BLEAttributeType type = bleattr->type();
        pr_debug(LOG_MODULE_BLE, "The Attribute type:%d", type);
        if (BLETypeCharacteristic == type)
        {
            BLECharacteristicImp *blecharacteritic = (BLECharacteristicImp*)bleattr;
            blecharacteritic->cccdValueChanged();
        }
    }
    return retValue;
}

