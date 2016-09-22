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

#include "BLECentralRole.h"


void ble_central_device_found(const bt_addr_le_t *addr, 
                              int8_t rssi, 
                              uint8_t type,
                              const uint8_t *ad, 
                              uint8_t len)
{
	char dev[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(addr, dev, sizeof(dev));
	pr_debug(LOG_MODULE_BLE, "[DEVICE]: %s, AD evt type %u, AD data len %u, RSSI %i\n",
	       dev, type, len, rssi);

    BLECentralRole::instance()->handleDeviceFound(addr, rssi, type,
                                                  ad, len);
}


BLECentralRole* BLECentralRole::_ble_central_ins = NULL;

BLECentralRole *BLECentralRole::instance()
{
    if (NULL == _ble_central_ins)
    {
        _ble_central_ins = new BLECentralRole();
    }
    return _ble_central_ins;
}

BLECentralRole::BLECentralRole():
    _central(NULL), _adv_event_handle(NULL)
{
    memset(_peripherial, 0, sizeof (_peripherial));
    for (int i = 0; i < BLE_MAX_CONN_CFG; i++)
    {
        _peripherial[i] = new BLEPeripheralHelper(this);
    }
    memset (&_scan_param, 0x00, sizeof (_scan_param));
    _central.setAddress(_local_bda);
}


BLECentralRole::~BLECentralRole()
{
    for (int i = 0; i < BLE_MAX_CONN_CFG; i++)
    {
        delete (_peripherial[i]);
        //_peripherial[i] = NULL;
    }
}

const BLECentralHelper *BLECentralRole::central(void) const
{
    return &_central;
}

bool BLECentralRole::connect(const bt_addr_le_t *addr, const bt_le_conn_param_t *param)
{
    BLEPeripheralHelper* temp   = NULL;
    BLEPeripheralHelper* unused = NULL;
    bool link_existed = false;
    bool retval = false;
    
    // Find free peripheral Items
    for (int i = 0; i < BLE_MAX_CONN_CFG; i++)
    {
        temp = _peripherial[i];
        if (true == *temp)
        {
            if (*temp == *addr)
            {
                // Connect request has scheduled but connection don't established.
                //  The central can see the ADV and no need to send connect request.
                link_existed = true;
                break;
            }
        }
        else
        {
            if (NULL == unused)
            {
                unused = temp;
            }
        }
    }
    
    if (!link_existed)
    {
        // Send connect request
        bt_conn_t* conn = bt_conn_create_le(addr, param);
        if (NULL != conn)
        {
            unused->setAddress(*addr);
            retval = true;
            bt_conn_unref(conn);
        }
    }
    return retval;
}

bool BLECentralRole::startScan()
{
    int err = bt_le_scan_start(&_scan_param, ble_central_device_found);
    if (err)
    {
        pr_info(LOG_MODULE_BLE, "Scanning failed to start (err %d)\n", err);
        return false;
    }
    return true;
}

bool BLECentralRole::startScan(const bt_le_scan_param_t &scan_param)
{
    setScanParam(scan_param);
    return startScan();
}

void BLECentralRole::setScanParam(const bt_le_scan_param_t &scan_param)
{
    memcpy(&_scan_param, &scan_param, sizeof (_scan_param));
}

const bt_le_scan_param_t* BLECentralRole::getScanParam()
{
    return &_scan_param;
}


bool BLECentralRole::stopScan()
{
    int err = bt_le_scan_stop();
    if (err)
    {
        pr_info(LOG_MODULE_BLE, "Stop LE scan failed (err %d)\n", err);
        return false;
    }
    return true;
}

BLEPeripheralHelper* BLECentralRole::peripheral(bt_conn_t *conn)
{
    BLEPeripheralHelper* temp   = NULL;
    const bt_addr_le_t *addr = bt_conn_get_dst(conn);
    // Find free peripheral Items
    for (int i = 0; i < BLE_MAX_CONN_CFG; i++)
    {
        temp = _peripherial[i];
        if (*temp == *addr)
        {
            return temp;
        }
    }
    return NULL;
}


void BLECentralRole::handleDeviceFound(const bt_addr_le_t *addr, 
                                       int8_t rssi, 
                                       uint8_t type,
                                       const uint8_t *ad, 
                                       uint8_t data_len)
{
    const uint8_t *data = ad;
    
    if (_adv_event_handle == NULL)
    {
        return;
    }
    
	/* We're only interested in connectable events */
	if (type == BT_LE_ADV_IND || type == BT_LE_ADV_DIRECT_IND)
    {
		pr_debug(LOG_MODULE_BLE, "%s", __FUNCTION__);
        
        while (data_len > 1)
        {
    		uint8_t len = data[0];

    		/* Check for early termination */
    		if (len == 0) {
    			return;
    		}

    		if ((len + 1 > data_len) || (data_len < 2)) {
    			pr_info(LOG_MODULE_BLE, "AD malformed\n");
    			return;
    		}

    		if (!_adv_event_handle(data[1], &data[2], len - 1, addr))
            {
    			return;
    		}

    		data_len -= len + 1;
    		data += len + 1;
	    }
		pr_debug(LOG_MODULE_BLE, "%s: done", __FUNCTION__);
	}
}

void BLECentralRole::handleConnectEvent(bt_conn_t *conn, uint8_t err)
{
    if (_event_handlers[BLEConnected])
    {
        BLEPeripheralHelper *temp = peripheral(conn);
        _event_handlers[BLEConnected](*temp);
    }
}

void BLECentralRole::handleDisconnectEvent(bt_conn_t *conn, uint8_t reason)
{
    if (_event_handlers[BLEDisconnected])
    {
        BLEPeripheralHelper *temp = peripheral(conn);
        _event_handlers[BLEDisconnected](*temp);
        temp->linkLost();
    }
}

void BLECentralRole::handleParamUpdated(bt_conn_t *conn, 
                        uint16_t interval,
                        uint16_t latency, 
                        uint16_t timeout)
{
    if (_event_handlers[BLEUpdateParam])
    {
        BLEPeripheralHelper *temp = peripheral(conn);
        temp->setConnectionParameters(interval, interval, latency, timeout);
        _event_handlers[BLEUpdateParam](*temp);
    }
}

void BLECentralRole::setEventHandler(BLERoleEvent event, BLERoleEventHandler callback)
{
    
    if (event < sizeof(_event_handlers))
    {
        _event_handlers[event] = callback;
    }
}

void BLECentralRole::setAdvertiseHandler(ble_advertise_handle_cb_t advcb)
{
    _adv_event_handle = advcb;
}

BleStatus BLECentralRole::addAttribute(BLEAttribute& attribute)
{
    BleStatus err = BLE_STATUS_SUCCESS;
    for (int i = 0; i < BLE_MAX_CONN_CFG; i++)
    {
        err = _peripherial[i]->addAttribute(attribute);
        if (err != BLE_STATUS_SUCCESS) 
        {
            break;
        }
    }
    return err;
}

bool BLECentralRole::begin()
{
    BleStatus status;
    status = _init();
    if (status != BLE_STATUS_SUCCESS) 
    {
        return false;
    }
    return true;
}

bool BLECentralRole::disconnect()
{
    return true;
}


