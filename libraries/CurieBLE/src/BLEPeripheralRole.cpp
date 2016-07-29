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

#include "BLEPeripheralRole.h"

#include "BLECharacteristic.h"
#include "BLEDescriptor.h"
#include "BLEService.h"

BLEPeripheralRole* BLEPeripheralRole::_ins = NULL;

BLEPeripheralRole* BLEPeripheralRole::instance()
{
    if (NULL == _ins)
    {
        _ins = new BLEPeripheralRole();
    }
    return _ins;
}

BLEPeripheralRole::BLEPeripheralRole(void) :
    _state(BLE_PERIPH_STATE_NOT_READY),
    _min_conn_interval(DEFAULT_MIN_CONN_INTERVAL),
    _max_conn_interval(DEFAULT_MAX_CONN_INTERVAL),
    _peripheral(NULL),
    _central(this)
{
    memset(_event_handlers, 0x00, sizeof(_event_handlers));
    _peripheral.setAddress(_local_bda);
}

BLEPeripheralRole::~BLEPeripheralRole(void)
{
}

bool BLEPeripheralRole::begin()
{
    BleStatus status;

    if (BLE_PERIPH_STATE_NOT_READY != _state)
        return BLE_STATUS_WRONG_STATE;

    status = _init();
    if (status != BLE_STATUS_SUCCESS) {
        return false;
    }
    _state = BLE_PERIPH_STATE_READY;
    
    // Set device name    
    setDeviceName();
    // Register profile
    _peripheral.registerProfile();
    delay(2); // Temp solution for send data fast will makes ADV data set failed
    return true;
}

void
BLEPeripheralRole::poll()
{
    // no-op for now
    delay(1);
}

void
BLEPeripheralRole::setDeviceName(const char deviceName[])
{
    memset(_device_name, 0, sizeof(_device_name));
    if (deviceName && deviceName[0]) {
        int len = strlen(deviceName);
        if (len > BLE_MAX_DEVICE_NAME)
            len = BLE_MAX_DEVICE_NAME;
        memcpy(_device_name, deviceName, len);
        setDeviceName();
    }
}

void
BLEPeripheralRole::setDeviceName()
{
    int len = strlen(_device_name);
    bt_le_set_device_name(_device_name, len);
}

void
BLEPeripheralRole::setConnectionInterval(const unsigned short minConnInterval, const unsigned short maxConnInterval)
{
    _min_conn_interval = minConnInterval;
    _max_conn_interval = maxConnInterval;

    if (_min_conn_interval < MIN_CONN_INTERVAL) {
        _min_conn_interval = MIN_CONN_INTERVAL;
    } else if (_min_conn_interval > MAX_CONN_INTERVAL) {
        _min_conn_interval = MAX_CONN_INTERVAL;
    }

    if (_max_conn_interval < _min_conn_interval) {
        _max_conn_interval = _min_conn_interval;
    } else if (_max_conn_interval > MAX_CONN_INTERVAL) {
        _max_conn_interval = MAX_CONN_INTERVAL;
    }
}

void
BLEPeripheralRole::setEventHandler(BLERoleEvent event, BLERoleEventHandler callback)
{
  if (event < sizeof(_event_handlers)) {
    _event_handlers[event] = callback;
  }
}

bool
BLEPeripheralRole::disconnect()
{
    BleStatus status = BLE_STATUS_WRONG_STATE;

    if (BLE_PERIPH_STATE_CONNECTED == _state)
    {
        struct bt_conn *central_conn = bt_conn_lookup_addr_le(_central.bt_le_address());
        if (NULL != central_conn)
        {
            status = bt_conn_disconnect (central_conn, 
                                         BT_HCI_ERR_REMOTE_USER_TERM_CONN);
            bt_conn_unref(central_conn);
        }
    }
    return (status == BLE_STATUS_SUCCESS);
}

BLECentralHelper
BLEPeripheralRole::central()
{
    poll();

    return _central;
}

bool
BLEPeripheralRole::connected()
{
    poll();

    return _central;
}

void BLEPeripheralRole::addAttribute(BLEAttribute& attribute)
{
    _peripheral.addAttribute(attribute);
}

BleStatus
BLEPeripheralRole::stopAdvertising()
{
    int err_code = 0;
    BleStatus status = BLE_STATUS_WRONG_STATE;

    if (BLE_PERIPH_STATE_ADVERTISING == _state)
    {
        err_code = bt_le_adv_stop();
        status = errorno_to_ble_status(err_code);
    }

    if (BLE_STATUS_SUCCESS != status)
        return status;

    _state = BLE_PERIPH_STATE_READY;
    return BLE_STATUS_SUCCESS;
}

BleStatus
BLEPeripheralRole::startAdvertising(const struct bt_data *ad, 
                                    size_t ad_len,
                                    const struct bt_data *sd, 
                                    size_t sd_len)
{
    int ret;

    pr_info(LOG_MODULE_BLE, "%s-ad_len%d", __FUNCTION__, ad_len);
    if (_state != BLE_PERIPH_STATE_READY)
        return BLE_STATUS_WRONG_STATE;
    
    ret = bt_le_adv_start(&_adv_param, ad, ad_len, sd, sd_len);
    if (0 != ret)
    {
        pr_error(LOG_MODULE_APP, "[ADV] Start failed. Error: %d", ret);
        return BLE_STATUS_WRONG_STATE;
    }
    _state = BLE_PERIPH_STATE_ADVERTISING;
    return BLE_STATUS_SUCCESS;
}

void BLEPeripheralRole::setAdvertisingParam(uint8_t  type, 
                                            uint16_t interval_min,
                                            uint16_t interval_max)
{
    _adv_param.addr_type = _local_bda.type;
    _adv_param.type = type;
    _adv_param.interval_min = interval_min;
    _adv_param.interval_max = interval_max;
}

BleStatus
BLEPeripheralRole::stop(void)
{
    int err_code;
    BleStatus status;

    if (BLE_PERIPH_STATE_ADVERTISING == _state)
    {
        err_code = bt_le_adv_stop();
        status = errorno_to_ble_status(err_code);
    }
    else
        status = disconnect();

    if (BLE_STATUS_SUCCESS != status)
        return status;

    _state = BLE_PERIPH_STATE_READY;
    return BLE_STATUS_SUCCESS;
}

void BLEPeripheralRole::handleConnectEvent(struct bt_conn *conn, uint8_t err)
{
    // Update the central address
    const bt_addr_le_t *central_addr = bt_conn_get_dst(conn);
    _central.setAddress(*central_addr);
    
    pr_info(LOG_MODULE_BLE, "Connected: %d", err);
    // Call the CB
    if (_event_handlers[BLEConnected])
        _event_handlers[BLEConnected](_central);
}


void BLEPeripheralRole::handleDisconnectEvent(struct bt_conn *conn, uint8_t reason)
{
    struct bt_conn *central_conn = bt_conn_lookup_addr_le(_central.bt_le_address());
    if (conn == central_conn)
    {
        pr_info(LOG_MODULE_BLE, "Peripheral Disconnect reason: %d", reason);
        if (_event_handlers[BLEDisconnected])
            _event_handlers[BLEDisconnected](_central);
    }
    _central.clearAddress();
    if (NULL != central_conn)
    {
        bt_conn_unref(central_conn);
    }
}

void BLEPeripheralRole::handleParamUpdated(struct bt_conn *conn, 
                                        uint16_t interval,
                                        uint16_t latency, 
                                        uint16_t timeout)
{
    pr_info(LOG_MODULE_BLE, "Parameter updated\r\n\tConn: %p\r\n\tinterval: %d\r\n\tlatency: %d\r\n\ttimeout: %d", 
                        conn, interval, latency, timeout);
    if (_event_handlers[BLEUpdateParam])
        _event_handlers[BLEUpdateParam](_central);
}


