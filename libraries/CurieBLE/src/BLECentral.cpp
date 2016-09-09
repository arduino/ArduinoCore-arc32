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

#include "BLECentralRole.h"

#include "BLECentral.h"
#include "internal/ble_client.h"

bool BLECentral::startScan()
{
    return BLECentralRole::instance()->startScan();
}

bool BLECentral::startScan(float interval, float window)
{
    setScanParam(interval, window);
    return BLECentralRole::instance()->startScan();
}

bool BLECentral::stopScan()
{
    return BLECentralRole::instance()->stopScan();
}

bool BLECentral::connect(const bt_addr_le_t *addr, const ble_conn_param_t *param)
{
    bt_le_conn_param_t conn_param;
    
    conn_param.latency = param->latency;
    conn_param.interval_max = (uint16_t)MSEC_TO_UNITS(param->interval_max, UNIT_1_25_MS);
    conn_param.interval_min = (uint16_t)MSEC_TO_UNITS(param->interval_min, UNIT_1_25_MS);
    conn_param.timeout = MSEC_TO_UNITS(param->timeout, UNIT_10_MS);
    
    pr_debug(LOG_MODULE_BLE,"Latency-%d\r\nInterval min-%d, max-%d\r\ntimeout:%d",
            conn_param.latency,
            conn_param.interval_min,
            conn_param.interval_max,
            conn_param.timeout);
    
    return BLECentralRole::instance()->connect(addr, &conn_param);
}

void BLECentral::discover(BLEPeripheralHelper &peripheral)
{
    peripheral.discover();
}

void BLECentral::setEventHandler(BLERoleEvent event, BLERoleEventHandler callback)
{
    BLECentralRole::instance()->setEventHandler(event, callback);
}

void BLECentral::setAdvertiseHandler(ble_advertise_handle_cb_t advcb)
{
    BLECentralRole::instance()->setAdvertiseHandler(advcb);
}

void BLECentral::setScanParam(float interval, float window)
{
    bt_le_scan_param_t scan_param;
    scan_param.type         = BT_HCI_LE_SCAN_ACTIVE;
    scan_param.filter_dup   = BT_HCI_LE_SCAN_FILTER_DUP_ENABLE;
    scan_param.interval     = (uint16_t)MSEC_TO_UNITS(interval, UNIT_0_625_MS);;
    scan_param.window       = (uint16_t)MSEC_TO_UNITS(window, UNIT_0_625_MS);;
    BLECentralRole::instance()->setScanParam(scan_param);
}

BleStatus BLECentral::addAttribute(BLEAttribute& attribute)
{
    return BLECentralRole::instance()->addAttribute(attribute);
}

bool BLECentral::begin(void)
{
    bool retval = BLECentralRole::instance()->begin();
    if (!retval)
    {
        pr_error(LOG_MODULE_BLE,"%s: Intit failed", __FUNCTION__);
        return false;
    }
    
    // Start scan
    const bt_le_scan_param_t *scan_param = BLECentralRole::instance()->getScanParam();
    bt_le_scan_param_t zero_param;
    memset(&zero_param, 0x00, sizeof (zero_param));
    if (0 == memcmp(&zero_param, scan_param, sizeof (zero_param)))
    {
        // Not set the scan parameter.
        //  Use the default scan parameter to scan
        zero_param.type         = BT_HCI_LE_SCAN_ACTIVE;
        zero_param.filter_dup   = BT_HCI_LE_SCAN_FILTER_DUP_ENABLE;
        zero_param.interval     = BT_GAP_SCAN_FAST_INTERVAL;//BT_GAP_SCAN_SLOW_INTERVAL_1;//
        zero_param.window       = BT_GAP_SCAN_FAST_WINDOW; //BT_GAP_SCAN_SLOW_WINDOW_1;//
        retval = BLECentralRole::instance()->startScan(zero_param);
    }
    else
    {
        retval = BLECentralRole::instance()->startScan();
    }
    return retval;
}

BLEPeripheralHelper  *BLECentral::getPeerPeripheralBLE(BLEHelper& peripheral)
{
	return (BLEPeripheralHelper *)(&peripheral);
}


