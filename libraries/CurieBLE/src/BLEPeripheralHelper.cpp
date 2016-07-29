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

#include "BLEPeripheralHelper.h"

BLEAttribute *BLEPeripheralHelper::attribute(uint16_t handle)
{
    return _profile.attribute(handle);
}

BLEAttribute *BLEPeripheralHelper::attribute(struct bt_gatt_subscribe_params *params)
{
    return _profile.attribute(params);
}

void BLEPeripheralHelper::discover(const struct bt_gatt_attr *attr)
{
    // Not allow to call the discover
    if (NULL == _central)
    {
        return;
    }
    _profile.discover(attr);
}

void BLEPeripheralHelper::discover()
{
    if (NULL == _central)
    {
        return;
    }
    _profile.discover();
}

BLEPeripheralHelper::BLEPeripheralHelper(BLECentralRole* central): 
        _profile(this),
        _central(central)
{
    ;
}
BLEPeripheralHelper::~BLEPeripheralHelper()
{
    
}

bool BLEPeripheralHelper::disconnect(void)
{
    int err = 0;
    struct bt_conn* conn = bt_conn_lookup_addr_le(this->bt_le_address());
    if (NULL == conn)
    {
        return false;
    }
    
    err = bt_conn_disconnect (conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
    bt_conn_unref(conn);
    return (err == 0);
}

bool BLEPeripheralHelper::connected(void)
{
    struct bt_conn* conn = bt_conn_lookup_addr_le(this->bt_le_address());
    if (NULL == conn)
    {
        return false;
    }
    bt_conn_unref(conn);
    return true;
}

void BLEPeripheralHelper::linkLost(void)
{
    clearAddress();
    if (NULL != _central)
    {
        // Only central role need to do
        _profile.clearHandles();
    }
}

void BLEPeripheralHelper::addAttribute(BLEAttribute& attribute)
{
    _profile.addAttribute(attribute);
}

int BLEPeripheralHelper::registerProfile()
{
    return _profile.registerProfile();
}

uint16_t BLEPeripheralHelper::valueHandle(BLEAttribute *attr)
{
    return _profile.valueHandle(attr);
}

uint16_t BLEPeripheralHelper::cccdHandle(BLEAttribute *attr)
{
    return _profile.cccdHandle(attr);
}


