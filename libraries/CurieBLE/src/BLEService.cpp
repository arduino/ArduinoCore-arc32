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

#include "BLEService.h"
struct bt_uuid_16 BLEService::_gatt_primary_uuid = {BT_UUID_TYPE_16, BT_UUID_GATT_PRIMARY_VAL};
struct bt_uuid *BLEService::getPrimayUuid(void)
{
    return (struct bt_uuid *)&_gatt_primary_uuid;
}

BLEService::BLEService(const char* uuid) :
    BLEAttribute(uuid, BLETypeService)
{
}


void BLEService::discover(struct bt_gatt_discover_params *params)
{
    params->type = BT_GATT_DISCOVER_PRIMARY;
    
    params->uuid = this->uuid();
    // Start discovering
    _discoverying = true;
}

void BLEService::discover(const struct bt_gatt_attr *attr,
                          struct bt_gatt_discover_params *params)
{
    params->start_handle = attr->handle + 1;
    
    // Complete the discover
    _discoverying = false;
}

