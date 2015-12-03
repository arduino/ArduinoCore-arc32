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

BLEService::BLEService(const char* uuid) :
    BLEAttribute(uuid, BLETypeService)
{
}

bool
BLEService::add() {
    bt_uuid uuid = btUuid();
    uint16_t handle = 0;

    BleStatus status = ble_client_gatts_add_service(&uuid, BLE_GATT_SVC_PRIMARY, &handle);
    if (BLE_STATUS_SUCCESS == status) {
        setHandle(handle);
    }

    return (BLE_STATUS_SUCCESS == status);
}
