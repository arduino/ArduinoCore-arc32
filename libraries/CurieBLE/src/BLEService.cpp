/*
 * Copyright (c) 2015 Intel Corporation.  All rights reserved.
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
