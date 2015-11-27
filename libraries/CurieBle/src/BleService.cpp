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

#include "BleService.h"

BleService::BleService(void)
{
    _initialised = false;
    _connected = false;
}

BleService::BleService(uint16_t uuid16)
    : BleService()
{
    _uuid.type = BT_UUID16;
    _uuid.uuid16 = uuid16;
}

BleService::BleService(uint8_t uuid128[])
    : BleService()
{
    _uuid.type = BT_UUID128;
    memcpy(&_uuid.uuid128, uuid128, MAX_UUID_SIZE);
}

struct bt_uuid BleService::uuid()
{
    return _uuid;
}

BleStatus
BleService::addCharacteristic(BleCharacteristic &ch)
{
    BleStatus status;

    if (!_initialised)
        return BLE_STATUS_WRONG_STATE;
    if (_num_characteristics >= BLE_MAX_CHARACTERISTICS)
        return BLE_STATUS_ERROR;

    /* If this service has a 128-bit UUID, it shall be inherited
     * by included services, characteristics, and descriptors
     */
    if ((BT_UUID128 == _uuid.type) && (BT_UUID16 == ch._uuid.type))
        BLE_UUID16_TO_UUID128(ch._uuid, _uuid);

    status = ble_client_gatts_add_characteristic(_svc_handle,
                                                 &ch._char_data,
                                                 &ch._handles);
    if (BLE_STATUS_SUCCESS == status) {
        ch._initialised = true;
        ch._svc_handle = _svc_handle;
        ch._addCCCDescriptor();
        _characteristics[_num_characteristics++] = &ch;
    }

    return status;
}

BleCharacteristic *
BleService::_matchCharacteristic(uint16_t handle) const
{
    for (unsigned i = 0; i < _num_characteristics; i++) {
        BleCharacteristic *ch = _characteristics[i];
        if (handle == ch->_handles.value_handle)
            return ch;
    }

    /* Check the secondary (included) services as well */
    for (unsigned i = 0; i < _num_services; i++) {
        BleService *svc = _services[i];
        BleCharacteristic *ch = svc->_matchCharacteristic(handle);
        if (ch) /* We've found a match, so return here */
            return ch;
    }

    /* Not found */
    return NULL;
}

BleDescriptor *
BleService::_matchDescriptor(uint16_t handle) const
{
    for (unsigned i = 0; i < _num_characteristics; i++) {
        /* Check primary services */
        BleCharacteristic *ch = _characteristics[i];
        BleDescriptor *desc = ch->_matchDescriptor(handle);
        if (desc) /* We've found a match, so return here */
            return desc;
    }

    /* Check the secondary (included) services as well */
    for (unsigned i = 0; i < _num_services; i++) {
        BleService *svc = _services[i];
        BleDescriptor *desc = svc->_matchDescriptor(handle);
        if (desc) /* We've found a match, so return here */
            return desc;
    }

    /* Not found */
    return NULL;
}

BleStatus
BleService::addSecondaryService(BleService &service)
{
    BleStatus status;

    if (!_initialised)
        return BLE_STATUS_WRONG_STATE;
    if (!_primary)
        return BLE_STATUS_ERROR;
    if (_num_services >= BLE_MAX_INCLUDED_SERVICES)
        return BLE_STATUS_ERROR;

    /* If this service has a 128-bit UUID, it shall be inherited
     * by included services, characteristics, and descriptors
     */
    if ((BT_UUID128 == _uuid.type) && (BT_UUID16 == service._uuid.type))
        BLE_UUID16_TO_UUID128(service._uuid, _uuid);

    status = ble_client_gatts_add_service(&service._uuid,
                                          BLE_GATT_SVC_INCLUDED,
                                          &service._svc_handle);
    if (BLE_STATUS_SUCCESS != status)
        return status;

    status = ble_client_gatts_include_service(_svc_handle,
					      service._svc_handle);
    if (BLE_STATUS_SUCCESS != status)
        return status;

    service._initialised = true;
    service._primary = false;
    _services[_num_services++] = &service;

    return BLE_STATUS_ERROR;
}

BleService *
BleService::_matchService(uint16_t svc_handle) const
{
    for (unsigned i = 0; i < _num_services; i++) {
        BleService *service = _services[i];
        if (svc_handle == service->_svc_handle)
            return service;
    }

    /* Not found */
    return NULL;
}

void
BleService::_setConnectedState(boolean_t connected)
{
    _connected = connected;

    /* Cascade the connected-state update to characteristics
     * and included services */

    for (unsigned i = 0; i < _num_characteristics; i++)
        _characteristics[i]->_setConnectedState(connected);

    for (unsigned i = 0; i < _num_services; i++)
        _services[i]->_setConnectedState(connected);
}
