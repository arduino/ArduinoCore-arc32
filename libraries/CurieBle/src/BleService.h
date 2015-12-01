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

#ifndef _BLE_SERVICE_H_INCLUDED
#define _BLE_SERVICE_H_INCLUDED

#include "BleAttribute.h"
#include "BleCommon.h"
#include "BleCharacteristic.h"

#include "internal/ble_client.h"

/* Forward declaration needed for callback function prototype below */
class BlePeripheral;

/**
 * BLE GATT Service
 */
class BleService : public BleAttribute {
public:
    /**
     * Constructor for BLE Service with 16-bit UUID
     *
     * @param uuid16    16-bit UUID defined by BLE standard
     */
    BleService(const char* uuid);

protected:
    /**
     * Add a BLE Characteristic for this Service
     *
     * NOTE: If this service was configured with a custom 128-bit UUID,
     * and if this added characteristic was configured with a 16-bit UUID,
     * that 16-bit characteristic UUID will be converted to a 128-bit UUID
     * using this service's UUID as a base (only bytes 12-13 will differ).
     *
     * @param characteristic BLE Characteristic reference
     *
     * @return BleStatus indicating success or error
     *
     * @note This method must be called only before advertising is started.
     */
    BleStatus addCharacteristic(BleCharacteristic &characteristic);


    friend class BlePeripheral;

private:

    BleCharacteristic *_matchCharacteristic(uint16_t value_handle) const;
    BleDescriptor     *_matchDescriptor(uint16_t handle) const;
    void              _setConnectedState(boolean_t connected);

    boolean_t      _initialised;
    uint16_t       _svc_handle;

    BleCharacteristic *_characteristics[BLE_MAX_CHARACTERISTICS];
    uint32_t           _num_characteristics;
};

#endif // _BLE_SERVICE_H_INCLUDED
