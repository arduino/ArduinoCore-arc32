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

#include "BLEAttribute.h"
#include "BLECommon.h"

/**
 * BLE GATT Service
 */
class BLEService : public BLEAttribute {
public:
    /**
     * Constructor for BLE Service
     *
     * @param uuid    16-bit or 128-bit UUID (in string form) defined by BLE standard
     */
    BLEService(const char* uuid);

protected:
    friend BLEPeripheral;

    bool add(void);
};

#endif // _BLE_SERVICE_H_INCLUDED
