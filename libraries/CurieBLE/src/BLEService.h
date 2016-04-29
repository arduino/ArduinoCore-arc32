/*
 * Copyright (c) 2015 Intel Corporation.  All rights reserved.
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
