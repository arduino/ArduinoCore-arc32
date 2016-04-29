/*
 * Copyright (c) 2015 Intel Corporation.  All rights reserved.
 */
#ifndef _BLE_UUID_H_INCLUDED
#define _BLE_UUID_H_INCLUDED

#include "BLECommon.h"

class BLEUuid
{
public:
    BLEUuid(const char * str);

    bt_uuid uuid(void) const;

private:
    struct bt_uuid _uuid;
};

#endif // _BLE_UUID_H_INCLUDED
