/*
 * Copyright (c) 2015 Intel Corporation.  All rights reserved.
 */
#include "BLEUuid.h"

BLEUuid::BLEUuid(const char * str)
{
    char temp[] = {0, 0, 0};
    int strLength = strlen(str);
    int length = 0;

    memset(&_uuid, 0x00, sizeof(_uuid));

    for (int i = strLength - 1; i >= 0 && length < MAX_UUID_SIZE; i -= 2) {
        if (str[i] == '-') {
            i++;
            continue;
        }

        temp[0] = str[i - 1];
        temp[1] = str[i];

        _uuid.uuid128[length] = strtoul(temp, NULL, 16);

        length++;
    }

    if (length == 2) {
        _uuid.type = BT_UUID16;
    } else {
        _uuid.type = BT_UUID128;
    }
}

bt_uuid BLEUuid::uuid() const
{
    return _uuid;
}
