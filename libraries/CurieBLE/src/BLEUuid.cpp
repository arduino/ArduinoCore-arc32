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
