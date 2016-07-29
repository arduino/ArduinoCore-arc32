/*
 * Copyright (c) 2016 Intel Corporation.  All rights reserved.
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

#include "BLECentralHelper.h"

#include "BLEPeripheralRole.h"


BLECentralHelper::BLECentralHelper(BLEPeripheralRole* peripheral) :
  _peripheral(peripheral)
{
    clearAddress();
}

BLECentralHelper::operator bool() const {
    bt_addr_le_t zero;

    memset(&zero, 0, sizeof(zero));

    return (memcmp(&_address, &zero, sizeof(_address)) != 0);
}

bool
BLECentralHelper::operator==(const BLECentralHelper& rhs) const {
    return (memcmp(&_address, &rhs._address, sizeof(_address)) == 0);
}

bool
BLECentralHelper::operator!=(const BLECentralHelper& rhs) const {
    return !(*this == rhs);
}

bool
BLECentralHelper::connected() {
    poll();

    return (*this && *this == _peripheral->central());
}

const char* 
BLECentralHelper::address() const {
    static char address[18];

    String addressStr = "";

    for (int i = 5; i >= 0; i--) {
        unsigned char a = _address.val[i];

        if (a < 0x10) {
            addressStr += "0";
        }

        addressStr += String(a, 16);

        if (i > 0) {
            addressStr += ":";
        }
    }

    strcpy(address, addressStr.c_str());

    return address;
}

void
BLECentralHelper::poll() {
    _peripheral->poll();
}

bool
BLECentralHelper::disconnect() {
    if (connected()) {
        return _peripheral->disconnect();
    }

    return false;
}

void
BLECentralHelper::setAddress(bt_addr_le_t address) {
    _address = address;
}

void
BLECentralHelper::clearAddress() {
    memset(&_address, 0x00, sizeof(_address));
}
