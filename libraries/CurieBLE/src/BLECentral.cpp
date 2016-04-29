/*
 * Copyright (c) 2015 Intel Corporation.  All rights reserved.
 */
#include "BLECentral.h"
#include "BLEPeripheral.h"


BLECentral::BLECentral(BLEPeripheral* peripheral) :
  _peripheral(peripheral)
{
    clearAddress();
}

BLECentral::operator bool() const {
    ble_addr_t zero;

    memset(&zero, 0, sizeof(zero));

    return (memcmp(&_address, &zero, sizeof(_address)) != 0);
}

bool
BLECentral::operator==(const BLECentral& rhs) const {
    return (memcmp(&_address, &rhs._address, sizeof(_address)) == 0);
}

bool
BLECentral::operator!=(const BLECentral& rhs) const {
    return !(*this == rhs);
}

bool
BLECentral::connected() {
    poll();

    return (*this && *this == _peripheral->central());
}

const char* 
BLECentral::address() const {
    static char address[18];

    String addressStr = "";

    for (int i = 5; i >= 0; i--) {
        unsigned char a = _address.addr[i];

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
BLECentral::poll() {
    _peripheral->poll();
}

bool
BLECentral::disconnect() {
    if (connected()) {
        return _peripheral->disconnect();
    }

    return false;
}

void
BLECentral::setAddress(ble_addr_t address) {
    _address = address;
}

void
BLECentral::clearAddress() {
    memset(&_address, 0x00, sizeof(_address));
}
