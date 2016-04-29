/*
 * Copyright (c) 2015 Intel Corporation.  All rights reserved.
 */
#ifndef _BLE_CENTRAL_H_INCLUDED
#define _BLE_CENTRAL_H_INCLUDED

#include "BLECommon.h"

class BLEPeripheral;

class BLECentral {
    friend class BLEPeripheral;

    public:
        /**
         * Is the Central connected
         *
         * @return boolean_t true if the central is connected, otherwise false
         */
        bool connected(void);

        /**
         * Get the address of the Central in string form
         *
         * @return const char* address of the Central in string form
         */
        const char* address(void) const;
        
        /**
         * Disconnect the central if it is connected
         *
         */
        bool disconnect(void);

        /**
         * Poll the central for events
         */
        void poll(void);

        operator bool(void) const;
        bool operator==(const BLECentral& rhs) const;
        bool operator!=(const BLECentral& rhs) const;

    protected:
        BLECentral(BLEPeripheral* peripheral);
        void setAddress(ble_addr_t address);
        void clearAddress();

    private:
        BLEPeripheral* _peripheral;
        ble_addr_t     _address;
};

#endif
