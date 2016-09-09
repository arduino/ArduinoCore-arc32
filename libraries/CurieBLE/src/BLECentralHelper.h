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

#ifndef _BLE_CENTRAL_HELPER_H_INCLUDED
#define _BLE_CENTRAL_HELPER_H_INCLUDED

#include "BLECommon.h"
#include "BLEHelper.h"

class BLEPeripheralRole;

class BLECentralHelper: public BLEHelper{
    friend class BLEPeripheralRole;
    friend class BLECentralRole;

    public:
        /**
         * Is the Central connected
         *
         * @return boolean_t true if the central is connected, otherwise false
         */
        bool connected(void);
        
        /**
         * Disconnect the central if it is connected
         *
         */
        bool disconnect(void);

        /**
         * Poll the central for events
         */
        void poll(void);

    protected:
        BLECentralHelper(BLEPeripheralRole* peripheral);

    private:
        BLEPeripheralRole* _peripheral;
};

#endif
