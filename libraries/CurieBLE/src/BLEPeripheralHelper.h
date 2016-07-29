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

#ifndef _BLE_PERIPHERAL_HELPER_H_
#define _BLE_PERIPHERAL_HELPER_H_

#include "BLECommon.h"
#include "BLEHelper.h"
#include "BLEProfile.h"

class BLEAttribute;
class BLECentralRole;

class BLEPeripheralHelper : public BLEHelper {
    friend class BLECentralRole;
    friend class BLEPeripheralRole;
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
         * Add an attribute to the BLE Peripheral helper
         *
         * @param attribute Attribute to add to Peripheral
         *
         * @note This method must be called before the begin method
         */
        void addAttribute(BLEAttribute& attribute);
    
        BLEAttribute *attribute(struct bt_gatt_subscribe_params *params);
        BLEAttribute *attribute(uint16_t handle);
        
        /**
         * For central to discover the profile
         */
        void discover();
        void discover(const struct bt_gatt_attr *attr);

        // For peripheral to register the tree
        int registerProfile();
        void linkLost(void);
        
        // Get value handle.
        // 0 is invalid
        uint16_t valueHandle(BLEAttribute *attr);
        uint16_t cccdHandle(BLEAttribute *attr);

    protected:
        BLEPeripheralHelper(BLECentralRole* central);
        ~BLEPeripheralHelper();

    private:
        BLEProfile          _profile;
        BLECentralRole*     _central;
};

#endif

