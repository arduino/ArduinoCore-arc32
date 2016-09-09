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
         * @param[in] attribute Attribute to add to Peripheral
         *
         * @return  BleStatus indicating success or error
         *
         * @note This method must be called before the begin method
         */
        BleStatus addAttribute(BLEAttribute& attribute);
    
        /**
         * @brief   Get BLEAttribute by subscribe parameter
         *
         * @param[in]   params              Subscribe parameter
         *
         * @return  BLEAttribute *      NULL - Not found
         *                              Not NULL - The BLEAttribute object
         *
         * @note  none
         */
        BLEAttribute *attribute(bt_gatt_subscribe_params_t *params);
        
        /**
         * @brief   Get BLEAttribute by characteristic handle
         *
         * @param[in]   handle              The characteristic handle
         *
         * @return  BLEAttribute *      NULL - Not found
         *                              Not NULL - The BLEAttribute object
         *
         * @note  none
         */
        BLEAttribute *attribute(uint16_t handle);
        
        /**
         * @brief   Discover the BLE peripheral profile for central
         *
         * @param   none
         *
         * @return  none
         *
         * @note  This function only for the central device.
         *
         * @note  The central deivce didn't know the connected BLE's profile.
         *         Need send discover request to search the attribute in the BLE peripheral
         */
        void discover();
        
        /**
         * @brief   Process the discover response and 
         *           discover the BLE peripheral profile
         *
         * @param[in]   const bt_gatt_attr_t *     The gatt attribute response
         *
         * @return  uint8_t     BT_GATT_ITER_STOP   Stop discover the profile
         *                      BT_GATT_ITER_CONTINUE Continue to send the discover request
         *
         * @note  This function only for the central device.
         */
        uint8_t discover(const bt_gatt_attr_t *attr);

        /**
         * @brief   For peripheral to register the profile tree
         *
         * @param   none
         *
         * @return  int         0 - success
         *                      other - error code
         *
         * @note  none
         */
        int registerProfile();
        
        /**
         * @brief   Process the link lost event
         *
         * @param   none
         *
         * @return  none
         *
         * @note  none
         */
        void linkLost(void);
        
        /**
         * @brief   Get the characteristic value handle
         *
         * @param[in]   attr    Attribute object 
         *
         * @return  uint16_t    The value hander of attribute
         *                       0 is invalid
         *
         * @note  Only for central mode
         */
        uint16_t valueHandle(BLEAttribute *attr);
        
        /**
         * @brief   Get characteristic configuration descriptor value handle
         *
         * @param[in]   attr    Attribute object 
         *
         * @return  uint16_t    The value hander of attribute
         *                       0 is invalid
         *
         * @note  Only for central mode
         */
        uint16_t cccdHandle(BLEAttribute *attr);

    protected:
        BLEPeripheralHelper(BLECentralRole* central);
        ~BLEPeripheralHelper();

    private:
        BLEProfile          _profile;
        BLECentralRole*     _central;
};

#endif

