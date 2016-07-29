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

#ifndef _BLE_CENTRAL_H_INCLUDED
#define _BLE_CENTRAL_H_INCLUDED

#include "BLECommon.h"
#include "BLERoleBase.h"

class BLEAttribute;

class BLECentral{
public:
    /**
     * @brief   Start scan
     *
     * @param   none
     *
     * @return  bool    Indicate the success or error
     *
     * @note  none
     */
    bool startScan();
    
    /**
     * @brief   Start scan with scan parameter
     *
     * @param   none
     *
     * @return  bool    Indicate the success or error
     *
     * @note  none
     */
    bool startScan(const struct bt_le_scan_param &scan_param);
    
    /**
     * @brief   Stop scan
     *
     * @param   none
     *
     * @return  bool    Indicate the success or error
     *
     * @note  none
     */
    bool stopScan();
    
    /**
     * @brief   Schedule a connect request to peripheral to establish a connection
     *
     * @param   addr    The MAC address of peripheral device that want to establish connection
     *
     * @param   param   The connetion parameters
     *
     * @return  bool    Indicate the success or error
     *
     * @note  none
     */
    bool connect(const bt_addr_le_t *addr, const struct bt_le_conn_param *param);
    
    /**
     * @brief   Discover the peripheral device profile
     *
     * @param   peripheral  The Peripheral that need to discover the profile
     *
     * @return  none
     *
     * @note  none
     */
    void discover(BLEPeripheralHelper &peripheral);
    
    /**
     * @brief   Set the scan parameter
     *
     * @param   scan_param      The scan parameter want to be set
     *
     * @return  none
     *
     * @note  none
     */
    void setScanParam(const struct bt_le_scan_param &scan_param);
    
    /**
     * @brief   Add an attribute to the BLE Central Device
     *
     * @param   attribute   Attribute to add to Central
     *
     * @return  none
     *
     * @note  The attribute will used for discover the peripheral handler
     */
    void addAttribute(BLEAttribute& attribute);
    
    /**
     * Provide a function to be called when events related to this Device are raised
     *
     * @param event    Event type for callback
     * @param callback Pointer to callback function to invoke when an event occurs.
     */
    void setEventHandler(BLERoleEvent event, BLERoleEventHandler callback);
    
    /**
     * @brief   Provide a function to be called when scanned the advertisement
     *
     * @param   advcb   Pointer to callback function to invoke when advertisement received
     *
     * @return  none
     *
     * @note  none
     */
    void setAdvertiseHandler(ble_advertise_handle_cb_t advcb);
    
    /**
     * Setup attributes and start scan
     *
     * @return bool indicating success or error
     */
    bool begin(void);
protected:
private:
    
};

#endif
