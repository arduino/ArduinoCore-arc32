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

/**
 *  @brief A class defining the BLE central function
 *
 *  This class abstract the BLE central. 
 */
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
     * @param[in]   interval        The scan interval in ms
     *
     * @param[in]   window          The scan window in ms
     *
     * @return  bool    Indicate the success or error
     *
     * @note  none
     */
    bool startScan(float interval, float window);
    
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
     * @param[in]   addr    The MAC address of peripheral device that want to establish connection
     *
     * @param[in]   param   The connetion parameters
     *
     * @return  bool    Indicate the success or error
     *
     * @note  none
     */
    bool connect(const bt_addr_le_t *addr, const ble_conn_param_t *param);
    
    /**
     * @brief   Discover the peripheral device profile
     *
     * @param[in]   peripheral  The Peripheral that need to discover the profile
     *
     * @return  none
     *
     * @note  none
     */
    void discover(BLEPeripheralHelper &peripheral);
    
    /**
     * @brief   Set the scan parameter
     *
     * @param[in]   interval        The scan interval in ms
     *
     * @param[in]   window          The scan window in ms
     *
     * @return  none
     *
     * @note  1. The scale of the interval and window are 2.5 - 10240ms
     *        2. The scan interval and window are like below.
     *              The device can see the ADV packet in the window.
     *             window
     *              ----     ----
     *              |  |     |  |
     *            ---  -------  ----
     *              |interval|
     */
    void setScanParam(float interval, float window);
    
    /**
     * @brief   Add an attribute to the BLE Central Device
     *
     * @param[in]   attribute   Attribute to add to Central
     *
     * @return  BleStatus indicating success or error
     *
     * @note  The attribute will used for discover the peripheral handler
     *          Only need check return value at first call. Memory only alloc at first call
     */
    BleStatus addAttribute(BLEAttribute& attribute);
    
    /**
     * Provide a function to be called when events related to this Device are raised
     *
     * @param[in] event    Event type for callback
     * @param[in] callback Pointer to callback function to invoke when an event occurs.
     */
    void setEventHandler(BLERoleEvent event, BLERoleEventHandler callback);
    
    /**
     * @brief   Provide a function to be called when scanned the advertisement
     *
     * @param[in]   advcb   Pointer to callback function to invoke when advertisement received
     *
     * @return  none
     *
     * @note  none
     */
    void setAdvertiseHandler(ble_advertise_handle_cb_t advcb);
    
    /**
     * @brief Setup attributes and start scan
     *
     * @return bool indicating success or error
     */
    bool begin(void);
    
    /**
     * @brief Get peer peripheral device
     *
     *@param   peripheral peer peripheral device of the central board  
     *
     * @return pointer of peer peripheral device
     */
    BLEPeripheralHelper  *getPeerPeripheralBLE(BLEHelper& peripheral);	
protected:
private:
    
};

#endif
