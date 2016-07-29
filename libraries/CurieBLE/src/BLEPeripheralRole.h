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

#ifndef _BLE_PERIPHERALROLE_H_INCLUDED
#define _BLE_PERIPHERALROLE_H_INCLUDED

#include "internal/ble_client.h"

#include "BLECommon.h"
#include "BLERoleBase.h"
#include "BLEPeripheralHelper.h"

/**
 * BLE Peripheral Role
 */
class BLEPeripheralRole: public BLERoleBase{
public:
    /**
     * Default Constructor for BLE Peripheral Device
     */
    BLEPeripheralRole(void);

    /**
     * Destructor for BLE Peripheral Device
     */
    virtual ~BLEPeripheralRole(void);

    /**
     * Set the device name for the BLE Peripheral Device
     *
     * If device name is not set, a default name will be used instead
     *
     * @param deviceName  User-defined name string for this device.  Truncated if
     *                     more than maximum allowed string length (20 bytes).
     *
     * @note This method must be called before the begin method
     */
    void setDeviceName(const char *deviceName);

    /**
     * Set the min and max connection interval BLE Peripheral Device
     *
     * @param minConnInterval Minimum connection interval (1.25 ms units), minimum 0x0006 (7.5ms)
     * @param maxConnInterval Maximum connection interval (1.25 ms units), maximum 0x095f (2998.75ms)
     *
     * @note This method must be called before the begin method
     */
    void setConnectionInterval(const unsigned short minConnInterval, const unsigned short maxConnInterval);

    /**
     * Add an attribute to the BLE Peripheral Device
     *
     * @param attribute Attribute to add to Peripheral
     *
     * @return BleStatus indicating success or error
     *
     * @note This method must be called before the begin method
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
     * Setup attributes and start advertising
     *
     * @return bool indicating success or error
     */
    bool begin(void);

    /**
     * Poll the peripheral for events
     */
    void poll(void);

    /**
     * Stop advertising and disconnect a central if connected
     */
    BleStatus stop(void);

    /**
     * Disconnect the central connected if there is one connected
     *
     * @return bool indicating success or error
     */
    bool disconnect(void);

    /**
     * Setup attributes and start advertising
     *
     * @return BleStatus indicating success or error
     */
    BLECentralHelper central(void);

    /**
     * Is a central connected?
     *
     * @return boolean_t true if central connected, otherwise false
     */
    bool connected(void);
    
    /**
     * @brief   Start peripheral advertising
     *
     * @param   ad      The ADV data array
     *
     * @param   ad_len  The ADV data array length
     *
     * @param   sd      The Scan response data array
     *
     * @param   sd_len  The Scan response data array length
     *
     * @return  BleStatus
     *
     * @note  none
     */
    BleStatus startAdvertising(const struct bt_data *ad, 
                               size_t ad_len,
                               const struct bt_data *sd, 
                               size_t sd_len);
	
	/**
	 * @brief   Stop send advertisement
	 *
	 * @param   none
	 *
	 * @return  none
	 *
	 * @note  none
	 */
    BleStatus stopAdvertising();
	
    /**
     * @brief   Set advertising parameter
     *
     * @param   type            Advertising type
     *
     * @param   interval_min    Minimum Advertising Interval (N * 0.625)
     *
     * @param   interval_max    Maximum Advertising Interval (N * 0.625)
     *
     * @return  none
     *
     * @note  none
     */
    void setAdvertisingParam(uint8_t  type, 
                             uint16_t interval_min,
                             uint16_t interval_max);
    
    /**
     * @brief   Get BLE Peripheral instance.
     *
     * @param   none
     *
     * @return  BLEPeripheralRole*  The BLE perpheral instance
     *
     * @note    Singleton. Only have one object to communicate with 
     *           stack and manage the device
     */
    static BLEPeripheralRole* instance();
    
protected:
    /**
     * @brief   Handle the connected event
     *
     * @param   conn    The object that established the connection
     *
     * @param   err     The code of the process
     *
     * @return  none
     *
     * @note  none
     */
    void handleConnectEvent(struct bt_conn *conn, uint8_t err);
    
    /**
     * @brief   Handle the disconnected event
     *
     * @param   conn    The object that lost the connection
     *
     * @param   reason  The link lost reason
     *
     * @return  none
     *
     * @note  none
     */
    void handleDisconnectEvent(struct bt_conn *conn, uint8_t reason);
    
    /**
     * @brief   Handle the conntion update request
     *
     * @param   conn    The connection object that need to process the update request
     *
     * @param   interval    The connection interval
     *
     * @param   latency     The connection latency
     *
     * @param   timeout     The connection timeout
     *
     * @return  none
     *
     * @note  none
     */
    void handleParamUpdated(struct bt_conn *conn, 
                            uint16_t interval,
				            uint16_t latency, 
				            uint16_t timeout);

private:

    /**
     * Set the device name to Nordic BLE's profile
     *
     * @param none
     *
     * @note none
     */
    void setDeviceName();

    enum BLEPeripheralState {
        BLE_PERIPH_STATE_NOT_READY = 0,
        BLE_PERIPH_STATE_READY,
        BLE_PERIPH_STATE_ADVERTISING,
        BLE_PERIPH_STATE_CONNECTED,
    };

    BLEPeripheralState _state;

    uint16_t   _min_conn_interval;
    uint16_t   _max_conn_interval;
    
    struct bt_le_adv_param _adv_param;
    
    BLEPeripheralHelper _peripheral;
    BLECentralHelper    _central;

    BLERoleEventHandler _event_handlers[BLERoleEventLast];
    static BLEPeripheralRole *_ins;
};

#endif // _BLE_DEVICE_H_INCLUDED
