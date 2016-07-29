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

#ifndef __BLEROLEBASE_H__
#define __BLEROLEBASE_H__

#include "BLECommon.h"

/**
 * BLE Events
 */
enum BLERoleEvent {
  BLEConnected = 0,
  BLEDisconnected = 1,
  BLEUpdateParam,
  
  BLERoleEventLast
};

class BLEHelper;

typedef void (*BLERoleEventHandler)(BLEHelper &role);


class BLERoleBase{
public:
    virtual bool begin()=0;
    virtual bool disconnect()=0;
    BLERoleBase();
    
    /**
     * Is connected?
     *
     * @return boolean_t true if established connection, otherwise false
     */
    bool connected (void) {return m_connected;}
    
    /**
     * Set TX output power
     *
     * @param   tx_power    The antenna TX power
     *
     * @return boolean_t true if established connection, otherwise false
     */
    void setTxPower (int8_t tx_power);
protected:
    virtual BleStatus _init(void);
    
    friend void bleConnectEventHandler(struct bt_conn *conn, 
                                        uint8_t err, 
                                        void *param);
    friend void bleDisconnectEventHandler(struct bt_conn *conn, 
                                            uint8_t reason, 
                                            void *param);
    friend void bleParamUpdatedEventHandler(struct bt_conn *conn, 
                                             uint16_t interval,
                                             uint16_t latency, 
                                             uint16_t timeout, 
                                             void *param);
    
    /**
     * @brief   Handle the connected event
     *
     * @param   conn    The object that established the connection
     *
     * @param   err     The code of the process
     *
     * @return  none
     *
     * @note  virtual function. Just define the interface and the children need to implement
     */
    virtual void handleConnectEvent(struct bt_conn *conn, uint8_t err) = 0;
    
    /**
     * @brief   Handle the disconnected event
     *
     * @param   conn    The object that lost the connection
     *
     * @param   reason  The link lost reason
     *
     * @return  none
     *
     * @note  virtual function. Just define the interface and the children need to implement
     */
    virtual void handleDisconnectEvent(struct bt_conn *conn, uint8_t reason) = 0;
    
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
     * @note  virtual function. Just define the interface and the children need to implement
     */
    virtual void handleParamUpdated (struct bt_conn *conn, 
                                     uint16_t interval,
        				             uint16_t latency, 
        				             uint16_t timeout) = 0;

    char       _device_name[BLE_MAX_DEVICE_NAME+1];
    bt_addr_le_t _local_bda;
    BLERoleEventHandler _event_handlers[BLERoleEventLast];

private:
    bool m_connected;
    static uint8_t m_init_cnt; // Reserved for support multi-role at same time
};

#endif

