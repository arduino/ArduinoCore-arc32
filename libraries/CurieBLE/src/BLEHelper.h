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

#ifndef _BLE_HELPER_H_
#define _BLE_HELPER_H_


class BLEHelper {
    public:
        /**
         * Is the Central connected
         *
         * @return boolean_t true if the central is connected, otherwise false
         */
        virtual bool connected(void) = 0;

        /**
         * Get the address of the BLE in string format
         *
         * @return const char* address of the BLE in string format
         */
        const char* address(void) const;
        
        /**
         * @brief   Get the address of the BLE in raw format
         *
         * @param   none
         *
         * @return  const bt_addr_le_t *    address of the BLE in raw format
         *
         * @note  none
         */
        const bt_addr_le_t *bt_le_address(void) const;
        /**
         * Disconnect the central if it is connected
         *
         */
        virtual bool disconnect(void) = 0;

        /**
         * Poll the central for events
         */
        void poll(void);

        operator bool(void) const;
        bool operator==(const BLEHelper& rhs) const;
        bool operator==(const bt_addr_le_t& rhs) const;
        bool operator!=(const BLEHelper& rhs) const;
        
        /**
         * @brief   Get the connection paramter
         *
         * @param[out]   user_conn_params    connection paramter
         *                                Minimum Connection Interval (ms)
         *                                Maximum Connection Interval (ms)
         *                                Connection Latency 
         *                                Supervision Timeout (ms)      
         *
         * @return        none   
         *
         * @note  none
         */
        void getConnParams(ble_conn_param_t &user_conn_params);
        
        /**
         * @brief   Set the connection paramter and send connection 
         *           update request
         *
         * @param[in]   intervalmin     Minimum Connection Interval (ms)
         *
         * @param[in]   intervalmax     Maximum Connection Interval (ms)
         *
         * @return  none
         *
         * @note  none
         */
        void setConnectionInterval(float minInterval, 
                                   float maxInterval);
                                   
        /**
         * @brief   Set the connection paramter and send connection 
         *           update request
         *
         * @param[in]   intervalmin     Minimum Connection Interval (ms)
         *
         * @param[in]   intervalmax     Maximum Connection Interval (ms)
         *
         * @param[in]   latency         Connection Latency
         *
         * @param[in]   timeout         Supervision Timeout (ms)
         *
         * @return  none
         *
         * @note  none
         */
        void setConnectionInterval(float minInterval, 
                                   float maxInterval, 
                                   uint16_t latency, 
                                   uint16_t timeout);                          
        
        /**
         * @brief   Just set the connection parameter. 
         *           Not send out connection update request.
         *
         * @param[in]   intervalmin     Minimum Connection Interval (N * 1.25 ms)
         *
         * @param[in]   intervalmax     Maximum Connection Interval (N * 1.25 ms)
         *
         * @param[in]   latency         Connection Latency
         *
         * @param[in]   timeout         Supervision Timeout (N * 10 ms)
         *
         * @return  none
         *
         * @note    The user should care the unit
         */
        void setConnectionParameters(uint16_t intervalmin, 
                                     uint16_t intervalmax, 
                                     uint16_t latency, 
                                     uint16_t timeout);
        
        /**
         * @brief   Schedule the link connection update request
         *
         * @param[in]   intervalmin     Minimum Connection Interval (N * 1.25 ms)
         *
         * @param[in]   intervalmax     Maximum Connection Interval (N * 1.25 ms)
         *
         * @param[in]   latency         Connection Latency
         *
         * @param[in]   timeout         Supervision Timeout (N * 10 ms)
         *
         * @return  none
         *
         * @note        The user should care the unit
         */
        void updateConnectionInterval(uint16_t intervalmin, 
                                      uint16_t intervalmax, 
                                      uint16_t latency, 
                                      uint16_t timeout);
        
        /**
         * @brief   Schedule the link connection update request
         *
         * @return  none
         *
         * @note    The connection update request will not send if 
         *           parameter doesn't changed
         */
        void updateConnectionInterval();

    protected:
        void setAddress(const bt_addr_le_t &address);
        void clearAddress();
        BLEHelper();
        virtual ~BLEHelper();

    private:
        bt_addr_le_t   _address; /// BT low energy address
        bt_le_conn_param_t _conn_params; /// Connection parameter
};

#endif


