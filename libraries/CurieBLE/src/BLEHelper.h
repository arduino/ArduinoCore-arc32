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
         * Get the address of the Central in string form
         *
         * @return const char* address of the Central in string form
         */
        const char* address(void) const;
        
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
        void operator=(const BLEHelper& rhs);
        void setConn(struct bt_conn *conn);
        
        const struct bt_le_conn_param *getConnParams();
        void setConnParames(uint16_t intervalmin, 
                            uint16_t intervalmax, 
                            uint16_t latency, 
                            uint16_t timeout);

    protected:
        void setAddress(bt_addr_le_t address);
        void clearAddress();
        BLEHelper();
        virtual ~BLEHelper();

    private:
        bt_addr_le_t   _address;
        //struct bt_conn *_conn;
        struct bt_le_conn_param _conn_params;
};

#endif


