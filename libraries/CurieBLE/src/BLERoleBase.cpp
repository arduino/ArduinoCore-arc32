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

#include "internal/ble_client.h"

#include "BLERoleBase.h"

void bleConnectEventHandler(struct bt_conn *conn, 
                            uint8_t err, 
                            void *param)
{
    BLERoleBase* p = (BLERoleBase*)param;

    p->handleConnectEvent(conn, err);
}


void bleDisconnectEventHandler(struct bt_conn *conn, 
                                uint8_t reason, 
                                void *param)
{
    BLERoleBase* p = (BLERoleBase*)param;
    
    pr_info(LOG_MODULE_BLE, "Connect lost. Reason: %d", reason);

    p->handleDisconnectEvent(conn, reason);
}

void bleParamUpdatedEventHandler(struct bt_conn *conn, 
                                 uint16_t interval,
                                 uint16_t latency, 
                                 uint16_t timeout, 
                                 void *param)
{
    BLERoleBase* p = (BLERoleBase*)param;

    p->handleParamUpdated(conn, interval, latency, timeout);
}

uint8_t BLERoleBase::m_init_cnt = 0;

void BLERoleBase::setTxPower (int8_t tx_power)
{
    ble_gap_set_tx_power(tx_power);
}


BleStatus
BLERoleBase::_init()
{
	// Curie may support multi-role at same time in future.
	//  Make sure the BLE only init once.
	if (this->m_init_cnt == 0)
	{
		ble_client_init(bleConnectEventHandler, this,
		                 bleDisconnectEventHandler, this,
		                 bleParamUpdatedEventHandler, this);
	}
	this->m_init_cnt++;

    return BLE_STATUS_SUCCESS;
}

BLERoleBase::BLERoleBase(): m_connected(false)
{
    memset (_event_handlers, 0x00, sizeof (_event_handlers));
    ble_client_get_factory_config(&_local_bda, _device_name);
}


