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


#ifndef __BLECALLBACKS_H__
#define __BLECALLBACKS_H__

uint8_t profile_notify_process (bt_conn_t *conn,
                                bt_gatt_subscribe_params_t *params,
                                const void *data, uint16_t length);
uint8_t profile_read_rsp_process(bt_conn_t *conn, int err,
                                 bt_gatt_read_params_t *params,
                                 const void *data, 
                                 uint16_t length);
int profile_longflush_process(struct bt_conn *conn,
                              const struct bt_gatt_attr *attr, 
                              uint8_t flags);
ssize_t profile_longwrite_process(struct bt_conn *conn,
                                     const struct bt_gatt_attr *attr,
                                     const void *buf, uint16_t len,
                                     uint16_t offset);
ssize_t profile_write_process(bt_conn_t *conn,
                              const bt_gatt_attr_t *attr,
                              const void *buf, uint16_t len,
                              uint16_t offset);
ssize_t profile_read_process(bt_conn_t *conn,
                             const bt_gatt_attr_t *attr,
                             void *buf, uint16_t len,
                             uint16_t offset);

uint8_t profile_discover_process(bt_conn_t *conn,
                                 const bt_gatt_attr_t *attr,
                                 bt_gatt_discover_params_t *params);

void bleConnectEventHandler(bt_conn_t *conn, 
                            uint8_t err, 
                            void *param);

void bleDisconnectEventHandler(bt_conn_t *conn, 
                                uint8_t reason, 
                                void *param);

void bleParamUpdatedEventHandler(bt_conn_t *conn, 
                                 uint16_t interval,
                                 uint16_t latency, 
                                 uint16_t timeout, 
                                 void *param);

void ble_central_device_found(const bt_addr_le_t *addr, 
                              int8_t rssi, 
                              uint8_t type,
                              const uint8_t *ad, 
                              uint8_t len);

uint8_t profile_service_read_rsp_process(bt_conn_t *conn, 
                                 int err,
                                 bt_gatt_read_params_t *params,
                                 const void *data, 
                                 uint16_t length);

#endif

