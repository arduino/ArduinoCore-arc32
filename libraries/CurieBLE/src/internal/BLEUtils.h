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


namespace BLEUtils
{
    String macAddressBT2String(const bt_addr_le_t &bd_addr);
    void macAddressString2BT(const char* mac_str, bt_addr_le_t &bd_addr);
    bool macAddressValid(const bt_addr_le_t &bd_addr);
    bool macAddressSame(const bt_addr_le_t &bd_addr1, const bt_addr_le_t &bd_addr2);
    bt_addr_le_t* bleGetLoalAddress();
    void uuidString2BT(const char* uuid, bt_uuid_t* pstuuid);
    void uuidBT2String(const bt_uuid_t* pstuuid, char* uuid);
    bool uuidBTSame(const bt_uuid_t* pstuuid1,
                    const bt_uuid_t* pstuuid2);
    
    BLEDevice& getLoacalBleDevice();
    bool isLocalBLE(const BLEDevice& device);
}

