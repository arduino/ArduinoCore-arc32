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


#include "CurieBLE.h"
#include "BLEUtils.h"
#include "internal/ble_client.h"

String BLEUtils::macAddressBT2String(const bt_addr_le_t &bd_addr)
{
    char mac_string[BT_ADDR_STR_LEN];
    snprintf(mac_string, BT_ADDR_STR_LEN, "%2.2X:%2.2X:%2.2X:%2.2X:%2.2X:%2.2X",
            bd_addr.val[5], bd_addr.val[4], bd_addr.val[3],
            bd_addr.val[2], bd_addr.val[1], bd_addr.val[0]);
    String temp(mac_string);
    return temp;
}

void BLEUtils::macAddressString2BT(const char* mac_str, bt_addr_le_t &bd_addr)
{
    char temp[] = {0, 0, 0};
    int strLength = strlen(mac_str);
    int length = 0;
    
    bd_addr.type = BT_ADDR_LE_PUBLIC;

    for (int i = strLength - 1; i >= 0 && length < BLE_ADDR_LEN; i -= 2)
    {
        if (mac_str[i] == ':') 
        {
            i++;
            continue;
        }

        temp[0] = mac_str[i - 1];
        temp[1] = mac_str[i];

        bd_addr.val[length] = strtoul(temp, NULL, 16);

        length++;
    }
    
}

bool BLEUtils::macAddressSame(const bt_addr_le_t &bd_addr1, 
                              const bt_addr_le_t &bd_addr2)
{
    bool temp = true;//(memcmp(bd_addr1.val, bd_addr2.val, 6) != 0);//
    #if 1
    for (int i = 0; i < 6; i++)
    {
        if (bd_addr1.val[i] != bd_addr2.val[i])
        {
            temp = false;
            break;
        }
    }
    #endif
    return temp;
    
}

bool BLEUtils::macAddressValid(const bt_addr_le_t &bd_addr)
{
    bool temp = false;
#if 0
    static const bt_addr_le_t zero = {0,{0,0,0,0,0,0}};
    temp = (memcmp(bd_addr.val, zero.val, 6) != 0);
#else
    for (int i = 0; i < 6; i++)
    {
        if (bd_addr.val[i] != 0)
        {      
//pr_info(LOG_MODULE_BLE, "%s-idx %d-%.2x:%.2x", __FUNCTION__, i ,bd_addr.val[i], zero.val[i]);
//pr_info(LOG_MODULE_BLE,"%s",BLEUtils::macAddressBT2String(zero).c_str());
            temp = true;
            break;
        }
    }
#endif
    return temp;
}


bt_addr_le_t* BLEUtils::bleGetLoalAddress()
{
    static bt_addr_le_t board_addr;
    if (false == macAddressValid(board_addr))
        ble_client_get_mac_address(&board_addr);
    return &board_addr;
}



void BLEUtils::uuidString2BT(const char* uuid, bt_uuid_t* pstuuid)
{
    char temp[] = {0, 0, 0};
    int strLength = strlen(uuid);
    int length = 0;
    bt_uuid_128_t uuid_tmp;
    
    memset (&uuid_tmp, 0x00, sizeof(uuid_tmp));

    for (int i = strLength - 1; i >= 0 && length < MAX_UUID_SIZE; i -= 2)
    {
        if (uuid[i] == '-') 
        {
            i++;
            continue;
        }

        temp[0] = uuid[i - 1];
        temp[1] = uuid[i];

        uuid_tmp.val[length] = strtoul(temp, NULL, 16);

        length++;
    }

    if (length == 2)
    {
        uint16_t temp = (uuid_tmp.val[1] << 8)| uuid_tmp.val[0];
        uint8_t* uuid16_val = (uint8_t*)&((bt_uuid_16_t*)(&uuid_tmp.uuid))->val;
        uuid_tmp.uuid.type = BT_UUID_TYPE_16;
        memcpy(uuid16_val, &temp, sizeof (uint16_t));
    }
    else
    {
        uuid_tmp.uuid.type = BT_UUID_TYPE_128;
    }
    memcpy(pstuuid, &uuid_tmp, sizeof (uuid_tmp));
}

void BLEUtils::uuidBT2String(const bt_uuid_t* pstuuid, char* uuid)
{
    unsigned int tmp1, tmp5;
    uint16_t tmp0, tmp2, tmp3, tmp4;
    // TODO: Change the magic number 37
    switch (pstuuid->type) {
    case BT_UUID_TYPE_16:
        memcpy(&tmp0, &BT_UUID_16(pstuuid)->val, sizeof(tmp0));
        snprintf(uuid, 37, "%.4x", tmp0);
        break;
    case BT_UUID_TYPE_128:
        memcpy(&tmp0, &BT_UUID_128(pstuuid)->val[0], sizeof(tmp0));
        memcpy(&tmp1, &BT_UUID_128(pstuuid)->val[2], sizeof(tmp1));
        memcpy(&tmp2, &BT_UUID_128(pstuuid)->val[6], sizeof(tmp2));
        memcpy(&tmp3, &BT_UUID_128(pstuuid)->val[8], sizeof(tmp3));
        memcpy(&tmp4, &BT_UUID_128(pstuuid)->val[10], sizeof(tmp4));
        memcpy(&tmp5, &BT_UUID_128(pstuuid)->val[12], sizeof(tmp5));
        snprintf(uuid, 37, "%.8x-%.4x-%.4x-%.4x-%.8x%.4x",
             tmp5, tmp4, tmp3, tmp2, tmp1, tmp0);
        break;
    default:
        memset(uuid, 0, 37);
        return;
    }
}

bool BLEUtils::uuidBTSame(const bt_uuid_t* pstuuid1,
                          const bt_uuid_t* pstuuid2)
{
    bool temp = (pstuuid1->type == pstuuid2->type);
    if (true == temp)
    {
        if (pstuuid1->type == BT_UUID_TYPE_16)
        {
            temp = (0 == memcmp(&BT_UUID_16(pstuuid1)->val, &BT_UUID_16(pstuuid2)->val, 2));
        }
        else
        {
            temp = (0 == memcmp(BT_UUID_128(pstuuid1)->val, BT_UUID_128(pstuuid2)->val, 16));
        }
    }
    return temp;
    
}

BLEDevice& BLEUtils::getLoacalBleDevice()
{
    return BLE;
}

bool BLEUtils::isLocalBLE(const BLEDevice& device)
{
    return (device == BLE);
}



