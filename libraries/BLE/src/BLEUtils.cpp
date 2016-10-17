
#include "ArduinoBLE.h"
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

    for (int i = strLength - 1; i >= 0 && length < MAX_UUID_SIZE; i -= 2)
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

bool BLEUtils::macAddressValid(const bt_addr_le_t &bd_addr)
{
    static const bt_addr_le_t zero = {0,{0,0,0,0,0,0}};
    bool temp = (memcmp(bd_addr.val, zero.val, 6) != 0);//false;//
    #if 0
    for (int i = 0; i < 6; i++)
    {
        if (bd_addr.val[i] != zero.val[i])
        {
        
pr_info(LOG_MODULE_BLE, "%s-idx %d-%.2x:%.2x", __FUNCTION__, i ,bd_addr.val[i], zero.val[i]);
pr_info(LOG_MODULE_BLE,"%s",BLEUtils::macAddressBT2String(zero).c_str());
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
        uuid_tmp.uuid.type = BT_UUID_TYPE_16;
        ((bt_uuid_16_t*)(&uuid_tmp.uuid))->val = temp;
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

BLEDevice& BLEUtils::getLoacalBleDevice()
{
    return BLE;
}

bool BLEUtils::isLocalBLE(BLEDevice& device)
{
    return (device == BLE);
}



