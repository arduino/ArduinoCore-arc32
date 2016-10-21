
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

