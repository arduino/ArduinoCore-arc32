
namespace BLEUtils
{
    String macAddressBT2String(const bt_addr_le_t &bd_addr);
    void macAddressString2BT(const char* mac_str, bt_addr_le_t &bd_addr);
    bool macAddressValid(const bt_addr_le_t &bd_addr);
    bt_addr_le_t* bleGetLoalAddress();
    void uuidString2BT(const char* uuid, bt_uuid_t* pstuuid);
    void uuidBT2String(const bt_uuid_t* pstuuid, char* uuid);
    BLEDevice& getLoacalBleDevice();
    bool isLocalBLE(BLEDevice& device);
}

