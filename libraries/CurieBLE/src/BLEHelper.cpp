
#include "BLECentralHelper.h"

#include "BLEPeripheral.h"


BLEHelper::BLEHelper() 
{
    clearAddress();
    memset(&_conn_params, 0x00, sizeof(_conn_params));
    _conn_params.interval_max = BT_GAP_INIT_CONN_INT_MAX;
    _conn_params.interval_min = BT_GAP_INIT_CONN_INT_MIN;
    _conn_params.latency = 0;
    _conn_params.timeout = 400;
}
  
BLEHelper::~BLEHelper()
{
    #if 0
    if (NULL != _conn)
    {
        bt_conn_unref(_conn);
    }
    #endif
}

#if 0
void BLEHelper::setConn(struct bt_conn *conn)
{
    if (conn == _conn)
    {
        return;
    }
    
    if (NULL != _conn)
    {
        bt_conn_unref(_conn);
    }
    _conn = conn;
}
#endif

BLEHelper::operator bool() const 
{
    bt_addr_le_t zero;

    memset(&zero, 0, sizeof(zero));

    return (memcmp(&_address, &zero, sizeof(_address)) != 0);
}

bool BLEHelper::operator==(const BLEHelper& rhs) const 
{
    return (memcmp(&_address, &rhs._address, sizeof(_address)) == 0);
}

bool
BLEHelper::operator==(const bt_addr_le_t& address) const {
    return (memcmp(&_address, &address, sizeof(_address)) == 0);
}

bool
BLEHelper::operator!=(const BLEHelper& rhs) const {
    return !(*this == rhs);
}

const char* 
BLEHelper::address() const {
    static char address[18];

    String addressStr = "";

    for (int i = 5; i >= 0; i--) {
        unsigned char a = _address.val[i];

        if (a < 0x10) {
            addressStr += "0";
        }

        addressStr += String(a, 16);

        if (i > 0) {
            addressStr += ":";
        }
    }

    strcpy(address, addressStr.c_str());

    return address;
}
/*
const bt_addr_t *BLEHelper::address(void) const
{
    return (bt_addr_t *)_address.val;
}
*/

const bt_addr_le_t *BLEHelper::bt_le_address(void) const
{
    return &_address;
}

void
BLEHelper::poll() {
    delay(1);
}

void
BLEHelper::setAddress(bt_addr_le_t address) {
    _address = address;
}

void
BLEHelper::clearAddress() {
    memset(&_address, 0x00, sizeof(_address));
}

const struct bt_le_conn_param *BLEHelper::getConnParams()
{
    return &_conn_params;
}

void BLEHelper::setConnParames(uint16_t intervalmin, 
                               uint16_t intervalmax, 
                               uint16_t latency, 
                               uint16_t timeout)
{
    _conn_params.interval_max = intervalmin;
    _conn_params.interval_min = intervalmax;
    _conn_params.latency = latency;
    _conn_params.timeout = timeout;
    
}


