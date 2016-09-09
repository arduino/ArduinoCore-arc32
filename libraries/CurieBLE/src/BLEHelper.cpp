
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
}

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
BLEHelper::setAddress(const bt_addr_le_t &address) {
    memcpy(&_address, &address, sizeof(bt_addr_le_t));
}

void
BLEHelper::clearAddress() {
    memset(&_address, 0x00, sizeof(_address));
}

void BLEHelper::getConnParams(ble_conn_param_t &user_conn_params)
{
    user_conn_params.interval_min = UNITS_TO_MSEC(_conn_params.interval_min, UNIT_1_25_MS);
    user_conn_params.interval_max = UNITS_TO_MSEC(_conn_params.interval_max, UNIT_1_25_MS);
    user_conn_params.timeout = UNITS_TO_MSEC(_conn_params.timeout, UNIT_10_MS);
    user_conn_params.latency = _conn_params.latency;
}

void BLEHelper::setConnectionParameters(uint16_t intervalmin, 
                                        uint16_t intervalmax, 
                                        uint16_t latency, 
                                        uint16_t timeout)
{
    _conn_params.interval_max = intervalmin;
    _conn_params.interval_min = intervalmax;
    _conn_params.latency = latency;
    _conn_params.timeout = timeout;
}

void BLEHelper::updateConnectionInterval(uint16_t intervalmin, 
                                         uint16_t intervalmax, 
                                         uint16_t latency, 
                                         uint16_t timeout)
{
    setConnectionParameters(intervalmin, intervalmax, latency, timeout);
    updateConnectionInterval();
}

void BLEHelper::updateConnectionInterval()
{
    bt_conn_t* conn = bt_conn_lookup_addr_le(&_address);
    int ret = 0;
    if (NULL != conn)
    {
        ret = bt_conn_le_param_update(conn, &_conn_params);
        pr_debug(LOG_MODULE_BLE, "%s-ret:%d",__FUNCTION__, ret);
        bt_conn_unref(conn);
    }
}

void BLEHelper::setConnectionInterval(float minInterval, 
                                      float maxInterval)
{
    uint16_t minVal = (uint16_t)MSEC_TO_UNITS(minInterval, UNIT_1_25_MS);
    uint16_t maxVal = (uint16_t)MSEC_TO_UNITS(maxInterval, UNIT_1_25_MS);
    _conn_params.interval_min = minVal;
    _conn_params.interval_max = maxVal;
    updateConnectionInterval();
}

void BLEHelper::setConnectionInterval(float minInterval, 
                                      float maxInterval,
                                      uint16_t latency,
                                      uint16_t timeout)
{
    uint16_t minVal = (uint16_t)MSEC_TO_UNITS(minInterval, UNIT_1_25_MS);
    uint16_t maxVal = (uint16_t)MSEC_TO_UNITS(maxInterval, UNIT_1_25_MS);
    uint16_t timeoutVal = MSEC_TO_UNITS(timeout, UNIT_10_MS);
    _conn_params.interval_min = minVal;
    _conn_params.interval_max = maxVal;
     _conn_params.timeout = timeoutVal;
    updateConnectionInterval();
}


