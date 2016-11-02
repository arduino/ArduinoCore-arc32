/*
  BLE Device API
  Copyright (c) 2016 Arduino LLC. All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301 USA
*/
#include "ArduinoBLE.h"
#include "BLEDeviceManager.h"
#include "BLEProfileManager.h"

#include "internal/ble_client.h"

#include <atomic.h>
#include "../src/services/ble/conn_internal.h"

#include "BLEUtils.h"
#include "BLECallbacks.h"

BLEDeviceManager* BLEDeviceManager::_instance;

BLEDeviceManager::BLEDeviceManager():
    _min_conn_interval(0),
    _max_conn_interval(0),
    _adv_critical_local_name(""),
    _has_service_uuid(false),
    _has_service_solicit_uuid(false),
    _appearance(0),
    _manufacturer_data_length(0),
    _adv_type(0),
    _adv_data_idx(0),
    _local_name(""),
    _state(BLE_PERIPH_STATE_NOT_READY),
    _local_ble(NULL)
{
    memset(&_local_bda, 0, sizeof(_local_bda));
    memset(&_wait_for_connect_peripheral, 0, sizeof(_wait_for_connect_peripheral));
    
    memset(&_service_uuid, 0, sizeof(_service_uuid));
    memset(&_service_solicit_uuid, 0, sizeof(_service_solicit_uuid));
    memset(_adv_data, 0, sizeof(_adv_data));
    
    memset(&_peer_central, 0, sizeof (bt_addr_le_t));
    
    ble_client_get_factory_config(&_local_bda, _device_name);
    
    _adv_param.type = BT_LE_ADV_IND;
    _adv_param.addr_type = _local_bda.type;
    _adv_param.interval_min = 0xA0;
    _adv_param.interval_max = 0xF0;
    
    _scan_param.type = BT_HCI_LE_SCAN_ACTIVE;
    _scan_param.filter_dup   = BT_HCI_LE_SCAN_FILTER_DUP_ENABLE;
    _scan_param.interval = BT_GAP_SCAN_FAST_INTERVAL;
    _scan_param.window = BT_GAP_SCAN_FAST_WINDOW;

    memset(_peer_adv_buffer, 0, sizeof(_peer_adv_buffer));
    memset(_peer_adv_mill, 0, sizeof(_peer_adv_mill));
    memset(&_adv_accept_critical, 0, sizeof(_adv_accept_critical));
    memset(&_adv_critical_service_uuid, 0, sizeof(_adv_critical_service_uuid));
    
    memset(_peer_peripheral, 0, sizeof(_peer_peripheral));
    memset(_device_events, 0, sizeof(_device_events));
    memset(_manufacturer_data, 0, sizeof(_manufacturer_data));
    memset(_peer_adv_data, 0, sizeof(_peer_adv_data));
    memset(_peer_adv_data_len, 0, sizeof(_peer_adv_data_len));
}

BLEDeviceManager::~BLEDeviceManager()
{
    
}

bool BLEDeviceManager::begin(BLEDevice *device)
{
    if (NULL == _local_ble && false == *device)
    {
        _local_ble = device;
        _local_ble->setAddress(_local_bda);
        
        // Set device name    
        setDeviceName();
        _state = BLE_PERIPH_STATE_READY;
        delay(4);
        // TODO: Olny allow call one time
        ble_client_init (bleConnectEventHandler, this,
                         bleDisconnectEventHandler, this,
                         bleParamUpdatedEventHandler, this);
        return true;
    }
    else
    {
        return false;
    }
}

void BLEDeviceManager::poll()
{}

void BLEDeviceManager::end()
{}

bool BLEDeviceManager::connected(BLEDevice *device)
{
    bt_conn_t* conn = bt_conn_lookup_addr_le(device->bt_le_address());
    bool retval = false;
    //pr_debug(LOG_MODULE_BLE, "%s-%d: add-%s", __FUNCTION__, __LINE__, device->address().c_str());
    if (NULL != conn)
    {
        //pr_debug(LOG_MODULE_BLE, "%s-%d: state-%d", __FUNCTION__, __LINE__,conn->state);
        if (conn->state == BT_CONN_CONNECTED)
        {
            retval = true;
        }
        bt_conn_unref(conn);
    }
    return retval;
}

bool BLEDeviceManager::disconnect(BLEDevice *device)
{
    int err = 0;
    bt_conn_t* conn = bt_conn_lookup_addr_le(device->bt_le_address());
    if (NULL == conn)
    {
        return false;
    }
    
    err = bt_conn_disconnect (conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
    bt_conn_unref(conn);
    return (err == 0);
}

void BLEDeviceManager::setAdvertisedServiceUuid(const char* advertisedServiceUuid)
{
    _has_service_uuid = true;
    BLEUtils::uuidString2BT(advertisedServiceUuid, (bt_uuid_t *)&_service_uuid);
}

void BLEDeviceManager::setServiceSolicitationUuid(const char* serviceSolicitationUuid)
{
    _has_service_solicit_uuid = true;
    BLEUtils::uuidString2BT(serviceSolicitationUuid, (bt_uuid_t *)&_service_solicit_uuid);
}

void BLEDeviceManager::setManufacturerData(const unsigned char manufacturerData[], 
                                           unsigned char manufacturerDataLength)
{
    if (manufacturerDataLength > BLE_MAX_ADV_SIZE)
    {
        manufacturerDataLength = BLE_MAX_ADV_SIZE;
    }
    _manufacturer_data_length = manufacturerDataLength;
    memcpy(_manufacturer_data, manufacturerData, manufacturerDataLength);
}

void BLEDeviceManager::setLocalName(const char *localName)
{
    _local_name = localName;
}

void BLEDeviceManager::setAdvertisingInterval(float advertisingInterval)
{
    uint16_t interval = (uint16_t) MSEC_TO_UNITS(advertisingInterval, UNIT_0_625_MS);
    
    _adv_param.interval_min = interval;
    _adv_param.interval_max = interval;
}

void BLEDeviceManager::setConnectionInterval(float minimumConnectionInterval, 
                                      float maximumConnectionInterval,
                                      uint16_t latency, 
                                      uint16_t timeout)
{
}

void BLEDeviceManager::setConnectionInterval(float minimumConnectionInterval, 
                                             float maximumConnectionInterval)
{
    
}

bool BLEDeviceManager::setTxPower(int txPower)
{
    ble_gap_set_tx_power(txPower);
    return true;
}

void BLEDeviceManager::setConnectable(bool connectable)
{
    uint8_t type = BT_LE_ADV_IND;
    if (connectable == false)
    {
        type = BT_LE_ADV_NONCONN_IND;
    }
    _adv_param.type = type;
}

void BLEDeviceManager::setDeviceName(const char* deviceName)
{
    memset(_device_name, 0, sizeof(_device_name));
    if (deviceName && deviceName[0])
    {
        int len = strlen(deviceName);
        if (len > BLE_MAX_DEVICE_NAME)
            len = BLE_MAX_DEVICE_NAME;
        memcpy(_device_name, deviceName, len);
        setDeviceName();
    }
}

void
BLEDeviceManager::setDeviceName()
{
    int len = strlen(_device_name);
    bt_le_set_device_name(_device_name, len);
}

void BLEDeviceManager::setAppearance(unsigned short appearance)
{
    _appearance = appearance;
}

BLE_STATUS_T
BLEDeviceManager::_advDataInit(void)
{
    uint8_t lengthTotal = 2; // Flags data length
    _adv_data_idx = 0;
    
    /* Add flags */
    _adv_type = (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR);
    _adv_data[_adv_data_idx].type = BT_DATA_FLAGS;
    _adv_data[_adv_data_idx].data = &_adv_type;
    _adv_data[_adv_data_idx].data_len = 1;
    _adv_data_idx++;
    
    if (_has_service_uuid) 
    {
        uint8_t type;
        uint8_t length;
        uint8_t *data = NULL;
        
        pr_info(LOG_MODULE_BLE, "ADV Type-%d", _service_uuid.uuid.type);
        if (BT_UUID_TYPE_16 == _service_uuid.uuid.type)
        {
            //UINT16_TO_LESTREAM(adv_tmp, uuid.uuid16);
            data = (uint8_t *)&(((bt_uuid_16_t *)&_service_uuid)->val);
            length = UUID_SIZE_16;
            type = BT_DATA_UUID16_ALL;
        }
        else if (BT_UUID_TYPE_128 == _service_uuid.uuid.type)
        {
            data = _service_uuid.val;
            length = UUID_SIZE_128;
            type = BT_DATA_UUID128_ALL;
        }
        if (NULL != data)
        {
            _adv_data[_adv_data_idx].type = type;
            _adv_data[_adv_data_idx].data = data;
            _adv_data[_adv_data_idx].data_len = length;
            _adv_data_idx++;
            lengthTotal += length;
            
            pr_info(LOG_MODULE_BLE, "Service UUID Len -%d", length);
        }
    }
    
    if (_has_service_solicit_uuid) 
    {
        uint8_t type;
        uint8_t length;
        uint8_t *data = NULL;
        
        pr_info(LOG_MODULE_BLE, "ADV Type-%d", _service_solicit_uuid.uuid.type);
        if (BT_UUID_TYPE_16 == _service_solicit_uuid.uuid.type)
        {
            //UINT16_TO_LESTREAM(adv_tmp, uuid.uuid16);
            data = (uint8_t *)&(((bt_uuid_16_t *)&_service_solicit_uuid)->val);
            length = UUID_SIZE_16;
            type = BT_DATA_SOLICIT16;
        }
        else if (BT_UUID_TYPE_128 == _service_solicit_uuid.uuid.type)
        {
            data = _service_solicit_uuid.val;
            length = UUID_SIZE_128;
            type = BT_DATA_SOLICIT128;
        }
        if (NULL != data)
        {
            _adv_data[_adv_data_idx].type = type;
            _adv_data[_adv_data_idx].data = data;
            _adv_data[_adv_data_idx].data_len = length;
            _adv_data_idx++;
            lengthTotal += length;
            
            pr_info(LOG_MODULE_BLE, "Service UUID Len -%d", length);
        }
    }

    if (_local_name.length() > 0)
    {
        /* Add device name (truncated if too long) */
        _adv_data[_adv_data_idx].type = BT_DATA_NAME_COMPLETE;
        _adv_data[_adv_data_idx].data = (const uint8_t*)_local_name.c_str();
        _adv_data[_adv_data_idx].data_len = _local_name.length();
        _adv_data_idx++;
        
        lengthTotal +=  _local_name.length();
        pr_info(LOG_MODULE_BLE, "Local Name -%s", _local_name.c_str());
        pr_info(LOG_MODULE_BLE, "Local Name Len -%d", _local_name.length());
    }
    
    if (_manufacturer_data_length > 0)
    {
        // Add manufacturer data
        _adv_data[_adv_data_idx].type = BT_DATA_MANUFACTURER_DATA;
        _adv_data[_adv_data_idx].data = _manufacturer_data;
        _adv_data[_adv_data_idx].data_len = _manufacturer_data_length;
        _adv_data_idx++;
        
        lengthTotal +=  _manufacturer_data_length;
    }
    
#if 0
    if (_service_data)
    {
        /* Add Service Data (if it will fit) */

        /* A 128-bit Service Data UUID won't fit in an Advertising packet */
        if (BT_UUID_TYPE_16 != _service_data_uuid->type)
        {
            /* We support service data only for 16-bit service UUID */
            return BLE_STATUS_NOT_SUPPORTED;
        }

        uint8_t block_len = sizeof(uint16_t) + _service_data_length;
        if (1 + block_len > BLE_MAX_ADV_SIZE)
        {
            // Service data block is too large.
            return BLE_STATUS_ERROR_PARAMETER;
        }
        
        _adv_data[_adv_data_idx].type = BT_DATA_SVC_DATA16;
        _adv_data[_adv_data_idx].data = _service_data_buf;
        _adv_data[_adv_data_idx].data_len = block_len;
        _adv_data_idx++;

        uint8_t *adv_tmp = _service_data_buf;

        UINT16_TO_LESTREAM(adv_tmp, (((bt_uuid_16_t *)_service_data_uuid)->val));
        memcpy(adv_tmp, _service_data, _service_data_length);
        
        lengthTotal += block_len;
        pr_info(LOG_MODULE_BLE, "SVC Len -%d", block_len);
    }
#endif

    if (lengthTotal > BLE_MAX_ADV_SIZE)
    {
        pr_error(LOG_MODULE_BLE, "ADV Total length-%d", lengthTotal);
        // Service data block is too large.
        return BLE_STATUS_ERROR_PARAMETER;
    }
    return BLE_STATUS_SUCCESS;
}

BLE_STATUS_T BLEDeviceManager::startAdvertising()
{
    int ret;
    BLE_STATUS_T status;
    status = _advDataInit();
    if (BLE_STATUS_SUCCESS != status)
    {
        return status;
    }

    pr_info(LOG_MODULE_BLE, "%s-ad_len%d", __FUNCTION__, _adv_data_idx);
    if (_state != BLE_PERIPH_STATE_READY)
        return BLE_STATUS_WRONG_STATE;
    
    ret = bt_le_adv_start(&_adv_param, _adv_data, _adv_data_idx, NULL, 0);
    if (0 != ret)
    {
        pr_error(LOG_MODULE_APP, "[ADV] Start failed. Error: %d", ret);
        return BLE_STATUS_WRONG_STATE;
    }
    delay(10);
    _state = BLE_PERIPH_STATE_ADVERTISING;
    return BLE_STATUS_SUCCESS;
}

BLE_STATUS_T BLEDeviceManager::stopAdvertising()
{
    int err_code = 0;
    BLE_STATUS_T status = BLE_STATUS_WRONG_STATE;

    if (BLE_PERIPH_STATE_ADVERTISING == _state)
    {
        err_code = bt_le_adv_stop();
        status = errorno_to_ble_status(err_code);
    }

    if (BLE_STATUS_SUCCESS != status)
        return status;

    _state = BLE_PERIPH_STATE_READY;
    return BLE_STATUS_SUCCESS;
}

BLEDevice BLEDeviceManager::central()
{   
    BLEDevice temp(&_peer_central);
    return temp;
}

BLEDevice BLEDeviceManager::peripheral()
{
    // TODO
    BLEDevice temp;
    return temp;
}

bool BLEDeviceManager::startScanning()
{
    int err = bt_le_scan_start(&_scan_param, ble_central_device_found);
    if (err)
    {
        pr_info(LOG_MODULE_BLE, "Scanning failed to start (err %d)\n", err);
        return false;
    }
    return true;
}

bool BLEDeviceManager::startScanningWithDuplicates()
{
    // TODO: enable disable duplicate
    return false;
}

bool BLEDeviceManager::stopScanning()
{
    int err = bt_le_scan_stop();
    if (0 != err)
    {
        pr_info(LOG_MODULE_BLE, "Stop LE scan failed (err %d)\n", err);
        return false;
    }
    return true;
}

void BLEDeviceManager::setAdvertiseCritical(String name)
{
    _adv_critical_local_name = name;
    _adv_accept_critical.type = BT_DATA_NAME_COMPLETE;
    _adv_accept_critical.data_len = name.length();
    _adv_accept_critical.data = (const uint8_t*)_adv_critical_local_name.c_str();
}

void BLEDeviceManager::setAdvertiseCritical(BLEService& service)
{
    BLEUtils::uuidString2BT(service.uuid(),(bt_uuid_t *)&_adv_critical_service_uuid);
    uint8_t type = 0;
    uint8_t length = 0;
    uint8_t *data = NULL;
    
    pr_info(LOG_MODULE_BLE, "ADV Type-%d", _service_uuid.uuid.type);
    if (BT_UUID_TYPE_16 == _service_uuid.uuid.type)
    {
        //UINT16_TO_LESTREAM(adv_tmp, uuid.uuid16);
        data = (uint8_t *)&(((bt_uuid_16_t *)&_service_uuid)->val);
        length = UUID_SIZE_16;
        type = BT_DATA_UUID16_ALL;
    }
    else if (BT_UUID_TYPE_128 == _service_uuid.uuid.type)
    {
        data = _service_uuid.val;
        length = UUID_SIZE_128;
        type = BT_DATA_UUID128_ALL;
    }
    _adv_accept_critical.type = type;
    _adv_accept_critical.data_len = length;
    _adv_accept_critical.data = data;
}

bool BLEDeviceManager::hasLocalName() const
{
    return (_local_name.length() != 0);
}

bool BLEDeviceManager::hasAdvertisedServiceUuid() const
{
    // TODO: 
    return false;
}

bool BLEDeviceManager::hasAdvertisedServiceUuid(int index) const
{
    // TODO: 
    return false;
}

int BLEDeviceManager::advertisedServiceUuidCount() const
{
    return 0;
}

String BLEDeviceManager::localName() const
{
    return _local_name;
}

String BLEDeviceManager::advertisedServiceUuid() const
{
    // TODO
    return "";
}

String BLEDeviceManager::advertisedServiceUuid(int index) const
{
    // TODO
    return "";
}

int BLEDeviceManager::rssi() const
{
    return 0;
}

bool BLEDeviceManager::connect(BLEDevice &device)
{
    // 
    uint64_t timestamp = millis();
    uint64_t timestampcur = timestamp;
    bool ret = true;
    bt_addr_le_copy(&_wait_for_connect_peripheral, device.bt_le_address());
    startScanning();
    
    pr_debug(LOG_MODULE_BLE, "%s-%d", __FUNCTION__, __LINE__);
    // Wait for the connection
    while (ret && (true == BLEUtils::macAddressValid(_wait_for_connect_peripheral)))
    {
        timestampcur = millis();
        // TODO: dismiss the magic number
        ret = (timestampcur - timestamp < 3000); // Time out
    }
    
    pr_debug(LOG_MODULE_BLE, "%s-%d", __FUNCTION__, __LINE__);
        
    if (ret == false)
    {
        memset(&_wait_for_connect_peripheral, 0, sizeof(_wait_for_connect_peripheral));
    }
    return ret;
}

bool BLEDeviceManager::connectToDevice(BLEDevice &device)
{
    bt_addr_le_t* temp   = NULL;
    bt_addr_le_t* unused = NULL;
    bool link_existed = false;
    bool retval = false;
    
    pr_debug(LOG_MODULE_BLE, "%s-%d-1", __FUNCTION__, __LINE__);
    
    // Find free peripheral Items
    for (int i = 0; i < BLE_MAX_CONN_CFG; i++)
    {
        temp = &_peer_peripheral[i];
        if (true == BLEUtils::macAddressValid(*temp))
        {
            if (bt_addr_le_cmp(temp, device.bt_le_address()) == 0)
            {
                // Connect request has scheduled but connection don't established.
                //  The central can see the ADV and no need to send connect request.
                link_existed = true;
                break;
            }
        }
        else
        {
            if (NULL == unused)
            {
                unused = temp;
            }
        }
    }
    pr_debug(LOG_MODULE_BLE, "%s-%d:link_existed-%d unused-%p", __FUNCTION__, __LINE__, link_existed, unused);
    
    if (!link_existed && NULL != unused)
    {
    pr_debug(LOG_MODULE_BLE, "%s-%d-Device:%s", __FUNCTION__, __LINE__, device.address().c_str());
        // Send connect request
        bt_conn_t* conn = bt_conn_create_le(device.bt_le_address(), device.bt_conn_param());
        if (NULL != conn)
        {
            memcpy(unused, device.bt_le_address(), sizeof(bt_addr_le_t));
            retval = true;
            bt_conn_unref(conn);
        }
    }
    return retval;
}

String BLEDeviceManager::deviceName()
{
    return _device_name;
}

int BLEDeviceManager::appearance()
{
    return _appearance;
}

BLEDeviceManager* BLEDeviceManager::instance()
{
    if (_instance == NULL)
    {
        _instance = new BLEDeviceManager();
    }
    return _instance;
}

void BLEDeviceManager::setEventHandler(BLEDeviceEvent event, 
                                       BLEDeviceEventHandler eventHandler)
{
    if (event < BLEDeviceLastEvent)
        _device_events[event] = eventHandler;
}

void BLEDeviceManager::handleConnectEvent(bt_conn_t *conn, uint8_t err)
{
    struct bt_conn_info role_info;
    bt_conn_get_info(conn, &role_info);
    pr_info(LOG_MODULE_BLE, "%s-%d: role-%d", __FUNCTION__, __LINE__, role_info.role);
    if (BT_CONN_ROLE_SLAVE == role_info.role)
    {
        // Central has established the connection with this peripheral device
        memcpy(&_peer_central, bt_conn_get_dst(conn), sizeof (bt_addr_le_t));
    }
    else
    {
        memset(&_wait_for_connect_peripheral, 0, sizeof(_wait_for_connect_peripheral));
        // Peripheral has established the connection with this Central device
        BLEProfileManager::instance()->handleConnectedEvent(bt_conn_get_dst(conn));
    }
    
    if (NULL != _device_events[BLEConnected])
    {
        BLEDevice tempdev(bt_conn_get_dst(conn));
        _device_events[BLEConnected](tempdev);
    }
}

void BLEDeviceManager::handleDisconnectEvent(bt_conn_t *conn, uint8_t reason)
{
    struct bt_conn_info role_info;
    bt_conn_get_info(conn, &role_info);
    pr_info(LOG_MODULE_BLE, "%s-%d: role-%d", __FUNCTION__, __LINE__, role_info.role);
    if (BT_CONN_ROLE_SLAVE == role_info.role)
    {
        // Central has established the connection with this peripheral device
        memset(&_peer_central, 0, sizeof (bt_addr_le_t));
    }
    else
    {
        bt_addr_le_t* temp   = NULL;
        const bt_addr_le_t* disConnAddr = bt_conn_get_dst(conn);
        for (int i = 0; i < BLE_MAX_CONN_CFG; i++)
        {
            temp = &_peer_peripheral[i];
            if (bt_addr_le_cmp(temp, disConnAddr) == 0)
            {
                memset(temp, 0, sizeof(bt_addr_le_t));
                break;
            }
        }
        // Peripheral has established the connection with this Central device
        BLEProfileManager::instance()->handleDisconnectedEvent(bt_conn_get_dst(conn));
    }
    
    if (NULL != _device_events[BLEDisconnected])
    {
        BLEDevice tempdev(bt_conn_get_dst(conn));
        _device_events[BLEDisconnected](tempdev);
    }
}

void BLEDeviceManager::handleParamUpdated (bt_conn_t *conn, 
                                           uint16_t interval,
                                           uint16_t latency, 
                                           uint16_t timeout)
{
    if (NULL != _device_events[BLEConParamUpdate])
    {
        BLEDevice tempdev(bt_conn_get_dst(conn));
        _device_events[BLEConParamUpdate](tempdev);
    }
}

bool BLEDeviceManager::advertiseDataProc(uint8_t type, 
                                         const uint8_t *dataPtr, 
                                         uint8_t data_len)
{
    //Serial1.print("[AD]:");
    //Serial1.print(type);
    //Serial1.print(" data_len ");
    //Serial1.println(data_len);
    
    //const bt_data_t zero = {0, 0,0};
    if (_adv_accept_critical.type == 0 && 
        _adv_accept_critical.data_len == 0 &&
        _adv_accept_critical.data == NULL)
    {
        // Not set the critical. Accept all.
        return true;
    }
    if (type == _adv_accept_critical.type &&
        data_len == _adv_accept_critical.data_len &&
        0 == memcmp(dataPtr, _adv_accept_critical.data, data_len))
    {
        // Now Only support 1 critical. Change those code if want support multi-criticals
        return true;
    }

    return false;
}

BLEDevice BLEDeviceManager::available()
{
    BLEDevice tempdevice;
    bt_addr_le_t* temp = NULL;
    uint64_t timestamp = millis();
    uint8_t index = BLE_MAX_ADV_BUFFER_CFG;
    uint8_t i = 0;
    uint64_t max_delta = 0;
    
    for (i = 0; i < BLE_MAX_ADV_BUFFER_CFG; i++)
    {
        uint64_t timestamp_delta = timestamp - _peer_adv_mill[i];
        temp = &_peer_adv_buffer[i];
        if ((timestamp_delta <= 2000) && (max_delta < timestamp_delta))
        {
            max_delta = timestamp_delta;
            index = i;
        }
    }
    //pr_debug(LOG_MODULE_BLE, "%s-%d:index %d, i-%d", __FUNCTION__, __LINE__, index, i);
    
    if (index < BLE_MAX_ADV_BUFFER_CFG)
    {
        temp = &_peer_adv_buffer[index];
        if (true == BLEUtils::macAddressValid(*temp))
        {
            tempdevice.setAddress(*temp);
            pr_debug(LOG_MODULE_BLE, "%s-%d:Con addr-%s", __FUNCTION__, __LINE__, BLEUtils::macAddressBT2String(*temp).c_str());
            _peer_adv_mill[index] -= 2000; // Set it as expired
        }
    }
    return tempdevice;
}

bool BLEDeviceManager::setAdvertiseBuffer(const bt_addr_le_t* bt_addr,
                                          const uint8_t *ad, 
                                          uint8_t data_len)
{
    bt_addr_le_t* temp = NULL;
    uint64_t timestamp = millis();
    uint8_t index = BLE_MAX_ADV_BUFFER_CFG;
    uint8_t i = 0;
    uint64_t max_delta = 0;
    bool retval = false;
    //pr_debug(LOG_MODULE_BLE, "%s-%d-1", __FUNCTION__, __LINE__);
    for (i = 0; i < BLE_MAX_ADV_BUFFER_CFG; i++)
    {
        uint64_t timestamp_delta = timestamp - _peer_adv_mill[i];
        temp = &_peer_adv_buffer[i];
        if (max_delta < timestamp_delta)
        {
            max_delta = timestamp_delta;
            if (max_delta > 2000) // expired
                index = i;
        }
        
        if (bt_addr_le_cmp(temp, bt_addr) == 0)
        //if (memcpy(temp->val, bt_addr->val, 6) == 0)
        {
            // The device alread in the buffer
            index = i;
            break;
        }
    }
    //pr_debug(LOG_MODULE_BLE, "%s-%d:index %d, i-%d", __FUNCTION__, __LINE__, index, i);
    
    //pr_debug(LOG_MODULE_BLE, "%s-%d-2", __FUNCTION__, __LINE__);
    if (index < BLE_MAX_ADV_BUFFER_CFG)
    {
        temp = &_peer_adv_buffer[index];
        if (i >= BLE_MAX_ADV_BUFFER_CFG)
        {
            memcpy(temp, bt_addr, sizeof (bt_addr_le_t));
        }
        if (data_len > BLE_MAX_ADV_SIZE)
        {
            data_len = BLE_MAX_ADV_SIZE;
        }
        memcpy(_peer_adv_data[index], ad, data_len);
        _peer_adv_data_len[index] = data_len;
        // Update the timestamp
        _peer_adv_mill[index] = timestamp;
        retval = true;
    }
    
    return retval;
}
    
void BLEDeviceManager::handleDeviceFound(const bt_addr_le_t *addr, 
                                         int8_t rssi, 
                                         uint8_t type,
                                         const uint8_t *ad, 
                                         uint8_t data_len)
{
    const uint8_t *data = ad;
    
    /* We're only interested in connectable events */
    if (type == BT_LE_ADV_IND || type == BT_LE_ADV_DIRECT_IND)
    {
        //pr_debug(LOG_MODULE_BLE, "%s-%d", __FUNCTION__, __LINE__);
        while (data_len > 1)
        {
            uint8_t len = data[0];

            /* Check for early termination */
            if (len == 0)
            {
                return;
            }

            if ((len + 1 > data_len) || (data_len < 2)) {
                pr_info(LOG_MODULE_BLE, "AD malformed\n");
                return;
            }

            if (true == advertiseDataProc(data[1], &data[2], len - 1))
            {
                if (true == BLEUtils::macAddressValid(_wait_for_connect_peripheral))
                {
                    // Not add to the buffer when try to establish the connection
                    if (true == BLEUtils::macAddressSame(*addr, _wait_for_connect_peripheral))
                    {
                        BLEDevice testdev(addr);
                        stopScanning();
                        connectToDevice(testdev);
                    }
                }
                else
                {
                    // The critical is accepted
                    //  Find the oldest and expired buffer 
                    if(false == setAdvertiseBuffer(addr, ad, data_len))
                    {
                        pr_info(LOG_MODULE_BLE, "No buffer to store the ADV\n");
                    }
                }
                pr_debug(LOG_MODULE_BLE, "%s-%d: Done", __FUNCTION__, __LINE__);
                return;
            }

            data_len -= len + 1;
            data += len + 1;
        }
        //pr_debug(LOG_MODULE_BLE, "%s: done", __FUNCTION__);
    }
    
}


    