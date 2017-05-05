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
#include "CurieBLE.h"
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
    _peer_temp_dev_index(0),
    _adv_critical_local_name(""),
    _wait_for_connect_peripheral_adv_data_len(0),
    _wait_for_connect_peripheral_scan_rsp_data_len(0),
    _wait_for_connect_peripheral_adv_rssi(0),
    _available_for_connect_peripheral_adv_data_len(0),
    _available_for_connect_peripheral_scan_rsp_data_len(0),
    _available_for_connect_peripheral_adv_rssi(0),
    _available_for_connect_peripheral_connectable(false),
    _connecting(false),
    _has_service_uuid(false),
    _has_service_solicit_uuid(false),
    _manufacturer_data_length(0),
    _service_data_length(0),
    _adv_type(0),
    _adv_data_idx(0),
    _scan_rsp_data_idx(0),
    _local_name(""),
    _state(BLE_PERIPH_STATE_NOT_READY),
    _local_ble(NULL),
    _peer_peripheral_index(0),
    _duplicate_filter_header(0),
    _duplicate_filter_tail(0),
    _adv_duplicate_filter_enabled(false)
{
    memset(&_local_bda, 0, sizeof(_local_bda));
    
    
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
    memset(_peer_adv_data, 0, sizeof(_peer_adv_data));
    memset(_peer_adv_data_len, 0, sizeof(_peer_adv_data_len));
    memset(_peer_scan_rsp_data, 0, sizeof(_peer_scan_rsp_data));
    memset(_peer_scan_rsp_data_len, -1, sizeof(_peer_scan_rsp_data_len));
    memset(_peer_adv_rssi, 0, sizeof(_peer_adv_rssi));
    
    memset(_peer_adv_connectable, 0, sizeof(_peer_adv_connectable));
    
    memset(_peer_temp_adv_buffer, 0, sizeof(_peer_temp_adv_buffer));
    memset(_peer_temp_adv_data, 0, sizeof(_peer_temp_adv_data));
    memset(_peer_temp_adv_data_len, 0, sizeof(_peer_temp_adv_data_len));
    memset(_peer_temp_adv_connectable, 0, sizeof(_peer_adv_connectable));
    
    memset(&_adv_accept_critical, 0, sizeof(_adv_accept_critical));
    memset(&_adv_critical_service_uuid, 0, sizeof(_adv_critical_service_uuid));
    memset(&_adv_accept_device, 0, sizeof(_adv_accept_device));
    
    memset(&_wait_for_connect_peripheral, 0, sizeof(_wait_for_connect_peripheral));
    memset(&_wait_for_connect_peripheral_adv_data, 0, sizeof(_wait_for_connect_peripheral_adv_data));
    memset(&_wait_for_connect_peripheral_scan_rsp_data, 0, sizeof(_wait_for_connect_peripheral_scan_rsp_data));
    
    memset(&_available_for_connect_peripheral_adv_data, 0, sizeof(_available_for_connect_peripheral_adv_data));
    memset(&_available_for_connect_peripheral_scan_rsp_data, 0, sizeof(_available_for_connect_peripheral_scan_rsp_data));
    
    memset(&_service_uuid, 0, sizeof(_service_uuid));
    memset(&_service_solicit_uuid, 0, sizeof(_service_solicit_uuid));
    memset(_manufacturer_data, 0, sizeof(_manufacturer_data));
    
    memset(&_service_data_uuid, 0, sizeof(_service_data_uuid));
    memset(_service_data, 0, sizeof(_service_data));
    memset(_service_data_buf, 0, sizeof(_service_data_buf));
    
    memset(_adv_data, 0, sizeof(_adv_data));
    memset(_scan_rsp_data, 0, sizeof(_scan_rsp_data));
    
    memset(_peer_peripheral, 0, sizeof(_peer_peripheral));
    memset(_peer_peripheral_adv_data, 0, sizeof(_peer_peripheral_adv_data));
    memset(_peer_peripheral_adv_data_len, 0, sizeof(_peer_peripheral_adv_data_len));
    memset(_peer_peripheral_scan_rsp_data, 0, sizeof(_peer_peripheral_scan_rsp_data));
    memset(_peer_peripheral_scan_rsp_data_len, 0, sizeof(_peer_peripheral_scan_rsp_data_len));
    memset(_peer_peripheral_adv_rssi, 0, sizeof(_peer_peripheral_adv_rssi));
    
    memset(_device_events, 0, sizeof(_device_events));
}

BLEDeviceManager::~BLEDeviceManager()
{
    
}

bool BLEDeviceManager::begin(BLEDevice *device)
{
    if (NULL == _local_ble)
    {
        _local_ble = device;
        bt_le_set_mac_address(_local_bda);
        
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
{
    if (NULL != _device_events[BLEDiscovered])
    {
        BLEDevice tempdev = available();
        
        while (tempdev)
        {
            _device_events[BLEDiscovered](tempdev);
            tempdev = available();
        }
    }
}

void BLEDeviceManager::end()
{
    stopScanning();
    stopAdvertising();
    // Disconnect the connections
    disconnect(&BLE);
}

bool BLEDeviceManager::connected(const BLEDevice *device) const
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

bool BLEDeviceManager::disconnectSingle(const bt_addr_le_t *peer)
{
    int err = 0;
    
    bt_conn_t* conn = bt_conn_lookup_addr_le(peer);
    if (NULL == conn)
    {
        return false;
    }
    
    err = bt_conn_disconnect (conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
    while (err == 0 && conn->state != BT_CONN_DISCONNECTED)
    {
        delay(10);
    }
    bt_conn_unref(conn);
    return true;
}


bool BLEDeviceManager::disconnect(BLEDevice *device)
{
    bool ret = true;
    if (false == BLEUtils::isLocalBLE(*device))
    {
        // Remote device disconnect one
        ret = disconnectSingle(device->bt_le_address());
    }
    else
    {
        // Local device disconnect all connections
        if (true == BLEUtils::macAddressValid(_peer_central))
        {
            ret = disconnectSingle(&_peer_central);
        }
        
        for (int i = 0; i < BLE_MAX_CONN_CFG; i++)
        {
            if (true == BLEUtils::macAddressValid(_peer_peripheral[i]))
            {
                ret = disconnectSingle(&_peer_central);
            }
        }
    }
    return ret;
}

void BLEDeviceManager::setAdvertisedServiceUuid(const char* advertisedServiceUuid)
{
    _has_service_uuid = true;
    BLEUtils::uuidString2BT(advertisedServiceUuid, (bt_uuid_t *)&_service_uuid);
}

void BLEDeviceManager::setAdvertisedServiceData(const bt_uuid_t* serviceDataUuid,
                                                const uint8_t* serviceData,
                                                uint8_t serviceDataLength)
{
    memcpy(&_service_data_uuid, serviceDataUuid, sizeof(_service_data_uuid));
    if (serviceDataLength > BLE_MAX_ADV_SIZE)
    {
        serviceDataLength = BLE_MAX_ADV_SIZE;
    }
    
    memcpy(_service_data, serviceData, serviceDataLength);
    _service_data_length = serviceDataLength;
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

void BLEDeviceManager::getConnectionInterval(BLEDevice *device, 
                                             bt_le_conn_param* conn_param)
{
    bt_conn_t* conn = bt_conn_lookup_addr_le(device->bt_le_address());
    if (NULL != conn)
    {
        conn_param->interval_max = conn->le.interval;
        conn_param->interval_min = conn->le.interval;
        conn_param->latency = conn->le.latency;
        conn_param->timeout = conn->le.timeout;
        bt_conn_unref(conn);
    }
}

int BLEDeviceManager::setConnectionInterval(BLEDevice *device)
{
    bt_conn_t* conn = bt_conn_lookup_addr_le(device->bt_le_address());
    int ret = 0;
    if (NULL != conn)
    {
        ret = bt_conn_le_param_update(conn, device->bt_conn_param());
        pr_debug(LOG_MODULE_BLE, "%s-ret:%d",__FUNCTION__, ret);
        bt_conn_unref(conn);
    }
    return ret;
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
        if (NULL != _local_ble)
        {
            setDeviceName();
        }
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
    BLEProfileManager::instance()->setAppearance(appearance);
}

BLE_STATUS_T
BLEDeviceManager::setAdvertiseData(uint8_t type, const uint8_t* data, uint8_t length)
{
    uint8_t lengthOfAdv = 0; // Flags data length
    uint8_t lengthOfScanRsp = 0; // Flags data length
    bt_data_t *fill_area = NULL;
    
    // Get the length of the Advertisement
    for (uint8_t i = 0; i < _adv_data_idx; i++)
    {
        lengthOfAdv += _adv_data[i].data_len + 2;
    }
    
    for (uint8_t i = 0; i < _scan_rsp_data_idx; i++)
    {
        lengthOfAdv += _scan_rsp_data[i].data_len + 2;
    }
        
        
    if (((length + lengthOfAdv) < BLE_MAX_ADV_SIZE) && 
        (_adv_data_idx < ARRAY_SIZE(_adv_data)))
    {
        fill_area = &_adv_data[_adv_data_idx];
        _adv_data_idx++;
    }
    else if ((length + lengthOfScanRsp) < BLE_MAX_ADV_SIZE && 
             (_scan_rsp_data_idx < ARRAY_SIZE(_scan_rsp_data)))
    {
        fill_area = &_scan_rsp_data[_scan_rsp_data_idx];
        _scan_rsp_data_idx++;
    }
    else
    {
        // Service data block is too large.
        return BLE_STATUS_ERROR_PARAMETER;
    }

    if (fill_area)
    {
        fill_area->type = type;
        fill_area->data = data;
        fill_area->data_len = length;
        
        pr_info(LOG_MODULE_BLE, "ADV type %d Len - %d",type, length);
    }
    return BLE_STATUS_SUCCESS;
}

BLE_STATUS_T
BLEDeviceManager::_advDataInit(void)
{
    BLE_STATUS_T ret = BLE_STATUS_SUCCESS;
    // Clear the indexs
    _adv_data_idx = 0;
    _scan_rsp_data_idx = 0;
    
    /* Add flags */
    _adv_type = (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR);
    ret = setAdvertiseData (BT_DATA_FLAGS, &_adv_type, sizeof(_adv_type));
    
    if (_has_service_solicit_uuid && 
        (BLE_STATUS_SUCCESS == ret)) 
    {
        uint8_t type;
        uint8_t length;
        uint8_t *data = NULL;
        
        pr_info(LOG_MODULE_BLE, "ADV Type-%d", _service_solicit_uuid.uuid.type);
        if (BT_UUID_TYPE_16 == _service_solicit_uuid.uuid.type)
        {
            data = (uint8_t *)&(((bt_uuid_16_t *)&_service_solicit_uuid)->val);
            length = UUID_SIZE_16;
            type = BT_DATA_SOLICIT16;
        }
        else  // Sid. KW, default is BT_UUID_TYPE_128
        {
            data = _service_solicit_uuid.val;
            length = UUID_SIZE_128;
            type = BT_DATA_SOLICIT128;
        }
        
        ret = setAdvertiseData(type, data, length);
    }
    
    if (_has_service_uuid && 
        (BLE_STATUS_SUCCESS == ret)) 
    {
        uint8_t type;
        uint8_t length;
        uint8_t *data = NULL;
        
        pr_info(LOG_MODULE_BLE, "ADV Type-%d", _service_uuid.uuid.type);
        if (BT_UUID_TYPE_16 == _service_uuid.uuid.type)
        {
            data = (uint8_t *)&(((bt_uuid_16_t *)&_service_uuid)->val);
            length = UUID_SIZE_16;
            type = BT_DATA_UUID16_ALL;
        }
        else //  Sid. KW, default is BT_UUID_TYPE_128
        {
            data = _service_uuid.val;
            length = UUID_SIZE_128;
            type = BT_DATA_UUID128_ALL;
        }
        ret = setAdvertiseData(type, data, length);
    }
    
    if (_manufacturer_data_length > 0  && 
        (BLE_STATUS_SUCCESS == ret))
    {
        ret = setAdvertiseData (BT_DATA_MANUFACTURER_DATA, 
                                _manufacturer_data, 
                                _manufacturer_data_length);
    }

    if (_local_name.length() > 0  && 
        (BLE_STATUS_SUCCESS == ret))
    {
        uint8_t length = _local_name.length();
        ret = setAdvertiseData (BT_DATA_NAME_COMPLETE, 
                                (const uint8_t*)_local_name.c_str(), 
                                length);
    }
    
    if (_service_data_length > 0  && 
        (BLE_STATUS_SUCCESS == ret))
    {
        /* Add Service Data (if it will fit) */

        /* A 128-bit Service Data UUID won't fit in an Advertising packet */
        if (BT_UUID_TYPE_16 != _service_data_uuid.uuid.type)
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
        
        ret = setAdvertiseData (BT_DATA_SVC_DATA16, 
                                _service_data_buf, 
                                block_len);

        uint8_t *adv_tmp = _service_data_buf;

        memcpy(adv_tmp, &((bt_uuid_16_t*)&_service_data_uuid)->val, sizeof(uint16_t));
        adv_tmp += 2;
        memcpy(adv_tmp, _service_data, _service_data_length);
    }
    
    return ret;
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
    
    ret = bt_le_adv_start(&_adv_param, 
                          _adv_data, _adv_data_idx, 
                          _scan_rsp_data, _scan_rsp_data_idx);
    if (0 != ret)
    {
        pr_error(LOG_MODULE_APP, "[ADV] Start failed. Error: %d", ret);
        return BLE_STATUS_WRONG_STATE;
    }
    delay(10);
    _state = BLE_PERIPH_STATE_ADVERTISING;
    return BLE_STATUS_SUCCESS;
}

bool BLEDeviceManager::advertising()
{
    return (BLE_PERIPH_STATE_ADVERTISING == _state);
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
    BLEDevice temp;
    for (int i = 0; i < BLE_MAX_CONN_CFG; i++)
    {
        if (_peer_peripheral_index >= BLE_MAX_CONN_CFG)
        {
            _peer_peripheral_index = 0;
        }
        const bt_addr_le_t & addr = _peer_peripheral[_peer_peripheral_index];
        _peer_peripheral_index++;
        
        if (true == BLEUtils::macAddressValid(addr))
        {
            temp.setAddress(addr);
            break;
        }
    }
    return temp;
}

void BLEDeviceManager::_clearAdvertiseBuffer()
{
    
    // Clear the previous found ADV
    memset(_peer_temp_adv_buffer, 0, sizeof(_peer_temp_adv_buffer));
    memset(_peer_temp_adv_data, 0, sizeof(_peer_temp_adv_data));
    memset(_peer_temp_adv_data_len, 0, sizeof(_peer_temp_adv_data_len));
    memset(_peer_temp_adv_connectable, 0, sizeof(_peer_adv_connectable));
    
    memset(_peer_adv_buffer, 0, sizeof(_peer_adv_buffer));
    memset(_peer_adv_mill, 0, sizeof(_peer_adv_mill));
    memset(_peer_adv_data, 0, sizeof(_peer_adv_data));
    memset(_peer_adv_data_len, 0, sizeof(_peer_adv_data_len));
    memset(_peer_scan_rsp_data, 0, sizeof(_peer_scan_rsp_data));
    memset(_peer_scan_rsp_data_len, 0, sizeof(_peer_scan_rsp_data_len));
    memset(_peer_adv_rssi, 0, sizeof(_peer_adv_rssi));
    
}

bool BLEDeviceManager::startScanningWithDuplicates()
{
    _adv_duplicate_filter_enabled = false;
    _scan_param.filter_dup   = BT_HCI_LE_SCAN_FILTER_DUP_ENABLE;

    _clearAdvertiseBuffer();
    
    int err = bt_le_scan_start(&_scan_param, ble_central_device_found);
    if (err)
    {
        pr_info(LOG_MODULE_BLE, "Scanning failed to start (err %d)\n", err);
        return false;
    }
    return true;
}

bool BLEDeviceManager::startScanningNewPeripherals()
{
    _adv_duplicate_filter_enabled = true;
    memset(_peer_duplicate_address_buffer, 0, sizeof(_peer_duplicate_address_buffer));
    _duplicate_filter_header = _duplicate_filter_tail = 0;

    _clearAdvertiseBuffer();
    
    _scan_param.filter_dup   = BT_HCI_LE_SCAN_FILTER_DUP_ENABLE;
    int err = bt_le_scan_start(&_scan_param, ble_central_device_found);
    if (err)
    {
        pr_info(LOG_MODULE_BLE, "Scanning failed to start (err %d)\n", err);
        return false;
    }
    return true;
}

bool BLEDeviceManager::stopScanning()
{
    int err = bt_le_scan_stop();

    if (err)  // Sid. TODO: KW detected bt_le_scan_stop return only 0.
    {
        pr_info(LOG_MODULE_BLE, "Stop LE scan failed (err %d)\n", err);
        return false;
    }
    return true;
}

void BLEDeviceManager::clearAdvertiseCritical()
{
    memset(&_adv_accept_critical, 0, sizeof(_adv_accept_critical));
    memset(&_adv_accept_device, 0, sizeof(_adv_accept_device));
    //memset(&_adv_critical_service_uuid, 0, sizeof(_adv_critical_service_uuid));
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
    
    pr_info(LOG_MODULE_BLE, "ADV Type-%d", _adv_critical_service_uuid.uuid.type);
    if (BT_UUID_TYPE_16 == _adv_critical_service_uuid.uuid.type)
    {
        data = (uint8_t *)&(((bt_uuid_16_t *)&_adv_critical_service_uuid)->val);
        length = UUID_SIZE_16;
        type = BT_DATA_UUID16_ALL;
    }
    else  // Sid. KW, default is BT_UUID_TYPE_128
    {
        data = _adv_critical_service_uuid.val;
        length = UUID_SIZE_128;
        type = BT_DATA_UUID128_ALL;
    }
    _adv_accept_critical.type = type;
    _adv_accept_critical.data_len = length;
    _adv_accept_critical.data = data;
}

void BLEDeviceManager::setAdvertiseCritical(const char* macaddress)
{
    BLEUtils::macAddressString2BT(macaddress, _adv_accept_device);
}

bool BLEDeviceManager::getDataFromAdvertiseByType(const BLEDevice* device,
                                                  const uint8_t eir_type, 
                                                  const uint8_t* &data,
                                                  uint8_t &data_len) const
{
    const uint8_t* adv_data = NULL;
    uint8_t adv_data_len = 0;
    bool retval = false;
    bool scan_response_proced = false;
    
    getDeviceAdvertiseBuffer(device->bt_le_address(),
                             adv_data,
                             adv_data_len);

    while (NULL != adv_data)
    {
        while (adv_data_len > 1)
        {
            uint8_t len = adv_data[0];
            uint8_t type = adv_data[1];

            /* Check for early termination */
            if ((len == 0) || ((len + 1) > adv_data_len)) {
                break;
            }

            if (type == eir_type)
            {
                if (len >= BLE_MAX_ADV_SIZE)
                {
                    len = BLE_MAX_ADV_SIZE-1;
                }
                data = &adv_data[2];
                data_len = len - 1;
                retval = true;
                break;
            }

            adv_data_len -= len + 1;
            adv_data += len + 1;
        }
        if (retval == true || scan_response_proced == true)
        {
            break;
        }
        getDeviceScanResponseBuffer(device->bt_le_address(),
                                    adv_data,
                                    adv_data_len);
        scan_response_proced = true;
    }
    return retval;
}


bool BLEDeviceManager::hasLocalName(const BLEDevice* device) const
{
    if (BLEUtils::isLocalBLE(*device) == true)
    {
        return (_local_name.length() != 0);
    }
    
    const uint8_t* local_name = NULL;
    uint8_t local_name_len = 0;
    bool retval = getDataFromAdvertiseByType(device, 
                                             BT_DATA_NAME_COMPLETE,
                                             local_name, 
                                             local_name_len);
    if (false == retval)
    {
        retval = getDataFromAdvertiseByType(device, 
                                            BT_DATA_NAME_SHORTENED,
                                            local_name, 
                                            local_name_len);
    }
    return retval;
}

bool BLEDeviceManager::hasManufacturerData(const BLEDevice* device) const
{
    if (BLEUtils::isLocalBLE(*device) == true)
    {
        return (_manufacturer_data_length != 0);
    }
    
    const uint8_t* manufactgurer_data = NULL;
    uint8_t manufactgurer_data_len = 0;
    return getDataFromAdvertiseByType(device, 
                                      BT_DATA_MANUFACTURER_DATA,
                                      manufactgurer_data, 
                                      manufactgurer_data_len);
}

bool BLEDeviceManager::getManufacturerData (const BLEDevice* device, 
                                            uint8_t* manu_data, 
                                            uint8_t&manu_data_len) const
{
    if (BLEUtils::isLocalBLE(*device) == true)
    {
        return (_manufacturer_data_length != 0);
    }
    
    const uint8_t* manufactgurer_data = NULL;
    uint8_t manufactgurer_data_len = 0;
    bool retval = getDataFromAdvertiseByType(device, 
                                              BT_DATA_MANUFACTURER_DATA,
                                              manufactgurer_data, 
                                              manufactgurer_data_len);
    if (retval)
    {
        memcpy (manu_data, manufactgurer_data, manufactgurer_data_len);
        manu_data_len = manufactgurer_data_len;
    }
    return retval;
}

bool BLEDeviceManager::hasAdvertisedServiceUuid(const BLEDevice* device) const
{
    if (BLEUtils::isLocalBLE(*device) == true)
    {
        return _has_service_uuid;
    }
    
    uint8_t service_cnt = advertisedServiceUuidCount(device);
    return (service_cnt > 0);
}

bool BLEDeviceManager::hasAdvertisedServiceUuid(const BLEDevice* device, int index) const
{
    uint8_t service_cnt = advertisedServiceUuidCount(device);
    return (service_cnt > index);
}

void BLEDeviceManager::getDeviceAdvertiseBuffer(const bt_addr_le_t* addr, 
                                                const uint8_t* &adv_data,
                                                uint8_t &adv_len) const
{
    const bt_addr_le_t* temp   = NULL;
    // Connected device
    for (int i = 0; i < BLE_MAX_CONN_CFG; i++)
    {
        temp = &_peer_peripheral[i];
        if (bt_addr_le_cmp(temp, addr) == 0)
        {
            adv_data = _peer_peripheral_adv_data[i];
            adv_len = _peer_peripheral_adv_data_len[i];
            return;
        }
    }
    
    // Connecting device
    if (bt_addr_le_cmp(&_wait_for_connect_peripheral, addr) == 0)
    {
        adv_data = _wait_for_connect_peripheral_adv_data;
        adv_len = _wait_for_connect_peripheral_adv_data_len;
        return;
    }
    
    // Available device
    if (bt_addr_le_cmp(&_available_for_connect_peripheral, addr) == 0)
    {
        adv_data = _available_for_connect_peripheral_adv_data;
        adv_len = _available_for_connect_peripheral_adv_data_len;
        return;
    }
    return;
}

void BLEDeviceManager::getDeviceScanResponseBuffer(const bt_addr_le_t* addr, 
                                                   const uint8_t* &adv_data,
                                                   uint8_t &adv_len) const
{
    const bt_addr_le_t* temp   = NULL;
    // Connected device
    for (int i = 0; i < BLE_MAX_CONN_CFG; i++)
    {
        temp = &_peer_peripheral[i];
        if (bt_addr_le_cmp(temp, addr) == 0)
        {
            adv_data = _peer_peripheral_scan_rsp_data[i];
            adv_len = _peer_peripheral_scan_rsp_data_len[i];
            return;
        }
    }
    
    // Connecting device
    if (bt_addr_le_cmp(&_wait_for_connect_peripheral, addr) == 0)
    {
        adv_data = _wait_for_connect_peripheral_scan_rsp_data;
        adv_len = _wait_for_connect_peripheral_scan_rsp_data_len;
        return;
    }
    
    // Available device
    if (bt_addr_le_cmp(&_available_for_connect_peripheral, addr) == 0)
    {
        adv_data = _available_for_connect_peripheral_scan_rsp_data;
        adv_len = _available_for_connect_peripheral_scan_rsp_data_len;
        return;
    }
    return;
}

int BLEDeviceManager::advertisedServiceUuidCount(const BLEDevice* device) const
{
    const uint8_t* adv_data = NULL;
    uint8_t adv_data_len = 0;
    uint8_t service_cnt = 0;
    
    if (BLEUtils::isLocalBLE(*device) == true)
    {
        if (_has_service_uuid)
            service_cnt++;
        return  service_cnt;
    }
    
    getDeviceAdvertiseBuffer(device->bt_le_address(),
                             adv_data,
                             adv_data_len);
    if (NULL == adv_data)
    {
        return service_cnt;
    }
    
    while (adv_data_len > 1)
    {
        uint8_t len = adv_data[0];
        uint8_t type = adv_data[1];

        /* Check for early termination */
        if (len == 0 || ((len + 1) > adv_data_len))
        {
            return service_cnt;
        }

	/* Sid, 2/15/2017.  Sandeep reported that Apple devices may use
	   BT_DATA_UUID16_SOME and BT_DATA_UUID128_SOME in addition to ALL.
	   Practically, these types are same as ALL. */
        if (type == BT_DATA_UUID16_ALL ||
            type == BT_DATA_UUID128_ALL ||
	    type == BT_DATA_UUID16_SOME ||
	    type == BT_DATA_UUID128_SOME)
        {
            service_cnt++;
        }

        adv_data_len -= len + 1;
        adv_data += len + 1;
    }
    return service_cnt;
}

String BLEDeviceManager::localName(const BLEDevice* device) const
{
    if (BLEUtils::isLocalBLE(*device) == true)
    {
        return _local_name;
    }

    const uint8_t* local_name = NULL;
    uint8_t local_name_len = 0;
    String temp("");
    char local_name_buff[BLE_MAX_ADV_SIZE];
    bool retval = getDataFromAdvertiseByType(device, 
                                             BT_DATA_NAME_COMPLETE,
                                             local_name, 
                                             local_name_len);
    if (false == retval)
    {
        retval = getDataFromAdvertiseByType(device,
                                            BT_DATA_NAME_SHORTENED,
                                            local_name,
                                            local_name_len);
    }

    if (true == retval) 
    {
        if (local_name_len >= BLE_MAX_ADV_SIZE)
        {
            local_name_len = BLE_MAX_ADV_SIZE - 1;
        }
        memcpy(local_name_buff, local_name, local_name_len);
        local_name_buff[local_name_len] = '\0';
        temp = local_name_buff;
    }
    
    return temp;
}

String BLEDeviceManager::advertisedServiceUuid(const BLEDevice* device) const
{
    return advertisedServiceUuid(device, 0);
}

String BLEDeviceManager::advertisedServiceUuid(const BLEDevice* device, int index) const
{
    const uint8_t* adv_data = NULL;
    uint8_t adv_data_len = 0;
    uint8_t service_cnt = 0;
    bt_uuid_128_t service_uuid;
    char uuid_string[37];
    
    memset(uuid_string, 0, sizeof(uuid_string));
    
    if (BLEUtils::isLocalBLE(*device) == true)
    {
        // Local device only support advertise 1 service now.
        if (_has_service_uuid && index == 0)
        {
            BLEUtils::uuidBT2String(&_service_uuid.uuid, uuid_string);
        }
        return  String(uuid_string);
    }
    
    getDeviceAdvertiseBuffer(device->bt_le_address(),
                             adv_data,
                             adv_data_len);
    
    if ((uint8_t *)NULL == adv_data)
    {
        return String(uuid_string);
    }
    
    while (adv_data_len > 1)
    {
        uint8_t len = adv_data[0];
        uint8_t type = adv_data[1];

        /* Check for early termination */
        if (len == 0 || ((len + 1) > adv_data_len))
        {
            break;
        }

        if (type == BT_DATA_UUID16_ALL ||
            type == BT_DATA_UUID128_ALL ||
	    type == BT_DATA_UUID16_SOME ||
	    type == BT_DATA_UUID128_SOME)
        {
            service_cnt++;
        }
        
        if (index < service_cnt)
        {
            if (type == BT_DATA_UUID16_ALL ||
                type == BT_DATA_UUID16_SOME)
            {
                service_uuid.uuid.type = BT_UUID_TYPE_16;
                memcpy(&BT_UUID_16(&service_uuid.uuid)->val, &adv_data[2], 2);
            }
            else
            {
                service_uuid.uuid.type = BT_UUID_TYPE_128;
                memcpy(service_uuid.val, &adv_data[2], 16);
            }
            
            BLEUtils::uuidBT2String(&service_uuid.uuid, uuid_string);
            
            break;
        }

        adv_data_len -= len + 1;
        adv_data += len + 1;
    }
    return String(uuid_string);
}

int BLEDeviceManager::rssi(const BLEDevice* device) const
{
    const bt_addr_le_t* temp   = NULL;
    const bt_addr_le_t* addr   = device->bt_le_address();
    // Connected device
    for (int i = 0; i < BLE_MAX_CONN_CFG; i++)
    {
        temp = &_peer_peripheral[i];
        if (bt_addr_le_cmp(temp, addr) == 0)
        {
            return _peer_peripheral_adv_rssi[i];
        }
    }
    
    // Connecting device
    if (bt_addr_le_cmp(&_wait_for_connect_peripheral, addr) == 0)
    {
        return _wait_for_connect_peripheral_adv_rssi;
    }
    
    // Available device
    if (bt_addr_le_cmp(&_available_for_connect_peripheral, addr) == 0)
    {
        return _available_for_connect_peripheral_adv_rssi;
    }
    return 0;
}

bool BLEDeviceManager::connect(BLEDevice &device)
{
    // 
    uint64_t timestamp = millis();
    uint64_t timestampcur = timestamp;
    bool ret = true;
    if (_available_for_connect_peripheral_connectable == false)
    {
        return false;
    }
    
    bt_addr_le_copy(&_wait_for_connect_peripheral, device.bt_le_address());
    // Buffer the ADV data
    memcpy(_wait_for_connect_peripheral_adv_data, _available_for_connect_peripheral_adv_data, BLE_MAX_ADV_SIZE);
    memcpy(_wait_for_connect_peripheral_scan_rsp_data, _available_for_connect_peripheral_scan_rsp_data, BLE_MAX_ADV_SIZE);
    _wait_for_connect_peripheral_adv_data_len = _available_for_connect_peripheral_adv_data_len;
    _wait_for_connect_peripheral_scan_rsp_data_len = _available_for_connect_peripheral_scan_rsp_data_len;
    _wait_for_connect_peripheral_adv_rssi = _available_for_connect_peripheral_adv_rssi;

    startScanningWithDuplicates();
    
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
        if (_connecting == true)
        {
            ret = true;
            while (_connecting == true)
            {
                pr_info(LOG_MODULE_BLE, "%s-%d: Connect request sent, wait for response", __FUNCTION__, __LINE__);
                delay(2000);
            }
        }
        else
        {
            stopScanning();
        }
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
                // Buffer the ADV data
                memcpy(_peer_peripheral_adv_data[i], 
                       _wait_for_connect_peripheral_adv_data, 
                       BLE_MAX_ADV_SIZE);
                _peer_peripheral_adv_data_len[i] = _wait_for_connect_peripheral_adv_data_len;
                memcpy(_peer_peripheral_scan_rsp_data[i], 
                       _wait_for_connect_peripheral_scan_rsp_data, 
                       BLE_MAX_ADV_SIZE);
                _peer_peripheral_scan_rsp_data_len[i] = _wait_for_connect_peripheral_scan_rsp_data_len;
                _peer_peripheral_adv_rssi[i] = _wait_for_connect_peripheral_adv_rssi;
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
            _connecting = true;
            bt_conn_unref(conn);
        }
    }
    return retval;
}

String BLEDeviceManager::deviceName(const BLEDevice* device)
{
    if (BLEUtils::isLocalBLE(*device) == true)
    {
        return _device_name;
    }
    return String("");
}

int BLEDeviceManager::appearance()
{
    return BLEProfileManager::instance()->getAppearance();
}

BLEDeviceManager* BLEDeviceManager::instance()
{
    if (_instance == NULL)
    {
        _instance = new BLEDeviceManager();
        BLE_LIB_ASSERT(_instance != NULL);
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
        // Peripheral has established the connection with this Central device
        memset(&_wait_for_connect_peripheral, 0, sizeof(_wait_for_connect_peripheral));
        _connecting = false;
    }
    // The peripheral and central can work as GATT server. Reserve one buffer for peer device
    BLEProfileManager::instance()->handleConnectedEvent(bt_conn_get_dst(conn));
    
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
                memset(_peer_peripheral_adv_data[i], 0, BLE_MAX_ADV_SIZE);
                _peer_peripheral_adv_data_len[i] = 0;
                _peer_peripheral_adv_rssi[i] = 0;
                memset(_peer_peripheral_scan_rsp_data[i], 0, BLE_MAX_ADV_SIZE);
                _peer_peripheral_scan_rsp_data_len[i] = 0;
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

bool BLEDeviceManager::deviceInDuplicateFilterBuffer(const bt_addr_le_t* addr)
{
    bool retVal = false;
    for (uint8_t i = 0; 
         i < (sizeof(_peer_duplicate_address_buffer) / sizeof(bt_addr_le_t)); 
         i++)
    {
        if (0 == bt_addr_le_cmp(addr, &_peer_duplicate_address_buffer[i]))
        {
            retVal = true;
            break;
        }
    }
    return retVal;
}

void BLEDeviceManager::updateDuplicateFilter(const bt_addr_le_t* addr)
{
    uint8_t i = (_duplicate_filter_header + 1) % (ARRAY_SIZE(_peer_duplicate_address_buffer));
    if (deviceInDuplicateFilterBuffer(addr))
    {
        return;
    }
    bt_addr_le_copy(&_peer_duplicate_address_buffer[_duplicate_filter_header],
                 addr);
    if (i == _duplicate_filter_tail)
    {
        _duplicate_filter_tail = (_duplicate_filter_tail + 1) % (ARRAY_SIZE(_peer_duplicate_address_buffer));
    }
    _duplicate_filter_header = i;
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
        if ((timestamp_delta <= 2000) && (max_delta < timestamp_delta) && (_peer_scan_rsp_data_len[i] >= 0 || !_peer_adv_connectable[i]))
        {
            // Eable the duplicate filter
            if (_adv_duplicate_filter_enabled && 
                true == deviceInDuplicateFilterBuffer(temp))
            {
                _peer_adv_mill[i] -= 2000; // Invalid the item
                continue;
            }
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
            bt_addr_le_copy(&_available_for_connect_peripheral, temp);
            memcpy(_available_for_connect_peripheral_adv_data, _peer_adv_data[index], BLE_MAX_ADV_SIZE);
            memcpy(_available_for_connect_peripheral_scan_rsp_data, _peer_scan_rsp_data[index], BLE_MAX_ADV_SIZE);
            _available_for_connect_peripheral_scan_rsp_data_len = _peer_scan_rsp_data_len[index];
            _available_for_connect_peripheral_adv_data_len = _peer_adv_data_len[index];
            _available_for_connect_peripheral_adv_rssi = _peer_adv_rssi[index];
            _available_for_connect_peripheral_connectable = _peer_adv_connectable[index];
            //pr_debug(LOG_MODULE_BLE, "%s-%d:Con addr-%s", __FUNCTION__, __LINE__, BLEUtils::macAddressBT2String(*temp).c_str());
            _peer_adv_mill[index] -= 2000; // Set it as expired
            if (_adv_duplicate_filter_enabled)
            {
                updateDuplicateFilter(temp);
            }
        }
    }
    return tempdevice;
}

bool BLEDeviceManager::setAdvertiseBuffer(const bt_addr_le_t* bt_addr,
                                          const uint8_t *ad, 
                                          uint8_t data_len,
                                          int8_t rssi,
                                          bool connectable)
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
            {
                index = i;
                _peer_scan_rsp_data_len[index] = -1; // Invalid the scan response
            }
        }
        
        if (bt_addr_le_cmp(temp, bt_addr) == 0)
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
        _peer_adv_rssi[index] = rssi;
        // Update the timestamp
        _peer_adv_mill[index] = timestamp;
        _peer_adv_connectable[index] = connectable;
        retval = true;
    }
    
    return retval;
}

bool BLEDeviceManager::setScanRespBuffer(const bt_addr_le_t* bt_addr,
                                          const uint8_t *ad, 
                                          uint8_t data_len,
                                          int8_t rssi)
{
    bt_addr_le_t* temp = NULL;
    uint64_t timestamp = millis();
    uint8_t index = BLE_MAX_ADV_BUFFER_CFG;
    uint8_t i = 0;
    bool retval = false;
    //pr_debug(LOG_MODULE_BLE, "%s-%d-1", __FUNCTION__, __LINE__);
    for (i = 0; i < BLE_MAX_ADV_BUFFER_CFG; i++)
    {
        temp = &_peer_adv_buffer[i];
        
        if (bt_addr_le_cmp(temp, bt_addr) == 0)
        {
            // The device alread in the buffer
            index = i;
            break;
        }
    }
    
    //pr_debug(LOG_MODULE_BLE, "%s-%d", __FUNCTION__, __LINE__);
    if (index < BLE_MAX_ADV_BUFFER_CFG)
    {
        if (data_len > BLE_MAX_ADV_SIZE)
        {
            data_len = BLE_MAX_ADV_SIZE;
        }
        memcpy(_peer_scan_rsp_data[index], ad, data_len);
        _peer_scan_rsp_data_len[index] = data_len;
        //_peer_adv_rssi[index] = rssi;
        // Update the timestamp
        _peer_adv_mill[index] = timestamp;
        retval = true;
    }
    
    return retval;
}

uint8_t BLEDeviceManager::getTempAdvertiseIndexFromBuffer(const bt_addr_le_t* bt_addr)
{
    bt_addr_le_t* temp = NULL;
    uint8_t i = 0;
    
    for (i = 0; i < BLE_MAX_ADV_BUFFER_CFG; i++)
    {
        temp = &_peer_temp_adv_buffer[i];
        
        if (bt_addr_le_cmp(temp, bt_addr) == 0)
        {
            // The device alread in the buffer
            break;
        }
    }
    
    return i;
}

void BLEDeviceManager::setTempAdvertiseBuffer(const bt_addr_le_t* bt_addr, 
                                              int8_t rssi, 
                                              const uint8_t *ad, 
                                              uint8_t data_len, 
                                              bool connectable)
{
    bt_addr_le_t* temp = NULL;
    uint8_t i = getTempAdvertiseIndexFromBuffer(bt_addr);
    if (i >= BLE_MAX_ADV_BUFFER_CFG)
    {
        _peer_temp_dev_index = (_peer_temp_dev_index + 1) % BLE_MAX_ADV_BUFFER_CFG;
        i = _peer_temp_dev_index;
    }
    
    temp = &_peer_temp_adv_buffer[i];
    memcpy(temp, bt_addr, sizeof (bt_addr_le_t));
    if (data_len > BLE_MAX_ADV_SIZE)
    {
        data_len = BLE_MAX_ADV_SIZE;
    }
    
    memcpy(_peer_temp_adv_data[i], ad, data_len);
    _peer_temp_adv_data_len[i] = data_len;
    _peer_temp_adv_connectable[i] = connectable;
    
    return;
}

void BLEDeviceManager::advertiseAcceptHandler(const bt_addr_le_t *addr, 
                                              int8_t rssi, 
                                              uint8_t type,
                                              const uint8_t *ad, 
                                              uint8_t data_len)
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
        const uint8_t *adv_data = ad;
        uint8_t adv_data_len = data_len;
        bool connectable = (BT_LE_ADV_NONCONN_IND != type);
        bool update_advertise_data = true;
        // The critical is accepted
        //  Find the oldest and expired buffer 
        if (BT_LE_ADV_SCAN_RSP == type)
        {
            update_advertise_data = false;
            pr_debug(LOG_MODULE_BLE, "%s-%d", __FUNCTION__, __LINE__);
            if (false == setScanRespBuffer(addr, ad, data_len, rssi))
            {
                // Find the device in the ADV temp buffer
                uint8_t tempIndex = getTempAdvertiseIndexFromBuffer(addr);
                if (tempIndex < BLE_MAX_ADV_BUFFER_CFG)
                {
                    adv_data = _peer_temp_adv_data[tempIndex];
                    adv_data_len = _peer_temp_adv_data_len[tempIndex];
                    connectable = _peer_temp_adv_connectable[tempIndex];
                    update_advertise_data = true;
                }
            }
        }
        pr_debug(LOG_MODULE_BLE, "%s-%d", __FUNCTION__, __LINE__);
        
        if (true == update_advertise_data)
        {
            if (false == setAdvertiseBuffer(addr, 
                                            adv_data, 
                                            adv_data_len, 
                                            rssi, 
                                            connectable))
            {
                pr_info(LOG_MODULE_BLE, "No buffer to store the ADV\n");
            }
            else if (BT_LE_ADV_SCAN_RSP == type)
            {
                setScanRespBuffer(addr, ad, data_len, rssi);
            }
        }
    }
    
}

void BLEDeviceManager::handleDeviceFound(const bt_addr_le_t *addr, 
                                         int8_t rssi, 
                                         uint8_t type,
                                         const uint8_t *ad, 
                                         uint8_t data_len)
{
    const uint8_t *data = ad;
    uint8_t real_adv_len = data_len;
    
    /* We're only interested in connectable events */
    //pr_debug(LOG_MODULE_BLE, "%s-%d", __FUNCTION__, __LINE__);
    // Filter address
    if (BLEUtils::macAddressValid(_adv_accept_device) == true && 
       (memcmp(addr->val, _adv_accept_device.val, sizeof (addr->val)) != 0))
    {
        pr_debug(LOG_MODULE_BLE, "%s-%d", __FUNCTION__, __LINE__);
        return;
    }
    
    while (data_len > 1)
    {
        uint8_t len = data[0];

        /* Check for early termination */
        if (len == 0)
        {
            return;
        }

        if ((len + 1) > data_len) {    // Sid. KW, cannot be (data_len < 2)
            pr_info(LOG_MODULE_BLE, "AD malformed\n");
            return;
        }

        if (true == advertiseDataProc(data[1], &data[2], len - 1))
        {
            advertiseAcceptHandler(addr, rssi, type, ad, real_adv_len);
            //pr_debug(LOG_MODULE_BLE, "%s-%d: Done", __FUNCTION__, __LINE__);
            return;
        }

        data_len -= len + 1;
        data += len + 1;
    }
    //pr_debug(LOG_MODULE_BLE, "%s: done", __FUNCTION__);
    // Doesn't accept the ADV/scan data
    // Check it in the buffer
    if (BT_LE_ADV_SCAN_RSP == type)
    {
        // Find the ADV and set response
        setScanRespBuffer(addr, ad, real_adv_len, rssi);
    }
    else
    {
        // Add advertise into buffer
        setTempAdvertiseBuffer(addr, rssi, ad, real_adv_len, BT_LE_ADV_NONCONN_IND != type);
    }
    
}


    
