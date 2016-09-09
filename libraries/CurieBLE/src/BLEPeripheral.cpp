/*
 * Copyright (c) 2015 Intel Corporation.  All rights reserved.
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

#include "BLEPeripheral.h"
#include "BLEPeripheralRole.h"

//#include "BLECharacteristic.h"

BLEPeripheral::BLEPeripheral(void) :
    _local_name(NULL),
    _appearance(0),
    _adv_data_idx(0)
{
    memset(_adv_data, 0x00, sizeof(_adv_data));
    
    // Default Advertising parameter
    setConnectable(true);
}

BLEPeripheral::~BLEPeripheral(void)
{
}

bool BLEPeripheral::begin()
{
    bool ret = false;
    
    pr_info(LOG_MODULE_BLE, "%s: %d", __FUNCTION__, 1);
    
    ret = BLEPeripheralRole::instance()->begin();
    if (!ret)
    {
        return false;
    }
    
    pr_info(LOG_MODULE_BLE, "%s: %d", __FUNCTION__, 2);
    
    return (startAdvertising() == BLE_STATUS_SUCCESS);
}

void
BLEPeripheral::poll()
{
    // no-op for now
    delay(1);
}

void
BLEPeripheral::end()
{
    BLEPeripheralRole::instance()->stop();
}

void
BLEPeripheral::setAdvertisedServiceUuid(const bt_uuid_t* advertisedServiceUuid)
{
    _advertise_service_uuid = advertisedServiceUuid;
}

void
BLEPeripheral::setLocalName(const char* localName)
{
    _local_name = localName;
}

void
BLEPeripheral::setAdvertisedServiceData(const bt_uuid_t* serviceDataUuid, 
                                        uint8_t* serviceData, 
                                        uint8_t serviceDataLength)
{
    _service_data_uuid = serviceDataUuid;
    _service_data = serviceData;
    _service_data_length = serviceDataLength;
}

void 
BLEPeripheral::setAdvertisingInterval(float interval_min,
                                      float interval_max)
{
    uint16_t max = (uint16_t) MSEC_TO_UNITS(interval_max, UNIT_0_625_MS);
    uint16_t min = (uint16_t) MSEC_TO_UNITS(interval_min, UNIT_0_625_MS);
    BLEPeripheralRole::instance()->setAdvertisingInterval(min, max);
}

void 
BLEPeripheral::setAdvertisingInterval(float advertisingInterval)
{
    setAdvertisingInterval(advertisingInterval, advertisingInterval);
}

void
BLEPeripheral::setConnectable(bool connectable)
{
    uint8_t type = BT_LE_ADV_IND;
    if (connectable == false)
    {
        type = BT_LE_ADV_NONCONN_IND;
    }
    BLEPeripheralRole::instance()->setAdvertisingType(type);
}

void
BLEPeripheral::setDeviceName(const char deviceName[])
{
    BLEPeripheralRole::instance()->setDeviceName(deviceName);
}

void
BLEPeripheral::setAppearance(const uint16_t appearance)
{
    _appearance = appearance;
}

void
BLEPeripheral::setConnectionInterval(const unsigned short minConnInterval, const unsigned short maxConnInterval)
{
    BLEPeripheralRole::instance()->setConnectionInterval(minConnInterval,
                                                         maxConnInterval);
}

void
BLEPeripheral::setEventHandler(BLERoleEvent event, BLERoleEventHandler callback)
{
    BLEPeripheralRole::instance()->setEventHandler(event, callback);
}

bool
BLEPeripheral::disconnect()
{
    return BLEPeripheralRole::instance()->disconnect();
}

BLECentralHelper
BLEPeripheral::central()
{
    return BLEPeripheralRole::instance()->central();
}

bool
BLEPeripheral::connected()
{
    return BLEPeripheralRole::instance()->connected();
}

BleStatus
BLEPeripheral::addAttribute(BLEAttribute& attribute)
{
    return BLEPeripheralRole::instance()->addAttribute(attribute);
}


BleStatus
BLEPeripheral::_advDataInit(void)
{
    uint8_t lengthTotal = 2; // Flags data length
    _adv_data_idx = 0;
    
    /* Add flags */
    _adv_type = (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR);
    _adv_data[_adv_data_idx].type = BT_DATA_FLAGS;
    _adv_data[_adv_data_idx].data = &_adv_type;
    _adv_data[_adv_data_idx].data_len = 1;
    _adv_data_idx++;

    if (_advertise_service_uuid) 
    {
        uint8_t type;
        uint8_t length;
        uint8_t *data = NULL;
        
        pr_info(LOG_MODULE_BLE, "ADV Type-%d", _advertise_service_uuid->type);
        if (BT_UUID_TYPE_16 == _advertise_service_uuid->type)
        {
            //UINT16_TO_LESTREAM(adv_tmp, uuid.uuid16);
            data = (uint8_t *)&(((bt_uuid_16_t *)_advertise_service_uuid)->val);
            length = UUID_SIZE_16;
            type = BT_DATA_UUID16_ALL;
        }
        else if (BT_UUID_TYPE_128 == _advertise_service_uuid->type)
        {
            data = ((bt_uuid_128_t *)_advertise_service_uuid)->val;
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

    if (_local_name)
    {
        /* Add device name (truncated if too long) */
        _adv_data[_adv_data_idx].type = BT_DATA_NAME_COMPLETE;
        _adv_data[_adv_data_idx].data = (const uint8_t*)_local_name;
        _adv_data[_adv_data_idx].data_len = strlen(_local_name);
        _adv_data_idx++;
        
        lengthTotal += strlen(_local_name);
        pr_info(LOG_MODULE_BLE, "Local Name -%s", _local_name);
        pr_info(LOG_MODULE_BLE, "Local Name Len -%d", strlen(_local_name));
    }

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
    if (lengthTotal > BLE_MAX_ADV_SIZE)
    {
        pr_error(LOG_MODULE_BLE, "ADV Total length-%d", lengthTotal);
        // Service data block is too large.
        return BLE_STATUS_ERROR_PARAMETER;
    }
    return BLE_STATUS_SUCCESS;
}

BleStatus
BLEPeripheral::startAdvertising()
{
    BleStatus status = BLE_STATUS_SUCCESS;
    status = _advDataInit();
    if (BLE_STATUS_SUCCESS != status)
    {
        return status;
    }
    status = BLEPeripheralRole::instance()->startAdvertising(_adv_data, 
                                                             _adv_data_idx, 
                                                             NULL, 
                                                             0);
    return status;
}

BleStatus
BLEPeripheral::stopAdvertising()
{
    BleStatus status = BLE_STATUS_SUCCESS;
    
    status = BLEPeripheralRole::instance()->stopAdvertising();
    return status;
}

BLECentralHelper  *BLEPeripheral::getPeerCentralBLE(BLEHelper& central)
{
	return (BLECentralHelper *)(&central);
}
