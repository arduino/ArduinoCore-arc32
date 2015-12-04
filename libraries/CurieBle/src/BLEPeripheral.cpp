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

#include "BLECharacteristic.h"
#include "BLEDescriptor.h"
#include "BLEService.h"
#include "BLEUuid.h"


#define BLE_DISCONNECT_REASON_LOCAL_TERMINATION 0x16

void
blePeripheralGapEventHandler(ble_client_gap_event_t event, struct ble_gap_event *event_data, void *param)
{
    BLEPeripheral* p = (BLEPeripheral*)param;

    p->handleGapEvent(event, event_data);
}

void
blePeripheralGattsEventHandler(ble_client_gatts_event_t event, struct ble_gatts_evt_msg *event_data, void *param)
{
    BLEPeripheral* p = (BLEPeripheral*)param;

    p->handleGattsEvent(event, event_data);
}

BLEPeripheral::BLEPeripheral(void) :
    _state(BLE_PERIPH_STATE_NOT_READY),
    _advertise_service_uuid(NULL),
    _local_name(NULL),
    _appearance(0),
    _central(this),
    _attributes(NULL),
    _num_attributes(0),
    _last_added_characteritic(NULL)
{
    memset(_event_handlers, 0x00, sizeof(_event_handlers));

    ble_client_get_factory_config(&_local_bda, _device_name);
}

BLEPeripheral::~BLEPeripheral(void)
{
    if (this->_attributes) {
        free(this->_attributes);
    }
}

bool BLEPeripheral::begin()
{
    BleStatus status;

    status = _init();
    if (status != BLE_STATUS_SUCCESS) {
        return false;
    }

    /* Populate advertising data
     */
    _advDataInit();

    status = ble_client_gap_wr_adv_data(_adv_data, _adv_data_len);
    if (BLE_STATUS_SUCCESS != status) {
        return false;
    }

    uint16_t lastServiceHandle = 0;

    for (int i = 0; i < _num_attributes; i++) {
        BLEAttribute* attribute = _attributes[i];
        BLEAttributeType type = attribute->type();
        bool addResult = false;

        if (BLETypeService == type) {
            BLEService* service = (BLEService*)attribute;

            addResult = service->add();

            lastServiceHandle = service->handle();
        } else if (BLETypeCharacteristic == type) {
            BLECharacteristic* characteristic = (BLECharacteristic*)attribute;

            addResult = characteristic->add(lastServiceHandle);
        } else if (BLETypeDescriptor == type) {
            BLEDescriptor *descriptor = (BLEDescriptor*)attribute;

            if (strcmp(descriptor->uuid(), "2901") == 0 ||
                strcmp(descriptor->uuid(), "2902") == 0 ||
                strcmp(descriptor->uuid(), "2903") == 0 ||
                strcmp(descriptor->uuid(), "2904") == 0) {
                continue; // skip
            }

            addResult = descriptor->add(lastServiceHandle);
        }

        if (!addResult) {
            return false;
        }
    }

    return (_startAdvertising() == BLE_STATUS_SUCCESS);
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
    _stop();
}

void
BLEPeripheral::setAdvertisedServiceUuid(const char* advertisedServiceUuid)
{
    _advertise_service_uuid = advertisedServiceUuid;
}

void
BLEPeripheral::setLocalName(const char* localName)
{
    _local_name = localName;
}

void
BLEPeripheral::setDeviceName(const char deviceName[])
{
    memset(_device_name, 0, sizeof(_device_name));
    if (deviceName && deviceName[0]) {
        int len = strlen(deviceName);
        if (len > BLE_MAX_DEVICE_NAME)
            len = BLE_MAX_DEVICE_NAME;
        memcpy(_device_name, deviceName, len);
    }
}

void
BLEPeripheral::setAppearance(const uint16_t appearance)
{
    _appearance = appearance;
}

void
BLEPeripheral::setEventHandler(BLEPeripheralEvent event, BLEPeripheralEventHandler callback)
{
  if (event < sizeof(_event_handlers)) {
    _event_handlers[event] = callback;
  }
}

void
BLEPeripheral::addAttribute(BLEAttribute& attribute)
{
    if (_attributes == NULL) {
        _attributes = (BLEAttribute**)malloc(BLEAttribute::numAttributes() * sizeof(BLEAttribute*));
    }

    _attributes[_num_attributes] = &attribute;
    _num_attributes++;

    BLEAttributeType type = attribute.type();

    if (BLETypeCharacteristic == type) {
        _last_added_characteritic = (BLECharacteristic*)&attribute;
    } else if (BLETypeDescriptor == type) {
        if (_last_added_characteritic) {
            BLEDescriptor* descriptor = (BLEDescriptor*)&attribute;

            if (strcmp("2901", descriptor->uuid()) == 0) {
                _last_added_characteritic->setUserDescription(descriptor);
            } else if (strcmp("2904", descriptor->uuid()) == 0) {
                _last_added_characteritic->setPresentationFormat(descriptor);
            }
        }
    }
}

bool
BLEPeripheral::disconnect()
{
    BleStatus status;

    if (BLE_PERIPH_STATE_CONNECTED == _state) {
        status = ble_client_gap_disconnect(BLE_DISCONNECT_REASON_LOCAL_TERMINATION);
    } else {
        status = BLE_STATUS_WRONG_STATE;
    }

    return (status == BLE_STATUS_SUCCESS);
}

BLECentral
BLEPeripheral::central()
{
    poll();

    return _central;
}

bool
BLEPeripheral::connected()
{
    poll();

    return _central;
}

BleStatus
BLEPeripheral::_init()
{
    BleStatus status;
    int8_t txPower = 127;

    if (BLE_PERIPH_STATE_NOT_READY != _state)
        return BLE_STATUS_WRONG_STATE;

    status = ble_client_init(blePeripheralGapEventHandler, this,
                             blePeripheralGattsEventHandler, this);
    if (BLE_STATUS_SUCCESS != status) {
        return status;
    }

    status = ble_client_gap_set_enable_config(_device_name, &_local_bda, _appearance, txPower);
    if (BLE_STATUS_SUCCESS != status) {
        return status;
    }

    _state = BLE_PERIPH_STATE_READY;
    return BLE_STATUS_SUCCESS;
}

void
BLEPeripheral::_advDataInit(void)
{
    uint8_t *adv_tmp = _adv_data;

    memset(_adv_data, 0, sizeof(_adv_data));

    /* Add flags */
    *adv_tmp++ = 2;
    *adv_tmp++ = BLE_ADV_TYPE_FLAGS;
    *adv_tmp++ = BLE_SVC_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;
    _adv_data_len = 3;

    if (_advertise_service_uuid) {
        BLEUuid bleUuid = BLEUuid(_advertise_service_uuid);
        struct bt_uuid uuid = bleUuid.uuid();

        if (BT_UUID16 == uuid.type) {
            uint8_t *adv_tmp = &_adv_data[_adv_data_len];
            *adv_tmp++ = (1 + sizeof(uint16_t)); /* Segment data length */
            *adv_tmp++ = BLE_ADV_TYPE_INC_16_UUID;
            UINT16_TO_LESTREAM(adv_tmp, uuid.uuid16);
            _adv_data_len += (2 + sizeof(uint16_t));
        } else if (BT_UUID128 == uuid.type) {
            uint8_t *adv_tmp = &_adv_data[_adv_data_len];
            *adv_tmp++ = (1 + MAX_UUID_SIZE); /* Segment data length */
            *adv_tmp++ = BLE_ADV_TYPE_INC_128_UUID;
            memcpy(adv_tmp, uuid.uuid128, MAX_UUID_SIZE);
            _adv_data_len += (2 + MAX_UUID_SIZE);
        }
    }

    if (_local_name) {
        /* Add device name (truncated if too long) */
        uint8_t calculated_len;

        adv_tmp = &_adv_data[_adv_data_len];
        if (_adv_data_len + strlen(_local_name) + 2 <= BLE_MAX_ADV_SIZE) {
            *adv_tmp++ = strlen(_local_name) + 1;
            *adv_tmp++ = BLE_ADV_TYPE_COMP_LOCAL_NAME;
            calculated_len = strlen(_local_name);
        } else {
            *adv_tmp++ = BLE_MAX_ADV_SIZE - _adv_data_len - 1;
            *adv_tmp++ = BLE_ADV_TYPE_SHORT_LOCAL_NAME;
            calculated_len = BLE_MAX_ADV_SIZE - _adv_data_len - 2;
        }

        memcpy(adv_tmp, _local_name, calculated_len);
        _adv_data_len += calculated_len + 2;
    }
}

BleStatus
BLEPeripheral::_startAdvertising()
{
    BleStatus status;

    if (_state != BLE_PERIPH_STATE_READY)
        return BLE_STATUS_WRONG_STATE;

    status = ble_client_gap_start_advertise(0); // 0 = no timeout
    if (BLE_STATUS_SUCCESS != status)
        return status;

    _state = BLE_PERIPH_STATE_ADVERTISING;
    return BLE_STATUS_SUCCESS;
}

BleStatus
BLEPeripheral::_stop(void)
{
    BleStatus status;

    if (BLE_PERIPH_STATE_ADVERTISING == _state)
        status = ble_client_gap_stop_advertise();
    else
        status = disconnect();

    if (BLE_STATUS_SUCCESS != status)
        return status;

    _state = BLE_PERIPH_STATE_READY;
    return BLE_STATUS_SUCCESS;
}

void
BLEPeripheral::handleGapEvent(ble_client_gap_event_t event, struct ble_gap_event *event_data)
{
    if (BLE_CLIENT_GAP_EVENT_CONNECTED == event) {
        _state = BLE_PERIPH_STATE_CONNECTED;
        _central.setAddress(event_data->connected.peer_bda);

        if (_event_handlers[BLEConnected]) {
            _event_handlers[BLEConnected](_central);
        }
    } else if (BLE_CLIENT_GAP_EVENT_DISCONNECTED == event) {

        for (int i = 0; i < _num_attributes; i++) {
            BLEAttribute* attribute = _attributes[i];

            if (attribute->type() == BLETypeCharacteristic) {
                BLECharacteristic* characteristic = (BLECharacteristic*)attribute;

                characteristic->setCccdValue(_central, 0x0000); // reset CCCD
            }
        }

        if (_event_handlers[BLEDisconnected])
            _event_handlers[BLEDisconnected](_central);

        _state = BLE_PERIPH_STATE_READY;
        _central.clearAddress();

        _startAdvertising();
    } else if (BLE_CLIENT_GAP_EVENT_CONN_TIMEOUT == event) {
        _state = BLE_PERIPH_STATE_READY;

        _startAdvertising();
    }
}

void
BLEPeripheral::handleGattsEvent(ble_client_gatts_event_t event, struct ble_gatts_evt_msg *event_data)
{
    if (BLE_CLIENT_GATTS_EVENT_WRITE == event) {
        uint16_t handle = event_data->wr.attr_handle;

        for (int i = 0; i < _num_attributes; i++) {
            BLEAttribute* attribute = _attributes[i];

            if (attribute->type() != BLETypeCharacteristic) {
                continue;
            }

            BLECharacteristic* characteristic = (BLECharacteristic*)attribute;

            if (characteristic->valueHandle() == handle) {
                characteristic->setValue(_central, event_data->wr.data, event_data->wr.len);
                break;
            } else if (characteristic->cccdHandle() == handle) {
                uint16_t cccdValue = 0;

                memcpy(&cccdValue, event_data->wr.data, event_data->wr.len);

                characteristic->setCccdValue(_central, cccdValue);
                break;
            }
        }
    }
}
