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

#include "BlePeripheral.h"

#include "BleCharacteristic.h"
#include "BleDescriptor.h"
#include "BleService.h"
#include "BleUuid.h"


#define BLE_DISCONNECT_REASON_LOCAL_TERMINATION 0x16

void
blePeripheralGapEventHandler(ble_client_gap_event_t event, struct ble_gap_event *event_data, void *param)
{
    BlePeripheral* p = (BlePeripheral*)param;

    p->handleGapEvent(event, event_data);
}

void
blePeripheralGattsEventHandler(ble_client_gatts_event_t event, struct ble_gatts_evt_msg *event_data, void *param)
{
    BlePeripheral* p = (BlePeripheral*)param;

    p->handleGattsEvent(event, event_data);
}

BlePeripheral::BlePeripheral(void) :
    _central(this),
    _attributes(NULL),
    _num_attributes(0)
{
    _appearance = 0;
    _state = BLE_PERIPH_STATE_NOT_READY;

    memset(_event_handlers, 0x00, sizeof(_event_handlers));

    ble_client_get_factory_config(&_local_bda, _local_name);
}

BlePeripheral::~BlePeripheral(void)
{
    if (this->_attributes) {
        free(this->_attributes);
    }
}

BleStatus BlePeripheral::begin()
{
    BleStatus status;

    status = _init();
    if (status != BLE_STATUS_SUCCESS) {
        return status;
    }

    uint16_t lastServiceHandle = 0;

    for (int i = 0; i < _num_attributes; i++) {
        BleAttribute* attribute = _attributes[i];
        BleAttributeType type = attribute->type();

        if (BleTypeService == type) {
            BleService* service = (BleService*)attribute;

            status = service->add();

            lastServiceHandle = service->handle();
        } else if (BleTypeCharacteristic == type) {
            BleCharacteristic* characteristic = (BleCharacteristic*)attribute;

            characteristic->add(lastServiceHandle);
        } else if (BleTypeDescriptor == type) {
            // TODO: handle UUID 0x2901 and 0x2904 in a special way

            BleDescriptor *descriptor = (BleDescriptor*)attribute;

            status = descriptor->add(lastServiceHandle);
        }

        if (status != BLE_STATUS_SUCCESS) {
            return status;
        }
    }

    return _startAdvertising();
}

void
BlePeripheral::poll()
{
    // no-op for now
}

void
BlePeripheral::end()
{
    _stop();
}

BleStatus
BlePeripheral::setAdvertisedServiceUuid(const char* advertisedServiceUuid)
{
    BleUuid bleUuid = BleUuid(advertisedServiceUuid);
    struct bt_uuid uuid = bleUuid.uuid();


    /* Append UUID to advertising data
     * TODO - we could pack multiple UUIDs of the same type together
     *        to conserve bytes in the advertising data payload
     */
    if ((BT_UUID16 == uuid.type) &&
        (_adv_data_len + 2 + sizeof(uint16_t) <= BLE_MAX_ADV_SIZE)) {
        uint8_t *adv_tmp = &_adv_data[_adv_data_len];
        *adv_tmp++ = (1 + sizeof(uint16_t)); /* Segment data length */
        *adv_tmp++ = BLE_ADV_TYPE_INC_16_UUID;
        UINT16_TO_LESTREAM(adv_tmp, uuid.uuid16);
        _adv_data_len += (2 + sizeof(uint16_t));
    } else if ((BT_UUID128 == uuid.type) &&
               (_adv_data_len + 2 + MAX_UUID_SIZE <= BLE_MAX_ADV_SIZE)) {
        uint8_t *adv_tmp = &_adv_data[_adv_data_len];
        *adv_tmp++ = (1 + MAX_UUID_SIZE); /* Segment data length */
        *adv_tmp++ = BLE_ADV_TYPE_INC_128_UUID;
        memcpy(adv_tmp, uuid.uuid128, MAX_UUID_SIZE);
        _adv_data_len += (2 + MAX_UUID_SIZE);
    } else {
        /* Not enough space in advertising PDU for service UUID */
    }

    return BLE_STATUS_SUCCESS;
}

BleStatus
BlePeripheral::setLocalName(const char localName[])
{
    memset(_local_name, 0, sizeof(_local_name));
    if (localName && localName[0]) {
        int len = strlen(localName);
        if (len > BLE_MAX_DEVICE_NAME)
            len = BLE_MAX_DEVICE_NAME;
        memcpy(_local_name, localName, len);
    }

    return BLE_STATUS_SUCCESS;
}

BleStatus
BlePeripheral::setAppearance(const uint16_t appearance)
{
    _appearance = appearance;

    return BLE_STATUS_SUCCESS;
}

void
BlePeripheral::setEventHandler(BlePeripheralEvent event, BlePeripheralEventHandler callback)
{
  if (event < sizeof(_event_handlers)) {
    _event_handlers[event] = callback;
  }
}

BleStatus
BlePeripheral::addAttribute(BleAttribute& attribute)
{
    if (_attributes == NULL) {
        _attributes = (BleAttribute**)malloc(BleAttribute::numAttributes() * sizeof(BleAttribute*));
    }

    _attributes[_num_attributes] = &attribute;
    _num_attributes++;

    return BLE_STATUS_SUCCESS;
}


BleStatus
BlePeripheral::disconnect()
{
    BleStatus status;

    if (BLE_PERIPH_STATE_CONNECTED == _state)
        status = ble_client_gap_disconnect(BLE_DISCONNECT_REASON_LOCAL_TERMINATION);
    else
        status = BLE_STATUS_WRONG_STATE;

    return status;
}

BleCentral
BlePeripheral::central()
{
    poll();

    return _central;
}

boolean_t
BlePeripheral::connected()
{
    poll();

    return _central;
}

BleStatus
BlePeripheral::_init()
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

    status = ble_client_gap_set_enable_config(_local_name, &_local_bda, _appearance, txPower);
    if (BLE_STATUS_SUCCESS != status) {
        return status;
    }

    /* Populate initial advertising data
     * This may be extended later with Service UUIDs
     */
    _advDataInit();

    _state = BLE_PERIPH_STATE_READY;
    return BLE_STATUS_SUCCESS;
}

void
BlePeripheral::_advDataInit(void)
{
    uint8_t *adv_tmp = _adv_data;

    _adv_data_set = false;
    memset(_adv_data, 0, sizeof(_adv_data));

    /* Add flags */
    *adv_tmp++ = 2;
    *adv_tmp++ = BLE_ADV_TYPE_FLAGS;
    *adv_tmp++ = BLE_SVC_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;
    _adv_data_len = 3;

    if (_appearance) {
        /* Add appearance */
        adv_tmp = &_adv_data[_adv_data_len];
        *adv_tmp++ = 3;
        *adv_tmp++ = BLE_ADV_TYPE_APPEARANCE;
        UINT16_TO_LESTREAM(adv_tmp, _appearance);
        _adv_data_len += 4;
    }

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

BleStatus
BlePeripheral::_startAdvertising()
{
    BleStatus status;

    if (_state != BLE_PERIPH_STATE_READY)
        return BLE_STATUS_WRONG_STATE;

    /* Advertising data can only be set once in this way */
    if (!_adv_data_set) {
        status = ble_client_gap_wr_adv_data(_adv_data, _adv_data_len);
        if (BLE_STATUS_SUCCESS != status)
            return status;
        _adv_data_set = true;
    }

    status = ble_client_gap_start_advertise(0); // 0 = no timeout
    if (BLE_STATUS_SUCCESS != status)
        return status;

    _state = BLE_PERIPH_STATE_ADVERTISING;
    return BLE_STATUS_SUCCESS;
}

BleStatus
BlePeripheral::_stop(void)
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
BlePeripheral::handleGapEvent(ble_client_gap_event_t event, struct ble_gap_event *event_data)
{
    if (BLE_CLIENT_GAP_EVENT_CONNECTED == event) {
        _state = BLE_PERIPH_STATE_CONNECTED;
        _central.setAddress(event_data->connected.peer_bda);

        if (_event_handlers[BleConnected]) {
            _event_handlers[BleConnected](_central);
        }
    } else if (BLE_CLIENT_GAP_EVENT_DISCONNECTED == event) {

        for (int i = 0; i < _num_attributes; i++) {
            BleAttribute* attribute = _attributes[i];

            if (attribute->type() == BleTypeCharacteristic) {
                BleCharacteristic* characteristic = (BleCharacteristic*)attribute;

                characteristic->setCccdValue(_central, 0x0000); // reset CCCD
            }
        }

        if (_event_handlers[BleDisconnected])
            _event_handlers[BleDisconnected](_central);

        _state = BLE_PERIPH_STATE_READY;
        _central.clearAddress();

        _startAdvertising();
    } else if (BLE_CLIENT_GAP_EVENT_CONN_TIMEOUT == event) {
        _state = BLE_PERIPH_STATE_READY;

        _startAdvertising();
    }
}

void BlePeripheral::handleGattsEvent(ble_client_gatts_event_t event, struct ble_gatts_evt_msg *event_data)
{
    if (BLE_CLIENT_GATTS_EVENT_WRITE == event) {
        uint16_t handle = event_data->wr.attr_handle;

        for (int i = 0; i < _num_attributes; i++) {
            BleAttribute* attribute = _attributes[i];

            if (attribute->type() != BleTypeCharacteristic) {
                continue;
            }

            BleCharacteristic* characteristic = (BleCharacteristic*)attribute;

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
