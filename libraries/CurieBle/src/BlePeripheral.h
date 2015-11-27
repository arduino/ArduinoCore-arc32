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

#ifndef _BLE_PERIPHERAL_H_INCLUDED
#define _BLE_PERIPHERAL_H_INCLUDED

#include "BleAddress.h"
#include "BleCentral.h"
#include "BleService.h"

/**
 * BLE Peripheral Device State
 */
enum BlePeripheralState {
    BLE_PERIPH_STATE_NOT_READY = 0,
    BLE_PERIPH_STATE_READY,
    BLE_PERIPH_STATE_ADVERTISING,
    BLE_PERIPH_STATE_CONNECTED,
};

/**
 * BLE Peripheral Device Events
 */
enum BlePeripheralEvent {
    BLE_PERIPH_EVENT_CONNECTED = 0,
    BLE_PERIPH_EVENT_DISCONNECTED,
    BLE_PERIPH_EVENT_ADV_TIMEOUT,
    BLE_PERIPH_EVENT_CONN_TIMEOUT,
    BLE_PERIPH_EVENT_RSSI_CHANGE, /* NOTE: Currently unsupported */
};

/* Forward declaration needed for callback function prototype below */
class BlePeripheral;

/** Function prototype for BLE Peripheral Device event callback */
typedef void (*BlePeripheralEventCb)(BlePeripheral &peripheral, BlePeripheralEvent event, void *arg);

/* TODO - consider splitting this into a BleDevice base class, and derive
 * BlePeripheral and BleCentral classes from it.  For now, we only support Peripheral mode
 */
/**
 * BLE Peripheral Device
 */
class BlePeripheral {
public:
    /**
     * Default Constructor for BLE Peripheral Device
     */
    BlePeripheral(void);

    int begin();
    void poll();
    void end();

    BleStatus setAdvertisedServiceUuid(struct bt_uuid advertisedServiceUuid);

    /**
     * Set the broadcast name for the BLE Peripheral Device
     *
     * If broadcast name is not set, a default name will be used instead
     *
     * @param localName  User-defined name string for this device.  Truncated if
     *                   more than maximum allowed string length (20 bytes).
     *
     * @note This method must be called before the init method
     */
    BleStatus setLocalName(const char *localName);

    /**
     * Get the current broadcast name for the BLE Peripheral Device
     *
     * @param localName  Array to be filled with a copy of the name string.
     *                   Array size must be at least 20 bytes.
     */
    void getLocalName(char localName[]) const;

    /**
     * Set the broadcast appearance type for the BLE Peripheral Device
     *
     * See https://developer.bluetooth.org/gatt/characteristics/Pages/CharacteristicViewer.aspx?u=org.bluetooth.characteristic.gap.appearance.xml
     * for available options.  If not set, no appearance type will be specified
     * in broadcast information.
     *
     * @param appearance Appearance category identifier as defined by BLE Standard
     *
     * @return BleStatus indicating success or error
     *
     * @note This method must be called before the init method
     */
    BleStatus setAppearance(const uint16_t appearance);

    /**
     * Get the current apperance type for the BLE Peripheral Device
     *
     * @param appearance Appearance category identifier currently set for this device
     */
    void getAppearance(uint16_t &appearance) const;

    BleStatus disconnect();

    BleCentral central();
    bool connected();

    /**
     * Initialise the BLE Peripheral Device
     *
     * @param name       [Optional] User-defined name for this device.  This defaults
     *                   to "Arduino101nnnn" where nnnn will be substituted with the
     *                   last 4 digits of the Bluetooth MAC address assigned for this
     *                   device.
     * @param appearance [Optional] Appearance 16-bit UUID from BLE Standard
     * @param txPower    [Optional] Transmit Power in dB. Defaults to maximum +127 dB
     *
     * @return BleStatus indicating success or error
     *
     * @note This method must be called only once, and before any other methods
     *       in this class (except setName or setAppearance)
     */
    BleStatus init(int8_t txPower = 127);

    /**
     * Add a BLE Primary Service for this Device
     *
     * @param service   BLE Primary Service object reference
     *
     * @return BleStatus indicating success or error
     *
     * @note This method must be called only before advertising is started.
     */
    BleStatus addPrimaryService(BleService &service_uuid);

    /**
     * Get the current state of this Device
     *
     * @param state Current state of this Device
     *
     * @return BleStatus indicating success or error
     */
    BleStatus getState(BlePeripheralState &state) const;

    /**
     * Get the current Bluetooth Device Address (MAC) of this Device
     *
     * @param address Current Bluetooth Device Address
     *
     * @return BleStatus indicating success or error
     */
    BleStatus getLocalAddress(BleDeviceAddress &address) const;

    /**
     * Get the Bluetooth Device Address (MAC) of the connected peer (if in the connected state)
     *
     * @param address Current Bluetooth Device Address
     *
     * @return BleStatus indicating success or error
     */
    BleStatus getPeerAddress(BleDeviceAddress &address) const;

    /**
     * Provide a function to be called when events related to this Device are raised
     *
     * @param callback Pointer to callback function to invoke when an event occurs.
     * @param arg      [Optional] Opaque argument which will be passed in the callback.
     */
    void setEventCallback(BlePeripheralEventCb callback, void *arg = NULL);

private:
    /**
     * Start advertising / accept connections
     *
     * @param advTimeout  Advertising timeout (0 = no timeout)
     * @param autoRestart Re-start advertising automatically (unless stop() is called)
     *
     * @return BleStatus indicating success or error
     */
    BleStatus start(uint16_t advTimeout = 0, boolean_t autoRestart = true);

    /**
     * Stop advertising / disconnect
     *
     * @return BleStatus indicating success or error
     */
    BleStatus stop(void);

    friend void blePeripheralGapEventHandler(ble_client_gap_event_t event, struct ble_gap_event *event_data, void *param);
    friend void blePeripheralGattsEventHandler(ble_client_gatts_event_t event, struct ble_gatts_evt_msg *event_data, void *param);

    BleService        *_matchService(uint16_t svc_handle) const;
    BleCharacteristic *_matchCharacteristic(uint16_t handle) const;
    BleDescriptor     *_matchDescriptor(uint16_t handle) const;
    void              _setConnectedState(boolean_t connected);

    void _advDataInit(void);

    BlePeripheralState   _state;

    boolean_t  _initialised;
    char       _localName[BLE_MAX_DEVICE_NAME+1];
    uint16_t   _appearance;
    uint8_t    _adv_data[BLE_MAX_ADV_SIZE];
    uint8_t    _adv_data_len;
    boolean_t  _adv_data_set;
    boolean_t  _adv_auto_restart;
    ble_addr_t _local_bda;
    ble_addr_t _peer_bda;
    boolean_t  _connected;
    BleCentral _central;

    BlePeripheralEventCb _event_cb;
    void                *_event_cb_arg;

    BleService *_services[BLE_MAX_PRIMARY_SERVICES];
    uint32_t    _num_services;
};

#endif // _BLE_DEVICE_H_INCLUDED
