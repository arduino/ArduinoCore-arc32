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

#ifndef _BLE_DEVICE_H_INCLUDED
#define _BLE_DEVICE_H_INCLUDED

#include "BleService.h"

/**
 * BLE GAP supported address types.
 */
enum BleDeviceAddressType {
    BLE_DEVICE_ADDR_PUBLIC = 0,
    BLE_DEVICE_ADDR_PRIVATE_RANDOM_STATIC,
    BLE_DEVICE_ADDR_RRIVATE_RANDOM_PRIVATE_RESOLVABLE,
    BLE_DEVICE_ADDR_PRIVATE_RANDOM_PRIVATE_NONRESOLVABLE
};

/**
 * BT/BLE Device Address Length.
 */
#define BLE_DEVICE_ADDR_LEN 6

/**
 * BT/BLE Device Address
 */
typedef struct _BleDeviceAddress {
	BleDeviceAddressType type;         /**< BLE Address type */
	uint8_t addr[BLE_DEVICE_ADDR_LEN]; /**< BT Mac address, little endian format */
} BleDeviceAddress;

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
     * Constructor for BLE Peripheral Device
     *
     * @param name User-defined name for this device
     * @param appearance [Optional] Appearance 16-bit UUID defined by BLE Standard
     */
    BlePeripheral(const char *name, const uint16_t appearance = 0);

    /**
     * Initialise the BLE Peripheral Device
     *
     * @param txPower Can be optionally substituted with calibrated RSSI, if known
     *
     * @note This method must be called before any other methods in this class
     */
    BleStatus init(int8_t txPower = 127);

    /**
     * Add a BLE Primary Service for this Device
     *
     * @param service BLE Primary Service reference
     * @param advertise If true, this service will be included in BLE Advertising
     *                  packets if sufficient space is available
     *
     * @return BleStatus indicating success or error
     *
     * @note This method must be called only before advertising is started.
     */
    BleStatus addPrimaryService(BleService &service, boolean_t advertise);

    /**
     * Get the current state of this Device
     *
     * @param state Current state of this Device
     *
     * @return BleStatus indicating success or error
     */
    BleStatus getState(BlePeripheralState &state);

    /**
     * Get the current Bluetooth Device Address (MAC) of this Device
     *
     * @param address Current Bluetooth Device Address
     *
     * @return BleStatus indicating success or error
     */
    BleStatus getLocalAddress(BleDeviceAddress &address);

    /**
     * Get the Bluetooth Device Address (MAC) of the connected peer (if in the connected state)
     *
     * @param address Current Bluetooth Device Address
     *
     * @return BleStatus indicating success or error
     */
    BleStatus getPeerAddress(BleDeviceAddress &address);

    /**
     * Provide a function to be called when events related to this Device are raised
     *
     * @param callback Pointer to callback function to invoke when an event occurs.
     * @param arg      [Optional] Opaque argument which will be passed in the callback.
     *
     * @return BleStatus indicating success or error
     */
    BleStatus setEventCallback(BlePeripheralEventCb callback, void *arg = NULL);

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

private:
    friend void blePeripheralGapEventHandler(ble_client_gap_event_t event, struct ble_gap_event *event_data, void *param);
    friend void blePeripheralGattsEventHandler(ble_client_gatts_event_t event, struct ble_gatts_evt_msg *event_data, void *param);

    BleService        *_matchService(uint16_t svc_handle);
    BleCharacteristic *_matchCharacteristic(uint16_t handle);
    BleDescriptor     *_matchDescriptor(uint16_t handle);

    void _advDataInit(void);

    BlePeripheralState   _state;

    boolean_t  _initialised;
    char       _name[BLE_MAX_DEVICE_NAME];
    uint16_t   _appearance;
    uint16_t   _manuf_id;
    uint8_t    _adv_data[BLE_MAX_ADV_SIZE];
    uint8_t    _adv_data_len;
    boolean_t  _adv_data_set;
    boolean_t  _adv_auto_restart;
    ble_addr_t _peer_bda;

    BlePeripheralEventCb _event_cb;
    void                *_event_cb_arg;

    BleService *_services[BLE_MAX_PRIMARY_SERVICES];
    uint32_t    _num_services;
};

#endif // _BLE_DEVICE_H_INCLUDED
