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

#include "internal/ble_client.h"

#include "BLEAttribute.h"
#include "BLECentral.h"
#include "BLECharacteristic.h"
#include "BLECommon.h"

/**
 * BLE Peripheral Events
 */
enum BLEPeripheralEvent {
  BLEConnected = 0,
  BLEDisconnected = 1,

  BLEPeripheralEventLast = 2
};

/** Function prototype for BLE Peripheral Device event callback */
typedef void (*BLEPeripheralEventHandler)(BLECentral &central);

/**
 * BLE Peripheral
 */
class BLEPeripheral {
public:
    /**
     * Default Constructor for BLE Peripheral Device
     */
    BLEPeripheral(void);

    /**
     * Destructor for BLE Peripheral Device
     */
    virtual ~BLEPeripheral(void);

    /**
     * Set the service UUID that the BLE Peripheral Device advertises
     *
     * @param advertisedServiceUuid  16-bit or 128-bit UUID to advertis
     *                               (in string form)
     *
     * @note This method must be called before the begin method
     */
    void setAdvertisedServiceUuid(const char* advertisedServiceUuid);

    /**
     * Set the local name that the BLE Peripheral Device advertises
     *
     * @param localName  local name to advertise
     *
     * @note This method must be called before the begin method
     */
    void setLocalName(const char* localName);

    /**
     * Set the device name for the BLE Peripheral Device
     *
     * If device name is not set, a default name will be used instead
     *
     * @param device  User-defined name string for this device.  Truncated if
     *                   more than maximum allowed string length (20 bytes).
     *
     * @note This method must be called before the begin method
     */
    void setDeviceName(const char *deviceName);

    /**
     * Set the appearance type for the BLE Peripheral Device
     *
     * See https://developer.bluetooth.org/gatt/characteristics/Pages/CharacteristicViewer.aspx?u=org.bluetooth.characteristic.gap.appearance.xml
     * for available options.
     *
     * @param appearance Appearance category identifier as defined by BLE Standard
     *
     * @return BleStatus indicating success or error
     *
     * @note This method must be called before the begin method
     */
    void setAppearance(const unsigned short appearance);

    /**
     * Add an attribute to the BLE Peripheral Device
     *
     * @param attribute Attribute to add to Peripheral
     *
     * @return BleStatus indicating success or error
     *
     * @note This method must be called before the begin method
     */
    void addAttribute(BLEAttribute& attribute);

    /**
     * Provide a function to be called when events related to this Device are raised
     *
     * @param event    Event type for callback
     * @param callback Pointer to callback function to invoke when an event occurs.
     */
    void setEventHandler(BLEPeripheralEvent event, BLEPeripheralEventHandler callback);

    /**
     * Setup attributes and start advertising
     *
     * @return bool indicating success or error
     */
    bool begin(void);

    /**
     * Poll the peripheral for events
     */
    void poll(void);

    /**
     * Stop advertising and disconnect a central if connected
     */
    void end(void);

    /**
     * Disconnect the central connected if there is one connected
     *
     * @return bool indicating success or error
     */
    bool disconnect(void);

    /**
     * Setup attributes and start advertising
     *
     * @return BleStatus indicating success or error
     */
    BLECentral central(void);

    /**
     * Is a central connected?
     *
     * @return boolean_t true if central connected, otherwise false
     */
    bool connected(void);

protected:
    friend void blePeripheralGapEventHandler(ble_client_gap_event_t event, struct ble_gap_event *event_data, void *param);
    friend void blePeripheralGattsEventHandler(ble_client_gatts_event_t event, struct ble_gatts_evt_msg *event_data, void *param);

    void handleGapEvent(ble_client_gap_event_t event, struct ble_gap_event *event_data);
    void handleGattsEvent(ble_client_gatts_event_t event, struct ble_gatts_evt_msg *event_data);

private:
    BleStatus _init(void);
    BleStatus _startAdvertising(void);
    BleStatus _stop(void);

    void _advDataInit(void);

private:

    enum BLEPeripheralState {
        BLE_PERIPH_STATE_NOT_READY = 0,
        BLE_PERIPH_STATE_READY,
        BLE_PERIPH_STATE_ADVERTISING,
        BLE_PERIPH_STATE_CONNECTED,
    };

    BLEPeripheralState _state;

    const char* _advertise_service_uuid;
    const char* _local_name;
    char       _device_name[BLE_MAX_DEVICE_NAME+1];
    uint16_t   _appearance;
    uint8_t    _adv_data[BLE_MAX_ADV_SIZE];
    uint8_t    _adv_data_len;
    ble_addr_t _local_bda;
    BLECentral _central;

    BLEPeripheralEventHandler _event_handlers[BLEPeripheralEventLast];

    BLEAttribute** _attributes;
    uint16_t _num_attributes;

    BLECharacteristic* _last_added_characteritic;
};

#endif // _BLE_DEVICE_H_INCLUDED
