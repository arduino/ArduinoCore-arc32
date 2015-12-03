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

#ifndef _BLE_CHARACTERISTIC_H_INCLUDED
#define _BLE_CHARACTERISTIC_H_INCLUDED

#include "BleAttribute.h"
#include "BleCentral.h"

/**
 * BLE Characteristic Events
 */
enum BleCharacteristicEvent {
    BleWritten = 0,
    BleSubscribed = 1,
    BleUnsubscribed = 2,

    BleCharacteristicEventLast = 3
};

/* Forward declaration needed for callback function prototype below */
class BleCharacteristic;
class BlePeripheral;

/** Function prototype for BLE Characteristic event callback */
typedef void (*BleCharacteristicEventHandler)(BleCentral &central, BleCharacteristic &characteristic);

/**
 * BLE Characteristic Property types
 */
enum BleProperty {
    // broadcast (0x01) not supported
    BleRead                 = 0x02,
    BleWriteWithoutResponse = 0x04,
    BleWrite                = 0x08,
    BleNotify               = 0x10,
    BleIndicate             = 0x20
};

/**
 * BLE GATT Characteristic
 */
class BleCharacteristic : public BleAttribute {
public:
    /**
     * Constructor for BLE Characteristic
     *
     * @param uuid         16-bit or 128-bit UUID (in string form) defined by BLE standard
     * @param properties   Characteristic property mask
     * @param maxLength    Maximum data length required for characteristic value (<= BLE_MAX_ATTR_DATA_LEN)
     */
    BleCharacteristic(const char* uuid,
                      const uint8_t properties,
                      const uint16_t maxLength);

    /**
     * Constructor for BLE Characteristic
     *
     * @param uuid         16-bit or 128-bit UUID (in string form) defined by BLE standard
     * @param properties   Characteristic property mask
     * @param value        String value for characteristic (string length (<= BLE_MAX_ATTR_DATA_LEN))
     */
    BleCharacteristic(const char* uuid,
                      const uint8_t properties,
                      const char* value);

    /**
     * Set the current value of the Characteristic
     *
     * @param value  New value to set, as a byte array.  Data is stored in internal copy.
     * @param length Length, in bytes, of valid data in the array to write.
     *               Must not exceed maxLength set for this characteristic.
     *
     * @return BleStatus indicating success or error
     */
    BleStatus setValue(const uint8_t value[], uint16_t length);

    /**
     * Get the property mask of the Characteristic
     *
     * @return uint8_t property mask of the Characteristic
     */
    uint8_t properties(void) const;

    /**
     * Get the size of the Characteristic
     *
     * @return uint16_t size of characateristic in bytes
     */
    uint16_t valueSize(void) const;

    /**
     * Get data pointer to the value of the Characteristic
     *
     * @return const uint8_t* pointer to the value of the Characteristic
     */
    const uint8_t* value(void) const;

    /**
     * Get the length of the value of the Characteristic
     *
     * @return uint16_t size of characateristic value in bytes
     */
    uint16_t valueLength() const;

    uint8_t operator[] (int offset) const;

    /**
     * Has the value of the Characteristic been written by a central
     *
     * @return boolean_t true is central has updated characteristic value, otherwise false
     */
    boolean_t written(void);

    /**
     * Is a central listening for notifications or indications of the Characteristic
     *
     * @return boolean_t true is central is subscribed, otherwise false
     */
    boolean_t subscribed(void);

    /**
     * Provide a function to be called when events related to this Characteristic are raised
     *
     * @param event Event type to set event handler for
     * @param callback  Pointer to callback function to invoke when the event occurs.
     */
    void setEventHandler(BleCharacteristicEvent event, BleCharacteristicEventHandler callback);

protected:
    BleStatus add(uint16_t serviceHandle);

    uint16_t valueHandle(void);

    uint16_t cccdHandle(void);

    void setValue(BleCentral& central, const uint8_t value[], uint16_t length);
    void setCccdValue(BleCentral& central, uint16_t value);

    friend class BlePeripheral;

private:
    void _setValue(const uint8_t value[], uint16_t length);

private:
    uint8_t _properties;
    uint16_t _max_len;
    uint8_t  _data[BLE_MAX_ATTR_DATA_LEN];
    uint16_t _data_len;
    boolean_t _written;
    uint16_t _cccd_value;
    uint16_t _value_handle;
    uint16_t _cccd_handle;
    
    BleCharacteristicEventHandler _event_handlers[BleCharacteristicEventLast];
};

#endif // _BLE_CHARACTERISTIC_H_INCLUDED
