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

#include "BLEAttribute.h"
#include "BLECentral.h"
#include "BLEDescriptor.h"

/**
 * BLE Characteristic Events
 */
enum BLECharacteristicEvent {
    BLEWritten = 0,
    BLESubscribed = 1,
    BLEUnsubscribed = 2,

    BLECharacteristicEventLast = 3
};

/* Forward declaration needed for callback function prototype below */
class BLECharacteristic;
class BLEPeripheral;

/** Function prototype for BLE Characteristic event callback */
typedef void (*BLECharacteristicEventHandler)(BLECentral &central, BLECharacteristic &characteristic);

/**
 * BLE Characteristic Property types
 */
enum BLEProperty {
    // broadcast (0x01) not supported
    BLERead                 = 0x02,
    BLEWriteWithoutResponse = 0x04,
    BLEWrite                = 0x08,
    BLENotify               = 0x10,
    BLEIndicate             = 0x20
};

/**
 * BLE GATT Characteristic
 */
class BLECharacteristic : public BLEAttribute {
public:
    /**
     * Constructor for BLE Characteristic
     *
     * @param uuid         16-bit or 128-bit UUID (in string form) defined by BLE standard
     * @param properties   Characteristic property mask
     * @param maxLength    Maximum data length required for characteristic value (<= BLE_MAX_ATTR_DATA_LEN)
     */
    BLECharacteristic(const char* uuid,
                      const unsigned char properties,
                      const unsigned short maxLength);

    /**
     * Constructor for BLE Characteristic
     *
     * @param uuid         16-bit or 128-bit UUID (in string form) defined by BLE standard
     * @param properties   Characteristic property mask
     * @param value        String value for characteristic (string length (<= BLE_MAX_ATTR_DATA_LEN))
     */
    BLECharacteristic(const char* uuid,
                      const unsigned char properties,
                      const char* value);

    virtual ~BLECharacteristic();

    /**
     * Set the current value of the Characteristic
     *
     * @param value  New value to set, as a byte array.  Data is stored in internal copy.
     * @param length Length, in bytes, of valid data in the array to write.
     *               Must not exceed maxLength set for this characteristic.
     *
     * @return bool true set value success, false on error
     */
    bool setValue(const unsigned char value[], unsigned short length);

    /**
     * Get the property mask of the Characteristic
     *
     * @return unsigned char property mask of the Characteristic
     */
    unsigned char properties(void) const;

    /**
     * Get the (maximum) size of the Characteristic
     *
     * @return unsigned size of characateristic in bytes
     */
    unsigned short valueSize(void) const;

    /**
     * Get data pointer to the value of the Characteristic
     *
     * @return const unsigned char* pointer to the value of the Characteristic
     */
    const unsigned char* value(void) const;

    /**
     * Get the current length of the value of the Characteristic
     *
     * @return unsigned short size of characateristic value in bytes
     */
    unsigned short valueLength() const;

    unsigned char operator[] (int offset) const;

    /**
     * Has the value of the Characteristic been written by a central
     *
     * @return bool true is central has updated characteristic value, otherwise false
     */
    bool written(void);

    /**
     * Is a central listening for notifications or indications of the Characteristic
     *
     * @return bool true is central is subscribed, otherwise false
     */
    bool subscribed(void);

    /**
     * Provide a function to be called when events related to this Characteristic are raised
     *
     * @param event Event type to set event handler for
     * @param callback  Pointer to callback function to invoke when the event occurs.
     */
    void setEventHandler(BLECharacteristicEvent event, BLECharacteristicEventHandler callback);

protected:
    bool add(uint16_t serviceHandle);

    uint16_t valueHandle(void);

    uint16_t cccdHandle(void);

    void setValue(BLECentral& central, const uint8_t value[], uint16_t length);
    void setCccdValue(BLECentral& central, uint16_t value);

    void setUserDescription(BLEDescriptor *descriptor);
    void setPresentationFormat(BLEDescriptor *descriptor);

    friend class BLEPeripheral;

private:
    void _setValue(const uint8_t value[], uint16_t length);

private:
    unsigned char _properties;
    unsigned short _value_size;
    unsigned short _value_length;
    unsigned char* _value;
    bool _written;

    uint16_t _cccd_value;
    uint16_t _value_handle;
    uint16_t _cccd_handle;

    BLEDescriptor* _user_description;
    BLEDescriptor* _presentation_format;
    
    BLECharacteristicEventHandler _event_handlers[BLECharacteristicEventLast];
};

#endif // _BLE_CHARACTERISTIC_H_INCLUDED
