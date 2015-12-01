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

#ifndef _BLE_DESCRIPTOR_H_INCLUDED
#define _BLE_DESCRIPTOR_H_INCLUDED

#include "BleAttribute.h"
#include "BleCentral.h"
#include "BleCommon.h"

#include "internal/ble_client.h"

/**
 * BLE Descriptor Events
 */
enum BleDescriptorEvent {
    BLE_DESC_EVENT_WRITE = 0,
};

/* Forward declaration needed for callback function prototype below */
class BleCharacteristic;
class BleDescriptor;

/** Function prototype for BLE Characteristic event callback */
typedef void (*BleDescriptorEventCb)(BleCentral& central, BleDescriptor &descriptor, BleDescriptorEvent event, void *arg);

/**
 * BLE GATT Descriptor class
 */
class BleDescriptor : public BleAttribute {
public:
    BleDescriptor(const char* uuid, const uint8_t value[], uint16_t valueLength);
    BleDescriptor(const char* uuid, const char* value);

    uint16_t valueSize() const;
    const uint8_t* value() const;
    uint16_t valueLength() const;
    uint8_t operator[] (int offset) const;

protected:
    /**
     * Set the current value of the Descriptor
     *
     * @param value  New value to set, as a byte array.  Data is stored in internal copy.
     * @param length Length, in bytes, of valid data in the array to write.
     *               Must not exceed BLE_MAX_ATTR_DATA_LEN.
     *
     * @return BleStatus indicating success or error
     */
    BleStatus setValue(const uint8_t value[],
                       const uint16_t length);

    /**
     * Provide a function to be called when events related to this Descriptor are raised
     *
     * @param callback Pointer to callback function to invoke when an event occurs, or NULL to disable.
     * @param arg      [Optional] Opaque argument which will be passed in the callback.
     */
    void setEventCallback(BleDescriptorEventCb callback, void *arg = NULL);

    /**
     * Constructor for BLE Descriptor with 16-bit UUID
     *
     * @param uuid16       16-bit UUID defined by BLE standard
     * @param maxLength    Maximum data length required for descriptor value (<= BLE_MAX_ATTR_DATA_LEN)
     * @param clientAccess Access permissions for remote client
     */
    BleDescriptor(const char* uuid,
                  const uint16_t maxLength,
                  const BleClientAccessMode clientAccess);

    friend class BlePeripheral;
    friend class BleCharacteristic;

private:
    BleDescriptor(const uint16_t maxLength,
                  const BleClientAccessMode clientAccess);

    BleStatus _setValue(void);

    struct ble_gatts_descriptor _desc;
    uint16_t                    _handle;
    boolean_t                   _initialised;
    BleDescriptorEventCb        _event_cb;
    void                        *_event_cb_arg;

    uint8_t _data[BLE_MAX_ATTR_DATA_LEN];
};

#endif // _BLE_DESCRIPTOR_H_INCLUDED
