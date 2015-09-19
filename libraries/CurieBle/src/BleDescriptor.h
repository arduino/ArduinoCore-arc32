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
typedef void (*BleDescriptorEventCb)(BleDescriptor &descriptor, BleDescriptorEvent event, void *arg);

/**
 * BLE GATT Descriptor class
 */
class BleDescriptor {
public:
    /**
     * Constructor for BLE Descriptor with 16-bit UUID
     *
     * @param uuid16       16-bit UUID defined by BLE standard
     * @param maxLength    Maximum data length required for descriptor value (<= BLE_MAX_ATTR_DATA_LEN)
     * @param clientAccess Access permissions for remote client
     */
    BleDescriptor(const uint16_t uuid16,
                  const uint16_t maxLength,
                  const BleClientAccessMode clientAccess);

    /**
     * Constructor for BLE Descriptor with 128-bit UUID
     *
     * @param uuid128      128-bit custom-defined UUID
     * @param maxLength    Maximum data length required for descriptor value (<= BLE_MAX_ATTR_DATA_LEN)
     * @param clientAccess Access permissions for remote client
     */
    BleDescriptor(const uint8_t uuid128[],
                  const uint16_t maxLength,
                  const BleClientAccessMode clientAccess);

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

    /* Alternative methods to set descriptor value */
    BleStatus setValue(const String &str);
    BleStatus setValue(const char *cstr);
    BleStatus setValue(const char &value);
    BleStatus setValue(const unsigned char &value);
    BleStatus setValue(const short &value);
    BleStatus setValue(const unsigned short &value);
    BleStatus setValue(const int &value);
    BleStatus setValue(const unsigned int &value);
    BleStatus setValue(const long &value);
    BleStatus setValue(const unsigned long &value);

    /**
     * Get the current value of the Descriptor
     *
     * @param value  Current value, as a byte array.  Data is read from internal copy.
     * @param length Length, in bytes, of valid data read into the array.
     *
     * @return BleStatus indicating success or error
     */
    BleStatus getValue(uint8_t value[], uint16_t &length) const;

    /* Alternative methods to get descriptor value */
    BleStatus getValue(String &str) const; /* WARNING - assumes characteristic value is a null-terminated string */
    BleStatus getValue(char *cstr) const;  /* WARNING - ensure cstr buffer size is big enough */
    BleStatus getValue(char &value) const;
    BleStatus getValue(unsigned char &value) const;
    BleStatus getValue(short &value) const;
    BleStatus getValue(unsigned short &value) const;
    BleStatus getValue(int &value) const;
    BleStatus getValue(unsigned int &value) const;
    BleStatus getValue(long &value) const;
    BleStatus getValue(unsigned long &value) const;

    /**
     * Provide a function to be called when events related to this Descriptor are raised
     *
     * @param callback Pointer to callback function to invoke when an event occurs, or NULL to disable.
     * @param arg      [Optional] Opaque argument which will be passed in the callback.
     */
    void setEventCallback(BleDescriptorEventCb callback, void *arg = NULL);

private:
    friend class BleCharacteristic;
    friend void blePeripheralGattsEventHandler(ble_client_gatts_event_t event, struct ble_gatts_evt_msg *event_data, void *param);

    BleDescriptor(const uint16_t maxLength,
                  const BleClientAccessMode clientAccess);

    BleStatus _setValue(void);
    void      _setConnectedState(boolean_t connected);

    struct ble_gatts_descriptor _desc;
    struct bt_uuid              _uuid;
    uint16_t                    _handle;
    uint16_t                    _char_handle;
    boolean_t                   _initialised;
    boolean_t                   _connected;
    BleDescriptorEventCb        _event_cb;
    void                        *_event_cb_arg;

    uint8_t _data[BLE_MAX_ATTR_DATA_LEN];
};

#endif // _BLE_DESCRIPTOR_H_INCLUDED
