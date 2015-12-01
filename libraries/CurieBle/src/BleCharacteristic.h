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
#include "BleCommon.h"
#include "BleDescriptor.h"

#include "internal/ble_client.h"

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
class BleService;
class BlePeripheral;

/** Function prototype for BLE Characteristic event callback */
typedef void (*BleCharacteristicEventHandler)(BleCentral &central, BleCharacteristic &characteristic);

enum BleProperty {
  // BleBroadcast            = 0x01,
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
     * @param uuid         UUID (string) defined by BLE standard
     * @param properties   Characteristic property mask
     * @param maxLength    Maximum data length required for characteristic value (<= BLE_MAX_ATTR_DATA_LEN)
     */
    BleCharacteristic(const char* uuid,
                      const uint8_t properties,
                      const uint16_t maxLength);

    /**
     * Constructor for BLE Characteristic
     *
     * @param uuid         UUID (string) defined by BLE standard
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
    BleStatus setValue(const uint8_t value[], const uint16_t length);

    /* Alternative methods to set characteristic value based on data type */
    BleStatus setValue(const String &str);
    BleStatus setValue(const char *cstr);
    BleStatus setValue(const char &value);

    uint16_t valueSize() const;
    const uint8_t* value() const;
    uint16_t valueLength() const;
    uint8_t operator[] (int offset) const;

    boolean_t written();
    boolean_t subscribed();

    /**
     * Provide a function to be called when events related to this Characteristic are raised
     *
     * @param callback Pointer to callback function to invoke when an event occurs.
     * @param arg      [Optional] Opaque argument which will be passed in the callback.
     */
    void setEventHandler(BleCharacteristicEvent event, BleCharacteristicEventHandler callback);

protected:
    /**
     * Add an optional User-Description descriptor
     *
     * Refer to https://developer.bluetooth.org for further information
     * on this standard GATT Descriptor (UUID 0x2901)
     * https://developer.bluetooth.org/gatt/descriptors/Pages/DescriptorViewer.aspx?u=org.bluetooth.descriptor.gatt.characteristic_user_description.xml
     *
     * @param description User-Description string
     *
     * @return BleStatus indicating success or error
     *
     * @note This method must only be called BEFORE the characteristic is added to a service
     */
    BleStatus addUserDescription(const char *description);

    /**
     * Add an optional Presentation-Format descriptor
     *
     * Refer to https://developer.bluetooth.org for further information
     * on this standard GATT Descriptor (UUID 0x2904)
     * https://developer.bluetooth.org/gatt/descriptors/Pages/DescriptorViewer.aspx?u=org.bluetooth.descriptor.gatt.characteristic_presentation_format.xml
     *
     * @param format      Refer to Bluetooth SIG documentation for more information
     * @param exponent    Refer to Bluetooth SIG documentation for more information
     * @param unit        Refer to Bluetooth SIG documentation for more information
     * @param nameSpace   Refer to Bluetooth SIG documentation for more information
     * @param description Refer to Bluetooth SIG documentation for more information
     *
     * @return BleStatus indicating success or error
     *
     * @note This method must only be called BEFORE the characteristic is added to a service
     */
    BleStatus addPresentationFormat(const uint8_t  format,
                                    const int8_t   exponent,
                                    const uint16_t unit,
                                    const uint8_t  nameSpace,
                                    const uint16_t description);

    /**
     * Add a BLE Descriptor for this Characteristic
     *
     * @param descriptor BLE Descriptor reference
     *
     * @return BleStatus indicating success or error
     *
     * @note This method must be called only before the characteristic is added to a service
     */
    BleStatus addDescriptor(BleDescriptor &descriptor);

    friend void _cccdEventHandler(BleCentral &central, BleDescriptor &cccd, BleDescriptorEvent event, void *arg);

    void onCccdWrite(BleCentral& central, uint16_t value);
    void setValue(BleCentral& central, const uint8_t value[], uint16_t length);

    friend class BlePeripheral;
    friend class BleService;

private:
    BleCharacteristic(const uint16_t maxLength,
                      const uint8_t properties);

    BleDescriptor *_matchDescriptor(uint16_t handle) const;
    BleStatus      _setValue(void);
    void           _addCCCDescriptor(void);
    void           _cccdEvent(BleDescriptor &cccd,
                              BleDescriptorEvent event,
                              void *arg);
    void          _setConnectedState(boolean_t connected);

    boolean_t                _initialised;
    boolean_t                _connected;
    boolean_t                _written;
    uint8_t                  _properties;
    BleCharacteristicEventHandler _event_handlers[BleCharacteristicEventLast];

    uint16_t                        _svc_handle;
    struct ble_gatts_characteristic _char_data;
    struct ble_gatts_char_handles   _handles;

    BleDescriptor _cccd;
    boolean_t     _notifyEnabled;
    boolean_t     _indicateEnabled;

    uint8_t  _data[BLE_MAX_ATTR_DATA_LEN];
    uint16_t _data_len;

    uint8_t _user_desc_data[BLE_MAX_ATTR_DATA_LEN];
    struct ble_gatt_char_user_desc _user_desc;
    struct ble_gatt_pf_desc _pf_desc;

    BleDescriptor *_descriptors[BLE_MAX_DESCRIPTORS];
    uint32_t       _num_descriptors;
};

#endif // _BLE_CHARACTERISTIC_H_INCLUDED
