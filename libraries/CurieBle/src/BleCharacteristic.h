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

#include "BleCommon.h"
#include "BleDescriptor.h"

#include "internal/ble_client.h"

/**
 * BLE Characteristic Events
 */
enum BleCharacteristicEvent {
    BLE_CHAR_EVENT_WRITE = 0,
    BLE_CHAR_EVENT_INDICATION_ACK,
};

/* Forward declaration needed for callback function prototype below */
class BleCharacteristic;
class BleService;
class BlePeripheral;

/** Function prototype for BLE Characteristic event callback */
typedef void (*BleCharacteristicEventCb)(BleCharacteristic &characteristic, BleCharacteristicEvent event, void *arg);

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
class BleCharacteristic {
public:
    /**
     * Constructor for BLE Characteristic with 16-bit UUID
     *
     * @param uuid16       16-bit UUID defined by BLE standard
     * @param maxLength    Maximum data length required for characteristic value (<= BLE_MAX_ATTR_DATA_LEN)
     * @param clientAccess Access permissions for remote client
     * @param clientNotify Notification mode to alert remote client when characteristic value is updated (@ref setValue)
     */
    BleCharacteristic(const uint16_t uuid16,
                      const uint16_t maxLength,
                      const uint8_t properties);

    /**
     * Constructor for BLE Characteristic with 128-bit UUID
     *
     * @param uuid128      128-bit custom-defined UUID
     * @param maxLength    Maximum data length required for characteristic value (<= BLE_MAX_ATTR_DATA_LEN)
     * @param clientAccess Access permissions for remote client
     * @param clientNotify Notification mode to alert remote client when characteristic value is updated (@ref setValue)
     */
    BleCharacteristic(const uint8_t uuid128[],
                      const uint16_t maxLength,
                      const uint8_t properties);

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
    BleStatus setValue(const unsigned char &value);
    BleStatus setValue(const short &value);
    BleStatus setValue(const unsigned short &value);
    BleStatus setValue(const int &value);
    BleStatus setValue(const unsigned int &value);
    BleStatus setValue(const long &value);
    BleStatus setValue(const unsigned long &value);

    /**
     * Get the current value of the Characteristic
     *
     * @param value  Current value, as a byte array.  Data is read from internal copy.
     * @param length Length, in bytes, of valid data read into the array.
     *
     * @return BleStatus indicating success or error
     */
    BleStatus getValue(uint8_t value[], uint16_t &length) const;

    /* Alternative methods to get characteristic value */
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
     * Provide a function to be called when events related to this Characteristic are raised
     *
     * @param callback Pointer to callback function to invoke when an event occurs.
     * @param arg      [Optional] Opaque argument which will be passed in the callback.
     */
    void setEventCallback(BleCharacteristicEventCb callback, void *arg = NULL);

private:
    friend class BlePeripheral;
    friend class BleService;
    friend void  blePeripheralGattsEventHandler(ble_client_gatts_event_t event, struct ble_gatts_evt_msg *event_data, void *param);
    friend void _cccdEventHandler(BleDescriptor &cccd, BleDescriptorEvent event, void *arg);

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
    uint8_t                  _properties;
    BleCharacteristicEventCb _event_cb;
    void                     *_event_cb_arg;

    uint16_t                        _svc_handle;
    struct bt_uuid                  _uuid;
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
