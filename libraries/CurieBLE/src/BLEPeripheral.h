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

#include "BLECommon.h"
#include "BLERoleBase.h"
#include "BLEPeripheralHelper.h"

/** Function prototype for BLE Peripheral Device event callback */
typedef void (*BLEPeripheralEventHandler)(BLECentralHelper &central);

/**
 * BLE Peripheral
 */
class BLEPeripheral{
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
    void setAdvertisedServiceUuid(const struct bt_uuid* advertisedServiceUuid);

    /**
     * Set the local name that the BLE Peripheral Device advertises
     *
     * @param localName  local name to advertise
     *
     * @note This method must be called before the begin method
     */
    void setLocalName(const char* localName);

    /**
     * Set the Service Data that the BLE Peripheral Device advertises
     *
     * @param serviceDataUuid 16-bit Service UUID for this Service Data
     *   (in string form). Must match the UUID parameter
     *   of setAdvertisedServiceUuid(). To fit into BLE_MAX_ADV_SIZE,
     *   the UUID must be a 16-bit UUID.
     *
     * @param serviceData binary array of Service Data.
     *
     * @param serviceDataLength length (bytes) of serviceData[]
     *
     * @note the entire advertising packet must be no more than
     *   BLE_MAX_ADV_SIZE bytes, which is currently 31.
     *   This likely means that if you use Service Data
     *   there will not be room for a Local Name.
     *
     * @note if serviceDataUuid isn't 16-bits long, or if
     *   serviceDataLength won't fit in the advertising block,
     *   the service data will silently not be copied
     *   into the advertising block.
     */
    void setAdvertisedServiceData(const struct bt_uuid* serviceDataUuid, 
                                  uint8_t* serviceData, 
                                  uint8_t serviceDataLength);
    /**
     * Set the ADV parameters about the ADV-Type and interval
     *
     * @param   type            Advertising types
     *
     * @param   interval_min    Minimum Advertising Interval (N * 0.625)
     *
     * @param   interval_max    Maximum Advertising Interval (N * 0.625)
     *
     * @note    none
     */
    void setAdvertisingParam(uint8_t  type, 
                             uint16_t interval_min,
                             uint16_t interval_max);
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
     * Set the min and max connection interval BLE Peripheral Device
     *
     * @param minConnInterval Minimum connection interval (1.25 ms units), minimum 0x0006 (7.5ms)
     * @param maxConnInterval Maximum connection interval (1.25 ms units), maximum 0x095f (2998.75ms)
     *
     * @note This method must be called before the begin method
     */
    void setConnectionInterval(const unsigned short minConnInterval, const unsigned short maxConnInterval);

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
    void setEventHandler(BLERoleEvent event, BLERoleEventHandler callback);

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
    BLECentralHelper central(void);

    /**
     * Is a central connected?
     *
     * @return boolean_t true if central connected, otherwise false
     */
    bool connected(void);
    
	/**
	 * @brief   Init the ADV data and start send advertisement
	 *
	 * @param   none
	 *
	 * @return  BleStatus		0 - Success. Others - error code
	 *
	 * @note  none
	 */
    BleStatus startAdvertising(void);
	
	/**
	 * @brief   Stop send advertisement
	 *
	 * @param   none
	 *
	 * @return  BleStatus		0 - Success. Others - error code
	 *
	 * @note  none
	 */
	BleStatus stopAdvertising(void);
	
protected:
    void handleConnectEvent(struct bt_conn *conn, uint8_t err);
    void handleDisconnectEvent(struct bt_conn *conn, uint8_t reason);
    void handleParamUpdated(struct bt_conn *conn, 
                            uint16_t interval,
				            uint16_t latency, 
				            uint16_t timeout);

private:
    
    BleStatus _stop(void);

    BleStatus _advDataInit(void);

private:
    const char* _local_name;
    
    const struct bt_uuid* _service_data_uuid;
    uint8_t* _service_data;
    uint8_t  _service_data_length;
    uint8_t  _service_data_buf[BLE_MAX_ADV_SIZE];
    
    uint16_t   _appearance;
    
    const struct bt_uuid* _advertise_service_uuid;
    
    uint8_t    _adv_type;
    struct bt_data _adv_data[4];
    size_t       _adv_data_idx;
};

#endif // _BLE_DEVICE_H_INCLUDED
