/*
  BLE Device API
  Copyright (c) 2016 Arduino LLC. All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301 USA
*/

#ifndef ARDUINO_BLE_DEVICE_MANAGER_H
#define ARDUINO_BLE_DEVICE_MANAGER_H

#include <Arduino.h>

class BLEDeviceManager
{
  public:
    /**
     * @brief   The BLE device constructure
     *
     * @param   bleaddress  BLE device address
     *
     * @return  none
     *
     * @note  none
     */
    BLEDeviceManager();
    
    virtual ~BLEDeviceManager();

    
    /**
     * @brief Initiliaze the BLE hardware
     *
     * @return bool indicating success or error
     *
     * @note  This method are for real BLE device. 
     *          Not for peer BLE device.
     */
    bool begin(BLEDevice *device);
    
    /**
     * @brief   Poll for events
     *
     * @param   none
     *
     * @return  none
     *
     * @note  This method are for real BLE device. 
     *          Not for peer BLE device.
     */
    void poll(); // Do we need add the return value or 
                 //  input parameter to get the events?
                 // Events may inlcue:
                 //  GAP : Connected, Disconnected, Update connetion parameter
                 //  GATT: Discovered
    
    /**
     * @brief   Deinitiliaze the BLE hardware
     *
     * @param   none
     *
     * @return  none
     *
     * @note  This method are for real BLE device. 
     *          Not for peer BLE device.
     */
    void end(); 

    /**
     * @brief   Is the device connected with another BLE device.
     *
     * @param   none
     *
     * @return  none
     *
     * @note  none
     */
    bool connected(BLEDevice *device);
    
    /**
     * @brief   Disconnect the connected device/s.
     *
     * @param   none
     *
     * @return  none
     *
     * @note  The BLE may connected multiple devices.
     *          This call will disconnect all conected devices.
     */
    bool disconnect(BLEDevice *device);
    
    void setEventHandler(BLEDeviceEvent event, 
                         BLEDeviceEventHandler eventHandler);
    /**
     * @brief   Set the service UUID that the BLE Peripheral Device advertises
     *
     * @param[in] advertisedServiceUuid  16-bit or 128-bit UUID to advertis
     *                               (in string form)
     *
     * @note This method must be called before the begin method
     *        Only for peripheral mode.
     */
    void setAdvertisedServiceUuid(const char* advertisedServiceUuid);
    
    /**
     * @brief   Set the service that the BLE Peripheral Device will advertise this UUID
     *
     * @param[in] service  The service the will in advertise data.
     *
     * @note This method must be called before the begin method
     *        Only for peripheral mode.
     */
    void setAdvertisedService(const BLEService& service);
    
    /**
     * @brief   Set the service UUID that is solicited in the BLE Peripheral 
     *           Device advertises
     *
     * @param[in] advertisedServiceUuid  16-bit or 128-bit UUID to advertis
     *                               (in string form)
     *
     * @note This method must be called before the begin method
     *        Only for peripheral mode.
     */
    void setServiceSolicitationUuid(const char* serviceSolicitationUuid);

    /**
     * @brief   Set the manufacturer data in the BLE Peripheral Device advertises
     *
     * @param[in] manufacturerData          The data about manufacturer will 
     *                                       be set in advertisement
     * @param[in] manufacturerDataLength    The length of the manufacturer data
     *
     * @note This method must be called before the begin method
     *        Only for peripheral mode.
     */
    void setManufacturerData(const unsigned char manufacturerData[], 
                             unsigned char manufacturerDataLength);
    
    /**
     * Set the local name that the BLE Peripheral Device advertises
     *
     * @param[in] localName  local name to advertise
     *
     * @note This method must be called before the begin method
     */
    void setLocalName(const char *localName);

    /**
     * @brief   Set advertising interval
     *
     * @param[in]   advertisingInterval    Advertising Interval in ms
     *
     * @return  none
     *
     * @note  none
     */
    void setAdvertisingInterval(float advertisingInterval);
    
    /**
     * @brief   Set the connection parameters and send connection 
     *           update request in both BLE peripheral and central
     *
     * @param[in]   intervalmin     Minimum Connection Interval (ms)
     *
     * @param[in]   intervalmax     Maximum Connection Interval (ms)
     *
     * @param[in]   latency         Connection Latency
     *
     * @param[in]   timeout         Supervision Timeout (ms)
     *
     * @return  none
     *
     * @note  none
     */
    void setConnectionInterval(float minimumConnectionInterval, 
                               float maximumConnectionInterval,
                               uint16_t latency, 
                               uint16_t timeout);
    
    /**
     * @brief   Set the min and max connection interval and send connection 
     *           update request in both BLE peripheral and central
     *
     * @param[in]   intervalmin     Minimum Connection Interval (ms)
     *
     * @param[in]   intervalmax     Maximum Connection Interval (ms)
     *
     * @return  none
     *
     * @note  none
     */
    void setConnectionInterval(float minimumConnectionInterval, 
                               float maximumConnectionInterval);
    
    /**
     * @brief   Set TX power of the radio in dBM
     *
     * @param[in]   tx_power    The antenna TX power
     *
     * @return boolean_t true if established connection, otherwise false
     */
    bool setTxPower(int txPower);
    
    /**
     * @brief   Set advertising type as connectable/non-connectable
     *
     * @param[in]   connectable     true  - The device connectable
     *                              false - The device non-connectable
     *
     * @return  none
     *
     * @note  Only for peripheral mode.
     *         Default value is connectable
     */
    void setConnectable(bool connectable);

    /**
     * @brief   Set the value of the device name characteristic 
     *
     * @param[in] device  User-defined name string for this device.  Truncated if
     *                   more than maximum allowed string length (20 bytes).
     *
     * @note This method must be called before the begin method
     *       If device name is not set, a default name will be used
     */
    void setDeviceName(const char* deviceName);
    void setDeviceName();
    /**
     * @brief   Set the appearance type for the BLE Peripheral Device
     *
     * See https://developer.bluetooth.org/gatt/characteristics/Pages/CharacteristicViewer.aspx?u=org.bluetooth.characteristic.gap.appearance.xml
     * for available options.
     *
     * @param[in] appearance Appearance category identifier as defined by BLE Standard
     *
     * @return BleStatus indicating success or error
     *
     * @note This method must be called before the begin method
     */
    void setAppearance(unsigned short appearance);

    /**
     * @brief   Add a Service to the BLE Peripheral Device
     *
     * @param[in] attribute     The service that will add to Peripheral
     *
     * @return BLE_STATUS_T     Indicating success or error type
     *
     * @note This method must be called before the begin method
     */
    BLE_STATUS_T addService(BLEService& attribute);
    
    /**
     * @brief   Construct the ADV data and start send advertisement
     *
     * @param   none
     *
     * @return  BLE_STATUS_T       0 - Success. Others - error code
     *
     * @note  none
     */
    BLE_STATUS_T startAdvertising();
    
    /**
     * @brief   Stop send advertisement
     *
     * @param   none
     *
     * @return  none
     *
     * @note  none
     */
    BLE_STATUS_T stopAdvertising();

    /**
     * @brief   Get currently connected central 
     *
     * @return BLEDeviceManager Connected central device
     *
     * @note  Peripheral mode only
     */
    BLEDevice central();
    
    /**
     * @brief   Get currently connected peripheral 
     *
     * @param   none
     *
     * @return  none
     *
     * @note  Central mode only. How to distinguish the peripheral?
     */
    BLEDevice peripheral();
    
    /**
     * @brief   Release the resources when link lost
     *
     * @param   none
     *
     * @return  none
     *
     * @note  Peer devices only. Do nothing if local BLE device called.
     */
    void linkLost();
    
    operator bool() const;

    // central mode
    void setAdvertiseCritical(String name);
    void setAdvertiseCritical(BLEService& service);
    bool startScanning(); // start scanning for peripherals
    bool startScanningWithDuplicates(); // start scanning for peripherals, and report all duplicates
    bool stopScanning(); // stop scanning for peripherals
    
    void setAcceptAdvertiseLocalName(String name);
    void setAcceptAdvertiseLocalName(BLEService& service);
    void setAcceptAdvertiseCallback(String name);
    
    BLEDevice available(); // retrieve a discovered peripheral

    bool hasLocalName() const; // does the peripheral advertise a local name
    bool hasAdvertisedServiceUuid() const; // does the peripheral advertise a service
    bool hasAdvertisedServiceUuid(int index) const; // does the peripheral advertise a service n
    int advertisedServiceUuidCount() const; // number of services the peripheral is advertising

    String localName() const; // returns the advertised local name as a String
    String advertisedServiceUuid() const; // returns the advertised service as a UUID String
    String advertisedServiceUuid(int index) const; // returns the nth advertised service as a UUID String

    int rssi() const; // returns the RSSI of the peripheral at discovery

    bool connect(BLEDevice &device); // connect to the peripheral
    bool connectToDevice(BLEDevice &device);

    String deviceName(); // read the device name attribute of the peripheral, and return String value
    int appearance(); // read the appearance attribute of the peripheral and return value as int

    static BLEDeviceManager* instance();
    
    void handleConnectEvent(bt_conn_t *conn, uint8_t err);
    void handleDisconnectEvent(bt_conn_t *conn, uint8_t reason);
    void handleParamUpdated (bt_conn_t *conn, 
                             uint16_t interval,
                             uint16_t latency, 
                             uint16_t timeout);
    void handleDeviceFound(const bt_addr_le_t *addr, 
                           int8_t rssi, 
                           uint8_t type,
                           const uint8_t *ad, 
                           uint8_t data_len);
    
protected:
    
private:
    BLE_STATUS_T _advDataInit(void);
    bool advertiseDataProc(uint8_t type, 
                           const uint8_t *dataPtr, 
                           uint8_t data_len);
    bool setAdvertiseBuffer(const bt_addr_le_t* bt_addr);

private:
    uint16_t   _min_conn_interval;
    uint16_t   _max_conn_interval;
    bt_addr_le_t _local_bda;
    char       _device_name[BLE_MAX_DEVICE_NAME+1];
    
    // For Central
    bt_le_scan_param_t _scan_param;     // Scan parameter
    bt_addr_le_t _peer_adv_buffer[3];   // Accepted peer device adress
    uint64_t     _peer_adv_mill[3];     // The ADV found time stamp
    bt_data_t   _adv_accept_critical;   // The filters for central device
    String  _adv_critical_local_name;
    bt_uuid_128_t _dv_critical_service_uuid;
    bt_addr_le_t _wait_for_connect_peripheral;
    
    // For peripheral
    struct bt_le_adv_param _adv_param;
    bool _has_service_uuid;
    bt_uuid_128_t _service_uuid;
    bt_uuid_128_t _service_solicit_uuid;
    uint16_t   _appearance;
    
    // ADV data for peripheral
    uint8_t     _adv_type;
    bt_data_t   _adv_data[4];
    size_t      _adv_data_idx;
    
    String      _local_name;
    // Peripheral states
    enum BLEPeripheralState {
        BLE_PERIPH_STATE_NOT_READY = 0,
        BLE_PERIPH_STATE_READY,
        BLE_PERIPH_STATE_ADVERTISING,
        BLE_PERIPH_STATE_CONNECTED,
    };

    BLEPeripheralState _state;
    
    // Local
    static BLEDeviceManager* _instance;
    BLEDevice *_local_ble;
    // Connected device object
    bt_addr_le_t _peer_central;
    bt_addr_le_t _peer_peripheral[BLE_MAX_CONN_CFG];

    BLEDeviceEventHandler _device_events[BLEDeviceLastEvent];
};

#endif
