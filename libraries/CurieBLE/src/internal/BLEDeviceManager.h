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
    bool connected(const BLEDevice *device) const;
    
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
    void setAdvertisedServiceData(const bt_uuid_t* serviceDataUuid,
                                  const uint8_t* serviceData,
                                  uint8_t serviceDataLength);

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

    bool advertising();
    
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
    
    operator bool() const;

    // central mode
    void clearAdvertiseCritical();
    void setAdvertiseCritical(String name);
    void setAdvertiseCritical(BLEService& service);
    bool startScanning(); // start scanning for peripherals
    bool startScanningWithDuplicates(); // start scanning for peripherals, and report all duplicates
    bool stopScanning(); // stop scanning for peripherals
    
    void setAcceptAdvertiseLocalName(String name);
    void setAcceptAdvertiseLocalName(BLEService& service);
    void setAcceptAdvertiseCallback(String name);
    
    BLEDevice available(); // retrieve a discovered peripheral

    bool hasLocalName(const BLEDevice* device) const; // does the peripheral advertise a local name
    bool hasAdvertisedServiceUuid(const BLEDevice* device) const; // does the peripheral advertise a service
    bool hasAdvertisedServiceUuid(const BLEDevice* device, int index) const; // does the peripheral advertise a service n
    int advertisedServiceUuidCount(const BLEDevice* device) const; // number of services the peripheral is advertising

    String localName(const BLEDevice* device) const; // returns the advertised local name as a String
    String advertisedServiceUuid(const BLEDevice* device) const; // returns the advertised service as a UUID String
    String advertisedServiceUuid(const BLEDevice* device, int index) const; // returns the nth advertised service as a UUID String

    int rssi(const BLEDevice* device) const; // returns the RSSI of the peripheral at discovery

    bool connect(BLEDevice &device); // connect to the peripheral
    bool connectToDevice(BLEDevice &device);

    String deviceName(const BLEDevice* device); // read the device name attribute of the peripheral, and return String value
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
    bool setAdvertiseBuffer(const bt_addr_le_t* bt_addr,
                            const uint8_t *ad, 
                            uint8_t data_len,
                            int8_t rssi);
    void getDeviceAdvertiseBuffer(const bt_addr_le_t* addr, 
                                  const uint8_t* &adv_data,
                                  uint8_t &adv_len) const;
    bool disconnectSingle(const bt_addr_le_t *peer);

private:
    uint16_t   _min_conn_interval;
    uint16_t   _max_conn_interval;
    bt_addr_le_t _local_bda;
    char       _device_name[BLE_MAX_DEVICE_NAME + 1];
    
    // For Central
    bt_le_scan_param_t _scan_param;     // Scan parameter
    bt_addr_le_t _peer_adv_buffer[BLE_MAX_ADV_BUFFER_CFG];   // Accepted peer device adress
    uint64_t     _peer_adv_mill[BLE_MAX_ADV_BUFFER_CFG];     // The ADV found time stamp
    uint8_t    _peer_adv_data[BLE_MAX_ADV_BUFFER_CFG][BLE_MAX_ADV_SIZE];
    uint8_t    _peer_adv_data_len[BLE_MAX_ADV_BUFFER_CFG];
    int8_t     _peer_adv_rssi[BLE_MAX_ADV_BUFFER_CFG];
    bt_data_t   _adv_accept_critical;   // The filters for central device
    String  _adv_critical_local_name;
    bt_uuid_128_t _adv_critical_service_uuid;
    
    bt_addr_le_t _wait_for_connect_peripheral;
    uint8_t    _wait_for_connect_peripheral_adv_data[BLE_MAX_ADV_SIZE];
    uint8_t    _wait_for_connect_peripheral_adv_data_len;
    int8_t     _wait_for_connect_peripheral_adv_rssi;
    
    bt_addr_le_t _available_for_connect_peripheral;
    uint8_t    _available_for_connect_peripheral_adv_data[BLE_MAX_ADV_SIZE];
    uint8_t    _available_for_connect_peripheral_adv_data_len;
    int8_t     _available_for_connect_peripheral_adv_rssi;
    volatile bool    _connecting;
    
    // For peripheral
    struct bt_le_adv_param _adv_param;
    bool _has_service_uuid;
    bt_uuid_128_t _service_uuid;
    bool _has_service_solicit_uuid;
    bt_uuid_128_t _service_solicit_uuid;
    uint16_t   _appearance;
    uint8_t    _manufacturer_data[BLE_MAX_ADV_SIZE];
    uint8_t    _manufacturer_data_length;
    bt_uuid_128_t _service_data_uuid;
    uint8_t     _service_data[BLE_MAX_ADV_SIZE];
    uint8_t     _service_data_buf[BLE_MAX_ADV_SIZE];
    uint8_t     _service_data_length;
    
    // ADV data for peripheral
    uint8_t     _adv_type;
    bt_data_t   _adv_data[6];  // KW: fount _advDataInit() can use 6 slots.
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
    uint8_t      _peer_peripheral_index;
    uint8_t    _peer_peripheral_adv_data[BLE_MAX_CONN_CFG][BLE_MAX_ADV_SIZE];
    uint8_t    _peer_peripheral_adv_data_len[BLE_MAX_CONN_CFG];
    uint8_t    _peer_peripheral_adv_rssi[BLE_MAX_CONN_CFG];

    BLEDeviceEventHandler _device_events[BLEDeviceLastEvent];
};

#endif
