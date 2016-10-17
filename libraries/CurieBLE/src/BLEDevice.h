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

#ifndef ARDUINO_BLE_DEVICE_H
#define ARDUINO_BLE_DEVICE_H

#include <Arduino.h>

enum BLEDeviceEvent {
  BLEConnected = 0,         // BLE device connected
  BLEDisconnected = 1,      // BLE device disconnected 
  BLEConParamUpdate = 2,    // Update the connection parameter 
                            //  Connection update request in central
                            //  Connection parameter updated in peripheral
  BLEDiscovered,            // The scanned BLE device
  BLEDeviceLastEvent
};

typedef void (*BLEDeviceEventHandler)(BLEDevice device);

class BLEDevice
{
  public:
    /**
     * @brief   The BLE device constructure
     *
     * @param   none
     *
     * @return  none
     *
     * @note  none
     */
    BLEDevice();
    
    /**
     * @brief   The BLE device constructure
     *
     * @param[in]   bledevice  BLE device
     *
     * @return  none
     *
     * @note  none
     */
    BLEDevice(const BLEDevice* bledevice);
    BLEDevice(const BLEDevice& bledevice);
    virtual ~BLEDevice();

    
    /**
     * @brief   Initiliaze the BLE hardware
     *
     * @return  bool indicating success or error
     *
     * @note  This method are for real BLE device. 
     *          Not for peer BLE device.
     */
    bool begin();
    
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
     * @return   bool indicating success or error
     *
     * @note  none
     */
    bool connected() const;
    
    /**
     * @brief   Disconnect the connected device/s.
     *
     * @param   none
     *
     * @return   bool indicating success or error    
     *
     * @note  The BLE may connected multiple devices.
     *        This call will disconnect all conected devices.
     */
    bool disconnect();
    
    
    /**
     * @brief   Get the BLE address of the BLE in string format
     *
     * @param   none
     *
     * @return String   The address of the BLE in string format
     *
     * @note  none
     */
    String address() const;

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
    //void setConnectionInterval(int minimumConnectionInterval, 
    //                           int maximumConnectionInterval,
    //                           uint16_t latency, 
    //                           uint16_t timeout);
    
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
    void setConnectionInterval(int minimumConnectionInterval, 
                               int maximumConnectionInterval);
    
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
     * @return int     Indicating success or error type @enum BLE_STATUS_T
     *
     * @note This method must be called before the begin method
     */
    int addService(BLEService& attribute);
    
    /**
     * @brief   Construct the ADV data and start send advertisement
     *
     * @param   none
     *
     * @return  int       0 - Success. Others - error code @enum BLE_STATUS_T
     *
     * @note  none
     */
    int advertise();
    
    /**
     * @brief   Stop send advertisement
     *
     * @param   none
     *
     * @return  none
     *
     * @note  none
     */
    void stopAdvertise();

    /**
     * @brief   Get currently connected central 
     *
     * @return BLEDevice Connected central device
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
    bool operator==(const BLEDevice& device) const;
    bool operator!=(const BLEDevice& device) const;
    BLEDevice& operator=(const BLEDevice& device);
    // central mode

    //void scanForAddress(String address); // Not include in baseline. Add here as feature for feature release.
    
    /**
     * @brief   Start scanning for peripherals without filter
     *
     * @param   none
     *
     * @return  none
     *
     * @note  none
     */
    void scan();
    
    /**
     * @brief   Start scanning for peripherals with filter
     *
     * @param[in] withDuplicates        true - with duplicate filter
     *                                  false- without duplicate filter
     *
     * @return  none
     *
     * @note  option to filter out duplicate addresses for Arduino.
     *          The current only support fileter duplicate mode.
     */
    void scan(bool withDuplicates);
    
    /**
     * @brief   Start scanning for peripherals and filter by device name in ADV
     *
     * @param   name    The device's local name.
     *
     * @return  none
     *
     * @note   option to filter out duplicate addresses for Arduino.
     *          The current only support fileter duplicate mode.
     */
    void scanForName(String name);
    
    /**
     * @brief   Start scanning for peripherals and filter by device name in ADV
     *
     * @param[in]   name    The device's local name.
     *
     * @param[in]   withDuplicates      true - with duplicate filter
     *                                  false- without duplicate filter
     *
     * @return  none
     *
     * @note   option to filter out duplicate addresses for Arduino.
     *          The current only support fileter duplicate mode.
     */
    void scanForName(String name, bool withDuplicates);
    
    /**
     * @brief   Start scanning for peripherals and filter by service in ADV
     *
     * @param   service    The service
     *
     * @return  none
     *
     * @note  none
     */
    void scanForUuid(String uuid);
    
    /**
     * @brief   Start scanning for peripherals and filter by service in ADV
     *
     * @param[in]   service    The service
     *
     * @param[in]   withDuplicates      true - with duplicate filter
     *                                  false- without duplicate filter
     *
     * @return  none
     *
     * @note   option to filter out duplicate addresses for Arduino.
     *          The current only support fileter duplicate mode.
     */
    void scanForUuid(String uuid, bool withDuplicates); 
    
    /**
     * @brief   Stop scanning for peripherals
     *
     * @param   none
     *
     * @return  none
     *
     * @note  none
     */
    void stopScan();
    
    /**
     * @brief   Retrieve a discovered peripheral
     *
     * @param   none
     *
     * @return  BLEDevice   The BLE device that central scanned
     *
     * @note  none
     */
    BLEDevice available();

    /**
     * @brief   Does the peripheral advertise a local name
     *
     * @param   none
     *
     * @return  none
     *
     * @note  none //TODO: The implementation doesn't save the ADV's local name.
     */
    bool hasLocalName() const;
    
    bool hasAdvertisedServiceUuid() const; // does the peripheral advertise a service
    bool hasAdvertisedServiceUuid(int index) const; // does the peripheral advertise a service n
    int advertisedServiceUuidCount() const; // number of services the peripheral is advertising

    String localName() const; // returns the advertised local name as a String
    String advertisedServiceUuid() const; // returns the advertised service as a UUID String
    String advertisedServiceUuid(int index) const; // returns the nth advertised service as a UUID String

    int rssi() const; // returns the RSSI of the peripheral at discovery

    bool connect(); // connect to the peripheral
    bool discoverAttributes(); // discover the peripheral's attributes
    bool discoverAttributesByService(const char* svc_uuid);

    String deviceName(); // read the device name attribute of the peripheral, and return String value
    //int appearance(); // read the appearance attribute of the peripheral and return value as int

    // For GATT
    /**
     * @brief   returns the number of services the BLE device has
     *
     * @param   none
     *
     * @return  int     The number of services
     *
     * @note  none
     */
    int serviceCount() const; 
    
    /**
     * @brief   Does the peripheral have a service with the specified UUID
     *
     * @param   uuid    The 128/16 bits UUID
     *
     * @return  bool    true - Found
     *                  false- Not found
     *
     * @note  none
     */
    bool hasService(const char* uuid) const;
    
    /**
     * @brief   Does the peripheral have an nth service with the specified UUID
     *
     * @param   uuid    The 128/16 bits UUID
     *
     * @param   index   The index
     *
     * @return  bool    true - Found
     *                  false- Not found
     *
     * @note  none
     */
    bool hasService(const char* uuid, int index) const;
    
    /**
     * @brief   Return the nth service of the peripheral
     *
     * @param   index   The index
     *
     * @return  BLEService    The BLE service
     *
     * @note  none
     */
    BLEService service(int index) const;
    
    /**
     * @brief   Return the service with the specified UUID
     *
     * @param   uuid    The 128/16 bits UUID
     *
     * @return  BLEService    The BLE service
     *
     * @note  none
     */
    BLEService service(const char * uuid) const;
    
    /**
     * @brief   Return the nth service with the specified UUID
     *
     * @param   uuid    The 128/16 bits UUID
     *
     * @param   index   The index
     *
     * @return  BLEService    The BLE service
     *
     * @note  none
     */
    BLEService service(const char * uuid, int index) const;

    /**
     * @brief   Returns the number of characteristics the BLE device has
     *
     * @param   none
     *
     * @return  int     The number of characteristics
     *
     * @note  none
     */
    int characteristicCount() const;
    
    /**
     * @brief   Does the device have a characteristic with the specified UUID
     *
     * @param   uuid    The 128/16 bits UUID
     *
     * @return  bool    true - Found
     *                  false- Not found
     *
     * @note  none
     */
    bool hasCharacteristic(const char* uuid) const;
    
    /**
     * @brief   Does the device have an nth characteristic with the 
     *           specified UUID
     *
     * @param   uuid    The 128/16 bits UUID
     *
     * @param   index   The index
     *
     * @return  bool    true - Found
     *                  false- Not found
     *
     * @note  none
     */
    bool hasCharacteristic(const char* uuid, int index) const;
    
    /**
     * @brief   Return the nth characteristic of the BLE device
     *
     * @param   index   The index
     *
     * @return  BLECharacteristic   The BLE characteristic
     *
     * @note  none
     */
    BLECharacteristic characteristic(int index) const;
    
    /**
     * @brief   Return the characteristic with the specified UUID
     *
     * @param   uuid    The 128/16 bits UUID
     *
     * @return  BLECharacteristic   The BLE characteristic
     *
     * @note  none
     */
    BLECharacteristic characteristic(const char * uuid) const;
    
    /**
     * @brief   Return the nth characteristic with the specified UUID
     *
     * @param   uuid    The 128/16 bits UUID
     *
     * @param   index   The index
     *
     * @return  BLECharacteristic   The BLE characteristic
     *
     * @note  none
     */
    BLECharacteristic characteristic(const char * uuid, int index) const;

    // event handler
    /**
     * @brief   Set the event callbacks
     *
     * @param   event           The BLE device event
     *
     * @param   eventHandler    The BLE device event handler
     *
     * @return  none
     *
     * @note  none
     */
    void setEventHandler(BLEDeviceEvent event, BLEDeviceEventHandler eventHandler); // set an event handler (callback)
    
protected:
    friend class BLECharacteristicImp;
    friend class BLEServiceImp;
    friend class BLEDeviceManager;
    friend class BLEProfileManager;
    friend class BLECharacteristic;
    friend class BLEDescriptor;
    friend class BLEService;
    friend uint8_t profile_notify_process (bt_conn_t *conn,
                                            bt_gatt_subscribe_params_t *params,
                                            const void *data, uint16_t length);
    friend uint8_t profile_read_rsp_process(bt_conn_t *conn, 
                                             int err,
                                             bt_gatt_read_params_t *params,
                                             const void *data, 
                                             uint16_t length);
    const bt_addr_le_t* bt_le_address() const;
    const bt_le_conn_param* bt_conn_param() const;
    void setAddress(const bt_addr_le_t& addr);
    
    void setAdvertiseData(const uint8_t* adv_data, uint8_t len);
    /**
     * @brief   The BLE device constructure
     *
     * @param[in]   bleaddress  BLE device address
     *
     * @return  none
     *
     * @note  none
     */
    BLEDevice(const bt_addr_le_t* bleaddress);
private:
    void preCheckProfile();
    
    /**
     * @brief   Start scanning for peripherals with/without duplicate filter
     *
     * @param[in] withDuplicates        true - with duplicate filter
     *                                  false- without duplicate filter
     *
     * @return  none
     *
     * @note  option to filter out duplicate addresses for Arduino.
     *          The current only support fileter duplicate mode.
     */
    bool startScan(bool withDuplicates);
    
private:
    bt_addr_le_t _bt_addr;
    
    bt_le_conn_param _conn_param;
};

#endif
