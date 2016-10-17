/*
 * Copyright (c) 2016 Intel Corporation.  All rights reserved.
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

#ifndef __BLE_PROFILE_MANAGER_H__
#define __BLE_PROFILE_MANAGER_H__

#include "CurieBLE.h"

#include "BLEServiceImp.h"

//#include "BLECommon.h"
//#include "BLEDevice.h"
//#include "BLEService.h"
typedef struct {
    bt_addr_le_t  address;
    uint16_t      handle;
}ServiceRead_t;

class BLEProfileManager{
public:
    /**
     * @brief   Get the BLEProfile Manager instance
     *
     * @param   none
     *
     * @return  BLEProfileManager*  BLE Profile manager
     *
     * @note  none
     */
    static BLEProfileManager* instance();
    
    /**
     * @brief   Add an service to the BLE Device
     *
     * @param[in] bledevice The BLE device that owned the service
     *
     * @param[in] service   The service to add to BLE device profile
     *
     * @return BleStatus indicating success or error
     *
     * @note This method must be called before the begin method in GATT server role
     *       Or be called in discover process.
     */
    BLEServiceImp *addService (BLEDevice &bledevice, BLEService& service);
    BLEServiceImp *addService (BLEDevice &bledevice, const bt_uuid_t* uuid);
    
    /**
     * @brief   Register the profile to Nordic BLE stack
     *
     * @param[in]   bledevice       The BLE Device
     *
     * @return  int     std C errno
     *
     * @note  none
     */
    int registerProfile(BLEDevice &bledevice);
    
    inline bool hasRegisterProfile(){return _profile_registered;}
    
    /**
     * @brief   Get the BLE's Characteristic implementation object by uuid and index
     *
     * @param[in]   bledevice   The BLE device
     *
     * @param[in]   uuid        The characteristic UUID
     *
     * @param[in]   index       The characteristic index in the profile
     *
     * @return  BLECharacteristicImp*   The BLE characteristic implementation object
     *
     * @note  none
     */
    BLECharacteristicImp* characteristic(const BLEDevice &bledevice, 
                                         const char* uuid, 
                                         int index);
    BLECharacteristicImp* characteristic(const BLEDevice &bledevice, 
                                         const char* uuid);
    BLECharacteristicImp* characteristic(const BLEDevice &bledevice, 
                                         int index);
    BLECharacteristicImp* characteristic(const BLEDevice &bledevice, 
                                         uint16_t handle);
    BLEServiceImp* service(const BLEDevice &bledevice, const char * uuid) const;
    BLEServiceImp* service(const BLEDevice &bledevice, int index) const;
    BLEServiceImp* service(const BLEDevice &bledevice, const bt_uuid_t* uuid) const;
    int serviceCount(const BLEDevice &bledevice) const;
    int characteristicCount(const BLEDevice &bledevice) const;
    
    uint8_t discoverResponseProc(bt_conn_t *conn,
                                 const bt_gatt_attr_t *attr,
                                 bt_gatt_discover_params_t *params);
    
    bool discoverAttributes(BLEDevice* device);
    bool discoverAttributesByService(BLEDevice* device, const bt_uuid_t* svc_uuid);
    void handleConnectedEvent(const bt_addr_le_t* deviceAddr);
    void handleDisconnectedEvent(const bt_addr_le_t* deviceAddr);
    void handleDisconnectedPutOffEvent();
    uint8_t serviceReadRspProc(bt_conn_t *conn, 
                               int err,
                               bt_gatt_read_params_t *params,
                               const void *data, 
                               uint16_t length);
protected:
    friend ssize_t profile_write_process(bt_conn_t *conn,
                                     const bt_gatt_attr_t *attr,
                                     const void *buf, uint16_t len,
                                     uint16_t offset);
    bool discoverService(BLEDevice* device, const bt_uuid_t* svc_uuid);
private:
    typedef LinkNode<BLEServiceImp *> BLEServiceLinkNodeHeader;
    typedef LinkNode<BLEServiceImp *>* BLEServiceNodePtr;
    typedef LinkNode<BLEServiceImp *> BLEServiceNode;
    
    typedef LinkNode<ServiceRead_t> ServiceReadLinkNodeHeader;
    typedef LinkNode<ServiceRead_t>* ServiceReadLinkNodePtr;
    typedef LinkNode<ServiceRead_t> ServiceReadLinkNode;
    
    BLEProfileManager();
    ~BLEProfileManager (void);
    
    void serviceDiscoverComplete(const BLEDevice &bledevice);
    
    int getDeviceIndex(const bt_addr_le_t* macAddr);
    int getDeviceIndex(const BLEDevice* device);
    /**
     * @brief   Get the unused service header index
     *
     * @param   none
     *
     * @return  int     The unused BLE profile index
     *
     * @note  This object has a buffer to manage all devices profile. 
     *          The buffer is an array. The different profiles 
     *          distinguished by BLE address.
     */
    int getUnusedIndex();
    
    /**
     * @brief   Get the Service header by BLE device
     *
     * @param[in]   bledevice   The BLE device
     *
     * @return  none
     *
     * @note  none
     */
    BLEServiceLinkNodeHeader* getServiceHeader(const BLEDevice &bledevice);
    const BLEServiceLinkNodeHeader* getServiceHeader(const BLEDevice &bledevice) const;
    
    /**
     * @brief   Get the BLE attribute counter based on services, characteristics
     *           and descriptors.
     *
     * @param   none
     *
     * @return  none
     *
     * @note  none
     */
    int getAttributeCount(BLEDevice &bledevice);
    
    /**
     * @brief   Discard the profile by BLE device
     *
     * @param[in]   bledevice   The BLE device
     *
     * @return  none
     *
     * @note  none
     */
    void clearProfile(BLEServiceLinkNodeHeader* serviceHeader);
    
    bool readService(const BLEDevice &bledevice, uint16_t handle);
    bool discovering();
    void setDiscovering(bool discover);
    void checkReadService();
    
private:
    // The last header is for local BLE
    BLEServiceLinkNodeHeader _service_header_array[BLE_MAX_CONN_CFG + 1]; // The connected devices' service and self service
    bt_addr_le_t  _addresses[BLE_MAX_CONN_CFG + 1]; // The BLE devices' address
    bt_addr_le_t  _discovering_ble_addresses;
    
    bool _start_discover;

    bool _discovering;
    uint64_t _discover_rsp_timestamp;
    bt_gatt_discover_params_t _discover_params[BLE_MAX_CONN_CFG];
    bt_uuid_128_t _discover_uuid[BLE_MAX_CONN_CFG];
    BLEServiceImp* _cur_discover_service;
    bool _discover_one_service;
    bt_gatt_read_params_t _read_params;
    bool _reading;
    ServiceReadLinkNodeHeader _read_service_header;
    
    bt_gatt_attr_t *_attr_base; // Allocate the memory for BLE stack
    int _attr_index;
    
    static BLEProfileManager* _instance; // The profile manager instance
    bool _profile_registered;
    uint8_t _disconnect_bitmap;
};

#endif

