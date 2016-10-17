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

#ifndef _BLE_SERVICE_IMP_H_INCLUDED
#define _BLE_SERVICE_IMP_H_INCLUDED

#include "CurieBLE.h"

#include "BLEAttribute.h"

#include "LinkList.h"

/**
 * BLE GATT Service
 */
class BLEServiceImp: public BLEAttribute{
public:
    /**
     * Constructor for BLE Service
     *
     * @param[in] uuid    16-bit or 128-bit UUID (in string form) defined by BLE standard
     */
    BLEServiceImp(BLEService& service);
    BLEServiceImp(const bt_uuid_t* uuid);
    ~BLEServiceImp();

    /**
     * @brief   Add a characteristic in service
     *
     * @param[in]   bledevice       The BLE device want to add the characteristic
     *
     * @param[in]   characteristic  The characteristic want to be added to service
     *
     * @return  none
     *
     * @note  none
     */
    int addCharacteristic(BLEDevice& bledevice, BLECharacteristic& characteristic);
    int addCharacteristic(BLEDevice& bledevice, 
                          const bt_uuid_t* uuid, 
                          uint16_t handle, 
                          unsigned char properties);
    int getCharacteristicCount();

    BLECharacteristicImp* characteristic(const bt_uuid_t* uuid);
    BLECharacteristicImp* characteristic(const char* uuid);
    BLECharacteristicImp* characteristic(int index);
    BLECharacteristicImp* characteristic(uint16_t handle);
    inline void setHandle(uint16_t handle){_start_handle = handle;}
    inline void setEndHandle(uint16_t handle){_end_handle = handle;}
    inline uint16_t endHandle(){return _end_handle;}
    inline uint16_t startHandle(){return _start_handle;}
    
    bool discoverAttributes(BLEDevice* device);
    uint8_t discoverResponseProc(bt_conn_t *conn,
                                 const bt_gatt_attr_t *attr,
                                 bt_gatt_discover_params_t *params);
    bool discovering();
    
    static bt_uuid_t *getPrimayUuid(void);
protected:
    friend class BLEProfileManager;
    
    int getAttributeCount();
    
    
    int updateProfile(bt_gatt_attr_t *attr_start, int& index);
private:
    typedef LinkNode<BLECharacteristicImp *>  BLECharacteristicLinkNodeHeader;
    typedef LinkNode<BLECharacteristicImp *>* BLECharacteristicNodePtr;
    typedef LinkNode<BLECharacteristicImp *>  BLECharacteristicNode;
    
    uint16_t _start_handle;
    uint16_t _end_handle;
    
    void releaseCharacteristic();
    BLECharacteristicImp *_cur_discover_chrc;

    static bt_uuid_16_t _gatt_primary_uuid;
    bt_gatt_discover_params_t _discover_params;
    
    BLECharacteristicLinkNodeHeader _characteristics_header; // The characteristic link list
};

#endif // _BLE_SERVICE_H_INCLUDED
