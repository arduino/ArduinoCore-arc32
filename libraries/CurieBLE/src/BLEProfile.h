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

#ifndef __BLE_PROFILE_H__
#define __BLE_PROFILE_H__

#include "BLECommon.h"
#include "BLEAttribute.h"
#include "BLECentralHelper.h"
#include "BLECharacteristic.h"
#include "BLEService.h"

class BLEProfile{
public:
    BLEProfile(BLEPeripheralHelper *peripheral);
    ~BLEProfile (void);
    
    /**
     * @brief   Add an attribute to the BLE Peripheral Device
     *
     * @param[in] attribute Attribute to add to Peripheral
     *
     * @return BleStatus indicating success or error
     *
     * @note This method must be called before the begin method
     */
    BleStatus addAttribute(BLEAttribute& attribute);
    
    /**
     * @brief   Register the profile to Nordic BLE stack
     *
     * @param   none
     *
     * @return  int     std C errno
     *
     * @note  none
     */
    int registerProfile();
    
    /**
     * @brief   Get BLEAttribute by subscribe parameter
     *
     * @param[in]   params              Subscribe parameter
     *
     * @return  BLEAttribute *      NULL - Not found
     *                              Not NULL - The BLEAttribute object
     *
     * @note  none
     */
    BLEAttribute *attribute(bt_gatt_subscribe_params_t *params);
    
    /**
     * @brief   Get BLEAttribute by characteristic handle
     *
     * @param[in]   handle              The characteristic handle
     *
     * @return  BLEAttribute *      NULL - Not found
     *                              Not NULL - The BLEAttribute object
     *
     * @note  none
     */
    BLEAttribute *attribute(uint16_t handle);
    
    /**
     * @brief   Process the discover response and 
     *           discover the BLE peripheral profile
     *
     * @param[in]   const bt_gatt_attr_t *     The gatt attribute response
     *
     * @return  uint8_t     BT_GATT_ITER_STOP   Stop discover the profile
     *                      BT_GATT_ITER_CONTINUE Continue to send the discover request
     *
     * @note  This function only for the central device.
     */
    uint8_t discover(const bt_gatt_attr_t *attr);
    
    /**
     * @brief   Discover the BLE peripheral profile
     *
     * @param   none
     *
     * @return  none
     *
     * @note  This function only for the central device.
     *
     * @note  The central deivce didn't know the connected BLE's profile.
     *         Need send discover request to search the attribute in the BLE peripheral
     */
    void discover();
    
    /**
     * @brief   Clear the handle in central mode
     *
     * @param   none
     *
     * @return  none
     *
     * @note    The peripheral can't call this.
     *          Because the central need discover the handles.
     *          Peripheral device only get the handle when register the profile.
     */
    void clearHandles(void);
    
    /**
     * @brief   Get the characteristic value handle
     *
     * @param[in]   attr            Attribute object 
     *
     * @return  uint16_t        The value handle
     *                           0 is invalid handle
     *
     * @note  none
     */
    uint16_t valueHandle(BLEAttribute *attr);
    
    /**
     * @brief   Get characteristic configuration descriptor value handle
     *
     * @param[in]   attr            Attribute object 
     *
     * @return  uint16_t        The value handle
     *                           0 is invalid handle
     *
     * @note  none
     */
    uint16_t cccdHandle(BLEAttribute *attr);
protected:
    friend ssize_t profile_write_process(bt_conn_t *conn,
                                     const bt_gatt_attr_t *attr,
                                     const void *buf, uint16_t len,
                                     uint16_t offset);
private:
    /**
     * @brief   Get BLEAttribute by UUID
     *
     * @param[in]   const bt_uuid_t*       The UUID
     *
     * @return  BLEAttribute *      NULL - Not found
     *                              Not NULL - The BLEAttribute object
     *
     * @note  Use the pointer value instead the UUID value.
     *         Because the uuid pointer in bt_gatt_attr is got from BLEAttribute
     *         So set this as private.
     */
    BLEAttribute *attribute(const bt_uuid_t* uuid);
    
    /**
     * @brief   Get bt_gatt_attr by BLEAttribute class
     *
     * @param[in]   BLEAttribute *      The BLEAttribute object
     *
     * @return  bt_gatt_attr_t*    NULL - Not found
     *                             Not NULL - The bt_gatt_attr in the stack
     *
     * @note  none
     */
    bt_gatt_attr_t* declarationAttr(BLEAttribute *attr);
    
    /**
     * @brief   Process the descriptor discover response
     *
     * @param[in]   const bt_gatt_attr_t *     The discover response
     *
     * @param[in]   BLEAttribute *                  The BLEAttribute object in discovering
     *
     * @return  none
     *
     * @note  none
     */
    void descriptorDiscoverRsp(const bt_gatt_attr_t *attr, BLEAttribute* bleattr);
    
    /**
     * @brief   Process the characteristic discover response
     *
     * @param[in]   const bt_gatt_attr_t *     The discover response
     *
     * @param[in]   BLEAttribute *                  The BLEAttribute object in discovering
     *
     * @return  none
     *
     * @note  none
     */
    void characteristicDiscoverRsp(const bt_gatt_attr_t *attr, BLEAttribute* bleattr);
    
private:
    BLEPeripheralHelper *_peripheral;
    bt_gatt_attr_t *_attr_base;
    int _attr_index;
    
    BLEAttribute** _attributes;
    uint16_t _num_attributes;
    
    bt_gatt_subscribe_params_t *_sub_param;
    int _sub_param_idx;
    
    bt_gatt_discover_params_t _discover_params;
};

#endif

