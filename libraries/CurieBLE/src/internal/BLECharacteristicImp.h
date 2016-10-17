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

#ifndef _BLE_CHARACTERISTICIMP_H_INCLUDED
#define _BLE_CHARACTERISTICIMP_H_INCLUDED

//#include "BLECommon.h"

//#include "BLEDevice.h"
//#include "BLEDescriptor.h"

#include "CurieBLE.h"
#include "BLEDescriptorImp.h"

#include "BLEDevice.h"

#include "LinkList.h"
class BLEDescriptorImp;
/**
 * BLE GATT Characteristic  : public BLEAttribute 
 */
class BLECharacteristicImp: public BLEAttribute{
public:
    
    virtual ~BLECharacteristicImp();

    
    /**
     * @brief   Add the characteristic's descriptor
     *
     * @param   descriptor  The descriptor for characteristic
     *
     * @return  none
     *
     * @note  none
     */
    int addDescriptor(BLEDescriptor& descriptor);
    int addDescriptor(const bt_uuid_t* uuid, 
                      unsigned char property, 
                      uint16_t handle);
    
    void releaseDescriptors();

    /**
     * @brief   Write the value of the characteristic
     *
     * @param   value   The value buffer that want to write to characteristic
     *
     * @param   length  The value buffer's length
     *
     * @param   offset  The offset in the characteristic's data
     *
     * @return  bool    true - Success, false - Failed
     *
     * @note  none
     */
    bool writeValue(const byte value[], int length);
    bool writeValue(const byte value[], int length, int offset);

    /**
     * Set the current value of the Characteristic
     *
     * @param[in] value  New value to set, as a byte array.  Data is stored in internal copy.
     * @param[in] length Length, in bytes, of valid data in the array to write.
     *               Must not exceed maxLength set for this characteristic.
     *
     * @return bool true set value success, false on error
     */
    bool setValue(const unsigned char value[], unsigned short length);

    /**
     * Get the property mask of the Characteristic
     *
     * @return unsigned char property mask of the Characteristic
     */
    unsigned char properties(void) const;

    /**
     * Get the (maximum) size of the Characteristic
     *
     * @return unsigned size of characateristic in bytes
     */
    unsigned short valueSize(void) const;

    /**
     * Get data pointer to the value of the Characteristic
     *
     * @return const unsigned char* pointer to the value of the Characteristic
     */
    const unsigned char* value(void) const;

    /**
     * Get the current length of the value of the Characteristic
     *
     * @return unsigned short size of characateristic value in bytes
     */
    unsigned short valueLength() const;

    unsigned char operator[] (int offset) const;

    /**
     * Has the value of the Characteristic been written by a central
     *
     * @return bool true is central has updated characteristic value, otherwise false
     */
    bool written(void);
    bool valueUpdated();

    /**
     * Is a central listening for notifications or indications of the Characteristic
     *
     * @return bool true is central is subscribed, otherwise false
     */
    bool subscribed(void);
    bool canNotify();
    bool canIndicate();
    
    bool subscribe(void);
    bool unsubscribe(void);

    /**
     * Provide a function to be called when events related to this Characteristic are raised
     *
     * @param[in] event Event type to set event handler for
     * @param[in] callback  Pointer to callback function to invoke when the event occurs.
     */
    void setEventHandler(BLECharacteristicEvent event, BLECharacteristicEventHandler callback);
    void setEventHandler(BLECharacteristicEvent event, BLECharacteristicEventHandlerOld callback);
    
    /**
     * @brief   Schedule the read request to read the characteristic in peripheral
     *
     * @param[in]   none
     *
     * @return  bool    Indicate the success or error
     *
     * @note  Only for central device
     */
    bool read();
    
    /**
     * @brief   Schedule the write request to update the characteristic in peripheral
     *
     * @param[in]   peripheral   The peripheral device that want to be updated
     * @param[in]   value       New value to set, as a byte array.  Data is stored in internal copy.
     * @param[in]   length      Length, in bytes, of valid data in the array to write.
     *                      Must not exceed maxLength set for this characteristic.
     *
     * @return  bool true set value success, false on error
     *
     * @note  none
     */
    bool write(const unsigned char value[], 
               uint16_t length);

    int descriptorCount() const;
    uint8_t discoverResponseProc(bt_conn_t *conn,
                                 const bt_gatt_attr_t *attr,
                                 bt_gatt_discover_params_t *params);
    
    bool discoverAttributes(BLEDevice* device);
    
    BLEDescriptorImp* descrptor(const bt_uuid_t* uuid);
    BLEDescriptorImp* descrptor(const char* uuid);
    BLEDescriptorImp* descrptor(int index);

protected:
    friend class BLEProfileManager;
    friend class BLEServiceImp;
    friend class BLECharacteristic;
    /**
     * Constructor for BLE Characteristic
     *
     * @param[in] characteristic    The characteristic
     * @param[in] bledevice         The device that has this characteristic
     */
    BLECharacteristicImp(BLECharacteristic& characteristic, const BLEDevice& bledevice);
    BLECharacteristicImp(const bt_uuid_t* uuid, 
                         unsigned char properties,
                         uint16_t handle,
                         const BLEDevice& bledevice);
    
    friend int profile_longflush_process(struct bt_conn *conn,
                                         const struct bt_gatt_attr *attr, 
                                         uint8_t flags);
    friend ssize_t profile_longwrite_process(struct bt_conn *conn,
                                     const struct bt_gatt_attr *attr,
                                     const void *buf, uint16_t len,
                                     uint16_t offset);
    
    int updateProfile(bt_gatt_attr_t *attr_start, int& index);
    
    int getAttributeCount();
    
    bool longCharacteristic();
    
    void setBuffer(const uint8_t value[], 
                   uint16_t length, 
                   uint16_t offset);
    void discardBuffer();
    void syncupBuffer2Value();
    
    /**
     * @brief   Get the characteristic value handle
     *
     * @param   none
     *
     * @return  none
     *
     * @note  Only for peripheral
     */
    uint16_t valueHandle(void);
    
    /**
     * @brief   Get characteristic configuration descriptor value handle
     *
     * @param   none
     *
     * @return  uint16_t        The value handle
     *                           0 is invalid handle
     *
     * @note  Only for peripheral
     */
    uint16_t cccdHandle(void);
    
    inline _bt_gatt_ccc_t* getCccCfg(void);
    inline bt_gatt_chrc_t* getCharacteristicAttValue(void);
    static bt_uuid_t* getCharacteristicAttributeUuid(void);
    static bt_uuid_t* getClientCharacteristicConfigUuid(void);
    
    /**
     * @brief   Get the characteristic permission
     *
     * @param   none
     *
     * @return  uint8_t The characteristic permission
     *
     * @note  none
     */
    uint8_t getPermission(void);
    
    /**
     * @brief   For central to discover the peripherial profile
     *
     * @param[in]   attr    The discover response
     *
     * @param[in]   params  The discover parameter that need to fill
     *
     * @return  none
     *
     * @note  Only for central
     */
    void discover(const bt_gatt_attr_t *attr,
                  bt_gatt_discover_params_t *params);
    
    /**
     * @brief   For central to discover the peripherial profile
     *
     * @param[in]   params  The discover parameter that need to fill
     *
     * @return  none
     *
     * @note  Only for central
     */
    void discover(bt_gatt_discover_params_t *params);
    
    /**
     * @brief   Get the subscribe parameter
     *
     * @param   none
     *
     * @return bt_gatt_subscribe_params_t * the subscribe parameter
     *
     * @note  Only for central
     */
   bt_gatt_subscribe_params_t* getSubscribeParams();

private:
    
    void setCCCDHandle(uint16_t handle);
    void setHandle(uint16_t handle);
    void _setValue(const uint8_t value[], uint16_t length, uint16_t offset);
    bool isClientCharacteristicConfigurationDescriptor(const bt_uuid_t* uuid);

private:
    // Those 2 UUIDs are used for define the characteristic.
    static bt_uuid_16_t _gatt_chrc_uuid;    // Characteristic UUID
    static bt_uuid_16_t _gatt_ccc_uuid;     // CCCD UUID
    
    unsigned short _value_size;
    unsigned short _value_length;
    unsigned char* _value;
    unsigned char* _value_buffer;
    bool _value_updated;

    uint16_t            _value_handle; // GATT client only
    uint16_t            _cccd_handle;  // GATT client only
    bt_gatt_discover_params_t _discover_params;// GATT client only
    
    bt_gatt_ccc_cfg_t   _ccc_cfg;
    _bt_gatt_ccc_t      _ccc_value;
    bt_gatt_chrc_t      _gatt_chrc;
    
    bt_gatt_attr_t *_attr_chrc_value; // GATT server only
    bt_gatt_attr_t *_attr_cccd; // GATT server only
    
    // For GATT Client to subscribe the Notification/Indication
    bt_gatt_subscribe_params_t _sub_params;
    bool        _subscribed;
    
    bool _reading;
    bt_gatt_read_params_t _read_params; // GATT read parameter
    
    typedef LinkNode<BLEDescriptorImp *>  BLEDescriptorLinkNodeHeader;
    typedef LinkNode<BLEDescriptorImp *>* BLEDescriptorNodePtr;
    typedef LinkNode<BLEDescriptorImp *>  BLEDescriptorNode;
        
    BLECharacteristicEventHandler _event_handlers[BLECharacteristicEventLast];
    BLECharacteristicEventHandlerOld  _oldevent_handlers[BLECharacteristicEventLast];
    BLEDescriptorLinkNodeHeader  _descriptors_header;
    BLEDevice   _ble_device;
};

#endif // _BLE_CHARACTERISTIC_H_INCLUDED
