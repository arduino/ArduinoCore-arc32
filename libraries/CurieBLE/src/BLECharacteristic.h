/*
  BLE Characteristic API
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

#ifndef ARDUINO_BLE_CHARACTERISTIC_H
#define ARDUINO_BLE_CHARACTERISTIC_H

#include "CurieBLE.h"

#include "BLEDevice.h"

enum BLECharacteristicEvent {
  BLEWritten = 0,
  BLESubscribed = 1,
  BLEUnsubscribed = 2,
  BLEValueUpdated = 3,
  BLECharacteristicEventLast
};

enum BLEProperty {
  BLEBroadcast            = 0x01,
  BLERead                 = 0x02,
  BLEWriteWithoutResponse = 0x04,
  BLEWrite                = 0x08,
  BLENotify               = 0x10,
  BLEIndicate             = 0x20
};

typedef void (*BLECharacteristicEventHandler)(BLEDevice bledev, BLECharacteristic characteristic);

typedef void (*BLECharacteristicEventHandlerOld)(BLECentral &central, BLECharacteristic &characteristic);

//#include "BLECharacteristicImp.h"

class BLECharacteristic: public BLEAttributeWithValue
{
public:
    BLECharacteristic();
    /**
     * @brief   Create a characteristic with specified value size
     *
     * @param   uuid        The UUID of the characteristic
     *
     * @param   properties  The properties of the characteristic
     *
     * @param   valueSize   The size of the characteristic data
     *
     * @return  none
     *
     * @note  none
     */
    BLECharacteristic(const char* uuid, 
                      unsigned char properties, 
                      unsigned short valueSize);
    
    /**
     * @brief   Create a characteristic with string value
     *
     * @param   uuid        The UUID of the characteristic
     *
     * @param   properties  The properties of the characteristic
     *
     * @param   value       The string of the characteristic data
     *
     * @return  none
     *
     * @note  The data length is string's size. Can't set a string that is longger
     *          than the this input
     */
    BLECharacteristic(const char* uuid, 
                      unsigned char properties, 
                      const char* value);

    BLECharacteristic(const BLECharacteristic&);

    virtual ~BLECharacteristic();

    /**
     * @brief   Is the characteristic valid
     *
     * @param   none
     *
     * @return  bool    true/false
     *
     * @note  Invalid characteristic is NULL pointer or all zero with UUID
     */
    virtual operator bool() const; // 

    /**
     * @brief   Get the characteristic's UUID string
     *
     * @param   none
     *
     * @return  const char*     The UUID string
     *
     * @note  none
     */
    const char* uuid() const;

    /**
     * @brief   Get the property mask of the characteristic
     *
     * @param   none
     *
     * @return  unsigned char       The property mask of the characteristic
     *
     * @note  none
     */
    unsigned char properties() const;

    /**
     * @brief   Get the maximum size of the value
     *
     * @param   none
     *
     * @return  int     The maximum size of the value
     *
     * @note  none
     */
    int valueSize() const;
    
    /**
     * @brief   Get the value buffer
     *
     * @param   none
     *
     * @return  const byte*     The value buffer
     *
     * @note  none
     */
    virtual const byte* value() const;
    
    /**
     * @brief   Get the current length of the value
     *
     * @param   none
     *
     * @return  int     The current length of the value string
     *
     * @note  TODO: How to handle if the data is RAW data? This API is danger
     */
    virtual int valueLength() const;
    
    /**
     * @brief   Get a byte of the value at the specified offset
     *
     * @param   none
     *
     * @return  byte    A byte of the value at the specified offset
     *
     * @note  none
     */
    virtual byte operator[] (int offset) const;
    BLECharacteristic& operator= (const BLECharacteristic& chrc);

    /**
     * Set the current value of the Characteristic
     *
     * @param[in] value  New value to set, as a byte array.  Data is stored in internal copy.
     * @param[in] length Length, in bytes, of valid data in the array to write.
     *               Must not exceed maxLength set for this characteristic.
     *
     * @return bool true set value success, false on error
     * @note    GATT Server only
     */
    bool setValue(const unsigned char value[], unsigned short length);

    /**
     * @brief   Write the value of the characteristic
     *
     * @param   value   The value buffer that want to write to characteristic
     *
     * @param   length  The value buffer's length
     *
     * @return  bool    true - Success, false - Failed
     *
     * @note  none
     */
    virtual bool writeValue(const byte value[], int length);
    
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
    bool writeValue(const byte value[], int length, int offset);
    
    /**
     * @brief   Write the value of the characteristic
     *
     * @param   value   The value string that want to write to characteristic
     *
     * @return  bool    true - Success, false - Failed
     *
     * @note  none
     */
    bool writeValue(const char* value);

    // peripheral mode
    bool broadcast(); // broadcast the characteristic value in the advertisement data
    
    // GATT server
    /**
     * @brief   Has the GATT client written a new value
     *
     * @param   none
     *
     * @return  bool    true - Written, false - Not changed
     *
     * @note  GATT server only. GATT client always return false.
     */
    bool written();
    
    /**
     * @brief   Is the GATT client subscribed
     *
     * @param   none
     *
     * @return  bool    true - Subscribed, false - Not subscribed
     *
     * @note  GATT server and client
     */
    bool subscribed();
    
    /**
     * @brief   Can a notification be sent to the GATT client
     *
     * @param   none
     *
     * @return  true - Yes, false - No
     *
     * @note  GATT server only
     */
    bool canNotify();
    
    /**
     * @brief   Can a indication be sent to the GATT client
     *
     * @param   none
     *
     * @return  true - Yes, false - No
     *
     * @note  GATT server only
     */
    bool canIndicate();
    
    // GATT
    /**
     * @brief   Can the characteristic be read (based on properties)
     *
     * @param   none
     *
     * @return  true - readable, false - None
     *
     * @note  none
     */
    bool canRead();
    
    /**
     * @brief   Can the characteristic be written (based on properties)
     *
     * @param   none
     *
     * @return  true - writable, false - None
     *
     * @note  none
     */
    bool canWrite();
    
    /**
     * @brief   Can the characteristic be subscribed to (based on properties)
     *
     * @param   none
     *
     * @return  true - Can be subscribed, false - No
     *
     * @note  What different with canUnsubscribe?
     */
    bool canSubscribe();
    
    /**
     * @brief   Can the characteristic be unsubscribed to (based on properties)
     *
     * @param   none
     *
     * @return  true - Can be unsubscribed, false - No
     *
     * @note  none
     */
    bool canUnsubscribe();

    /**
     * @brief   Read the characteristic value
     *
     * @param   none
     *
     * @return  bool    true - Success, false - Failed
     *
     * @note  Only for GATT client. Schedule read request to the GATT server
     */
    virtual bool read();
    
    /**
     * @brief   Write the charcteristic value
     *
     * @param   value   The value buffer that want to write to characteristic
     *
     * @param   length  The value buffer's length
     *
     * @return  bool    true - Success, false - Failed
     *
     * @note  Only for GATT client. Schedule write request to the GATT server
     */
    virtual bool write(const unsigned char* value, int length);
    
    /**
     * @brief   Subscribe to the characteristic
     *
     * @param   none
     *
     * @return  bool    true - Success, false - Failed
     *
     * @note  Only for GATT client. Schedule CCCD to the GATT server
     */
    bool subscribe();
    
    /**
     * @brief   Unsubscribe to the characteristic
     *
     * @param   none
     *
     * @return  bool    true - Success, false - Failed
     *
     * @note  Only for GATT client. Schedule CCCD to the GATT server
     */
    bool unsubscribe();


    /**
     * @brief   Read response or notification updated the characteristic
     *
     * @param   none
     *
     * @return  bool    true - Written, false - Not changed
     *
     * @note  GATT client only. GATT server always return false.
     */
    bool valueUpdated();
    
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
    
    /**
     * @brief   Get the number of descriptors the characteristic has
     *
     * @param   none
     *
     * @return  int     the number of descriptors the characteristic has
     *
     * @note  none
     */
    int descriptorCount() const;
    
    /**
     * @brief   Does the characteristic have a descriptor with the specified UUID
     *
     * @param   uuid        The descriptor's UUID
     *
     * @return  bool        true - Yes.     false - No
     *
     * @note  none
     */
    bool hasDescriptor(const char* uuid) const;
    
    /**
     * @brief   Does the characteristic have an nth descriptor with the specified UUID
     *
     * @param   uuid        The descriptor's UUID
     *
     * @param   index       The index of descriptor
     *
     * @return  bool        true - Yes.     false - No
     *
     * @note  none
     */
    bool hasDescriptor(const char* uuid, int index) const;
    
    /**
     * @brief   Get the nth descriptor of the characteristic
     *
     * @param   index   The index of descriptor
     *
     * @return  BLEDescriptor   The descriptor
     *
     * @note  none
     */
    BLEDescriptor descriptor(int index) const;
    
    /**
     * @brief   Get the descriptor with the specified UUID
     *
     * @param   uuid        The descriptor's UUID
     *
     * @return  BLEDescriptor   The descriptor
     *
     * @note  none
     */
    BLEDescriptor descriptor(const char * uuid) const;
    
    /**
     * @brief   Get the nth descriptor with the specified UUID
     *
     * @param   uuid        The descriptor's UUID
     *
     * @param   index       The index of descriptor
     *
     * @return  BLEDescriptor   The descriptor
     *
     * @note  none
     */
    BLEDescriptor descriptor(const char * uuid, int index) const;

    /**
     * @brief   Set an event handler (callback)
     *
     * @param   event           Characteristic event
     *
     * @param   eventHandler    The handler of characteristic
     *
     * @return  none
     *
     * @note  none
     */
    void setEventHandler(BLECharacteristicEvent event, 
                         BLECharacteristicEventHandler eventHandler);
    void setEventHandler(BLECharacteristicEvent event, 
                         BLECharacteristicEventHandlerOld eventHandler);
    
protected:
    friend class BLEDevice;
    friend class BLEService;
    friend class BLEServiceImp;
    /**
     * @brief   Create a characteristic with specified value size
     *
     * @param   characteristicImp  The implementation of the characteristic
     *
     * @param   bleDev      The peer BLE device
     *
     * @return  none
     *
     * @note  none
     */
    BLECharacteristic(BLECharacteristicImp *characteristicImp,
                      const BLEDevice *bleDev);
    
    /**
     * @brief   Create a characteristic with string value
     *
     * @param   uuid        The UUID of the characteristic
     *
     * @param   properties  The properties of the characteristic
     *
     * @param   value       The string of the characteristic data
     *
     * @param   bleDev      The peer BLE device
     *
     * @return  none
     *
     * @note  The data length is string's size. Can't set a string that is longger
     *          than the this input
     */
    //BLECharacteristic(const char* uuid, 
    //                  unsigned char properties, 
    //                  const char* value,
    //                  BLEDevice *bleDev);
    
    // For GATT
    void setBLECharacteristicImp(BLECharacteristicImp *characteristicImp);
    BLECharacteristicImp* fetchCharacteristicImp();
private:
    void _setValue(const uint8_t value[], uint16_t length);
    BLECharacteristicImp *getImplementation() const;
    
private:
    char    _uuid_cstr[37];  // The characteristic UUID
    BLEDevice _bledev;  // The GATT server BLE object. Only for GATT client to read/write
                        //  NULL - GATT server
                        //  None-NULL - GATT client
    BLECharacteristicImp *_internal;    // The real implementation of characteristic.
    BLECharacteristicImp *_chrc_local_imp;
    bool _broadcast;
protected:
    friend class BLECharacteristicImp;
    unsigned char _properties;      // The characteristic property
    
    unsigned short _value_size;       // The value size
    unsigned char* _value;          // The value. Will delete after create the _internal
    
    BLECharacteristicEventHandler _event_handlers[BLECharacteristicEventLast];  // Sid. Define the arr as in BLECharacteristicImp.h
    
    BLECharacteristicEventHandlerOld _oldevent_handlers[BLECharacteristicEventLast];
};

#endif

