/*
  BLE Descriptor API
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

#ifndef ARDUINO_BLE_DESCRIPTOR_H
#define ARDUINO_BLE_DESCRIPTOR_H

#include "CurieBLE.h"

#include "BLEDevice.h"

class BLEDescriptor
{
  public:
    BLEDescriptor();
    BLEDescriptor(const char* uuid, const unsigned char value[], unsigned short valueLength); // create a descriptor the specified uuid and value
    BLEDescriptor(const char* uuid, const char* value); // create a descriptor the specified uuid and string value

    BLEDescriptor(BLEDescriptorImp* descriptorImp, const BLEDevice *bleDev);
    BLEDescriptor(const BLEDescriptor&);
    BLEDescriptor& operator=(const BLEDescriptor&);

    virtual ~BLEDescriptor();

    const char* uuid() const;

    virtual const byte* value() const; // returns the value buffer
    virtual int valueLength() const; // returns the current length of the value
    
    virtual operator bool() const;  // is the descriptor valid (discovered from peripheral)

    unsigned char properties() const;
    int valueSize() const;
private:
    char    _uuid_cstr[37];  // The characteristic UUID
    BLEDevice _bledev; 
    
    unsigned char _properties;      // The characteristic property
    
    unsigned short _value_size;       // The value size
    unsigned char* _value;          // The value. Will delete after create the _internal


    // The API reserved for feature release
    // move here for temp
    
    /**
     * @brief   Write the value of the descriptor
     *
     * @param   value   The value buffer that want to write to descriptor
     *
     * @param   length  The value buffer's length
     *
     * @return  bool    true - Success, false - Failed
     *
     * @note  none
     */
    //virtual bool writeValue(const byte value[], int length);
    
    /**
     * @brief   Write the value of the descriptor
     *
     * @param   value   The value buffer that want to write to descriptor
     *
     * @param   length  The value buffer's length
     *
     * @param   offset  The offset in the descriptor's data
     *
     * @return  bool    true - Success, false - Failed
     *
     * @note  none
     */
    //bool writeValue(const byte value[], int length, int offset);
    
    /**
     * @brief   Write the value of the descriptor
     *
     * @param   value   The value string that want to write to descriptor
     *
     * @return  bool    true - Success, false - Failed
     *
     * @note  none
     */
    //bool writeValue(const char* value);
    //virtual byte operator[] (int offset) const; // returns a byte of the value at the specified offset

    // GATT client Write the value of the descriptor
    //virtual bool write(const byte value[], int length);
    //bool write(const byte value[], int length, int offset);
    //bool write(const char* value);
    //bool read();
};

#endif
