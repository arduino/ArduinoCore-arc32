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

    /**
     * @brief   Get the descriptor's UUID string
     *
     * @param   none
     *
     * @return  const char*     The UUID string
     *
     * @note  none
     */
    const char* uuid() const;

    
    /**
     * @brief   Get the value of descriptor
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
     * @note  none
     */
    virtual int valueLength() const;
    
    /**
     * @brief   Is the descriptor valid
     *
     * @param   none
     *
     * @return  bool    true/false
     *
     * @note  none
     */
    virtual operator bool() const;
    
    /**
     * @brief   Read the descriptor value
     *
     * @param   none
     *
     * @return  bool    true - Success, false - Failed
     *
     * @note  Only for GATT client. Schedule read request to the GATT server
     */
    bool read();

    /**
     * @brief   Get the property mask of the descriptor
     *
     * @param   none
     *
     * @return  unsigned char       The property mask of the descriptor
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
private:
    char    _uuid_cstr[37];  // The characteristic UUID
    BLEDevice _bledev; 
    
    unsigned char _properties;      // The characteristic property
    
    unsigned short _value_size;     // The value size
    unsigned char* _value;          // The value. Will delete after create the _internal
    BLEDescriptorImp *_internal;    // The real implementation of Descriptor
};

#endif
