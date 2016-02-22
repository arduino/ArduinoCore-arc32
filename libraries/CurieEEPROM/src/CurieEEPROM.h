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

#define ROM_WR_CTRL	0xb0100004
#define ROM_WR_DATA	0xb0100008
#define FLASH_STTS	0xb0000014
#define EEPROM_ADDR 0xfffff000
#define EEPROM_OFFSET 0x00001000
#define CTRL_REG    0xb0000018

#define EEPROM_SIZE 512 //EEPROM size in dwords


#include <inttypes.h>
#include "Arduino.h"

class CurieEEPROM 
{
public:
  CurieEEPROM() 
  {

  }
  void clear();
  void write(uint32_t address, uint32_t data);
  void write8(uint32_t address, uint8_t data);
  void update(uint32_t addr, uint32_t value);
  void update8(uint32_t addr, uint8_t value);
  uint32_t read(uint32_t addr);
  uint8_t read8(uint32_t addr);
  
  uint32_t& operator[](int i);
  
  int length();
  int begin();
  int end();
  
  //Functionality to 'get' and 'put' objects to and from EEPROM.
  template< typename T > T &get(uint32_t addr, T &t)
  {
    //make sure address is within valid range
    if(addr > 0x1FF)
    {
      return t;
    }
    
    int byteCount = sizeof(T);
    //return if object size is too big
    if(addr + byteCount > 0x7FC)
    {
      return t;
    }

    //allign address to 32-bit addressing
    addr*=sizeof(uint32_t);

    byte *bytes = to_bytes(t);
    for(int i = 0; i < byteCount; i++)
    {
      bytes[i] = read8(addr+i);
    }
    from_bytes(bytes, t);
    delete bytes;
    return t;
  }
  template< typename T > T put(uint32_t addr, T t)
  {
    //make sure address is within valid range
    if(addr > 0x1FF)
    {
      return t;
    }
    uint32_t rom_wr_ctrl = 0;
    int byteCount = sizeof(T);

    //return if object size is too big
    if(addr + byteCount > 0x7FC)
    {
      return t;
    }
    
    size_t size = (sizeof(T)/4 + (((sizeof(T)%4)>1) ? 1 : 0));
    uint32_t *dwords = to_dwords(t);
    
    //check if address is empty and available for writing new data
    bool blockAvailable = true;
    for(int i =0; i < size; i++)
    {
      uint32_t data32 = read(addr+i);
      if(data32 != 0xFFFFFFFF)
      {
        blockAvailable = false;
      }
    }
    if(blockAvailable)
    {
      for(int i = 0; i<size; i++)
      {
        write(addr+i, dwords[i]);
      }
    }
    else
    {
      //read entire 2k of data
      uint32_t blockdata[EEPROM_SIZE/4];
      for(int i = 0; i < EEPROM_SIZE/4; i++)
      {
        blockdata[i] = read(i*sizeof(uint32_t));
      }
      
      //update blockdata buffer
      for(int i = 0; i<size; i++)
      {
        blockdata[addr/4 + i] = dwords[i];
      }
      
      //clear EEPROM area
      clear();
      
      //write back all data with update data on passed address
      for(int i = 0; i < EEPROM_SIZE/4; i++)
      {
        //store data into ROM_WR_DATA register
        *(uint32_t*)(ROM_WR_DATA) = blockdata[i];
        addr = EEPROM_OFFSET + i*sizeof(uint32_t);
        rom_wr_ctrl = (addr)<<2; //shift left 2 bits to store offset into bits 19:2 (WR_ADDR)
        rom_wr_ctrl |= 0x00000001; //set (WR_REQ) bit
        *(uint32_t*)(ROM_WR_CTRL) = rom_wr_ctrl;
        delay(3); //give it enough time to finish writing
      }
    }
    delete dwords;
    return t;
  }

private:
  template< typename T > byte* to_bytes(const T& object)
  {
    size_t buffer_size = sizeof(object);
    byte *buffer = new byte[buffer_size];
    memcpy(buffer, &object, buffer_size);

    return buffer;
  }
  
  template< typename T > uint32_t* to_dwords(const T& object)
  {
    size_t buffer_size = sizeof(object);
    uint32_t *buffer = new uint32_t[buffer_size];
    memcpy(buffer, &object, buffer_size);
 
    return buffer;
  }
  
  template< typename T > T& from_bytes(byte* bytes, T& object )
  {
    memcpy(&object, bytes, sizeof(object));
    return object;
  } 
};

extern CurieEEPROM EEPROM;
