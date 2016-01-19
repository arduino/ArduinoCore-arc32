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

#define EEPROM_SIZE 2048 //EEPROM size in bytes


#include <inttypes.h>
#include "Arduino.h"
#include <array>

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
    //make sure address is valid
    if((addr > 0x7FC) || (addr%4))
    {
      return t;
    }
    int byteCount = sizeof(T);
    //return if size of object is greater than size of EEPROM
    if(byteCount > EEPROM_SIZE)
    {
      return t;
    }
    auto bytes = to_bytes(t); 
    for(int i = 0; i < byteCount; i++)
    {
      bytes[i] = read8(addr+i);
    }
    from_bytes(bytes, t);
    return t;
  }
  template< typename T > T put(uint32_t addr, T t)
  {
    //make sure address is valid
    if((addr > 0x7FC) || (addr%4))
    {
      return t;
    }
    uint32_t rom_wr_ctrl = 0;
    int byteCount = sizeof(T);
    //return if size of object is greater than size of EEPROM
    if(byteCount > EEPROM_SIZE)
    {
      return t;
    }
    const auto dwords = to_dwords(t);
    
    //check if address is empty and available for writing new data
    bool blockAvailable = true;
    for(int i =0; i < (sizeof(T)/4 + (((sizeof(T)%4)>1) ? 1 : 0)); i++)
    {
      uint32_t data32 = read(addr+i*sizeof(uint32_t));
      if(data32 != 0xFFFFFFFF)
      {
        blockAvailable = false;
      }
    }
    if(blockAvailable)
    {
      for(int i = 0; i<sizeof(dwords)/4; i++)
      {
        write(addr+i*sizeof(uint32_t), dwords[i]);
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
      for(int i = 0; i<sizeof(dwords)/4; i++)
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
    return t;
  }

private:
  template< typename T > std::array< byte, sizeof(T) >  to_bytes(const T& object)
  {
    std::array< byte, sizeof(T) > bytes ;

    const byte* begin = reinterpret_cast< const byte* >( std::addressof(object)) ;
    const byte* end = begin + sizeof(T) ;
    std::copy( begin, end, std::begin(bytes)) ;

    return bytes;
  }
  
  template< typename T > std::array< uint32_t, (sizeof(T)/4 + (((sizeof(T)%4)>1) ? 1 : 0)) >  to_dwords( const T& object )
  {
    std::array< uint32_t, (sizeof(T)/4 + (((sizeof(T)%4)>1) ? 1 : 0)) > dwords;

    const uint32_t* begin = reinterpret_cast< const uint32_t* >( std::addressof(object)) ;
    const uint32_t* end = begin + (sizeof(T)/4 + (((sizeof(T)%4)>1) ? 1 : 0));
    std::copy( begin, end, std::begin(dwords));

    return dwords;
  }
  
  template< typename T > T& from_bytes(std::array< byte, sizeof(T) >& bytes, T& object )
  {
    byte* begin_object = reinterpret_cast< byte* >( std::addressof(object) ) ;
    std::copy( std::begin(bytes), std::end(bytes), begin_object ) ;
    
    return object;
  } 
};

extern CurieEEPROM EEPROM;
