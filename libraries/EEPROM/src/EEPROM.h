/*
  EEPROM.h - EEPROM library
  Original Copyright (c) 2006 David A. Mellis.  All right reserved.
  New version by Christopher Andrews 2015.
  Curie porting by Intel and Arduino LLC - 2016
  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.
  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.
  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#ifndef EEPROM_h
#define EEPROM_h

#define ROM_WR_CTRL 0xb0100004
#define ROM_WR_DATA 0xb0100008
#define FLASH_STTS  0xb0000014
#define EEPROM_ADDR 0xfffff000
#define EEPROM_OFFSET 0x00001000
#define CTRL_REG    0xb0000018

#define EEPROM_SIZE 2048 //EEPROM size in bytes


#include <inttypes.h>
#include "Arduino.h"

/* Curie specific implementation of "atomic" read8 and write8 on OTP flash storage */

void CurieClear()
{
  //erase the 2k bytes of the eeprom section inside the otp area
  *(uint32_t*)(ROM_WR_CTRL) = 0x4002;
  //wait for erase to be complete
  #if 0
  while(((*(uint32_t*)FLASH_STTS) & 0x01) == 0) { // TODO: wait for FLASH_STTS.ER_DONE to be set to 1
    delay(1);
  }
  #endif
  delay(5);
}

void CurieRestoreMemory(uint32_t* buffer, uint32_t size)
{
  uint32_t rom_wr_ctrl = 0;
  uint32_t address;

  for (uint32_t i=0; i<size; i++) {

    uint32_t data32 = buffer[i];
    if (data32 == 0xFFFFFFFF) {
      continue;
    }

    //store data into ROM_WR_DATA register
    *(uint32_t*)(ROM_WR_DATA) = data32;
    address = i * 4 + EEPROM_OFFSET;
    rom_wr_ctrl = (address)<<2; //shift left 2 bits to store offset into bits 19:2 (WR_ADDR)
    rom_wr_ctrl |= 0x00000001; //set (WR_REQ) bit
    *(uint32_t*)(ROM_WR_CTRL) = rom_wr_ctrl;

    delay(3); //give it enough time to finish writing
  }
}

uint8_t CurieRead8(uint32_t address)
{
  if((address > 0x7FF))
  {
    return 0;
  }
  int offset = address%4;
  uint32_t value = *(uint32_t*)(EEPROM_ADDR+(address/4)*4);
  value = (value >> ((3-offset)*8)) & 0xFF;
  return (uint8_t)value;
}

uint32_t CurieRead32(uint32_t address)
{
  if((address > 0x7FF))
  {
    return 0;
  }
  uint32_t value = *(uint32_t*)(EEPROM_ADDR+(address/4)*4);
  return value;
}

void CurieWrite8(uint32_t address, uint8_t data)
{
  //make sure address is valid
  if((address > 0x7FF))
  {
    return;
  }

  uint8_t currentValue = CurieRead8(address);
  //only do something if value is different from what is currently stored
  if(currentValue==data)
  {
    return;
  }

  uint32_t currentDword = CurieRead32(address);

  int offset = address%4;

  uint32_t data32 = (currentDword & ~(uint32_t)(0xFF << ((3-offset)*8)));
  data32 = data32 | (data << ((3-offset)*8));

  if (currentValue != 0xFF) {
    uint32_t dump[EEPROM_SIZE/4];
    memcpy(dump, (uint32_t *)EEPROM_ADDR, EEPROM_SIZE);
    dump[(address >> 2)] = data32;
    CurieClear();
    CurieRestoreMemory((uint32_t *)dump, EEPROM_SIZE/sizeof(uint32_t));
    return;
  }

  uint32_t rom_wr_ctrl = 0;

  //store data into ROM_WR_DATA register
  *(uint32_t*)(ROM_WR_DATA) = data32;
  address = ((address >> 2) << 2) + EEPROM_OFFSET;
  rom_wr_ctrl = (address)<<2; //shift left 2 bits to store offset into bits 19:2 (WR_ADDR)
  rom_wr_ctrl |= 0x00000001; //set (WR_REQ) bit
  *(uint32_t*)(ROM_WR_CTRL) = rom_wr_ctrl;

  delay(3); //give it enough time to finish writing
}

/***
    EERef class.

    This object references an EEPROM cell.
    Its purpose is to mimic a typical byte of RAM, however its storage is the EEPROM.
    This class has an overhead of two bytes, similar to storing a pointer to an EEPROM cell.
***/

struct EERef{

    EERef( const int index )
        : index( index )                 {}

    //Access/read members.
    uint8_t operator*() const            { return CurieRead8( (uint32_t) index ); }
    operator const uint8_t() const       { return **this; }

    //Assignment/write members.
    EERef &operator=( const EERef &ref ) { return *this = *ref; }
    EERef &operator=( uint8_t in )       { return CurieWrite8( (uint32_t) index, in ), *this;  }
    EERef &operator +=( uint8_t in )     { return *this = **this + in; }
    EERef &operator -=( uint8_t in )     { return *this = **this - in; }
    EERef &operator *=( uint8_t in )     { return *this = **this * in; }
    EERef &operator /=( uint8_t in )     { return *this = **this / in; }
    EERef &operator ^=( uint8_t in )     { return *this = **this ^ in; }
    EERef &operator %=( uint8_t in )     { return *this = **this % in; }
    EERef &operator &=( uint8_t in )     { return *this = **this & in; }
    EERef &operator |=( uint8_t in )     { return *this = **this | in; }
    EERef &operator <<=( uint8_t in )    { return *this = **this << in; }
    EERef &operator >>=( uint8_t in )    { return *this = **this >> in; }
    
    EERef &update( uint8_t in )          { return  in != *this ? *this = in : *this; }
    
    /** Prefix increment/decrement **/
    EERef& operator++()                  { return *this += 1; }
    EERef& operator--()                  { return *this -= 1; }
    
    /** Postfix increment/decrement **/
    uint8_t operator++ (int){ 
        uint8_t ret = **this;
        return ++(*this), ret;
    }

    uint8_t operator-- (int){ 
        uint8_t ret = **this;
        return --(*this), ret;
    }
    
    int index; //Index of current EEPROM cell.
};

/***
    EEPtr class.
    
    This object is a bidirectional pointer to EEPROM cells represented by EERef objects.
    Just like a normal pointer type, this can be dereferenced and repositioned using 
    increment/decrement operators.
***/

struct EEPtr{

    EEPtr( const int index )
        : index( index )                {}
        
    operator const int() const          { return index; }
    EEPtr &operator=( int in )          { return index = in, *this; }
    
    //Iterator functionality.
    bool operator!=( const EEPtr &ptr ) { return index != ptr.index; }
    EERef operator*()                   { return index; }
    
    /** Prefix & Postfix increment/decrement **/
    EEPtr& operator++()                 { return ++index, *this; }
    EEPtr& operator--()                 { return --index, *this; }
    EEPtr operator++ (int)              { return index++; }
    EEPtr operator-- (int)              { return index--; }

    int index; //Index of current EEPROM cell.
};

/***
    EEPROMClass class.
    
    This object represents the entire EEPROM space.
    It wraps the functionality of EEPtr and EERef into a basic interface.
    This class is also 100% backwards compatible with earlier Arduino core releases.
***/

struct EEPROMClass{

    //Basic user access methods.
    EERef operator[]( const int idx )    { return idx; }
    uint8_t read( int idx )              { return EERef( idx ); }
    void write( int idx, uint8_t val )   { (EERef( idx )) = val; }
    void update( int idx, uint8_t val )  { EERef( idx ).update( val ); }
    
    //STL and C++11 iteration capability.
    EEPtr begin()                        { return 0x00; }
    EEPtr end()                          { return length(); } //Standards requires this to be the item after the last valid entry. The returned pointer is invalid.
    uint16_t length()                    { return EEPROM_SIZE; }
    
    //Functionality to 'get' and 'put' objects to and from EEPROM.
    template< typename T > T &get( int idx, T &t ){
        EEPtr e = idx;
        uint8_t *ptr = (uint8_t*) &t;
        for( int count = sizeof(T) ; count ; --count, ++e )  *ptr++ = *e;
        return t;
    }
    
    template< typename T > const T &put( int idx, const T &t ){
        EEPtr e = idx;
        const uint8_t *ptr = (const uint8_t*) &t;
        for( int count = sizeof(T) ; count ; --count, ++e )  (*e).update( *ptr++ );
        return t;
    }
};

static EEPROMClass EEPROM;
#endif