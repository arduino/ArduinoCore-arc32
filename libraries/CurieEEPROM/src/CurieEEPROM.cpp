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
#include "CurieEEPROM.h"

CurieEEPROM EEPROM;

void CurieEEPROM::clear()
{
  //erase the 2k bytes of the eeprom section inside the otp area
  *(uint32_t*)(ROM_WR_CTRL) = 0x4002;
  //wait for erase to be complete
  //TODO: while(((*(uint32_t*)FLASH_STTS) & 0x01) == 0) //wait for FLASH_STTS.ER_DONE to be set to 1
  delay(1000);
}

void CurieEEPROM::write(uint32_t address, uint32_t data)
{
  //make sure address is within valid range
  if(address > 0x1FF)
  {
    return;
  }
  
  uint32_t currentValue = read(address);
  //only do something if value is different from what is currently stored
  if(currentValue==data)
  {
    return;
  }
  
  //allign address to 32-bit addressing
  address*=sizeof(uint32_t);
  
  uint32_t rom_wr_ctrl = 0;
  //make sure address is valid
  if((address > 0x7FC) || (address%4))
  {
    return;
  }

  //check if address is available for writing a new value. If not erase the whole 2K block and re-write the rest of the data
  if(currentValue!=0xFFFFFFFF)
  {
    //read entire 2k of data
    uint32_t blockdata[EEPROM_SIZE/4];
    for(int i = 0; i < EEPROM_SIZE/4; i++)
    {
      blockdata[i] = read(i*sizeof(uint32_t));
    }
    
    //update blockdata buffer
    int blockIndex = address/4;
    blockdata[blockIndex] = data;
    
    //clear EEPROM area
    clear();
    
    //write back all data with update data on passed address
    for(int i = 0; i < EEPROM_SIZE/4; i++)
    {
      //store data into ROM_WR_DATA register
      *(uint32_t*)(ROM_WR_DATA) = blockdata[i];
      address = EEPROM_OFFSET + i*sizeof(uint32_t);
      rom_wr_ctrl = (address)<<2; //shift left 2 bits to store offset into bits 19:2 (WR_ADDR)
      rom_wr_ctrl |= 0x00000001; //set (WR_REQ) bit
      *(uint32_t*)(ROM_WR_CTRL) = rom_wr_ctrl;
      delay(3); //give it enough time to finish writing
    }
  }
  else
  {
    //store data into ROM_WR_DATA register
    *(uint32_t*)(ROM_WR_DATA) = data;
    address += EEPROM_OFFSET;
    rom_wr_ctrl = (address)<<2; //shift left 2 bits to store offset into bits 19:2 (WR_ADDR)
    rom_wr_ctrl |= 0x00000001; //set (WR_REQ) bit
    *(uint32_t*)(ROM_WR_CTRL) = rom_wr_ctrl;
    delay(3); //give it enough time to finish writing
  }
}

void CurieEEPROM::write8(uint32_t address, uint8_t data)
{
  uint8_t currentValue = read8(address);
  //only do something if value is different from what is currently stored
  if(currentValue==data)
  {
    return;
  }
  
  uint32_t rom_wr_ctrl = 0;
  //make sure address is valid
  if((address > 0x7FF))
  {
    return;
  }
  
  //check if address is available for writing a new value. If not erase the whole 2K block and re-write the rest of the data
  if(currentValue!=0xFF)
  {
    //read entire 2k of data
    uint32_t blockdata[EEPROM_SIZE/4];
    for(int i = 0; i < EEPROM_SIZE/4; i++)
    {
      blockdata[i] = read(i*sizeof(uint32_t));
    }
    
    //update blockdata buffer
    int offset = address%4;
    int blockIndex = address/4;
    uint32_t data32 = blockdata[blockIndex];
    uint8_t data_array[sizeof(uint32_t)];
    memcpy(data_array, &data32, sizeof(uint32_t));
    data_array[offset] = data;
    memcpy(&data32, &data_array, sizeof(uint32_t));
    blockdata[blockIndex] = data32;
    
    //clear EEPROM area
    clear();
    
    //write back all data with update data on passed address
    for(int i = 0; i < EEPROM_SIZE/4; i++)
    {
      //store data into ROM_WR_DATA register
      *(uint32_t*)(ROM_WR_DATA) = blockdata[i];
      address = EEPROM_OFFSET + i*sizeof(uint32_t);
      rom_wr_ctrl = (address)<<2; //shift left 2 bits to store offset into bits 19:2 (WR_ADDR)
      rom_wr_ctrl |= 0x00000001; //set (WR_REQ) bit
      *(uint32_t*)(ROM_WR_CTRL) = rom_wr_ctrl;
      delay(3); //give it enough time to finish writing
    }
  }
  else
  {
    int offset = address%4;
    uint32_t data32 = 0xffffff00 + (uint32_t)data;
    for(int i = 0; i < offset; i++)
    {
      data32<<=8;
      data32+=0x000000ff;
    }
    //store data into ROM_WR_DATA	register
    *(uint32_t*)(ROM_WR_DATA) = data32;
    address -= offset;
    address += EEPROM_OFFSET;
    rom_wr_ctrl = (address)<<2; //shift left 2 bits to store offset into bits 19:2 (WR_ADDR)
    rom_wr_ctrl |= 0x00000001; //set (WR_REQ) bit
    *(uint32_t*)(ROM_WR_CTRL) = rom_wr_ctrl;
    delay(3); //give it enough time to finish writing
  } 
}

void CurieEEPROM::update(uint32_t addr, uint32_t value)
{
  write(addr, value);
}
void CurieEEPROM::update8(uint32_t addr, uint8_t value)
{
  write8(addr, value);
}
  
uint32_t CurieEEPROM::read(uint32_t address)
{
  //make sure address is within valid range
  if(address > 0x1FF)
  {
    return 0;
  }
  
  //allign address to 32-bit addressing
  address*=sizeof(uint32_t);
  
  address +=  EEPROM_ADDR;
  return *(uint32_t*)(address);
}

uint8_t CurieEEPROM::read8(uint32_t address)
{
  if((address > 0x7FF))
  {
    return 0;
  }
  int offset = address%4;
  uint32_t value = *(uint32_t*)(EEPROM_ADDR+(address-offset));
  for(int i = 0; i < offset; i++)
  {
    value>>=8;
  }
  value &= 0x000000FF;
  return (uint8_t)value;
}

uint32_t& CurieEEPROM::operator[](int i)
{
  return *(uint32_t*)(EEPROM_ADDR+i*sizeof(uint32_t));
}

int CurieEEPROM::length()
{
  return EEPROM_SIZE;
}

int CurieEEPROM::begin()
{
  return 0x00;
}

int CurieEEPROM::end()
{
  return length();
}
