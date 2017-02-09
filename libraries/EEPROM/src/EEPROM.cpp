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

#include "EEPROM.h"

EEPROMClass EEPROM;

void CurieClear()
{
  //erase the 2k bytes of the eeprom section inside the otp area
  *(uint32_t*)(ROM_WR_CTRL) = 0x4002;
  //wait for erase to be complete
  #if 0
  // TODO: wait for FLASH_STTS.ER_DONE to be set to 1
  while(((*(uint32_t*)FLASH_STTS) & 0x01) == 0) {
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
    //shift left 2 bits to store offset into bits 19:2 (WR_ADDR)
    rom_wr_ctrl = (address)<<2;
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
  //shift left 2 bits to store offset into bits 19:2 (WR_ADDR)
  rom_wr_ctrl = (address)<<2;
  rom_wr_ctrl |= 0x00000001; //set (WR_REQ) bit
  *(uint32_t*)(ROM_WR_CTRL) = rom_wr_ctrl;

  delay(3); //give it enough time to finish writing
}
