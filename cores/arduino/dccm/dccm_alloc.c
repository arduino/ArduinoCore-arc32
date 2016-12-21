/*
Copyright (c) 2016 Intel Corporation.  All right reserved.

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
Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA

*/

#include "dccm_alloc.h"

uint16_t dccm_index = 0;

#ifdef __cplusplus
 extern "C" {
#endif

void* dccm_malloc(uint16_t size)
{
    if((size + dccm_index) > DCCM_SIZE)
    {
        return 0;
    }
    
    void* addr = (void*)(DCCM_START + dccm_index);
    dccm_index += size;
    return addr;
}

void *dccm_memalign(uint16_t size)
{
  if ((dccm_index +3) > DCCM_SIZE)
    return 0;

  dccm_index = (dccm_index + 3) & ~((uint16_t)0x3);  /* 4 byte addr alignment */
  return dccm_malloc(size);
}

#ifdef __cplusplus
}
#endif
