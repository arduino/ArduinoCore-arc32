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

#include <stdint.h>

#define DCCM_START  0x80000000
#define DCCM_SIZE 8192

#ifndef _DCCM_ALLOC_
#define _DCCM_ALLOC_

#ifdef __cplusplus
 extern "C" {
#endif

void* dccm_malloc(uint16_t size);

void *dccm_memalign(uint16_t size);

#ifdef __cplusplus
}
#endif


#endif
