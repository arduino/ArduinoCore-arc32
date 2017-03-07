/*
Copyright (c) 2017 Intel Corporation.  All right reserved.

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

#ifndef _Error_h
#define _Error_h

#include <stdint.h>
#include "os/os.h"
#include "infra/log.h"
#include "aux_regs.h"
#include "platform.h"

extern void error_halt();

extern void error_halt(uint8_t error_code);

extern void error_continue();

extern void error_continue(uint8_t error_code);

#endif
