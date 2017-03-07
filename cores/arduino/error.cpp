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

#include "error.h"

void error_continue()
{
    error_continue(1);
}

void error_continue(uint8_t error_code)
{
    uint32_t exc_addr = aux_reg_read(ARC_V2_EFA);
    uint32_t ecr = aux_reg_read(ARC_V2_ECR);

    pr_error(0, "Exception vector: 0x%x, cause code: 0x%x, parameter 0x%x\n",
    ARC_V2_ECR_VECTOR(ecr),
    ARC_V2_ECR_CODE(ecr),
    ARC_V2_ECR_PARAMETER(ecr));
    pr_error(0, "Address 0x%x\n", exc_addr);
    shared_data->error_code = error_code;
}

void error_halt()
{
    error_halt(1);
}

void error_halt(uint8_t error_code)
{
    error_continue(error_code);
    __asm__("flag 0x01") ; /* Halt the CPU */
}
