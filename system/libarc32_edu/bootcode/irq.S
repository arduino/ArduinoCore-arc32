/*
Copyright (c) 2015 Intel Corporation.  All right reserved.

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
#include "aux_regs.h"

.globl _do_isr
.type _do_isr,%function

.globl _IsrTable
.type _IsrTable,%object

.text
.balign 4
_do_isr:
    lr r0, [ARC_V2_ICAUSE]
    sub r0, r0, 16

    mov r1, _IsrTable
    add3 r0, r1, r0   /* table entries are 8-bytes wide */

    ld r1, [r0, 4] /* ISR into r1 */
    jl_s.d [r1]
    ld_s r0, [r0] /* delay slot: ISR parameter into r0  */

    /* back from ISR */
    rtie
    nop
