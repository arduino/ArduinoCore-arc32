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

.globl _do_fault
.type _do_fault,%function

.globl _exit_halt
.type _exit_halt,%function

.globl _do_isr
.type _do_isr,%function

.section .int_vector_table
.balign 4
_start:
    .word _do_reset
.rep 15
    .word _do_fault
.endr
.rep 52  
    .word _do_isr
.endr

.section .text
.balign 4
_do_reset:
    /* Ensure interrupts are initially disabled */
    clri
    /* Switch to Interrupt Vector Table defined above*/
    mov r0, @_start
    sr r0, [ARC_V2_IRQ_VECT_BASE]
    /* Set up stack pointer */
    mov sp, @__stack_start
    /* Enable instruction cache */
    mov r0, 0x20
    sr r0, [ARC_V2_IC_CTRL]
    /* Jump to C init function */
    j @_main

/*
 * Default, generic exception handler:
 * Sets the halt flag and spins for ever.
 */
.balign 4
_do_fault:
_exit_halt:
    /* Set halt flag
    flag 0x0 */
    nop
    j @_Fault
    nop
    nop
    /* loop forever */
    j @_do_fault
    nop

/*
 * The default, generic hardware IRQ handler.
 * It only decodes the source of IRQ and calls the appropriate handler
 */
.balign 4
_do_isr:
    /* Init the SP for the FIRQ bank */
    mov sp, @__firq_stack_start
    /* Save the loop count related register */
    mov r0, lp_count
    lr r1, [ARC_V2_LP_START]
    lr r2, [ARC_V2_LP_END]
    push_s r0
    push_s r1
    push_s r2
    /* Read IRQ Cause */
    lr r0, [ARC_V2_ICAUSE]
    sub r0, r0, 16

    mov r1, _IsrTable
    add2 r0, r1, r0   /* table entries are 4-bytes wide */

    ld r1, [r0] /* ISR into r1 */
    jl_s.d [r1]
    nop

    /* Restore the loop count related register */
    pop_s r2
    pop_s r1
    pop_s r0
	
    sr r2, [ARC_V2_LP_END]
    sr r1, [ARC_V2_LP_START]
    mov lp_count, r0
    rtie

