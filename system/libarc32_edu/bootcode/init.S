#include "aux_regs.h"

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
//    mov sp, @_DefaultStack + STACK_SIZE
    mov sp, @__stack_start
    /* Enable instruction cache */
    mov r0, 0
    sr r0, [ARC_V2_IC_CTRL]
    /* Configure the interrupt priority threshold of the ARC core to 3 */
    seti 0x23
    /* Jump to C init function */
    j @_main

.balign 4
_do_fault:
    /* Set halt flag */
    flag 0x01
    nop
    nop
    nop
    /* loop forever */
    j @_do_fault
    nop
