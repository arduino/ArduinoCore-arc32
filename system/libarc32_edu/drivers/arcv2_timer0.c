/* arcv2_timer0.c - ARC timer 0 device driver */

/*
 * Copyright (c) 2014 Wind River Systems, Inc.
 *
 * The right to copy, distribute, modify or otherwise make use
 * of this software may be licensed only pursuant to the terms
 * of an applicable Wind River license agreement.
 */

/*modification history
modification history
--------------------
03Nov14,j_b  written
*/

/*
DESCRIPTION
This module implements a VxMicro device driver for the ARCv2 processor timer 0
and provides the standard "system clock driver" interfaces.

\INTERNAL IMPLEMENTATION DETAILS
The ARCv2 processor timer provides a 32-bit incrementing, wrap-to-zero counter. 

The device driver is also part of a nanokernel-only system, but omits more
complex capabilities (such as tickless idle support) that are only used in
conjunction with a microkernel.
*/

#include "arcv2_timer0.h"
#include "conf.h"
#include "interrupt.h"

#define ONE_MILLISECOND	    ARCV2_TIMER0_CLOCK_FREQ/1000


/* globals */

uint32_t volatile timer0_overflows = 0x00;


/*******************************************************************************
*
* arcv2_timer0_enable - enable the timer with the given limit/countup value
*
* This routine sets up the timer for operation by:
* - setting value to which the timer will count up to;
* - setting the timer's start value to zero; and
* - enabling interrupt generation.
*
* RETURNS: N/A
*
* \NOMANUAL
*/

static inline __attribute__((always_inline))
void arcv2_timer0_enable(uint32_t count)
{
    /* ensure that the timer will not generate interrupts */
    aux_reg_write(ARC_V2_TMR0_CONTROL, 0);
    /* write the limit value */
    aux_reg_write(ARC_V2_TMR0_LIMIT, count);
    /* count only when not halted for debug and enable interrupts */
    aux_reg_write(ARC_V2_TMR0_CONTROL,
                  ARC_V2_TMR_CTRL_NH | ARC_V2_TMR_CTRL_IE);
    /* clear the count value */
    aux_reg_write(ARC_V2_TMR0_COUNT, 0);
}

/*******************************************************************************
*
* _arcv2_timer0_int_handler - system clock periodic tick handler
*
* This routine handles the system clock periodic tick interrupt.
* It increments number of milliseconds since sketch begun.
*
* RETURNS: N/A
*
* \NOMANUAL
*/
void _arcv2_timer0_int_handler(void)
{

    /* clear the interrupt (by writing 0 to IP bit of the control register) */
    aux_reg_write(ARC_V2_TMR0_CONTROL,
                  ARC_V2_TMR_CTRL_NH | ARC_V2_TMR_CTRL_IE);
    /* Increment number of Timer0 overflows */
    timer0_overflows++;
}

/*******************************************************************************
*
* timer0_driver_init - initialize and enable the system clock
*
* This routine is used to program the ARCv2 timer to deliver interrupts at the
* 1 millisecond rate specified via the ONE_MILLISECOND macro.
*
* RETURNS: N/A
*/
void timer0_driver_init(void)
{
    /* connect specified routine/parameter to the timer 0 interrupt vector */
    interrupt_connect(ARCV2_IRQ_TIMER0, _arcv2_timer0_int_handler, 0);
    /* configure timer to overflow and fire an IRQ every 1 ms */
    arcv2_timer0_enable(ONE_MILLISECOND);
    /* Everything has been configured. It is now safe to enable the interrupt */
    interrupt_enable(ARCV2_IRQ_TIMER0);
}
