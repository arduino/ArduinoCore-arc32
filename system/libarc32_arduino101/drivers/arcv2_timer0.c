/* arcv2_timer0.c - ARC timer 0 device driver */

/*
 * Copyright (c) 2014-2015 Wind River Systems, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1) Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2) Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3) Neither the name of Wind River Systems nor the names of its contributors
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */


/*
DESCRIPTION
This module implements a VxMicro device driver for the ARCv2 processor timer 0
and provides the standard "system clock driver" interfaces.

\INTERNAL IMPLEMENTATION DETAILS
The ARCv2 processor timer provides a 32-bit incrementing, wrap-to-zero counter. 

*/

#include "arcv2_timer0.h"
#include "conf.h"
#include "interrupt.h"

#define FREE_RUN_TIMER		0xFFFFFFFF

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
* _arcv2_timer0_int_handler - Timer0 ISR
*
* This routine handles the overflows of 32-bit free-run Timer0.
* In this way a virtually 64-bit RTC is created:
*   timer0_overflows = high double word of virtual RTC.
*   COUNT0 of Tiemr0 = low double word of virtual RTC.
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
* This routine is used to the ARCv2 Timer as a free-run timer.
* It delivers interrupts every 0xFFFFFFFF clocks.
*
* RETURNS: N/A
*/
void timer0_driver_init(void)
{
    /* connect specified routine/parameter to the timer 0 interrupt vector */
    interrupt_connect(ARCV2_IRQ_TIMER0, _arcv2_timer0_int_handler);
    /* Enable Timer0 as a free-run timer. */
    arcv2_timer0_enable(FREE_RUN_TIMER);

    /* Everything has been configured. It is now safe to enable the interrupt */
    interrupt_enable(ARCV2_IRQ_TIMER0);
}
