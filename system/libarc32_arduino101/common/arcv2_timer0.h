/* arcv2_timer0.h - ARC timer 0 device driver */

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


#ifndef _ARCV2_TIMER0__H_
#define _ARCV2_TIMER0__H_

#include <stdint.h>
#include "aux_regs.h"

#ifdef __cplusplus
extern "C" {
#endif


/* Increments every Timer0 overflow.
 * Timer0 is configured as a free-run timer; it overflows every 0xFFFFFFFF
 * clocks.
 */
extern uint32_t volatile timer0_overflows;


/*******************************************************************************
*
* timer0_driver_init - initialize and enable the system clock
*
* This routine is used to program the ARCv2 Timer as a free-run timer.
* It delivers interrupts every 0xFFFFFFFF clocks.
*
* RETURNS: N/A
*/
void timer0_driver_init(void);


/*******************************************************************************
*
* arcv2_timer0_count_get - get the current counter value
*
* This routine gets the value from the timer's count register.  This
* value is the 'time' elapsed from the starting count (assumed to be 0).
*
* RETURNS: the current counter value
*
* \NOMANUAL
*/
static inline __attribute__((always_inline))
uint32_t arcv2_timer0_count_get(void)
{
    return (aux_reg_read(ARC_V2_TMR0_COUNT));
}


/*******************************************************************************
*
* arcv2_timer0_control_get - get the value of CONTROL0 aux register
*
* This routine gets the value from the timer's control register.
*
* RETURNS: the value of CONTROL0 auxiliary register.
*
* \NOMANUAL
*/
static inline __attribute__((always_inline))
uint32_t arcv2_timer0_control_get(void)
{
    return (aux_reg_read(ARC_V2_TMR0_CONTROL));
}


#ifdef __cplusplus
}
#endif

#endif /* _ARCV2_TIMER0__H_ */
