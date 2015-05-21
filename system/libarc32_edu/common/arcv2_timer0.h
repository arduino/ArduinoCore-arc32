/* arcv2_timer0.h - ARC timer 0 device driver */

/*
 * Copyright (c) 2014 Wind River Systems, Inc.
 *
 * The right to copy, distribute, modify or otherwise make use
 * of this software may be licensed only pursuant to the terms
 * of an applicable Wind River license agreement.
 */

#ifndef _ARCV2_TIMER0__H_
#define _ARCV2_TIMER0__H_

#include <stdint.h>
#include "aux_regs.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Increments every Timer0 overflow.
 * Timer0 is configured to overflow and fire an IRQ every 1 millisecond
 */
extern uint32_t volatile timer0_overflows;

/* It is incremented every ms by Timer0 IRQ handler but it overflows at
 * MAX_OVERFLOWS_US */
extern uint32_t volatile timer0_overflows_us;

/*******************************************************************************
*
* timer0_driver_init - initialize and enable the system clock
*
* This routine is used to program the ARCv2 timer to deliver interrupts at the
* 1 millisecond rate specified via the ONE_MILLISECOND macro.
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
