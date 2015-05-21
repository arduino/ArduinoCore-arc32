/* arcv2_timer1.h - ARC timer 1 device driver */

/*
 * Copyright (c) 2014 Wind River Systems, Inc.
 *
 * The right to copy, distribute, modify or otherwise make use
 * of this software may be licensed only pursuant to the terms
 * of an applicable Wind River license agreement.
 */

#ifndef _ARCV2_TIMER1__H_
#define _ARCV2_TIMER1__H_

#include <stdint.h>



#ifdef __cplusplus
 extern "C" {
#endif

/*******************************************************************************
*
* timer1_driver_init - initialize timer1 and enable interrupt
*
* RETURNS: N/A
*/
void timer1_driver_init(void(*int_handler)(void), uint32_t ticktime_ms);

/*******************************************************************************
*
* arcv2_timer1_count_get - get the current counter value
*
* This routine gets the value from the timer's count register.  This
* value is the 'time' elapsed from the starting count (assumed to be 0).
*
* RETURNS: the current counter value
*
* \NOMANUAL
*/
uint32_t arcv2_timer1_count_get(void);


/*******************************************************************************
*
* timer1_disable - Disables Timer1 interrupt generation.
*
* This routine disables timer interrupt generation and delivery.
* Note that the timer's counting cannot be stopped by software.
*
* RETURNS: N/A
*/
void timer1_disable(void);


#ifdef __cplusplus
}
#endif

#endif /* _ARCV2_TIMER1__H_ */
