/* arcv2_timer1.c - ARC timer 1 device driver */

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


/*******************************************************************************
*
* timer_driver - initialize timer1 and enable interrupt
*
* RETURNS: N/A
*/

#ifdef __cplusplus
 extern "C" {
#endif

void timer1_driver_init(void(*int_handler)(void), uint32_t ticktime_ms);
uint32_t timer1_read(void);

#ifdef __cplusplus
}
#endif

#endif /* _ARCV2_TIMER1__H_ */
