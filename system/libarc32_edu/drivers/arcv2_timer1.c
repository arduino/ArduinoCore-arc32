/* arcv2_timer1.c - ARC timer 1 device driver */

/*
 * Copyright (c) 2014 Wind River Systems, Inc.
 *
 * The right to copy, distribute, modify or otherwise make use
 * of this software may be licensed only pursuant to the terms
 * of an applicable Wind River license agreement.
 */

/*
modification history
--------------------
03Nov14,j_b  written
*/

/*
DESCRIPTION
This module implements a VxMicro device driver for the ARCv2 processor timer 1

\INTERNAL IMPLEMENTATION DETAILS
The ARCv2 processor timer provides a 32-bit incrementing, wrap-to-zero counter.

The device driver is also part of a nanokernel-only system, but omits more
complex capabilities (such as tickless idle support) that are only used in
conjunction with a microkernel.
*/


#include "arcv2_timer1.h"
#include "aux_regs.h"
#include "conf.h"
#include "../bootcode/interrupt.h"

#define ARCV2_TIMER1_CLOCK_FREQ   32000000	/* 32MHz reference clock */

/* defines */

#define ARC_V2_TMR_CTRL_IE	0x1		/* interrupt enable */
#define ARC_V2_TMR_CTRL_NH	0x2		/* count only while not halted */
#define ARC_V2_TMR_CTRL_W	0x4		/* watchdog mode enable */
#define ARC_V2_TMR_CTRL_IP	0x8		/* interrupt pending flag */

/* globals */

void (* timer1_user_int_handler)(void) = 0x00;

/* locals */

/* forward declarations */

uint32_t timer1_read(void);

/*******************************************************************************
*
* arcv2_timer_enable - enable the timer with the given limit/countup value
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
void arcv2_timer_enable
    (
    uint32_t count    /* count to which timer is to increment to */
    )
{
    aux_reg_write(ARC_V2_TMR1_LIMIT, count); /* write the limit value */
    /* count only when not halted for debug and enable interrupts */
    aux_reg_write(ARC_V2_TMR1_CONTROL, ARC_V2_TMR_CTRL_NH | ARC_V2_TMR_CTRL_IE);
    aux_reg_write(ARC_V2_TMR1_COUNT, 0); /* write the start value */
}

/*******************************************************************************
*
* arcv2_timer_count_get - get the current counter value
*
* This routine gets the value from the timer's count register.  This
* value is the 'time' elapsed from the starting count (assumed to be 0).
*
* RETURNS: the current counter value
*
* \NOMANUAL
*/
static inline __attribute__((always_inline))
uint32_t arcv2_timer1_count_get(void)
{
    return (aux_reg_read(ARC_V2_TMR1_COUNT));
}

/*******************************************************************************
 *
 * arcv2_timer_limit_get - get the limit/countup value
 *
 * This routine gets the value from the timer's limit register, which is the
 * value to which the timer will count up to.
 *
 * RETURNS: the current counter value
 *
 * \NOMANUAL
 */
static inline __attribute__((always_inline))
uint32_t arcv2_timer1_limit_get(void)
{
    return (aux_reg_read(ARC_V2_TMR1_LIMIT));
}

/*******************************************************************************
*
* _arcv2_timer_int_handler - system clock periodic tick handler
*
* This routine handles the system clock periodic tick interrupt.  A TICK_EVENT
* event is pushed onto the microkernel stack.
*
* RETURNS: N/A
*
* \NOMANUAL
*/
void _arcv2_timer1_int_handler(void)
{
    /* clear the interrupt (by writing 0 to IP bit of the control register) */
    aux_reg_write(ARC_V2_TMR1_CONTROL, ARC_V2_TMR_CTRL_NH | ARC_V2_TMR_CTRL_IE);
    /* execute callback specified by the user */
    if (0x00 != timer1_user_int_handler)
	    timer1_user_int_handler();
}


/*******************************************************************************
*
* timer_driver - initialize timer1 and enable interrupt
*
* RETURNS: N/A
*/

void timer1_driver_init(void(*int_handler)(void), uint32_t ticktime_ms)
{
	int tickunit;

	/* ensure that the timer will not generate interrupts */
	aux_reg_write(ARC_V2_TMR1_CONTROL, 0);
	aux_reg_write(ARC_V2_TMR1_COUNT, 0);	/* clear the count value */

	/* connect specified routine/parameter to the timer 0 interrupt vector */
	interrupt_connect(ARCV2_IRQ_TIMER1, _arcv2_timer1_int_handler, 0);
	timer1_user_int_handler = int_handler;
#if 0
	(void) nanoCpuIntConnect(_WRS_CONFIG_ARCV2_TIMER1_INT_LVL,
			_WRS_CONFIG_ARCV2_TIMER1_INT_PRI,
			_arcv2_timer1_int_handler, 0);
#endif

	tickunit = (ARCV2_TIMER1_CLOCK_FREQ / 1000) * ticktime_ms;

	/*
	 * Set the reload value to achieve the configured tick rate, enable the
	 * counter and interrupt generation.
	 *
	 * The global variable 'tickunit' represents the #cycles/tick.
	 */

	arcv2_timer_enable(tickunit);

	/* Everything has been configured. It is now safe to enable the interrupt */
	interrupt_enable(ARCV2_IRQ_TIMER1);
	/* Enable global ARC interrupts */
//	interrupt_unlock(ARCV2_SETI_IRQ_LVL_2);
	interrupt_unlock(0);
#if 0
	nanoCpuIntEnable (_WRS_CONFIG_ARCV2_TIMER1_INT_LVL);
#endif
}

/*******************************************************************************
*
* timer_read - read the BSP timer hardware
*
* This routine returns the current time in terms of timer hardware clock cycles.
*
* RETURNS: up counter of elapsed clock cycles
*/

uint32_t timer1_read(void)
{
    return arcv2_timer1_count_get();
}


/*******************************************************************************
*
* timer_disable - stop announcing ticks into the kernel
*
* This routine disables timer interrupt generation and delivery.
* Note that the timer's counting cannot be stopped by software.
*
* RETURNS: N/A
*/

void timer1_disable(void)
{
    uint32_t saved;
    uint32_t ctrl_val;   /* timer control register value */

    saved = interrupt_lock();

    /* disable interrupt generation */
    ctrl_val = aux_reg_read(ARC_V2_TMR0_CONTROL);
    aux_reg_write(ARC_V2_TMR0_CONTROL, ctrl_val & ~ARC_V2_TMR_CTRL_IE);

    interrupt_unlock(saved);

    /* disable interrupt in the interrupt controller */
    interrupt_disable(ARCV2_IRQ_TIMER1);
#if 0
    nanoCpuIntDisable (ARCV2_IRQ_TIMER1);
#endif
}
