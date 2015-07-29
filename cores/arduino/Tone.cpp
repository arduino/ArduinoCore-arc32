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

#include "Arduino.h"
#include "pins_arduino.h"
#include "interrupt.h"
#include "conf.h"
#include "aux_regs.h"

#define ACTIVE 1
#define INACTIVE 0
#define CONVERT_US 1000000

/* globals */
static uint32_t current_pin;
static uint32_t timer_interval;
static int32_t duration_left;
static uint32_t tone_state = INACTIVE;

#ifdef __cplusplus
 extern "C" {
#endif

static void timer1_disable_tone(void);
static void timer1_isr(void);
static void timer1_init_tone(uint32_t ticktime_us);

static void timer1_disable_tone(void)
{
    uint32_t saved;
    uint32_t ctrl_val;   /* timer control register value */

    saved = interrupt_lock();

    /* disable interrupt generation */
    ctrl_val = aux_reg_read(ARC_V2_TMR1_CONTROL);
    aux_reg_write(ARC_V2_TMR1_CONTROL, ctrl_val & ~ARC_V2_TMR_CTRL_IE);

    interrupt_unlock(saved);

    /* disable interrupt in the interrupt controller */
    interrupt_disable(ARCV2_IRQ_TIMER1);
}

static void timer1_isr(void)
{
    static uint32_t pin_state = 0;

    /* clear the interrupt (by writing 0 to IP bit of the control register) */
    aux_reg_write(ARC_V2_TMR1_CONTROL, ARC_V2_TMR_CTRL_NH | ARC_V2_TMR_CTRL_IE);

    if (duration_left != 0) {
        pin_state = !pin_state;
        digitalWrite(current_pin, pin_state);
        if (duration_left > 0)
            duration_left--;
    } else {
        timer1_disable_tone();
        digitalWrite(current_pin,LOW);
    }
}

static void timer1_init_tone(uint32_t ticktime_us)
{
    int tickunit;

    /* ensure that the timer will not generate interrupts */
    aux_reg_write(ARC_V2_TMR1_CONTROL, 0);
    aux_reg_write(ARC_V2_TMR1_COUNT, 0);    /* clear the count value */

    /* connect specified routine/parameter to the timer 0 interrupt vector */
    interrupt_connect(ARCV2_IRQ_TIMER1, timer1_isr);

    tickunit = 32 * ticktime_us;

    /*
     * Set the reload value to achieve the configured tick rate, enable the
     * counter and interrupt generation.
     *
     * The global variable 'tickunit' represents the #cycles/tick.
     */

    aux_reg_write(ARC_V2_TMR1_LIMIT, tickunit); /* write the limit value */
    /* count only when not halted for debug and enable interrupts */
    aux_reg_write(ARC_V2_TMR1_CONTROL, ARC_V2_TMR_CTRL_NH | ARC_V2_TMR_CTRL_IE);
    aux_reg_write(ARC_V2_TMR1_COUNT, 0); /* write the start value */

    /* Everything has been configured. It is now safe to enable the interrupt */
    interrupt_enable(ARCV2_IRQ_TIMER1);
}

#ifdef __cplusplus
}
#endif

void tone(uint32_t _pin, unsigned int frequency, unsigned long duration)
{
    if(frequency != 0) {
        timer_interval = (CONVERT_US / frequency) >> 1;
    } else
        return;

    /* duration of tone is never-ending, until a noTone() is called, unless a non-zero duration is specified by the caller */
    if (duration > 0) {
        duration *= 1000;
        duration_left = duration / timer_interval;
    } else {
        duration_left = -1;
    }

    if (tone_state == ACTIVE) {
        /* if tone already active on another pin, just return */
        if (current_pin != _pin) {
            return;
        } else { /*If the tone is playing on the same pin, set its frequency.*/
            timer1_disable_tone();
            pinMode(_pin, OUTPUT);
            timer1_init_tone(timer_interval);
        }
    } else { /*  set the pin as output, and configure the timer (init) to set the frequency (of the timer callbacks) */
        tone_state = ACTIVE;
        current_pin = _pin;
        pinMode(_pin, OUTPUT);
        timer1_init_tone(timer_interval);
    }
}

void noTone(uint32_t _pin)
{
    /* if tone already active on specified pin, disable the timer */
    if (current_pin == _pin) {
        timer1_disable_tone();
        digitalWrite(_pin, 0);
        tone_state = INACTIVE;
    }
}
