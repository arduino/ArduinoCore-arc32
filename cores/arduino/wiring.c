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

#include "wiring.h"
#include "arcv2_timer0.h"
#include "data_type.h"
#include "conf.h"
#include "interrupt.h"
#include "aux_regs.h"
#include "board.h"

#define FREQ_MHZ    ((ARCV2_TIMER0_CLOCK_FREQ)/1000000)

void delay(uint32_t msec)
{
    uint32_t no_of_irqs = timer0_overflows + msec;
    uint32_t microseconds = arcv2_timer0_count_get();

    while(timer0_overflows < no_of_irqs){
        yield();
      /* Enable interrupts, sets interrupts threshold to 2 and go to sleep. */
	__asm__ volatile ("sleep %0" :: "i" (INTERRUPT_ENABLE | INTERRUPT_THRESHOLD));
    }
   /* For the last fraction of millisecond don't go to sleep - you'll wake up
    * too late - just spin */
    while ((arcv2_timer0_count_get() < microseconds) &&
		    (timer0_overflows ==  no_of_irqs));
}


uint32_t millis(void)
{
    return timer0_overflows;
}


uint32_t micros(void)
{
    uint32_t tmr0_ctrl_reg;
    uint32_t microsecs;

    uint32_t flags = interrupt_lock();

    tmr0_ctrl_reg = arcv2_timer0_control_get();
    if(tmr0_ctrl_reg && ARC_V2_TMR0_CONTROL_IP_MASK){
	/* The IP bit is set, means the timer overflowed but the IRQ handler
	 * hasn't updated the timer0_overflows value because the IRQs are
	 * disabled; it is manually incremented here  */
        microsecs = arcv2_timer0_count_get();
	microsecs = (timer0_overflows + 1) * 1000 + microsecs;
    }else{
	/* The timer hasn't reached LIMIT from the point where we disabled the
	 * interrupts until here => read COUNT0 and check again the overflow
	 * condition */
	microsecs = arcv2_timer0_count_get();
	tmr0_ctrl_reg = arcv2_timer0_control_get();
	if(tmr0_ctrl_reg && ARC_V2_TMR0_CONTROL_IP_MASK){
	    /* The COUNT0 reached LIMIT0 while reading COUNT0 value and
	     * possibly overflowed */
	    microsecs = arcv2_timer0_count_get();
	    microsecs = (timer0_overflows + 1) * 1000 + microsecs;
	}else{
	    microsecs = timer0_overflows * 1000 + microsecs;
	}
    }

    interrupt_unlock(flags);

    return microsecs;
}

void delayMicroseconds(uint32_t usec)
{
    /* Function parameter is stored in R0 register */
    __asm__ __volatile__ (
    "mpy.f r0, r0, 32	    \n\t" /* delay_ticks = delay_usec * 32 */
    "bz.d _end		    \n\t" /* if usec == 0 goto end */
    /* Subtract the function call overhead and the time spent by the previous
     * instructions of the function.
     * T function_call = 4 clks
     * T mpy + T bz = 3 + 3 = 6 clks
     * T exit_function + loop_precision ~ 5 clks
     * The above described timings were computed using a scope and infinite
     * loops in the code, meaning the instructions were most likely executed
     * from ICACHE. 
     * */
    "sub r0, r0, 15	    \n\t"
    /* Minimum value of r0 = 32 => the above subtraction cannot overflow */
    "_repeat:		    \n\t"
    /* T sub.f = 1 clk; T bnc.nd = 3 clks */
    "sub.f r0, r0, 4	    \n\t"
    /* Repeat above subtraction until carry flag is set */
    "bnc.nd _repeat	    \n\t"
    "_end:		    \n\t"
    : /* output parameters of ASM instructions */
    : /* input parameters */
    : "r0" /* registers killed by above ASM instructions. */
    );
}


