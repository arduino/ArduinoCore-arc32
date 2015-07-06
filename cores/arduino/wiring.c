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
      /* Go to sleep and wait to be awaken by Timer0 (or an external) 
       * interrupt. */
	__asm__ volatile ("sleep");
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
   __asm__ __volatile__ (
   /* Disable interrupts - we don't want to be disturbed */
   "clri r2			    \n\t"
   /* Load in r3 value of timer0_overflows */
   "ld r3, %0			    \n\t"
   /* Use only the least-significant 22 bits of timer0_overflows */
   "and r3, r3, 0x3FFFFF	    \n\t"
   /* Read COUNT0 register */
   "lr r0, [0x21]		    \n\t"
   /* Read CONTORL0 register */
   "lr r1, [0x22]		    \n\t"
   /* If CONTROL0.IP is set COUNT0 reached LIMIT0 => r0 value might not be
    * accurate => read COUNT0 again */
   "bbit0.nt r1, 3, continue	    \n\t"
   /* Read COUNT0 again*/
   "lr r0, [0x21]		    \n\t"
   /* Timer0 overflowed => timer0_overflows++ */
   "add r3, r3, 1		    \n\t"
   /***/
   "continue:			    \n\t"
	   /* Compute microseconds time-stamp */
   /* Transform milliseconds in microseconds */
   "mpy r3, r3, 1000		    \n\t"
   /* Transform ticks in microseconds */
   "div r0, r0, 32		    \n\t"
   /* Store the final result in R0 register.
    * The function returns the result in R0 register. */
   "add r0, r0, r3		    \n\t"
   "seti r2			    \n\t"
   :	    /* Output parameters and their constraints */
   : "m"(timer0_overflows)   /* Input parameters and their constraints */
   : "r0", "r1", "r2", "r3" /* Killed registers */
			);
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


