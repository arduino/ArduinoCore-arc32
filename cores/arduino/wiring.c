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
#include "data_type.h"
#include "conf.h"
#include "interrupt.h"
#include "aux_regs.h"
#include "board.h"

#define FREQ_MHZ	((ARCV2_TIMER0_CLOCK_FREQ)/1000000)
static const uint64_t	 MS_TO_CLKS = (FREQ_MHZ * 1000);

static uint64_t getTimeStampClks(void)
{
    __asm__ volatile (
   /* Disable interrupts - we don't want to be disturbed */
   "clri r2			    \n\t"
   /* Load in r1 value of timer0_overflows */
   "ld r1, %0			    \n\t"
   /* Read COUNT0 register */
   "lr r0, [0x21]		    \n\t"
   /* Read CONTROL0 register */
   "lr r3, [0x22]		    \n\t"
   /* If CONTROL0.IP is set COUNT0 reached LIMIT0 => r1 value might not be
    * accurate => read COUNT0 again */
   "bbit0.nt r3, 3, end	    \n\t"
   /* Read COUNT0 again*/
   "lr r0, [0x21]		    \n\t"
   /* Timer0 overflowed => timer0_overflows++ */
   "add r1, r1, 1		    \n\t"
   /***/
   "end:			    \n\t"
   "seti r2			    \n\t"
   : /* Output parameters and their constraints */
   :  "m"(timer0_overflows)   /* Input parameters and their constraints */
   :  "r0", "r1", "r2", "r3" /* Killed registers */
   );
}

void delay(uint32_t msec)
{
    uint64_t initial_timestamp = getTimeStampClks();
    uint64_t delay_clks = msec * MS_TO_CLKS;

    while (getTimeStampClks() - initial_timestamp < delay_clks) {
        yield();
    }
}


uint64_t millis(void)
{
    uint64_t timestamp = getTimeStampClks();
    return (uint64_t)(timestamp / (FREQ_MHZ * 1000));
}

uint64_t micros(void)
{
    uint64_t timestamp = getTimeStampClks();
    /* Divide by FREQ_MHZ and return */
    return (timestamp >> 5);
}
