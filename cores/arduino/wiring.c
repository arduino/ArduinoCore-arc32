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

static inline __attribute__((always_inline))
uint64_t getTimeStampClks(void)
{
	uint32_t time_stamp;
	int key = interrupt_lock();
	uint64_t ret = timer0_overflows;
	time_stamp = aux_reg_read(ARC_V2_TMR0_COUNT);
	if (aux_reg_read(ARC_V2_TMR0_CONTROL) & (0x01 << 3)) {
		time_stamp = aux_reg_read(ARC_V2_TMR0_COUNT);
		ret++;
	}
	interrupt_unlock(key);
	return ((ret << 32) | time_stamp);
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
