/*
  Copyright (c) 2011 Arduino.  All right reserved.
  Copyright (c) 2015 Intel Corporation.  All right reserved (delayMicroseconds).

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. 
  See the GNU Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#ifndef _WIRING_
#define _WIRING_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "wiring_constants.h"
#include "arcv2_timer0.h"


/**
 *
 */
extern void initVariant( void ) ;
extern void init( void ) ;

/**
 * \brief Returns the number of milliseconds since the Arduino 101 board began
 * running the current program.
 *
 * This number will practically never overflow (go back to zero).
 *
 * \return Number of milliseconds since the program started (uint64_t).
 */
extern uint64_t millis( void ) ;

/**
 * \brief Returns the number of microseconds since the Arduino 101 board began
 * running the current program.
 *
 * This number will practically never overflow (go back to zero).
 * It will overflow after more than 18000 years.
 * This function has a resolution of 2 microseconds.
 *
 * \note There are 1,000 microseconds in a millisecond and 1,000,000
 * microseconds in a second.
 */
extern uint64_t micros( void ) ;

/**
 * \brief Pauses the program for the amount of time (in milliseconds) specified
 *  as parameter.
 *
 *  This function relies on Timer0 interrupts, therefore it shouldn't be called
 *  from a context with interrupts disabled.
 *  It doesn't use CPU's power management features.
 *
 * \param dwMs the number of milliseconds to pause (uint32_t)
 *
 * \note There are 1000 milliseconds in a second.
 */
extern void delay( uint32_t dwMs ) ;


/**
 * \brief Pauses the program for the amount of time (in microseconds) specified
 *  as parameter.
 *
 *  The precision is +- 0.5 microsecond.
 *  It doesn't disable the interrupts and it doesn't rely on interrupts,
 *  meaning this function works reliable even if the interrupts are disabled.
 *  It doesn't use CPU's power management features.
 *
 * \param dwUs the number of microseconds to pause (uint32_t)
 *	Accepted range: from 1 microsecond up to 0x07FFFFFF microseconds.
 *	If dwUs > above specifies threshold, the delay overflows.
 *	E.g. If dwUs = 0x08000000 (0x07FFFFFF + 1), it actually means dwUs = 0
 */
static inline __attribute__ ((always_inline))
void delayMicroseconds(uint32_t dwUs)
{
    if (0 == dwUs) return;
    uint32_t init_count = arcv2_timer0_count_get();
    /* Multiply microseconds with FREQ_MHZ to transform them in clocks */
    uint32_t clocks = dwUs << 5;

    while (arcv2_timer0_count_get() - init_count < clocks);
}

static inline __attribute__ ((always_inline))
void delayTicks(uint32_t ticks)
{
  // Each tick is 1/32 uS
  // TODO: improve using asm
  if(ticks == 0)
      return;
  else if(ticks < 10)
  {
    // just do a 5 tick delay to be close enough
    __builtin_arc_nop();
    __builtin_arc_nop();
    __builtin_arc_nop();
    __builtin_arc_nop();
    __builtin_arc_nop();
  }
  else  // compensate for the overhead
  {
    ticks -= 10;
    uint32_t init_count = arcv2_timer0_count_get();
    while (arcv2_timer0_count_get() - init_count < ticks);
  }
}

#ifdef __cplusplus
}
#endif

#endif /* _WIRING_ */
