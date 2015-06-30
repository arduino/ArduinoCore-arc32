/*
  Copyright (c) 2011 Arduino.  All right reserved.

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

#ifndef _Tone_h
#define _Tone_h

#include <stdint.h>

/*
 * \brief Generates a square wave of the specified frequency (and 50% duty cycle) on a pin.
 *        A duration can be specified, otherwise the wave continues until a call to noTone()
 * \param pin
 * \param val
 */
extern void tone( uint32_t _pin, unsigned int frequency, unsigned long duration = 0);
/*
 * \brief Stops the generation of a square wave triggered by tone(). Has no effect if no tone is being generated.
 *
 * \param pin
 */

extern void noTone( uint32_t _pin ) ;

#endif
