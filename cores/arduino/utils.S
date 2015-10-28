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


.globl getTimeStampClks
.type getTimeStampClks, %function

getTimeStampClks:
   /* Disable interrupts - we don't want to be disturbed */
   clri r2
   /* Load in r1 value of timer0_overflows */
   ld r1, [timer0_overflows]
   /* Read COUNT0 register */
   lr r0, [0x21]
   /* Read CONTROL0 register */
   lr r3, [0x22]
   /* If CONTROL0.IP is set COUNT0 reached LIMIT0 => r1 value might not be
    * accurate => read COUNT0 again */
   bbit0.nt r3, 3, end
   /* Read COUNT0 again*/
   lr r0, [0x21]
   /* Timer0 overflowed => timer0_overflows++ */
   add r1, r1, 1
   /***/
end:
    seti r2

