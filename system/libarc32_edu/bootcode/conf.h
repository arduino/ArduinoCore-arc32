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

#ifndef _ARCV2_CONF__H_
#define _ARCV2_CONF__H_


#define _WRS_CONFIG_NUM_IRQS	64

#define _RAM_END		(0xa8010000 + 0x4000)

#define STACK_SIZE		1024
#define ISR_STACK_SIZE		1024

#define ARCV2_IRQ_TIMER0		16
#define ARCV2_IRQ_TIMER1		17


#endif /* _ARCV2_CONF__H_ */
