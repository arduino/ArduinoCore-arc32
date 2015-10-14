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
#ifndef __INTERRUPT_H__
#define __INTERRUPT_H__

#define INTERRUPT_ENABLE    (1 << 4)
 /* According to IRQ_BUILD register the ARC core has only 2 interrupt priority
  * levels (0 and 1).
  * */
#define INTERRUPT_THRESHOLD (1)

#ifdef __cplusplus
 extern "C" {
#endif

extern void interrupt_connect(unsigned int irq, void (*isr)(void));
extern void interrupt_disconnect(unsigned int irq);
extern void interrupt_enable(unsigned int irq);
extern void interrupt_disable(unsigned int irq);
extern void interrupt_priority_set (int irq, unsigned char priority);

extern void interrupt_unit_device_init(void);

static inline __attribute__((always_inline))
unsigned int interrupt_lock(void)
{
    unsigned int key;

    __asm__ volatile ("clri %0" : "=r" (key));
    return key;
}

static inline __attribute__((always_inline))
void interrupt_unlock(unsigned int key)
{
    __asm__ volatile ("seti %0" :: "ir" (key));
}
#ifdef __cplusplus
}
#endif

#endif /* __INTERRUPT_H__ */
