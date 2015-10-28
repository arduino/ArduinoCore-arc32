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
#include "conf.h"
#include "aux_regs.h"
#include "interrupt.h"
#include "board.h"


struct _IsrTableEntry
{
    void (*isr)(void);
};

struct _IsrTableEntry __attribute__((section(".data"))) _IsrTable[SS_NUM_IRQS];

static void _dummy_isr(void)
{
    __asm__ ("flag 0x01"); /* Set the halt flag => halt the CPU  */
    for(;;); /* This infinite loop is intentional and requested by design */
}

void interrupt_connect(unsigned int irq, void (*isr)(void))
{
    int index = irq - SS_NUM_EXCEPTIONS;
    unsigned int flags = interrupt_lock();
    _IsrTable[index].isr = isr;
    interrupt_unlock(flags);
}

void interrupt_disconnect(unsigned int irq)
{
    int index = irq - SS_NUM_EXCEPTIONS;
    _IsrTable[index].isr = _dummy_isr;
}

void interrupt_enable(unsigned int irq)
{
    unsigned int flags = interrupt_lock();
    aux_reg_write (ARC_V2_IRQ_SELECT, irq);
    aux_reg_write (ARC_V2_IRQ_TRIGGER, ARC_V2_INT_LEVEL);
    aux_reg_write (ARC_V2_IRQ_ENABLE, ARC_V2_INT_ENABLE);
    interrupt_unlock(flags);
}

void interrupt_disable(unsigned int irq)
{
    unsigned int flags = interrupt_lock();
    aux_reg_write (ARC_V2_IRQ_SELECT, irq);
    aux_reg_write (ARC_V2_IRQ_ENABLE, ARC_V2_INT_DISABLE);
    interrupt_unlock(flags);
}

void interrupt_priority_set (int irq, unsigned char priority)
{
    unsigned int flags = interrupt_lock();
    aux_reg_write (ARC_V2_IRQ_SELECT, irq);
    aux_reg_write (ARC_V2_IRQ_PRIORITY, priority);
    interrupt_unlock(flags);
}

void interrupt_unit_device_init(void)
{
    int irq_index;
    int min_irq_no = VECTOR_FROM_IRQ(0);
    int max_irq_no = VECTOR_FROM_IRQ(SS_NUM_IRQS);
    for (irq_index = min_irq_no; irq_index < max_irq_no; irq_index++)
    {
        aux_reg_write(ARC_V2_IRQ_SELECT, irq_index);
        aux_reg_write(ARC_V2_IRQ_PRIORITY, (INTERRUPT_THRESHOLD - 1) );
        aux_reg_write(ARC_V2_IRQ_ENABLE, ARC_V2_INT_DISABLE);
        aux_reg_write(ARC_V2_IRQ_TRIGGER, ARC_V2_INT_LEVEL);
    }
    /* Setup automatic (hardware) context saving feature */
    aux_reg_write(ARC_V2_AUX_IRQ_CTRL, AUX_IRQ_CTRL_SAVE_ALL);
    /* Configure the interrupt priority threshold and enable interrupts */
    __builtin_arc_seti(INTERRUPT_ENABLE | INTERRUPT_THRESHOLD);
    /* Invalidate I_CACHE */
    __builtin_arc_sr(0x0, ARC_V2_IC_IVIC);
    __builtin_arc_nop();
    __builtin_arc_nop();
    __builtin_arc_nop();
}

