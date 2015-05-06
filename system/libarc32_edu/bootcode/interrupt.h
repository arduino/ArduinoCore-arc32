#ifndef __INTERRUPT_H__
#define __INTERRUPT_H__

extern void interrupt_connect(unsigned int irq,
			      void (*isr)(void *arg),
			      void *arg);
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

#endif /* __INTERRUPT_H__ */
