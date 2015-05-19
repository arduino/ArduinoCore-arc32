/*
 * TODO: Copyright header
 * */

#include "wiring.h"
#include "arcv2_timer0.h"
#include "data_type.h"
#include "conf.h"
#include "interrupt.h"
#include "aux_regs.h"

#define FREQ_MHZ    ((ARCV2_TIMER0_CLOCK_FREQ)/1000000)

void delay(uint32_t msec)
{
    if(0 == msec)
	    return;
    uint32_t no_of_irqs = timer0_overflows;
    uint32_t microseconds = arcv2_timer0_count_get() / FREQ_MHZ;

    while(timer0_overflows - no_of_irqs < msec){
//        yield();
      /* Enter sleep and enable interrupts and sets interrupts threshold to 3 */
        __asm__ volatile ("sleep 0x13"); 
    }
   /* For the last fraction of millisecond don't go to sleep - you'll wake up
    * too late - just spin */
    while (arcv2_timer0_count_get() / FREQ_MHZ < microseconds);
}


uint32_t millis(void)
{
    return timer0_overflows;
}


uint32_t micros(void)
{
    uint32_t tmr0_ctrl_reg;
    uint32_t microsecs;

    uint32_t flags = interrupt_lock();

    tmr0_ctrl_reg = arcv2_timer0_control_get();
    if(tmr0_ctrl_reg && ARC_V2_TMR0_CONTROL_IP_MASK){
	/* The IP bit is set, means the timer overflowed but the IRQ handler
	 * hasn't updated the timer0_overflows value because the IRQs are
	 * disabled; it is manually incremented here  */
        microsecs = arcv2_timer0_count_get();
	microsecs = (timer0_overflows + 1) * 1000 + microsecs;
    }else{
	/* The timer hasn't reached LIMIT from the point where we disabled the
	 * interrupts until here => read COUNT0 and check again the overflow
	 * condition */
	microsecs = arcv2_timer0_count_get();
	tmr0_ctrl_reg = arcv2_timer0_control_get();
	if(tmr0_ctrl_reg && ARC_V2_TMR0_CONTROL_IP_MASK){
	    /* The COUNT0 reached LIMIT0 while reading COUNT0 value and
	     * possibly overflowed */
	    microsecs = arcv2_timer0_count_get();
	    microsecs = (timer0_overflows + 1) * 1000 + microsecs;
	}else{
	    microsecs = timer0_overflows * 1000 + microsecs;
	}
    }

    interrupt_unlock(flags);

    return microsecs;
}
