

#ifndef _ARCV2_TIMER1__H_
#define _ARCV2_TIMER1__H_

#include <stdint.h>


/*******************************************************************************
*
* timer_driver - initialize timer1 and enable interrupt
*
* RETURNS: N/A
*/

#ifdef __cplusplus
 extern "C" {
#endif

void timer1_driver_init(void(*int_handler)(void), uint32_t ticktime_ms);
uint32_t timer1_read(void);

#ifdef __cplusplus
}
#endif

#endif /* _ARCV2_TIMER1__H_ */
