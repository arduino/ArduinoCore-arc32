/*
 * CurieRTC.h - RTC library for Arduino101
 * This library is intended to be uses with Arduino Time.h library functions
 */

#ifndef CurieRTC_h
#define CurieRTC_h

#define RTC_CCVR    0xb0000400 // Current Counter Value Register
#define RTC_CMR     0xb0000404 // Counter Match Register
#define RTC_CLR     0xb0000408 // Counter Load Register
#define RTC_CCR     0xb000040C // Counter Control Register
#define RTC_STAT    0xb0000410 // Interrupt Status Register
#define RTC_RSTAT   0xb0000414 // Interrupt Raw Status Register
#define RTC_EOI     0xb0000418 // End of Interrupt Register

#include <Time.h>

// library interface description
class CurieRTC
{
  // user-accessible "public" interface
  public:
    CurieRTC();
    static time_t get();
    static void set(time_t t);
};

#ifdef RTC
#undef RTC // workaround for Arduino Due, which defines "RTC"...
#endif

extern CurieRTC RTC;

#endif
 

