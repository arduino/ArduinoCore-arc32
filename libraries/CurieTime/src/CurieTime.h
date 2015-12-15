/*
 * CurieTime.h - Time library for Arduino101
 */

#ifndef CurieTime_h
#define CurieTime_h

#define RTC_CCVR    (volatile int*)0xb0000400 // Current Counter Value Register
#define RTC_CMR     0xb0000404 // Counter Match Register
#define RTC_CLR     (volatile int*)0xb0000408 // Counter Load Register
#define RTC_CCR     0xb000040C // Counter Control Register
#define RTC_STAT    0xb0000410 // Interrupt Status Register
#define RTC_RSTAT   0xb0000414 // Interrupt Raw Status Register
#define RTC_EOI     0xb0000418 // End of Interrupt Register

// The following API is based on Paul Stoffregen's Arduino Time Library:
//   https://github.com/PaulStoffregen/Time 

unsigned long now(); // current time as seconds since Jan 1 1970 

int year();                     // current year as an integer
int year(unsigned long  t);     // year of t as an integer
int month();                    // current month as an integer (1 - 12)
int month(unsigned long  t);    // month of t as an integer (1 - 12)
int day();                      // current day as an integer (1 - 31)
int day(unsigned long  t);      // day of t as an integer (1 - 31)
int hour();                     // current hour as an integer (0 - 23)
int hour(unsigned long  t);     // hour of t as an integer (0 - 23)
int minute();                   // current minute as an integer (0 - 59)
int minute(unsigned long  t);   // minute of t as an integer (0 - 59)
int second();                   // current second as an integer (0 - 59)
int second(unsigned long  t);   // second of t as an integer (0 - 59)

void setTime(int hour, int minute, int second, int day, int month, int year); // set the current time
void setTime(unsigned long t); // set the current time from seconds since Jan 1 1970

#endif
