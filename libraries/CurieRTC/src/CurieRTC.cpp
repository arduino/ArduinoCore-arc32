/*
 * CurieRTC.c - RTC library for Arduino101
  
  Copyright (c) 2015 Intel Corporation.  All rights reserved
  This library is intended to be uses with Arduino Time.h library functions

  The library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
  
 */

#include <time.h>

#include "CurieRTC.h"

#define YEAR_OFFSET     1900
#define MONTH_OFFSET    1

unsigned long now()
{
    return *RTC_CCVR;
}

struct tm* nowTm() {
    time_t t = now();
  
    return gmtime(&t);
}

int year()
{
    struct tm* tm = nowTm();

    return (tm->tm_year + YEAR_OFFSET);
}

int month()
{
    struct tm* tm = nowTm();

    return (tm->tm_mon + MONTH_OFFSET);
}

int day()
{
    struct tm* tm = nowTm();

    return tm->tm_mday;
}

int hour()
{
    struct tm* tm = nowTm();

    return tm->tm_hour;
}

int minute()
{
    struct tm* tm = nowTm();

    return tm->tm_min;
}

int second()
{
    struct tm* tm = nowTm();

    return tm->tm_sec;
}

void setTime(unsigned long t)
{
    *RTC_CLR = t;
}

void setTime(int hour, int minute, int second, int day, int month, int year)
{
    struct tm tm;
    time_t t;

    tm.tm_year = year - YEAR_OFFSET;
    tm.tm_mon = month - MONTH_OFFSET;
    tm.tm_mday = day;
    tm.tm_hour = hour;
    tm.tm_min = minute;
    tm.tm_sec = second;
    tm.tm_isdst = -1;

    t = mktime(&tm);

    setTime(t);
}
