/*
 * CurieTime.cpp - Time library for Arduino101
 * Copyright (c) 2015 Intel Corporation.  All rights reserved
 */
#include <time.h>

#include "CurieTime.h"

#define YEAR_OFFSET     1900
#define MONTH_OFFSET    1

unsigned long now()
{
    return *RTC_CCVR;
}

struct tm* tTm(unsigned long  t)
{
    time_t time = t;
  
    return gmtime(&time);
}

int year()
{
    unsigned long t = now();

    return year(t);
}

int year(unsigned long t)
{
    struct tm* tm = tTm(t);

    return (tm->tm_year + YEAR_OFFSET);
}

int month()
{
    unsigned long t = now();

    return month(t);
}

int month(unsigned long t)
{
    struct tm* tm = tTm(t);

    return (tm->tm_mon + MONTH_OFFSET);
}

int day()
{
    unsigned long t = now();

    return day(t);
}

int day(unsigned long t)
{
    struct tm* tm = tTm(t);

    return tm->tm_mday;
}

int hour()
{
    unsigned long t = now();

    return hour(t);
}

int hour(unsigned long t)
{
    struct tm* tm = tTm(t);

    return tm->tm_hour;
}

int minute()
{
    unsigned long t = now();

    return minute(t);
}

int minute(unsigned long t)
{
    struct tm* tm = tTm(t);

    return tm->tm_min;
}

int second()
{
    unsigned long t = now();

    return second(t);
}

int second(unsigned long t)
{
    struct tm* tm = tTm(t);

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
