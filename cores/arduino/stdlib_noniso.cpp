/* 
 core_esp8266_noniso.c - nonstandard (but usefull) conversion functions
 Copyright (c) 2014 Ivan Grokhotkov. All rights reserved.
 This file is part of the esp8266 core for Arduino environment.
 
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
 Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 Modified 03 April 2015 by Markus Sattler
 */

#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include "stdlib_noniso.h"
#include "math.h"
#include "inttypes.h"

#define ASCII_ZERO  0x30

int atoi(const char* s) {
    return (int) atol(s);
}

long atol(const char* s) {
    char * tmp;
    return strtol(s, &tmp, 10);
}

double atof(const char* s) {
    char * tmp;
    return strtod(s, &tmp);
}

void reverse(char* begin, char* end) {
    char *is = begin;
    char *ie = end - 1;
    while(is < ie) {
        char tmp = *ie;
        *ie = *is;
        *is = tmp;
        ++is;
        --ie;
    }
}

char* itoa( int val, char *string, int radix )
{
    return ltoa( val, string, radix ) ;
}

char* ltoa( long val, char *string, int radix )
{
    char tmp[33];
    char *tp = tmp;
    long i;
    unsigned long v;
    int sign;
    char *sp;

    if ( string == NULL )
    {
        return 0 ;
    }

    if (radix > 36 || radix <= 1)
    {
        return 0 ;
    }

    sign = (radix == 10 && val < 0);
    if (sign)
    {
        v = -val;
    }
    else
    {
        v = (unsigned long)val;
    }

    while (v || tp == tmp)
    {
        i = v % radix;
        v = v / radix;
        if (i < 10)
            *tp++ = i+'0';
        else
            *tp++ = i + 'a' - 10;
    }

    sp = string;

    if (sign)
        *sp++ = '-';
    while (tp > tmp)
        *sp++ = *--tp;
    *sp = 0;

    return string;
}

char* utoa( unsigned int val, char *string, int radix )
{
    return ultoa( val, string, radix ) ;
}

char* ultoa( unsigned long val, char *string, int radix )
{
    char tmp[33];
    char *tp = tmp;
    long i;
    unsigned long v = val;
    char *sp;

    if ( string == NULL )
    {
        return 0;
    }

    if (radix > 36 || radix <= 1)
    {
        return 0;
    }

    while (v || tp == tmp)
    {
        i = v % radix;
        v = v / radix;
        if (i < 10)
            *tp++ = i+'0';
        else
            *tp++ = i + 'a' - 10;
    }

    sp = string;

    while (tp > tmp)
        *sp++ = *--tp;
      *sp = 0;

    return string;
}

void shiftOutDigit(double *number, int count, char *s)
{
    double tmp = *number;
    int digit;

    while( count ) {
      tmp *= 10.0;
      digit = (int)tmp;
      *s++ = '0' + digit;
      tmp -= (double)digit;
      count--;
    }
    *number = tmp;
}

int digitsBe4Decimal(double number)
{
  int cnt = 1;  // Always has one digit

  // Count -ve sign as one digit
  if(number < 0.0) {
    cnt++;
    number = -number;
  }

  // Count the number of digits beyond the 1st, basically, the exponent.
  while(number >= 10.0) {
    number /= 10;
    cnt++;
  }
  return cnt;
}

char *dtostrf(double number, signed char width, unsigned char prec, char *s)
{
    char *out;
    unsigned long long integer;
    double fraction, rounding;
    int digit, before, i;
    int delta;

    if (isnan(number)) {
        strcpy(s, "nan");
        return s;
    }
    if (isinf(number)) {
        strcpy(s, "inf");
        return s;
    }

    out = s;
    before = digitsBe4Decimal(number);

    // check if padding is required
    if (width < 0) {
        delta = (-width) - (before + prec + 1);
        for (i = 0; i < delta; ++i) *out++ = ' ';
        width = 0;
    }

    // Handle negative numbers
    if (number < 0.0) {
      *out = '-';
      number = -number;
    }

    // rounding up to the precision
    rounding = 0.5;
    for (i = 0; i < prec; ++i)
        rounding /= 10.0;
    number += rounding;

    // seperate integral and fractional parts
    integer = (unsigned long long) number;
    fraction = (double) (number - integer);

    // generate chars for each digit of the integral part
    i = before;
    while (integer >= 10) {
        digit = integer % 10;
        out[(i--) - 1] = ASCII_ZERO + digit;
        integer /= 10;
    }

    out[i - 1] = ASCII_ZERO + integer;
    out += before;
    if (!prec) goto end;

    // generate chars for each digit of the fractional part
    *out++ = '.';
    for (i = 0; i < prec; ++i) {
        fraction *= 10.0;
        digit = ((unsigned long long) fraction) % 10;
        *out++ = (char) (ASCII_ZERO + digit);
    }

end:
    // check if padding is required
    if (width > 0) {
        delta = width - (before + prec + 1);
        for (i = 0; i < delta; ++i) *out++ = ' ';
    }

    *out = 0;
    return s;
}
