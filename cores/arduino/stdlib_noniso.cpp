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

char * dtostrf(double number, signed char width, unsigned char prec, char *s) {

    if (isnan(number)) {
        strcpy(s, "nan");
        return s;
    }
    if (isinf(number)) {
        strcpy(s, "inf");
        return s;
    }

    char *out = s;
    int expCnt = 0, digit, totalWidth;
    double tmp, rounding;

    // Check for left adjustment (spaces)
    while(width < 0) {
      *out++ = ' ';
      width++;
    }
    totalWidth = (int)width;

    // Handle negative numbers
    if (number < 0.0) {
      *out++ = '-';
      number = -number;
      if(totalWidth > 0) totalWidth--;
    }

    // Rounding up to the precision
    tmp = number;
    rounding = 0.5;
    for(int i=0; i < prec; i++)
      rounding /= 10.0;
    tmp += rounding;

    // Shifting the number to the right
    while( tmp >= 10.0 ) {
      tmp /= 10.0;
      expCnt++;
    }

    // 1st, print the single digit left after shifting
    digit = (int)tmp;
    *out++ = '0' + digit;
    tmp -= (double)digit;
    if(totalWidth > 0) totalWidth--;

    // Then the integer portion
    shiftOutDigit(&tmp, expCnt, out);
    out += expCnt;
    if(totalWidth > 0) totalWidth -= expCnt;

    // Then the decimal portion
    if( prec ) {
      *out++ = '.';
      shiftOutDigit(&tmp, prec, out);
      if(totalWidth > 0) totalWidth -= (prec + 1);
      out += prec;
    }

    // Right adjustment
    while(totalWidth > 0) {
      *out++ = ' ';
      totalWidth--;
    }

    *out = 0;  // End of string
    return s;
}
