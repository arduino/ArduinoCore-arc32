/*
  Arduino.h - Main include file for the Arduino SDK
  Copyright (c) 2005-2013 Arduino Team.  All right reserved.

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
*/

#ifndef Arduino_h
#define Arduino_h

#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <math.h>

#include "pins_arduino.h"

#include "binary.h"
//#include "itoa.h"

#if ARDUINO < 10607
#error IDE version incompatible with Arduino 101. Please upgrade to version 1.6.7 or newer.
#endif

#ifdef __cplusplus
extern "C"{
#endif // __cplusplus

#include "wiring_constants.h"

#define clockCyclesPerMicrosecond() ( SYSTEM_CORE_CLOCK / 1000000L )
#define clockCyclesToMicroseconds(a) ( ((a) * 1000L) / (SYSTEM_CORE_CLOCK / 1000L) )
#define microsecondsToClockCycles(a) ( (a) * (SYSTEM_CORE_CLOCK / 1000000L) )

#define digitalPinToInterrupt(P)   ( P )

void yield(void);

/* sketch */
extern void setup( void ) ;
extern void loop( void ) ;

typedef void (*voidFuncPtr)( void ) ;

/* Define attribute */
#define WEAK __attribute__ ((weak))

#define INVALID 0xFFFFFFFF

/* Types used for the tables below */
/* TODO - consider using smaller types to optimise storage space */
typedef const struct _PinDescription
{
        uint32_t                ulGPIOId;               // GPIO port pin
        uint32_t                ulGPIOPort;             // GPIO port ID
        uint32_t                ulGPIOType;             // LMT or SS
        uint32_t                ulGPIOBase;             // GPIO register base address
        uint32_t                ulSocPin;               // SoC pin number
        uint32_t                ulPinMode;              // Current SoC pin mux mode
        uint32_t                ulPwmChan;              // PWM channel
        uint32_t                ulPwmScale;             // PWM frequency scaler
        uint32_t                ulAdcChan;              // ADC channel
        uint32_t                ulInputMode;            // Pin mode
} PinDescription;

#ifdef OUT
/* Types used for the tables below */
typedef struct _PinDescription
{
  // TODO Pio* pPort ;
  uint32_t ulPin ;
  uint32_t ulPeripheralId ;
  // TODO EPioType ulPinType ;
  uint32_t ulPinConfiguration ;
  uint32_t ulPinAttribute ;
  EAnalogChannel ulAnalogChannel ; /* Analog pin in the Arduino context (label on the board) */
  EAnalogChannel ulADCChannelNumber ; /* ADC Channel number in the SAM device */
  EPWMChannel ulPWMChannel ;
  ETCChannel ulTCChannel ;
} PinDescription ;
#endif


/* Pins table to be instanciated into variant.cpp */
extern PinDescription g_APinDescription[] ;

extern uint32_t pwmPeriod[4];

extern uint8_t pinmuxMode[NUM_DIGITAL_PINS];

#ifdef __cplusplus
} // extern "C"

#include "WCharacter.h"
#include "WString.h"
#include "Tone.h"
#include "WMath.h"
#include "HardwareSerial.h"
#include "wiring_pulse.h"

#endif // __cplusplus

// Include board variant
#include "variant.h"

#include "wiring.h"
#include "wiring_digital.h"
#include "wiring_analog.h"
#include "wiring_shift.h"
#include "WInterrupts.h"
#include "dccm/dccm_alloc.h"

#endif // Arduino_h
