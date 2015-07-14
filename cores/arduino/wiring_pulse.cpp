/*
Copyright (c) 2015 Intel Corporation.  All right reserved.

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
Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA

*/

#include "Arduino.h"
#include "wiring_pulse.h"
#include "arcv2_timer0.h"
#include "scss_registers.h"
#include "portable.h"


				/* min * secs * millis * micros */
#define MAX_PULSE_WIDTH	    ((uint32_t)(5 * 60 * 1000 * 1000))

static volatile uint64_t startMicroseconds;
static volatile uint64_t stopMicroseconds;
static volatile uint32_t firstIrqTriggered;
static volatile uint32_t lastIrqTriggered;

static uint32_t nextIrqMode;

static uint32_t int_polarity_reg;
static uint32_t GpioID;
static uint32_t	pinIsSoCGpio;



static void pulseInGpioISR(void)
{
    if (!firstIrqTriggered) {
        startMicroseconds = micros();
        firstIrqTriggered = 1;
        /* Change Polarity of GPIO Interrupt */
        if (FALLING == nextIrqMode) {
		if (pinIsSoCGpio)
                    CLEAR_MMIO_BIT(int_polarity_reg, GpioID);
                else
                    CLEAR_ARC_BIT(int_polarity_reg, GpioID);
	} else {
		if (pinIsSoCGpio)
                    SET_MMIO_BIT(int_polarity_reg, GpioID);
                else
                    SET_ARC_BIT(int_polarity_reg, GpioID);
	}
    } else {
        stopMicroseconds = micros();
	lastIrqTriggered = 1;
    }

}

uint32_t pulseIn( uint32_t pin, uint32_t state, uint32_t timeout )
{
    uint32_t pulseLengthMicroseconds = 0;
    uint64_t initTimestamp;
    uint64_t buffTimestamp;
    PinDescription *p = &g_APinDescription[pin];

    pinIsSoCGpio = (p->ulGPIOType == SOC_GPIO) ? 1 : 0;
    GpioID = p->ulGPIOId;

    if (pinIsSoCGpio) {
        switch (p->ulGPIOPort) {
	case SOC_GPIO_32:
            int_polarity_reg = SOC_GPIO_BASE_ADDR + SOC_GPIO_INTPOLARITY;
            break;
        }
    } else {
        switch (p->ulGPIOPort) {
	case SS_GPIO_8B0:
            int_polarity_reg = SS_GPIO_8B0_BASE_ADDR + SS_GPIO_INT_POLARITY;
            break;
        case SS_GPIO_8B1:
            int_polarity_reg = SS_GPIO_8B1_BASE_ADDR + SS_GPIO_INT_POLARITY;
            break;
        }
    }

    if (LOW == state) {
       nextIrqMode = RISING;
       attachInterrupt(pin, pulseInGpioISR, FALLING);
    } else {
        nextIrqMode = FALLING;
	attachInterrupt(pin, pulseInGpioISR, RISING);
    }
    initTimestamp = micros();

    /* wait <timeout> microseconds for the pulse */
    while (!firstIrqTriggered) {
        buffTimestamp = micros();
        if (buffTimestamp - initTimestamp > timeout )
            goto end;
    }
    /* Wait for pulse to finish */
    while (!lastIrqTriggered) {
        buffTimestamp = micros();
        if (buffTimestamp - startMicroseconds > MAX_PULSE_WIDTH)
            goto end;
    }
    /* The pulse is gone => compute its length */
    pulseLengthMicroseconds = stopMicroseconds - startMicroseconds;

end:
   detachInterrupt(pin);
   firstIrqTriggered = 0;
   lastIrqTriggered = 0;
   startMicroseconds = 0;
   stopMicroseconds = 0;

   return pulseLengthMicroseconds;
}
