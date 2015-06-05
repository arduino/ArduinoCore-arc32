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

#include "WInterrupts.h"
#include "wiring_digital.h"
#include "gpio.h"
#include "interrupt.h"

/* Saves the interrupt state from STATUS32 register before disabling the
 * interrupts */
static uint32_t irq_flags;
/* Works kinda like a semaphore: make sure one has called noInterrupts() before
 * interrupts() */
static uint32_t noInterrupts_executed;

void attachInterrupt(uint32_t pin, void(*callback)(void), uint32_t mode)
{
    if (pin >= NUM_DIGITAL_PINS) {
#ifdef DEBUG
        __asm__("flag 0x01") ; /* Halt the CPU */
#endif
        return;
    }
    PinDescription *p = &g_APinDescription[pin];
    gpio_cfg_data_t config;
    DRIVER_API_RC ret;

    /* First de-config the GPIO in case it was previously configured as IRQ */
    if (p->ulGPIOType == SS_GPIO)
        ret = ss_gpio_deconfig(p->ulGPIOPort, p->ulGPIOId);
    else
        ret = soc_gpio_deconfig(p->ulGPIOPort, p->ulGPIOId);
#ifdef DEBUG
    if (ret != DRV_RC_OK)
        __asm__("flag 0x01") ; /* Halt the CPU */
#endif

    switch (mode) {
    case LOW:
        config.int_type = LEVEL;
        config.int_polarity = ACTIVE_LOW;
        break;
    case HIGH:
        config.int_type = LEVEL;
        config.int_polarity = ACTIVE_HIGH;
	break;
    case RISING:
        config.int_type = EDGE;
        config.int_polarity = ACTIVE_HIGH;
	break;
    case FALLING:
        config.int_type = EDGE;
        config.int_polarity = ACTIVE_LOW;
	break;
    case CHANGE:
	if (p->ulGPIOType == SOC_GPIO) {
            config.int_type = DOUBLE_EDGE;
            config.int_polarity = ACTIVE_LOW;
	} else { /* ARC GPIOs don't support both edges IRQ */
            config.int_type = EDGE;
            config.int_polarity = ACTIVE_LOW;
	}
	break;
    default:
#ifdef DEBUG
	__asm__("flag 0x01"); /* Halt the CPU */
#endif
	return;
    }
    config.gpio_type = GPIO_INTERRUPT;
    config.int_debounce = DEBOUNCE_ON;
    config.int_ls_sync = LS_SYNC_OFF;
    config.gpio_cb = callback;

    if (p->ulGPIOType == SS_GPIO)
        ret = ss_gpio_set_config(p->ulGPIOPort, p->ulGPIOId, &config);
    else
	ret = soc_gpio_set_config(p->ulGPIOPort, p->ulGPIOId, &config);
#ifdef DEBUG
    if (ret != DRV_RC_OK)
        __asm__("flag 0x01") ; /* Halt the CPU */
#endif
}


void detachInterrupt(uint32_t pin)
{
    if (pin >= NUM_DIGITAL_PINS) {
#ifdef DEBUG
        __asm__("flag 0x01") ; /* Halt the CPU */
#endif
        return;
    }
    PinDescription *p = &g_APinDescription[pin];
    DRIVER_API_RC ret;

    if (p->ulGPIOType == SS_GPIO)
        ret = ss_gpio_deconfig(p->ulGPIOPort, p->ulGPIOId);
    else
        ret = soc_gpio_deconfig(p->ulGPIOPort, p->ulGPIOId);
#ifdef DEBUG
    if (ret != DRV_RC_OK)
        __asm__("flag 0x01") ; /* Halt the CPU */
#endif
}


void interrupts(void)
{
    if (noInterrupts_executed) {
        noInterrupts_executed = 0;
	interrupt_unlock(irq_flags);
    }
}

void noInterrupts(void)
{
    if (!noInterrupts_executed) {
        noInterrupts_executed = 1;
        irq_flags = interrupt_lock();
    }
}
