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
#include <stdarg.h>
#include <string.h>

#include "drivers/soc_gpio.h"
#include "scss_registers.h"
#include "cfw_platform.h"

#define GPIO_2 2

#define PIN13 GPIO_2

#define OUTPUT 0
#define INPUT  1

#define LOW  0
#define HIGH 1

static void configure_soc_gpio(uint8_t pin, uint8_t mode)
{
    gpio_cfg_data_t cfg;
    uint8_t bit;

    memset(&cfg, 0, sizeof(cfg));

    cfg.gpio_type = (mode == OUTPUT) ? GPIO_OUTPUT : GPIO_INPUT;

    switch(pin) {
    case 13:
        bit = 2;
        break;
    default:
        // Invalid/not-supported
        return;
    };

    soc_gpio_set_config(SOC_GPIO_32, bit, &cfg);
}

void pinMode(uint8_t pin, uint8_t mode)
{
    switch(pin) {
    case 13:
        configure_soc_gpio(pin, mode);
        break;
    default:
        // Invalid/not-supported
        return;
    };
}

void digitalWrite(uint8_t pin, uint8_t state)
{
    uint8_t bit;

    switch(pin) {
    case 13:
        bit = 2;
        break;
    default:
        // Invalid/not-supported
        return;
    };

    soc_gpio_write(SOC_GPIO_32, bit, state);
}


void setup(void)
{
}

void loop(void)
{
}

int main(void)
{
    /* CFW IPC initialisation */
    cfw_platform_init(false);

    soc_gpio_enable(SOC_GPIO_32);
    SET_PIN_MODE(2, QRK_PMUX_SEL_MODEA);

    setup();
    for(;;) {
//	    __asm__("nop");
        loop();
    }

    __builtin_unreachable();
}
