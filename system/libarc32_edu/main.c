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
#include <string.h>

#include "drivers/soc_gpio.h"
#include "drivers/arcv2_timer1.h"
#include "scss_registers.h"

#define GPIO_2 2

#define PIN13 GPIO_2

#define OUTPUT 0
#define INPUT  1

#define LOW  0
#define HIGH 1

static void configure_soc_gpio(pin, mode)
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

void pinMode(pin, mode)
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

void digitalWrite(pin, state)
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

#define TIMER1_TICK	500
void timer1_user_isr(void)
{
    uint8_t static pin_state = LOW;
    pin_state = !pin_state;
    digitalWrite(13, pin_state);
}

void setup(void)
{
    pinMode(13, OUTPUT);
    timer1_driver_init(timer1_user_isr, TIMER1_TICK);
}

#define DELAY_CYCLES 1000000
void loop(void)
{
    unsigned i;
    for (i = 0; i < DELAY_CYCLES; i++)
        digitalWrite(13, HIGH);
    for (i = 0; i < DELAY_CYCLES; i++)
        digitalWrite(13, LOW);
}

int main(void)
{
    soc_gpio_enable(SOC_GPIO_32);
    SET_PIN_MODE(2, QRK_PMUX_SEL_MODEA);

    setup();
    for(;;)
	    __asm__("nop");
//        loop();

    __builtin_unreachable();
}
