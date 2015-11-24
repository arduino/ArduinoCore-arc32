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
#include "portable.h"

#ifdef __cplusplus
 extern "C" {
#endif

/* Standard Arduino PWM resolution */
static int _writeResolution = 8;
static int _readResolution = 10;


void analogWriteResolution(int res)
{
    _writeResolution = res;
}

void analogReadResolution(int res)
{
    _readResolution = res;
}

static inline uint32_t mapResolution(uint32_t value, uint32_t from, uint32_t to)
{
     if (from == to)
         return value;
     if (from > to)
         return value >> (from-to);
     else
         return value << (to-from);
}

void analogWrite(uint8_t pin, int val)
{
    if (! digitalPinHasPWM(pin))
    {
        if(val > 127)
        {
            digitalWrite(pin, HIGH);
        }
        else
        {
            digitalWrite(pin, LOW);
        }
        return;
    }

    if (val <= 0) {
        /* Use GPIO for 0% duty cycle (always off)  */
        pinMode(pin, OUTPUT);
        digitalWrite(pin, LOW);
    } else if (val >= ((1 << _writeResolution) - 1)) {
        /* Use GPIO for 100% duty cycle (always on)  */
        pinMode(pin, OUTPUT);
        digitalWrite(pin, HIGH);
    } else {
        /* PWM for everything in between */
        PinDescription *p = &g_APinDescription[pin];
        uint32_t hcnt = mapResolution(val, _writeResolution, PWM_RESOLUTION);
        uint32_t lcnt = PWM_MAX_DUTY_CYCLE - hcnt;
        uint32_t offset;

        /* For Arduino Uno compatibilty, we scale up frequency on certain pins */
        hcnt >>= p->ulPwmScale;
        lcnt >>= p->ulPwmScale;

        /* Each count must be > 0 */
        if (hcnt < PWM_MIN_DUTY_CYCLE)
            hcnt = PWM_MIN_DUTY_CYCLE;
        if (lcnt < PWM_MIN_DUTY_CYCLE)
            lcnt = PWM_MIN_DUTY_CYCLE;

        /* Set the high count period (duty cycle) */
        offset = ((p->ulPwmChan * QRK_PWM_N_LCNT2_LEN) + QRK_PWM_N_LOAD_COUNT2);
        MMIO_REG_VAL(QRK_PWM_BASE_ADDR + offset) = hcnt;
        
        /* Set the low count period (duty cycle) */
        offset = ((p->ulPwmChan * QRK_PWM_N_REGS_LEN) + QRK_PWM_N_LOAD_COUNT1);
        MMIO_REG_VAL(QRK_PWM_BASE_ADDR + offset) = lcnt;

        if (p->ulPinMode != PWM_MUX_MODE) {
            /* start the PWM output */
            offset = ((p->ulPwmChan * QRK_PWM_N_REGS_LEN) + QRK_PWM_N_CONTROL);
            SET_MMIO_MASK(QRK_PWM_BASE_ADDR + offset, QRK_PWM_CONTROL_ENABLE);

            /* Disable pull-up and set pin mux for PWM output */
            SET_PIN_PULLUP(p->ulSocPin, 0);
            SET_PIN_MODE(p->ulSocPin, PWM_MUX_MODE);
            p->ulPinMode = PWM_MUX_MODE;
        }
    }
}
uint32_t analogRead(uint32_t pin)
{

    uint32_t val = 0;

    /* allow for channel or pin numbers */
    if (pin < 6) pin += A0;

    PinDescription *p = &g_APinDescription[pin];

    /* Disable pull-up and set pin mux for ADC output */
    if (p->ulPinMode != ADC_MUX_MODE) {
       SET_PIN_MODE(p->ulSocPin, ADC_MUX_MODE);
       p->ulPinMode = ADC_MUX_MODE;
       SET_PIN_PULLUP(p->ulSocPin,0);
    }

    /* Reset sequence pointer */
    SET_ARC_MASK(ADC_CTRL, ADC_SEQ_PTR_RST);
    /* Update sequence table */
    WRITE_ARC_REG(p->ulAdcChan, ADC_SEQ);
    /* Reset sequence pointer & start sequencer */
    SET_ARC_MASK(ADC_CTRL, ADC_SEQ_PTR_RST | ADC_SEQ_START | ADC_ENABLE);
    /* Poll for ADC data ready status (DATA_A) */
    while((READ_ARC_REG(ADC_INTSTAT) & ADC_INT_DATA_A) == 0);
    /* Pop the data sample from FIFO to sample register */
    SET_ARC_MASK(ADC_SET, ADC_POP_SAMPLE);
    /* Read sample from sample register */
    val = READ_ARC_REG( ADC_SAMPLE);
    /* Clear the DATA_A status bit */
    SET_ARC_MASK( ADC_CTRL, ADC_CLR_DATA_A);

    return mapResolution(val, ADC_RESOLUTION, _readResolution);

}

#ifdef __cplusplus
}
#endif
