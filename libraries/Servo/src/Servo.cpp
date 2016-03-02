/*
 Servo.cpp - Interrupt driven Servo library for Arduino using ARC Timer 1 on Arduino 101 boards
 Copyright (c) 2009 Michael Margolis.  All right reserved.

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

 Copyright (c) 2015 Intel Corporation. Implementation of Servo library for Arduino 101
 */

#include "interrupt.h"
#include "Arduino.h"
#include "conf.h"
#include "aux_regs.h"
#include "Servo.h"

static servo_t servos[MAX_SERVOS];         // static array of servo structures
static volatile int32_t Channel;           // counter for the servo being pulsed for timer 1
static uint32_t ServoCount;                // the total number of attached servos

/************ static functions common to all instances ***********************/
static void timer1_disable_servo(void);
static void timer1_isr_servo(void);
static void timer1_init_servo(uint32_t ticktime);

static void timer1_disable_servo(void)
{
    uint32_t saved;
    uint32_t ctrl_val;   /* timer control register value */

    saved = interrupt_lock();

    /* disable interrupt generation */
    ctrl_val = aux_reg_read(ARC_V2_TMR1_CONTROL);
    aux_reg_write(ARC_V2_TMR1_CONTROL, ctrl_val & ~ARC_V2_TMR_CTRL_IE);

    interrupt_unlock(saved);

    /* disable interrupt in the interrupt controller */
    interrupt_disable(ARCV2_IRQ_TIMER1);
}

static void timer1_isr_servo(void)
{
    static uint32_t total_count = 0;
    uint32_t time_leftover = 0;

    /* clear the interrupt (by writing 0 to IP bit of the control register) */
    aux_reg_write(ARC_V2_TMR1_CONTROL, ARC_V2_TMR_CTRL_NH | ARC_V2_TMR_CTRL_IE);

    if (Channel < 0) {
        /* channel set to -1 indicated that refresh interval completed so reset timer count */
        total_count = 0;
    }
    else {
        if (servos[Channel].Pin.isActive == true) {
            /* pulse this channel low if activated */
            digitalWrite(servos[Channel].Pin.nbr, LOW);
        }
    }

    Channel++;    // increment to the next channel

    if(Channel < (int32_t)ServoCount) {
        total_count +=  servos[Channel].ticks;
        timer1_init_servo(servos[Channel].ticks);
        if (servos[Channel].Pin.isActive == true) {
            /* its an active channel so pulse it high */
            digitalWrite(servos[Channel].Pin.nbr, HIGH);
        }
    }
    else {
        Channel = -1;
        /* finished all channels so wait for the refresh period to expire before starting over */
        if ((total_count)  < usToTicks(REFRESH_INTERVAL)) {
            time_leftover = usToTicks(REFRESH_INTERVAL) - total_count;
            timer1_init_servo(time_leftover);
        } else {
            /* missed the refresh period, re-run this function to begin a new cycle immediately */
            timer1_isr_servo();
        }
    }
}

static void timer1_init_servo(uint32_t ticktime)
{
    /* ensure that the timer will not generate interrupts */
    aux_reg_write(ARC_V2_TMR1_CONTROL, 0);
    aux_reg_write(ARC_V2_TMR1_COUNT, 0);    /* clear the count value */

    /* connect specified routine/parameter to the timer 0 interrupt vector */
    interrupt_connect(ARCV2_IRQ_TIMER1, timer1_isr_servo);

    /*
     * Set the reload value to achieve the configured tick rate, enable the
     * counter and interrupt generation.
     *
     * The global variable 'tickunit' represents the #cycles/tick.
     */

    aux_reg_write(ARC_V2_TMR1_LIMIT, ticktime); /* write the limit value */
    /* count only when not halted for debug and enable interrupts */
    aux_reg_write(ARC_V2_TMR1_CONTROL, ARC_V2_TMR_CTRL_NH | ARC_V2_TMR_CTRL_IE);
    aux_reg_write(ARC_V2_TMR1_COUNT, 0); /* write the start value */

    /* Everything has been configured. It is now safe to enable the interrupt */
    interrupt_enable(ARCV2_IRQ_TIMER1);
}

static boolean isTimerActive()
{
    /* returns true if any servo is active on this timer */
    for (uint32_t channel=0; channel < MAX_SERVOS; channel++) {
        if (servos[channel].Pin.isActive == true)
            return true;
        }

    return false;
}

/****************** end of static functions ******************************/
Servo::Servo()
{
    if ( ServoCount < MAX_SERVOS) {
        this->servoIndex = ServoCount++;
        servos[this->servoIndex].ticks = usToTicks(DEFAULT_PULSE_WIDTH);
    } else {
        this->servoIndex = INVALID_SERVO;
    }
}

uint32_t Servo::attach(int pin)
{
    return this->attach(pin, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
}

uint32_t Servo::attach(int pin, int min, int max)
{
    if (this->servoIndex < MAX_SERVOS) {
        pinMode( pin, OUTPUT) ;
        servos[this->servoIndex].Pin.nbr = pin;
        this->min  = (MIN_PULSE_WIDTH - min)/4;   //resolution of min/max is 4 uS
        this->max  = (MAX_PULSE_WIDTH - max)/4;

        if (isTimerActive() == false) {
            Channel = -1;
            timer1_init_servo(servos[this->servoIndex].ticks);
        }

        /* this must be set after the check for isTimerActive */
        servos[this->servoIndex].Pin.isActive = true;
    }

    return this->servoIndex;
}

void Servo::detach()
{
    servos[this->servoIndex].Pin.isActive = false;

    if (isTimerActive() == false) {
        timer1_disable_servo();
    }
}

void Servo::write(int value)
{
    /* treat values less than 544 as angles in degrees(valid values in microseconds are handled as microseconds) */
    if (value < MIN_PULSE_WIDTH) {
        if (value < 0) {
            value = 0;
        }
        if (value > 180) {
            value = 180;
        }

        value = map(value, 0, 180, SERVO_MIN(),  SERVO_MAX());
    }

    this->writeMicroseconds(value);
}

void Servo::writeMicroseconds(int value)
{
    /* calculate and store the values for the given channel */
    byte channel = this->servoIndex;

    /* ensure chnannel is valid */
    if ((channel < MAX_SERVOS)) {
        /* ensure pulse width is valid */
        if (value < SERVO_MIN()) {
            value = SERVO_MIN();
        } else if (value > SERVO_MAX()) {
            value = SERVO_MAX();
        }

        value = value - TRIM_DURATION;
        value = usToTicks(value);

        noInterrupts();
        servos[channel].ticks = value;
        interrupts();
    }
}

int Servo::read()
{
    /* return the value as degrees */
    return  map( this->readMicroseconds()+1, SERVO_MIN(), SERVO_MAX(), 0, 180);
}

int Servo::readMicroseconds()
{
    unsigned int pulsewidth;

    if (this->servoIndex != INVALID_SERVO) {
        pulsewidth = ticksToUs(servos[this->servoIndex].ticks)  + TRIM_DURATION;
    } else {
        pulsewidth  = 0;
    }

    return pulsewidth;
}

bool Servo::attached()
{
    return servos[this->servoIndex].Pin.isActive;
}
