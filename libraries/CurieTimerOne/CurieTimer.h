//***************************************************************
//
// Copyright (c) 2015 Intel Corporation.  All rights reserved.
//
// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation; either
// version 2.1 of the License, or (at your option) any later version.
//
// This library is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public
// License along with this library; if not, write to the Free Software
// Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
//
//***************************************************************


//***************************************************************
//
// File: CurieTimer.h
//
// Description:
//   Class definitions for library CurieTimer.
//
// Notes:
//
//   - Arc has two timers, Timer-0 is used.  Please check the file
//     arcv2_timer0.cpp.
//
// Cautions:
//   - The module, Tone.cpp, also makes use of Arc Timer-1 which
//     is used here.
//
//***************************************************************


#ifndef CurieTime_h
#define CurieTime_h

#include <Arduino.h>
#include <aux_regs.h>
#include <interrupt.h>
#include <conf.h>

// Timer-1 is clocked at ARCV2_TIMER1_CLOCK_FREQ defined in conf.h
const unsigned int HZ_USEC = (ARCV2_TIMER1_CLOCK_FREQ / 1000000);  // Hz per micro second.
const unsigned int MAX_PERIOD = (0x0FFFFFFFF / HZ_USEC);

// The two ARC timers.
const unsigned int ARC_TIMER_0 = 0;
const unsigned int ARC_TIMER_1 = 1;

// Duty cycle range.
const int MAX_DUTY_RANGE = 1023;

// TIMERx_CONTROL register.
const unsigned int ARC_TIMER_EN_INTR_BIT_FLAG = 0x01;  // Bit 0. Enable timer interrupt.
const unsigned int ARC_TIMER_NON_HALT_ONLY_BIT_FLAG = 0x02;  // Bit 1. Allow timer to run in Halted mode.
const unsigned int ARC_TIMER_WATCHDOG_BIT_FLAG = 0x04;  // Bit 2. Make timer a watchdog timer.
const unsigned int ARC_TIMER_INTR_PENDING_BIT_FLAG = 0x08;  // Bit 3. Interrupt pending and clearing bit.

typedef enum {
  SUCCESS = 0,
  INVALID_PERIOD,
  INVALID_DUTY_CYCLE
} timerErrType;

typedef enum {
  IDLE = 0,
  RUNNING,
  PAUSED
} timerStateType;


//
//  Class:  CurieTimer
//
//  Description:
//    This class describes the functionalities of a Arc Timer.  It has the knowledge
//  of only 2 timers available in the h/w - a limitation.  The timers are sourced by
//  a 32 HHz clock, a constant, used in this class, defined in the conf.h file.
//

class CurieTimer
{
  public:
    // Constructor.  Default is timer-1.
    CurieTimer(const unsigned int timerNum = ARC_TIMER_1);

    // Set up the timer with the input period, in usec, and the call back routine.
    // Period stays with the timer until it is killed or re/start another round.
    inline int start(const unsigned int timerPeriodUsec = 0,
		     void (*userCallBack)() = NULL) {
      return( init((timerPeriodUsec * HZ_USEC), userCallBack) ); }

    // Restarting the timer, start counter from 0.
    int restart(const unsigned int timerPeriodUsec) { return start(timerPeriodUsec); }

    // Kill the timer, put it back to power up default.
    void kill(void);

    // Attach or detach the user call back routine.
    void attachInterrupt(void (*userCallBack)());
    void detachInterrupt(void) { return attachInterrupt(NULL); };

    // Timer interrupt count.
    inline unsigned int readTickCount(void) { return tickCnt; }
    // Read and reset timer interrupt count.
    unsigned int rdRstTickCount(void);

    // Pausing the timer = no count up, no interrupt.
    void pause(void);
    inline void stop(void) { return pause(); }
    // Resume the timer from where it was paused.
    void resume(void);

    // Start software PWM.  Note that the timer is consumed once PWM is set.
    int pwmStart(unsigned int outputPin, double dutyPercentage, unsigned int periodUsec);

    // Start software PWM.  Use a range of 0-1023 for duty cycle. 0=always Low, 1023=always high
    int pwmStart(unsigned int outputPin, int dutyRange, unsigned int periodUsec);

    // Stop software PWM.  Put the time back to default and de-assert the selected port.
    inline void pwmStop(void) { kill(); return digitalWrite(pwmPin, LOW); }

    // Generic timer ISR.  It will call user call back routine if set.
    void timerIsr(void);

    void pwmCallBack(void);

  private:
    unsigned int timerCountAddr;
    unsigned int timerControlAddr;
    unsigned int timerLimitAddr;
    unsigned int timerIrqNum;

    unsigned int tickCnt;

    timerStateType currState;
    unsigned int pauseCntrl;
    unsigned int pauseCount;

    bool dutyToggle;
    unsigned int pwmPin;
    unsigned int dutyCycle;
    unsigned int nonDutyCycle;

    void (*isrFuncPtr)();
    void (*userCB)();
    void (*pwmCB)();

    // Init:  Kick off a timer by initializing it with a period.
    int init(const unsigned int periodHz, void (*userCallBack)());
};

extern CurieTimer  CurieTimerOne;


#endif // CurieTime_h

