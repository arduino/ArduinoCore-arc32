//***************************************************************
//
// Copyright (c) 2015 Intel Corporation.  All rights reserved.
//
// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation; either
// version 2.1 of the License, or (at your option) any later version.

// This library is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// Lesser General Public License for more details.

// You should have received a copy of the GNU Lesser General Public
// License along with this library; if not, write to the Free Software
// Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
//
//***************************************************************

//***************************************************************
//
// Module: CurieTimerOne
//
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


#include "CurieTimerOne.h"
#include <conf.h>

CurieTimer  CurieTimerOne;

static void timerOneIsrWrapper(void)
{
  CurieTimerOne.timerIsr();
}
static void timerOnePwmCbWrapper(void)
{
  CurieTimerOne.pwmCallBack();
}


CurieTimer::CurieTimer() :
  tickCnt(0), currState(IDLE), userCB(NULL)
{
  timerCountAddr = ARC_V2_TMR1_COUNT;
  timerControlAddr = ARC_V2_TMR1_CONTROL;
  timerLimitAddr = ARC_V2_TMR1_LIMIT;
  timerIrqNum = ARCV2_IRQ_TIMER1;
  isrFuncPtr = &timerOneIsrWrapper;
  pwmCB = &timerOnePwmCbWrapper;
}


// Method:  kill
// Description:
//   Set the timer back to power up default condition.

void CurieTimer::kill()
{
  interrupt_disable(timerIrqNum);  // Disable Timer at controller
  aux_reg_write(timerControlAddr, 0);  // Disable Timer itself
  aux_reg_write(timerLimitAddr, 0);
  aux_reg_write(timerCountAddr, 0);

  tickCnt = 0;
  userCB = NULL;
  currState = IDLE;
  pauseCntrl = pauseCount = dutyCycle = nonDutyCycle = periodInUsec = 0;
}


void CurieTimer::attachInterrupt(void (*userCallBack)())
{
  unsigned int reg;

  // Disable timer interrupt first.
  reg = aux_reg_read(timerControlAddr) & ~ARC_TIMER_EN_INTR_BIT_FLAG;
  aux_reg_write(timerControlAddr, reg);

  // Record the user call back routine.
  userCB = userCallBack;

  // Let timer interrupt go again.
  reg |= ARC_TIMER_EN_INTR_BIT_FLAG;
  aux_reg_write(timerControlAddr, reg);
}


unsigned int CurieTimer::rdRstTickCount(void)
{
  unsigned int tmp;

  tmp = tickCnt;
  tickCnt = 0;
  return tmp;
}


void CurieTimer::pause(void)
{
  if(currState != RUNNING)
    return;

  pauseCntrl = aux_reg_read(timerControlAddr);
  pauseCount = aux_reg_read(timerCountAddr);

  interrupt_disable(timerIrqNum);  // Disable Timer at controller
  aux_reg_write(timerControlAddr, 0);  // Disable Timer itself

  currState = PAUSED;
}


void CurieTimer::resume(void)
{
  if(currState != PAUSED)
    return;

  aux_reg_write(timerCountAddr, pauseCount);
  aux_reg_write(timerControlAddr, pauseCntrl);
  currState = RUNNING;

  // Re-enable timer interrupt once timer is reloaded with previous stop values.
  interrupt_enable(timerIrqNum);
}


// Method: pwmStart
//   Kick off a timer as the mean to time a pwm.  Calculate the durations of the duty cycle
// and they are loaded into the timer in the pwm call back.

int CurieTimer::pwmStart(unsigned int outputPin, double dutyPercentage, unsigned int periodUsec)
{
  unsigned int pwmPeriod;

  // Safe guard against periodUsec overflow when convert to hz.
  if((periodUsec == 0) || (periodUsec >= MAX_PERIOD_USEC))
    return -(INVALID_PERIOD);

  if((dutyPercentage < 0.0) || (dutyPercentage > 100.0))
    return -(INVALID_DUTY_CYCLE);

  periodInUsec = periodUsec;
  pwmPin = outputPin;
  pinMode(pwmPin, OUTPUT);

  if(dutyPercentage == 0.0) {
    // If PWM is already running, pause to stop it from interfering, and set pin to LOW
    pause(); 
    digitalWrite(pwmPin, LOW);
    return SUCCESS;
  }

  if(dutyPercentage == 100.0) {
    // If PWM is already running, pause to stop it from interfering, and set pin to HIGH
    pause(); 
    digitalWrite(pwmPin, HIGH);
    return SUCCESS;
  }

  pwmPeriod = periodUsec * HZ_USEC;

  dutyCycle = (unsigned int)(((double)pwmPeriod / 100.0) * dutyPercentage);
  nonDutyCycle = pwmPeriod - dutyCycle;

  // S/w overhead is about 4-5 usec. The shortest up or down cycle is set to be 10 usec.
  if(dutyCycle < (10 * HZ_USEC))
    dutyCycle = (10 * HZ_USEC);
  if(nonDutyCycle < (10 * HZ_USEC))
    nonDutyCycle = (10 * HZ_USEC);

  // Account for s/w overhead.
  dutyCycle -= (4 *  HZ_USEC);
  nonDutyCycle -= (4 *  HZ_USEC);

  dutyToggle = true;
  digitalWrite(pwmPin, HIGH);
  // Should return value back to caller.
  return init(dutyCycle, pwmCB);
}


int CurieTimer::pwmStart(unsigned int outputPin, int dutyRange, unsigned int periodUsec)
{
  if((dutyRange < 0) || (dutyRange > MAX_DUTY_RANGE))
    return -(INVALID_DUTY_CYCLE);

  return pwmStart(outputPin, ((double)dutyRange/(double)MAX_DUTY_RANGE) * 100.0, periodUsec);
}


// Method:  pwmCallBack
//   Software PWM ISR.  Timer generates interrupt and calls this method in
// its ISR.  This routine is responsible to toggle the PWM signal according
// to duty cycle duration.

inline void CurieTimer::pwmCallBack(void)
{
  dutyToggle = !dutyToggle;

  digitalWrite(pwmPin, dutyToggle ? HIGH : LOW);

  aux_reg_write(timerLimitAddr, dutyToggle ? dutyCycle : nonDutyCycle);
  aux_reg_write(timerCountAddr, 0);
}


// Method:  setPeriod
//   This method is for making this library backward compatible to the AVR
// TimerOne library.  The routine changs the time period by pausing the timer
// and resume it with the new period.  This is not timer re-initialization.

void CurieTimer::setPeriod(unsigned long microseconds)
{
  unsigned int periodHz;

  if((microseconds == 0) || (microseconds > MAX_PERIOD_USEC))
    microseconds = 1000000;

  periodInUsec = microseconds;
  periodHz = microseconds * HZ_USEC;
  pause();

  aux_reg_write(timerLimitAddr, periodHz);  // Load Timer period

  if(pauseCount >= periodHz)
    pauseCount = 0;

  resume();
}


// Method:  init
//   Kick off the timer with a period, in Hz, provided.  Initialize
// timer to run in non-halt mode, enable timer interrupt.  Always install
// the generic ISR and initialize the user call back if provided.

int CurieTimer::init(const unsigned int periodHz,
  void (*userCallBack)())
{
  if((periodHz == 0) || (periodHz > MAX_PERIOD_HZ))
    return -(INVALID_PERIOD);

  interrupt_disable(timerIrqNum);  // Disable Timer at controller
  aux_reg_write(timerControlAddr, 0);  // Disable Timer itself

  if(userCallBack != NULL)
    userCB = userCallBack;
  aux_reg_write(timerLimitAddr, periodHz);  // Load Timer period

  aux_reg_write(timerCountAddr, 0);  // Reset variables
  tickCnt = 0;

  if(isrFuncPtr != NULL) {  // Enable timer running with interrupt
    interrupt_connect(timerIrqNum, isrFuncPtr);
    aux_reg_write(timerControlAddr, ARC_TIMER_EN_INTR_BIT_FLAG |
		  ARC_TIMER_NON_HALT_ONLY_BIT_FLAG);
    interrupt_enable(timerIrqNum);
  }
  else {  // Timer runs without interrupt
    aux_reg_write(timerControlAddr, ARC_TIMER_NON_HALT_ONLY_BIT_FLAG);
  }

  currState = RUNNING;
  return SUCCESS;
}


// Method:  timerIsr
//   It's the generic timer ISR.  Will call user call back here.

inline void CurieTimer::timerIsr(void)
{
  unsigned int reg;

  tickCnt++;  // Account for the interrupt

  if(userCB != NULL)  // Call user ISR if available
    userCB();

  // Clear the interrupt pending bit upon exit.
  reg = aux_reg_read(timerControlAddr) & ~ARC_TIMER_INTR_PENDING_BIT_FLAG;
  aux_reg_write(timerControlAddr, reg);
}


