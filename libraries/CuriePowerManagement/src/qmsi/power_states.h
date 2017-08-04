/*
 * Copyright (c) 2016, Intel Corporation
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of the Intel Corporation nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE INTEL CORPORATION OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef __POWER_STATES_H__
#define __POWER_STATES_H__

/**
 * SoC Power mode control for Quark SE Microcontrollers.
 *
 * Available SoC states are:
 *     - Low Power Sensing Standby (LPSS)
 *     - Sleep
 *
 * LPSS can only be enabled from the Sensor core,
 * refer to @ref ss_power_soc_lpss_enable for further details.
 *
 * @defgroup groupSoCPower Quark SE SoC Power states
 * @{
 */

/**
 * Enter SoC sleep state.
 *
 * Put the SoC into sleep state until next SoC wake event.
 *
 * - Core well is turned off
 * - Always on well is on
 * - Hybrid Clock is off
 * - RTC Clock is on
 *
 * Possible SoC wake events are:
 * 	- Low Power Comparator Interrupt
 * 	- AON GPIO Interrupt
 * 	- AON Timer Interrupt
 * 	- RTC Interrupt
 */
void power_soc_sleep(void);

/**
 * Enter SoC deep sleep state.
 *
 * Put the SoC into deep sleep state until next SoC wake event.
 *
 * - Core well is turned off
 * - Always on well is on
 * - Hybrid Clock is off
 * - RTC Clock is on
 *
 * Possible SoC wake events are:
 * 	- Low Power Comparator Interrupt
 * 	- AON GPIO Interrupt
 * 	- AON Timer Interrupt
 * 	- RTC Interrupt
 *
 * This function puts 1P8V regulators and 3P3V into Linear Mode.
 */
void power_soc_deep_sleep(void);

/**
 * @}
 */

#endif /* __POWER_STATES_H__ */
