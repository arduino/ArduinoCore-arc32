/*
 * Copyright (c) 2015, Intel Corporation. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef CLK_SYSTEM_H_
#define CLK_SYSTEM_H_

#include <stdint.h>

#define CLK_GATE_OFF (0)
#define CLK_GATE_ON  (~CLK_GATE_OFF)

/**
 * @defgroup clk_gate Clock gating driver
 * Clk Gate driver API.
 * @ingroup common_drivers
 * @{
 */

/**
 *  Clock gate data which contain register and bits implicated.
 */
struct clk_gate_info_s {
	uint32_t clk_gate_register;  /*!< register changed for clock gate */
	uint32_t bits_mask;          /*!< mask used for clock gate */
};

/**
*  Configure clock gate to specified device
*
*  @param  clk_gate_info   : pointer to a clock gate data structure
*  @param  value           : state of clock gate desired
*/
void set_clock_gate(struct clk_gate_info_s* clk_gate_info, uint32_t value);

/** @} */

#endif /* CLK_SYSTEM_H_ */
