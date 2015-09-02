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

#ifndef SOC_REGISTER_H_
#define SOC_REGISTER_H_

#include "scss_registers.h"

/*
 * CREG defines
 */
#define     CREG_CLK_CTRL_SPI0      (27)
#define     CREG_CLK_CTRL_SPI1      (28)
#define     CREG_CLK_CTRL_I2C0      (29)
#define     CREG_CLK_CTRL_I2C1      (30)
#define     CREG_CLK_CTRL_ADC       (31)

/* PVP */
#define PVP_REGISTER_BASE   (0xB0600000)

#define QRK_PVP_NCR              (PVP_REGISTER_BASE + 0x00)
#define QRK_PVP_COMP             (PVP_REGISTER_BASE + 0x04)
#define QRK_PVP_LCOMP            (PVP_REGISTER_BASE + 0x08)
#define QRK_PVP_IDX_DIST         (PVP_REGISTER_BASE + 0x0C)
#define QRK_PVP_CAT              (PVP_REGISTER_BASE + 0x10)
#define QRK_PVP_AIF              (PVP_REGISTER_BASE + 0x14)
#define QRK_PVP_MINIF            (PVP_REGISTER_BASE + 0x18)
#define QRK_PVP_MAXIF            (PVP_REGISTER_BASE + 0x1C)
#define QRK_PVP_TESTCOMP         (PVP_REGISTER_BASE + 0x20)
#define QRK_PVP_TESTCAT          (PVP_REGISTER_BASE + 0x24)
#define QRK_PVP_NID              (PVP_REGISTER_BASE + 0x28)
#define QRK_PVP_GCR              (PVP_REGISTER_BASE + 0x2C)
#define QRK_PVP_RSTCHAIN         (PVP_REGISTER_BASE + 0x30)
#define QRK_PVP_NSR              (PVP_REGISTER_BASE + 0x34)
#define QRK_PVP_FORGET_NCOUNT    (PVP_REGISTER_BASE + 0x3C)


#define QRK_PVP_TESTCOMP         (PVP_REGISTER_BASE + 0x20)
#define QRK_PVP_TESTCAT          (PVP_REGISTER_BASE + 0x24)
#define QRK_PVP_NID              (PVP_REGISTER_BASE + 0x28)
#define QRK_PVP_GCR              (PVP_REGISTER_BASE + 0x2C)
#define QRK_PVP_RSTCHAIN         (PVP_REGISTER_BASE + 0x30)
#define QRK_PVP_NSR              (PVP_REGISTER_BASE + 0x34)
#define QRK_PVP_FORGET_NCOUNT    (PVP_REGISTER_BASE + 0x3C)

/* DMAC Base address */
#define DMAC_REGISTER_BASE          (0xB0700000)

#define AUX_INTERRUPT_CAUSE             (0x40a)
#define AUX_EXECUTION_RET               (0x400)
#define AUX_EXECUTION_STATUS            (0x402)
#define AUX_EXCEPTION_CAUSE             (0x403)
#define AUX_EXECUTION_ADDR              (0x404)

#define CACHE_ENABLE            0x00
#define CACHE_DISABLE           0x01
#define CACHE_DIRECT            0x00
#define CACHE_CACHE_CONTROLLED  0x20

#define ARC_V2_AUX_IRQ_CTRL_BLINK       (1 << 9)
#define ARC_V2_AUX_IRQ_CTRL_LOOP_REGS  (1 << 10)
#define ARC_V2_AUX_IRQ_CTRL_14_REGS           7
#define ARC_V2_AUX_IRQ_CTRL_16_REGS           8
#define ARC_V2_AUX_IRQ_CTRL_32_REGS          16
#define ARC_V2_DEF_IRQ_LEVEL                 15
#define ARC_V2_WAKE_IRQ_LEVEL                15

#define ARC_V2_IC_CTRL       0x011
#define ARC_V2_AUX_IRQ_CTRL  0x00e

#endif
