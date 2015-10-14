/* aux_regs.h - ARCv2 auxiliary registers definitions */

/*
 * Copyright (c) 2014 Wind River Systems, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1) Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2) Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3) Neither the name of Wind River Systems nor the names of its contributors
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

/*
DESCRIPTION

Definitions for auxiliary registers.
*/

#ifndef _ARC_V2_AUX_REGS__H_
#define _ARC_V2_AUX_REGS__H_

#define ARC_V2_LP_START         0x002
#define ARC_V2_LP_END           0x003
#define ARC_V2_STATUS32         0x00a
#define ARC_V2_STATUS32_P0      0x00b
#define ARC_V2_AUX_IRQ_CTRL     0x00e
#define ARC_V2_IC_IVIC		0x010
#define ARC_V2_IC_CTRL          0x011
#define ARC_V2_TMR0_COUNT       0x021
#define ARC_V2_TMR0_CONTROL     0x022
#define ARC_V2_TMR0_LIMIT       0x023
#define ARC_V2_IRQ_VECT_BASE    0x025
#define ARC_V2_AUX_IRQ_ACT      0x043
#define ARC_V2_TMR1_COUNT       0x100
#define ARC_V2_TMR1_CONTROL     0x101
#define ARC_V2_TMR1_LIMIT       0x102
#define ARC_V2_IRQ_PRIO_PEND    0x200
#define ARC_V2_AUX_IRQ_HINT     0x201
#define ARC_V2_IRQ_PRIORITY     0x206
#define ARC_V2_ERET             0x400
#define ARC_V2_ERSTATUS         0x402
#define ARC_V2_ECR              0x403
#define ARC_V2_EFA              0x404
#define ARC_V2_ICAUSE           0x40a
#define ARC_V2_IRQ_SELECT       0x40b
#define ARC_V2_IRQ_ENABLE       0x40c
#define ARC_V2_IRQ_TRIGGER      0x40d
#define ARC_V2_IRQ_STATUS       0x40f
#define ARC_V2_IRQ_PULSE_CANCEL 0x415
#define ARC_V2_IRQ_PENDING      0x416

#define ARC_V2_TMR0_CONTROL_IP_MASK	(0x01 << 3)
#define AUX_IRQ_CTRL_SAVE_ALL		(0x1F | (0x01 << 9) | (0x01 << 10))

/* STATUS32/STATUS32_P0 bits */
#define ARC_V2_STATUS32_H     (1 <<  0)
#define ARC_V2_STATUS32_E(x)  ((x) << 1)
#define ARC_V2_STATUS32_AE_BIT 5
#define ARC_V2_STATUS32_AE    (1 <<  ARC_V2_STATUS32_AE_BIT)
#define ARC_V2_STATUS32_DE    (1 <<  6)
#define ARC_V2_STATUS32_U     (1 <<  7)
#define ARC_V2_STATUS32_V     (1 <<  8)
#define ARC_V2_STATUS32_C     (1 <<  9)
#define ARC_V2_STATUS32_N     (1 << 10)
#define ARC_V2_STATUS32_Z     (1 << 11)
#define ARC_V2_STATUS32_L     (1 << 12)
#define ARC_V2_STATUS32_DZ    (1 << 13)
#define ARC_V2_STATUS32_SC    (1 << 14)
#define ARC_V2_STATUS32_ES    (1 << 15)
#define ARC_V2_STATUS32_RB(x) ((x) << 16)
#define ARC_V2_STATUS32_IE    (1 << 31)

#define ARC_V2_INT_DISABLE 0
#define ARC_V2_INT_ENABLE  1
#define ARC_V2_INT_LEVEL 0
#define ARC_V2_INT_PULSE 1

/* exception cause register masks */
#define ARC_V2_ECR_VECTOR(X)    ((X & 0xff0000) >> 16)
#define ARC_V2_ECR_CODE(X)      ((X & 0xff00) >> 8)
#define ARC_V2_ECR_PARAMETER(X) (X & 0xff)

#define aux_reg_read(reg)  __builtin_arc_lr(reg)
#define aux_reg_write(reg, val) __builtin_arc_sr((unsigned int)val, reg)

#endif /* _ARC_V2_AUX_REGS__H_ */
