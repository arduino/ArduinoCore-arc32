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

#ifndef __PORTABLE_H__
#define __PORTABLE_H__

#include "soc_register.h"
#include "../bootcode/interrupt.h"

#define DECLARE_INTERRUPT_HANDLER
#define SET_INTERRUPT_HANDLER(_vec_, _isr_) \
    do { \
        interrupt_connect((_vec_), (_isr_)); \
        interrupt_enable((_vec_)); \
    } while(0)

#define WRITE_ARC_REG(value, reg) \
    __builtin_arc_sr((value), (volatile uint32_t)(reg))

#define READ_ARC_REG(reg) \
    __builtin_arc_lr((volatile uint32_t)(reg))

#define CLEAR_ARC_BIT(reg, bit) \
 do { \
    uint32_t saved = interrupt_lock(); \
    WRITE_ARC_REG(READ_ARC_REG(reg) & ~(1 << bit), reg); \
    interrupt_unlock(saved); \
 } while(0)

#define SET_ARC_BIT(reg, bit) \
 do { \
    uint32_t saved = interrupt_lock(); \
    WRITE_ARC_REG(READ_ARC_REG(reg) | (1 << bit), reg); \
    interrupt_unlock(saved); \
 } while(0)

#define CLEAR_ARC_MASK(reg, mask) \
 do { \
    uint32_t saved = interrupt_lock(); \
    WRITE_ARC_REG(READ_ARC_REG(reg) & ~(mask), reg); \
    interrupt_unlock(saved); \
 } while(0)

#define SET_ARC_MASK(reg, mask) \
 do { \
    uint32_t saved = interrupt_lock(); \
    WRITE_ARC_REG(READ_ARC_REG(reg) | (mask), reg); \
    interrupt_unlock(saved); \
 } while(0)

#define CLEAR_MMIO_BIT(reg, bit) \
 do { \
    uint32_t saved = interrupt_lock(); \
    MMIO_REG_VAL(reg) &= ~(1 << bit); \
    interrupt_unlock(saved); \
 } while(0)

#define SET_MMIO_BIT(reg, bit) \
 do { \
    uint32_t saved = interrupt_lock(); \
    MMIO_REG_VAL(reg) |= (1 << bit); \
    interrupt_unlock(saved); \
 } while(0)

#define CLEAR_MMIO_MASK(reg, mask) \
 do { \
    uint32_t saved = interrupt_lock(); \
    MMIO_REG_VAL(reg) &= ~(mask); \
    interrupt_unlock(saved); \
 } while(0)

#define SET_MMIO_MASK(reg, mask) \
 do { \
    uint32_t saved = interrupt_lock(); \
    MMIO_REG_VAL(reg) |= (mask); \
    interrupt_unlock(saved); \
 } while(0)

#endif
