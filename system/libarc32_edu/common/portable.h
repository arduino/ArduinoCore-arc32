/* aux_regs.h - ARCv2 auxiliary registers definitions */

/*
 * Copyright (c) 2014 Wind River Systems, Inc.
 *
 * The right to copy, distribute or otherwise make use of this software
 * may be licensed only pursuant to the terms of an applicable Wind River
 * license agreement.
 */

#ifndef __PORTABLE_H__
#define __PORTABLE_H__

#include "soc_register.h"
#include "../bootcode/interrupt.h"

#define DECLARE_INTERRUPT_HANDLER
#define SET_INTERRUPT_HANDLER(_vec_, _isr_) \
    do { \
        interrupt_connect((_vec_), (_isr_), NULL); \
        interrupt_enable((_vec_)); \
    } while(0)

#define WRITE_ARC_REG(value, reg) \
    __builtin_arc_sr(value, (volatile uint32_t)reg)

#define READ_ARC_REG(reg) \
    __builtin_arc_lr((volatile uint32_t)reg)

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
