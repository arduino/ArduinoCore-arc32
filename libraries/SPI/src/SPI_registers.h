/*
 * Copyright (c) 2015 Intel Corporation.  All rights reserved.
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.

 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.

 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
 *
 */
#ifndef _SPI_REGISTERS_H_
#define _SPI_REGISTERS_H_

#include "portable.h"

/* Macro to access SPI1 Master controller register offset */
#define SPI1_M_REG_VAL(reg) \
    MMIO_REG_VAL_FROM_BASE(SOC_MST_SPI1_REGISTER_BASE, (reg))

/* SoC SPI device register offsets  */
#define     CTRL0                       (0x00)              /* SoC SPI Control Register 1 */
#define     SPIEN                       (0x08)              /* SoC SPI Enable Register */
#define     SER                         (0x10)              /* SoC SPI Slave Enable Register */
#define     BAUDR                       (0x14)              /* SoC SPI Baud Rate Select */
#define     TXFL                        (0x20)              /* SoC SPI Transmit FIFO Level Register */
#define     RXFL                        (0x24)              /* SoC SPI Receive FIFO Level Register */
#define     SR                          (0x28)              /* SoC SPI Status Register */
#define     IMR                         (0x2C)              /* SoC SPI Interrupt Mask Register */
#define     DR                          (0x60)              /* SoC SPI Data Register */

/* SPI specific macros */
#define     SPI_DISABLE_INT             (0x0)               /* Disable SoC SPI Interrupts */
#define     SPI_ENABLE                  (0x1)               /* Enable SoC SPI Device */
#define     SPI_DISABLE                 (0x0)               /* Disable SoC SPI Device */
#define     SPI_STATUS_BUSY             (0x1)               /* Busy status */

#define     SPI_8_BIT                   (7)                 /*  8-bit frame size */
#define     SPI_16_BIT                  (15)                /* 16-bit frame size */
#define     SPI_24_BIT                  (23)                /* 24-bit frame size */
#define     SPI_32_BIT                  (31)                /* 32-bit frame size */

#define     SPI_MODE_MASK               (0xC0)              /* CPOL = bit 7, CPHA = bit 6 on CTRL0 */
#define     SPI_MODE_SHIFT              (6)
#define     SPI_FSIZE_MASK              (0x1F0000)          /* Valid frame sizes: 1- 32 bits */
#define     SPI_FSIZE_SHIFT             (16)
#define     SPI_CLOCK_MASK              (0xFFFE)            /* Clock divider: any even value in the ragnge 2-65534 */

#define     SPI_FIFO_DEPTH              (8UL)

#define     ENABLE_SPI_MASTER_1         (0x1 << 15)
#define     DISABLE_SPI_MASTER_1        (~ENABLE_SPI_MASTER_1)

#define     SPI_BASE_CLOCK              (CLOCK_SPEED*1000*1000) /* CLOCK_SPEED in MHz */

/* Utility function to reverse the bit order of a 32-bit word */
static inline uint32_t arc32_bit_reverse(register uint32_t src, register uint32_t count) 
{
    register uint32_t dst = 0;
    /* Copy the specified number of least-significant bits from src to dst,
     * reversing the bit order in the process.  Uses rotate-through-carry
     * instructions in a zero-overhead loop.
     */
    asm volatile ("mov lp_count, %4\n"
                  "lp 1f\n"
                  "rrc.f %1, %3\n"
                  "rlc.f %0, %2\n"
                  "1:\n"
                  : "=r"(dst), "=r"(src)
                  : "0"(dst), "1"(src), "r"(count)
                  : "lp_count"
        );
    return dst;
}
#define SPI_REVERSE_8(b)  arc32_bit_reverse((b), 8)
#define SPI_REVERSE_16(b) arc32_bit_reverse((b), 16)
#define SPI_REVERSE_24(b) arc32_bit_reverse((b), 24)
#define SPI_REVERSE_32(b) arc32_bit_reverse((b), 32)

#endif // _SPI_REGISTERS_H_
