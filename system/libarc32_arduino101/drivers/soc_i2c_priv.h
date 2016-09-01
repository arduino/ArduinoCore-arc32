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

#ifndef SOC_I2C_PRIV_H_
#define SOC_I2C_PRIV_H_


#define INT_I2C_0_MASK          (0x448)
#define INT_I2C_1_MASK          (0x44C)

#define SCSS_BASE               SCSS_REGISTER_BASE

#define INT_I2C_MASK_RW         (uint32_t)(0xb0800448)

/* QRK interrupt values */
#define QRK_I2C_INT_MASK        (0x01010100)

/* QRK interrupt values */

#define SOC_ENABLE_RX_TX_INT_I2C                                               \
    (IC_INTR_RX_UNDER | IC_INTR_RX_OVER | IC_INTR_RX_FULL | IC_INTR_TX_OVER |  \
     IC_INTR_TX_EMPTY | IC_INTR_TX_ABRT | IC_INTR_STOP_DET)
#define SOC_ENABLE_TX_INT_I2C                                                  \
    (IC_INTR_TX_OVER | IC_INTR_TX_EMPTY | IC_INTR_TX_ABRT | IC_INTR_STOP_DET)
#define SOC_ENABLE_INT_I2C_SLAVE                                               \
	(IC_INTR_TX_OVER | IC_INTR_TX_ABRT | IC_INTR_RD_REQ | IC_INTR_RX_DONE |    \
    IC_INTR_RX_FULL | IC_INTR_RX_OVER | IC_INTR_RX_UNDER | IC_INTR_STOP_DET)

#define SOC_ENABLE_TX_INT_I2C_SLAVE (0x00000260)
#define SOC_ENABLE_RX_INT_I2C_SLAVE (0x00000204)
#define SOC_DISABLE_ALL_I2C_INT     (0x00000000)

/* IC_CON speed settings (bit 1:2) */

#define I2C_STD_SPEED   01
#define I2C_FAST_SPEED  10
#define I2C_HIGH_SPEED  11

/* IC_CON addressing settings (bit 4) */

#define I2C_7BIT_ADDR   0
#define I2C_10BIT_ADDR  1

/* IC_CON Low count and high count default values */
// TODO verify values for high and fast speed
#define I2C_STD_HCNT 	(CLOCK_SPEED * 4)
#define I2C_STD_LCNT 	(CLOCK_SPEED * 5)
#define I2C_FS_HCNT 	((CLOCK_SPEED * 6) / 8)
#define I2C_FS_LCNT     ((CLOCK_SPEED * 7) / 8)
#define I2C_HS_HCNT 	((CLOCK_SPEED * 6) / 8)
#define I2C_HS_LCNT 	((CLOCK_SPEED * 7) / 8)

/* IC_DATA_CMD Data transfer mode settings (bit 8) */
#define I2C_STATE_READY                 (0)
#define I2C_CMD_SEND                    (1 << 0)
#define I2C_CMD_RECV                    (1 << 1)
#define I2C_CMD_SLAVE_RECV 				I2C_CMD_RECV
#define I2C_CMD_SLAVE_SEND 				(1 << 2)
#define I2C_CMD_ERROR                   (1 << 3)

/* Reset vectors for configuration registers */
#define IC_CON_RST      ((uint32_t)0x7e)
#define IC_TAR_RST      ((uint32_t)0x55)

/* FIFO size */
#define FIFO_SIZE       16

/* APB I2C register offsets */
#define IC_CON                          (0x00)
#define IC_TAR                          (0x04)
#define IC_SAR                          (0x08)
#define IC_HS_MADDR                     (0x0c)
#define IC_DATA_CMD                     (0x10)
#define IC_STD_SCL_HCNT                 (0x14)
#define IC_STD_SCL_LCNT                 (0x18)
#define IC_FS_SCL_HCNT                  (0x1c)
#define IC_FS_SCL_LCNT                  (0x20)
#define IC_HS_SCL_HCNT                  (0x24)
#define IC_HS_SCL_LCNT                  (0x28)
#define IC_INTR_STAT                    (0x2c)
#define IC_INTR_MASK                    (0x30)
#define IC_RAW_INTR_STAT                (0x34)
#define IC_RX_TL                        (0x38)
#define IC_TX_TL                        (0x3c)
#define IC_CLR_INTR                     (0x40)
#define IC_CLR_RX_UNDER                 (0x44)
#define IC_CLR_RX_OVER                  (0x48)
#define IC_CLR_TX_OVER                  (0x4c)
#define IC_CLR_RD_REQ                   (0x50)
#define IC_CLR_TX_ABRT                  (0x54)
#define IC_CLR_RX_DONE                  (0x58)
#define IC_CLR_ACTIVITY                 (0x5c)
#define IC_CLR_STOP_DET                 (0x60)
#define IC_CLR_START_DET                (0x64)
#define IC_CLR_GEN_CALL                 (0x68)
#define IC_ENABLE                       (0x6c)
#define IC_STATUS                       (0x70)
#define IC_TXFLR                        (0x74)
#define IC_RXFLR                        (0x78)
#define IC_SDA_HOLD                     (0x7c)
#define IC_TX_ABRT_SOURCE               (0x80)
#define IC_SLV_DATA_NACK_ONLY           (0x84)
#define IC_DMA_CR                       (0x88)
#define IC_DMA_TDLR                     (0x8c)
#define IC_DMA_RDLR                     (0x90)
#define IC_SDA_SETUP                    (0x94)
#define IC_ACK_GENERAL_CALL             (0x98)
#define IC_ENABLE_STATUS                (0x9c)
#define IC_FS_SPKLEN                    (0xa0)
#define IC_HS_SPKLEN                    (0xa4)
#define IC_CLR_RESTART_DET              (0xa8)
#define IC_COMP_PARAM_1                 (0xf4)
#define IC_COMP_VERSION                 (0xf8)
#define IC_COMP_TYPE                    (0xfc)

#define RESTART_ALLOWED                     0 // WARNING TODO Check whether this will be compile time or driver option

/* Specific bits used to set certain regs */
#define IC_RESTART_BIT                  (1 << 10)
#define IC_CMD_BIT                      (1 << 8) /* part of IC_DATA_CMD register, sets direction of current byte - set (1) = read, unset (0) = write */
#define IC_STOP_BIT                     (1 << 9) /* part of IC_DATA_CMD, by setting this bit last byte of transfer is indicated */
#define IC_TX_INTR_MODE                 (1 << 8) /* part of IC_CON registers - set TX interrupt mode*/
#define IC_ENABLE_BIT                   (1 << 0)
#define IC_ABORT_BIT                    (1 << 1)
#define IC_SLAVE_DISABLE_BIT            (1 << 6)
#define IC_STOP_DET_IFADDRESSED         (1 << 7)
#define IC_MASTER_EN_BIT                (1 << 0)
#define IC_RESTART_EN_BIT               (1 << 5)
#define IC_MASTER_ADDR_MODE_BIT         (1 << 4)
#define IC_SLAVE_ADDR_MODE_BIT          (1 << 3)
#define IC_ACTIVITY                     (1 << 0)
#define IC_TAR_10BITADDR_MASTER         (1 << 12)

/* Out of convention */
#define IC_SPEED_POS                    2

#define ZERO_REG                        ((uint32_t)(0x00000000))

/* Clock gating */
#define CLK_I2C_0_ENABLE                (1 << 19)
#define CLK_I2C_1_ENABLE                (1 << 20)
#define CLK_I2C_0_DISABLE               (~CLK_I2C_0_ENABLE)
#define CLK_I2C_1_DISABLE               (~CLK_I2C_1_ENABLE)

#define STATUS_DELAY    1000 /* User configurable option - waits for controller status change (no interrupt available - occurs only when two transactions are initiated in short time) - normally shouldn't take more than 2-3 cycles, cycle is exited when set */

/* Interrupt handler statuses - bit masking for IC_RAW_INTR_STATUS register - passed by callbacks */
#define IC_INTR_RX_UNDER 				(1 << 0)
#define IC_INTR_RX_OVER 				(1 << 1)  		/* RX fifo overflow */
#define IC_INTR_RX_FULL 				(1 << 2)  		/* RX fifo full */
#define IC_INTR_TX_OVER 				(1 << 3)  		/* TX fifo overflow */
#define IC_INTR_TX_EMPTY 				(1 << 4) 		/* TX fifo empty */
#define IC_INTR_RD_REQ 					(1 << 5)   		/* SLAVE - read request received */
#define IC_INTR_TX_ABRT 				(1 << 6)  		/* TX aborted - TODO reason */
#define IC_INTR_RX_DONE 				(1 << 7)  		/* SLAVE - read on master over */
#define IC_INTR_ACTIVITY    			(1 << 8) 		/* Activity on I2C - automatically cleared by ISR */
#define IC_INTR_STOP_DET 				(1 << 9)    	/* STOP condition on line */
#define IC_INTR_START_DET 				(1 << 10)   	/* START condition on line */
#define IC_INTR_GEN_CALL 				(1 << 11)    	/* General call issued - disabled */
#define IC_INTR_RESTART_DET 			(1 << 12) 		/* SLAVE - restart condition - disabled  */
#define IC_INTR_MST_ON_HOLD 			(1 << 13) 		/* Master on hold - disabled */

#define IC_STATUS_ACTIVITY 				(1 << 0)
#define IC_STATUS_TFNF 					(1 << 1)
#define IC_STATUS_TFE 					(1 << 2)
#define IC_STATUS_RFNE 					(1 << 3)
#define IC_STATUS_RFF 					(1 << 4)
#define IC_STATUS_MST_ACTIVITY 			(1 << 5)
#define IC_STATUS_SLV_ACTIVITY 			(1 << 6)

#endif
