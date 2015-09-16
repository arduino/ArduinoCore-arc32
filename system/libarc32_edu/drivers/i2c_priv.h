/*******************************************************************************
 *
 * Synopsys DesignWare Sensor and Control IP Subsystem IO Software Driver and
 * documentation (hereinafter, "Software") is an Unsupported proprietary work
 * of Synopsys, Inc. unless otherwise expressly agreed to in writing between
 * Synopsys and you.
 *
 * The Software IS NOT an item of Licensed Software or Licensed Product under
 * any End User Software License Agreement or Agreement for Licensed Product
 * with Synopsys or any supplement thereto. You are permitted to use and
 * redistribute this Software in source and binary forms, with or without
 * modification, provided that redistributions of source code must retain this
 * notice. You may not view, use, disclose, copy or distribute this file or
 * any information contained herein except pursuant to this license grant from
 * Synopsys. If you do not agree with this notice, including the disclaimer
 * below, then you are not authorized to use the Software.
 *
 * THIS SOFTWARE IS BEING DISTRIBUTED BY SYNOPSYS SOLELY ON AN "AS IS" BASIS
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE HEREBY DISCLAIMED. IN NO EVENT SHALL SYNOPSYS BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
 * DAMAGE.
 *
 ******************************************************************************/

#ifndef I2C_PRIV_H_
#define I2C_PRIV_H_

#include <stdint.h>

#include "drivers/eiaextensions.h"

/*  EIA I2C device registers        */
#define     I2C_CON                 (0x00)   // i2c control
#define     I2C_DATA_CMD            (0x01)   // i2c Rx/Tx Data Buffer and Command
#define     I2C_SS_SCL_CNT          (0x02)   // Standard Speed i2c SCL High/Low Counts */
#define     I2C_FS_SCL_CNT          (0x04)   // Fast Speed i2c SCL Low High/Low Counts */
#define     I2C_INTR_STAT           (0x06)   // i2c Interrupt Status */
#define     I2C_INTR_MASK           (0x07)   // i2c Interrupt Mask */
#define     I2C_TL                  (0x08)   // i2c Tx/Rx FIFO Thresholds */
#define     I2C_CLR_INTR            (0x0a)   // Clear combined and Individual Interrupts */
#define     I2C_STATUS              (0x0b)   // i2c Status */
#define     I2C_TXFLR               (0x0c)   // Transmit FIFO Level Register */
#define     I2C_RXFLR               (0x0d)   // Receive FIFO Level Register */
#define     I2C_SDA_CONFIG          (0x0e)   // SDA Hold/Setup Time Length Reg */
#define     I2C_TX_ABRT_SOURCE      (0x0f)   // i2c Transmit Abort Status Reg */
#define     I2C_ENABLE_STATUS       (0x11)   // Enable Status Register */


/* Interrupt Register Fields */
#define     R_START_DETECTED        (1 << 10)
#define     R_STOP_DETECTED         (1 << 9)
#define     R_ACTIVITY              (1 << 8)
#define     R_RX_DONE               (1 << 7)
#define     R_TX_ABRT               (1 << 6)
#define     R_RD_REQ                (1 << 5)
#define     R_TX_EMPTY              (1 << 4)
#define     R_TX_OVER               (1 << 3)
#define     R_RX_FULL               (1 << 2)
#define     R_RX_OVER               (1 << 1)
#define     R_RX_UNDER              (1 << 0)


/*  I2C Status Register Fields. */
#define     I2C_STATUS_ACTIVITY     (0x01)
#define     I2C_STATUS_TFNF         (0x02) /* (1 << 1) */
#define     I2C_STATUS_TFE          (0x04) /* (1 << 2) */
#define     I2C_STATUS_RFNE         (0x08) /* (1 << 3) */
#define     I2C_STATUS_RFF          (0x10) /* (1 << 4) */
#define     I2C_STATUS_MASTER_ACT   (0x20) /* (1 << 5) */
#define     I2C_STATUS_SLAVE_ACT    (0x40) /* (1 << 6) */

/* I2C TX Abort Source Register Fields */
#define     I2C_ABRT_SLVFLUSH_TXFIFO  (1 << 13)
#define     I2C_TX_FLUSH_COUNT_POS    24

/* Other macros. */
#define     I2C_CLK_ENABLED         (1 << 31)
#define     ENABLE_I2C              (1)
#define     DISABLE_I2C             (0)
#define     I2C_RESTART_CMD         (0x400)
#define     I2C_STOP_CMD            (0x200)
#define     I2C_READ_CMD            (0x100)
#define     I2C_PUSH_DATA           (0xc0000000)
#define     I2C_POP_DATA            (0x80000000)
#define     I2C_RESTART_EN          (1 << 7)
#define     I2C_SLAVE_DSB           (1 << 8)
#define     TX_ABORT                (1 << 1)

#define     I2C_ENABLE_MASTER       (1 << 0)
#define     I2C_ENABLE_SLAVE        (1 << 6)

/* I2C default interrupt enable & disable macro's. */
#define     I2C_INT_ENB             (R_TX_ABRT | R_RX_FULL | R_RX_OVER | R_STOP_DETECTED)
#define     I2C_INT_DSB             (0x00)

/* I2C device state. */
#define     I2C_STATE_CLOSED        (0)
#define     I2C_STATE_READY         (1)
#define     I2C_STATE_RECEIVE       (2)
#define     I2C_STATE_TRANSMIT      (3)
#define     I2C_STATE_DISABLED      (4)

/* I2C working speeds. */
#define     STANDARD_SPEED          (0x01)
#define     FAST_SPEED              (0x02)

#ifndef __GNUC__
typedef _Interrupt void (*I2C_ISR) ();
#else
typedef void (*I2C_ISR)();
#endif
typedef void (*IO_CB_FUNC)(uint32_t);

/* Private data structure maintained by the driver */
typedef struct i2c_info {
    uint32_t        reg_base;       // base address of device register set
    uint8_t         instID;
    uint8_t         state;
    uint16_t        fifo_depth;
    /* Transmitted bytes. */
    uint32_t        total_read_bytes;
    uint32_t        total_write_bytes;
    uint32_t        tx_len;
    uint32_t        rx_len;
    uint32_t        rx_tx_len;      // tx_len + rx_len
    uint8_t         *i2c_write_buff;
    uint8_t         *i2c_read_buff;

    /* Callbacks */
    IO_CB_FUNC      tx_cb;
    IO_CB_FUNC      rx_cb;
    IO_CB_FUNC      err_cb;
    uint32_t        cb_rx_data;
    uint32_t        cb_tx_data;
    uint32_t        cb_err_data;
    /* Interrupt numbers and handlers */
    uint8_t         vector_err;             // ISR vector, error interrupt
    uint8_t         vector_rx_avail;        // ISR vector, rx interrupt
    uint8_t         vector_tx_req;          // ISR vector, tx interrupt
    uint8_t         vector_rd_req;          // ISR vector, read request interrupt
    uint8_t         vector_stop_detected;   // ISR vector, stop detect interrupt
    I2C_ISR         isr_err;                // I2C device ISR, error interrupt
    I2C_ISR         isr_rx_avail;           // I2C device ISR, rx interrupt
    I2C_ISR         isr_tx_req;             // I2C device ISR, tx interrupt
    I2C_ISR         isr_rd_req;             // I2C device ISR, read request interrupt
    I2C_ISR         isr_stop_detected;      // I2C device ISR, stop detect interrupt

    /* status return code */
    uint32_t        status_code;
    /* SSS Interrupt Routing Mask Registers */
    uint32_t        i2c_rx_avail_mask;
    uint32_t        i2c_tx_req_mask;
    uint32_t        i2c_err_mask;
    uint32_t        i2c_stop_detected_mask;
    /* CREG Master clock gate bit location  */
    uint8_t         creg_i2c_clk_ctrl;
} i2c_info_t, *i2c_info_pt;


/*  I2C software configs   */
#define     I2C_ALLOW_RESTART       (1)
#define     I2C_TX_FIFO_THRESHOLD   (0)
#define     I2C_RX_FIFO_THRESHOLD   (0)    //  Value 0 means 1 element in Rx FIFO triggers interrupt

/* I2C IOCTLs default values    */
#define     I2C_SPKLEN              (0x05)
#define     I2C_SETUP_TIME          (0x64)
#define     I2C_HOLD_TIME           (0x01)
#define     I2C_SPEED_MODE          FAST_SPEED
#define     I2C_SLAVE_ADDR          (0x55)   // Default for IO_I2C_SLAVE_SET_ADDR and IO_I2C_MASTER_SET_TARGET_ADDR
#define     I2C_SS_SCL_HIGH_COUNT   (0x008A)
#define     I2C_SS_SCL_LOW_COUNT    (0x009D)
#define     I2C_FS_SCL_HIGH_COUNT   (0x0013)
#define     I2C_FS_SCL_LOW_COUNT    (0x0027)

#define     REG_WRITE( reg, x )   _sr( (unsigned)(x), (unsigned)(dev->reg_base + reg) )
#define     REG_READ( reg )       _lr( (unsigned)(dev->reg_base + reg) )
#define     REG_WRITE_BITS( reg, x, y, len, pos )   REG_WRITE( reg, ( (((x)          & ~( (~(0xffffffff << len)) << pos ))  \
                                                                    | (((y) << pos)  &  ( (~(0xffffffff << len)) << pos ))) ))

#endif
