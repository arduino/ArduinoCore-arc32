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

/*******************************************************************************
 *
 * Modifications Copyright (c) 2015, Intel Corporation. All rights reserved.
 *
 ******************************************************************************/

#ifndef SPI_PRIV_H_
#define SPI_PRIV_H_

/* SPI software configs */

#define SPI_TX_FIFO_THRESHOLD   (2)
#define SPI_RX_FIFO_THRESHOLD   (0)

/* EIA SPI device registers  */
#define     CTRL                (0x00)  /* SPI Control Register 1 */
#define     SPIEN               (0x02)  /* SPI Enable Register */
#define     TIMING              (0x04)  /* SPI Timing Register */
#define     FTLR                (0x05)  /* SPI FIFO Threshold Levels Register */
#define     TXFLR               (0x07)  /* SPI Transmit FIFO Level Register */
#define     RXFLR               (0x08)  /* SPI Receive  FIFO Level Register */
#define     SR                  (0x09)  /* SPI Status Register */
#define     INTR_STAT           (0x0a)  /* SPI Interrupt Status Register */
#define     INTR_MASK           (0x0b)  /* SPI Interrupt Mask Register */
#define     CLR_INTR            (0x0c)  /* SPI Interrupts Clear Register  */
#define     DR                  (0x0d)  /* SPI Data Register */

// CTRL
#define     SPI_NDF_SET_MASK    (0x0000ffff)
#define     SPI_SRL_SET_MASK    (0xfffff7ff)
#define     SPI_TMOD_SET_MASK   (0xfffffcff)
#define     SPI_SCPL_SET_MASK   (0xffffff7f)
#define     SPI_SCPH_SET_MASK   (0xffffffbf)
#define     SPI_DFS_SET_MASK    (0xfffffff0)

#define     SPI_CLK_ENABLED     (1 << 15)

// SPIEN
#define     SPI_SER_SET_MASK    (0xffffff0f)
#define     SPI_ENB_SET_MASK    (0xfffffffe)

#define     SPI_ENABLE          (1)
#define     SPI_DISABLE         (0)

// TIMING
#define     SPI_SCKDEV_SET_MASK (0xffff0000)

// SR
#define     SPI_STATUS_RFF      (1 << 4)    // receive fifo full
#define     SPI_STATUS_RFNE     (1 << 3)    // receive fifo not empty
#define     SPI_STATUS_TFE      (1 << 2)    // transmit fifo empty
#define     SPI_STATUS_TFNF     (1 << 1)    // transmit fifo not full
#define     SPI_STATUS_BUSY     (1 << 0)

// INTR_STAT INTR_MASK CLR_INTR
#define     RXFIS               (1 << 4)    // receive fifo full (rx it)
#define     RXOIS               (1 << 3)    // receive fifo overflow (err it)
#define     RXUIS               (1 << 2)    // receive fifo underflow (err it)
#define     TXOIS               (1 << 1)    // transmit fifo overflow (err it)
#define     TXEIS               (1 << 0)    // transmit fifo empty (tx it)

#define     SPI_DISABLE_INT     (0x0)
#define     SPI_ENABLE_RX_INT   (RXFIS | RXOIS | RXUIS)
#define     SPI_ENABLE_TX_INT   (TXOIS | TXEIS)
#define     SPI_ENABLE_INT      (SPI_ENABLE_TX_INT | SPI_ENABLE_RX_INT)

#define     SPI_RX_CLR_INTR     (RXFIS)
#define     SPI_ERR_CLR_INTR    (RXOIS | RXUIS | TXOIS)
#define     SPI_TX_CLR_INTR     (TXEIS)

// DR
#define     SPI_POP_DATA        (0x80000000) // pop rx fifo content on DR
#define     SPI_PUSH_DATA       (0xc0000000) // push DR content on tx fifo

/* SPI device states */
#define     SPI_STATE_READY     (0)
#define     SPI_STATE_TRANSMIT  (1)
#define     SPI_STATE_ERROR     (2)

#define     BAUD_DIVISOR        1000


typedef void (*SPI_ISR) ();
typedef void (*IO_CB_FUNC)( uint32_t );


typedef struct {
  IO_CB_FUNC        cb;
} io_cb_t;

/* Private data structure maintained by the driver. */
typedef struct spi_info_struct
{
    uint32_t        reg_base; /* base address of device register set */
    /* TX & RX Buffer and lengths */
    uint32_t        tx_len;
    uint32_t        rx_len;
    uint32_t        tx_count;
    uint32_t        rx_count;
    uint8_t *       tx_buf;
    uint8_t *       rx_buf;
    uint16_t *      rx_buf_16;
    uint8_t         state;
    uint8_t         mode;
    uint8_t         instID;
    /* Data frame Size */
    uint8_t         dfs;
    /* Callbacks */
    IO_CB_FUNC      xfer_cb;
    uint32_t        cb_xfer_data;
    IO_CB_FUNC      err_cb;
    uint32_t        cb_err_data;
    /* Interrupt numbers and handlers */
    uint8_t         rx_vector; /* ISR vectors */
    uint8_t         tx_vector;
    uint8_t         err_vector;
    SPI_ISR         rx_isr; /* SPI device ISRs */
    SPI_ISR         tx_isr;
    SPI_ISR         err_isr;
    uint16_t        fifo_depth;
    /* SSS Interrupt Routing Mask Registers */
    uint32_t        spi_rx_avail_mask;
    uint32_t        spi_tx_req_mask;
    uint32_t        spi_err_mask;
    /* CREG Master clock gate bit location  */
    uint8_t         creg_spi_clk_ctrl;
} spi_info_t, *spi_info_pt;


#define     REG_WRITE( reg, x )   _sr( (unsigned)(x), (unsigned)(dev->reg_base + reg) )
#define     REG_READ( reg )       _lr( (unsigned)(dev->reg_base + reg) )
#define     REG_WRITE_BITS( reg, x, y, len, pos )   REG_WRITE( reg, ( (((x)          & ~( (~(0xffffffff << len)) << pos ))  \
                                                                    | (((y) << pos)  &  ( (~(0xffffffff << len)) << pos ))) ))



#endif /*   SPI_PRIV_H_ */
