/* --------------------------------------------------------------------
**
** Synopsys DesignWare AMBA Software Driver Kit and
** documentation (hereinafter, "Software") is an Unsupported
** proprietary work of Synopsys, Inc. unless otherwise expressly
** agreed to in writing between Synopsys and you.
**
** The Software IS NOT an item of Licensed Software or Licensed
** Product under any End User Software License Agreement or Agreement
** for Licensed Product with Synopsys or any supplement thereto. You
** are permitted to use and redistribute this Software in source and
** binary forms, with or without modification, provided that
** redistributions of source code must retain this notice. You may not
** view, use, disclose, copy or distribute this file or any information
** contained herein except pursuant to this license grant from Synopsys.
** If you do not agree with this notice, including the disclaimer
** below, then you are not authorized to use the Software.
**
** THIS SOFTWARE IS BEING DISTRIBUTED BY SYNOPSYS SOLELY ON AN "AS IS"
** BASIS AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
** LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
** FOR A PARTICULAR PURPOSE ARE HEREBY DISCLAIMED. IN NO EVENT SHALL
** SYNOPSYS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
** EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
** PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
** PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
** OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
** (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
** USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
** DAMAGE.
**
** --------------------------------------------------------------------
*/

/****************************************************************************************
 *
 * Modifications Copyright (c) 2015, Intel Corporation. All rights reserved.
 *
 ***************************************************************************************/

#include "intel_qrk_i2c.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>

#include "clk_system.h"
#include "platform.h"
#include "portable.h"
#include "scss_registers.h"

#include "soc_i2c_priv.h"

typedef uint8_t DATA_BUFF;
typedef void (*MY_ISR)();

typedef struct {
    uint32_t BASE; // base address of device register set

    volatile uint8_t
        state; /* last direction of transfer - used by ISR to call right
                  callback */
    uint16_t fifo_depth;
    /* Transmitted bytes. */
    uint32_t total_read_bytes;
    uint32_t total_write_bytes;
    uint32_t tx_len;
    uint32_t rx_len;
    uint32_t rx_tx_len; // tx_len + rx_len
    uint8_t *i2c_write_buff;
    uint8_t *i2c_read_buff;

    volatile uint8_t tx_watermark; /* TX watermark level */
    volatile uint8_t rx_watermark; /* RX watermark level */

    /* Callbacks */
    MY_ISR ISR;          /* pointer to ISR function */
    i2c_callback tx_cb;  /* Write callback */
    i2c_callback rx_cb;  /* Read callback */
    i2c_callback err_cb; /* Error callback */
    uint32_t cb_rx_data; /* pass data back for callbacks above (aligned to
                            callbacks) */
    uint32_t cb_tx_data;
    uint32_t cb_err_data;

    uint8_t send_restart;
    uint8_t send_stop;

    struct clk_gate_info_s clk_gate_info; /*!< clock gate data */

    /* Config params */
    I2C_SPEED speed;
    I2C_ADDR_MODE addr_mode;
    I2C_MODE_TYPE mode;
    uint32_t slave_addr;
    /* Slave specific */
    SOC_I2C_SLAVE_MODE slave_mode;
} i2c_internal_data_t;

/* device config keeper */
static i2c_internal_data_t devices[2];

static void soc_i2c_abort_transfer(i2c_internal_data_t *dev)
{
    volatile uint64_t timeout = 1000000 * 32;
    do {
        MMIO_REG_VAL_FROM_BASE(dev->BASE, IC_ENABLE) = IC_ABORT_BIT;

        if ((MMIO_REG_VAL_FROM_BASE(dev->BASE, IC_ENABLE) & IC_ABORT_BIT) == 0)
            return;
    } while (timeout-- > 0);

    return;
}

static void soc_i2c_enable_device(i2c_internal_data_t *dev, bool enable)
{
    volatile uint64_t timeout = 1000000 * 32;
    do {
        MMIO_REG_VAL_FROM_BASE(dev->BASE, IC_ENABLE) =
            enable ? IC_ENABLE_BIT : 0;

        if ((MMIO_REG_VAL_FROM_BASE(dev->BASE, IC_ENABLE_STATUS) &
             IC_ENABLE_BIT) == (enable ? IC_ENABLE_BIT : 0))
            return;
    } while (timeout-- > 0);

    return;
}

static void soc_end_data_transfer(i2c_internal_data_t *dev)
{
    uint32_t state = dev->state;

    if ((dev->mode == I2C_MASTER) && (dev->send_stop)) {
        soc_i2c_enable_device(dev, false);
    }

    dev->state = I2C_STATE_READY;

    if (I2C_CMD_RECV == state) {
        if (NULL != dev->rx_cb) {
            dev->cb_rx_data = dev->total_read_bytes;
            dev->total_read_bytes = 0;
            dev->rx_cb(dev->cb_rx_data);
        }
    } else if (I2C_CMD_SEND == state) {
        if (NULL != dev->tx_cb) {
            dev->tx_cb(dev->cb_tx_data);
        }
    } else if (I2C_CMD_SLAVE_SEND == state) {
        dev->tx_len = dev->total_write_bytes = 0;
    } else if (I2C_CMD_ERROR == state) {
        if (NULL != dev->err_cb) {
            dev->err_cb(dev->cb_err_data);
        }
    }
}

static void soc_i2c_recv_data(i2c_internal_data_t *dev)
{
    uint32_t rx_valid = 0;

    rx_valid = MMIO_REG_VAL_FROM_BASE(dev->BASE, IC_RXFLR);

    for (; dev->total_read_bytes < dev->rx_len && rx_valid > 0; rx_valid--) {
        dev->i2c_read_buff[dev->total_read_bytes++] =
            MMIO_REG_VAL_FROM_BASE(dev->BASE, IC_DATA_CMD);
    }

    return;
}

static void soc_i2c_xmit_data(i2c_internal_data_t *dev)
{
    uint32_t tx_limit, rx_limit;

    if (dev->mode == I2C_SLAVE) {
        if (dev->total_write_bytes == dev->tx_len) {
            if (NULL != dev->tx_cb) {
                dev->tx_cb(dev->cb_tx_data);
            }
        }
    }

    if (!dev->rx_tx_len) {
        return;
    }

    tx_limit = dev->fifo_depth - MMIO_REG_VAL_FROM_BASE(dev->BASE, IC_TXFLR);
    rx_limit = dev->fifo_depth - MMIO_REG_VAL_FROM_BASE(dev->BASE, IC_RXFLR);

    while (dev->total_write_bytes < dev->rx_tx_len && tx_limit > 0 &&
           rx_limit > 0) {
        uint32_t cmd = 0;
        if (dev->send_restart) {
            cmd |= IC_RESTART_BIT;
            dev->send_restart = false;
        }

        if (((dev->total_write_bytes + 1) == dev->rx_tx_len) &&
            dev->send_stop) {
            cmd |= IC_STOP_BIT;
        }

        if (dev->tx_len > 0) { // something to transmit
            cmd |= dev->i2c_write_buff[dev->total_write_bytes];
        } else {
            cmd |= IC_CMD_BIT;
            rx_limit--;
        }
        MMIO_REG_VAL_FROM_BASE(dev->BASE, IC_DATA_CMD) = cmd;
        tx_limit--;
        dev->total_write_bytes++;
    }
}

/* SOC I2C interrupt handler */
static void soc_i2c_isr(i2c_internal_data_t *dev)
{
    volatile uint32_t stat = 0;
    stat = MMIO_REG_VAL_FROM_BASE(dev->BASE, IC_INTR_STAT);

    dev->cb_err_data = 0;

    if (stat & IC_INTR_RX_UNDER)
        MMIO_REG_VAL_FROM_BASE(dev->BASE, IC_CLR_RX_UNDER);

    if (stat & IC_INTR_RX_OVER)
        MMIO_REG_VAL_FROM_BASE(dev->BASE, IC_CLR_RX_OVER);

    if (stat & IC_INTR_TX_OVER)
        MMIO_REG_VAL_FROM_BASE(dev->BASE, IC_CLR_TX_OVER);

    if (stat & IC_INTR_RD_REQ)
        MMIO_REG_VAL_FROM_BASE(dev->BASE, IC_CLR_RD_REQ);

    if (stat & IC_INTR_TX_ABRT) {
        dev->cb_err_data = MMIO_REG_VAL_FROM_BASE(dev->BASE, IC_TX_ABRT_SOURCE);
        MMIO_REG_VAL_FROM_BASE(dev->BASE, IC_CLR_TX_ABRT);
    }

    if (stat & IC_INTR_RX_DONE)
        MMIO_REG_VAL_FROM_BASE(dev->BASE, IC_CLR_RX_DONE);

    if (stat & IC_INTR_ACTIVITY)
        MMIO_REG_VAL_FROM_BASE(dev->BASE, IC_CLR_ACTIVITY);

    if (stat & IC_INTR_STOP_DET)
        MMIO_REG_VAL_FROM_BASE(dev->BASE, IC_CLR_STOP_DET);

    if (stat & IC_INTR_START_DET)
        MMIO_REG_VAL_FROM_BASE(dev->BASE, IC_CLR_START_DET);

    if (stat & IC_INTR_GEN_CALL)
        MMIO_REG_VAL_FROM_BASE(dev->BASE, IC_CLR_GEN_CALL);

    if (stat & (IC_INTR_TX_ABRT | IC_INTR_TX_OVER | IC_INTR_RX_OVER |
                IC_INTR_RX_UNDER)) {
        dev->state = I2C_CMD_ERROR;
        dev->send_stop = true;
        goto done;
    }

    if (!dev->send_stop && dev->total_write_bytes == dev->rx_tx_len &&
        dev->total_read_bytes == dev->rx_len) {
        int mask = MMIO_REG_VAL_FROM_BASE(dev->BASE, IC_INTR_MASK);
        mask &= ~IC_INTR_TX_EMPTY;
        MMIO_REG_VAL_FROM_BASE(dev->BASE, IC_INTR_MASK) = mask;
        goto done;
    }

    if (stat & IC_INTR_RX_FULL) {
        dev->state = I2C_CMD_RECV;
        soc_i2c_recv_data(dev);
    }

    if (stat & IC_INTR_TX_EMPTY) {
        soc_i2c_xmit_data(dev);
    }

    if (stat & IC_INTR_RD_REQ) {
        dev->state = I2C_CMD_SLAVE_SEND;
        soc_i2c_xmit_data(dev);
    }

    if (stat & IC_INTR_RX_DONE) {
        goto done;
    }

    if (stat & IC_INTR_STOP_DET) {
        goto done;
    }

    return;

done:
    soc_end_data_transfer(dev);
}

DECLARE_INTERRUPT_HANDLER void isr_dev_0()
{
    soc_i2c_isr(&devices[0]);
}

DECLARE_INTERRUPT_HANDLER void isr_dev_1()
{
    soc_i2c_isr(&devices[1]);
}

static void soc_i2c_master_init_transfer(i2c_internal_data_t *dev)
{
    volatile uint32_t ic_con = 0, ic_tar = 0;

    soc_i2c_enable_device(dev, false);

    /* Setup IC_CON */
    ic_con = MMIO_REG_VAL_FROM_BASE(dev->BASE, IC_CON);

    /* Set addressing mode - (initialisation = 7 bit) */
    if (I2C_10_Bit == dev->addr_mode) {
        ic_con |= IC_MASTER_ADDR_MODE_BIT;
        ic_tar = IC_TAR_10BITADDR_MASTER;
    } else {
        ic_con &= ~IC_MASTER_ADDR_MODE_BIT;
    }

    MMIO_REG_VAL_FROM_BASE(dev->BASE, IC_CON) = ic_con;

    /* Set slave address */
    MMIO_REG_VAL_FROM_BASE(dev->BASE, IC_TAR) = ic_tar | dev->slave_addr;

    /* Disable interrupts */
    MMIO_REG_VAL_FROM_BASE(dev->BASE, IC_INTR_MASK) = SOC_DISABLE_ALL_I2C_INT;

    /* Clear interrupts */
    MMIO_REG_VAL_FROM_BASE(dev->BASE, IC_CLR_INTR);

    soc_i2c_enable_device(dev, true);

    /* Enable necesary interrupts */
    MMIO_REG_VAL_FROM_BASE(dev->BASE, IC_INTR_MASK) = SOC_ENABLE_RX_TX_INT_I2C;

    return;
}

static DRIVER_API_RC soc_i2c_init(i2c_internal_data_t *dev)
{
    volatile uint32_t ic_con = 0;
    DRIVER_API_RC rc = DRV_RC_OK;
    uint32_t i = 0;

    dev->send_stop = true;

    soc_i2c_enable_device(dev, false);

    /* Setup IC_CON */
    ic_con = IC_STOP_DET_IFADDRESSED;

    /* Set master or slave mode - (initialisation = slave) */
    if (I2C_MASTER == dev->mode) {
        ic_con |= IC_SLAVE_DISABLE_BIT; /* SET TRUE */
        ic_con |= IC_MASTER_EN_BIT;
    } else {
        ic_con &= (~IC_SLAVE_DISABLE_BIT); /* SET TRUE */
        ic_con &= (~IC_MASTER_EN_BIT);
    }

    /* Set restart - so far compile time option - (initialisation = OFF) */
    ic_con |= IC_RESTART_EN_BIT;

    /* Set addressing mode - (initialisation = 7 bit) */
    if (I2C_10_Bit == dev->addr_mode) {
        ic_con |= IC_MASTER_ADDR_MODE_BIT;
        ic_con |= IC_SLAVE_ADDR_MODE_BIT;
    }

    /* Set speed */
    ic_con |= (dev->speed << 1);

    /* Set TX interrupt mode */
    ic_con |= IC_TX_INTR_MODE;

    /* Set the IC_CON register */
    MMIO_REG_VAL_FROM_BASE(dev->BASE, IC_CON) = ic_con;

    /* Wait for register to set */
    while (MMIO_REG_VAL_FROM_BASE(dev->BASE, IC_CON) != ic_con) {
        i++;
        if (i >= STATUS_DELAY) {
            rc = DRV_RC_FAIL;
        } /* Registers wasn't set successfuly - indicate I2C malfunction */
    }

    /* END of setup IC_CON */

    if (I2C_SLOW ==
        dev->speed) /* This is setter so prefering readability above speed */
    {
        /* Set HCNT */
        MMIO_REG_VAL_FROM_BASE(dev->BASE, IC_STD_SCL_HCNT) = I2C_STD_HCNT;
        /* Set LCNT */
        MMIO_REG_VAL_FROM_BASE(dev->BASE, IC_STD_SCL_LCNT) = I2C_STD_LCNT;
    } else if (I2C_FAST == dev->speed) {
        /* Set HCNT */
        MMIO_REG_VAL_FROM_BASE(dev->BASE, IC_FS_SCL_HCNT) = I2C_FS_HCNT;
        /* Set LCNT */
        MMIO_REG_VAL_FROM_BASE(dev->BASE, IC_FS_SCL_LCNT) = I2C_FS_LCNT;
    } else if (I2C_HS == dev->speed) {
        /* Set HCNT */
        MMIO_REG_VAL_FROM_BASE(dev->BASE, IC_HS_SCL_HCNT) = I2C_HS_HCNT;
        /* Set LCNT */
        MMIO_REG_VAL_FROM_BASE(dev->BASE, IC_HS_SCL_LCNT) = I2C_HS_LCNT;
    } else {
        rc = DRV_RC_FAIL;
    }

    /* Set RX fifo threshold level */
    MMIO_REG_VAL_FROM_BASE(dev->BASE, IC_RX_TL) = dev->rx_watermark;
    /* Set TX fifo threshold level */
    MMIO_REG_VAL_FROM_BASE(dev->BASE, IC_TX_TL) = dev->tx_watermark;

    dev->tx_len = dev->total_write_bytes = 0;
    dev->rx_len = dev->total_read_bytes = 0;

    if (dev->mode == I2C_SLAVE) {
        /* Set slave address */
        MMIO_REG_VAL_FROM_BASE(dev->BASE, IC_SAR) = dev->slave_addr;

        /* Disable interrupts */
        MMIO_REG_VAL_FROM_BASE(dev->BASE, IC_INTR_MASK) =
            SOC_DISABLE_ALL_I2C_INT;

        /* Clear interrupts */
        MMIO_REG_VAL_FROM_BASE(dev->BASE, IC_CLR_INTR);

        soc_i2c_enable_device(dev, true);

        /* Enable necesary interrupts */
        MMIO_REG_VAL_FROM_BASE(dev->BASE, IC_INTR_MASK) =
            SOC_ENABLE_INT_I2C_SLAVE;

        soc_i2c_enable_device(dev, true);
    }

    return rc;
}

DRIVER_API_RC soc_i2c_set_config(SOC_I2C_CONTROLLER controller_id,
                                 i2c_cfg_data_t *config)
{
    i2c_internal_data_t *dev;

    /* set current base based on config */
    if (controller_id == SOC_I2C_0) {
        dev = &devices[0];
        dev->ISR = &isr_dev_0;
        dev->BASE = SOC_I2C_0_BASE;
        dev->clk_gate_info.clk_gate_register = PERIPH_CLK_GATE_CTRL;
        dev->clk_gate_info.bits_mask = I2C0_CLK_GATE_MASK;
    } else if (controller_id == SOC_I2C_1) {
        dev = &devices[1];
        dev->ISR = &isr_dev_1;
        dev->BASE = SOC_I2C_1_BASE;
        dev->clk_gate_info.clk_gate_register = PERIPH_CLK_GATE_CTRL;
        dev->clk_gate_info.bits_mask = I2C1_CLK_GATE_MASK;
    } else {
        return DRV_RC_FAIL;
    }

    /* Copy passed in config data locally */
    dev->speed = config->speed;
    dev->addr_mode = config->addressing_mode;
    dev->mode = config->mode_type;
    dev->slave_addr = config->slave_adr;
    dev->rx_cb = config->cb_rx;
    dev->cb_rx_data = config->cb_rx_data;
    dev->tx_cb = config->cb_tx;
    dev->cb_tx_data = config->cb_tx_data;
    dev->err_cb = config->cb_err;
    dev->cb_err_data = config->cb_err_data;

    /* Clear interrupts */
    MMIO_REG_VAL_FROM_BASE(dev->BASE, IC_CLR_INTR);

    if (controller_id == SOC_I2C_0) {
        SET_INTERRUPT_HANDLER(SOC_I2C0_INTERRUPT, dev->ISR);
        /* Unmask interrupts */
        SOC_UNMASK_INTERRUPTS(INT_I2C_0_MASK);
    } else {
        SET_INTERRUPT_HANDLER(SOC_I2C1_INTERRUPT, dev->ISR);
        /* Unmask interrupts */
        SOC_UNMASK_INTERRUPTS(INT_I2C_1_MASK);
    }

    if (I2C_SLAVE == dev->mode) {
        /* Set reset values (moved from reset dev - was called only once) */
        dev->rx_watermark = 0; // TODO test different watermark levels
        dev->tx_watermark = 0;
        dev->fifo_depth = 1;
    } else {
        /* Set reset values (moved from reset dev - was called only once) */
        dev->rx_watermark = 0; // TODO test different watermark levels
        dev->tx_watermark = 0;
        dev->fifo_depth = 16;
    }

    return DRV_RC_OK;
}

DRIVER_API_RC soc_i2c_get_config(SOC_I2C_CONTROLLER controller_id,
                                 i2c_cfg_data_t *config)
{
    /* TODO implement */
    controller_id = controller_id;
    config = config;
    return DRV_RC_FAIL;
}

DRIVER_API_RC soc_i2c_deconfig(SOC_I2C_CONTROLLER controller_id)
{
    i2c_internal_data_t *dev = NULL;
    DRIVER_API_RC rc = DRV_RC_OK;

    /* Controller we are using */
    if (controller_id == SOC_I2C_0) {
        dev = &devices[0];
    } else if (controller_id == SOC_I2C_1) {
        dev = &devices[1];
    } else {
        return DRV_RC_FAIL;
    }

    if (!dev->send_stop) {
        soc_i2c_abort_transfer(dev);
    }

    soc_i2c_enable_device(dev, false);

    /* Disable interrupts */
    MMIO_REG_VAL_FROM_BASE(dev->BASE, IC_INTR_MASK) = SOC_DISABLE_ALL_I2C_INT;

    /* Clear interrupts */
    MMIO_REG_VAL_FROM_BASE(dev->BASE, IC_CLR_INTR);

    return rc;
}

DRIVER_API_RC soc_i2c_clock_enable(SOC_I2C_CONTROLLER controller_id)
{
    i2c_internal_data_t *dev = NULL;

    if (controller_id == SOC_I2C_0) {
        dev = &devices[0];
    } else if (controller_id == SOC_I2C_1) {
        dev = &devices[1];
    } else {
        return DRV_RC_FAIL;
    }

    set_clock_gate(&dev->clk_gate_info, CLK_GATE_ON);

    soc_i2c_init(dev);

    return DRV_RC_OK;
}

DRIVER_API_RC soc_i2c_clock_disable(SOC_I2C_CONTROLLER controller_id)
{
    i2c_internal_data_t *dev = NULL;

    if (controller_id == SOC_I2C_0) {
        dev = &devices[0];
    } else if (controller_id == SOC_I2C_1) {
        dev = &devices[1];
    } else {
        return DRV_RC_FAIL;
    }

    set_clock_gate(&dev->clk_gate_info, CLK_GATE_OFF);

    return DRV_RC_OK;
}

DRIVER_API_RC soc_i2c_set_transfer_speed(SOC_I2C_CONTROLLER controller_id,
                                         uint32_t speed)
{
    volatile uint32_t ic_con = 0;
    i2c_internal_data_t *dev = NULL;

    if (controller_id == SOC_I2C_0) {
        dev = &devices[0];
    } else if (controller_id == SOC_I2C_1) {
        dev = &devices[1];
    } else {
        return DRV_RC_FAIL;
    }

    dev->speed = speed;

    soc_i2c_enable_device(dev, false);

    /* Setup IC_CON */
    ic_con = MMIO_REG_VAL_FROM_BASE(dev->BASE, IC_CON);

    ic_con |= (dev->speed << 1);

    MMIO_REG_VAL_FROM_BASE(dev->BASE, IC_CON) = ic_con;

    if (I2C_SLOW ==
        dev->speed) /* This is setter so prefering readability above speed */
    {
        /* Set HCNT */
        MMIO_REG_VAL_FROM_BASE(dev->BASE, IC_STD_SCL_HCNT) = I2C_STD_HCNT;
        /* Set LCNT */
        MMIO_REG_VAL_FROM_BASE(dev->BASE, IC_STD_SCL_LCNT) = I2C_STD_LCNT;
    } else if (I2C_FAST == dev->speed) {
        /* Set HCNT */
        MMIO_REG_VAL_FROM_BASE(dev->BASE, IC_FS_SCL_HCNT) = I2C_FS_HCNT;
        /* Set LCNT */
        MMIO_REG_VAL_FROM_BASE(dev->BASE, IC_FS_SCL_LCNT) = I2C_FS_LCNT;
    } else if (I2C_HS == dev->speed) {
        /* Set HCNT */
        MMIO_REG_VAL_FROM_BASE(dev->BASE, IC_HS_SCL_HCNT) = I2C_HS_HCNT;
        /* Set LCNT */
        MMIO_REG_VAL_FROM_BASE(dev->BASE, IC_HS_SCL_LCNT) = I2C_HS_LCNT;
    } else {
        return DRV_RC_FAIL;
    }

    return DRV_RC_OK;
}

DRIVER_API_RC soc_i2c_set_transfer_mode(SOC_I2C_CONTROLLER controller_id,
                                        uint32_t mode)
{
    i2c_internal_data_t *dev = NULL;

    if (controller_id == SOC_I2C_0) {
        dev = &devices[0];
    } else if (controller_id == SOC_I2C_1) {
        dev = &devices[1];
    } else {
        return DRV_RC_FAIL;
    }

    dev->addr_mode = mode;

    if (dev->mode == I2C_SLAVE) {
        volatile uint32_t ic_con = 0;

        soc_i2c_enable_device(dev, false);

        /* Setup IC_CON */
        ic_con = MMIO_REG_VAL_FROM_BASE(dev->BASE, IC_CON);

        if (I2C_10_Bit == dev->addr_mode) {
            ic_con |= IC_SLAVE_ADDR_MODE_BIT;
        } else {
            ic_con &= ~IC_SLAVE_ADDR_MODE_BIT;
        }

        MMIO_REG_VAL_FROM_BASE(dev->BASE, IC_CON) = ic_con;

        soc_i2c_enable_device(dev, true);
    }

    return DRV_RC_OK;
}

DRIVER_API_RC soc_i2c_slave_enable_tx(SOC_I2C_CONTROLLER controller_id,
                                      uint8_t *data_write,
                                      uint32_t data_write_len)
{
    i2c_internal_data_t *dev = NULL;

    /* Controller we are using */
    if (controller_id == SOC_I2C_0) {
        dev = &devices[0];
    } else if (controller_id == SOC_I2C_1) {
        dev = &devices[1];
    } else {
        return DRV_RC_FAIL;
    }

    dev->tx_len = data_write_len;
    dev->rx_tx_len = data_write_len;
    dev->i2c_write_buff = data_write;
    dev->total_write_bytes = 0;

    return DRV_RC_OK;
}

DRIVER_API_RC soc_i2c_slave_enable_rx(SOC_I2C_CONTROLLER controller_id,
                                      uint8_t *data_read,
                                      uint32_t data_read_len)
{
    i2c_internal_data_t *dev = NULL;

    /* Controller we are using */
    if (controller_id == SOC_I2C_0) {
        dev = &devices[0];
    } else if (controller_id == SOC_I2C_1) {
        dev = &devices[1];
    } else {
        return DRV_RC_FAIL;
    }

    dev->rx_len = data_read_len;
    dev->rx_tx_len = data_read_len;
    dev->i2c_read_buff = data_read;
    dev->total_read_bytes = 0;

    return DRV_RC_OK;
}

DRIVER_API_RC soc_i2c_master_write(SOC_I2C_CONTROLLER controller_id,
                                   uint8_t *data, uint32_t data_len,
                                   uint32_t slave_addr, bool no_stop)
{
    return soc_i2c_master_transfer(controller_id, data, data_len, 0, 0,
                                   slave_addr, no_stop);
}

DRIVER_API_RC soc_i2c_master_read(SOC_I2C_CONTROLLER controller_id,
                                  uint8_t *data, uint32_t data_len,
                                  uint32_t slave_addr, bool no_stop)
{
    return soc_i2c_master_transfer(controller_id, 0, 0, data, data_len,
                                   slave_addr, no_stop);
}

DRIVER_API_RC soc_i2c_master_transfer(SOC_I2C_CONTROLLER controller_id,
                                      uint8_t *data_write,
                                      uint32_t data_write_len,
                                      uint8_t *data_read,
                                      uint32_t data_read_len,
                                      uint32_t slave_addr, bool no_stop)
{
    bool need_init = true;
    i2c_internal_data_t *dev = NULL;

    /* Controller we are using */
    if (controller_id == SOC_I2C_0) {
        dev = &devices[0];
    } else if (controller_id == SOC_I2C_1) {
        dev = &devices[1];
    } else {
        return DRV_RC_FAIL;
    }

    /* Check for activity */
    if (I2C_OK != soc_i2c_status(controller_id, !dev->send_stop)) {
        return DRV_RC_FAIL;
    }

    if ((data_read_len == 0) && (data_write_len == 0)) {
        // Workaround: we know that we are doing I2C bus scan.
        data_read_len = 1;
        dev->send_restart = true;
    }

    if (data_read_len > 0) {
        dev->state = I2C_CMD_RECV;
    } else {
        dev->state = I2C_CMD_SEND;
    }
    dev->rx_len = data_read_len;
    dev->tx_len = data_write_len;
    dev->rx_tx_len = data_read_len + data_write_len;
    dev->i2c_write_buff = data_write;
    dev->i2c_read_buff = data_read;
    dev->total_read_bytes = 0;
    dev->total_write_bytes = 0;

    if (dev->slave_addr != slave_addr) {
        dev->send_restart = true;
    }

    dev->slave_addr = slave_addr;

    need_init = dev->send_stop || dev->send_restart;

    if (!no_stop) {
        dev->send_stop = true;
    } else {
        dev->send_stop = false;
    }

    if (need_init) {
        soc_i2c_master_init_transfer(dev);
    } else {
        /* Enable necesary interrupts */
        MMIO_REG_VAL_FROM_BASE(dev->BASE, IC_INTR_MASK) =
            SOC_ENABLE_RX_TX_INT_I2C;
    }

    return DRV_RC_OK;
}

DRIVER_I2C_STATUS_CODE soc_i2c_status(SOC_I2C_CONTROLLER controller_id,
                                      bool no_stop)
{
    uint32_t status = 0;
    DRIVER_I2C_STATUS_CODE rc = I2C_OK;
    i2c_internal_data_t *dev = NULL;

    if (controller_id == SOC_I2C_0) {
        dev = &devices[0];
    } else if (controller_id == SOC_I2C_1) {
        dev = &devices[1];
    } else {
        return DRV_RC_FAIL;
    }

    status = MMIO_REG_VAL_FROM_BASE(dev->BASE, IC_STATUS);
    if (((!no_stop) && (status & IC_STATUS_ACTIVITY)) ||
        (status & IC_STATUS_RFNE) || !(status & IC_STATUS_TFE)) {
        rc = I2C_BUSY;
    } else {
        uint32_t int_status = MMIO_REG_VAL_FROM_BASE(dev->BASE, IC_INTR_STAT);
        if (int_status & IC_INTR_TX_ABRT) {
            rc = I2C_TX_ABORT;
        }
        if (int_status & IC_INTR_TX_OVER) {
            rc = I2C_TX_OVER;
        }
        if (int_status & IC_INTR_RX_OVER) {
            rc = I2C_RX_OVER;
        }
        if (int_status & IC_INTR_RX_UNDER) {
            rc = I2C_RX_UNDER;
        }
    }

    return rc;
}
