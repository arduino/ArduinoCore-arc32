/** INTEL CONFIDENTIAL Copyright 2015 Intel Corporation All Rights Reserved.
  *
  * The source code contained or described herein and all documents related to
  * the source code ("Material") are owned by Intel Corporation or its suppliers
  * or licensors.
  * Title to the Material remains with Intel Corporation or its suppliers and
  * licensors.
  * The Material contains trade secrets and proprietary and confidential information
  * of Intel or its suppliers and licensors. The Material is protected by worldwide
  * copyright and trade secret laws and treaty provisions.
  * No part of the Material may be used, copied, reproduced, modified, published,
  * uploaded, posted, transmitted, distributed, or disclosed in any way without
  * Intel’s prior express written permission.
  *
  * No license under any patent, copyright, trade secret or other intellectual
  * property right is granted to or conferred upon you by disclosure or delivery
  * of the Materials, either expressly, by implication, inducement, estoppel or
  * otherwise.
  *
  * Any license under such intellectual property rights must be express and
  * approved by Intel in writing
  *
  ******************************************************************************/
/*
 * Intel SOC SPI driver
 *
 */

#include "soc_register.h"
#include "soc_spi_priv.h"
#include "intel_qrk_spi.h"
#include "portable.h"
#include "string.h"

/* Software workaround:
 * --------------------
 * We are currently encountering problems with the SPI0_Master chip select.
 * Although it should be drived automatically by the spi module itself during a transaction,
 * it is not the case.
 * This software workaround allows driving directly the cs pin using gpios.
 *  */
#define ATP_HW_V1

#if 0
#define DEBUG_SPI_DRIVER
extern int printk(const char *, ...);
#define DBG(args...) printk(args);
#else
#define DBG(args...)
#endif

#ifdef __cplusplus
 extern "C" {
#endif

static DRIVER_API_RC is_valid_controller(SOC_SPI_CONTROLLER id);
static void copy_config_data(spi_cfg_data_t *cfg, SOC_SPI_CONTROLLER controller_id);
/* IT subrountines */

static void spi_fill_fifo(soc_spi_info_pt dev);

struct soc_spi_cfg_driver_data
{
    spi_cfg_data_t cfg;
    uint16_t        tx_threshold;
    uint16_t        rx_threshold;
    SHIFT_REG_LOOP  srl;
};

#define SPI_MAX_CNT     (3)

static struct soc_spi_cfg_driver_data drv_config[SPI_MAX_CNT];

/* SPI master devices private data structures */
static soc_spi_info_pt spi_handles[SPI_MAX_CNT] = { 0 };

/* Interrupt Service Routines */
void soc_spi_mst0_ISR()
{
    soc_spi_ISR_proc(spi_handles[SOC_SPI_MASTER_0]);
}
void soc_spi_mst1_ISR()
{
    soc_spi_ISR_proc(spi_handles[SOC_SPI_MASTER_1]);
}
void soc_spi_slv_ISR()
{
    soc_spi_ISR_proc(spi_handles[SOC_SPI_SLAVE_0]);
}

static soc_spi_info_t soc_spi_devs[] = {
            { .instID = SOC_SPI_MASTER_0,
            .reg_base = SOC_MST_SPI0_REGISTER_BASE,
            .isr_vector = SOC_SPIM0_INTERRUPT,
            .isr = soc_spi_mst0_ISR,
            .fifo_depth = IO_SPI_MST0_FS,
            .spi_int_mask = INT_SPI_MST_0_MASK
            },
            { .instID = SOC_SPI_MASTER_1,
            .reg_base = SOC_MST_SPI1_REGISTER_BASE,
            .isr_vector = SOC_SPIM1_INTERRUPT,
            .isr = soc_spi_mst1_ISR,
            .fifo_depth = IO_SPI_MST1_FS,
            .spi_int_mask = INT_SPI_MST_1_MASK
            },
            { .instID = SOC_SPI_SLAVE_0,
            .reg_base = SOC_SLV_SPI_REGISTER_BASE,
            .isr_vector = SOC_SPIS0_INTERRUPT,
            .isr = soc_spi_slv_ISR,
            .fifo_depth = IO_SPI_SLV_FS,
            .spi_int_mask = INT_SPI_SLV_MASK
            },
            { .instID = SPI_MAX_CNT
            }
        };

#ifdef ATP_HW_V1
static void cs_config(uint8_t gpio)
{
    MMIO_REG_VAL_FROM_BASE(SOC_GPIO_BASE_ADDR, SOC_GPIO_SWPORTA_DDR) |= (1<<gpio);
}

static void cs_low(uint8_t gpio)
{
    MMIO_REG_VAL_FROM_BASE(SOC_GPIO_BASE_ADDR, SOC_GPIO_SWPORTA_DR) &= ~(1<<gpio);
}

static void cs_high(uint8_t gpio)
{
    MMIO_REG_VAL_FROM_BASE(SOC_GPIO_BASE_ADDR, SOC_GPIO_SWPORTA_DR) |= (1<<gpio);
}
#endif

DRIVER_API_RC soc_spi_set_config(SOC_SPI_CONTROLLER controller_id, spi_cfg_data_t *config)
{
    DRIVER_API_RC rc = DRV_RC_OK;
    soc_spi_info_pt dev = 0;
    struct soc_spi_cfg_driver_data *drv_cfg = 0;
    uint32_t reg = 0;

    copy_config_data(config, controller_id);
    drv_cfg = &drv_config[controller_id];

    /* check controller_id is valid */
    if((rc = is_valid_controller(controller_id)) != DRV_RC_OK)
    {
        return rc;
    }

    if(soc_spi_status(controller_id) == SPI_BUSY)
    {
        return DRV_RC_CONTROLLER_IN_USE;
    }

    dev = &soc_spi_devs[controller_id];
    spi_handles[controller_id] = dev;

    /* disable controller */
    MMIO_REG_VAL_FROM_BASE(dev->reg_base, SPIEN) &= SPI_DISABLE;

    /* Set frame size, bus mode and transfer mode */
    reg = (drv_cfg->cfg.data_frame_size << 16) | (drv_cfg->cfg.loopback_enable << 11) |
          (drv_cfg->cfg.bus_mode << 6) | (drv_cfg->cfg.txfr_mode << 8 );

    MMIO_REG_VAL_FROM_BASE(dev->reg_base, CTRL0) = reg;
    // TODO data_frame_size is not written to CTRL0 - default is 8 bit

    MMIO_REG_VAL_FROM_BASE(dev->reg_base, TXFTL) = drv_cfg->tx_threshold;
    MMIO_REG_VAL_FROM_BASE(dev->reg_base, RXFTL) = drv_cfg->rx_threshold;

    /* If the device is configured as a slave, an external master will set the baud rate */
    if(SOC_SPI_SLAVE_0 != controller_id)
    {
        /* Set SPI Clock Divider */
        reg = (FREQ_SPI_CLOCK_IN / (drv_cfg->cfg.speed*BAUD_DIVISOR));
        MMIO_REG_VAL_FROM_BASE(dev->reg_base, BAUDR) = reg;
    }
    else
    {
        /* Full duplex setup order for slave is initially unassigned */
        dev->fd_order = UNASSIGNED;
    }

    dev->dfs = drv_cfg->cfg.data_frame_size;
    dev->rx_len = 0;
    dev->tx_len = 0;
    dev->mode = drv_cfg->cfg.txfr_mode;
    dev->state = SPI_STATE_IDLE;

    /* User callbacks */
    dev->err_cb = drv_cfg->cfg.cb_err;
    dev->xfer_cb  = drv_cfg->cfg.cb_xfer;
    dev->slave_rx_cb = drv_cfg->cfg.cb_slave_rx;
    dev->cb_xfer_data = drv_cfg->cfg.cb_xfer_data;
    dev->cb_err_data = drv_cfg->cfg.cb_err_data;

    /* Register ISR */
    SET_INTERRUPT_HANDLER( dev->isr_vector, dev->isr);

    /* Setup SPI Interrupt Routing Mask Registers to allow interrupts through */
    SOC_UNMASK_INTERRUPTS(dev->spi_int_mask);

    /* Disable interrupts */
    MMIO_REG_VAL_FROM_BASE(dev->reg_base, IMR) = SPI_DISABLE_INT;
    /* Enable device */
    MMIO_REG_VAL_FROM_BASE(dev->reg_base, SPIEN) |= SPI_ENABLE;

    return DRV_RC_OK;
}

DRIVER_API_RC soc_spi_get_config(SOC_SPI_CONTROLLER controller_id, spi_cfg_data_t *config)
{
    // TODO - Check for no config structure
    config = &drv_config[controller_id].cfg;
    return DRV_RC_OK;
}

DRIVER_API_RC soc_spi_deconfig(SOC_SPI_CONTROLLER controller_id)
{
    soc_spi_info_pt dev     = 0;

    dev = &soc_spi_devs[controller_id];

    /* disable controller */
    MMIO_REG_VAL_FROM_BASE(dev->reg_base, SPIEN) &= SPI_DISABLE;

    /* Set SPI registers to hardware reset state */
    MMIO_REG_VAL_FROM_BASE(dev->reg_base, BAUDR)  = 0x0;
    MMIO_REG_VAL_FROM_BASE(dev->reg_base, TXFTL)  = 0x0;
    MMIO_REG_VAL_FROM_BASE(dev->reg_base, RXFTL)  = 0x0;
    MMIO_REG_VAL_FROM_BASE(dev->reg_base, TXFL)   = 0x0;
    MMIO_REG_VAL_FROM_BASE(dev->reg_base, RXFL)   = 0x0;
    MMIO_REG_VAL_FROM_BASE(dev->reg_base, SR)     = 0x0;
    MMIO_REG_VAL_FROM_BASE(dev->reg_base, ISR)    = 0x0;
    MMIO_REG_VAL_FROM_BASE(dev->reg_base, IMR)    = 0x0;
    MMIO_REG_VAL_FROM_BASE(dev->reg_base, RISR)   = 0x0;
    MMIO_REG_VAL_FROM_BASE(dev->reg_base, TXFOIC) = 0x0;
    MMIO_REG_VAL_FROM_BASE(dev->reg_base, RXFOIC) = 0x0;
    MMIO_REG_VAL_FROM_BASE(dev->reg_base, RXFUIC) = 0x0;
    MMIO_REG_VAL_FROM_BASE(dev->reg_base, MMIC)   = 0x0;
    MMIO_REG_VAL_FROM_BASE(dev->reg_base, ICR)    = 0x0;
    MMIO_REG_VAL_FROM_BASE(dev->reg_base, IDR)    = 0x0;

    /* Disable clock to peripheral */
    soc_spi_clock_disable(controller_id);

    return DRV_RC_OK;
}

DRIVER_API_RC soc_spi_clock_enable(SOC_SPI_CONTROLLER controller_id)
{
    /* Enable clock to peripheral */
    if(SOC_SPI_MASTER_0 == controller_id)
    {
        MMIO_REG_VAL(PERIPH_CLK_GATE_CTRL) |= ENABLE_SPI_MASTER_0;
    }
    else if (SOC_SPI_MASTER_1 == controller_id)
    {
        MMIO_REG_VAL(PERIPH_CLK_GATE_CTRL) |= ENABLE_SPI_MASTER_1;
    }
    else if (SOC_SPI_SLAVE_0 == controller_id)
    {
        MMIO_REG_VAL(PERIPH_CLK_GATE_CTRL) |= ENABLE_SPI_SLAVE;
    }
    else
    {
        return DRV_RC_INVALID_OPERATION;
    }
    return DRV_RC_OK;
}

DRIVER_API_RC soc_spi_clock_disable(SOC_SPI_CONTROLLER controller_id)
{
    /* Disable clock to peripheral */
    if(SOC_SPI_MASTER_0 == controller_id)
    {
        MMIO_REG_VAL(PERIPH_CLK_GATE_CTRL) &= DISABLE_SPI_MASTER_0;
    }
    else if (SOC_SPI_MASTER_1 == controller_id)
    {
        MMIO_REG_VAL(PERIPH_CLK_GATE_CTRL) &= DISABLE_SPI_MASTER_1;
    }
    else if (SOC_SPI_SLAVE_0 == controller_id)
    {
        MMIO_REG_VAL(PERIPH_CLK_GATE_CTRL) &= DISABLE_SPI_SLAVE;
    }
    else
    {
        return DRV_RC_INVALID_OPERATION;
    }

    return DRV_RC_OK;
}

#ifdef ATP_HW_V1
DRIVER_API_RC soc_spi_cs_hook(soc_spi_info_pt dev, int slave)
{
    /* If SPI_M0, use GPIO mode */
    if(dev->instID == SOC_SPI_MASTER_0)
    {
        switch(slave)
        {
            case SPI_SE_1:
            {
                dev->cs_gpio = SPIM0_CS_1_GPIO24;
                break;
            }
            case SPI_SE_2:
            {
                dev->cs_gpio = SPIM0_CS_2_GPIO25;
                break;
            }
            case SPI_SE_3:
            {
                dev->cs_gpio = SPIM0_CS_3_GPIO26;
                break;
            }
            case SPI_SE_4:
            {
                dev->cs_gpio = SPIM0_CS_4_GPIO27;
                break;
            }
            default:
            {
                return DRV_RC_INVALID_OPERATION;
            }
        }
        cs_config(dev->cs_gpio);
        cs_low(dev->cs_gpio);
    }
    return DRV_RC_OK;
}
#endif

DRIVER_API_RC soc_spi_transfer(SOC_SPI_CONTROLLER controller_id, uint8_t *tx_data, uint32_t tx_data_len, uint8_t *rx_data, uint32_t rx_data_len, int full_duplex, SPI_SLAVE_ENABLE slave)
{
    DRIVER_API_RC ret;
    soc_spi_info_pt dev    = &soc_spi_devs[controller_id];

    /* Check if device is in use */
    if (soc_spi_status(controller_id) == SPI_BUSY)
    {
        return DRV_RC_CONTROLLER_IN_USE;
    }

    if (full_duplex && (rx_data_len < tx_data_len)) {
        return DRV_RC_INVALID_OPERATION;
    }

    /* Setup communication parameters */
    dev->tx_len = tx_data_len;
    dev->tx_buf = tx_data;
    dev->rx_count = dev->tx_count = 0;
    dev->rx_len = rx_data_len;
    dev->rx_buf = rx_data;
    dev->discarded = 0;
    dev->dummy_tx = 0;

    dev->full_duplex = full_duplex;

    /* Disable device */
    MMIO_REG_VAL_FROM_BASE(dev->reg_base, SPIEN) &= SPI_DISABLE;

    //TODO: This might be used when we use the hw driven CS / not GPIO.
    //MMIO_REG_VAL_FROM_BASE(dev->reg_base, CTRL1) = dev->tx_len + dev->rx_len - 1;

    /* Enable slave device */
    MMIO_REG_VAL_FROM_BASE(dev->reg_base, SER) = slave;

    MMIO_REG_VAL_FROM_BASE(dev->reg_base, IMR) = SPI_DISABLE_INT;
    MMIO_REG_VAL_FROM_BASE(dev->reg_base, ICR);

    /* Enable device */
    MMIO_REG_VAL_FROM_BASE(dev->reg_base, SPIEN) |= SPI_ENABLE;

#ifdef ATP_HW_V1
    ret = soc_spi_cs_hook(dev, slave);
    if (ret != DRV_RC_OK)
    {
        return ret;
    }
#endif

    spi_fill_fifo(dev);
    MMIO_REG_VAL_FROM_BASE(dev->reg_base, IMR) = SPI_ENABLE_INT;
    return DRV_RC_OK;
}

DRIVER_SPI_STATUS_CODE soc_spi_status(SOC_SPI_CONTROLLER controller_id)
{
    DRIVER_SPI_STATUS_CODE rc = SPI_OK;
    soc_spi_info_pt dev     = &soc_spi_devs[controller_id];
    uint32_t    status  = 0;

    status = MMIO_REG_VAL_FROM_BASE(dev->reg_base, SR);
    if ((status & SPI_STATUS_BUSY)){
        rc = SPI_BUSY;
    }
    else if ((status & SPI_STATUS_TFE)){
        rc = SPI_TFE;
    }
    else if ((status & SPI_STATUS_RFNE)){
        rc = SPI_RFNE;
    }
    return rc;
}

/*
 * Check to see if this controller is part of the design ( determined at build time )
 */
static DRIVER_API_RC is_valid_controller(SOC_SPI_CONTROLLER id)
{
    // TODO
    // do checks here
    return DRV_RC_OK;
}

static void copy_config_data(spi_cfg_data_t *cfg, SOC_SPI_CONTROLLER controller_id)
{
    /* copy passed in config data locally */
    memcpy(&drv_config[controller_id].cfg, cfg, sizeof(drv_config[controller_id].cfg));

    drv_config[controller_id].tx_threshold          = SPI_TX_FIFO_THRESHOLD;
    drv_config[controller_id].rx_threshold          = SPI_RX_FIFO_THRESHOLD;
    drv_config[controller_id].srl                   = NORMAL_MODE;

}

void transfer_complete(soc_spi_info_pt dev)
{
    uint32_t tmp;

    do
    {
        tmp = MMIO_REG_VAL_FROM_BASE(dev->reg_base, SR);
    } while((tmp&SPI_STATUS_BUSY));

    dev->state = SPI_STATE_IDLE;
    MMIO_REG_VAL_FROM_BASE(dev->reg_base, IMR) = SPI_DISABLE_INT;
    MMIO_REG_VAL_FROM_BASE(dev->reg_base, ICR);

#ifdef ATP_HW_V1
    if(dev->instID == SOC_SPI_MASTER_0)
    {
        cs_high(dev->cs_gpio);
    }
#endif
    if (dev->xfer_cb != NULL)
    {
        dev->xfer_cb(dev->cb_xfer_data);
    }
}

void spi_fill_fifo(soc_spi_info_pt dev)
{
    uint32_t status = MMIO_REG_VAL_FROM_BASE(dev->reg_base, SR);
    DBG("%s: status: %x tc: %d rc: %d tlen: %d rlen: %d\n", __func__, status,
            dev->tx_count, dev->rx_count, dev->tx_len, dev->rx_len);
    while( (status & SPI_STATUS_TFNF) &&
           ((MMIO_REG_VAL_FROM_BASE(dev->reg_base, RXFL) +
                MMIO_REG_VAL_FROM_BASE(dev->reg_base, TXFL)) < dev->fifo_depth )) {
        if (dev->tx_count < dev->tx_len)
        {
            DBG("push data\n");
            MMIO_REG_VAL_FROM_BASE(dev->reg_base, DR) = dev->tx_buf[dev->tx_count++];
            if (dev->full_duplex)
            {
                dev->dummy_tx++;
            }
        }
        else if (dev->dummy_tx < dev->rx_len)
        {
            DBG("push dummy\n");
            MMIO_REG_VAL_FROM_BASE(dev->reg_base, DR) = 0;
            dev->dummy_tx ++;
        }
        else
        {
            /* xfer complete, wait for last tx_empty and wait for end of reception. */
            break;
        }
        status = MMIO_REG_VAL_FROM_BASE(dev->reg_base, SR);
    }
}


void soc_spi_ISR_proc(soc_spi_info_pt dev)
{
    uint32_t status = MMIO_REG_VAL_FROM_BASE(dev->reg_base, ISR);
    uint32_t clear = MMIO_REG_VAL_FROM_BASE(dev->reg_base, ICR);
#ifndef DEBUG_SPI_DRIVER
    (void)clear; /* Unused variable */
#endif
    DBG("I %x M:%x", status, MMIO_REG_VAL_FROM_BASE(dev->reg_base, IMR));
    if (status == 0) {
        return;
    }
    DBG("Interrupt : cl %x rxl: %d txl: %d\n", clear,
            MMIO_REG_VAL_FROM_BASE(dev->reg_base, RXFL),
            MMIO_REG_VAL_FROM_BASE(dev->reg_base, TXFL));
    DBG("%s: status: %x tc: %d rc: %d tlen: %d rlen: %d\n", __func__, status,
            dev->tx_count, dev->rx_count, dev->tx_len, dev->rx_len);

    /* Receive data */
    while(MMIO_REG_VAL_FROM_BASE(dev->reg_base, RXFL) > 0)
    {
        if (!dev->full_duplex && (dev->discarded < dev->tx_len))
        {
            uint32_t tmp;
            tmp = MMIO_REG_VAL_FROM_BASE(dev->reg_base, DR);
            DBG("discard %x\n", tmp);
            dev->discarded++;
        } else {
            if( dev->rx_count < dev->rx_len)
            {
                dev->rx_buf[dev->rx_count++] = MMIO_REG_VAL_FROM_BASE(dev->reg_base, DR);
                DBG("pop %x\n", dev->rx_buf[dev->rx_count -1 ]);
            }
        }
        if ((dev->rx_len > 0) && (dev->rx_count == dev->rx_len))
        {
            transfer_complete(dev);
            return;
        }
    }

    if (MMIO_REG_VAL_FROM_BASE(dev->reg_base, SR) & SPI_STATUS_TFE)
    {
        if (dev->tx_len > 0 && (dev->tx_count == dev->tx_len) && dev->rx_len == 0)
        {
            transfer_complete(dev);
            return;
        }
    }

    if ((dev->rx_count < dev->rx_len) || (dev->tx_count < dev->tx_len))
    {
        spi_fill_fifo(dev);
    }

    DBG("%s: status: %x tc: %d rc: %d tl: %d rl: %d\n", __func__, status,
            dev->tx_count, dev->rx_count, dev->tx_len, dev->rx_len);
    MMIO_REG_VAL_FROM_BASE(dev->reg_base, IMR) = 0;
    MMIO_REG_VAL_FROM_BASE(dev->reg_base, IMR) = SPI_ENABLE_INT;
}

#ifdef __cplusplus
}
#endif
