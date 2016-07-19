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

#include <stddef.h>
#include <stdlib.h>

#include "data_type.h"
#include "ss_i2c_iface.h"
#include "soc_register.h"
#include "io_config.h"
#include "i2c_priv.h"
#include "eiaextensions.h"
#include "ss_dw_i2c.h"
#include "portable.h"

#define I2C_SS_NUM_CONTROLLERS  (2)
#define IO_I2C_MST0_PRESENT
#define IO_I2C_MST1_PRESENT

#define SS_CTRL_ID(x)       (x-I2C_SENSING_0)                   /* Use this to translate from System controller ID to SSS ID */
#define I2C_ADDRESS_MASK   ~( (~(0xffffffff << 10)) << 9 )      /* Use this to zero I2C address in I2CON */

#ifdef __cplusplus
 extern "C" {
#endif

static DRIVER_API_RC is_valid_controller(I2C_CONTROLLER id);
static DRIVER_API_RC is_controller_available(I2C_CONTROLLER id);
static void copy_config_data(i2c_cfg_data_t *cfg, I2C_CONTROLLER controller_id);

/* I2C master devices private data structures */
static i2c_info_pt i2c_handles[I2C_SS_NUM_CONTROLLERS] = { 0 };


#ifdef IO_I2C_MST0_PRESENT
DECLARE_INTERRUPT_HANDLER static void i2c_mst0_err_ISR()
{
    i2c_mst_err_ISR_proc( i2c_handles[0] );
}
DECLARE_INTERRUPT_HANDLER static void i2c_mst0_rx_avail_ISR()
{
    i2c_mst_rx_avail_ISR_proc( i2c_handles[0] );
}
DECLARE_INTERRUPT_HANDLER static void i2c_mst0_tx_req_ISR()
{
    i2c_mst_tx_req_ISR_proc( i2c_handles[0] );
}
DECLARE_INTERRUPT_HANDLER static void i2c_mst0_stop_detected_ISR()
{
    i2c_mst_stop_detected_ISR_proc( i2c_handles[0] );
}
#endif
#ifdef IO_I2C_MST1_PRESENT
DECLARE_INTERRUPT_HANDLER static void i2c_mst1_err_ISR()
{
    i2c_mst_err_ISR_proc( i2c_handles[1] );
}
DECLARE_INTERRUPT_HANDLER static void i2c_mst1_rx_avail_ISR()
{
    i2c_mst_rx_avail_ISR_proc( i2c_handles[1] );
}
DECLARE_INTERRUPT_HANDLER static void i2c_mst1_tx_req_ISR()
{
    i2c_mst_tx_req_ISR_proc( i2c_handles[1] );
}
DECLARE_INTERRUPT_HANDLER static void i2c_mst1_stop_detected_ISR()
{
    i2c_mst_stop_detected_ISR_proc( i2c_handles[1] );
}
#endif



static i2c_info_t   i2c_master_devs[] = {
#ifdef IO_I2C_MST0_PRESENT
            { .instID = 0,
              .reg_base = AR_IO_I2C_MST0_CON,
              .fifo_depth = IO_I2C_MST0_FS,
              .vector_err = IO_I2C_MST0_INT_ERR,
              .isr_err = i2c_mst0_err_ISR,
              .vector_rx_avail = IO_I2C_MST0_INT_RX_AVAIL,
              .isr_rx_avail = i2c_mst0_rx_avail_ISR,
              .vector_tx_req = IO_I2C_MST0_INT_TX_REQ,
              .isr_tx_req = i2c_mst0_tx_req_ISR,
              .vector_stop_detected = IO_I2C_MST0_INT_STOP_DETECTED,
              .isr_stop_detected = i2c_mst0_stop_detected_ISR,
              .i2c_rx_avail_mask = SCSS_REGISTER_BASE + INT_SS_I2C_0_RX_AVAIL_MASK,
              .i2c_tx_req_mask = SCSS_REGISTER_BASE + INT_SS_I2C_0_TX_REQ_MASK,
              .i2c_stop_detected_mask = SCSS_REGISTER_BASE + INT_SS_I2C_0_STOP_DETECTED_MASK,
              .i2c_err_mask = SCSS_REGISTER_BASE + INT_SS_I2C_0_ERR_MASK,
              .creg_i2c_clk_ctrl = CREG_CLK_CTRL_I2C0 },
#endif
#ifdef IO_I2C_MST1_PRESENT
            { .instID = 1,
              .reg_base = AR_IO_I2C_MST1_CON,
              .fifo_depth = IO_I2C_MST1_FS,
              .vector_err = IO_I2C_MST1_INT_ERR,
              .isr_err = i2c_mst1_err_ISR,
              .vector_rx_avail = IO_I2C_MST1_INT_RX_AVAIL,
              .isr_rx_avail = i2c_mst1_rx_avail_ISR,
              .vector_tx_req = IO_I2C_MST1_INT_TX_REQ,
              .isr_tx_req = i2c_mst1_tx_req_ISR,
              .vector_stop_detected = IO_I2C_MST1_INT_STOP_DETECTED,
              .isr_stop_detected = i2c_mst1_stop_detected_ISR,
              .i2c_rx_avail_mask = SCSS_REGISTER_BASE + INT_SS_I2C_1_RX_AVAIL_MASK,
              .i2c_tx_req_mask = SCSS_REGISTER_BASE + INT_SS_I2C_1_TX_REQ_MASK,
              .i2c_stop_detected_mask = SCSS_REGISTER_BASE + INT_SS_I2C_1_STOP_DETECTED_MASK,
              .i2c_err_mask = SCSS_REGISTER_BASE + INT_SS_I2C_1_ERR_MASK,
              .creg_i2c_clk_ctrl = CREG_CLK_CTRL_I2C1 },
#endif
            { .instID = I2C_SS_NUM_CONTROLLERS }
        };


typedef struct ss_i2c_cfg_driver_data
{
    i2c_cfg_data_t public;

    struct ss_i2c_cfg_priv_data{
        uint32_t    hold_time,
                    setup_time,
                    spk_len,
                    ss_hcnt,
                    ss_lcnt,
                    fs_hcnt,
                    fs_lcnt,
                    rx_thresh,
                    tx_thresh;
    }priv;
}ss_i2c_cfg_driver_data_t;

static ss_i2c_cfg_driver_data_t drv_config[2];

DRIVER_API_RC ss_i2c_set_config(I2C_CONTROLLER controller_id, i2c_cfg_data_t *config)
{
    DRIVER_API_RC rc = DRV_RC_OK;
    i2c_info_pt   dev = 0;;
    uint32_t      i2c_con = 0;
    ss_i2c_cfg_driver_data_t *drv_cfg = 0;;

    copy_config_data(config, controller_id);
    drv_cfg = &drv_config[SS_CTRL_ID(controller_id)];

    /* check controller_id is valid and not in use */
    if((rc=is_valid_controller(controller_id)) != DRV_RC_OK)
    {
        return rc;
    }
    if((rc=is_controller_available(controller_id)) != DRV_RC_OK)
    {
        return rc;
    }

    /* do some check here to see if this controller is accessible form this core */

    dev = &i2c_master_devs[SS_CTRL_ID(controller_id)];
    dev->restart = false;
    i2c_handles[SS_CTRL_ID(controller_id)] = dev;

    /* enable clock to controller to allow reg writes */
    REG_WRITE( I2C_CON, I2C_CLK_ENABLED );
    /* disable interrupts while doing config */
    REG_WRITE( I2C_INTR_MASK, I2C_INT_DSB );

    i2c_con = ( I2C_CLK_ENABLED | (drv_cfg->priv.spk_len << 22) | (drv_cfg->public.slave_adr << 9) | (drv_cfg->public.speed << 3));
    i2c_con |= drv_cfg->public.addressing_mode << 5;    // 7 or 10 bit

    i2c_con |= I2C_RESTART_EN;

    REG_WRITE( I2C_CON, i2c_con );

    REG_WRITE( I2C_TL, ( drv_cfg->priv.tx_thresh << 16 ) | drv_cfg->priv.rx_thresh);
    REG_WRITE( I2C_SDA_CONFIG, ( drv_cfg->priv.setup_time << 16 ) | drv_cfg->priv.hold_time );
    REG_WRITE( I2C_SS_SCL_CNT, ( drv_cfg->priv.ss_hcnt << 16 ) | drv_cfg->priv.ss_lcnt );
    REG_WRITE( I2C_FS_SCL_CNT, ( drv_cfg->priv.fs_hcnt << 16 ) | drv_cfg->priv.fs_lcnt );


    /* user callbacks */
    dev->err_cb = drv_cfg->public.cb_err;
    dev->tx_cb  = drv_cfg->public.cb_tx;
    dev->rx_cb  = drv_cfg->public.cb_rx;
    dev->cb_err_data = drv_cfg->public.cb_err_data;
    dev->cb_tx_data  = drv_cfg->public.cb_tx_data;
    dev->cb_rx_data  = drv_cfg->public.cb_rx_data;

    dev->state = I2C_STATE_READY;


    /* disable device */
    i2c_con &= ~(I2C_ENABLE_MASTER);               // 0 disables master
    REG_WRITE( I2C_CON, i2c_con );

    /* set interrupt vector, mid/high priority */
    SET_INTERRUPT_HANDLER( dev->vector_err, dev->isr_err);
    SET_INTERRUPT_HANDLER( dev->vector_rx_avail, dev->isr_rx_avail);
    SET_INTERRUPT_HANDLER( dev->vector_tx_req, dev->isr_tx_req);
    SET_INTERRUPT_HANDLER( dev->vector_stop_detected, dev->isr_stop_detected);

    /*
     * SoC I2C config
     */
    /* Setup I2C Interrupt Routing Mask Registers to allow interrupts through */
    MMIO_REG_VAL(dev->i2c_rx_avail_mask) &= ENABLE_SSS_INTERRUPTS;
    MMIO_REG_VAL(dev->i2c_tx_req_mask) &= ENABLE_SSS_INTERRUPTS;
    MMIO_REG_VAL(dev->i2c_stop_detected_mask) &= ENABLE_SSS_INTERRUPTS;
    MMIO_REG_VAL(dev->i2c_err_mask) &= ENABLE_SSS_INTERRUPTS;

    return DRV_RC_OK;
}

DRIVER_API_RC ss_i2c_deconfig(I2C_CONTROLLER controller_id)
{
    i2c_info_pt dev = &i2c_master_devs[SS_CTRL_ID(controller_id)];
    uint32_t    creg = 0;

    dev = &i2c_master_devs[SS_CTRL_ID(controller_id)];
    /* Set I2C registers to hardware reset state */
    REG_WRITE( I2C_DATA_CMD,0);
    REG_WRITE( I2C_SS_SCL_CNT,0);
    REG_WRITE( I2C_FS_SCL_CNT,0);
    REG_WRITE( I2C_INTR_STAT,0);
    REG_WRITE( I2C_INTR_MASK,0);
    REG_WRITE( I2C_TL,0);
    REG_WRITE( I2C_CLR_INTR,0);
    REG_WRITE( I2C_STATUS,0);
    REG_WRITE( I2C_TXFLR,0);
    REG_WRITE( I2C_RXFLR,0);
    REG_WRITE( I2C_SDA_CONFIG,0);
    REG_WRITE( I2C_TX_ABRT_SOURCE,0);
    REG_WRITE( I2C_ENABLE_STATUS,0);

    creg = _lr(AR_IO_CREG_MST0_CTRL);       // CREG Master
    creg &= ~( 1 << dev->creg_i2c_clk_ctrl);

    /* disable interrupts while doing config */
    REG_WRITE( I2C_INTR_MASK, I2C_INT_DSB );
    /* disable controller */
    REG_WRITE( I2C_CON, 0);

    return DRV_RC_OK;
}

DRIVER_API_RC ss_i2c_clock_enable(I2C_CONTROLLER controller_id)
{
    i2c_info_pt dev = &i2c_master_devs[SS_CTRL_ID(controller_id)];
    uint32_t saved;

    /* Protect registers using lock and unlockl of interruptions */
    saved = interrupt_lock();

    /* enable clock to peripheral */
    WRITE_ARC_REG(READ_ARC_REG((volatile uint32_t)(AR_IO_CREG_MST0_CTRL)) | ( 1 << dev->creg_i2c_clk_ctrl),
                                   (volatile uint32_t)(AR_IO_CREG_MST0_CTRL));

    /* enable device */
    WRITE_ARC_REG(READ_ARC_REG((volatile uint32_t)(dev->reg_base+I2C_CON)) | I2C_ENABLE_MASTER,
                                   (volatile uint32_t)(dev->reg_base+I2C_CON));

    interrupt_unlock(saved);

    return DRV_RC_OK;
}

DRIVER_API_RC ss_i2c_clock_disable(I2C_CONTROLLER controller_id)
{
    i2c_info_pt dev = &i2c_master_devs[SS_CTRL_ID(controller_id)];
    uint32_t saved;

    ///* wait for tx empty and bus idle */
    //while((REG_READ(I2C_STATUS) & I2C_STATUS_MASTER_ACT) || !(REG_READ(I2C_STATUS) & I2C_STATUS_TFE));

    /* Protect registers using lock and unlockl of interruptions */
    saved = interrupt_lock();

    /* disable clock to peripheral */
    WRITE_ARC_REG(READ_ARC_REG((volatile uint32_t)(AR_IO_CREG_MST0_CTRL)) & ~( 1 << dev->creg_i2c_clk_ctrl),
                                   (volatile uint32_t)(AR_IO_CREG_MST0_CTRL));

    /* disable device */
    WRITE_ARC_REG(READ_ARC_REG((volatile uint32_t)(dev->reg_base+I2C_CON)) & ~(I2C_ENABLE_MASTER),
                                   (volatile uint32_t)(dev->reg_base+I2C_CON));

    interrupt_unlock(saved);

    return DRV_RC_OK;

}

DRIVER_API_RC ss_i2c_write(I2C_CONTROLLER controller_id, uint8_t *data_write, uint32_t data_write_len, uint32_t slave_addr, bool no_stop)
{
    return ss_i2c_transfer(controller_id, data_write, data_write_len, 0 , 0, slave_addr, no_stop);
}

DRIVER_API_RC ss_i2c_read(I2C_CONTROLLER controller_id, uint8_t *data_read, uint32_t data_read_len, uint32_t slave_addr, bool no_stop)
{
    return ss_i2c_transfer(controller_id, 0, 0, data_read, data_read_len, slave_addr, no_stop);
}

DRIVER_API_RC ss_i2c_transfer(I2C_CONTROLLER controller_id, uint8_t *data_write, uint32_t data_write_len, uint8_t *data_read, uint32_t data_read_len, uint32_t slave_addr, bool no_stop){
    i2c_info_pt dev = &i2c_master_devs[SS_CTRL_ID(controller_id)];
    uint32_t    i2c_con = 0;
    uint32_t saved;

    /* Ensure we're not in the middle of transmitting a previous buffer
     *
     * Note that the I2C_STATUS_MASTER_ACT may be asserted if we
     * haven't sent a STOP-bit previously.  In that case, we
     * depend on the I2C Master to automatically send a RESTART bit
     * on the bus if changing from transmit to receive or vice-versa.
     */
    if (!(REG_READ(I2C_STATUS) & I2C_STATUS_TFE))
    {
        return DRV_RC_FAIL;
    }

    if (!no_stop)
    {
        dev->send_stop = true;
    }
    else
    {
        dev->send_stop = false;
    }

    if ((data_read_len == 0) && (data_write_len == 0))
    {
        //Workaround: we know that we are doing I2C bus scan.
        data_read_len = 1; 
        dev->restart = true;
    }
    /* Protect registers using lock and unlock of interruptions */
    saved = interrupt_lock();

    i2c_con = REG_READ(I2C_CON);
    i2c_con &= I2C_ADDRESS_MASK;        // zero slave addr first
    i2c_con |= (slave_addr << 9);
    REG_WRITE(I2C_CON, i2c_con);

    if (data_read_len > 0)
    {
        dev->state = I2C_STATE_RECEIVE;
    }
    else
    {
        dev->state = I2C_STATE_TRANSMIT;
    }
    dev->rx_len = data_read_len;
    dev->tx_len = data_write_len;
    dev->rx_tx_len = data_read_len + data_write_len;
    dev->i2c_write_buff = data_write;
    dev->i2c_read_buff = data_read;
    dev->total_read_bytes = 0;
    dev->total_write_bytes = 0;
    i2c_fill_fifo(dev);

    interrupt_unlock(saved);

    REG_WRITE( I2C_INTR_MASK, I2C_INT_ENB | R_TX_EMPTY  );

    return DRV_RC_OK;
}

DRIVER_I2C_STATUS_CODE ss_i2c_status(I2C_CONTROLLER controller_id, bool no_stop)
{
    i2c_info_pt dev = &i2c_master_devs[SS_CTRL_ID(controller_id)];
    DRIVER_I2C_STATUS_CODE rc = I2C_OK;
    uint32_t    int_status = 0;
    uint32_t    status = 0;

    status = REG_READ(I2C_STATUS);
    if (((!no_stop) && (status & I2C_STATUS_MASTER_ACT)) || (status & I2C_STATUS_RFNE) || !(status & I2C_STATUS_TFE))
    {
        rc = I2C_BUSY;
    }
    else
    {
        int_status = REG_READ(I2C_INTR_STAT);
        if(int_status & R_TX_ABRT)
        {
            rc = I2C_TX_ABORT;
        }
        if(int_status & R_TX_OVER)
        {
            rc =  I2C_TX_OVER;
        }
        if(int_status & R_RX_OVER)
        {
            rc =  I2C_RX_OVER;
        }
        if(int_status & R_RX_UNDER)
        {
            rc =  I2C_RX_UNDER;
        }
    }

    return rc;
}

/*
 * Check to see if this controller is part of the design ( determined at build time )
 */
static DRIVER_API_RC is_valid_controller(I2C_CONTROLLER controller_id)
{

    // do checks here
    return DRV_RC_OK;
}
/*
 * Check to see if controller is not in use
 */
static DRIVER_API_RC is_controller_available(I2C_CONTROLLER controller_id)
{
//    i2c_info_pt dev = &i2c_master_devs[SS_CTRL_ID(controller_id)];

//    if(REG_READ(I2C_CON) & I2C_ENABLE_MASTER)
//        return DRV_RC_CONTROLLER_IN_USE;

    // do checks here
    return DRV_RC_OK;
}

static void copy_config_data(i2c_cfg_data_t *cfg, I2C_CONTROLLER controller_id)
{
    uint32_t id = SS_CTRL_ID(controller_id);

    /* copy passed in config data locally */
    drv_config[id].public.speed = cfg->speed;
    drv_config[id].public.addressing_mode = cfg->addressing_mode;
    drv_config[id].public.mode_type = cfg->mode_type;
    drv_config[id].public.slave_adr = cfg->slave_adr;
    drv_config[id].public.cb_rx = cfg->cb_rx;
    drv_config[id].public.cb_rx_data = cfg->cb_rx_data;
    drv_config[id].public.cb_tx = cfg->cb_tx;
    drv_config[id].public.cb_tx_data = cfg->cb_tx_data;
    drv_config[id].public.cb_err = cfg->cb_err;
    drv_config[id].public.cb_err_data = cfg->cb_err_data;

    /* TODO default private data - may need to look at this.*/
    drv_config[id].priv.hold_time = I2C_HOLD_TIME;
    drv_config[id].priv.setup_time = I2C_SETUP_TIME;
    drv_config[id].priv.spk_len = I2C_SPKLEN;
    drv_config[id].priv.ss_hcnt = I2C_SS_SCL_HIGH_COUNT;
    drv_config[id].priv.ss_lcnt = I2C_SS_SCL_LOW_COUNT;
    drv_config[id].priv.fs_hcnt = I2C_FS_SCL_HIGH_COUNT;
    drv_config[id].priv.fs_lcnt = I2C_FS_SCL_LOW_COUNT;
    drv_config[id].priv.rx_thresh = I2C_RX_FIFO_THRESHOLD;
    drv_config[id].priv.tx_thresh = I2C_TX_FIFO_THRESHOLD;

}

#ifdef __cplusplus
}
#endif
