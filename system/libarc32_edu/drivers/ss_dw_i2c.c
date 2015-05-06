/*
*
* CONFIDENTIAL AND PROPRIETARY INFORMATION
*
* Copyright (c) 2013 Synopsys, Inc. All rights reserved.
* This software and documentation contain confidential and
* proprietary information that is the property of
* Synopsys, Inc. The software and documentation are
* furnished under a license agreement and may be used
* or copied only in accordance with the terms of the license
* agreement. No part of the software and documentation
* may be reproduced, transmitted, or translated, in any
* form or by any means, electronic, mechanical, manual,
* optical, or otherwise, without prior written permission
* of Synopsys, Inc., or as expressly provided by the license agreement.
* Reverse engineering is prohibited, and reproduction,
* disclosure or use without specific written authorization
* of Synopsys Inc. is strictly forbidden.
*/


#include "data_type.h"
#include "i2c_priv.h"
#include "ss_dw_i2c.h"
#include "soc_register.h"

static void end_data_transfer (i2c_info_pt dev)
{
    uint32_t state;
    REG_WRITE( I2C_INTR_MASK, I2C_INT_DSB );
    state = dev->state;
    dev->state = I2C_STATE_READY;
    if ( I2C_STATE_RECEIVE == state )
    {
        if( _Usually( NULL != dev->rx_cb ) )
        {
            dev->rx_cb( dev->cb_rx_data );
        }
    }
    else if ( I2C_STATE_TRANSMIT == state )
    {
        if( _Usually( NULL != dev->tx_cb ) )
        {
            dev->tx_cb( dev->cb_tx_data );
        }
    }
}


static void recv_data(i2c_info_pt dev)
{
    uint32_t i, rx_cnt = 0;


    if (_Rarely(!dev->rx_len))
    {
        return;
    }

    rx_cnt = REG_READ( I2C_RXFLR );

    if (rx_cnt > dev->rx_len)
    {
        rx_cnt = dev->rx_len;
    }
    for (i = 0; i < rx_cnt; i++)
    {
        REG_WRITE( I2C_DATA_CMD, I2C_POP_DATA );
        _nop();
        dev->i2c_read_buff[i] = REG_READ( I2C_DATA_CMD );
    }
    dev->i2c_read_buff += i;
    dev->rx_len -= i;
    dev->total_read_bytes += i;
}


void i2c_fill_fifo(i2c_info_pt dev)
{
    uint32_t      i, tx_cnt, data;
    if (_Rarely(!dev->rx_tx_len))
    {
        return;
    }

    tx_cnt = dev->fifo_depth - REG_READ( I2C_TXFLR );
    if (tx_cnt > dev->rx_tx_len)
    {
        tx_cnt = dev->rx_tx_len;
    }

    for (i = 0; i < tx_cnt; i++)
    {
        if (dev->tx_len > 0){ // something to transmit
            data = I2C_PUSH_DATA | dev->i2c_write_buff[i];

            if( dev->tx_len == 1)
            {  // last byte to write
                if (dev->rx_len > 0) // repeated start  if something to read after
                    data |= I2C_RESTART_CMD;
                else
                    data |= I2C_STOP_CMD;
            }
            dev->tx_len -= 1;
            dev->total_write_bytes += 1;
        }
        else
        {    // something to read
            data = I2C_PUSH_DATA | I2C_READ_CMD;
            if (dev->rx_tx_len == 1) // last dummy byte to write
                data |= I2C_STOP_CMD;
        }
        REG_WRITE( I2C_DATA_CMD, data );
        dev->rx_tx_len -= 1;
    }
    dev->i2c_write_buff += i;
}

static void xmit_data(i2c_info_pt dev)
{
    int mask;
    i2c_fill_fifo(dev);
    if (dev->rx_tx_len <= 0)
    {
        mask = REG_READ( I2C_INTR_MASK );
        mask &= ~(R_TX_EMPTY);
        mask |= R_STOP_DETECTED;
        REG_WRITE(I2C_INTR_MASK, mask);
    }
}


void i2c_mst_err_ISR_proc(i2c_info_pt dev)
{
    uint32_t      status = REG_READ( I2C_INTR_STAT );

    if (_Rarely(!status))
    {
        return;
    }
    dev->status_code = 0;
    if (status & R_STOP_DETECTED) {
        REG_WRITE(I2C_CLR_INTR, R_STOP_DETECTED );
        //REG_WRITE(I2C_INTR_MASK, REG_READ(I2C_INTR_MASK) & ~R_STOP_DETECTED);
        end_data_transfer(dev);
    }
    if( status & R_TX_ABRT  )
    {
        dev->status_code = status;
        REG_WRITE( I2C_CLR_INTR, R_TX_ABRT );
        REG_WRITE( I2C_INTR_MASK, I2C_INT_DSB );
        dev->state = I2C_STATE_READY;
           if (_Usually(NULL != dev->err_cb))
           {
               dev->err_cb(dev->cb_err_data);
           }
    }
    if( (status & R_TX_OVER)||(status & R_RX_OVER) )
    {
        dev->status_code = status;
        REG_WRITE( I2C_CLR_INTR, R_TX_OVER|R_RX_OVER );
        REG_WRITE( I2C_INTR_MASK, I2C_INT_DSB );
        dev->state = I2C_STATE_READY;
        if (_Usually(NULL != dev->err_cb))
        {
            dev->err_cb(dev->cb_err_data);
        }
    }
}


void i2c_mst_rx_avail_ISR_proc(i2c_info_pt dev )
{

#ifndef NDEBUG
    uint32_t      status = REG_READ( I2C_INTR_STAT );

    if (_Rarely(!status))
    {
          return;
    }

    if (status & R_RX_FULL)
    {
#endif
        recv_data( dev );
        REG_WRITE( I2C_CLR_INTR, R_RX_FULL );
#ifndef NDEBUG
    }
#endif
}


void i2c_mst_tx_req_ISR_proc(i2c_info_pt dev)
{

#ifndef NDEBUG
    uint32_t      status = REG_READ( I2C_INTR_STAT );

    if (_Rarely(!status))
    {
          return;
    }

    if (status & R_TX_EMPTY)
    {
#endif
        xmit_data(dev);
        REG_WRITE( I2C_CLR_INTR, R_TX_EMPTY);
#ifndef NDEBUG
    }
#endif
}

/* Stop detected ISR - end up here if a STOP condition is asserted on the SDA and SCK signals */
void i2c_mst_stop_detected_ISR_proc(i2c_info_pt dev)
{
    uint32_t      status = REG_READ( I2C_INTR_STAT );

    if(status & R_STOP_DETECTED)
    {
        recv_data( dev );
        end_data_transfer( dev );
    }
    REG_WRITE( I2C_CLR_INTR, R_STOP_DETECTED );
}

