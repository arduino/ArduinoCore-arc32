/*
 * i2c.c - i2c library layer
 *
 * Copyright (C) 2015 Intel Corporation
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
 *
 */

#include "i2c.h"
#include "variant.h"

#define TIMEOUT_MS 16

static volatile uint8_t i2c_tx_complete[NUM_SS_I2C];
static volatile uint8_t i2c_rx_complete[NUM_SS_I2C];
static volatile uint8_t i2c_err_detect[NUM_SS_I2C];
static volatile uint32_t i2c_err_source[NUM_SS_I2C];

static volatile uint8_t i2c_slave[NUM_SS_I2C];

static void ss_i2c_0_rx(uint32_t dev_id)
{
    i2c_rx_complete[I2C_SENSING_0] = 1;
}

static void ss_i2c_1_rx(uint32_t dev_id)
{
    i2c_rx_complete[I2C_SENSING_1] = 1;
}

static void ss_i2c_0_tx(uint32_t dev_id)
{
    i2c_tx_complete[I2C_SENSING_0] = 1;
}

static void ss_i2c_1_tx(uint32_t dev_id)
{
    i2c_tx_complete[I2C_SENSING_1] = 1;
}

static void ss_i2c_0_err(uint32_t dev_id)
{
    i2c_err_detect[I2C_SENSING_0] = 1;
    i2c_err_source[I2C_SENSING_0] = dev_id;
}

static void ss_i2c_1_err(uint32_t dev_id)
{
    i2c_err_detect[I2C_SENSING_1] = 1;
    i2c_err_source[I2C_SENSING_1] = dev_id;
}

static int wait_rx_or_err(I2C_CONTROLLER controller_id)
{
    uint64_t timeout = TIMEOUT_MS * 200;

    while (timeout--) {
        if (i2c_err_detect[controller_id]) {
            if (i2c_err_source[controller_id] & I2C_ABRT_7B_ADDR_NOACK) {
                return I2C_ERROR_ADDRESS_NOACK; // NACK on transmit of address
            } else if (i2c_err_source[controller_id] & I2C_ABRT_TXDATA_NOACK) {
                return I2C_ERROR_DATA_NOACK; // NACK on transmit of data
            } else {
                return I2C_ERROR_OTHER; // other error
            }
        }
        if (i2c_rx_complete[controller_id]) {
            return I2C_OK;
        }
        delayMicroseconds(10);
    }

    return I2C_TIMEOUT;
}

static int wait_tx_or_err(I2C_CONTROLLER controller_id)
{
    uint64_t timeout = TIMEOUT_MS * 200;

    while (timeout--) {
        if (i2c_err_detect[controller_id]) {
            if (i2c_err_source[controller_id] & I2C_ABRT_7B_ADDR_NOACK) {
                return I2C_ERROR_ADDRESS_NOACK; // NACK on transmit of address
            } else if (i2c_err_source[controller_id] & I2C_ABRT_TXDATA_NOACK) {
                return I2C_ERROR_DATA_NOACK; // NACK on transmit of data
            } else {
                return I2C_ERROR_OTHER; // other error
            }
        }
        if (i2c_tx_complete[controller_id]) {
            return I2C_OK;
        }
        delayMicroseconds(10);
    }
    return I2C_TIMEOUT;
}

static int wait_dev_ready(I2C_CONTROLLER controller_id, bool no_stop)
{
    uint64_t timeout = TIMEOUT_MS * 200;
    int ret = 0;

    while (timeout--) {
        ret = ss_i2c_status(controller_id, no_stop);
        if (ret == I2C_OK) {
            return I2C_OK;
        } else if (ret == I2C_BUSY) {
            delayMicroseconds(10);
        } else {
            return I2C_TIMEOUT - ret;
        }
    }
    return I2C_TIMEOUT - ret;
}

int i2c_openadapter(I2C_CONTROLLER controller_id)
{
    int ret;

    i2c_cfg_data_t i2c_cfg;
    memset(&i2c_cfg, 0, sizeof(i2c_cfg_data_t));

    i2c_cfg.speed = I2C_SLOW;
    i2c_cfg.addressing_mode = I2C_7_Bit;
    i2c_cfg.mode_type = I2C_MASTER;

    if (controller_id == I2C_SENSING_0) {
        SET_PIN_MODE(24, I2C_MUX_MODE); //  SOC PIN (Arduino header pin 18)
        SET_PIN_MODE(25, I2C_MUX_MODE); // Txd SOC PIN (Arduino header pin 19)

        SET_PIN_PULLUP(24, 1);
        SET_PIN_PULLUP(25, 1);
        i2c_cfg.cb_tx = ss_i2c_0_tx;
        i2c_cfg.cb_rx = ss_i2c_0_rx;
        i2c_cfg.cb_err = ss_i2c_0_err;
    } else if (controller_id == I2C_SENSING_1) {
        SET_PIN_MODE(26, I2C_MUX_MODE); // Rxd SOC PIN (Arduino header pin 18)
        SET_PIN_MODE(27, I2C_MUX_MODE); // Txd SOC PIN (Arduino header pin 19)

        SET_PIN_PULLUP(26, 1);
        SET_PIN_PULLUP(27, 1);
        i2c_cfg.cb_tx = ss_i2c_1_tx;
        i2c_cfg.cb_rx = ss_i2c_1_rx;
        i2c_cfg.cb_err = ss_i2c_1_err;
    } else {
        return I2C_ERROR;
    }

    i2c_tx_complete[controller_id] = 0;
    i2c_rx_complete[controller_id] = 0;
    i2c_err_detect[controller_id] = 0;
    i2c_err_source[controller_id] = 0;

    ss_i2c_set_config(controller_id, &i2c_cfg);
    ss_i2c_clock_enable(controller_id);
    ret = wait_dev_ready(controller_id, false);

    return ret;
}

int i2c_openadapter_speed(I2C_CONTROLLER controller_id, int i2c_speed)
{
    int ret;
    i2c_cfg_data_t i2c_cfg;
    memset(&i2c_cfg, 0, sizeof(i2c_cfg_data_t));

    i2c_cfg.speed = i2c_speed;
    i2c_cfg.addressing_mode = I2C_7_Bit;
    i2c_cfg.mode_type = I2C_MASTER;

    if (controller_id == I2C_SENSING_0) {
        SET_PIN_MODE(24, I2C_MUX_MODE); //  SOC PIN (Arduino header pin 18)
        SET_PIN_MODE(25, I2C_MUX_MODE); // Txd SOC PIN (Arduino header pin 19)

        SET_PIN_PULLUP(24, 1);
        SET_PIN_PULLUP(25, 1);
        i2c_cfg.cb_tx = ss_i2c_0_tx;
        i2c_cfg.cb_rx = ss_i2c_0_rx;
        i2c_cfg.cb_err = ss_i2c_0_err;
    } else if (controller_id == I2C_SENSING_1) {
        SET_PIN_MODE(26, I2C_MUX_MODE); // Rxd SOC PIN (Arduino header pin 18)
        SET_PIN_MODE(27, I2C_MUX_MODE); // Txd SOC PIN (Arduino header pin 19)

        SET_PIN_PULLUP(26, 1);
        SET_PIN_PULLUP(27, 1);
        i2c_cfg.cb_tx = ss_i2c_1_tx;
        i2c_cfg.cb_rx = ss_i2c_1_rx;
        i2c_cfg.cb_err = ss_i2c_1_err;
    } else {
        return I2C_ERROR;
    }

    i2c_tx_complete[controller_id] = 0;
    i2c_rx_complete[controller_id] = 0;
    i2c_err_detect[controller_id] = 0;
    i2c_err_source[controller_id] = 0;

    ss_i2c_set_config(controller_id, &i2c_cfg);
    ss_i2c_clock_enable(controller_id);
    ret = wait_dev_ready(controller_id, false);

    return ret;
}

void i2c_setslave(I2C_CONTROLLER controller_id, uint8_t addr)
{
    i2c_slave[controller_id] = addr;
    return;
}

int i2c_writebytes(I2C_CONTROLLER controller_id, uint8_t *bytes, uint8_t length,
                   bool no_stop)
{
    int ret;

    i2c_tx_complete[controller_id] = 0;
    i2c_err_detect[controller_id] = 0;
    i2c_err_source[controller_id] = 0;
    ss_i2c_transfer(controller_id, bytes, length, 0, 0,
                    i2c_slave[controller_id], no_stop);
    ret = wait_tx_or_err(controller_id);
    if (ret)
        return ret;
    ret = wait_dev_ready(controller_id, no_stop);
    if (ret)
        return ret;
    return length;
}

int i2c_readbytes(I2C_CONTROLLER controller_id, uint8_t *buf, int length,
                  bool no_stop)
{
    int ret;

    i2c_rx_complete[controller_id] = 0;
    i2c_err_detect[controller_id] = 0;
    i2c_err_source[controller_id] = 0;
    ss_i2c_transfer(controller_id, 0, 0, buf, length, i2c_slave[controller_id],
                    no_stop);
    ret = wait_rx_or_err(controller_id);
    if (ret)
        return ret;
    ret = wait_dev_ready(controller_id, no_stop);
    if (ret)
        return ret;
    return length;
}
