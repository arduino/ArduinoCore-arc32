/*
 * soc_i2c.c - i2c library layer
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
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301 USA
 *
 */

#include "soc_i2c.h"
#include "i2c.h"
#include "variant.h"
#include <stdbool.h>

#define TIMEOUT_MS 16

static volatile uint8_t soc_i2c_master_tx_complete;
static volatile uint8_t soc_i2c_master_rx_complete;
static volatile uint8_t soc_i2c_err_detect;
static volatile uint32_t soc_i2c_err_source;

static volatile uint32_t soc_i2c_slave_address = 0;

static void soc_i2c_master_rx_callback(uint32_t dev_id)
{
    soc_i2c_master_rx_complete = 1;
}

static void soc_i2c_master_tx_callback(uint32_t dev_id)
{
    soc_i2c_master_tx_complete = 1;
}

static void soc_i2c_err_callback(uint32_t dev_id)
{
    soc_i2c_err_detect = 1;
    soc_i2c_err_source = dev_id;
}

static void (*soc_i2c_slave_rx_user_callback)(int) = NULL;
static void (*soc_i2c_slave_tx_user_callback)(void) = NULL;

static void soc_i2c_slave_rx_callback(uint32_t bytes)
{
    if (soc_i2c_slave_rx_user_callback) {
        soc_i2c_slave_rx_user_callback((int)bytes);
    }
}

static void soc_i2c_slave_tx_callback(uint32_t bytes)
{
    if (soc_i2c_slave_tx_user_callback) {
        soc_i2c_slave_tx_user_callback();
    }
}

void soc_i2c_slave_set_rx_user_callback(void (*onReceiveCallback)(int))
{
    soc_i2c_slave_rx_user_callback = onReceiveCallback;
}

void soc_i2c_slave_set_tx_user_callback(void (*onRequestCallback)(void))
{
    soc_i2c_slave_tx_user_callback = onRequestCallback;
}

static int soc_i2c_master_wait_rx_or_err()
{
    uint64_t timeout = TIMEOUT_MS * 200;
    while (timeout--) {
        if (soc_i2c_err_detect) {
            if (soc_i2c_err_source & I2C_ABRT_7B_ADDR_NOACK) {
                return I2C_ERROR_ADDRESS_NOACK; // NACK on transmit of address
            } else if (soc_i2c_err_source & I2C_ABRT_TXDATA_NOACK) {
                return I2C_ERROR_DATA_NOACK; // NACK on transmit of data
            } else {
                return I2C_ERROR_OTHER; // other error
            }
        }
        if (soc_i2c_master_rx_complete) {
            return I2C_OK;
        }
        delayMicroseconds(10);
    }
    return I2C_TIMEOUT;
}

static int soc_i2c_master_wait_tx_or_err()
{
    uint64_t timeout = TIMEOUT_MS * 200;
    while (timeout--) {
        if (soc_i2c_err_detect) {
            if (soc_i2c_err_source & I2C_ABRT_7B_ADDR_NOACK) {
                return I2C_ERROR_ADDRESS_NOACK; // NACK on transmit of address
            } else if (soc_i2c_err_source & I2C_ABRT_TXDATA_NOACK) {
                return I2C_ERROR_DATA_NOACK; // NACK on transmit of data
            } else {
                return I2C_ERROR_OTHER; // other error
            }
        }
        if (soc_i2c_master_tx_complete) {
            return I2C_OK;
        }
        delayMicroseconds(10);
    }
    return I2C_TIMEOUT;
}

static int soc_i2c_wait_dev_ready(SOC_I2C_CONTROLLER controller_id,
                                  bool no_stop)
{
    uint64_t timeout = TIMEOUT_MS * 200;
    int ret = 0;
    while (timeout--) {
        ret = soc_i2c_status(controller_id, no_stop);
        if (ret == I2C_OK) {
            return I2C_OK;
        }
        if (ret == I2C_BUSY) {
            delayMicroseconds(10);
        }
    }
    return I2C_TIMEOUT - ret;
}

int soc_i2c_openadapter(uint32_t address, int i2c_speed, int i2c_addr_mode)
{
    int ret = 0;

    // use I2C0
    SET_PIN_MODE(20, I2C_MUX_MODE);
    SET_PIN_MODE(21, I2C_MUX_MODE);

    SET_PIN_PULLUP(20, 1);
    SET_PIN_PULLUP(21, 1);

    i2c_cfg_data_t i2c_cfg;
    memset(&i2c_cfg, 0, sizeof(i2c_cfg_data_t));

    i2c_cfg.speed = i2c_speed;
    i2c_cfg.addressing_mode = i2c_addr_mode;
    if (address) {
        i2c_cfg.mode_type = I2C_SLAVE;
        i2c_cfg.cb_tx = soc_i2c_slave_tx_callback;
        i2c_cfg.cb_rx = soc_i2c_slave_rx_callback;
        i2c_cfg.cb_err = soc_i2c_err_callback;
    } else {
        i2c_cfg.mode_type = I2C_MASTER;
        i2c_cfg.cb_tx = soc_i2c_master_tx_callback;
        i2c_cfg.cb_rx = soc_i2c_master_rx_callback;
        i2c_cfg.cb_err = soc_i2c_err_callback;
        soc_i2c_master_tx_complete = 0;
        soc_i2c_master_rx_complete = 0;
    }
    i2c_cfg.slave_adr = address;
    soc_i2c_err_detect = 0;
    soc_i2c_err_source = 0;

    soc_i2c_set_config(SOC_I2C_0, &i2c_cfg);
    soc_i2c_clock_enable(SOC_I2C_0);

    ret = soc_i2c_wait_dev_ready(SOC_I2C_0, false);

    return ret;
}

void soc_i2c_master_set_slave_address(uint32_t addr)
{
    soc_i2c_slave_address = addr;
    return;
}

void soc_i2c_slave_set_rx_user_buffer(uint8_t *buffer, uint8_t length)
{
    soc_i2c_slave_enable_rx(SOC_I2C_0, buffer, length);
}

void soc_i2c_slave_set_tx_user_buffer(uint8_t *buffer, uint8_t length)
{
    soc_i2c_slave_enable_tx(SOC_I2C_0, buffer, length);
}

int soc_i2c_master_witebytes(uint8_t *bytes, uint8_t length, bool no_stop)
{
    int ret;

    soc_i2c_master_tx_complete = 0;
    soc_i2c_err_detect = 0;
    soc_i2c_err_source = 0;
    soc_i2c_master_transfer(SOC_I2C_0, bytes, length, 0, 0,
                            soc_i2c_slave_address, no_stop);
    ret = soc_i2c_master_wait_tx_or_err();
    if (ret)
        return ret;
    ret = soc_i2c_wait_dev_ready(SOC_I2C_0, no_stop);
    if (ret)
        return ret;
    return length;
}

int soc_i2c_master_readbytes(uint8_t *buf, int length, bool no_stop)
{
    int ret;

    soc_i2c_master_rx_complete = 0;
    soc_i2c_err_detect = 0;
    soc_i2c_err_source = 0;
    soc_i2c_master_transfer(SOC_I2C_0, 0, 0, buf, length, soc_i2c_slave_address,
                            no_stop);
    ret = soc_i2c_master_wait_rx_or_err();
    if (ret)
        return ret;
    ret = soc_i2c_wait_dev_ready(SOC_I2C_0, no_stop);
    if (ret)
        return ret;
    return length;
}
