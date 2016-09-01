/*
 * soc_i2c.c - i2c library layer
 *
 * Copyright (C) 2015, 2016 Intel Corporation
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

#include <stdbool.h>
#include "intel_qrk_i2c.h"
#include "variant.h"
#include "soc_i2c.h"
#include "i2c.h"

#define TIMEOUT_MS 16

static volatile uint8_t soc_i2c_master_tx_complete[NUM_SOC_I2C];
static volatile uint8_t soc_i2c_master_rx_complete[NUM_SOC_I2C];
static volatile uint8_t soc_i2c_err_detect[NUM_SOC_I2C];
static volatile uint32_t soc_i2c_err_source[NUM_SOC_I2C];

static volatile uint32_t soc_i2c_slave_address[NUM_SOC_I2C] = {0, 0};

static void soc_i2c0_master_rx_callback(uint32_t dev_id)
{
    soc_i2c_master_rx_complete[SOC_I2C_0] = 1;
}

static void soc_i2c1_master_rx_callback(uint32_t dev_id)
{
    soc_i2c_master_rx_complete[SOC_I2C_1] = 1;
}

static void soc_i2c0_master_tx_callback(uint32_t dev_id)
{
    soc_i2c_master_tx_complete[SOC_I2C_0] = 1;
}

static void soc_i2c1_master_tx_callback(uint32_t dev_id)
{
    soc_i2c_master_tx_complete[SOC_I2C_1] = 1;
}

static void soc_i2c0_err_callback(uint32_t dev_id)
{
    soc_i2c_err_detect[SOC_I2C_0] = 1;
    soc_i2c_err_source[SOC_I2C_0] = dev_id;
}

static void soc_i2c1_err_callback(uint32_t dev_id)
{
    soc_i2c_err_detect[SOC_I2C_1] = 1;
    soc_i2c_err_source[SOC_I2C_1] = dev_id;
}

static void (*soc_i2c0_slave_rx_user_callback)(int, void *) = NULL;
static void *soc_i2c0_slave_rx_user_cb_data_ptr = NULL;

static void (*soc_i2c1_slave_rx_user_callback)(int, void *) = NULL;
static void *soc_i2c1_slave_rx_user_cb_data_ptr = NULL;

static void (*soc_i2c0_slave_tx_user_callback)(void *) = NULL;
static void *soc_i2c0_slave_tx_user_cb_data_ptr = NULL;

static void (*soc_i2c1_slave_tx_user_callback)(void *) = NULL;
static void *soc_i2c1_slave_tx_user_cb_data_ptr = NULL;

static void soc_i2c0_slave_rx_callback(uint32_t bytes)
{
    if (soc_i2c0_slave_rx_user_callback) {
      soc_i2c0_slave_rx_user_callback((int)bytes, soc_i2c0_slave_rx_user_cb_data_ptr);
    }
}

static void soc_i2c1_slave_rx_callback(uint32_t bytes)
{
    if (soc_i2c1_slave_rx_user_callback) {
      soc_i2c1_slave_rx_user_callback((int)bytes, soc_i2c1_slave_rx_user_cb_data_ptr);
    }
}

static void soc_i2c0_slave_tx_callback(uint32_t bytes)
{
    if (soc_i2c0_slave_tx_user_callback) {
        soc_i2c0_slave_tx_user_callback(soc_i2c0_slave_tx_user_cb_data_ptr);
    }
}

static void soc_i2c1_slave_tx_callback(uint32_t bytes)
{
    if (soc_i2c1_slave_tx_user_callback) {
        soc_i2c1_slave_tx_user_callback(soc_i2c1_slave_tx_user_cb_data_ptr);
    }
}

void soc_i2c_slave_set_rx_user_callback(SOC_I2C_CONTROLLER controller_id, void (*onReceiveCallback)(int, void *), void *callerDataPtr)
{
  if (controller_id == SOC_I2C_0) {
    soc_i2c0_slave_rx_user_callback = onReceiveCallback;
    soc_i2c0_slave_rx_user_cb_data_ptr = callerDataPtr;
  } else {
    soc_i2c1_slave_rx_user_callback = onReceiveCallback;
    soc_i2c1_slave_rx_user_cb_data_ptr = callerDataPtr;
  }
}

void soc_i2c_slave_set_tx_user_callback(SOC_I2C_CONTROLLER controller_id, void (*onRequestCallback)(void *), void *callerDataPtr)
{
  if (controller_id == SOC_I2C_0) {
    soc_i2c0_slave_tx_user_callback = onRequestCallback;
    soc_i2c0_slave_tx_user_cb_data_ptr = callerDataPtr;
  } else {
    soc_i2c1_slave_tx_user_callback = onRequestCallback;
    soc_i2c1_slave_tx_user_cb_data_ptr = callerDataPtr;
  }
}

static int soc_i2c_master_wait_rx_or_err(SOC_I2C_CONTROLLER controller_id)
{
    uint64_t timeout = TIMEOUT_MS * 200;
    while (timeout--) {
        if (soc_i2c_err_detect[controller_id]) {
            if (soc_i2c_err_source[controller_id] &
		(I2C_ABRT_7B_ADDR_NOACK | I2C_ABRT_10ADDR1_NOACK | I2C_ABRT_10ADDR2_NOACK)) {
	      return I2C_ERROR_ADDRESS_NOACK; // NACK on transmit of address
            }
	    else if (soc_i2c_err_source[controller_id] & I2C_ABRT_TXDATA_NOACK) {
	      return I2C_ERROR_DATA_NOACK; // NACK on transmit of data
            } else {
	      return I2C_ERROR_OTHER; // other error
            }
        }
        if (soc_i2c_master_rx_complete[controller_id]) {
            return I2C_OK;
        }
        delayMicroseconds(10);
    }
    return I2C_TIMEOUT;
}

static int soc_i2c_master_wait_tx_or_err(SOC_I2C_CONTROLLER controller_id)
{
    uint64_t timeout = TIMEOUT_MS * 200;
    while (timeout--) {
        if (soc_i2c_err_detect[controller_id]) {
            if (soc_i2c_err_source[controller_id] &
		(I2C_ABRT_7B_ADDR_NOACK | I2C_ABRT_10ADDR1_NOACK | I2C_ABRT_10ADDR2_NOACK)) {
	      return I2C_ERROR_ADDRESS_NOACK; // NACK on transmit of address
            }
	    else if (soc_i2c_err_source[controller_id] & I2C_ABRT_TXDATA_NOACK) {
	      return I2C_ERROR_DATA_NOACK; // NACK on transmit of data
            } else {
	      return I2C_ERROR_OTHER; // other error
            }
        }
        if (soc_i2c_master_tx_complete[controller_id]) {
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

int soc_i2c_open_adapter(SOC_I2C_CONTROLLER controller_id, uint32_t address, int i2c_speed, int i2c_addr_mode)
{
    int ret = 0;

    if (controller_id == SOC_I2C_0) {
      SET_PIN_MODE(20, I2C_MUX_MODE);
      SET_PIN_MODE(21, I2C_MUX_MODE);

      SET_PIN_PULLUP(20, 1);
      SET_PIN_PULLUP(21, 1);
    } else {
      SET_PIN_MODE(22, I2C_MUX_MODE);
      SET_PIN_MODE(23, I2C_MUX_MODE);

      SET_PIN_PULLUP(22, 1);
      SET_PIN_PULLUP(23, 1);
    }

    i2c_cfg_data_t i2c_cfg;
    memset(&i2c_cfg, 0, sizeof(i2c_cfg_data_t));

    i2c_cfg.speed = i2c_speed;
    i2c_cfg.addressing_mode = i2c_addr_mode;
    if (address) {
        i2c_cfg.mode_type = I2C_SLAVE;
	if (controller_id == SOC_I2C_0) {
	  i2c_cfg.cb_err = soc_i2c0_err_callback;
	  i2c_cfg.cb_rx = soc_i2c0_slave_rx_callback;
	  i2c_cfg.cb_tx = soc_i2c0_slave_tx_callback;
	} else {
	  i2c_cfg.cb_err = soc_i2c1_err_callback;
	  i2c_cfg.cb_rx = soc_i2c1_slave_rx_callback;
	  i2c_cfg.cb_tx = soc_i2c1_slave_tx_callback;
	}
    } else {
        i2c_cfg.mode_type = I2C_MASTER;

	if (controller_id == SOC_I2C_0) {
	  i2c_cfg.cb_tx = soc_i2c0_master_tx_callback;
	  i2c_cfg.cb_rx = soc_i2c0_master_rx_callback;
	  i2c_cfg.cb_err = soc_i2c0_err_callback;
	} else {
	  i2c_cfg.cb_tx = soc_i2c1_master_tx_callback;
	  i2c_cfg.cb_rx = soc_i2c1_master_rx_callback;
	  i2c_cfg.cb_err = soc_i2c1_err_callback;
	}
        soc_i2c_master_tx_complete[controller_id] = 0;
        soc_i2c_master_rx_complete[controller_id] = 0;
    }
    i2c_cfg.slave_adr = address;
    soc_i2c_err_detect[controller_id] = 0;
    soc_i2c_err_source[controller_id] = 0;

    soc_i2c_set_config(controller_id, &i2c_cfg);
    soc_i2c_clock_enable(controller_id);

    ret = soc_i2c_wait_dev_ready(controller_id, false);

    return ret;
}

void soc_i2c_close_adapter(SOC_I2C_CONTROLLER controller_id)
{
    soc_i2c_deconfig(controller_id);
    soc_i2c_clock_disable(controller_id);

    if (controller_id == SOC_I2C_0) {
      SET_PIN_MODE(20, GPIO_MUX_MODE);
      SET_PIN_MODE(21, GPIO_MUX_MODE);
    } else {
      SET_PIN_MODE(22, GPIO_MUX_MODE);
      SET_PIN_MODE(23, GPIO_MUX_MODE);
    }

    return;
}

void soc_i2c_set_speed(SOC_I2C_CONTROLLER controller_id, uint32_t speed)
{
    soc_i2c_set_transfer_speed(controller_id, speed);
}

void soc_i2c_set_address_mode(SOC_I2C_CONTROLLER controller_id, uint32_t mode)
{
    soc_i2c_set_transfer_mode(controller_id, mode);
}

void soc_i2c_master_set_slave_address(SOC_I2C_CONTROLLER controller_id, uint32_t addr)
{
    soc_i2c_slave_address[controller_id] = addr;
    return;
}

void soc_i2c_slave_set_rx_user_buffer(SOC_I2C_CONTROLLER controller_id, uint8_t *buffer, uint8_t length)
{
    soc_i2c_slave_enable_rx(controller_id, buffer, length);
}

void soc_i2c_slave_set_tx_user_buffer(SOC_I2C_CONTROLLER controller_id, uint8_t *buffer, uint8_t length)
{
    soc_i2c_slave_enable_tx(controller_id, buffer, length);
}

int soc_i2c_master_witebytes(SOC_I2C_CONTROLLER controller_id, uint8_t *buf, uint8_t length, bool no_stop)
{
    int ret;

    soc_i2c_master_tx_complete[controller_id] = 0;
    soc_i2c_err_detect[controller_id] = 0;
    soc_i2c_err_source[controller_id] = 0;
    soc_i2c_master_transfer(controller_id, buf, length, 0, 0,
			    soc_i2c_slave_address[controller_id], no_stop);
    ret = soc_i2c_master_wait_tx_or_err(controller_id);
    if (ret)
        return ret;
    ret = soc_i2c_wait_dev_ready(controller_id, no_stop);
    if (ret)
        return ret;
    return length;
}

int soc_i2c_master_readbytes(SOC_I2C_CONTROLLER controller_id, uint8_t *buf, int length, bool no_stop)
{
    int ret;

    soc_i2c_master_rx_complete[controller_id] = 0;
    soc_i2c_err_detect[controller_id] = 0;
    soc_i2c_err_source[controller_id] = 0;
    soc_i2c_master_transfer(controller_id, 0, 0, buf, length,
			    soc_i2c_slave_address[controller_id], no_stop);
    ret = soc_i2c_master_wait_rx_or_err(controller_id);
    if (ret)
        return ret;
    ret = soc_i2c_wait_dev_ready(controller_id, no_stop);
    if (ret)
        return ret;
    return length;
}
