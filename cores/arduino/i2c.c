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

#include <ss_i2c_iface.h>
#include "i2c.h"
#include "variant.h"

#define TIMEOUT_MS 16

static volatile uint8_t i2c_tx_complete;
static volatile uint8_t i2c_rx_complete;
static volatile uint8_t i2c_err_detect;

static volatile uint8_t i2c_slave = 0;

static void ss_i2c_rx(uint32_t dev_id)
{
	i2c_rx_complete = 1;
}

static void ss_i2c_tx(uint32_t dev_id)
{
	i2c_tx_complete = 1;
}

static void ss_i2c_err(uint32_t dev_id)
{
	i2c_err_detect = 1;
}

static int wait_rx_or_err(bool no_stop){
	uint64_t timeout = TIMEOUT_MS;
	while(timeout--) {
		if (i2c_err_detect) {
			return I2C_ERROR;
		}
		if (!no_stop) {
			if (i2c_rx_complete) {
				return I2C_OK;
			}
		}
		delay(1);
	}
	if (!no_stop)
		return I2C_TIMEOUT;
	else
		return I2C_OK;
}

static int wait_tx_or_err(bool no_stop){
	uint64_t timeout = TIMEOUT_MS;
	while(timeout--) {
		if (i2c_err_detect) {
			return I2C_ERROR;
		}
		if (!no_stop) {
			if (i2c_tx_complete) {
			return I2C_OK;
			}
		}
		delay(1);
	}
	if (!no_stop)
		return I2C_TIMEOUT;
	else
		return I2C_OK;
}

static int wait_dev_ready(I2C_CONTROLLER controller_id, bool no_stop){
	uint64_t timeout = TIMEOUT_MS;
	while(timeout--) {
		int ret = ss_i2c_status(controller_id, no_stop);
		if (ret == I2C_OK) {
			return I2C_OK;
		}
		if (ret == I2C_BUSY) {
			delay(1);
		}
	}
	return I2C_TIMEOUT;
}


int i2c_openadapter(void)
{
	int ret;

	SET_PIN_MODE(24, I2C_MUX_MODE); // Rdx SOC PIN (Arduino header pin 18)
	SET_PIN_MODE(25, I2C_MUX_MODE); // Txd SOC PIN (Arduino header pin 19)

	SET_PIN_PULLUP(24, 1);
	SET_PIN_PULLUP(25, 1);

	i2c_cfg_data_t i2c_cfg;
	memset(&i2c_cfg, 0, sizeof(i2c_cfg_data_t));

	i2c_cfg.speed = I2C_SLOW;
	i2c_cfg.addressing_mode = I2C_7_Bit;
	i2c_cfg.mode_type = I2C_MASTER;
	i2c_cfg.cb_tx = ss_i2c_tx;
	i2c_cfg.cb_rx = ss_i2c_rx;
	i2c_cfg.cb_err = ss_i2c_err;

	i2c_tx_complete = 0;
	i2c_rx_complete = 0;
	i2c_err_detect = 0;

	ss_i2c_set_config(I2C_SENSING_0, &i2c_cfg);
	ss_i2c_clock_enable(I2C_SENSING_0);
	ret = wait_dev_ready(I2C_SENSING_0, false);

	return ret;
}

void i2c_setslave(uint8_t addr)
{
	i2c_slave = addr;
	return;
}

int i2c_writebytes(uint8_t *bytes, uint8_t length, bool no_stop)
{
	int ret;

	i2c_tx_complete = 0;
	i2c_err_detect = 0;
	ss_i2c_transfer(I2C_SENSING_0, bytes, length, 0, 0, i2c_slave, no_stop);
	ret = wait_tx_or_err(no_stop);
	if (ret)
		return ret;
	ret = wait_dev_ready(I2C_SENSING_0, no_stop);
	if (ret)
		return ret;
	return length;
}

int i2c_readbytes(uint8_t *buf, int length, bool no_stop)
{
	int ret;

	i2c_rx_complete = 0;
	i2c_err_detect = 0;
	ss_i2c_transfer(I2C_SENSING_0, 0, 0, buf, length, i2c_slave, no_stop);
	ret = wait_rx_or_err(no_stop);
	if (ret)
		return ret;
	ret = wait_dev_ready(I2C_SENSING_0, no_stop);
	if (ret)
		return ret;
	return length;
}
