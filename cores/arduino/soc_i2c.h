/*
 * soc_i2c.h
 *
 * Copyright (c) 2016 Intel Corporation
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
 * You should have received a copy of the GNU Lesser General
 * Public License along with this library; if not, write to the
 * Free Software Foundation, Inc., 59 Temple Place, Suite 330,
 * Boston, MA  02111-1307  USA
 */

#ifndef soc_i2c_h_
#define soc_i2c_h_

#include <inttypes.h>
#include <stdbool.h>
#include "intel_qrk_i2c.h"

#ifdef __cplusplus
extern "C" {
#endif

#define I2C_ABRT_10ADDR1_NOACK  (1 << 1)
#define I2C_ABRT_10ADDR2_NOACK  (1 << 2)

int soc_i2c_open_adapter(SOC_I2C_CONTROLLER controller_id, uint32_t address, int i2c_speed, int i2c_addr_mode);
void soc_i2c_close_adapter(SOC_I2C_CONTROLLER controller_id);
void soc_i2c_set_speed(SOC_I2C_CONTROLLER controller_id, uint32_t speed);
void soc_i2c_set_address_mode(SOC_I2C_CONTROLLER controller_id, uint32_t mode);
void soc_i2c_master_set_slave_address(SOC_I2C_CONTROLLER controller_id, uint32_t addr);
int soc_i2c_master_witebytes(SOC_I2C_CONTROLLER controller_id, uint8_t *bytes, uint8_t length, bool no_stop);
int soc_i2c_master_readbytes(SOC_I2C_CONTROLLER controller_id, uint8_t *buf, int length, bool no_stop);
  void soc_i2c_slave_set_rx_user_callback(SOC_I2C_CONTROLLER controller_id, void (*onReceiveCallback)(int, void *), void *callerDataPtr);
void soc_i2c_slave_set_tx_user_callback(SOC_I2C_CONTROLLER controller_id, void (*onRequestCallback)(void *), void *callerDataPtr);
void soc_i2c_slave_set_rx_user_buffer(SOC_I2C_CONTROLLER controller_id, uint8_t *buffer, uint8_t length);
void soc_i2c_slave_set_tx_user_buffer(SOC_I2C_CONTROLLER controller_id, uint8_t *buffer, uint8_t length);

#ifdef __cplusplus
}
#endif
#endif /* soc_i2c_h */
