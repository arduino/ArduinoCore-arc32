/*
 * i2c.h
 *
 * Copyright (c) 2013 Parav Nagarsheth
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

#ifndef i2c_h
#define i2c_h

#include <inttypes.h>
#include <stdbool.h>
#include "ss_i2c_iface.h"

#ifdef __cplusplus
extern "C"{
#endif

#define I2C_OK       0
#define I2C_TIMEOUT -10
#define I2C_ERROR   -20    /* Qi, 2016. */
#define I2C_ERROR_ADDRESS_NOACK (-2)
#define I2C_ERROR_DATA_NOACK    (-3)
#define I2C_ERROR_OTHER         (-4)

#define I2C_ABRT_7B_ADDR_NOACK  (1 << 0)
#define I2C_ABRT_TXDATA_NOACK   (1 << 3)

int i2c_openadapter(I2C_CONTROLLER controller_id);
int i2c_openadapter_speed(I2C_CONTROLLER controller_id, int i2c_speed);
void i2c_setslave(I2C_CONTROLLER controller_id, uint8_t addr);
int i2c_writebytes(I2C_CONTROLLER controller_id, uint8_t *bytes, uint8_t length, bool no_stop);
int i2c_readbytes(I2C_CONTROLLER controller_id, uint8_t *buf, int length, bool no_stop);

#ifdef __cplusplus
}
#endif
#endif /* i2c_h */
