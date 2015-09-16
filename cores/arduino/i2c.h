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
#ifdef __cplusplus
extern "C"{
#endif

#define I2C_OK       0
#define I2C_TIMEOUT -1
#define I2C_ERROR   -2

int i2c_openadapter(void);
void i2c_setslave(uint8_t addr);
int i2c_writebytes(uint8_t *bytes, uint8_t length, bool no_stop);
int i2c_readbytes(uint8_t *buf, int length, bool no_stop);

#ifdef __cplusplus
}
#endif
#endif /* i2c_h */
