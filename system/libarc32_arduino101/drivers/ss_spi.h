/*
 * Copyright (c) 2015 Intel Corporation.  All rights reserved.
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.

 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.

 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
 *
 */

#ifndef _SS_SPI_H_
#define _SS_SPI_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "common_spi.h"

/**
 * List of all controllers
 */
typedef enum {
    SPI_SENSING_0, /* Sensing SPI controller 0, accessible by Sensor Subsystem Core only */
    SPI_SENSING_1 /* Sensing SPI controller 1, accessible by Sensor Subsystem Core only */
} SPI_CONTROLLER;

void ss_spi_init(SPI_CONTROLLER controller_id, uint32_t speed,
                 SPI_BUS_MODE mode, SPI_DATA_FRAME_SIZE data_frame_size,
                 SPI_SLAVE_ENABLE slave);
void ss_spi_disable(SPI_CONTROLLER controller_id);
int ss_spi_xfer(SPI_CONTROLLER controller_id, uint8_t *buf, unsigned tx_cnt,
                unsigned rx_cnt);
void ss_spi_set_data_mode(SPI_CONTROLLER controller_id, uint8_t dataMode);
void ss_spi_set_clock_divider(SPI_CONTROLLER controller_id, uint8_t clockDiv);

#ifdef __cplusplus
}
#endif

#endif // _SS_SPI_H_
