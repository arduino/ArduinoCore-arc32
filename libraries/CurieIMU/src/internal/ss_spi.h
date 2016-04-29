/*
 * Copyright (c) 2015 Intel Corporation.  All rights reserved.
 */
#ifndef _SS_SPI_H_
#define _SS_SPI_H_

#ifdef __cplusplus
extern "C" {
#endif

void ss_spi_init();
int ss_spi_xfer(uint8_t *buf, unsigned tx_cnt, unsigned rx_cnt);

#ifdef __cplusplus
}
#endif

#endif // _SS_SPI_H_
