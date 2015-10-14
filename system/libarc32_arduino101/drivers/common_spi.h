/*
 * Copyright (c) 2015, Intel Corporation. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * Intel common SPI header
 */

#ifndef COMMON_SPI_H_
#define COMMON_SPI_H_

#include <stdint.h>

/**
 * @defgroup common_driver Common SPI
 * Serial Peripheral Interface bus drivers API.
 * @ingroup common_drivers
 * @{
 */

/**
 * Driver status return codes.
 */
typedef enum {
    SPI_OK = 0,                 /*!< SPI OK */
    SPI_BUSY,                   /*!< SPI busy */
    SPI_TFE,                    /*!< TX FIFO Empty */
    SPI_RFNE                    /*!< RX FIFO Not Empty */
}DRIVER_SPI_STATUS_CODE;

/**
 * SPI Bus Modes.
 */
typedef enum {                  /*!<   Mode    Clk Polarity    Clk Phase    */
    SPI_BUSMODE_0 = 0x00,       /*!<    0        0              0           */
    SPI_BUSMODE_1 = 0x01,       /*!<    1        0              1           */
    SPI_BUSMODE_2 = 0x02,       /*!<    2        1              0           */
    SPI_BUSMODE_3 = 0x03        /*!<    3        1              1           */
}SPI_BUS_MODE;

/**
 * SPI Transfer modes.
 */
typedef enum {
    SPI_TX_RX = 0,      /*!< SPI WRITE/READ MODE */
    SPI_TX_ONLY,        /*!< SPI WRITE ONLY */
    SPI_RX_ONLY,        /*!< SPI READ ONLY */
    SPI_EPROM_RD        /*!< SPI READ EEPROM MODE */
}SPI_TRANSFER_MODE;

/**
 * Slave selects.
 */
typedef enum {
    SPI_NO_SE   = 0,        /*!< SPI NONE SLAVE ENABLE */
    SPI_SE_1    = 0x01,     /*!< SPI SLAVE 1 ENABLE */
    SPI_SE_2    = 0x02,     /*!< SPI SLAVE 2 ENABLE */
    SPI_SE_3    = 0x04,     /*!< SPI SLAVE 3 ENABLE */
    SPI_SE_4    = 0x08      /*!< SPI SLAVE 4 ENABLE */
}SPI_SLAVE_ENABLE;

/**
 * Data frame sizes.
 */
typedef enum {
    SPI_4_BIT  = 3,         /*!< starts at 3 (SPI_4_BIT) because lower values are reserved */
    SPI_5_BIT  = 4,         /*!< setting 4 bits frame*/
    SPI_6_BIT  = 5,         /*!< setting 5 bits frame*/
    SPI_7_BIT  = 6,         /*!< setting 6 bits frame*/
    SPI_8_BIT  = 7,         /*!< setting 7 bits frame*/
    SPI_9_BIT  = 8,         /*!< setting 8 bits frame*/
    SPI_10_BIT = 9,         /*!< setting 9 bits frame*/
    SPI_11_BIT = 10,        /*!< setting 10 bits frame*/
    SPI_12_BIT = 11,        /*!< setting 11 bits frame*/
    SPI_13_BIT = 12,        /*!< setting 12 bits frame*/
    SPI_14_BIT = 13,        /*!< setting 13 bits frame*/
    SPI_15_BIT = 14,        /*!< setting 14 bits frame*/
    SPI_16_BIT = 15         /*!< setting 15 bits frame*/
}SPI_DATA_FRAME_SIZE;

/**
 * SPI mode types.
 */
typedef enum {
    SPI_MASTER = 0,     /*!< SPI MASTER MODE */
    SPI_SLAVE           /*!< SPI SLAVE MODE */
} SPI_MODE_TYPE;

/**
*    callback function signature.
*/
typedef void (*spi_callback)( uint32_t );

/**
* SPI controller configuration.
* Driver instantiates one of these with given parameters for each SPI
* controller configured using the "ss_spi_set_config" function
*/
typedef struct spi_cfg_data{
    uint32_t            speed;              /*!< SPI bus speed in KHz   */
    SPI_TRANSFER_MODE   txfr_mode;          /*!< Transfer mode          */
    SPI_DATA_FRAME_SIZE data_frame_size;    /*!< Data Frame Size ( 4 - 16 bits ) */
    SPI_SLAVE_ENABLE    slave_enable;       /*!< Slave Enable ( 0 = none - possibly used for Slaves that are selected by GPIO ) */
    SPI_BUS_MODE        bus_mode;           /*!< See SPI_BUS_MODE above for description */
    spi_callback        cb_xfer             /*!< callback function for notification of transmit completion by the SPI controller,  NULL if callback not required by application code */;
    uint32_t            cb_xfer_data;       /*!< this will be passed back by the callback routine - can be used as an controller identifier */
    spi_callback        cb_err;             /*!< callback function on transaction error, NULL if callback not required by application code */
    uint32_t            cb_err_data;        /*!< this will be passed back by the callback routine - can be used as an controller identifier */
    uint8_t             loopback_enable;    /*!< (ONLY available for SOC) this will enable loopback mode on spi module */
    spi_callback        cb_slave_rx;        /*!< (ONLY available for SOC) callback funciton for slave mode on spi module */
}spi_cfg_data_t;

/** @} */

#endif /* COMMON_SPI_H_ */
