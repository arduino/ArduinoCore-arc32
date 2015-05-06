/** INTEL CONFIDENTIAL Copyright 2015 Intel Corporation All Rights Reserved.
  *
  * The source code contained or described herein and all documents related to
  * the source code ("Material") are owned by Intel Corporation or its suppliers
  * or licensors.
  * Title to the Material remains with Intel Corporation or its suppliers and
  * licensors.
  * The Material contains trade secrets and proprietary and confidential information
  * of Intel or its suppliers and licensors. The Material is protected by worldwide
  * copyright and trade secret laws and treaty provisions.
  * No part of the Material may be used, copied, reproduced, modified, published,
  * uploaded, posted, transmitted, distributed, or disclosed in any way without
  * Intel's prior express written permission.
  *
  * No license under any patent, copyright, trade secret or other intellectual
  * property right is granted to or conferred upon you by disclosure or delivery
  * of the Materials, either expressly, by implication, inducement, estoppel or
  * otherwise.
  *
  * Any license under such intellectual property rights must be express and
  * approved by Intel in writing
  *
  ******************************************************************************/
/*
 * Intel common SPI header
 *
 */

#ifndef COMMON_SPI_H_
#define COMMON_SPI_H_

/**
 * \addtogroup common_driver
 * @{
 * \defgroup common_driver SPI: Serial Peripheral API
 * @{
 * \brief Definition of the structure and functions used by SPI ARC and SOC Drivers implementation.
 */

/*!
 * Driver status return codes
 */
typedef enum {
    SPI_OK = 0,
    SPI_BUSY,
    SPI_TFE,                    /* TX FIFO Empty */
    SPI_RFNE                    /* RX FIFO Not Empty */
}DRIVER_SPI_STATUS_CODE;

/*!
 * SPI Bus Modes
 */
typedef enum {                  /*!<   Mode    Clk Polarity    Clk Phase    */
    SPI_BUSMODE_0 = 0x00,       /*!<    0        0              0           */
    SPI_BUSMODE_1 = 0x01,       /*!<    1        0              1           */
    SPI_BUSMODE_2 = 0x02,       /*!<    2        1              0           */
    SPI_BUSMODE_3 = 0x03        /*!<    3        1              1           */
}SPI_BUS_MODE;

/*!
 * SPI Transfer modes
 */
typedef enum {
    SPI_TX_RX = 0,
    SPI_TX_ONLY,
    SPI_RX_ONLY,
    SPI_EPROM_RD
}SPI_TRANSFER_MODE;

/*!
 * Slave selects
 */
typedef enum {
    SPI_NO_SE   = 0,
    SPI_SE_1    = 0x01,
    SPI_SE_2    = 0x02,
    SPI_SE_3    = 0x04,
    SPI_SE_4    = 0x08
}SPI_SLAVE_ENABLE;

/*!
 * Data frame sizes
 */
typedef enum {
    SPI_4_BIT  = 3,          /*!< starts at 3 because values less than this are reserved */
    SPI_5_BIT  = 4,
    SPI_6_BIT  = 5,
    SPI_7_BIT  = 6,
    SPI_8_BIT  = 7,
    SPI_9_BIT  = 8,
    SPI_10_BIT = 9,
    SPI_11_BIT = 10,
    SPI_12_BIT = 11,
    SPI_13_BIT = 12,
    SPI_14_BIT = 13,
    SPI_15_BIT = 14,
    SPI_16_BIT = 15
}SPI_DATA_FRAME_SIZE;

/*!
 * SPI mode types
 */
typedef enum {
    SPI_MASTER = 0,
    SPI_SLAVE
} SPI_MODE_TYPE;

/*!
*  \brief  callback function signature
*/
typedef void (*spi_callback)( uint32_t );     /*!< callback function signature */

/*!
*  \brief   SPI controller configuration.
*           Driver instantiates one of these with given parameters for each SPI
*           controller configured using the "ss_spi_set_config" function
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

/**@} @}*/

#endif /* COMMON_SPI_H_ */