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
  * Intel’s prior express written permission.
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
 * Intel SOC SPI driver
 *
 */

#ifndef INTEL_QRK_SPI_H_
#define INTEL_QRK_SPI_H_

/**
 * \addtogroup common_drivers
 * @{
 * \defgroup spi SPI: Serial Peripheral Interface API
 * @{
 * \brief Definition of the structure and functions used by SPI Driver implementation.
 */

#include "data_type.h"
#include "common_spi.h"

/*!
 *  Clock speed into SPI peripheral - set at compile time
 */
#define FREQ_SPI_CLOCK_IN   (CLOCK_SPEED*1000*1000)        /* CLOCK_SPEED in MHz */

/*!
 * List of all controllers in host processor
 */
typedef enum {
    SOC_SPI_MASTER_0 = 0,     /* SPI master controller 0, accessible by both processing entities */
    SOC_SPI_MASTER_1,         /* SPI master controller 1, accessible by both processing entities */
    SOC_SPI_SLAVE_0           /* SPI slave controller */
}SOC_SPI_CONTROLLER;

#ifdef __cplusplus
 extern "C" {
#endif

/*! \fn     DRIVER_API_RC soc_spi_set_config(SOC_SPI_CONTROLLER controller_id, spi_cfg_data_t *config)
*
*  \brief   Function to configure specified SPI controller
*           Configuration parameters must be valid or an error is returned - see return values below.
*
*  \param   controller_id   : SPI  controller identifier
*  \param   config          : pointer to configuration structure
*
*  \return  RC_OK                           - on success,
*           RC_DEVICE_TYPE_NOT_SUPPORTED    - if device type is not supported by this controller
*           RC_INVALID_CONFIG               - if any configuration parameters are not valid
*           RC_CONTROLLER_IN_USE,           - if controller is in use
*           RC_CONTROLLER_NOT_ACCESSIBLE    - if controller is not accessible from this core
*           RC_FAIL                         - otherwise
*/
DRIVER_API_RC soc_spi_set_config(SOC_SPI_CONTROLLER controller_id, spi_cfg_data_t *config);

/*! \fn     DRIVER_API_RC soc_spi_get_config(SOC_SPI_CONTROLLER controller_id, spi_cfg_data_t *config)
*
*  \brief   Function to retrieve configuration of specified SPI controller
*
*  \param   controller_id   : SPI controller identifier
*  \param   config          : pointer to configuration structure to store current setup
*
*  \return  RC_OK   - on success
*           RC_FAIL - otherwise
*/
DRIVER_API_RC soc_spi_get_config(SOC_SPI_CONTROLLER controller_id, spi_cfg_data_t *config);

/*! \fn     DRIVER_API_RC soc_spi_deconfig(SOC_SPI_CONTROLLER controller_id)
*
*  \brief   Function to place SPI controller into a disabled and default state (as if hardware reset occurred)
*           This function assumes that there is no pending transaction on the SPI interface in question.
*           It is the responsibility of the calling application to do so.
*           Upon success, the specified SPI interface is clock gated in hardware,
*           it is no longer capable to generating interrupts, it is also configured into a default state
*
*  \param   controller_id   : SPI controller identifier
*
*  \return  RC_OK on success
*           RC_FAIL otherwise
*/
DRIVER_API_RC soc_spi_deconfig(SOC_SPI_CONTROLLER controller_id);

/*! \fn     DRIVER_API_RC soc_spi_clock_enable(SOC_SPI_CONTROLLER controller_id)
*
*  \brief   Function to enable the specified SPI controller
*           Upon success, the specified SPI interface is no longer clock gated in hardware, it is now
*           capable of transmitting and receiving on the SPI bus and of generating interrupts.
*
*  \param   controller_id   : SPI controller identifier
*
*  \return  RC_OK on success
*           RC_FAIL otherwise
*/
DRIVER_API_RC soc_spi_clock_enable(SOC_SPI_CONTROLLER controller_id);

/*! \fn     DRIVER_API_RC soc_spi_clock_disable(SOC_SPI_CONTROLLER controller_id)
*
*  \brief   Function to disable the specified SPI controller.
*           This function assumes that there is no pending transaction on the SPI interface in question.
*           It is the responsibility of the calling application to do so.
*           Upon success, the specified SPI interface is clock gated in hardware,
*           it is no longer capable of generating interrupts.
*
*  \param   controller_id   : SPI controller identifier
*
*  \return  RC_OK on success
*           RC_FAIL otherwise
*/
DRIVER_API_RC soc_spi_clock_disable(SOC_SPI_CONTROLLER controller_id);

/*! \fn     DRIVER_API_RC soc_spi_transfer(SOC_SPI_CONTROLLER controller_id, uint8_t *tx_data, uint32_t tx_data_len, uint8_t *rx_data, uint32_t rx_data_len, SOC_SPI_SLAVE_ENABLE slave)
*
*  \brief   Function to send a command and receive a result from the specified SPI slave
*
*  \param   controller_id   : SPI controller identifier
*  \param   tx_data         : pointer to cmd to transmit
*  \param   tx_data_len     : length of cmd to transmit
*  \param   rx_data         : pointer to data to receive
*  \param   rx_data_len     : length of data to receive
*  \param   full_duplex     : set to 1 if data received between tx_phase should be put in rx_data buffer
*                             rx_data_len should be >= tx_data_len !
*  \param   slave           : slave device to TX to and receive from
*
*  \return  RC_OK                   -   on success
*           RC_CONTROLLER_IN_USE    -   when device is busy
*           RC_FAIL                 -   otherwise
*/
DRIVER_API_RC soc_spi_transfer(SOC_SPI_CONTROLLER controller_id, uint8_t *tx_data, uint32_t tx_data_len, uint8_t *rx_data, uint32_t rx_data_len, int full_duplex, SPI_SLAVE_ENABLE slave);

/*! \fn     DRIVER_SPI_STATUS_CODE soc_spi_status(SOC_SPI_CONTROLLER controller_id)
*
*  \brief   Function to determine controllers current state
*
*  \param   controller_id   : SPI controller identifier
*
*  \return  SOC_SPI_OK       - controller ready
*           SOC_SPI_BUSY     - controller busy
*           SOC_SPI_TFE      - TX FIFO Empty
*           SOC_SPI_RFNE     - RX FIFO Not Empty
*/
DRIVER_SPI_STATUS_CODE soc_spi_status(SOC_SPI_CONTROLLER controller_id);

#ifdef __cplusplus
}
#endif

/**@} @}*/

#endif /* INTEL_QRK_SPI_H_ */
