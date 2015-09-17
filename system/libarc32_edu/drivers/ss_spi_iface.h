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
 * Intel SPI driver
 *
 */

#ifndef SS_SPI_IFACE_H_
#define SS_SPI_IFACE_H_

#include <stdint.h>
#include "common_spi.h"
#include "data_type.h"
#include "stdbool.h"
/**
 * @defgroup spi_arc_driver Sensor Subsystem SPI
 * Serial Peripheral Interface ARC driver API.
 * @ingroup arc_driver
 * @{
 */

/**
 *  Clock speed into SPI peripheral
 */
#define FREQ_SPI_CLOCK_IN   (CLOCK_SPEED*1000*1000)        /* CLOCK_SPEED in MHz */

/**
 * List of all controllers in system ( IA and SS )
 */
typedef enum {
    SPI_SENSING_0,          /* Sensing SPI controller 0, accessibly by Sensor Subsystem Core only */
    SPI_SENSING_1           /* Sensing SPI controller 1, accessible by Sensor Subsystem Core only */
} SPI_CONTROLLER;

/**
*  Function to configure specified SPI controller.
*
*  Configuration parameters must be valid or an error is returned - see return values below.
*
*  @param   controller_id   : SPI  controller_id identifier
*  @param   config          : pointer to configuration structure
*
*  @return
*           - DRV_RC_OK                           - on success,
*           - DRV_RC_DEVICE_TYPE_NOT_SUPPORTED    - if device type is not supported by this controller
*           - DRV_RC_INVALID_CONFIG               - if any configuration parameters are not valid
*           - DRV_RC_CONTROLLER_IN_USE,           - if controller is in use
*           - DRV_RC_CONTROLLER_NOT_ACCESSIBLE    - if controller is not accessible from this core
*           - DRV_RC_FAIL                         - otherwise
*/
DRIVER_API_RC ss_spi_set_config(SPI_CONTROLLER controller_id, spi_cfg_data_t *config);

/**
*  Function to retrieve configuration of specified SPI controller.
*
*  @param   controller_id   : SPI controller_id identifier
*  @param   config          : pointer to configuration structure to store current setup
*
*  @return
*           - DRV_RC_OK   - on success
*           - DRV_RC_FAIL - otherwise
*/
DRIVER_API_RC ss_spi_get_config(SPI_CONTROLLER controller_id, spi_cfg_data_t *config);

/**
*  Function to place SPI controller into a disabled and default state (as if hardware reset occurred).
*
*  This function assumes that there is no pending transaction on the SPI interface in question.
*  It is the responsibility of the calling application to do so.
*  Upon success, the specified SPI interface is clock gated in hardware,
*  it is no longer capable to generating interrupts, it is also configured into a default state
*
*  @param   sba_dev   : pointer to bus configuration data
*
*  @return
*           - DRV_RC_OK on success
*           - DRV_RC_FAIL otherwise
*/
DRIVER_API_RC ss_spi_deconfig(SPI_CONTROLLER controller_id);

/**
*  Function to enable the specified SPI controller.
*
*  Upon success, the specified SPI interface+ is no longer clock gated in hardware, it is now
*  capable of transmitting and receiving on the SPI bus and of generating interrupts.
*
*  @param   sba_dev   : pointer to bus configuration data
*
*  @return
*           - DRV_RC_OK on success
*           - DRV_RC_FAIL otherwise
*/
DRIVER_API_RC ss_spi_clock_enable(SPI_CONTROLLER controller_id);

/**
*  Function to disable the specified SPI controller.
*
*  This function assumes that there is no pending transaction on the SPI interface in question.
*  It is the responsibility of the calling application to do so.
*  Upon success, the specified SPI interface is clock gated in hardware,
*  it is no longer capable of generating interrupts.
*
*  @param   sba_dev   : pointer to bus configuration data
*
*  @return
*           - DRV_RC_OK on success
*           - DRV_RC_FAIL otherwise
*/
DRIVER_API_RC ss_spi_clock_disable(SPI_CONTROLLER controller_id);

/**
*  Function to send a command and receive a result from the specified SPI slave.
*
*  @param   controller_id   : SPI controller_id identifier
*  @param   tx_data         : pointer to cmd to transmit
*  @param   tx_data_len     : length of cmd to transmit
*  @param   rx_data         : pointer to data to receive
*  @param   rx_data_len     : length of data to receive
*  @param   slave           : slave device to TX to and receive from
*
*  @return
*           - DRV_RC_OK                   -   on success
*           - DRV_RC_CONTROLLER_IN_USE    -   when device is busy
*           - DRV_RC_FAIL                 -   otherwise
*/
DRIVER_API_RC ss_spi_transfer(SPI_CONTROLLER controller_id, uint8_t *tx_data, uint32_t tx_data_len, uint8_t *rx_data, uint32_t rx_data_len, SPI_SLAVE_ENABLE slave);

/**
*  Function to determine controllers current state.
*
*  @param   controller_id   : SPI controller_id identifier
*
*  @return
*           - SOC_SPI_OK       - controller ready
*           - SOC_SPI_BUSY     - controller busy
*/
DRIVER_SPI_STATUS_CODE ss_spi_status(SPI_CONTROLLER controller_id);

/** @} */

#endif  /* SS_SPI_IFACE_H_ */
