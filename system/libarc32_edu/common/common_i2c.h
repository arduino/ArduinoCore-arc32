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
 * Intel common I2C header
 */

#ifndef COMMON_I2C_H_
#define COMMON_I2C_H_

#include <stdint.h>

/**
 * @defgroup common_driver_i2c Common I2C
 * Inter Integrated Communication bus drivers API.
 * @ingroup common_drivers
 * @{
 */

/**
 * API types.
 */
typedef enum {
    I2C_OK = 0,             /*!< I2C BUS OK */
    I2C_BUSY,               /*!< I2C BUS BUSY */
    I2C_TX_ABORT,           /*!< I2C BUS TX ABORT */
    I2C_TX_OVER,            /*!< I2C BUS TX OVER */
    I2C_RX_OVER,            /*!< I2C BUS RX OVER */
    I2C_RX_UNDER,           /*!< I2C BUS RX UNDER */
}DRIVER_I2C_STATUS_CODE;

/**
 * I2C speeds.
 */
typedef enum {
    I2C_SLOW = 1,            /*!< (1) 0-100 Khz - note: Starts at 1 to remove need to translate before hardware write */
    I2C_FAST = 2,            /*!< (2) 400 Khz */
    I2C_HS = 3               /*!< (3) 3400 kbit/s mode added support for HS mode available only in SoC block */
} I2C_SPEED;

/**
 * I2C addressing modes.
 */
typedef enum {
    I2C_7_Bit = 0,           /*!< (0) 7 bit  */
    I2C_10_Bit,              /*!< (1) 10 bit */
} I2C_ADDR_MODE;

/**
 * I2C mode types.
 */
typedef enum {
    I2C_MASTER = 0,         /*!< MASTER WRITE MODE */
    I2C_SLAVE               /*!< MASTER READ MODE */
} I2C_MODE_TYPE;

/**
*  callback function signature.
*/
typedef void (*i2c_callback)( uint32_t );     /*!< callback function signature */


/**
*  I2C controller configuration.
*  Driver instantiates one of these with given parameters for each I2C
*  controller configured using the "ss_i2c_set_config" function
*/
typedef struct i2c_cfg_data{
    I2C_SPEED           speed;
    I2C_ADDR_MODE       addressing_mode;    /*!< 7 bit / 10 bit addressing */
    I2C_MODE_TYPE       mode_type;          /*!< Master or Slave */
    uint32_t            slave_adr;          /*!< I2C address if configured as a Slave */
    i2c_callback        cb_rx;              /*!< callback function for notification of data received and available to application, NULL if callback not required by application code */
    uint32_t            cb_rx_data;         /*!< this will be passed back by the callback routine - can be used as an controller identifier */
    i2c_callback        cb_tx               /*!< callback function for notification of transmit completion by the I2C controller,  NULL if callback not required by application code */;
    uint32_t            cb_tx_data;         /*!< this will be passed back by the callback routine - can be used as an controller identifier */
    i2c_callback        cb_err;             /*!< callback function on transaction error, NULL if callback not required by application code */
    uint32_t            cb_err_data;        /*!< this will be passed back by the callback routine - can be used as an controller identifier */
}i2c_cfg_data_t;

/** @} */

#endif /* COMMON_I2C_H_ */
